"""A BEHAVIOR simulator or preview that dies or stalls reports its exit code and log, at every stage."""
import os
from pathlib import Path
import signal
import stat
import sys
from types import SimpleNamespace

import pytest

from services.behavior import process as P
from services.behavior.backend import BehaviorBackend
from services.behavior.preview import PreviewProcess
from services.simulator.io import SimulatorExited

REPO = Path(__file__).resolve().parents[2]

FAKE = r'''
import json, os, socket, sys, time
from services.simulator.protocol import receive_packet, send_packet, VERSION
mode = os.environ["FAKE_MODE"]
if mode == "stall":
    print("Error: GPU foundation failed to initialize", flush=True)
    time.sleep(2.5)
    sys.exit(0)
s = socket.socket(); s.bind(("127.0.0.1", 0)); s.listen(1)
print(json.dumps({"event": "ready", "port": s.getsockname()[1]}), flush=True)
c, _ = s.accept()
while True:
    r = receive_packet(c)
    if r["op"] == mode:
        print("Traceback (most recent call last):\nRuntimeError: PhysX scene failed to load layout",
              file=sys.stderr, flush=True)
        os._exit(3)
    result = ({"task": "turning_on_radio"} if r["op"] == "metadata" else
              {"episode_id": "episode-1", "step_id": 0, "native_step_limit": 100} if r["op"] == "reset" else
              {"ok": True})
    send_packet(c, {"version": VERSION, "request_id": r["request_id"], "ok": True, "result": result})
'''


@pytest.fixture
def python(tmp_path, monkeypatch):
    (tmp_path / "fake_sim.py").write_text(FAKE)
    path = tmp_path / "python"  # Stands in for the env interpreter; ignores "-m services.behavior.server".
    path.write_text(f"#!/bin/sh\nexec {sys.executable} {tmp_path / 'fake_sim.py'}\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("PYTHONPATH", str(REPO))
    monkeypatch.setenv("BEHAVIOR_RUNTIME_ROOT", str(tmp_path / "runtime"))
    monkeypatch.setattr(P, "physical_gpu_index", lambda gpu: "0")  # No nvidia-smi.
    return str(path)


def simulator(python, tmp_path, monkeypatch, mode):
    monkeypatch.setenv("FAKE_MODE", mode)
    return P.BehaviorProcess(output_root=tmp_path / "results", python=python)


def start(sim, timeout=30):
    return sim.start(task="turning_on_radio", collection=0, instance_id=0, run_id="run-1", sim_gpu="0",
                     startup_timeout=timeout)


@pytest.mark.parametrize("stage", ["metadata", "reset"])
def test_simulator_dying_during_startup_reports_exit_code_and_log(python, tmp_path, monkeypatch, stage):
    sim = simulator(python, tmp_path, monkeypatch, stage)
    with pytest.raises(SimulatorExited) as error:
        start(sim)
    message = str(error.value)
    assert message.startswith(f"{stage} failed with EOFError")
    assert "simulator process exited with code 3" in message
    assert "RuntimeError: PhysX scene failed to load layout" in message
    assert sim.process is None and sim.slot is None and not sim.active


def test_simulator_stalling_at_startup_reports_its_log(python, tmp_path, monkeypatch):
    sim = simulator(python, tmp_path, monkeypatch, "stall")
    with pytest.raises(TimeoutError) as error:
        start(sim, timeout=1)
    assert str(error.value) == ("BEHAVIOR simulator did not become ready within 1s; simulator process still "
                                "running; its log: Error: GPU foundation failed to initialize")
    assert sim.process is None and sim.slot is None


def test_simulator_dying_on_finish_reports_exit_code_and_log(python, tmp_path, monkeypatch):
    sim = simulator(python, tmp_path, monkeypatch, "finish_pilot")
    assert start(sim)["episode_id"] == "episode-1"
    with pytest.raises(SimulatorExited) as error:
        sim.finish("close")
    assert str(error.value).startswith("finish_pilot failed with EOFError")
    assert "exited with code 3" in str(error.value) and "PhysX scene failed to load layout" in str(error.value)
    assert sim.finished
    sim.stop()


def test_closing_after_the_simulator_died_between_calls_reports_it_and_still_stops(python, tmp_path, monkeypatch):
    sim = simulator(python, tmp_path, monkeypatch, "none")
    start(sim)
    os.killpg(sim.process.pid, signal.SIGKILL)
    sim.process.wait(timeout=10)
    assert sim.active  # Nothing told the host yet.
    stopped = []
    backend = SimpleNamespace(sim=sim, preview=SimpleNamespace(stop=lambda: stopped.append(True)))
    with pytest.raises(SimulatorExited, match=r"^finish_pilot failed with .*simulator process exited with code -9"):
        BehaviorBackend.close(backend)
    assert sim.process is None and sim.slot is None and stopped == [True]


def preview(python, tmp_path, monkeypatch, mode):
    monkeypatch.setenv("FAKE_MODE", mode)
    return PreviewProcess(output_root=tmp_path / "preview", sim_gpu="0", python=python)


def test_preview_dying_during_startup_reports_exit_code_and_log(python, tmp_path, monkeypatch):
    process = preview(python, tmp_path, monkeypatch, "metadata")
    with pytest.raises(RuntimeError) as error:
        process.ensure(startup_timeout=30)
    message = str(error.value)
    assert message.startswith("BEHAVIOR preview simulator failed during startup with EOFError")
    assert "simulator process exited with code 3" in message and "PhysX scene failed to load layout" in message
    assert process.process is None and process.slot is None


def test_preview_stalling_at_startup_reports_its_log(python, tmp_path, monkeypatch):
    process = preview(python, tmp_path, monkeypatch, "stall")
    with pytest.raises(TimeoutError) as error:
        process.ensure(startup_timeout=1)
    assert str(error.value) == ("BEHAVIOR preview simulator did not start within 1s; simulator process still "
                                "running; its log: Error: GPU foundation failed to initialize")
