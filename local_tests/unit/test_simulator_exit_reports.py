"""A RoboDojo simulator that dies or stalls reports its exit code and log, at every stage."""
import os
from pathlib import Path
import stat
import sys

import pytest

from services.robodojo.mcp_server import SimulatorProcess
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
print(json.dumps({"event": "ready", "port": s.getsockname()[1], "metadata": {}}), flush=True)
c, _ = s.accept()
while True:
    r = receive_packet(c)
    if r["op"] == mode:
        print("Traceback (most recent call last):\nRuntimeError: PhysX scene failed to load layout",
              file=sys.stderr, flush=True)
        os._exit(3)
    result = ({"task": "fake_task"} if r["op"] == "metadata" else
              {"episode_id": "episode-1", "step_id": 0} if r["op"] == "reset" else {"ok": True})
    send_packet(c, {"version": VERSION, "request_id": r["request_id"], "ok": True, "result": result})
'''


@pytest.fixture
def simulator(tmp_path, monkeypatch):
    root = tmp_path / "project"
    (root / "RoboDojo" / "task" / "RoboDojo" / "config").mkdir(parents=True)
    (root / "RoboDojo" / "task" / "RoboDojo" / "config" / "fake_task.yml").write_text("{}")
    (tmp_path / "fake_sim.py").write_text(FAKE)
    python = tmp_path / "python"  # Stands in for the env interpreter; ignores "-m services.robodojo.server".
    python.write_text(f"#!/bin/sh\nexec {sys.executable} {tmp_path / 'fake_sim.py'}\n")
    python.chmod(python.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("PYTHONPATH", str(REPO))
    monkeypatch.setenv("ROBODOJO_XDG_RUNTIME_DIR", str(tmp_path / "xdg"))

    def make(mode):
        monkeypatch.setenv("FAKE_MODE", mode)
        return SimulatorProcess(project_root=root, python=str(python), output_root=tmp_path / "results")
    return make


def start(sim, timeout=30):
    return sim.start(task="fake_task", seed=0, eval_seed=0, run_id="run-1", sim_gpu="0", sim_port=0,
                     startup_timeout=timeout)


@pytest.mark.parametrize("stage", ["metadata", "reset"])
def test_simulator_dying_during_startup_reports_exit_code_and_log(simulator, stage):
    sim = simulator(stage)
    with pytest.raises(SimulatorExited) as error:
        start(sim)
    message = str(error.value)
    assert message.startswith(f"{stage} failed with EOFError")
    assert "simulator process exited with code 3" in message
    assert "RuntimeError: PhysX scene failed to load layout" in message
    assert sim.process is None and not sim.active


def test_simulator_dying_on_finish_reports_exit_code_and_log(simulator):
    sim = simulator("finish_pilot")
    assert start(sim) == {"episode_id": "episode-1", "step_id": 0}
    with pytest.raises(SimulatorExited) as error:
        sim.finish("test_finish")
    assert "finish_pilot failed" in str(error.value)
    assert "exited with code 3" in str(error.value)
    assert "PhysX scene failed to load layout" in str(error.value)
    sim.stop()


def test_startup_timeout_reports_the_simulator_log(simulator):
    sim = simulator("stall")
    with pytest.raises(TimeoutError) as error:
        start(sim, timeout=1)
    assert "did not become ready within 1s" in str(error.value)
    assert "simulator process still running" in str(error.value)
    assert "Error: GPU foundation failed to initialize" in str(error.value)
    assert sim.process is None


@pytest.mark.parametrize("ended, expected", [(False, "controller_disconnect"), (True, "terminal")])
def test_closing_summary_keeps_the_disconnect_reason(tmp_path, monkeypatch, ended, expected):
    import json
    import socket
    import threading
    from types import SimpleNamespace
    from services.robodojo.server import close_summary
    from services.robodojo.session import RoboDojoSession
    from services.simulator.rpc import serve

    session = RoboDojoSession.__new__(RoboDojoSession)
    session.output, session.episode_id, session.step_id = tmp_path, "episode-1", 12
    session.terminated, session.truncated, session.success = False, ended, False
    session.invalid_reason = session.finish_reason = session.video_writer = None
    session.finished, session.video_frames = False, 12
    session.native_step_limit = session.episode_step_limit = 700
    session.episode_mode, session.metadata = "exploration", {"task": "t", "control_dt": 0.04}
    manager = SimpleNamespace(get_score=lambda: [40.0], _gated_score_lst=[40.0], check_list=[[1]],
                              final_check_list=[[]], trigger_check_list=[[]], score_completed_count=[1],
                              final_score_completed_count=[0])
    session.env = SimpleNamespace(reward_manager=manager, get_score=True, unstable_envs=set(),
                                  enforce_step_limit=True, end_flag=[False])
    port = {}
    printed = threading.Event()

    def capture(*values, **kwargs):
        port.update(json.loads(values[0]))
        printed.set()

    monkeypatch.setattr("builtins.print", capture)

    def controller():  # Connects, then vanishes without finish_pilot.
        printed.wait(10)
        socket.create_connection(("127.0.0.1", port["port"])).close()

    threading.Thread(target=controller, daemon=True).start()
    serve(session, 0)
    close_summary(session)
    assert json.loads((tmp_path / "evaluation_outcome.json").read_text())["reason"] == expected
    assert json.loads((tmp_path / "summary.json").read_text())["reason"] == expected
