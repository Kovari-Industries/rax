"""The robot preview process restarts after a failure and reports how it ended."""
from pathlib import Path
import stat
import sys

import pytest

from services.robot_preview.mcp_server import PreviewProcess

REPO = Path(__file__).resolve().parents[2]

FAKE = r'''
import json, os, socket, sys, time
from pathlib import Path
from services.simulator.protocol import receive_packet, send_packet, VERSION
mode = os.environ["FAKE_MODE"]
counter = Path(os.environ["FAKE_COUNTER"])
instance = int(counter.read_text()) + 1 if counter.exists() else 1
counter.write_text(str(instance))
if mode == "stall":
    print("Error: preview renderer failed to initialize", flush=True)
    time.sleep(60)
print(0, flush=True)  # A bare JSON value on stdout must not stop the log reader.
s = socket.socket(); s.bind(("127.0.0.1", 0)); s.listen(1)
print(json.dumps({"event": "ready", "port": s.getsockname()[1]}), flush=True)
c, _ = s.accept()
while True:
    try:
        r = receive_packet(c)
    except Exception:
        sys.exit(0)
    if r["op"] == "preview" and mode == "die_first" and instance == 1:
        print("Traceback (most recent call last):\nRuntimeError: preview articulation exploded",
              file=sys.stderr, flush=True)
        os._exit(3)
    result = {"robot": "dual_arx_x5"} if r["op"] == "metadata" else {"instance": instance}
    send_packet(c, {"version": VERSION, "request_id": r["request_id"], "ok": True, "result": result})
'''


@pytest.fixture
def preview(tmp_path, monkeypatch):
    (tmp_path / "fake_preview.py").write_text(FAKE)
    python = tmp_path / "python"  # Stands in for the env interpreter; ignores "-m services.robot_preview.server".
    python.write_text(f"#!/bin/sh\nexec {sys.executable} {tmp_path / 'fake_preview.py'}\n")
    python.chmod(python.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("PYTHONPATH", str(REPO))
    monkeypatch.setenv("FAKE_COUNTER", str(tmp_path / "instances"))
    made = []

    def make(mode, startup_timeout=30):
        monkeypatch.setenv("FAKE_MODE", mode)
        process = PreviewProcess(tmp_path, str(python), tmp_path / "results", 0, startup_timeout)
        made.append(process)
        return process
    yield make
    for process in made:
        process.close(grace=0)


def test_preview_restarts_after_it_dies_and_reports_the_exit(preview):
    process = preview("die_first")
    with pytest.raises(RuntimeError) as error:
        process.request(robot_model="dual_arx_x5")
    message = str(error.value)
    assert message.startswith("Robot preview preview failed with EOFError")
    assert "simulator process exited with code 3" in message
    assert "RuntimeError: preview articulation exploded" in message
    assert process.process is None and process.rpc is None
    assert process.request(robot_model="dual_arx_x5") == {"instance": 2}


def test_preview_startup_timeout_stops_the_process_and_reports_its_log(preview):
    process = preview("stall", startup_timeout=1)
    with pytest.raises(TimeoutError) as error:
        process.start()
    assert "did not become ready within 1s" in str(error.value)
    assert "Error: preview renderer failed to initialize" in str(error.value)
    assert process.process is None


def test_preview_startup_survives_non_object_json_lines(preview):
    process = preview("healthy")
    assert process.request(robot_model="dual_arx_x5") == {"instance": 1}
    assert process.metadata == {"robot": "dual_arx_x5"}
