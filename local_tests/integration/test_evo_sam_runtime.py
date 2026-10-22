"""RAX frontend -> Evo socket -> Triton protocol peer -> shared SAM engine.

Requires the dedicated mock profile and Evo's test/rpc_fixture.py. Never uses
physical actuators. The peer permits blocked-inference fault injection.
"""
import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import time

import pytest

from auto_research_agent.api.evo import Evo
from auto_research_agent.api.runtime import Context
from auto_research_agent.api.sam import Sam
from services import arming
from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.evo_v3.backend import EvoV3Backend

pytestmark = pytest.mark.skipif(
    not os.environ.get("EVO_SAM_TEST_STATE"),
    reason="SAM mock-runtime qualification is opt-in",
)


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for simulation evidence")


def test_sam_socket_and_stops_while_gpu_is_blocked(monkeypatch):
    parent = Path(os.environ["EVO_AGENT_TEST_SESSION_ROOT"])
    parent.mkdir(parents=True, exist_ok=True)
    monkeypatch.setenv("RAX_ARTIFACT_ROOT", str(parent.parent))
    directory = Path(tempfile.mkdtemp(prefix="sam-", dir=parent))
    config = Configuration(
        root=directory,
        task="sam_check",
        robot="evo-v3",
        backend_options={"control_socket": os.environ["EVO_AGENT_TEST_SOCKET"]},
    )
    backend = EvoV3Backend(config)
    frontend = Frontend(Session(config), backend)
    listener = socket.socket(socket.AF_UNIX)
    listener.bind(str(directory / "mcp.sock"))
    listener.listen()
    stop = threading.Event()
    thread = threading.Thread(
        target=frontend.serve_socket, args=(listener, stop), daemon=True
    )
    thread.start()
    state = Path(os.environ["EVO_SAM_TEST_STATE"])
    try:
        with Context(str(directory / "mcp.sock")) as ctx, Context(
            str(directory / "mcp.sock")
        ) as urgent:
            caps = ctx.value("evo_capabilities")
            assert caps["simulated"] is True and caps["sam"]["configured"]
            wait_for(lambda: ctx.value("evo_status")["recording"])
            capture = ctx.value("evo_observe")
            ident = capture["observation_id"]
            sam = Sam(ctx)
            # Interactive submission and Python polling share the same MCP socket.
            result = sam.wait(
                ctx.value("evo_sam_segment", observation_id=ident, camera="base")
            )
            assert result["frames"][0]["observation_id"] == ident
            frame = result["frames"][0]
            width, height = frame["transform"]["width"], frame["transform"]["height"]
            boxed = sam.wait(
                sam.segment(
                    ctx.value("evo_observe")["observation_id"],
                    box=[0, 0, width, height],
                )
            )
            assert boxed["profile"] == "image-854x480"
            assert frame["capture_us"] == capture["cameras"]["base"]["capture_us"]
            assert (directory / "agent-workspace" / frame["overlay"]["path"]).is_file()
            # Captures need strict source ordering; use newly acquired frames.
            first = ctx.value("evo_observe")["observation_id"]
            time.sleep(0.55)
            second = ctx.value("evo_observe")["observation_id"]
            clip = sam.wait(
                sam.track([first, second], points=[{"pixel": [10, 10], "label": 1}])
            )
            assert [f["observation_id"] for f in clip["frames"]] == [first, second]
            with pytest.raises(RuntimeError, match="ordered"):
                sam.track([second, first])
            with pytest.raises(RuntimeError, match="calibration"):
                sam.segment(second, space="rectified")
            arming.write(directory, True, by="SAM simulation test")
            for tool in ("evo_cancel", "evo_hold"):
                for name in ("entered", "completed"):
                    (state / name).unlink(missing_ok=True)
                (state / "block").touch()
                job = sam.segment(ctx.value("evo_observe")["observation_id"])
                wait_for(lambda: (state / "entered").exists())
                action = ctx.value("evo_drive", velocity=[0.02, 0, 0], duration_s=5)
                time.sleep(0.1)
                args = (
                    {"action_id": action["action_id"]} if tool == "evo_cancel" else {}
                )
                urgent.value(tool, **args)
                stopped = Evo(urgent).wait(action, timeout_s=3)
                assert (
                    stopped["physical_stop_verified"]
                    and stopped["state"] == "cancelled"
                )
                assert not (state / "completed").exists()
                assert Sam(urgent).cancel(job)["state"] == "cancelled"
                (state / "block").unlink()
                wait_for(lambda: (state / "completed").exists(), 40)
                time.sleep(0.2)
                assert (
                    ctx.value("evo_sam_result", job_id=job["job_id"])["state"]
                    == "cancelled"
                )
            (directory / "sam-report.json").write_text(
                json.dumps(
                    {
                        "image": result["profile"],
                        "clip": clip["profile"],
                        "stop_paths": ["cancel", "hold"],
                        "physical_stop_verified": True,
                    },
                    indent=2,
                )
            )
    finally:
        (state / "block").unlink(missing_ok=True)
        arming.write(directory, False, by="simulation cleanup")
        stop.set()
        thread.join(3)
        listener.close()
        frontend.close()
