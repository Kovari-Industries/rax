"""Complete-path verification against the dedicated Evo agent simulation profile.

Run only with EVO_AGENT_TEST_SOCKET and EVO_AGENT_TEST_SESSION_ROOT set. The
runtime must report simulated actuators before this test writes an arming file.
The native model, Unix server, bridge, ROS controllers, and measured feedback
are supplied by the running Evo profile, not replaced by Python stubs.
"""
import json
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import socket
import tempfile
import threading
import time

import pytest

from auto_research_agent.api.runtime import Context
from auto_research_agent.api.evo import Evo
from services import arming
from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.evo_v3.backend import EvoV3Backend

pytestmark = pytest.mark.skipif(
    not os.environ.get("EVO_AGENT_TEST_SOCKET"),
    reason="dedicated Evo simulation is not running",
)


def _cloth_program_fixture(program):
    """Run real motion from deterministic cloth evidence, without cloth physics."""
    from auto_research_agent.api.cloth import Cloth, ClothSettings, Grasp, Pile, Scene

    assert program.value("evo_capabilities")["simulated"] is True
    palms = program.value("evo_fk", links=["left_palm", "right_palm"])["poses"]
    positions = program.value("evo_status")["state"]["positions"]
    x = sum(p["position"][0] for p in palms.values()) / 2
    z = sum(p["position"][2] for p in palms.values()) / 2
    yl, yr = palms["left_palm"]["position"][1], palms["right_palm"]["position"][1]
    orientations = {s: palms[s + "_palm"]["quaternion_xyzw"] for s in ("left", "right")}
    scene_state = "crumpled"
    folded = False

    def perceive(observation, check):
        nonlocal scene_state, folded
        if check.stage == "fold_select":
            folded = True
        if check.stage == "laid_down":
            scene_state = "folded" if folded else "spread"
        if check.stage == "stack_complete":
            scene_state = "stacked"
        near = x - 0.025 if scene_state in ("folded", "stacked") else x
        corners = ((near, yl, z), (near, yr, z), (x - 0.05, yr, z), (x - 0.05, yl, z))
        feature = "packet" if scene_state in ("folded", "stacked") else "corner"
        grasps = tuple(
            Grasp(str(i), p, orientations["left" if p[1] > 0 else "right"], feature)
            for i, p in enumerate(corners)
        )
        measured = program.value("evo_fk", links=["left_palm", "right_palm"])["poses"]
        return Scene(
            observation["observation_id"],
            scene_state,
            "simulation",
            grasps=grasps,
            corners=corners,
            held={"left": "verified", "right": "verified"},
            layers={"left": "packet", "right": "packet"},
            stable=True,
            supported=True,
            surface_z=z - 0.02,
            packet_bottom_z=min(p["position"][2] for p in measured.values()) - 0.02,
            pile=Pile(
                (x - 0.04, (yl + yr) / 2, z - 0.015),
                (yl - yr + 0.02, 0.05),
                yaw_rad=-1.5707963267948966,
            ),
            alignment_error_m=0.001,
            note="Deterministic software fixture; no physical cloth evidence",
        )

    settings = ClothSettings(
        open_rad={s: positions[s + "_finger_joint1"] for s in ("left", "right")},
        close_rad={
            s: positions[s + "_finger_joint1"] + 0.005 for s in ("left", "right")
        },
        approach_clearance_m=0.005,
        test_lift_m=0.003,
        inspection_lift_m=0.005,
        fold_clearance_m=0.01,
        transport_clearance_m=0.01,
        release_gap_m=0.001,
        alignment_tolerance_m=0.005,
        segment_s=3.0,
    )
    landing = {
        s: {
            "position": [x, yl if s == "left" else yr, z],
            "quaternion_xyzw": orientations[s],
        }
        for s in ("left", "right")
    }
    cloth = Cloth(program, perceive, settings)
    # One half-fold keeps this fixture inside the mock robot's initial workspace.
    result = cloth.crumpled_to_pile(landing, yl - yr, axes=("v",))
    assert result.state == "stacked"
    assert all(
        e["result"]["physical_stop_verified"] for e in cloth.evidence if "result" in e
    )
    return cloth.evidence


def test_all_actuators_and_stop_paths(monkeypatch):
    parent = Path(os.environ["EVO_AGENT_TEST_SESSION_ROOT"]).resolve()
    parent.mkdir(parents=True, exist_ok=True)
    monkeypatch.setenv("RAX_ARTIFACT_ROOT", str(parent.parent))
    directory = Path(tempfile.mkdtemp(prefix="backend-check-", dir=parent))
    config = Configuration(
        root=directory,
        task="backend_check",
        robot="evo-v3",
        backend_options={
            "control_socket": os.environ["EVO_AGENT_TEST_SOCKET"],
            "socket_timeout_s": 30.0,
        },
    )
    backend = EvoV3Backend(config)
    frontend = Frontend(Session(config), backend)
    results = []

    def call(name, **params):
        response = frontend.call(name, params)
        value = json.loads(response["content"][0]["text"])
        if response.get("isError"):
            raise RuntimeError(value)
        return value

    listener = None
    stop = threading.Event()
    thread = None
    try:
        capabilities = call("evo_capabilities")
        assert (
            capabilities["simulated"] is True
        ), "refusing to arm a non-simulation runtime"
        assert capabilities["control"] == "exclusive_agent"
        assert capabilities["tools"] == backend.tools()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            status = call("evo_status")
            if status["feedback_fresh"] and status["recording"]:
                break
            time.sleep(0.1)
        assert status["feedback_fresh"] and status["recording"]
        arming.write(directory, True, by="simulation test")

        class ToolContext:
            def value(self, name, **params):
                return call(name, **params)

        wait = Evo(ToolContext())

        def execute(name, **params):
            preview = call(
                "evo_preview", action={"kind": name.removeprefix("evo_"), **params}
            )
            assert preview["valid"] and not preview["scene_collision_checked"]
            action = call(name, **params)
            assert action["state"] == "accepted"
            result = wait.wait(action, timeout_s=35)
            assert (
                result["state"] == "succeeded" and result["physical_stop_verified"]
            ), result
            results.append(
                {
                    "tool": name,
                    "action_id": action["action_id"],
                    "state": result["state"],
                    "final_state": result["final_state"],
                }
            )
            return result

        q = status["state"]["positions"]
        targets = {
            f"{side}_joint{i}": q[f"{side}_joint{i}"]
            + (0.04 if side == "left" else -0.04)
            for side in ["left", "right"]
            for i in range(1, 8)
        }
        moved = execute("evo_move_joints", targets=targets)["final_state"]["positions"]
        assert all(abs(moved[name] - q[name]) > 0.02 for name in targets)
        for side in ["left", "right"]:
            execute(
                "evo_gripper", side=side, position_rad=q[side + "_finger_joint1"] - 0.02
            )
        execute("evo_move_carriage", position_m=q["carriage_joint"] + 0.01)
        execute(
            "evo_move_neck",
            yaw_rad=q["neck_yaw_joint"] + 0.04,
            pitch_rad=q["neck_pitch_joint"] + 0.04,
        )
        before = call("evo_status")["state"]["odometry"]["position"]
        driven = execute("evo_drive", velocity=[0.03, 0, 0], duration_s=0.2)[
            "final_state"
        ]["odometry"]["position"]
        assert sum((a - b) ** 2 for a, b in zip(driven, before)) > 1e-6
        # A drive shorter than a supervisor scheduling gap must still send motion.
        before = call("evo_status")["state"]["odometry"]["position"]
        short = execute("evo_drive", velocity=[0.03, 0, 0], duration_s=0.02)
        after = short["final_state"]["odometry"]["position"]
        assert sum((a - b) ** 2 for a, b in zip(after, before)) > 1e-10
        poses = call("evo_fk", links=["left_palm", "right_palm"])["poses"]
        for pose in poses.values():
            pose["position"][0] += 0.02
        execute(
            "evo_move_eef",
            frame="base_link",
            left=poses["left_palm"],
            right=poses["right_palm"],
        )
        # Coordinated path, body and persistent grip primitives use the same
        # native model, bridge and measured controllers as single targets.
        execute(
            "evo_set_grip",
            side="left",
            position_rad=q["left_finger_joint1"] - 0.03,
            retain=True,
        )
        current = call("evo_status")["state"]["positions"]
        execute(
            "evo_move_body",
            targets={
                "carriage_joint": current["carriage_joint"] + 0.005,
                "neck_yaw_joint": current["neck_yaw_joint"] + 0.01,
            },
            duration_s=2.0,
        )
        assert call("evo_status")["retained_grips"][
            "left_finger_joint1"
        ] == pytest.approx(q["left_finger_joint1"] - 0.03)
        palms = call("evo_fk", links=["left_palm", "right_palm"])["poses"]
        for pose in palms.values():
            pose["position"][2] += 0.003
        execute(
            "evo_follow_path",
            frame="base_link",
            waypoints=[
                {
                    "duration_s": 3.0,
                    "left": palms["left_palm"],
                    "right": palms["right_palm"],
                }
            ],
        )
        reached = call("evo_fk", links=["left_palm", "right_palm"])["poses"]
        for side in ("left", "right"):
            assert (
                sum(
                    (a - b) ** 2
                    for a, b in zip(
                        reached[side + "_palm"]["position"],
                        palms[side + "_palm"]["position"],
                    )
                )
                < 0.002**2
            )
        left, right = reached["left_palm"], reached["right_palm"]
        execute(
            "evo_move_pair",
            frame="base_link",
            waypoints=[
                {
                    "duration_s": 3.0,
                    "midpoint": [
                        (a + b) / 2 for a, b in zip(left["position"], right["position"])
                    ],
                    "separation": [
                        a - b for a, b in zip(left["position"], right["position"])
                    ],
                    "left_quaternion_xyzw": left["quaternion_xyzw"],
                    "right_quaternion_xyzw": right["quaternion_xyzw"],
                    "carriage_position_m": current["carriage_joint"] + 0.007,
                }
            ],
        )
        assert call("evo_status")["retained_grips"]
        hold = call("evo_hold")
        assert wait.wait(hold)["physical_stop_verified"]
        assert call("evo_status")["retained_grips"] == {}
        observation = call("evo_observe")
        assert set(observation["cameras"]) == {"base", "left_arm", "right_arm"}
        assert all(v["capture_time_state"] for v in observation["cameras"].values())
        history = call("evo_history")
        assert any(
            v["observation_id"] == observation["observation_id"]
            for v in history["observations"]
        )
        old = call("evo_history", observation_id=observation["observation_id"])
        assert old["observation_id"] == observation["observation_id"]
        assert (
            old["cameras"]["base"]["capture_us"]
            == observation["cameras"]["base"]["capture_us"]
        )
        assert all(not v["geometry_validated"] for v in observation["cameras"].values())
        rejected = frontend.call(
            "evo_camera_ray",
            {
                "observation_id": observation["observation_id"],
                "camera": "base",
                "pixel": [480, 270],
            },
        )
        assert rejected["isError"]
        rejected = frontend.call("evo_move_joints", {"targets": {"left_joint1": 100}})
        assert rejected["isError"]
        rejected = frontend.call(
            "evo_move_eef",
            {
                "frame": "base_link",
                "left": {"position": [100, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
            },
        )
        assert rejected["isError"]
        action = call("evo_drive", velocity=[0.03, 0, 0], duration_s=2)
        call("evo_cancel", action_id=action["action_id"])
        cancelled = wait.wait(action)
        assert cancelled["state"] == "cancelled" and cancelled["physical_stop_verified"]
        action = call("evo_drive", velocity=[0.03, 0, 0], duration_s=2)
        arming.write(directory, False, by="simulation test")
        disarmed = wait.wait(action)
        assert disarmed["state"] == "failed" and disarmed["physical_stop_verified"]
        arming.write(directory, True, by="simulation test")
        # Python Context uses the same frontend socket as an interactive MCP client.
        listener = socket.socket(socket.AF_UNIX)
        listener.bind(str(directory / "mcp.sock"))
        listener.listen()
        thread = threading.Thread(
            target=frontend.serve_socket,
            args=(listener,),
            kwargs={"stop": stop},
            daemon=True,
        )
        thread.start()
        with Context(str(directory / "mcp.sock")) as program:
            assert program.value("evo_capabilities")["tools"] == capabilities["tools"]
            current = program.value("evo_status")["state"]["positions"]
            result = Evo(program).move(
                "evo_move_neck",
                yaw_rad=current["neck_yaw_joint"],
                pitch_rad=current["neck_pitch_joint"],
            )
            assert result["state"] == "succeeded" and result["physical_stop_verified"]
        socket_stops = []
        # Keep an ordinary call pending in the real RAX frontend while cancellation
        # crosses its public socket, Evo, bridge, and mock controllers.
        for stop_tool, moving_tool in (
            ("evo_cancel", "evo_drive"),
            ("evo_hold", "evo_drive"),
            ("evo_cancel", "evo_follow_path"),
            ("evo_hold", "evo_follow_path"),
        ):
            blocked = threading.Event()
            release = threading.Event()
            original_rpc = backend._rpc

            def slow_observation(method, params, **kwargs):
                if method == "observe":
                    blocked.set()
                    assert release.wait(5), "observation was not released"
                return original_rpc(method, params, **kwargs)

            with Context(str(directory / "mcp.sock")) as program, Context(
                str(directory / "mcp.sock")
            ) as observer:
                if moving_tool == "evo_drive":
                    action = program.value(
                        moving_tool, velocity=[0.03, 0, 0], duration_s=5
                    )
                else:
                    palms = program.value("evo_fk", links=["left_palm", "right_palm"])[
                        "poses"
                    ]
                    for p in palms.values():
                        p["position"][2] += 0.005
                    action = program.value(
                        moving_tool,
                        frame="base_link",
                        waypoints=[
                            {
                                "duration_s": 5.0,
                                "left": palms["left_palm"],
                                "right": palms["right_palm"],
                            }
                        ],
                    )
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    status = program.value("evo_status")
                    moving = (
                        abs(status["state"]["odometry"]["velocity"][0]) > 0.01
                        if moving_tool == "evo_drive"
                        else any(
                            abs(v) > 1e-5
                            for v in status["state"]["velocities"].values()
                        )
                    )
                    if moving:
                        break
                    time.sleep(0.01)
                assert moving, status
                with monkeypatch.context() as patch, ThreadPoolExecutor(
                    max_workers=1
                ) as pool:
                    patch.setattr(backend, "_rpc", slow_observation)
                    pending = pool.submit(observer.value, "evo_observe")
                    try:
                        assert blocked.wait(1)
                        program._socket.settimeout(2)
                        with pytest.raises(RuntimeError, match="Frontend workers busy"):
                            program.value(
                                "evo_drive", velocity=[0.03, 0, 0], duration_s=0.2
                            )
                        args = (
                            {"action_id": action["action_id"]}
                            if stop_tool == "evo_cancel"
                            else {}
                        )
                        assert program.value(stop_tool, **args)["state"] == "stopping"
                        result = Evo(program).wait(action, timeout_s=3)
                        assert (
                            result["state"] == "cancelled"
                            and result["physical_stop_verified"]
                        ), result
                        assert (
                            not pending.done()
                        ), "stop must complete before ordinary work is released"
                        assert (
                            not backend._stop.is_set()
                        ), "cancellation must not depend on lease expiry"
                        socket_stops.append(
                            {"tool": stop_tool, "motion": moving_tool, "result": result}
                        )
                    finally:
                        release.set()
                    assert pending.result(timeout=2)["cameras"]
                assert program.value("evo_status")["active_action"] is None
        with Context(str(directory / "mcp.sock")) as program:
            cloth_evidence = _cloth_program_fixture(program)
        # A lost client stops renewing its lease; no reconnect may replay its action.
        action = call("evo_drive", velocity=[0.03, 0, 0], duration_s=2)
        backend._stop.set()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            status = call("evo_status")
            lost = status.get("last_action")
            if lost and lost["action_id"] == action["action_id"]:
                break
            time.sleep(0.05)
        assert (
            lost["action_id"] == action["action_id"]
            and lost["state"] == "failed"
            and lost["physical_stop_verified"]
        )
        assert status["owner"] is None
        assert frontend.call(
            "evo_drive", {"velocity": [0.03, 0, 0], "duration_s": 0.2}
        )["isError"]
        (directory / "verification.json").write_text(
            json.dumps(
                {
                    "actuators": results,
                    "cloth_fixture": cloth_evidence,
                    "cancel": cancelled,
                    "frontend_socket_stops": socket_stops,
                    "disarm": disarmed,
                    "lease_loss": lost,
                },
                indent=2,
            )
        )
    finally:
        arming.write(directory, False, by="simulation test cleanup")
        stop.set()
        if thread:
            thread.join(2)
        if listener:
            listener.close()
        frontend.close()


def test_recorder_failure_stops_action(monkeypatch):
    """The caller selects the recorder from this isolated runtime's process list."""
    import signal

    raw_pid = os.environ.get("EVO_AGENT_TEST_RECORDER_PID")
    if not raw_pid:
        pytest.skip("recorder failure injection was not requested")
    parent = Path(os.environ["EVO_AGENT_TEST_SESSION_ROOT"]).resolve()
    monkeypatch.setenv("RAX_ARTIFACT_ROOT", str(parent.parent))
    directory = Path(tempfile.mkdtemp(prefix="recorder-check-", dir=parent))
    config = Configuration(
        root=directory,
        task="recorder_check",
        robot="evo-v3",
        backend_options={"control_socket": os.environ["EVO_AGENT_TEST_SOCKET"]},
    )
    backend = EvoV3Backend(config)
    frontend = Frontend(Session(config), backend)

    class ToolContext:
        def value(self, name, **params):
            response = frontend.call(name, params)
            value = json.loads(response["content"][0]["text"])
            assert not response.get("isError"), value
            return value

    context = ToolContext()
    try:
        assert context.value("evo_capabilities")["simulated"] is True
        pid = int(raw_pid)
        command = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        runtime = Path(os.environ["EVO_AGENT_TEST_SOCKET"]).parent.parent
        assert b"bag" in command and b"record" in command
        assert any(
            arg.startswith(str(runtime / "bags").encode() + b"/") for arg in command
        )
        arming.write(directory, True, by="simulation recorder failure test")
        action = context.value("evo_drive", velocity=[0.03, 0, 0], duration_s=5)
        time.sleep(0.1)
        os.kill(pid, signal.SIGINT)
        result = Evo(context).wait(action)
        assert result["state"] == "failed" and result["physical_stop_verified"], result
        assert result["reason"] == "recording unavailable", result
        assert not context.value("evo_status")["recording"]
        (directory / "verification.json").write_text(json.dumps(result, indent=2))
    finally:
        arming.write(directory, False, by="simulation test cleanup")
        frontend.close()


def test_restart_rejects_previous_requests():
    """Repeat an accepted wire request after an operator restarts the mock runtime."""
    previous = os.environ.get("EVO_AGENT_TEST_PREVIOUS_SESSION")
    if not previous:
        pytest.skip("previous session for restart verification was not supplied")
    events = [
        json.loads(line)
        for line in (Path(previous) / "operator/evo-actions.jsonl")
        .read_text()
        .splitlines()
    ]
    owner = next(e["session_id"] for e in events if e["event"] == "session_open")
    accepted = next(e for e in events if e["event"] == "action_accepted")

    def request(method, params, request_id="restart-check", session_id=None):
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(5)
            connection.connect(os.environ["EVO_AGENT_TEST_SOCKET"])
            connection.sendall(
                (
                    json.dumps(
                        {
                            "version": 1,
                            "id": request_id,
                            "session_id": session_id,
                            "method": method,
                            "params": params,
                        }
                    )
                    + "\n"
                ).encode()
            )
            return json.loads(connection.makefile("rb").readline(1024 * 1024))

    assert request("capabilities", {})["result"]["simulated"] is True
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        before = request("status", {})["result"]
        if before["feedback_fresh"] and before["recording"]:
            break
        time.sleep(0.1)
    assert before["feedback_fresh"] and before["recording"]
    assert before["owner"] is None and before["active_action"] is None
    assert before["last_action"] is None
    action = dict(accepted["action"])
    kind = action.pop("kind")
    replay = request(kind, action, accepted["request_id"], owner)
    assert not replay["ok"] and "no active session" in replay["error"]["message"]
    heartbeat = request("heartbeat", {"sequence": 10000}, session_id=owner)
    assert not heartbeat["ok"]
    time.sleep(0.3)
    after = request("status", {})["result"]
    assert after["active_action"] is None and after["last_action"] is None
    assert after["state"]["positions"] == before["state"]["positions"]
    (Path(previous) / "restart-verification.json").write_text(
        json.dumps(
            {
                "replay": replay,
                "heartbeat": heartbeat,
                "before": before,
                "after": after,
            },
            indent=2,
        )
    )


def test_shutdown_stops_active_action(monkeypatch):
    """Stop only the explicitly selected mock orchestrator while it is driving."""
    import signal

    if os.environ.get("EVO_AGENT_TEST_ALLOW_SHUTDOWN") != "1":
        pytest.skip("operator shutdown injection was not requested")
    runtime = Path(os.environ["EVO_AGENT_TEST_SOCKET"]).parent.parent
    pid = int((runtime / "evo.pid").read_text())
    command = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    runtime_environment = [
        entry
        for entry in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
        if entry.startswith(b"EVO_RUNTIME_DIR=")
    ]
    assert b"up" in command and b"active" in command
    assert runtime_environment == [("EVO_RUNTIME_DIR=" + str(runtime)).encode()]
    parent = Path(os.environ["EVO_AGENT_TEST_SESSION_ROOT"]).resolve()
    monkeypatch.setenv("RAX_ARTIFACT_ROOT", str(parent.parent))
    directory = Path(tempfile.mkdtemp(prefix="shutdown-check-", dir=parent))
    config = Configuration(
        root=directory,
        task="shutdown_check",
        robot="evo-v3",
        backend_options={"control_socket": os.environ["EVO_AGENT_TEST_SOCKET"]},
    )
    frontend = Frontend(Session(config), EvoV3Backend(config))
    try:
        response = frontend.call("evo_capabilities", {})
        assert json.loads(response["content"][0]["text"])["simulated"] is True
        arming.write(directory, True, by="simulation shutdown test")
        response = frontend.call(
            "evo_drive", {"velocity": [0.03, 0, 0], "duration_s": 5}
        )
        assert not response.get("isError"), response
        action = json.loads(response["content"][0]["text"])
        time.sleep(0.1)
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 12
        result = None
        while time.monotonic() < deadline:
            events = [
                json.loads(line)
                for line in (directory / "operator/evo-actions.jsonl")
                .read_text()
                .splitlines()
            ]
            result = next(
                (
                    e["result"]
                    for e in events
                    if e["event"] == "action_result"
                    and e["result"]["action_id"] == action["action_id"]
                ),
                None,
            )
            if result:
                break
            time.sleep(0.05)
        assert result and result["state"] == "failed", result
        assert (
            result["physical_stop_verified"] and result["reason"] == "backend stopped"
        ), result
        while Path(f"/proc/{pid}").exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not Path(f"/proc/{pid}").exists(), "mock orchestrator did not exit"
        (directory / "verification.json").write_text(json.dumps(result, indent=2))
    finally:
        arming.write(directory, False, by="simulation test cleanup")
        frontend.close()
