"""The RAX adapter talks only to the versioned Evo service."""
import base64
import contextlib
import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace

import pytest
from services.evo_v3.backend import EvoV3Backend, EvoRefused, TOOLS


@contextlib.contextmanager
def service(tmp_path, handler):
    path = tmp_path / "evo.sock"
    server = socket.socket(socket.AF_UNIX)
    server.bind(str(path))
    server.listen()
    server.settimeout(0.05)
    stop = threading.Event()
    received = []

    workers = []

    def handle(connection):
        with connection, connection.makefile("rb") as reader:
            request = json.loads(reader.readline())
            received.append(request)
            method = request["method"]
            if method == "open_session":
                result = {
                    "session_id": request["params"]["session_id"],
                    "lease_s": 0.5,
                }
            elif method == "heartbeat":
                result = {"lease_s": 0.5}
            elif method == "close_session":
                result = {"state": "stopping", "physical_stop_verified": False}
            else:
                result = handler(request)
            if result is None:
                return  # Drop the response after the request reached the server.
            if isinstance(result, bytes):
                connection.sendall(result)
            else:
                response = {
                    "version": 1,
                    "id": request["id"],
                    "ok": True,
                    "result": result,
                }
                if isinstance(result, EvoRefused):
                    response.update(
                        ok=False, error={"code": "refused", "message": str(result)}
                    )
                    response.pop("result")
                connection.sendall(json.dumps(response).encode() + b"\n")

    def serve():
        while not stop.is_set():
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            worker = threading.Thread(target=handle, args=(connection,), daemon=True)
            workers.append(worker)
            worker.start()

    thread = threading.Thread(target=serve)
    thread.start()
    client = EvoV3Backend(
        SimpleNamespace(root=tmp_path, backend_options={"control_socket": str(path)})
    )
    try:
        yield client, received
    finally:
        try:
            client.close()
        finally:
            stop.set()
            thread.join(1)
            server.close()
            for worker in workers:
                worker.join(1)


def test_constructor_and_idle_close_do_not_connect(tmp_path):
    client = EvoV3Backend(SimpleNamespace(root=tmp_path, backend_options={}))
    client.close()
    client.close()
    assert client.session_id is None
    with pytest.raises(ValueError, match="unknown Evo options"):
        EvoV3Backend(
            SimpleNamespace(
                root=tmp_path, backend_options={"camera_directory": "/obsolete"}
            )
        )


@pytest.mark.parametrize(
    "name,args",
    [
        ("evo_status", {"speed": 1}),
        ("evo_towel_align", {}),
        ("evo_drive", {"velocity": [1, 2], "duration_s": 1}),
        ("evo_drive", {"velocity": [0, 0, 0], "duration_s": 0}),
        ("evo_drive", {"velocity": [0, 0, 0], "duration_s": 11}),
        ("evo_move_joints", {"targets": {"left_joint1": float("nan")}}),
        ("evo_move_joints", {"targets": {"left_joint1": True}}),
        ("evo_gripper", {"side": "left", "position_m": 0.1}),
        (
            "evo_move_eef",
            {
                "frame": "world",
                "left": {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
            },
        ),
    ],
)
def test_invalid_request_never_reaches_evo(tmp_path, name, args):
    with service(tmp_path, lambda _: {}) as (client, received):
        with pytest.raises(ValueError):
            client.call(name, args)
        assert not received


def test_motion_has_identity_and_independent_lease(tmp_path):
    with service(
        tmp_path, lambda r: {"action_id": "action-a", "state": "accepted"}
    ) as (client, received):
        result, _ = client.call("evo_move_neck", {"yaw_rad": 0.1, "pitch_rad": 0})
        assert result == {"action_id": "action-a", "state": "accepted"}
        deadline = time.monotonic() + 1
        while (
            sum(r["method"] == "heartbeat" for r in received) < 2
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        actions = [r for r in received if r["method"] == "move_neck"]
        assert len(actions) == 1 and actions[0]["session_id"] == client.session_id
        assert sum(r["method"] == "heartbeat" for r in received) >= 2
        assert len({r["id"] for r in received}) == len(received)


@pytest.mark.parametrize(
    "response",
    [None, {}, b'{"version":9,"id":"wrong","ok":true,"result":{}}\n', b"not-json\n"],
)
def test_uncertain_motion_is_never_retried_and_lease_is_withdrawn(tmp_path, response):
    with service(tmp_path, lambda _: response) as (client, received):
        with pytest.raises((ConnectionError, ValueError)):
            client.call("evo_move_carriage", {"position_m": 0.2})
        assert client._stop.is_set()
        with pytest.raises(ConnectionError, match="start a new session"):
            client.call("evo_move_carriage", {"position_m": 0.2})
        assert sum(r["method"] == "move_carriage" for r in received) == 1


def test_confirmed_refusal_preserves_session(tmp_path):
    with service(tmp_path, lambda _: EvoRefused("joint limit: left_joint1")) as (
        client,
        received,
    ):
        with pytest.raises(EvoRefused, match="joint limit"):
            client.call("evo_move_joints", {"targets": {"left_joint1": 99}})
        assert not client._stop.is_set()


def test_partial_observation_is_saved_without_invented_geometry(tmp_path):
    jpeg = b"\xff\xd8fixture\xff\xd9"
    observation = {
        "ok": False,
        "observation_id": "capture-a",
        "cameras": {
            "base": {
                "jpeg": base64.b64encode(jpeg).decode(),
                "width": 960,
                "height": 540,
                "capture_us": 123,
                "geometry_validated": False,
                "camera_to_base": None,
            }
        },
        "errors": {"left_arm": "camera unavailable", "right_arm": "camera unavailable"},
    }
    with service(tmp_path, lambda _: observation) as (client, received):
        value, images = client.call("evo_observe", {})
        assert not value["ok"] and len(images) == 1 and client.session_id is None
        metadata = (
            tmp_path
            / "agent-workspace/runtime/frames"
            / value["frame_id"]
            / "meta.json"
        )
        assert json.loads(metadata.read_text()) == value
        assert (
            tmp_path / "agent-workspace" / value["cameras"]["base"]["path"]
        ).read_bytes() == jpeg
        assert not value["cameras"]["base"]["geometry_validated"]


def test_shared_contract_fixture():
    fixtures = json.loads(Path("services/evo_v3/protocol_fixtures.json").read_text())
    assert fixtures["capabilities_request"]["version"] == 1
    assert fixtures["accepted_response"]["result"]["state"] == "accepted"
    names = {tool["name"] for tool in TOOLS}
    assert names == {
        "evo_move_base",
        "evo_scene",
        "evo_detect_buttons",
        "evo_capabilities",
        "evo_status",
        "evo_observe",
        "evo_history",
        "evo_fk",
        "evo_ik",
        "evo_preview",
        "evo_camera_ray",
        "evo_move_body",
        "evo_follow_path",
        "evo_move_pair",
        "evo_set_grip",
        "evo_move_joints",
        "evo_move_eef",
        "evo_gripper",
        "evo_move_carriage",
        "evo_move_neck",
        "evo_drive",
        "evo_action_status",
        "evo_cancel",
        "evo_hold",
    }


def test_registry_workspace_and_python_helpers(tmp_path):
    from services.backends import backend_class
    from services.workspace import Setup, build, recipe_for

    assert backend_class("evo-v3") is EvoV3Backend
    setup = Setup(robot="evo-v3", observation_profile="rgb-only", gemini=False)
    assert {t["name"] for t in TOOLS} <= set(setup.tools)
    workspace = tmp_path / "ws"
    build(recipe_for(), setup, workspace)
    text = (workspace / ".agents/skills/robot-interface/SKILL.md").read_text()
    assert "evo_action_status" in text and "position_rad" in text
    assert (workspace / "api/evo.py").is_file()
    assert (workspace / "api/cloth.py").is_file()
    assert (workspace / "api/cloth_geometry.py").is_file()
    assert (workspace / ".agents/skills/cloth-handling/SKILL.md").is_file()
    with pytest.raises(ValueError, match="RGB-only"):
        Setup(robot="evo-v3", observation_profile="rgbd")


def test_evo_runtime_paths_and_arming_are_protected(tmp_path):
    from harness.claude_cli.launcher import protected_paths, denied_hosts

    config = SimpleNamespace(
        root=tmp_path,
        robot="evo-v3",
        backend_options={"control_socket": "/tmp/operator-evo/evo.sock"},
    )
    protected = protected_paths(config)
    assert {"localhost", "127.0.0.1", "::1"} <= set(denied_hosts(config))
    for path in [
        Path("/tmp/operator-evo/evo.sock"),
        Path("/tmp/operator-evo"),
        Path.home() / ".local/evo",
        Path.home() / "evo_ws",
        tmp_path / "operator",
    ]:
        assert path in protected
    config.backend_options["control_socket"] = str(tmp_path / "agent/agent.sock")
    with pytest.raises(ValueError, match="outside the private Evo"):
        protected_paths(config)


def test_wait_helper_returns_only_terminal_result():
    from auto_research_agent.api.evo import Evo

    class Context:
        calls = []

        def value(self, name, **args):
            self.calls.append((name, args))
            return {
                "state": "running" if len(self.calls) == 1 else "succeeded",
                "physical_stop_verified": True,
            }

    context = Context()
    result = Evo(context).wait({"action_id": "a"})
    assert result["state"] == "succeeded"
    assert all(
        name == "evo_action_status" and args == {"action_id": "a"}
        for name, args in context.calls
    )


def test_evo_history_skill_matches_the_tool_contract(tmp_path):
    from services.workspace import Setup, build, recipe_for

    workspace = tmp_path / "ws"
    build(
        recipe_for(),
        Setup(robot="evo-v3", observation_profile="rgb-only", gemini=False),
        workspace,
    )
    skill = (workspace / ".agents/skills/live-stream/SKILL.md").read_text()
    assert "evo_history {}" in skill
    assert 'ctx.value("evo_history")' in skill
    assert "runtime/frames/" in skill
    for unsupported in (
        "seconds_ago",
        "contact sheet",
        "runtime/stream/",
        "api.stream",
    ):
        assert unsupported not in skill
    client = EvoV3Backend(SimpleNamespace(root=tmp_path, backend_options={}))
    client.validate("evo_history", {})
    with pytest.raises(ValueError, match="unknown arguments"):
        client.validate("evo_history", {"seconds_ago": [0]})


def test_gripper_discovery_uses_joint_angle_units():
    gripper = next(tool for tool in TOOLS if tool["name"] == "evo_gripper")
    assert (
        "radians" in gripper["description"] and "metres" not in gripper["description"]
    )
    assert "position_rad" in gripper["inputSchema"]["required"]


@contextlib.contextmanager
def frontend_socket(client, tmp_path, audit=lambda _: None):
    from services.controller.frontend import Frontend

    frontend = Frontend(
        SimpleNamespace(config=client.config, audit=audit, close=lambda: None),
        client,
    )
    path = tmp_path / "mcp.sock"
    stop = threading.Event()
    with socket.socket(socket.AF_UNIX) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        listener.bind(str(path))
        listener.listen()
        thread = threading.Thread(
            target=frontend.serve_socket, args=(listener, stop), daemon=True
        )
        thread.start()
        try:
            yield path
        finally:
            stop.set()
            thread.join(2)
            frontend.close()
            assert not thread.is_alive()


@pytest.mark.parametrize("stop_tool", ["evo_cancel", "evo_hold"])
@pytest.mark.parametrize("slow_tool", ["evo_observe", "evo_fk", "evo_detect_buttons"])
def test_frontend_socket_stop_bypasses_slow_request(tmp_path, stop_tool, slow_tool):
    from concurrent.futures import ThreadPoolExecutor
    from auto_research_agent.api.runtime import Context

    blocked = threading.Event()
    release = threading.Event()

    def handler(request):
        if request["method"] == slow_tool.removeprefix("evo_"):
            blocked.set()
            assert release.wait(3), "test did not release ordinary request"
            return {"cameras": {}} if slow_tool in {"evo_observe", "evo_detect_buttons"} else {"poses": {}}
        if request["method"] in ("cancel", "hold"):
            return {"action_id": "action-a", "state": "stopping"}
        if request["method"] == "action_status":
            return {
                "action_id": "action-a",
                "state": "cancelled",
                "physical_stop_verified": True,
            }
        return {"action_id": "action-a", "state": "accepted"}

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as path:
        with Context(str(path)) as ordinary, ThreadPoolExecutor(max_workers=1) as pool:
            action = ordinary.value(
                "evo_follow_path",
                frame="base_link",
                waypoints=[
                    {
                        "duration_s": 3.0,
                        "left": {
                            "position": [0.4, 0.2, 0.6],
                            "quaternion_xyzw": [0, 0, 0, 1],
                        },
                    }
                ],
            )
            params = {"evo_observe": {}, "evo_fk": {"links": ["left_palm"]},
                      "evo_detect_buttons": {"camera": "left_arm", "score_threshold": 0.6}}[slow_tool]
            pending = pool.submit(ordinary.value, slow_tool, **params)
            try:
                assert blocked.wait(1)
                # Establish a new MCP connection while an ordinary request is blocked.
                with Context(str(path)) as urgent:
                    urgent._socket.settimeout(1)
                    args = (
                        {"action_id": action["action_id"]}
                        if stop_tool == "evo_cancel"
                        else {}
                    )
                    # Refuse ordinary work instead of letting it run after this stop.
                    with pytest.raises(RuntimeError, match="Frontend workers busy"):
                        urgent.value("evo_move_neck", yaw_rad=0.2, pitch_rad=0)
                    assert urgent.value(stop_tool, **args)["state"] == "stopping"
                    assert urgent.value(
                        "evo_action_status", action_id=action["action_id"]
                    )["physical_stop_verified"]
                assert not pending.done()
                assert any(
                    r["method"] == stop_tool.removeprefix("evo_") for r in received
                )
                assert (
                    not client._stop.is_set()
                ), "stop must reach Evo while its lease remains live"
            finally:
                release.set()
            pending.result(timeout=2)
            assert sum(r["method"] == "follow_path" for r in received) == 1
            assert not any(r["method"] == "move_neck" for r in received)


def test_frontend_socket_stop_bypasses_blocked_image_write(tmp_path):
    from auto_research_agent.api.runtime import Context

    observed = threading.Event()
    encoded = base64.b64encode(
        b"\xff\xd8" + b"a" * (2 * 1024 * 1024) + b"\xff\xd9"
    ).decode()

    def handler(request):
        if request["method"] == "observe":
            observed.set()
            return {"cameras": {"base": {"jpeg": encoded}}}
        if request["method"] == "hold":
            return {"state": "stopping"}
        return {"action_id": "action-a", "state": "accepted"}

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as path:
        with Context(str(path)) as urgent, socket.socket(
            socket.AF_UNIX
        ) as image_reader:
            urgent.value("evo_move_neck", yaw_rad=0.1, pitch_rad=0)
            image_reader.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            image_reader.connect(str(path))
            image_reader.sendall(
                json.dumps(
                    {
                        "jsonrpc": "2.0",
                        "id": "image",
                        "method": "tools/call",
                        "params": {"name": "evo_observe", "arguments": {}},
                    }
                ).encode()
                + b"\n"
            )
            assert observed.wait(1)
            # Wait for a response byte but do not consume it. The full image cannot fit.
            image_reader.settimeout(2)
            assert image_reader.recv(1, socket.MSG_PEEK)
            urgent._socket.settimeout(1)
            assert urgent.value("evo_hold")["state"] == "stopping"
            assert any(r["method"] == "hold" for r in received)


@pytest.mark.parametrize("stop_tool", ["evo_cancel", "evo_hold"])
@pytest.mark.parametrize("audit_fault", ["blocked", "failed"])
def test_socket_stop_bypasses_status_and_audit(tmp_path, stop_tool, audit_fault):
    from concurrent.futures import ThreadPoolExecutor
    from auto_research_agent.api.runtime import Context

    entered, release = threading.Event(), threading.Event()

    def audit(event):
        if event["tool"] == "evo_status":
            entered.set()
            assert release.wait(3)
        elif audit_fault == "failed":
            raise OSError("audit storage full")

    with service(tmp_path, lambda _: {"state": "stopping"}) as (client, received):
        client._ensure_session()
        with frontend_socket(client, tmp_path, audit=audit) as path:
            with Context(str(path)) as status, Context(str(path)) as stopper:
                with ThreadPoolExecutor() as pool:
                    pending = pool.submit(status.value, "evo_status")
                    try:
                        assert entered.wait(1)
                        stopper._socket.settimeout(1)
                        args = {"action_id": "a"} if stop_tool == "evo_cancel" else {}
                        assert stopper.value(stop_tool, **args)["state"] == "stopping"
                        assert not pending.done()
                        assert any(
                            r["method"] == stop_tool.removeprefix("evo_")
                            for r in received
                        )
                    finally:
                        release.set()
                    pending.result(timeout=2)


@pytest.mark.parametrize("stop_tool", ["evo_cancel", "evo_hold"])
@pytest.mark.parametrize("response", [None, EvoRefused("server busy")])
def test_failed_stop_withdraws_lease_through_frontend_socket(
    tmp_path, stop_tool, response
):
    from auto_research_agent.api.runtime import Context

    with service(tmp_path, lambda _: response) as (client, received):
        client._ensure_session()
        with frontend_socket(client, tmp_path) as path, Context(str(path)) as context:
            args = {"action_id": "a"} if stop_tool == "evo_cancel" else {}
            with pytest.raises(Exception):
                context.value(stop_tool, **args)
            assert client._stop.is_set()
            assert client._failure
            with pytest.raises(ConnectionError):
                client._ensure_session()
            assert (
                len(
                    [
                        r
                        for r in received
                        if r["method"] == stop_tool.removeprefix("evo_")
                    ]
                )
                == 1
            )


@pytest.mark.parametrize("stop_tool", ["evo_cancel", "evo_hold"])
def test_socket_stop_timeout_withdraws_lease(tmp_path, stop_tool):
    from auto_research_agent.api.runtime import Context

    release = threading.Event()

    def handler(_):
        assert release.wait(3)
        return None

    with service(tmp_path, handler) as (client, received):
        client.options["socket_timeout_s"] = 30
        client._ensure_session()
        with frontend_socket(client, tmp_path) as path, Context(str(path)) as context:
            context._socket.settimeout(1)
            try:
                args = {"action_id": "a"} if stop_tool == "evo_cancel" else {}
                with pytest.raises(Exception):
                    context.value(stop_tool, **args)
                assert client._stop.is_set()
                assert client._failure
            finally:
                release.set()
            assert (
                len(
                    [
                        r
                        for r in received
                        if r["method"] == stop_tool.removeprefix("evo_")
                    ]
                )
                == 1
            )


def test_cloth_action_fixtures_cross_the_adapter_with_session_ownership(tmp_path):
    fixtures = json.loads(Path("services/evo_v3/protocol_fixtures.json").read_text())
    with service(tmp_path, lambda r: {"action_id": r["id"], "state": "accepted"}) as (
        client,
        received,
    ):
        for action in fixtures["cloth_actions"]:
            args = {k: v for k, v in action.items() if k != "kind"}
            client.call("evo_" + action["kind"], args)
        for action in fixtures["cloth_actions"]:
            request = next(r for r in received if r["method"] == action["kind"])
            assert request["session_id"] == client.session_id
            assert request["params"] == {k: v for k, v in action.items() if k != "kind"}


@pytest.mark.parametrize(
    "args",
    [
        {"side": "left", "position_rad": 0.1, "retain": 1},
        {"side": "left", "position_rad": 0.1, "retain": "yes"},
    ],
)
def test_retained_grip_requires_boolean_intent(tmp_path, args):
    with service(tmp_path, lambda _: {}) as (client, received):
        with pytest.raises(ValueError):
            client.call("evo_set_grip", args)
        assert received == []


def test_image_history_retrieval_saves_artifact_and_keeps_capture_identity(tmp_path):
    capture = {
        "observation_id": "old-capture",
        "cameras": {
            "base": {
                "jpeg": base64.b64encode(b"\xff\xd8jpeg fixture\xff\xd9").decode(),
                "capture_us": 42,
                "geometry_validated": False,
            }
        },
    }
    with service(tmp_path, lambda _: capture) as (client, received):
        value, images = client.call("evo_history", {"observation_id": "old-capture"})
        assert value["observation_id"] == "old-capture"
        assert value["cameras"]["base"]["capture_us"] == 42
        assert (
            images
            and (
                tmp_path / "agent-workspace" / value["cameras"]["base"]["path"]
            ).read_bytes()
            == b"\xff\xd8jpeg fixture\xff\xd9"
        )
        assert received[0]["params"] == {"observation_id": "old-capture"}
