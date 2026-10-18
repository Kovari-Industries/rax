"""DexJoCo robot-only motion preview and rotation math: contract, validation, packets and routing.

No simulator: the backend runs against fake episode and preview processes. Tests that need the
official sources read them (never import them) from the pinned DexJoCo checkout.
"""
import ast
import json
import math
import re
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services.benchmarks import ALL_UNSCOPED_TOOLS, BENCHMARKS, DEXJOCO_TOOLS
from services.dexjoco import rotation_math as RM
from services.dexjoco import tasks as TK
from services.dexjoco import tools as T
from services.mcp_contract import Contract
from services.simulator.io import RejectedBeforeAction

from local_tests.unit.test_dexjoco_backend import FakeProcess, assert_public, backend_for, needs_source

PREVIEW = {T.PREVIEW_DESCRIBE, T.PREVIEW_RUN}
HOME = [-0.443, 0.0, 1.517, 0.0, -1.0, 0.0, 0.0]  # Flange pose at the official home posture.
STATE = HOME + [0.0] * 12 + [0.263, 0.0, 0.0, 0.0]


def row(state=STATE):
    return RM.rotation_math({"operation": "row_from_state", "state": list(state)})["row"]


class FakePreview:
    """In-memory stand-in for services.dexjoco.preview.PreviewProcess."""

    def __init__(self, error=None):
        self.runs, self.stopped, self.error = [], False, error
        self.metadata = {"cameras": {c: {"width": 8, "height": 8, "frame": "world"} for c in T.PREVIEW_CAMERAS}}

    def run(self, state, actions):
        if self.error:
            raise self.error
        self.runs.append((state, actions))
        steps = [i for i in range(1, len(actions) + 1) if i % T.FRAME_STRIDE == 0 or i == len(actions)]
        frames = [{"step": s, "state": [float(s)] * len(state),
                   "images": {c: np.full((8, 8, 3), 50 + s, np.uint8) for c in T.PREVIEW_CAMERAS}}
                  for s in [0, *steps]]
        return {"status": "Success", "rows": len(actions), "frames": frames,
                "settle": {"converged": True, "control_steps": 4, "flange_position_error_m": 1e-4},
                "start_state": list(state), "end_state": [v + 1.0 for v in state],
                "rows_with_clipped_hand_targets": 0, "seconds": 0.1}

    def stop(self):
        self.stopped = True


def preview_backend(tmp_path, profile="rgbd", task="pick_bucket", preview=None):
    backend = backend_for(tmp_path, profile, task=task)
    backend.preview = preview or FakePreview()
    return backend


# ------------------------------------------------------------------ registry
def test_preview_and_rotation_math_are_dexjoco_tools_that_never_move_the_episode():
    bench = BENCHMARKS["dexjoco"]
    assert PREVIEW | {T.ROTATION_MATH} <= DEXJOCO_TOOLS
    assert not (PREVIEW | {T.ROTATION_MATH}) & bench.mutating and not bench.previewable
    assert bench.unscoped_tools == {T.ROTATION_MATH} and T.ROTATION_MATH in ALL_UNSCOPED_TOOLS
    # The episode server first (dashboards and cleanup read it), then the preview server.
    assert bench.simulator_modules == ("services.dexjoco.server", "services.dexjoco.preview_server")
    rgb_only = Contract(observation_profile="rgb-only", benchmark="dexjoco")
    assert PREVIEW | {T.ROTATION_MATH} <= rgb_only.robot_names
    assert {t["name"] for t in bench.definitions()} == DEXJOCO_TOOLS


def test_rotation_math_is_session_wide_and_the_preview_environment_scoped():
    contract = Contract(benchmark="dexjoco", environments=3)
    assert T.ROTATION_MATH not in contract.scoped_tools and PREVIEW <= contract.scoped_tools
    schemas = {t["name"]: t["inputSchema"] for t in contract.robot_definitions()}
    assert "env_id" not in schemas[T.ROTATION_MATH]["properties"]
    assert "env_id" in schemas[T.PREVIEW_RUN]["required"]
    contract.require_robot(T.ROTATION_MATH, {"operation": "convert_rotation"})
    with pytest.raises(ValueError, match="env_id"):
        contract.require_robot(T.ROTATION_MATH, {"env_id": 0})
    with pytest.raises(ValueError, match="env_id"):
        contract.require_robot(T.PREVIEW_RUN, {"actions": []})


def test_in_container_session_tools_are_every_benchmarks_unscoped_tools():
    from auto_research_agent.api.runtime import SESSION_TOOLS, Environment
    assert SESSION_TOOLS == {"gemini_generate", "status"} | ALL_UNSCOPED_TOOLS
    calls = []
    context = SimpleNamespace(call=lambda tool, **arguments: calls.append((tool, arguments)))
    robot = Environment(context, 2)
    robot.call(T.ROTATION_MATH, operation="convert_rotation")
    robot.call(T.PREVIEW_RUN, actions=[])
    assert [("env_id" in arguments) for _, arguments in calls] == [False, True]
    assert calls[1][1]["env_id"] == 2


def test_every_benchmark_serves_exactly_its_unscoped_tools():
    for bench in BENCHMARKS.values():
        with pytest.raises(ValueError, match="unscoped"):
            bench.unscoped_call(bench.status_tool, {})
    value = BENCHMARKS["dexjoco"].unscoped_call(T.ROTATION_MATH, {
        "operation": "convert_rotation", "rotation": {"axis_angle": {"axis": [0, 0, 2], "angle_rad": math.pi / 2}}})
    assert np.allclose(value["rotation"]["rotation_vector"], [0, 0, math.pi / 2])


# ---------------------------------------------------------------- validation
def test_preview_request_validation():
    single, dual = row(), row(STATE[:7] + STATE[:7] + STATE[7:] + STATE[7:])
    assert len(single) == 22 and len(dual) == 44
    T.validate_request(T.PREVIEW_RUN, {"actions": [single] * 3}, action_dim=22)
    T.validate_request(T.PREVIEW_RUN, {"actions": [single], "initial_state": STATE}, action_dim=22)
    T.validate_request(T.PREVIEW_RUN, {"actions": [dual], "initial_state": STATE[:7] * 2 + STATE[7:] * 2})
    T.validate_request(T.PREVIEW_DESCRIBE, {})
    zero_quaternion = STATE[:3] + [0.0] * 4 + STATE[7:]
    for bad in ({}, {"actions": []}, {"actions": [single], "steps": 3},
                {"actions": [single], "initial_state": STATE[:-1]},
                {"actions": [single], "initial_state": STATE[:7] * 2 + STATE[7:] * 2},
                {"actions": [single], "initial_state": zero_quaternion},
                {"actions": [single], "initial_state": STATE[:-1] + [float("nan")]},
                {"actions": [[float("inf")] * 22]}):
        with pytest.raises(RejectedBeforeAction):
            T.validate_request(T.PREVIEW_RUN, bad, action_dim=22)
    with pytest.raises(RejectedBeforeAction, match="22-D"):
        T.validate_request(T.PREVIEW_RUN, {"actions": [dual]}, action_dim=22)
    with pytest.raises(RejectedBeforeAction):
        T.validate_request(T.PREVIEW_DESCRIBE, {"actions": [single]})


def test_rotation_math_validation():
    T.validate_request(T.ROTATION_MATH, {"operation": "convert_rotation", "rotation": {"rotation_vector": [0, 0, 1]}})
    for bad in ({}, {"operation": "rotate"}, {"operation": "convert_rotation"},
                {"operation": "convert_rotation", "rotation": {"rotation_vector": [0, 0, 1]}, "state": STATE},
                {"operation": "convert_rotation", "rotation": {"rotation_vector": [0, 0, 1], "quaternion_wxyz": [1, 0, 0, 0]}},
                {"operation": "convert_rotation", "rotation": {"quaternion_wxyz": [0, 0, 0, 0]}},
                {"operation": "convert_rotation", "rotation": {"axis_angle": {"axis": [0, 0, 0], "angle_rad": 1}}},
                {"operation": "compose_pose", "pose_a": {}, "pose_b": {"position_m": [0, 0, 1]}},
                {"operation": "flange_target", "point_target_m": [0, 0, 1], "rotation": {"rotation_vector": [0, 0, 1]}},
                {"operation": "row_from_state", "state": STATE[:-1]}):
        with pytest.raises(RejectedBeforeAction):
            T.validate_request(T.ROTATION_MATH, bad)
    with pytest.raises(RejectedBeforeAction, match="rotation matrix"):  # A reflection is not a rotation.
        RM.rotation_math({"operation": "convert_rotation", "rotation": {"matrix": [[1, 0, 0], [0, 1, 0], [0, 0, -1]]}})


# ----------------------------------------------------------- rotation values
def test_rotation_math_conversions_are_the_official_ones():
    from scipy.spatial.transform import Rotation
    value = RM.rotation_math({"operation": "convert_rotation",
                              "rotation": {"axis_angle": {"axis": [0, 0, 1], "angle_rad": math.pi / 2}}})["rotation"]
    assert np.allclose(value["rotation_vector"], [0, 0, math.pi / 2])
    assert np.allclose(value["quaternion_wxyz"], [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])
    assert np.allclose(value["matrix"], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
    # The measured home quaternion becomes the rotation vector the official hold sends.
    home = RM.rotation_math({"operation": "convert_rotation", "rotation": {"quaternion_wxyz": HOME[3:7]}})["rotation"]
    official = Rotation.from_quat(HOME[3:7], scalar_first=True).as_rotvec()
    assert np.allclose(home["rotation_vector"], official) and math.isclose(home["angle_rad"], math.pi)
    assert np.allclose(home["quaternion_wxyz"], Rotation.from_rotvec(official).as_quat(scalar_first=True))


def test_near_rotation_vector_keeps_rows_on_one_side_of_pi():
    rotation = {"rotation_vector": [math.pi - 0.1, 0, 0]}
    plain = RM.rotation_math({"operation": "convert_rotation", "rotation": rotation})["rotation"]
    near = RM.rotation_math({"operation": "convert_rotation", "rotation": rotation,
                             "near_rotation_vector": [-3.1, 0, 0]})["rotation"]
    assert np.allclose(plain["rotation_vector"], [math.pi - 0.1, 0, 0])
    assert np.allclose(near["rotation_vector"], [-(math.pi + 0.1), 0, 0])
    # Both describe the same orientation; the official conversion gives sign-flipped quaternions.
    assert np.allclose(np.abs(near["quaternion_wxyz"]), np.abs(plain["quaternion_wxyz"]))
    assert np.allclose(near["matrix"], plain["matrix"])


def test_poses_compose_relate_and_place_a_hand_point():
    flange = {"position_m": [0.1, -0.2, 1.3], "rotation": {"quaternion_wxyz": HOME[3:7]}}
    tip = {"position_m": [0.15, 0.02, -0.03], "rotation": {"axis_angle": {"axis": [0, 1, 0], "angle_rad": 0.3}}}
    world = RM.rotation_math({"operation": "compose_pose", "pose_a": flange, "pose_b": tip})["pose"]
    # The flange's z axis points down at home, so a point 3 cm below the flange along -z is above it.
    assert np.allclose(world["position_m"], [0.25, -0.22, 1.33])
    back = RM.rotation_math({"operation": "relative_pose", "pose_a": flange,
                             "pose_b": {"position_m": world["position_m"],
                                        "rotation": {"rotation_vector": world["rotation_vector"]}}})["pose"]
    assert np.allclose(back["position_m"], tip["position_m"]) and np.allclose(back["rotation_vector"], [0, 0.3, 0])
    assert back["row_prefix"] == back["position_m"] + back["rotation_vector"]
    target = RM.rotation_math({"operation": "flange_target", "point_target_m": world["position_m"],
                               "point_offset_m": tip["position_m"], "rotation": flange["rotation"]})["flange_pose"]
    assert np.allclose(target["position_m"], flange["position_m"])
    assert np.allclose(target["quaternion_wxyz"], HOME[3:7]) or np.allclose(target["quaternion_wxyz"], -np.array(HOME[3:7]))


def test_row_from_state_is_the_official_hold_row():
    from scipy.spatial.transform import Rotation
    hand = list(np.linspace(0.0, 0.5, 16))
    value = RM.rotation_math({"operation": "row_from_state", "state": HOME + hand})
    rotvec = Rotation.from_quat(HOME[3:7], scalar_first=True).as_rotvec().tolist()
    assert value["row"] == HOME[:3] + rotvec + hand and value["row_dim"] == 22
    left = [0.0, 0.4, 1.5, 0.0, -1.0, 0.0, 0.0]
    right_hand, left_hand = hand, [v + 1 for v in hand]
    dual = RM.rotation_math({"operation": "row_from_state", "state": HOME + left + right_hand + left_hand})
    # State [right pose, left pose, right hand, left hand] -> row [right pose, right hand, left pose, left hand].
    assert dual["row"] == HOME[:3] + rotvec + right_hand + left[:3] + rotvec + left_hand and dual["row_dim"] == 44


# ------------------------------------------------------------------- backend
def test_describe_never_starts_the_preview(tmp_path):
    backend = preview_backend(tmp_path)
    value, images = backend.call(T.PREVIEW_DESCRIBE, {})
    assert images == [] and backend.preview.runs == []
    assert set(value["cameras"]) == set(T.PREVIEW_CAMERAS) and value["counts_toward_episode_steps"] is False
    assert "22-D" in value["inputs"]["actions"] and "23-D" in value["inputs"]["initial_state"]


def test_preview_defaults_to_the_measured_episode_state(tmp_path):
    backend = preview_backend(tmp_path)
    backend.start(formal=False, seed=0)
    backend.call(T.STEP, {"actions": [row()] * 2})
    before, observes = backend.sim.step_id, sum(op == "observe" for op, _ in backend.sim.requests)
    value, images = backend.call(T.PREVIEW_RUN, {"actions": [row()] * 13})
    state, actions = backend.preview.runs[-1]
    measured = FakeProcess(tmp_path).observation()["state"]
    assert state == measured and len(actions) == 13 and len(images) == len(T.PREVIEW_CAMERAS)
    # The last motion's observation is current: no extra observe request.
    assert sum(op == "observe" for op, _ in backend.sim.requests) == observes
    assert backend.sim.step_id == before and "step_id" not in value
    assert value["live_step_id"] == before and value["live_episode_id"] == backend.sim.episode_id
    assert value["start_source"] == "episode_measured_state" and value["requested_start_state"] == measured
    assert value["end_state"] == [v + 1.0 for v in measured] and value["start_settle"]["converged"] is True
    assert_public(value)
    backend._raw = None  # A fresh backend observes once for the state.
    backend.call(T.PREVIEW_RUN, {"actions": [row()]})
    assert sum(op == "observe" for op, _ in backend.sim.requests) == observes + 1


def test_explicit_initial_state_wins_and_needs_no_episode(tmp_path):
    backend = preview_backend(tmp_path)
    value, _ = backend.call(T.PREVIEW_RUN, {"actions": [row()], "initial_state": STATE})
    assert backend.preview.runs[-1][0] == STATE and value["live_step_id"] is None
    assert value["start_source"] == "initial_state" and backend.sim.requests == []


def test_preview_without_episode_or_state_or_with_another_robots_rows_is_rejected(tmp_path):
    backend = preview_backend(tmp_path)
    with pytest.raises(RuntimeError, match="initial_state"):
        backend.call(T.PREVIEW_RUN, {"actions": [row()]})
    with pytest.raises(RejectedBeforeAction, match="22-D"):
        backend.call(T.PREVIEW_RUN, {"actions": [[0.0] * 44], "initial_state": STATE})
    with pytest.raises(RejectedBeforeAction, match="23-D"):
        backend.call(T.PREVIEW_RUN, {"actions": [row()], "initial_state": STATE + STATE})
    assert backend.preview.runs == []


def test_frames_are_saved_with_their_states_and_only_final_frames_returned(tmp_path):
    backend = preview_backend(tmp_path)
    value, images = backend.call(T.PREVIEW_RUN, {"actions": [row()] * 13, "initial_state": STATE})
    sequence = value["frame_sequence"]
    assert sequence["frame_count"] == 3 and len(images) == len(T.PREVIEW_CAMERAS)
    assert sequence["manifest_path"].startswith("runtime/frames/dexjoco_preview/")
    manifest = json.loads((backend.frame_workspace / sequence["manifest_path"]).read_text())
    assert [f["preview_row"] for f in manifest["frames"]] == [0, 10, 13]
    assert [f["state"][0] for f in manifest["frames"]] == [0.0, 10.0, 13.0]
    from services.controller.recording import IMAGE_NAME
    for frame in manifest["frames"]:
        assert {item["camera"] for item in frame["files"]} == set(T.PREVIEW_CAMERAS)
        for item in frame["files"]:
            assert IMAGE_NAME.fullmatch(Path(item["path"]).name)
            assert (backend.frame_workspace / item["path"]).is_file()


def test_rotation_math_runs_in_the_backend_without_an_episode(tmp_path):
    backend = preview_backend(tmp_path)
    value, images = backend.call(T.ROTATION_MATH, {"operation": "row_from_state", "state": STATE})
    assert images == [] and value["row"] == row() and backend.sim.requests == []


def gateway_for(backend, tmp_path):
    from services.controller.gateway import Gateway
    from services.controller.recording import RunRecording
    recording = RunRecording(tmp_path / "published", "operate-" + "a" * 32)
    gateway = Gateway(backend, None, contract=backend.contract, audit=recording.event, charge=lambda *_: None,
                      refund=lambda *_: None, budget=lambda: {},
                      publish_sequence=lambda value: recording.sequence(value, backend.frame_workspace))
    token = gateway.acquire()

    def call(name, arguments):
        return gateway.handle(token, {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                                      "params": {"name": name, "arguments": arguments}})
    return gateway, recording, call


def test_recording_publishes_preview_frames_without_counting_episode_steps(tmp_path):
    backend = preview_backend(tmp_path)
    gateway, recording, call = gateway_for(backend, tmp_path)
    backend.start(formal=False, seed=0)
    call(T.STEP, {"actions": [row()] * 2})
    reply = call(T.PREVIEW_RUN, {"actions": [row()] * 7})
    value = json.loads(reply["result"]["content"][0]["text"])
    assert value["frame_sequence"]["frame_count"] == 2 and not recording.errors
    assert value["frame_sequence"]["manifest_path"].startswith("runtime/autonomous_controller/runs/")
    assert recording.start_step == recording.end_step == 2 and value["episode_ended"] is False
    assert recording.paths["final_response"].endswith("calls/000001/response.json")


def test_a_failed_preview_reports_its_error_and_leaves_the_episode_running(tmp_path):
    error = RuntimeError("Preview simulator failed with EOFError: closed; simulator process exited with code -11; "
                         "its log: Segmentation fault; it restarts on the next preview call")
    backend = preview_backend(tmp_path, preview=FakePreview(error))
    gateway, _, call = gateway_for(backend, tmp_path)
    backend.start(formal=False, seed=0)
    reply = call(T.PREVIEW_RUN, {"actions": [row()]})
    assert "Segmentation fault" in reply["error"]["message"] and "exited with code -11" in reply["error"]["message"]
    assert reply["error"]["data"]["episode_ended"] is False and not gateway.terminal
    assert "error" not in call(T.STEP, {"actions": [row()]})


def test_close_stops_the_preview(tmp_path):
    backend = preview_backend(tmp_path)
    backend.close()
    assert backend.preview.stopped


# ------------------------------------------------------------------ frontend
def test_rotation_math_is_served_without_an_episode(tmp_path):
    from services.controller.frontend import ResearchFrontend
    from services.controller.supervisor import Supervisor
    from test_controller_backend import Backend, Runner, configuration
    config = replace(configuration(tmp_path), task="pick_bucket", benchmark="dexjoco", eval_seed=0, formal_eval_seed=1)
    supervisor = Supervisor(config, Backend, runner_factory=Runner)
    try:
        front = ResearchFrontend(supervisor, tmp_path / "workspace")
        names = [t["name"] for t in front.tools()]
        assert names.count(T.ROTATION_MATH) == 1
        reply = front.call(T.ROTATION_MATH, {"operation": "row_from_state", "state": STATE})
        assert not reply.get("isError") and json.loads(reply["content"][0]["text"])["row"] == row()
        bad = front.call(T.ROTATION_MATH, {"operation": "row_from_state", "state": STATE[:5]})
        assert bad["isError"] and "official state vector" in bad["content"][0]["text"]
        with pytest.raises(RuntimeError, match="No interactive episode"):
            front.call(T.PREVIEW_DESCRIBE, {})
        with pytest.raises(PermissionError):
            front.call("not_a_tool", {"operation": "compose_pose"})
    finally:
        supervisor.close()


# ------------------------------------------------------- official control
OPSPACE_STEP = '''
def step(self, action):
    for _ in range(self._n_substeps):
        tau_right = opspace(model=self._model, data=self._data, site_id=1, dof_ids=self._r, pos=p, ori=q,
                            joint=_PANDA_HOME, gravity_comp=True, pos_gains=(400.0, 400.0, 400.0), damping_ratio=4)
        tau_left = opspace(model=self._model, data=self._data, site_id=2, dof_ids=self._l, pos=p, ori=q,
                           joint=_PANDA_HOME, gravity_comp=True, pos_gains=(400.0, 400.0, 400.0), damping_ratio={})
'''


def test_opspace_arguments_are_read_from_the_step_source():
    from services.dexjoco.preview_session import opspace_keywords
    home = [0.0, -0.785, 0.0, -2.35, 0.0, 1.57, math.pi / 4]
    assert opspace_keywords(OPSPACE_STEP.format(4), {"_PANDA_HOME": home}.__getitem__) == {
        "joint": home, "gravity_comp": True, "pos_gains": (400.0, 400.0, 400.0), "damping_ratio": 4}
    with pytest.raises(ValueError, match="different arguments per arm"):
        opspace_keywords(OPSPACE_STEP.format(2), {"_PANDA_HOME": home}.__getitem__)
    with pytest.raises(ValueError, match="no opspace"):
        opspace_keywords("def step(self, action):\n    return action\n", {}.__getitem__)


def official_envs():
    envs = TK.source_root() / "dexjoco" / "dexjoco" / "sim" / "envs"
    for name in TK.OFFICIAL_TASKS:
        path = envs / f"panda_{name}_env.py"
        yield name, path, path.read_text()


@needs_source
def test_every_official_env_steps_with_the_same_controller_and_home():
    from services.dexjoco.preview_session import opspace_keywords
    found = {}
    for name, path, source in official_envs():
        tree = ast.parse(source)
        step = next(node for cls in tree.body if isinstance(cls, ast.ClassDef)
                    for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "step")
        found[name] = opspace_keywords(ast.get_source_segment(source, step), lambda n: n, name)
        home = next(node.value for node in tree.body if isinstance(node, ast.Assign)
                    and getattr(node.targets[0], "id", None) == "_PANDA_HOME")
        assert "(0, -0.785, 0, -2.35, 0, 1.57, np.pi / 4)" in ast.get_source_segment(source, home), name
    assert all(value == {"joint": "_PANDA_HOME", "gravity_comp": True, "pos_gains": (400.0, 400.0, 400.0),
                         "damping_ratio": 4} for value in found.values()), found


@needs_source
def test_each_task_previews_the_robot_its_official_arena_includes():
    from services.dexjoco.preview_session import robot_file
    for name, path, source in official_envs():
        arena = re.search(r'_XML_PATH = _HERE / "xmls" / "([^"]+)"', source).group(1)
        robot = robot_file(SimpleNamespace(_XML_PATH=path.parent / "xmls" / arena))
        dual = TK.task_info(name)["dual_arm"]
        assert robot.name == ("panda_allegro_bimanual.xml" if dual else "panda_allegro_copy.xml"), name
        assert robot.is_file()
