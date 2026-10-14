"""BEHAVIOR robot-only motion preview: contract, start state, plan rows and recording.

No simulator: the backend runs against fake episode and preview processes.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from services.behavior import robot as R
from services.behavior import tools as T
from services.benchmarks import BEHAVIOR_TOOLS, BENCHMARKS, ROBODOJO_TOOLS
from services.mcp_contract import Contract
from services.simulator.io import RejectedBeforeAction

from local_tests.unit.test_behavior_backend import FakeProcess, assert_public, backend_for, hold_row

STATE = {"trunk": [0.1, 0.2, 0.3, 0.0], "arm_left": [0.0] * 7, "arm_right": [0.0] * 7, "grippers": [1.0, -1.0]}


class FakePreview:
    def __init__(self):
        self.runs, self.measured, self.stopped = [], [], False
        self.metadata = {"cameras": {c: {"width": 8, "height": 8} for c in T.PREVIEW_CAMERAS}}

    def run(self, state, actions, measured=False):
        self.runs.append((state, actions))
        self.measured.append(measured)
        steps = [i for i in range(1, len(actions) + 1) if i % R.FRAME_STRIDE == 0 or i == len(actions)]
        frames = [{"step": s, "images": {c: np.full((8, 8, 3), 50, np.uint8) for c in T.PREVIEW_CAMERAS}}
                  for s in [0, *steps]]
        end = R.decode_proprio(np.arange(61.0) / 100)
        return {"status": "Success", "rows": len(actions), "frames": frames, "start_proprio": end,
                "end_proprio": end, "base_motion": R.Odometry().pose(), "seconds": 0.5}

    def stop(self):
        self.stopped = True


class PlanningProcess(FakeProcess):
    def request(self, op, **kwargs):
        if op == "motion_plan_rows":
            self.requests.append((op, kwargs))
            return {"motion_plan_id": kwargs["motion_plan_id"], "rows": [hold_row().tolist()] * 9}
        return super().request(op, **kwargs)


def preview_backend(tmp_path, process=FakeProcess, profile="rgbd"):
    backend = backend_for(tmp_path, profile)
    backend.sim = process(tmp_path / "native" / "results", success=False)
    backend.preview = FakePreview()
    return backend


# ------------------------------------------------------------------ contract
def test_preview_tools_are_behavior_only_and_not_episode_motion():
    assert {T.PREVIEW_DESCRIBE, T.PREVIEW_RUN} <= BEHAVIOR_TOOLS
    assert not {T.PREVIEW_DESCRIBE, T.PREVIEW_RUN} & ROBODOJO_TOOLS
    assert not {T.PREVIEW_DESCRIBE, T.PREVIEW_RUN} & BENCHMARKS["behavior"].mutating
    assert {T.PREVIEW_DESCRIBE, T.PREVIEW_RUN} <= Contract(observation_profile="rgb-only", benchmark="behavior").robot_names
    assert T.PREVIEW_RUN not in Contract().robot_names


def test_preview_request_validation():
    row = hold_row().tolist()
    T.validate_request(T.PREVIEW_RUN, {"actions": [row]})
    T.validate_request(T.PREVIEW_RUN, {"motion_plan_id": "abc", "initial_state": STATE})
    for bad in ({}, {"actions": [row], "motion_plan_id": "abc"}, {"actions": []},
                {"actions": [row], "initial_state": {**STATE, "trunk": [0.0] * 3}},
                {"actions": [row], "initial_state": {k: v for k, v in STATE.items() if k != "grippers"}},
                {"actions": [row], "initial_state": {**STATE, "grippers": [2.0, 0.0]}}):
        with pytest.raises((RejectedBeforeAction, ValueError)):
            T.validate_request(T.PREVIEW_RUN, bad)


def test_describe_never_starts_the_preview(tmp_path):
    backend = preview_backend(tmp_path)
    value, images = backend.call(T.PREVIEW_DESCRIBE, {})
    assert images == [] and backend.preview.runs == []
    assert set(value["cameras"]) == set(T.PREVIEW_CAMERAS) and value["counts_toward_episode_steps"] is False


# ---------------------------------------------------------------- start state
def test_preview_defaults_to_the_measured_episode_state(tmp_path):
    backend = preview_backend(tmp_path)
    backend.start(formal=False, seed=0)
    backend.call("behavior_step", {"actions": [hold_row().tolist()] * 2})
    before = backend.sim.step_id
    value, images = backend.call(T.PREVIEW_RUN, {"actions": [hold_row().tolist()] * 13})
    state, actions = backend.preview.runs[-1]
    fields = R.decode_proprio(np.arange(61.0))
    assert state["trunk"] == fields["trunk_qpos"] and state["arm_left"] == fields["arm_left_qpos"]
    assert state["grippers"] == [1, 1] and state["gripper_left_qpos"] == fields["gripper_left_qpos"]
    assert len(actions) == 13 and backend.preview.measured[-1] is True  # Drift past a limit is clamped.
    assert backend.sim.step_id == before and "step_id" not in value
    assert value["live_step_id"] == before and value["live_episode_id"] == backend.sim.episode_id
    assert "gripper_left_qpos" not in value["start_state"]
    assert_public(value)


def test_explicit_initial_state_wins_and_needs_no_episode(tmp_path):
    backend = preview_backend(tmp_path)
    value, _ = backend.call(T.PREVIEW_RUN, {"actions": [hold_row().tolist()], "initial_state": STATE})
    assert backend.preview.runs[-1][0] == STATE and value["live_step_id"] is None
    assert backend.preview.measured[-1] is False  # Caller input is validated, not clamped.
    assert all(op != "observe" for op, _ in backend.sim.requests)


def test_preview_without_episode_or_state_is_rejected(tmp_path):
    backend = preview_backend(tmp_path)
    with pytest.raises(RuntimeError, match="initial_state"):
        backend.call(T.PREVIEW_RUN, {"actions": [hold_row().tolist()]})
    with pytest.raises(RuntimeError, match="motion_plan_id"):
        backend.call(T.PREVIEW_RUN, {"motion_plan_id": "abc", "initial_state": STATE})
    assert backend.preview.runs == []


def test_motion_plan_id_previews_the_cached_plan_rows(tmp_path):
    backend = preview_backend(tmp_path, PlanningProcess)
    backend.start(formal=False, seed=0)
    value, _ = backend.call(T.PREVIEW_RUN, {"motion_plan_id": "plan-1"})
    assert ("motion_plan_rows", {"motion_plan_id": "plan-1"}) in backend.sim.requests
    assert len(backend.preview.runs[-1][1]) == 9 and value["source"] == {"motion_plan_id": "plan-1"}


def test_planner_plan_rows_keep_the_plan_executable():
    from types import SimpleNamespace
    from services.behavior.planner import ArmPlanner
    planner = ArmPlanner.__new__(ArmPlanner)
    planner.session = SimpleNamespace(episode_id="ep", step_id=4)
    planner.plans, planner.retired = {}, {}
    rows = np.zeros((3, R.ACTION_DIM))
    planner._store("p", ("ep", 4, rows))
    assert planner.plan_rows("p")["rows"] is rows and "p" in planner.plans
    planner.session.step_id = 5
    with pytest.raises(RejectedBeforeAction, match="Plan_Stale"):
        planner.plan_rows("p")
    with pytest.raises(RejectedBeforeAction, match="q cannot run: no preview_only plan with this id was made"):
        planner.plan_rows("q")


def test_a_replaced_or_executed_plan_says_why_it_cannot_run():
    from types import SimpleNamespace
    from services.behavior.planner import ArmPlanner
    planner = ArmPlanner.__new__(ArmPlanner)
    planner.plans, planner.retired = {}, {}
    executed = []
    planner.session = SimpleNamespace(episode_id="ep", step_id=4,
                                      _execute=lambda rows, kind: executed.append(kind) or {"step_id": 9})
    rows = np.zeros((3, R.ACTION_DIM))
    planner._store("first", ("ep", 4, rows))
    planner._store("second", ("ep", 4, rows))
    # The first plan, made just before the second, is gone: the agent learns it was replaced.
    for use in (planner.plan_rows, planner.execute):
        with pytest.raises(RejectedBeforeAction, match="first cannot run: replaced by the newer plan second; "
                                                       "only the latest preview_only plan is kept"):
            use("first")
    assert planner.execute("second")["executed"] is True and executed == ["free_space_move"]
    with pytest.raises(RejectedBeforeAction, match="second cannot run: already executed from step 4"):
        planner.execute("second")


# --------------------------------------------------------- frames and recording
def test_frames_are_saved_and_only_final_frames_returned(tmp_path):
    backend = preview_backend(tmp_path)
    value, images = backend.call(T.PREVIEW_RUN, {"actions": [hold_row().tolist()] * 13, "initial_state": STATE})
    sequence = value["frame_sequence"]
    assert sequence["frame_count"] == 4 and len(images) == len(T.PREVIEW_CAMERAS)
    assert sequence["manifest_path"].startswith("runtime/frames/behavior_preview/")
    manifest = json.loads((backend.frame_workspace / sequence["manifest_path"]).read_text())
    assert [f["preview_row"] for f in manifest["frames"]] == [0, 6, 12, 13]
    from services.controller.recording import IMAGE_NAME
    for frame in manifest["frames"]:
        assert {item["camera"] for item in frame["files"]} == set(T.PREVIEW_CAMERAS)
        for item in frame["files"]:
            assert IMAGE_NAME.fullmatch(Path(item["path"]).name)
            assert (backend.frame_workspace / item["path"]).is_file()


def test_recording_publishes_preview_frames_without_counting_episode_steps(tmp_path):
    from services.controller.gateway import Gateway
    from services.controller.recording import RunRecording
    backend = preview_backend(tmp_path)
    recording = RunRecording(tmp_path / "published", "operate-" + "a" * 32)
    gateway = Gateway(backend, None, contract=backend.contract, audit=recording.event, charge=lambda *_: None,
                      refund=lambda *_: None, budget=lambda: {},
                      publish_sequence=lambda value: recording.sequence(value, backend.frame_workspace))
    token = gateway.acquire()
    backend.start(formal=False, seed=0)
    call = lambda name, arguments: gateway.handle(token, {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                                                          "params": {"name": name, "arguments": arguments}})
    call("behavior_step", {"actions": [hold_row().tolist()] * 2})
    reply = call(T.PREVIEW_RUN, {"actions": [hold_row().tolist()] * 7})
    value = json.loads(reply["result"]["content"][0]["text"])
    assert value["frame_sequence"]["frame_count"] == 3 and not recording.errors
    assert recording.start_step == recording.end_step == 2
    assert recording.paths["final_response"].endswith("calls/000001/response.json")


def test_close_stops_the_preview(tmp_path):
    backend = preview_backend(tmp_path)
    backend.close()
    assert backend.preview.stopped


def test_planner_keeps_one_curobo_generator_per_process(monkeypatch):
    """Two generators' CUDA graphs in one simulator crashed it; switching variants frees the old one."""
    import sys
    from types import ModuleType, SimpleNamespace
    import torch
    from services.behavior.planner import ArmPlanner
    built, calls = [], []
    curobo = ModuleType("omnigibson.action_primitives.curobo")
    curobo.CuRoboEmbodimentSelection = SimpleNamespace(ARM="arm")
    curobo.CuRoboMotionGenerator = lambda robot, robot_cfg_path, **kw: built.append(robot_cfg_path["arm"]) or object()
    util = ModuleType("curobo.util_file")
    util.get_task_configs_path, util.join_path = lambda: "", lambda *p: "/".join(p)
    util.load_yaml = lambda path: {"lbfgs": {}}
    for name, module in (("curobo", ModuleType("curobo")), (util.__name__, util),
                         ("omnigibson", ModuleType("omnigibson")),
                         ("omnigibson.action_primitives", ModuleType("omnigibson.action_primitives")),
                         (curobo.__name__, curobo)):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(torch.cuda, "synchronize", lambda: calls.append("sync"))
    monkeypatch.setattr(torch.cuda, "empty_cache", lambda: calls.append("empty"))
    planner = ArmPlanner(SimpleNamespace(robot=SimpleNamespace(curobo_path={"arm": "r1pro_curobo_arm.yaml"})))
    first = planner.generator(True)
    assert planner.generator(True) is first and calls == []
    planner.generator(False)
    assert list(planner._generators) == ["arm_no_torso"] and calls == ["sync", "empty"]
    planner.generator(True)
    assert list(planner._generators) == ["arm"]
    assert built == ["r1pro_curobo_arm.yaml", "r1pro_curobo_arm_no_torso.yaml", "r1pro_curobo_arm.yaml"]
