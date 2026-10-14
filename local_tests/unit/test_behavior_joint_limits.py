"""Regression requests from both radio rehearsals; no simulator or live processes."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services.behavior import robot as R, tools as T
from services.behavior.preview_session import PreviewSession
from services.behavior.session import BehaviorSession
from services.controller.gateway import Gateway
from services.mcp_contract import Contract
from services.simulator.io import RejectedBeforeAction

EVIDENCE = json.loads((Path(__file__).parent / "fixtures/behavior_radio_joint_limits.json").read_text())
LOWER = np.asarray(EVIDENCE["limits"]["lower"])
UPPER = np.asarray(EVIDENCE["limits"]["upper"])


@pytest.mark.parametrize("sample", EVIDENCE["samples"])
def test_recorded_rehearsal_requests_clamp_to_real_limits_and_report(sample):
    actions = np.asarray(sample["actions"])
    original = actions.copy()
    report = {}
    rows = R.validate_actions(actions, LOWER, UPPER, diagnostics=report)
    assert np.array_equal(actions, original)
    assert rows.dtype == np.float32
    assert R.joint_limit_violations(rows, LOWER, UPPER) == []
    assert report["count"] > 0 and report["max_excess_rad"] < 1e-5
    assert report["tolerance_rad"] == 1e-4
    for item in report["corrections"]:
        assert rows[item["row_index"], item["action_index"]] == item["clamped_rad"]


@pytest.mark.parametrize("part", ["trunk", "arm_left", "arm_right"])
@pytest.mark.parametrize("direction", [-1, 1])
def test_tolerance_boundary_is_bounded_and_larger_targets_reject_whole_batch(part, direction):
    # Exact binary bounds avoid testing decimal rounding instead of the policy.
    low, high = np.full(23, -1.0), np.ones(23)
    index = R.ACTION_SLICES[part].start
    rows = np.zeros((2, 23))
    rows[0, index] = direction * (1 + R.JOINT_LIMIT_TOLERANCE_RAD)
    report = {}
    assert R.validate_actions(rows, low, high, diagnostics=report)[0, index] == direction
    rows[1, index] = direction * (1 + R.JOINT_LIMIT_TOLERANCE_RAD * 1.01)
    original = rows.copy()
    with pytest.raises(RejectedBeforeAction, match=rf"{part}.*row 1, joint 0.*excess.*tolerance"):
        R.validate_actions(rows, low, high)
    assert np.array_equal(rows, original)


@pytest.mark.parametrize("index,value", [(0, 1.00001), (14, -1.00001), (22, 1.00001), (7, np.nan), (3, np.inf)])
def test_normalized_commands_and_nonfinite_targets_remain_rejected(index, value):
    row = np.zeros(23)
    row[index] = value
    with pytest.raises(RejectedBeforeAction):
        R.validate_actions([row], LOWER, UPPER)


def test_correction_report_is_bounded_but_counts_the_entire_batch():
    row = EVIDENCE["samples"][0]["actions"][0]
    report = {}
    R.validate_actions([row] * R.MAX_ACTION_ROWS, LOWER, UPPER, diagnostics=report)
    assert report["count"] == R.MAX_ACTION_ROWS
    assert len(report["corrections"]) == 20 and report["corrections_truncated"]


def test_joint_step_validates_before_execution_and_publishes_actual_rows():
    session = BehaviorSession.__new__(BehaviorSession)
    session.lower, session.upper = LOWER, UPPER
    executed = []
    def execute(rows, **kwargs):
        executed.append(rows)
        return kwargs["extra"] or {}
    session._execute = execute
    actions = np.asarray(EVIDENCE["samples"][0]["actions"])
    result = session.chunk_step(actions)
    assert result["joint_target_clamping"]["count"] == 10
    assert executed[0][0, 18] == UPPER[18]
    actions[-1, 18] = UPPER[18] + 0.001
    with pytest.raises(RejectedBeforeAction):
        session.chunk_step(actions)
    assert len(executed) == 1


def preview_session():
    session = PreviewSession.__new__(PreviewSession)
    session.lower, session.upper = LOWER, UPPER
    session.th = SimpleNamespace(as_tensor=lambda v, **_: np.asarray(v), float32=np.float32)
    session.moves = []
    session.env = SimpleNamespace(step=lambda row: session.moves.append(row))
    session.starts = []
    session._set_state = lambda state: session.starts.append(state)
    session._planar_pose = lambda: (0, 0, 0)
    session._proprio = lambda: np.zeros(61)
    session._frame = lambda step: {"step": step}
    return session


def initial_state(row):
    return {**{p: row[R.ACTION_SLICES[p]].tolist() for p in ("trunk", "arm_left", "arm_right")},
            "grippers": [1, 1]}


def test_preview_clamps_rows_and_measured_initial_state_before_setting_state():
    session = preview_session()
    actions = np.asarray(EVIDENCE["samples"][1]["actions"])
    state = initial_state(actions[0])
    result = session.run(state, actions)
    assert result["joint_target_clamping"]["count"] == 1
    assert result["initial_state_joint_clamping"]["count"] == 1
    assert session.starts[0]["arm_left"][5] == LOWER[12]
    assert result["initial_state"]["arm_left"][5] == LOWER[12]
    assert state["arm_left"][5] < LOWER[12]  # Caller input remains intact.
    assert all(not R.joint_limit_violations(row, LOWER, UPPER) for row in session.moves)


@pytest.mark.parametrize("excess", [1.22e-4, 0.02])
def test_preview_clamps_a_measured_start_state_past_a_limit(excess):
    """The default preview starts from the live measured joints: drift past a limit (seen at
    1.22e-4 rad in a real session) is clamped and reported, never rejected as caller input."""
    session = preview_session()
    row = np.clip(np.zeros(R.ACTION_DIM), LOWER, UPPER)
    state = initial_state(row)
    state["arm_left"][5] = LOWER[12] - excess
    result = session.run(state, [row.tolist()], measured=True)
    assert session.starts[0]["arm_left"][5] == pytest.approx(LOWER[12])
    report = result["initial_state_joint_clamping"]
    assert report["policy"] == "clamp_measured_hold" and report["count"] == 1
    assert report["max_excess_rad"] == pytest.approx(excess)
    assert state["arm_left"][5] == LOWER[12] - excess  # The measurement itself is not altered.
    explicit = preview_session()
    with pytest.raises(RejectedBeforeAction, match="initial_state:"):
        explicit.run(state, [row.tolist()])
    assert explicit.starts == []


@pytest.mark.parametrize("bad_input", ["last_action", "initial_state", "nonfinite_state"])
def test_preview_rejects_entire_request_before_any_state_or_physics_change(bad_input):
    session = preview_session()
    actions = np.asarray(EVIDENCE["samples"][1]["actions"])
    state = initial_state(actions[0])
    if bad_input == "last_action":
        actions[-1, 12] = LOWER[12] - 0.001
    else:
        state["arm_left"][5] = LOWER[12] - 0.001 if bad_input == "initial_state" else np.nan
    with pytest.raises(RejectedBeforeAction, match="initial_state:" if bad_input != "last_action" else "row 44"):
        session.run(state, actions)
    assert session.starts == session.moves == []


@pytest.mark.parametrize("excess", [2e-4, 0.02])
@pytest.mark.parametrize("direction", [-1, 1])
def test_base_move_bounds_the_measured_hold_and_reports_it(excess, direction):
    session = BehaviorSession.__new__(BehaviorSession)
    session.lower, session.upper = LOWER, UPPER
    session.terminated = session.truncated = False
    session.odometry = R.Odometry()
    row = np.clip(EVIDENCE["samples"][0]["actions"][0], LOWER, UPPER)
    indices = [R.ACTION_SLICES[p].start for p in ("trunk", "arm_left", "arm_right")]
    bounds = UPPER if direction > 0 else LOWER
    row[indices] = bounds[indices] + direction * excess
    session.obs = {"proprio_fields": {p + "_qpos": row[R.ACTION_SLICES[p]]
                                    for p in ("trunk", "arm_left", "arm_right")}}
    session.grippers = [1, 1]
    executed = []
    def execute(rows, **kwargs):
        executed.extend(rows)
        return kwargs["extra"]
    session._execute = execute
    result = session.base_move(0, 0, 0)
    assert len(executed) == 3 and result["joint_target_clamping"]["count"] == 3
    assert result["joint_target_clamping"]["policy"] == "clamp_measured_hold"
    assert result["joint_target_clamping"]["max_excess_rad"] == pytest.approx(excess)
    assert all(np.array_equal(action[indices], bounds[indices].astype(np.float32)) for action in executed)
    assert all(R.joint_limit_violations([action], LOWER, UPPER) == [] for action in executed)
    assert np.array_equal(session.obs["proprio_fields"]["trunk_qpos"], row[R.ACTION_SLICES["trunk"]])
    # The identical values explicitly submitted as targets must still reject.
    with pytest.raises(RejectedBeforeAction):
        session.chunk_step([row])
    assert len(executed) == 3


@pytest.mark.parametrize("arm", ["left", "right"])
def test_eef_move_clamps_noncommanded_measured_holds_and_reports(monkeypatch, arm):
    import sys
    from types import ModuleType
    import torch as th
    from services.behavior.planner import ArmPlanner

    # Exercise the actual row generator with CPU tensors and no OmniGibson/GPU.
    transforms = ModuleType("omnigibson.utils.transform_utils")
    transforms.quat2mat = lambda q: th.eye(3)
    transforms.orientation_error = lambda a, b: th.zeros((1, 3))
    usd = ModuleType("omnigibson.utils.usd_utils")
    usd.ControllableObjectViewAPI = SimpleNamespace(get_link_index=lambda *_: 1)
    for name, module in (("omnigibson", ModuleType("omnigibson")),
                         ("omnigibson.utils", ModuleType("omnigibson.utils")),
                         (transforms.__name__, transforms), (usd.__name__, usd)):
        monkeypatch.setitem(sys.modules, name, module)
    session = BehaviorSession.__new__(BehaviorSession)
    session.th, session.lower, session.upper = th, LOWER, UPPER
    session.terminated = session.truncated = False
    session.grippers = [1, 1]
    row = np.zeros(23)
    other = "arm_right" if arm == "left" else "arm_left"
    indices = [R.ACTION_SLICES["trunk"].start, R.ACTION_SLICES[other].start]
    row[indices] = UPPER[indices] + 2e-4
    session.obs = {"proprio_fields": {p + "_qpos": row[R.ACTION_SLICES[p]].copy()
                                     for p in ("trunk", "arm_left", "arm_right")}}
    session.dof = lambda name: list(range(7))
    robot = SimpleNamespace(
        n_dof=7, articulation_root_path="/robot", eef_links={arm: SimpleNamespace(prim_path="/robot/eef")},
        get_relative_eef_pose=lambda **_: (th.zeros(3), th.tensor([0, 0, 0, 1.])),
        get_joint_positions=lambda: th.zeros(7), get_relative_jacobian=lambda: th.zeros((1, 6, 7)))
    executed = []
    def execute(rows, **kwargs):
        executed.extend(rows)
        return kwargs["extra"] or {}
    session._execute = execute
    planner = ArmPlanner.__new__(ArmPlanner)
    planner.session, planner.robot = session, robot
    result = planner.eef_move(arm, [0, 0, 0], [0, 0, 0, 1])
    assert result["reached"] and len(executed) == 2
    report = result["joint_target_clamping"]
    assert report["count"] == 2 and report["policy"] == "clamp_measured_hold"
    assert report["max_excess_rad"] == pytest.approx(2e-4)
    assert all(np.array_equal(action[indices], UPPER[indices].astype(np.float32)) for action in executed)
    assert session.obs["proprio_fields"][other + "_qpos"][0] == row[indices[-1]]


@pytest.mark.parametrize("value", [np.nan, np.inf])
def test_generated_holds_still_reject_nonfinite_measurements(value):
    row = np.zeros(23)
    row[3] = value
    state = initial_state(row)
    with pytest.raises(RejectedBeforeAction, match="finite"):
        R.bounded_hold_action({p + "_qpos": state[p] for p in ("trunk", "arm_left", "arm_right")},
                              [1, 1], LOWER, UPPER)


@pytest.mark.parametrize("tool", [T.STEP, T.PREVIEW_RUN])
def test_native_rejection_keeps_gateway_trusted_and_allows_corrected_retry(tool):
    error = RejectedBeforeAction("arm_left targets exceed joint limits")
    def dispatch(name, arguments):
        if error:
            raise error
        return {"step_id": 1}, []
    backend = SimpleNamespace(validate=T.validate_request, call=dispatch)
    gateway = Gateway(backend, None, contract=Contract(benchmark="behavior"), audit=lambda _: None,
                      charge=lambda *_: None, refund=lambda *_: None, budget=lambda: {})
    token = gateway.acquire()
    request = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
               "params": {"name": tool, "arguments": {"actions": [[0] * 23]}}}
    failure = gateway.handle(token, request)["error"]["data"]
    assert failure["stage"] == "native_execution" and "exceed joint limits" in failure["reason"]
    assert failure["type"] == "RejectedBeforeAction" and not failure["episode_ended"] and not gateway.terminal
    error = None
    assert "result" in gateway.handle(token, request)


def test_measurement_diagnostics_are_exposed_without_clipping_proprio(tmp_path):
    from local_tests.unit.test_behavior_backend import backend_for
    backend = backend_for(tmp_path)
    backend.start(formal=False, seed=0)
    raw = backend.sim.request("observe")
    raw["proprio_fields"]["arm_right_qpos"][3] = 0.3491041958332062
    row = R.hold_action(raw["proprio_fields"], [1, 1])
    raw["joint_limit_violations"] = R.joint_limit_violations([row], LOWER, UPPER)
    packet, _ = backend._observation_packet(raw)
    assert packet["proprio"]["arm_right_qpos"][3] == row[18]
    assert packet["joint_limit_violations"] == raw["joint_limit_violations"]


def test_preview_backend_publishes_both_correction_reports(tmp_path):
    from local_tests.unit.test_behavior_preview import STATE, preview_backend
    backend = preview_backend(tmp_path)
    run = backend.preview.run
    def corrected(state, actions, measured=False):
        return {**run(state, actions, measured), "joint_target_clamping": {"count": 1},
                "initial_state_joint_clamping": {"count": 2}, "initial_state": {**state, "trunk": [0] * 4}}
    backend.preview.run = corrected
    value, _ = backend.call(T.PREVIEW_RUN, {"actions": [[0] * 23], "initial_state": STATE})
    assert value["joint_target_clamping"]["count"] == 1
    assert value["initial_state_joint_clamping"]["count"] == 2
    assert value["start_state"]["trunk"] == [0] * 4
