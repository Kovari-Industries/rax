from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from services.robodojo.mcp_server import RoboDojoMCP, SimulatorProcess, _agent_safe
from services.robodojo.pixel_geometry import locate_pixel_selections
from services.robodojo.pose_math import pose_math
from services.robodojo.session import RoboDojoSession


def camera_parameters(translation=(0.0, 0.0, 0.0)):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = translation
    return {
        "resolution": [101, 81],
        "intrinsic_matrix": [[100, 0, 50], [0, 100, 40], [0, 0, 1]],
        "camera_to_world_usd": transform.tolist(),
    }


def test_exploration_plays_the_scheduled_layout_across_bridge_restarts(tmp_path):
    """The host schedule cycles layouts; a repeat is played as scheduled, never bumped to
    another (possibly nonexistent or held-out) layout, even by a restarted bridge."""
    class FakeSim:
        active = False
        process = None
        step_id = 0
        episode_id = None
        metadata = {"control_frequency_hz": 25.0, "physics_frequency_hz": 250.0, "cameras": [],
                    "step_limit_enforced": True, "episode_step_limit": 10, "native_reference_step_limit": 10}

        def start(self, **kwargs):
            self.start_arguments = kwargs
            self.active, self.process, self.episode_id, self.run_id = True, object(), "episode", kwargs["run_id"]
            self.task = kwargs["task"]
            return {"episode_id": "episode", "step_id": 0}

        def finish(self, reason):
            self.active = False

        def stop(self):
            self.active, self.process = False, None

        @staticmethod
        def request(operation, **kwargs):
            return {"instruction": "make_kong", "states": np.zeros(14, dtype=np.float32),
                    "eef_positions": np.zeros((2, 3), dtype=np.float32),
                    "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
                    "environment_origin_world_m": np.zeros(3, dtype=np.float32), "camera_parameters": {}}

    class Bridge(RoboDojoMCP):
        def _validate_trial_task(self, task):
            pass

        def _await_human_setup(self, **kwargs):
            return dict(self.pending_setup)

    played = []
    for revision, seed in enumerate([28, 29, 0, 1, 29]):
        bridge = Bridge.__new__(Bridge)  # A fresh bridge per episode, as the supervisor connects.
        bridge.human_setup_path = tmp_path / "operator" / "setup.json"
        bridge.consumed_setup_path = tmp_path / "operator" / "setup.json.consumed"
        bridge.demonstration_root = tmp_path / "demonstrations"
        bridge.sim = FakeSim()
        bridge._consumed_setup_revisions = set()
        bridge._human_evaluation_steps = set()
        bridge.episode_timeout_seconds = 60
        bridge.pending_setup = {
            "revision": f"rev{revision}", "task": "make_kong", "seed": seed, "eval_seed": 0, "sim_gpu": "0",
            "sim_port": 0, "startup_timeout": 180, "include_depth": True, "include_camera_parameters": True,
            "enforce_step_limit": False, "demonstration_context": "none", "max_exploration_episodes": 5}
        packet, _ = bridge.call("robodojo_request_human_setup", {})
        played.append((bridge.sim.start_arguments["seed"], bridge.sim.start_arguments["eval_seed"],
                       packet["environment_spec"]["seed"]))
    assert played == [(28, 0, 28), (29, 0, 29), (0, 0, 0), (1, 0, 1), (29, 0, 29)]
    assert not list((tmp_path / "operator").glob("*exploration-seeds*"))


def test_free_space_tool_exposes_complete_policy_trajectory():
    tool = next(
        item
        for item in RoboDojoMCP.tool_definitions()
        if item["name"] == "robodojo_free_space_move"
    )
    properties = tool["inputSchema"]["properties"]
    assert "include_trajectory" in properties
    assert "max_preview_samples" not in properties
    assert "include_actions" not in properties
    assert "max_steps" not in properties
    assert "short or long" in tool["description"]
    assert "rotations" in tool["description"]


def test_eef_tool_describes_chunk_and_actual_local_caps():
    tool = next(
        item
        for item in RoboDojoMCP.tool_definitions()
        if item["name"] == "robodojo_step_eef"
    )
    schema = tool["inputSchema"]
    assert schema["type"] == "object"
    assert schema["required"] == ["targets"]
    assert set(schema["properties"]) == {"targets"}
    assert schema["additionalProperties"] is False
    assert "oneOf" not in schema
    assert schema["properties"]["targets"]["maxItems"] == 50
    dual = schema["properties"]["targets"]["items"]
    assert dual["required"] == ["left", "right"]
    assert dual["additionalProperties"] is False
    assert dual["properties"]["left"]["additionalProperties"] is False
    assert "0.02 m" in tool["description"]
    assert "0.1 rad" in tool["description"]
    assert "+/-0.05 rad" in tool["description"]


def test_joint_tool_schema_has_one_exact_batch_shape():
    tool = next(
        item
        for item in RoboDojoMCP.tool_definitions()
        if item["name"] == "robodojo_step"
    )
    schema = tool["inputSchema"]
    assert schema["required"] == ["actions"]
    assert set(schema["properties"]) == {"actions"}
    assert schema["additionalProperties"] is False
    assert "oneOf" not in schema
    assert schema["properties"]["actions"]["minItems"] == 1
    assert schema["properties"]["actions"]["items"]["minItems"] == 14
    assert schema["properties"]["actions"]["items"]["maxItems"] == 14


def test_pose_math_tool_exposes_high_level_operations_without_episode():
    tool = next(
        item
        for item in RoboDojoMCP.tool_definitions()
        if item["name"] == "robodojo_pose_math"
    )
    operations = tool["inputSchema"]["properties"]["operation"]["enum"]
    assert operations == [
        "convert_rotation",
        "compose_pose",
        "relative_pose",
        "format_target",
        "extract_target",
    ]
    assert "does not require an episode" in tool["description"]

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.project_root = Path(__file__).resolve().parents[2]
    result, images = server.call(
        "robodojo_pose_math",
        {
            "operation": "convert_rotation",
            "rotation": {
                "representation": "euler_xyz_extrinsic_rad",
                "value": [0, 0, np.pi / 2],
            },
        },
    )
    assert images == []
    np.testing.assert_allclose(
        result["quaternion_wxyz"], [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    )
    assert result["scene_state_accessed"] is False
    assert result["physical_steps"] == 0


@pytest.mark.parametrize(
    "representation",
    [
        "quaternion_wxyz",
        "quaternion_xyzw",
        "euler_xyz_extrinsic_rad",
        "axis_angle_vector_rad",
        "rotation_matrix_row_major",
    ],
)
@pytest.mark.upstream
def test_pose_math_rotation_representations_round_trip(representation, isaac_math_root):
    root = isaac_math_root
    source = {
        "representation": "euler_xyz_extrinsic_rad",
        "value": [0.2, -0.3, 0.4],
    }
    converted = pose_math(
        root,
        {
            "operation": "convert_rotation",
            "rotation": source,
            "output_representation": representation,
        },
    )["rotation"]
    expected = pose_math(
        root,
        {"operation": "convert_rotation", "rotation": source},
    )["quaternion_wxyz"]
    recovered = pose_math(
        root,
        {"operation": "convert_rotation", "rotation": converted},
    )["quaternion_wxyz"]
    np.testing.assert_allclose(recovered, expected, atol=1e-10)


@pytest.mark.parametrize("delta_frame", ["local", "environment"])
@pytest.mark.upstream
def test_pose_math_compose_and_relative_are_inverse(delta_frame, isaac_math_root):
    root = isaac_math_root
    base = {
        "position_m": [0.2, -0.1, 0.8],
        "rotation": {
            "representation": "euler_xyz_extrinsic_rad",
            "value": [0.1, -0.2, 0.3],
        },
    }
    delta = {
        "position_m": [0.01, -0.02, 0.03],
        "rotation": {
            "representation": "axis_angle_vector_rad",
            "value": [0.0, 0.0, 0.15],
        },
    }
    composed = pose_math(
        root,
        {
            "operation": "compose_pose",
            "base_pose": base,
            "delta_pose": delta,
            "delta_frame": delta_frame,
        },
    )["pose"]
    recovered = pose_math(
        root,
        {
            "operation": "relative_pose",
            "base_pose": base,
            "target_pose": {
                "position_m": composed["position_m"],
                "rotation": {
                    "representation": "quaternion_wxyz",
                    "value": composed["quaternion_wxyz"],
                },
            },
            "delta_frame": delta_frame,
            "output_representation": "axis_angle_vector_rad",
        },
    )["delta_pose"]
    np.testing.assert_allclose(recovered["position_m"], delta["position_m"], atol=1e-10)
    np.testing.assert_allclose(recovered["rotation"]["value"], [0, 0, 0.15], atol=1e-10)
    # Reuse only input-schema fields, not the output-only quaternion shortcut.
    round_trip = pose_math(root, {
        "operation": "compose_pose", "base_pose": base, "delta_frame": delta_frame,
        "delta_pose": {key: recovered[key] for key in ("position_m", "rotation")},
    })["pose"]
    np.testing.assert_allclose(round_trip["position_m"], composed["position_m"], atol=1e-10)
    np.testing.assert_allclose(round_trip["quaternion_wxyz"], composed["quaternion_wxyz"], atol=1e-10)


@pytest.mark.upstream
def test_empty_pose_delta_is_identity(isaac_math_root):
    base = {"position_m": [0.2, 0.3, 0.4],
            "rotation": {"representation": "quaternion_wxyz", "value": [1, 0, 0, 0]}}
    result = pose_math(isaac_math_root, {
        "operation": "compose_pose", "base_pose": base, "delta_pose": {},
    })["pose"]
    assert result["position_m"] == base["position_m"]
    assert result["quaternion_wxyz"] == [1, 0, 0, 0]


@pytest.mark.parametrize("bad_target", [
    {"left": {}},
    {arm: {"position": [0, 0, 1], "quaternion_wxyz": [0, 0, 0, 0],
           "gripper_closed": False} for arm in ("left", "right")},
    {arm: {"position": [0, 0, 1], "quaternion_wxyz": [1, 0, 0, 0],
           "gripper_closed": "false"} for arm in ("left", "right")},
])
def test_entire_eef_batch_is_validated_before_any_native_request(bad_target):
    class NoNativeCalls:
        active = True

        def request(self, *args, **kwargs):
            pytest.fail("Validation must finish before any native RPC")

    valid = {arm: {"position": [0, 0, 1], "quaternion_wxyz": [1, 0, 0, 0],
                   "gripper_closed": False} for arm in ("left", "right")}
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = NoNativeCalls()
    with pytest.raises(ValueError):
        server.call("robodojo_step_eef", {"targets": [valid, bad_target]})


@pytest.mark.parametrize('count,remaining', [(12, 100), (50, 100), (50, 3)])
def test_larger_eef_batches_stop_at_native_horizon(count, remaining):
    class Sim:
        active = True
        step_id = 0
        episode_id = 'test'
        metadata = {}

        def request(self, op, **args):
            if op == 'eef_joint_target':
                return {'action': [0]*14}
            assert op == 'chunk_step'
            next_step = self.step_id + 1
            return {'step_id': next_step, 'frames': [{'step_id': next_step}],
                    'steps': [{'truncated': next_step >= remaining}]}

    valid = {arm: {'position': [0, 0, 1], 'quaternion_wxyz': [1, 0, 0, 0],
                   'gripper_closed': False} for arm in ('left', 'right')}
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = Sim()
    server._observation_sequence_packet = lambda frames, transition: (transition, [])
    value, _ = server.call('robodojo_step_eef', {'targets': [valid]*count})
    assert value['step_id'] == min(count, remaining)
    assert value['frame_count'] == min(count, remaining)


@pytest.mark.parametrize("override", [
    {"arm": "both"}, {"preview_only": "true"},
    {"target": {"position": [0, 0, 1], "quaternion_wxyz": [0, 0, 0, 0]}},
    {"target": {"position": [0, 0, 1], "quaternion_wxyz": [1, 0, 0, 0],
                "gripper_opening": None}},
])
def test_free_space_validation_does_not_poison_native_connection(override):
    class NoNativeCalls:
        active = True

        def request(self, *args, **kwargs):
            pytest.fail("Invalid motion input reached native RPC")

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = NoNativeCalls()
    arguments = {"arm": "left", "target": {"position": [0, 0, 1],
                                           "quaternion_wxyz": [1, 0, 0, 0]}}
    with pytest.raises(ValueError):
        server.call("robodojo_free_space_move", {**arguments, **override})


@pytest.mark.upstream
def test_pose_math_formats_exact_eef_and_curobo_targets(isaac_math_root):
    root = isaac_math_root
    pose = {
        "position_m": [0.1, 0.2, 0.9],
        "rotation": {"representation": "quaternion_xyzw", "value": [0, 0, 0, 2]},
    }
    step = pose_math(
        root,
        {
            "operation": "format_target",
            "pose": pose,
            "target_kind": "step_eef",
            "gripper_closed": False,
        },
    )["target"]
    assert step == {
        "position": [0.1, 0.2, 0.9],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "gripper_closed": False,
    }
    free_space = pose_math(
        root,
        {
            "operation": "format_target",
            "pose": pose,
            "target_kind": "free_space_move",
            "gripper_opening": 0.4,
        },
    )["target"]
    assert free_space == {
        "position": [0.1, 0.2, 0.9],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "gripper_opening": 0.4,
    }


def test_mcp_filters_native_task_evaluation_recursively():
    result = _agent_safe(
        {
            "success": True,
            "reward": 1.0,
            "native_reward": 1.0,
            "native_score": 0.75,
            "native_score_percent": 75.0,
            "operator_decision_source": "automatic_native_success",
            "terminated": True,
            "truncated": False,
            "status": "Success",
            "steps": [
                {
                    "success": True,
                    "reward_state": {"hidden": True},
                    "terminated": True,
                    "measured_state": [0.0],
                }
            ],
        }
    )
    assert result["status"] == "Success"  # planner/tool status is not task success
    assert result["episode_ended"] is True
    assert result["task_complete"] is True
    assert result["steps"][0] == {
        "measured_state": [0.0],
        "episode_ended": True,
        "task_complete": True,
    }

    def keys(value):
        if isinstance(value, dict):
            return set(value).union(*(keys(item) for item in value.values()))
        if isinstance(value, list):
            return set().union(*(keys(item) for item in value))
        return set()

    assert not {
        "success",
        "reward",
        "reward_state",
        "native_reward",
        "native_score",
        "native_score_percent",
        "operator_decision_source",
        "terminated",
        "truncated",
    }.intersection(keys(result))


@pytest.mark.parametrize("success", [True, False])
@pytest.mark.parametrize("truncated", [True, False])
def test_terminal_native_outcome_is_disclosed(success, truncated):
    result = _agent_safe({"success": success, "terminated": not truncated,
                          "truncated": truncated, "reward": 1.0})
    assert result == {"episode_ended": True, "task_complete": success}


def test_intermediate_or_unknown_outcome_is_not_invented():
    assert _agent_safe({"success": True, "terminated": False}) == {
        "episode_ended": False,
    }
    assert _agent_safe({"terminated": True}) == {"episode_ended": True}
    assert _agent_safe({"success": True, "status": "Success"}) == {
        "status": "Success",
    }


def test_curobo_executes_by_default_and_previews_only_when_requested():
    tool = next(
        item
        for item in RoboDojoMCP.tool_definitions()
        if item["name"] == "robodojo_free_space_move"
    )
    properties = tool["inputSchema"]["properties"]
    assert "execute" not in properties
    assert properties["preview_only"]["default"] is False

    class FakeSim:
        active = True

        def request(self, operation, **kwargs):
            assert operation == "free_space_move"
            return kwargs

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = FakeSim()
    server._motion_result_packet = lambda result: (result, [])
    target = {
        "position": [0.0, 0.0, 1.0],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    direct, _ = server.call(
        "robodojo_free_space_move", {"arm": "left", "target": target}
    )
    assert "execute" not in direct
    assert direct["preview_only"] is False

    preview, _ = server.call(
        "robodojo_free_space_move",
        {"arm": "left", "target": target, "preview_only": True},
    )
    assert "execute" not in preview
    assert preview["preview_only"] is True
    for obsolete in ("execute", "max_preview_samples"):
        with pytest.raises(ValueError, match="Unknown free_space_move"):
            server.call("robodojo_free_space_move",
                        {"arm": "left", "target": target, obsolete: True})


def test_human_setup_spec_is_single_use_and_controls_environment(tmp_path):
    setup = {
        "revision": "operator-test-1",
        "task": "build_tower",
        "seed": 3,
        "eval_seed": 4,
        "sim_gpu": "1",
        "sim_port": 19113,
        "startup_timeout": 900,
        "include_depth": True,
        "include_camera_parameters": False,
        "enforce_step_limit": False,
        "demonstration_context": "none",
    }

    class FakeSim:
        active = False
        process = None
        step_id = 0
        episode_id = None
        task = None
        run_id = None
        metadata = None

        def __init__(self):
            self.start_count = 0
            self.finish_reasons = []

        def start(self, **kwargs):
            self.start_count += 1
            self.start_arguments = kwargs
            self.active = True
            self.process = object()
            self.step_id = 0
            self.episode_id = f"episode-{self.start_count}"
            self.task = kwargs["task"]
            self.run_id = kwargs["run_id"]
            self.metadata = {
                "control_frequency_hz": 25.0,
                "physics_frequency_hz": 250.0,
                "cameras": ["cam_high"],
                "step_limit_enforced": kwargs["enforce_step_limit"],
                "episode_step_limit": 1050,
                "native_reference_step_limit": 1050,
            }
            return {"episode_id": self.episode_id, "step_id": 0}

        def finish(self, reason):
            self.finish_reasons.append(reason)
            self.active = False
            return {"reason": reason}

        def stop(self):
            self.active = False
            self.process = None

        @staticmethod
        def request(operation, **kwargs):
            assert operation == "teacher_observation"
            assert not kwargs
            return {
                "instruction": "build_tower",
                "states": np.zeros(14, dtype=np.float32),
                "eef_positions": np.zeros((2, 3), dtype=np.float32),
                "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
                "environment_origin_world_m": np.zeros(3, dtype=np.float32),
                "camera_parameters": {},
            }

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.human_setup_path = tmp_path / "operator_setup.json"
    server.consumed_setup_path = tmp_path / "operator_setup.json.consumed"
    server.sim = FakeSim()
    server._consumed_setup_revisions = set()
    server._human_evaluation_steps = set()
    server._human_evaluation_count = 0
    setup["max_exploration_episodes"] = 2
    server.pending_setup = setup

    packet, images = server.call("robodojo_request_human_setup", {})
    assert images == []
    assert packet["environment_spec"] == {
        "operator_selected": True,
        "revision": "operator-test-1",
        "task": "build_tower",
        "seed": 3,
        "eval_seed": 4,
        "episode_mode": "exploration",
        "depth_enabled": True,
        "camera_parameters_enabled": False,
        "step_limit_enforced": True,
        "native_step_limit_enforced": True,
        "episode_step_limit": 1050,
        "native_reference_step_limit": 1050,
        "episode_timeout_seconds": 6300,
        "exploration_episodes_started": 1,
        "max_exploration_episodes": 2,
        "exploration_episodes_remaining": 1,
        "control_frequency_hz": 25.0,
        "physics_frequency_hz": 250.0,
        "cameras": ["cam_high"],
        "step_accounting": {
            "limit_unit": "executed_25hz_environment_control_block",
            "joint_action_row_steps": 1,
            "eef_waypoint_steps": 1,
            "joint_target_frequency_hz": 25.0,
            "curobo_step_formula": "one step per executed 25Hz joint target row",
            "read_only_or_planning_steps": 0,
            "robot_preview_counts_toward_robodojo_steps": False,
        },
    }
    assert packet["performance_cost"] == {
        "environment_setup_count": 1,
        "environment_reset_count": 0,
        "exploration_episodes_started": 1,
        "max_exploration_episodes": 2,
        "current_episode_interaction_steps": 0,
        "total_interaction_steps": 0,
        "human_evaluation_request_count": 0,
    }
    assert server.sim.start_arguments["enforce_step_limit"] is False
    assert server.sim.start_arguments["enable_depth"] is True
    assert server.sim.start_arguments["enable_camera_parameters"] is False
    assert server._consumed_setup_revisions == {"operator-test-1"}
    assert server._load_consumed_setup_revisions() == {"operator-test-1"}

    server.sim.step_id = 11
    with pytest.raises(RuntimeError, match="already consumed"):
        server.call("robodojo_request_human_setup", {})
    setup["revision"] = "operator-test-2"
    setup["seed"] = 5
    packet, images = server.call("robodojo_request_human_setup", {})
    assert packet["environment_spec"]["exploration_episodes_remaining"] == 0
    assert packet["environment_spec"]["episode_step_limit"] == 1050
    with pytest.raises(RuntimeError, match="Exploration episode budget exhausted"):
        server.call("robodojo_request_human_setup", {})
    server.sim.step_id = 7
    setup["revision"] = "operator-test-3"
    setup["enforce_step_limit"] = True
    with pytest.raises(RuntimeError, match="Exploration episode budget exhausted"):
        server.call("robodojo_request_human_setup", {})
    packet, images = server.call("robodojo_request_formal_episode", {})
    assert images == []
    assert server.sim.finish_reasons == ["human_approved_reset", "human_approved_formal_episode"]
    assert packet["environment_spec"]["episode_mode"] == "formal"
    assert packet["environment_spec"]["step_limit_enforced"] is True
    assert packet["environment_spec"]["episode_step_limit"] == 1050
    assert packet["performance_cost"]["environment_setup_count"] == 3
    assert packet["performance_cost"]["environment_reset_count"] == 2
    assert packet["performance_cost"]["total_interaction_steps"] == 18
    assert packet["performance_cost"]["current_episode_interaction_steps"] == 0
    assert server._load_consumed_setup_revisions() == {
        "operator-test-1",
        "operator-test-2",
        "operator-test-3",
    }
    assert server.consumed_setup_path.stat().st_mode & 0o777 == 0o600

    with pytest.raises(RuntimeError, match="formal episode is final"):
        server.call("robodojo_request_human_setup", {})
    with pytest.raises(RuntimeError, match="second formal episode"):
        server.call("robodojo_request_formal_episode", {})


def test_formal_episode_rejects_a_setup_without_the_native_step_limit():
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.pending_setup = {"revision": "unbounded", "task": "make_kong", "seed": 0,
                            "enforce_step_limit": False, "max_exploration_episodes": 1}
    server._consumed_setup_revisions = set()
    server.sim = object()

    with pytest.raises(RuntimeError, match="formal episode requires the native step limit"):
        server.call("robodojo_request_formal_episode", {})


@pytest.mark.parametrize("native_success", [True, False])
def test_evaluation_reads_the_simulators_record_of_this_step(tmp_path, native_success):
    class FakeSim:
        active = True
        episode_id = "episode"
        step_id = 7
        task = "make_kong"
        output_dir = tmp_path
        metadata = {"control_frequency_hz": 25.0}

        @staticmethod
        def request(operation, **kwargs):
            assert operation == "teacher_observation"
            assert not kwargs
            return {
                "instruction": "make_kong",
                "states": np.zeros(14, dtype=np.float32),
                "eef_positions": np.zeros((2, 3), dtype=np.float32),
                "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
                "environment_origin_world_m": np.zeros(3, dtype=np.float32),
                "camera_parameters": {},
            }

    state = tmp_path / "sim" / "operator_state.json"
    state.parent.mkdir()
    state.write_text(json.dumps({"episode_id": "episode", "task": "make_kong", "step_id": 7,
                                 "reward": {"native_success": native_success}}))
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = FakeSim()
    server._environment_spec = {"episode_mode": "exploration"}
    server._human_evaluation_steps = set()
    server._human_evaluation_count = 0

    result, images = server.call(
        "robodojo_request_human_evaluation",
        {"agent_assessment": "The visible structure appears complete.", "confidence": 0.8},
    )
    assert images == []
    assert result["total_interaction_steps"] == 7
    assert result["performance_cost"]["human_evaluation_request_count"] == 1
    receipt = result["human_evaluation_request"]
    assert receipt["status"] == "completed"
    assert receipt["task_complete"] is native_success
    assert (receipt["episode_id"], receipt["step_id"], receipt["task"]) == ("episode", 7, "make_kong")
    assert receipt["episode_mode"] == "exploration"
    assert receipt["confidence"] == 0.8
    assert receipt["native_task_evaluation_disclosed"] is False
    assert ("does not count as formal success" in result.get("next_action", "")) is native_success
    log = [json.loads(line) for line in (tmp_path / "human_evaluation_requests.jsonl").read_text().splitlines()]
    assert [entry["status"] for entry in log] == ["pending", "completed"]
    assert log[1]["task_complete"] is native_success
    with pytest.raises(ValueError, match="already requested"):
        server.call("robodojo_request_human_evaluation", {"agent_assessment": "Duplicate request."})
    # A record from another step is never accepted as this step's result.
    server.sim.step_id = 8
    with pytest.raises(RuntimeError, match="the simulator last recorded step_id 7, but this episode's is 8"):
        server.call("robodojo_request_human_evaluation", {"agent_assessment": "Next step."})


@pytest.mark.parametrize("decision", [
    {"status": "completed", "task_complete": False},
    {"status": "completed", "task_complete": True},
    None,
])
def test_formal_evaluation_closes_before_decision_and_prevents_retry(tmp_path, decision):
    class Sim:
        active = True
        episode_id = "formal"
        step_id = 7
        task = "make_kong"
        output_dir = tmp_path
        stopped = False

        def request(self, op):
            assert op == "teacher_observation"
            return {}

        def finish(self, reason):
            assert reason == "formal_evaluation_submitted"

        def stop(self):
            self.stopped = True
            self.active = False

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = Sim()
    server._environment_spec = {"episode_mode": "formal"}
    server._human_evaluation_steps = set()
    server._human_evaluation_count = 0
    server._observation_packet = lambda observation: ({}, [])

    def await_decision(record):
        assert server.sim.stopped
        assert server._formal_evaluation_submitted
        if decision is None:
            raise RuntimeError("Native evaluation unavailable")
        return decision

    server._native_evaluation = await_decision
    if decision is None:
        with pytest.raises(RuntimeError, match="Native evaluation unavailable"):
            server.call("robodojo_request_human_evaluation", {"agent_assessment": "Ready"})
    else:
        packet, _ = server.call("robodojo_request_human_evaluation", {"agent_assessment": "Ready"})
        assert packet["environment_closed"] and packet["formal_attempt_ended"]
    for name in ("robodojo_step", "robodojo_request_human_evaluation",
                 "robodojo_request_human_setup", "robodojo_request_formal_episode"):
        with pytest.raises(RuntimeError, match="formal attempt ended"):
            server.call(name, {})
    assert server._performance_cost()["total_interaction_steps"] == 7


def test_pixel_depth_unprojection_uses_cv_to_usd_camera_axes():
    observation = {
        "camera_parameters": {"cam_high": camera_parameters()},
        "cam_high_depth_m": np.full((81, 101), 2.0, dtype=np.float32),
        "environment_origin_world_m": [0, 0, 0],
    }
    result = locate_pixel_selections(
        observation,
        [{"camera": "cam_high", "pixel": [60, 50]}],
        coordinate_space="image_pixels",
        method="auto",
        depth_window_radius=0,
    )
    assert result["status"] == "Success"
    assert result["method"] == "depth_unprojection"
    np.testing.assert_allclose(
        result["position_environment_m"], [0.2, -0.2, -2.0], atol=1e-8
    )
    assert not result["privilege_boundary"]["uses_task_object_pose"]


def test_pixel_triangulation_recovers_intersection_without_depth():
    observation = {
        "camera_parameters": {
            "cam_high": camera_parameters((-0.5, 0, 0)),
            "cam_left_wrist": camera_parameters((0.5, 0, 0)),
        },
        "environment_origin_world_m": [0, 0, 0],
    }
    result = locate_pixel_selections(
        observation,
        [
            {"camera": "cam_high", "pixel": [75, 40]},
            {"camera": "cam_left_wrist", "pixel": [25, 40]},
        ],
        method="auto",
    )
    assert result["status"] == "Success"
    assert result["method"] == "multi_view_triangulation"
    np.testing.assert_allclose(result["position_environment_m"], [0, 0, -2], atol=1e-8)
    assert result["quality"]["max_point_to_ray_residual_m"] < 1e-8


def test_pixel_tool_rejects_stale_observation_step_before_geometry():
    class FakeSim:
        active = True
        step_id = 4

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = FakeSim()
    with pytest.raises(ValueError, match="Stale pixel selection"):
        server.call(
            "robodojo_pixel_to_position",
            {
                "observation_step_id": 3,
                "selections": [{"camera": "cam_high", "pixel": [0, 0]}],
            },
        )


def test_motion_frame_sequence_saves_all_frames_and_returns_only_final(tmp_path):
    class FakeSim:
        def __init__(self):
            self.episode_id = "episode"
            self.step_id = 12
            self.task = "task"
            self.run_id = "test_run"
            self.metadata = {
                "control_frequency_hz": 25.0,
                "step_limit_enforced": False,
            }

    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = FakeSim()
    server.frame_root = tmp_path / "frames"
    rgb = np.zeros((4, 5, 3), dtype=np.uint8)
    depth = np.ones((4, 5), dtype=np.float32)
    frame = {
        "cam_high": rgb,
        "cam_left_wrist": rgb,
        "cam_right_wrist": rgb,
        "cam_high_depth_m": depth,
        "cam_left_wrist_depth_m": depth,
        "cam_right_wrist_depth_m": depth,
        "states": np.zeros(14, dtype=np.float32),
        "eef_positions": np.zeros((2, 3), dtype=np.float32),
        "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
        "camera_parameters": {},
        "instruction": "task",
    }
    transition = {"steps": [{"step_id": 11}, {"step_id": 12}]}
    packet, content = server._observation_sequence_packet(
        [frame, frame], transition=transition
    )
    assert packet["total_interaction_steps"] == 12
    assert "remaining_steps" not in packet
    assert packet["frame_sequence"]["frame_count"] == 2
    assert packet["frame_sequence"]["returned_images"] == "final_frame_only"
    assert packet["frame_sequence"]["frame_index_range"] == [0, 1]
    assert packet["frame_sequence"]["frame_path_pattern"].endswith(
        "/frame_{frame_index:06d}_{camera}_{kind}.png"
    )
    assert {item["camera"] for item in packet["frame_sequence"]["streams"]} == {
        "cam_high",
        "cam_left_wrist",
        "cam_right_wrist",
    }
    assert packet["frame_sequence"]["final_frame"][
        "time_from_execution_start_s"
    ] == pytest.approx(0.08)
    assert "frames" not in packet["frame_sequence"]
    assert len(content) == 6
    assert [item["content_index"] for item in packet["attachments"]] == list(
        range(1, 7)
    )
    manifest_path = (
        tmp_path
        / "frames"
        / "robodojo"
        / "test_run"
        / packet["frame_sequence"]["sequence_id"]
        / "manifest.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["frame_count"] == 2
    assert [item["time_from_execution_start_s"] for item in manifest["frames"]] == (
        pytest.approx([0.04, 0.08])
    )
    files = [file for frame in manifest["frames"] for file in frame["files"]]
    assert len(files) == 12
    assert all(
        (manifest_path.parent / Path(item["path"]).name).is_file() for item in files
    )


def test_terminal_failure_before_native_limit_is_not_truncation():
    class FakeEnv:
        def __init__(self):
            self.end_flag = [True]
            self.success = [False]
            self.enforce_step_limit = True

    session = RoboDojoSession.__new__(RoboDojoSession)
    session.env = FakeEnv()
    session.step_id = 99
    session.native_step_limit = 100
    session.episode_step_limit = 100

    assert session._update_terminal_state() is True
    assert session.truncated is False
    assert session.terminated is True


@pytest.mark.parametrize("formal", [False, True])
def test_both_episode_modes_enable_original_native_horizon(tmp_path, formal):
    from types import SimpleNamespace

    env = SimpleNamespace(
        step_lim=600, dt=0.004,
        obs_manager=SimpleNamespace(collect_interval=10, collect_freq=25),
    )
    session = RoboDojoSession(env, tmp_path, "make_kong", enforce_step_limit=formal)
    assert env.enforce_step_limit is True
    assert session.episode_step_limit == 600
    assert session.metadata["native_step_limit_enforced"] is True
    assert session.episode_mode == ("formal" if formal else "exploration")


def test_exploration_ends_at_native_limit_and_rejects_more_actions():
    class FakeEnv:
        end_flag = [False]
        success = [False]
        enforce_step_limit = True

    session = RoboDojoSession.__new__(RoboDojoSession)
    session.env = FakeEnv()
    session.native_step_limit = 100
    session.episode_step_limit = 100
    session.step_id = 99
    session.finished = False
    session.terminated = False
    session.truncated = False

    assert session._update_terminal_state() is False
    session.step_id = 100
    assert session._update_terminal_state() is True
    assert session.truncated is True
    assert session.terminated is False
    with pytest.raises(Exception, match="Episode finished"):
        session.chunk_step(np.zeros((1, 14), dtype=np.float32))


@pytest.mark.upstream
def test_native_step_limit_checks_are_gated_by_enforcement_flag():
    path = (
        Path(__file__).resolve().parents[2]
        / "RoboDojo"
        / "src"
        / "eval_client"
        / "eval_env.py"
    )
    if not path.is_file():
        pytest.skip("Bootstrap the pinned RoboDojo checkout to check native horizon guards")
    source = path.read_text(encoding="utf-8")
    assert source.count("self.take_action_cnt[env_idx] >= self.step_lim") == 1
    assert "self.take_action_cnt[env_idx] == self.step_lim" not in source
    assert source.count("self._step_limit_reached(env_idx)") >= 3
    assert "def take_control_sequence(" not in source


def test_main_mcp_and_service_do_not_import_gpt_as_policy():
    root = Path(__file__).resolve().parents[2]
    files = [root / "services" / "robodojo" / "mcp_server.py"]
    files.extend((root / "services" / "robodojo").glob("*.py"))
    combined = "\n".join(path.read_text(encoding="utf-8") for path in files)
    assert "GPT-as-Policy" not in combined
    assert "hybrid_rollout" not in combined


def test_status_reports_the_native_episode_end_without_intermediate_success(tmp_path):
    class RPC:
        def __init__(self):
            self.replies = [
                # Intermediate native success while the episode continues stays hidden.
                {"step_id": 699, "steps": [{"step_id": 699, "terminated": False, "truncated": False,
                                            "success": True}]},
                {"step_id": 700, "steps": [{"step_id": 700, "terminated": False, "truncated": True,
                                            "success": False}]},
            ]

        def request(self, op, **kwargs):
            assert op == "chunk_step"
            return self.replies.pop(0)

    sim = SimulatorProcess(project_root=tmp_path, python="python", output_root=tmp_path / "results")
    sim.rpc, sim.process, sim.episode_id, sim.step_id, sim.task, sim.metadata = (
        RPC(), object(), "episode-1", 698, "make_kong", {})
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.sim = sim

    def status():
        value, _ = server._call("robodojo_status", {})
        return _agent_safe(value)

    assert status()["episode_ended"] is False and "task_complete" not in status()
    sim.request("chunk_step", actions=[])
    assert status()["episode_ended"] is False and "task_complete" not in status()
    sim.request("chunk_step", actions=[])
    after = status()
    assert (after["episode_ended"], after["task_complete"]) == (True, False)
    assert not {"success", "terminated", "truncated", "native_success"} & set(after)
