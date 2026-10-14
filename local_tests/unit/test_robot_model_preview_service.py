from __future__ import annotations

import ast
import json
from pathlib import Path

import numpy as np
import pytest

from services.robot_preview.mcp_server import RobotPreviewMCP
from services.robot_preview.session import RobotPreviewSession


class FakeEnvironment:
    dt = 0.004
    ticks_per_observation = 10

    def __init__(self):
        self.state = None
        self.windows = []

    def set_state(self, state):
        self.state = np.asarray(state, dtype=np.float32).copy()

    def observation(self):
        return {
            "states": self.state.copy(),
            "eef_positions": np.zeros((2, 3), dtype=np.float32),
            "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
            "camera_parameters": {},
        }

    @staticmethod
    def action_controls(action):
        return [np.asarray(action, dtype=np.float32).copy() for _ in range(10)]

    def step_inner_controls(self, controls):
        assert len(controls) == 10
        self.windows.append(controls)
        if np.asarray(controls[-1]).shape == (14,):
            self.state = np.asarray(controls[-1], dtype=np.float32).copy()

    def close(self):
        pass


def test_robot_preview_executes_joint_targets_and_returns_25hz_frames():
    environment = FakeEnvironment()
    session = RobotPreviewSession(environment, robot_model="dual_arx_x5",
                                  camera_model="robodojo_gemini_wrist_v1")
    actions = np.zeros((21, 14), dtype=np.float32)
    actions[:, 0] = np.linspace(0, 0.2, 21)
    result = session.preview(robot_model="dual_arx_x5",
                             camera_model="robodojo_gemini_wrist_v1",
                             initial_state=np.zeros(14), actions=actions)
    execution = result["execution"]
    assert execution["action_count"] == execution["frame_count"] == 21
    assert execution["observation_frequency_hz"] == 25
    np.testing.assert_array_equal(execution["actions"], actions)
    np.testing.assert_array_equal(execution["frames"][-1]["states"], actions[-1])
    assert not result["contract"]["task_environment_loaded"]


def test_robot_preview_schema_only_accepts_joint_targets():
    schema = next(t["inputSchema"] for t in RobotPreviewMCP.tools()
                  if t["name"] == "robot_preview_run")
    assert set(schema["required"]) == set(schema["properties"]) == {
        "robot_model", "camera_model", "initial_state", "actions"}
    assert schema["additionalProperties"] is False
    assert schema["properties"]["actions"]["items"]["minItems"] == 14
    assert schema["properties"]["actions"]["maxItems"] == 50
    assert RobotPreviewMCP.description()["robot_preview_run_input_schema"] == schema


@pytest.mark.parametrize("extra", ["input_type", "eef_targets", "dense_controls"])
def test_preview_rejects_obsolete_inputs_before_native_start(extra):
    server = RobotPreviewMCP.__new__(RobotPreviewMCP)
    base = {"robot_model": "dual_arx_x5", "camera_model": "robodojo_gemini_wrist_v1",
            "initial_state": [0.0] * 14, "actions": [[0.0] * 14]}
    with pytest.raises(ValueError, match="Unknown"):
        server.preview({**base, extra: None})
    del base["actions"]
    with pytest.raises(ValueError, match="Missing"):
        server.preview(base)


@pytest.mark.parametrize("actions", [[], [[0.0] * 13], [[float("nan")] * 14],
                                    [[2.0] * 14]])
def test_preview_validates_actions_before_changing_state(actions):
    environment = FakeEnvironment()
    session = RobotPreviewSession(environment, robot_model="dual_arx_x5",
                                  camera_model="robodojo_gemini_wrist_v1")
    with pytest.raises(ValueError):
        session.preview(robot_model="dual_arx_x5", camera_model="robodojo_gemini_wrist_v1",
                        initial_state=[0.0] * 14, actions=actions)
    assert environment.state is None


@pytest.mark.parametrize("override", [
    {"robot_model": "other"}, {"camera_model": "other"},
    {"actions": [[0.0] * 14] * 51}, {"actions": []},
    {"initial_state": [float("nan")] * 14},
])
def test_preview_bridge_rejects_invalid_request_before_rpc(override):
    server = RobotPreviewMCP.__new__(RobotPreviewMCP)
    # No native attribute: reaching native.request would fail this test.
    request = {"robot_model": "dual_arx_x5", "camera_model": "robodojo_gemini_wrist_v1",
               "initial_state": [0.0] * 14, "actions": [[0.0] * 14]}
    with pytest.raises(ValueError):
        server.preview({**request, **override})


def test_bounded_frames_fit_rpc_even_without_compression():
    from services.simulator.protocol import MAX_WIRE_BYTES, MAX_DECOMPRESSED_BYTES
    from services.robodojo.trajectory import MAX_TRAJECTORY_ACTIONS
    # Three reviewed 640x480 cameras: RGB uint8 + depth float32, plus initial frame.
    frame_bytes = 3 * 640 * 480 * (3 + 4)
    payload_bytes = (MAX_TRAJECTORY_ACTIONS + 1) * frame_bytes
    assert payload_bytes + 1024 * 1024 < min(MAX_WIRE_BYTES, MAX_DECOMPRESSED_BYTES)


def test_robot_preview_service_has_no_task_scene_imports():
    root = Path(__file__).resolve().parents[2] / "services" / "robot_preview"
    forbidden = (
        "task",
        "env.scene_manager",
        "env.seed_manager",
        "src.eval_client",
    )
    imports = []
    for path in root.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
    for imported in imports:
        assert not any(
            imported == name or imported.startswith(name + ".") for name in forbidden
        )


def test_preview_mcp_saves_sequence_and_attaches_only_final_frame(tmp_path):
    rgb = np.zeros((4, 5, 3), dtype=np.uint8)
    depth = np.ones((4, 5), dtype=np.float32)

    def observation(value):
        return {
            "cam_high": rgb + value,
            "cam_left_wrist": rgb + value,
            "cam_right_wrist": rgb + value,
            "cam_high_depth_m": depth,
            "cam_left_wrist_depth_m": depth,
            "cam_right_wrist_depth_m": depth,
            "states": np.full(14, value, dtype=np.float32),
            "eef_positions": np.zeros((2, 3), dtype=np.float32),
            "eef_quaternions_wxyz": np.zeros((2, 4), dtype=np.float32),
            "camera_parameters": {},
        }

    class FakeNative:
        def __init__(self):
            self.metadata = {"object_free": True}

        @staticmethod
        def request(**arguments):
            assert arguments["robot_model"] == "dual_arx_x5"
            return {
                "status": "Success",
                "initial_observation": observation(0),
                "execution": {
                    "observation_frequency_hz": 25.0,
                    "frames": [observation(1), observation(2)],
                },
                "contract": {"task_environment_loaded": False},
            }

    server = RobotPreviewMCP.__new__(RobotPreviewMCP)
    server.native = FakeNative()
    server.frame_root = tmp_path / "frames"
    packet, content = server.preview(
        {
            "robot_model": "dual_arx_x5",
            "camera_model": "robodojo_gemini_wrist_v1",
            "initial_state": [0.0] * 14,
            "actions": [[0.0] * 14] * 2,
        }
    )
    assert len(content) == 6
    assert packet["frame_sequence"]["returned_images"] == "final_frame_only"
    assert packet["frame_sequence"]["frame_index_range"] == [0, 1]
    assert packet["frame_sequence"]["frame_path_pattern"].endswith(
        "/frame_{frame_index:06d}_{camera}_{kind}.png"
    )
    assert packet["frame_sequence"]["initial_frame_path_pattern"].endswith(
        "/initial_{camera}_{kind}.png"
    )
    assert "frames" not in packet["frame_sequence"]
    assert packet["final_observation"]["states"] == [2.0] * 14
    manifest_path = (
        tmp_path
        / "frames"
        / "robot_preview"
        / packet["frame_sequence"]["sequence_id"]
        / "manifest.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["frame_count"] == 2
    assert len(manifest["initial_observation"]["files"]) == 6
    assert sum(len(item["files"]) for item in manifest["frames"]) == 12
