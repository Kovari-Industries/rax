"""Stateless robot-only preview operations for the separate Isaac service."""

from __future__ import annotations

import numpy as np

from services.robodojo.trajectory import JOINT_TARGET_CONTRACT, MAX_TRAJECTORY_ACTIONS


def validate_preview_inputs(*, robot_model, camera_model, initial_state, actions):
    """Reject malformed requests in the MCP bridge before starting/using Isaac."""
    if robot_model != "dual_arx_x5":
        raise ValueError("robot_model must be dual_arx_x5")
    if camera_model != "robodojo_gemini_wrist_v1":
        raise ValueError("camera_model must be robodojo_gemini_wrist_v1")
    initial_state = np.asarray(initial_state, dtype=np.float32)
    if initial_state.shape != (14,) or not np.isfinite(initial_state).all():
        raise ValueError("initial_state must be a finite 14-D vector")
    if np.any((initial_state[[6, 13]] < 0) | (initial_state[[6, 13]] > 1)):
        raise ValueError("initial_state gripper openings must be in [0, 1]")
    actions = np.asarray(actions, dtype=np.float32)
    if (
        actions.ndim != 2 or actions.shape[1] != 14
        or not 1 <= len(actions) <= MAX_TRAJECTORY_ACTIONS or not np.isfinite(actions).all()
    ):
        raise ValueError(f"actions must be 1..{MAX_TRAJECTORY_ACTIONS} finite 14-D rows")
    if np.any((actions[:, [6, 13]] < 0) | (actions[:, [6, 13]] > 1)):
        raise ValueError("action gripper openings must be in [0, 1]")
    return initial_state, actions


class RobotPreviewSession:
    def __init__(self, environment, *, robot_model, camera_model):
        self.environment = environment
        self.terminated = self.truncated = False
        self.metadata = {
            "service": "robot_model_preview",
            "robot_model": robot_model,
            "camera_model": camera_model,
            "physics_dt": environment.dt,
            "physics_frequency_hz": 1.0 / environment.dt,
            "observation_dt": environment.dt * environment.ticks_per_observation,
            "observation_frequency_hz": 1.0
            / (environment.dt * environment.ticks_per_observation),
            "action_dim": 14,
            "state_dim": 14,
            "cameras": ["cam_high", "cam_left_wrist", "cam_right_wrist"],
            "scene_geometry": [],
            "task_environment_loaded": False,
            "task_objects_loaded": False,
            "reward_or_success_available": False,
            "privileged_state_available": False,
            "prediction_scope": "robot articulation dynamics, kinematics, and robot-only camera rendering",
        }

    def preview(
        self,
        *,
        robot_model,
        camera_model,
        initial_state,
        actions,
    ):
        initial_state, actions = validate_preview_inputs(
            robot_model=robot_model, camera_model=camera_model,
            initial_state=initial_state, actions=actions,
        )
        self.environment.set_state(initial_state)
        initial_observation = self.environment.observation()
        frames = []
        for action in actions:
            self.environment.step_inner_controls(self.environment.action_controls(action))
            frames.append(self.environment.observation())
        execution = {
            "action_count": len(actions),
            "actions": actions.tolist(),
            "joint_target_contract": JOINT_TARGET_CONTRACT,
            "action_interpolation": "native RoboDojo measured-state interpolation, then hold",
        }
        execution.update(
            observation_frequency_hz=self.metadata["observation_frequency_hz"],
            frame_count=len(frames),
            frames=frames,
        )
        return {
            "status": "Success",
            "initial_observation": initial_observation,
            "execution": execution,
            "contract": {
                "inputs_used": [
                    "whitelisted robot model",
                    "whitelisted camera model",
                    "initial robot state",
                    "requested action/control sequence",
                ],
                "task_environment_loaded": False,
                "task_objects_loaded": False,
                "object_positions_available": False,
                "depth_world_reconstruction": False,
                "outputs_mean": "robot-only counterfactual, not task-scene prediction",
            },
        }

    def dispatch(self, op, args):
        if op == "metadata":
            return self.metadata
        if op == "preview":
            return self.preview(**args)
        raise ValueError("Unsupported robot-preview operation: " + op)

    def _write_summary(self, reason):
        del reason

    def close(self):
        self.environment.close()
