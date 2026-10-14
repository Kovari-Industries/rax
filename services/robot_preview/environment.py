"""Minimal robot/camera-only Isaac environment with no RoboDojo task scene."""

from __future__ import annotations

import numpy as np
import torch
from omegaconf import OmegaConf
from services.robodojo.camera_calibration import rendered_intrinsics
from services.robodojo.kinematics import measured_joint_state


class RobotOnlyEnvironment:
    """Two X5 articulations, three cameras, a dome light, and nothing else."""

    def __init__(self, sim_config, robot_config, camera_config, app):
        # Imports stay local so module inspection and non-native MCP discovery
        # do not start or import Isaac Sim.
        from env.camera_manager.camera_manager import CameraManager
        from env.camera_manager.capture.tiled_capture_manager import TiledCaptureManager
        from env.environment.base_env import BaseEnv
        from env.robot_manager.robot_manager import RobotManager

        class _Environment(BaseEnv):
            def __init__(nested_self):
                nested_self.sim_config = sim_config
                nested_self.num_envs = 1
                nested_self.env_spacing = float(sim_config.scene.env_spacing)
                nested_self.device = sim_config.get("device", "cpu")
                nested_self.dt = float(sim_config.dt)
                nested_self.robot_manager = RobotManager(
                    num_envs=1,
                    env_spacing=nested_self.env_spacing,
                    config=robot_config,
                    dt=nested_self.dt,
                    device=nested_self.device,
                )
                scene_cfg = nested_self.robot_manager._get_SceneCfg(
                    num_envs=1, env_spacing=nested_self.env_spacing
                )
                super().__init__(sim_config, app=app, interactiveSceneCfg=scene_cfg)
                nested_self.robot_manager._load_camera_cfg(camera_config)
                nested_self.camera_manager = CameraManager(
                    1, camera_config, nested_self.device, seeds_per_env=[0]
                )
                nested_self.capture_manager = TiledCaptureManager(
                    1, camera_config, nested_self.camera_manager, nested_self.device
                )

            def _setup_scene(nested_self, sim):
                import isaaclab.sim as sim_utils

                nested_self.robot_manager.initialize(sim)
                sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)).func(
                    "/World/RobotPreviewLight", sim_utils.DomeLightCfg(intensity=3000.0)
                )
                super()._setup_scene(sim)

            def _post_setup_scene(nested_self, sim):
                super()._post_setup_scene(sim)
                nested_self.camera_manager.initialize(sim)
                nested_self.capture_manager.initialize(sim)
                nested_self.camera_manager.post_init()

            def close(nested_self):
                nested_self.capture_manager.destroy()
                nested_self.camera_manager.destroy()
                nested_self.robot_manager.close()
                super().close()

        self._env = _Environment()
        self.dt = float(sim_config.dt)
        self.ticks_per_observation = 10
        self._started = False

    @property
    def robot_manager(self):
        return self._env.robot_manager

    @property
    def camera_manager(self):
        return self._env.camera_manager

    def start(self):
        if self._started:
            return
        self._env.reset(seed=[0])
        self.robot_manager.reset()
        for _ in range(10):
            self._env.sim_step(render=False)
        self.camera_manager.reset()
        self._env.capture_manager.reset()
        # Tiled render products are attached by capture_manager.reset().  Match
        # RoboDojo's native episode bootstrap: submit Kit render work only
        # after that attachment, then advance through complete render periods
        # before the first annotator read.
        for _ in range(10):
            self._env.render()
        for _ in range(2 * self.ticks_per_observation):
            self._env.sim_step(render=True)
        self._started = True

    @staticmethod
    def _gripper_target(robot, opening):
        low, high = (float(value) for value in robot.gripper_scale)
        primary = (
            opening * (high - low) + low
            if robot.gripper_move["sign"] == 1
            else (1.0 - opening) * (high - low) + low
        )
        mimic = robot.gripper_move["mimic"]
        return [primary, primary * mimic[1] + mimic[2]]

    def set_state(self, state):
        state = np.asarray(state, dtype=np.float32)
        if state.shape != (14,) or not np.isfinite(state).all():
            raise ValueError("initial_state must be a finite 14-D vector")
        if np.any((state[[6, 13]] < 0) | (state[[6, 13]] > 1)):
            raise ValueError("initial gripper openings must be in [0, 1]")
        for robot, offset, key in zip(
            self.robot_manager.robot_list,
            (0, 7),
            self.robot_manager.robot_key,
            strict=True,
        ):
            positions = key.data.default_joint_pos.clone()
            velocities = torch.zeros_like(positions)
            positions[0, robot.arm_joint_indices] = torch.as_tensor(
                state[offset : offset + 6], device=positions.device
            )
            positions[0, robot.gripper_joint_indices] = torch.as_tensor(
                self._gripper_target(robot, float(state[offset + 6])),
                device=positions.device,
            )
            key.write_joint_state_to_sim(positions, velocities)
            key.set_joint_position_target(positions)
            key.set_joint_velocity_target(velocities)
        self._env.sim.scene.write_data_to_sim()
        self._env.sim.scene.update(dt=self.dt)
        self.robot_manager.control_manager.reset()
        self.robot_manager.set_robot_init_state([0])
        # Let camera mounts and PhysX views update without settling away from
        # the explicitly supplied state. A full render interval is required
        # after attaching the tiled render products; otherwise their first
        # read can wait indefinitely for a frame that was never rendered.
        for _ in range(self.ticks_per_observation):
            self._env.sim_step(render=True)

    def measured_state(self):
        return measured_joint_state(self.robot_manager)

    def observation(self):
        # Match EvalEnv.get_obs_batch(): render only after the final physics
        # step has updated the articulation transforms, then read the tiled
        # annotators. Rendering on the last inner tick happens before
        # scene.update(dt) in CustomDirectRLEnv and otherwise leaves the
        # camera products one update behind (or effectively frozen).
        self._env.render()
        captured = self._env.capture_manager.step(env_ids=[0])
        aliases = {"cam_head": "cam_high"}
        result = {
            "states": self.measured_state(),
            "eef_positions": [],
            "eef_quaternions_wxyz": [],
            "camera_parameters": {},
        }
        for robot in self.robot_manager.robot_list:
            end_pose = self.robot_manager.get_real_endpose(robot, env_idx_list=[0])[0]
            result["eef_positions"].append(end_pose[:3])
            result["eef_quaternions_wxyz"].append(end_pose[3:])
        result["eef_positions"] = np.asarray(result["eef_positions"], dtype=np.float32)
        result["eef_quaternions_wxyz"] = np.asarray(
            result["eef_quaternions_wxyz"], dtype=np.float32
        )
        for camera_id, camera_data in enumerate(captured):
            source_name = self.camera_manager.camera_names[0][camera_id]
            name = aliases.get(source_name, source_name)
            if "rgb" in camera_data:
                result[name] = np.asarray(
                    camera_data["rgb"][0]["data"], dtype=np.uint8
                )[..., :3]
            depth_key = (
                "distance_to_image_plane"
                if "distance_to_image_plane" in camera_data
                else "depth"
                if "depth" in camera_data
                else None
            )
            if depth_key is not None:
                depth = np.asarray(camera_data[depth_key][0]["data"], dtype=np.float32)
                if depth.ndim == 3 and depth.shape[-1] == 1:
                    depth = depth[..., 0]
                result[f"{name}_depth_m"] = depth
            rgb = result.get(name)
            resolution = (
                [int(rgb.shape[1]), int(rgb.shape[0])]
                if rgb is not None
                else list(self.camera_manager.cameras[0][camera_id]._resolution)
            )
            result["camera_parameters"][name] = {
                "camera_name": source_name,
                "resolution": resolution,
                "intrinsic_matrix": rendered_intrinsics(
                    self.camera_manager.get_camera_intrinsics(camera_id, 0)
                ).tolist(),
                "camera_to_world_usd": self.camera_manager.get_camera_extrinsics(
                    camera_id, 0
                ).tolist(),
                "extrinsic_convention": "camera_to_world_usd",
                "optical_convention": (
                    "USD +Y up/-Z forward; convert to CV +Y down/+Z forward for pinhole projection"
                ),
                "depth": {
                    "available": f"{name}_depth_m" in result,
                    "type": "distance_to_image_plane",
                    "unit": "meters",
                },
            }
        return result

    def action_controls(self, action):
        """Expand one 25 Hz normalized action exactly like RoboDojo EvalEnv."""
        from env.robot_manager.control_manager import MetaControl

        action = np.asarray(action, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError("action must be a finite 14-D vector")
        if np.any((action[[6, 13]] < 0) | (action[[6, 13]] > 1)):
            raise ValueError("gripper openings must be in [0, 1]")
        controls = [{} for _ in range(self.ticks_per_observation)]
        current = self.measured_state()
        interpolate_ticks = int(np.floor(self.ticks_per_observation * 0.8))
        for robot, offset in zip(self.robot_manager.robot_list, (0, 7), strict=True):
            arm_key = self.robot_manager.process_name(robot.arm_name)
            gripper_key = self.robot_manager.process_name(robot.gripper_name)
            physical_start = self.robot_manager.get_end_effector_real_val(
                robot, env_idx_list=[0]
            )[0][0]
            physical_target = self._gripper_target(robot, float(action[offset + 6]))[0]
            for tick in range(self.ticks_per_observation):
                if tick < interpolate_ticks:
                    alpha = (tick + 1) / (interpolate_ticks + 1)
                else:
                    alpha = 1.0
                q = (1.0 - alpha) * current[offset : offset + 6] + alpha * action[
                    offset : offset + 6
                ]
                gripper_primary = (
                    1.0 - alpha
                ) * physical_start + alpha * physical_target
                gripper_primary = np.clip(gripper_primary, *robot.gripper_scale)
                mimic = robot.gripper_move["mimic"]
                controls[tick][arm_key] = {"position": q, "velocity": np.zeros(6)}
                controls[tick][gripper_key] = {
                    "position": [
                        gripper_primary,
                        gripper_primary * mimic[1] + mimic[2],
                    ],
                    "velocity": [0.0, 0.0],
                }
        return [MetaControl(item) for item in controls]

    def step_inner_controls(self, controls):
        if len(controls) != self.ticks_per_observation:
            raise ValueError(f"Expected {self.ticks_per_observation} inner controls")
        for tick, control in enumerate(controls):
            self.robot_manager.control_robot([control])
            self._env.sim_step(render=tick == len(controls) - 1)

    def close(self):
        self._env.close()


def build_configs(root, *, robot_model, camera_model):
    """Resolve whitelisted lightweight configs; no task or scene config exists."""
    from utils.load_file import load_yaml

    if robot_model != "dual_arx_x5":
        raise ValueError(f"Unsupported robot model: {robot_model}")
    if camera_model != "robodojo_gemini_wrist_v1":
        raise ValueError(f"Unsupported camera model: {camera_model}")
    config_root = root / "env_cfg"
    sim = load_yaml(config_root / "sim" / "sim_config.yml")
    sim["scene"]["num_envs"] = 1
    sim["seed"] = [0]
    robot = load_yaml(config_root / "robot" / "dual_x5.yml")
    for item in robot["robots"]:
        item["need_planner"] = False
    camera = load_yaml(config_root / "camera" / "camera_config.yml")
    camera["default_frequency"] = 25
    camera.setdefault("annotator", {})
    for key in ("common", "cam_head", "cam_left_wrist", "cam_right_wrist"):
        annotation = camera["annotator"].setdefault(key, {})
        annotation["enabled"] = True
        annotation.setdefault("rgb_capture", {"type": "rgb", "device": "cpu"})
        annotation["distance_to_image_plane_capture"] = {
            "type": "distance_to_image_plane",
            "device": "cpu",
        }
    return OmegaConf.create(sim), OmegaConf.create(robot), OmegaConf.create(camera)
