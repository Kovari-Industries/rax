"""HumanoidArena episodes inside the Isaac Sim process (G1 + Dex3, frozen SONIC, semantic40).

The pinned Arena task (``isaaclab_twist2_g1``) owns the scene, its seeded randomization, the
50 Hz control cadence and the success predicate; the pinned SONIC action provider owns the
whole-body control. This session adds the harness contract: identity-checked RPC ops,
policy-input-only agent observations (front camera, joints, root orientation, Dex3 fingers),
the upstream fall rule, and the trusted operator files (``operator_state.json``,
``evaluation_outcome.json``, ``summary.json``) that evaluation and the dashboard read.
Nothing privileged is returned to the host except inside those operator-only files.
"""
from __future__ import annotations

import hashlib
import json
import os
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from services.humanoid import robot as R
from services.humanoid.sim.quat import inverse, multiply, rotate
from services.humanoid.tasks import task_info
from services.simulator.io import RejectedBeforeAction, action_execution_scope, reject_unless, require, write_json

SCHEMA = "humanoid_operator_state_v1"
# Isaac Sim 6.1 resolves a few authored asset references against the 5.1 asset root.
# RTX subframes per synchronized observation render (the original Arena camera path used 4).
RENDER_SUBFRAMES = 4
ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"


def plain(value):
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def digest(value):
    return hashlib.sha256(json.dumps(plain(value), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class HumanoidSession:
    def __init__(self, *, task, output, mode, include_depth, arena_root, sonic, seed):
        self.info = task_info(task)
        self.task = task
        self.mode = mode
        self.episode_mode = "formal" if mode == "official_test" else "exploration"
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.include_depth = bool(include_depth)
        self.arena_root = Path(arena_root).resolve()
        self.step_limit = int(self.info.max_steps)
        self.episode_id = self.episode_dir = None
        self.step_id = 0
        self.seed = None
        self.terminated = self.truncated = self.success = self.fallen = False
        self.invalid_reason = None  # set only for a concrete reason, e.g. episode_wall_timeout
        self.finish_reason = None
        self.obs = None
        self.env = self.provider = None
        self._build(sonic, int(seed))

    # ----------------------------------------------------------------- build
    def _build(self, sonic, seed):
        root = self.arena_root
        if not (root / "assets/objects").is_dir() or not (root / "assets/robots").is_dir():
            raise FileNotFoundError("Arena scene/robot assets are not provisioned (scripts/setup_humanoid_assets.sh)")
        os.environ["PROJECT_ROOT"] = str(root)
        os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
        # Freeze the OpenDoor predicate explicitly: door opening, not doorway traversal.
        os.environ.update({"OPEN_DOOR_STRICT_SUCCESS": "1", "OPEN_DOOR_STRICT_REQUIRE_LATCH_UNLOCKED": "1",
                           "OPEN_DOOR_STRICT_REQUIRE_LEAF_OPEN": "1", "OPEN_DOOR_STRICT_REQUIRE_GEOMETRY": "0",
                           "OPEN_DOOR_SUCCESS_LEAF_ANGLE_DEG": "60.0"})
        self.args = SimpleNamespace(
            task=self.info.gym_id, input_source="vla", gmt_backend="sonic", action_source="sonic_wholebody",
            robot_type="unitree_g1_refpose_v3_1", device="cuda:0", task_runtime_profile=self.info.runtime_profile,
            replay_file="", enable_dex3_dds=True, sonic_encoder_path=str(sonic["encoder"]),
            sonic_decoder_path=str(sonic["decoder"]), recording_save_dir=str(self.output / "upstream-recording"),
            sonic_effort_control=False, sonic_vla_root_rot6d_layout="row", sonic_vla_root_max_delta_deg=26.,
            enable_world_camera=False, enable_perspective_camera=False)
        from task_runtime_profiles import apply_task_runtime_profile
        apply_task_runtime_profile(self.args, context="live_inference")
        import carb
        settings = carb.settings.get_settings()
        settings.set_bool("/isaaclab/cameras_enabled", True)  # The shared Camera still reads this legacy key.
        for key in ("cloud", "default", "nvidia"):
            settings.set("/persistent/isaac/asset_root/" + key, ASSET_ROOT)
        import torch
        self.torch = torch
        self.inference_guard = torch.inference_mode()
        self.inference_guard.__enter__()
        from services.humanoid.sim.compat import install_local_transport, install_material_compat
        self.compatibility = install_material_compat()
        install_local_transport()
        import tasks  # noqa: F401  (pinned upstream Gym registrations)
        import gymnasium as gym
        from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
        from tasks.common_env_config import apply_env_config_yaml
        cfg = parse_env_cfg(self.info.gym_id, device="cuda:0", num_envs=1)
        apply_env_config_yaml(cfg, str(root / "tasks/common_env_config" / self.info.profile_file()),
                              task_name=self.info.gym_id, route_name="sonic")
        from services.humanoid.sim.task_adapter import TaskAdapter, missing_scene_assets
        missing = missing_scene_assets(cfg, root)
        if missing:
            raise FileNotFoundError("Required pinned task assets are unavailable; no scene substitution: " + ", ".join(missing))
        # Keep the upstream camera as the mount-calibration reference and render the same view
        # through a camera under the static environment root (no mixed USD/Fabric child transforms).
        import copy
        cfg.scene.front_camera.data_types = ["rgb"]
        cfg.scene.agent_front_camera = copy.deepcopy(cfg.scene.front_camera)
        cfg.scene.agent_front_camera.prim_path = "{ENV_REGEX_NS}/AgentFrontCamera"
        if self.include_depth:
            cfg.scene.agent_front_camera.data_types = ["rgb", "distance_to_image_plane"]
        # Headless Isaac disables scene queries by default; the prim-reset collider check needs them.
        cfg.sim.enable_scene_query_support = True
        cfg.object_reset_seed_source = "env_seed"
        cfg.seed = seed
        if abs(cfg.sim.dt * cfg.decimation - R.CONTROL_DT) > 1e-8:
            raise ValueError("Expected Arena 50 Hz control with the SONIC route's physics cadence")
        self.cfg = cfg
        self.env = gym.make(self.info.gym_id, cfg=cfg).unwrapped
        initializer = getattr(cfg, "initialize_task_scene", None)
        if callable(initializer):
            initializer(self.env, self.args)
        self.scene_patches = []
        from services.humanoid.sim.goal_collider import TASKS as GOAL_TASKS, use_exact_goal_mesh
        if self.task in GOAL_TASKS:
            # Authored before the reset below, which parses physics again.
            import omni.usd
            self.scene_patches = use_exact_goal_mesh(omni.usd.get_context().get_stage(), cfg)
        self.env.sim.reset()
        self.env.reset()
        from services.humanoid.sim.provider import create_provider
        self.provider = create_provider(self.env, self.args)
        self.env.action_provider = self.provider
        self.task_adapter = TaskAdapter(self.info, self.env, cfg)
        robot = self.env.scene["robot"].data
        self.joint_index = self.provider._sonic_idx
        names = [robot.joint_names[int(i)] for i in self.joint_index]
        if tuple(names) != R.JOINT_NAMES:
            raise RuntimeError("Arena joint order differs from the canonical29 contract")
        self.hand_index = torch.cat([self.provider._left_hand_idx, self.provider._right_hand_idx])
        if self.hand_index.numel() != 14:
            raise RuntimeError("Dex3 hand joints are unavailable; all 14 are required")
        self.hand_names = [robot.joint_names[int(i)] for i in self.hand_index]
        limits = robot.soft_joint_pos_limits[0, self.joint_index].cpu().numpy()
        self.camera_body = list(robot.body_names).index("torso_link")
        from services.humanoid.sim.fall import FallDetector
        self.fall_detector = FallDetector(robot.body_names)
        self.metadata = {
            "task": self.task, "benchmark": "humanoid", "mode": self.mode, "episode_mode": self.episode_mode,
            "native_step_limit": self.step_limit, "control_hz": R.CONTROL_HZ,
            "physics_hz": round(1 / cfg.sim.dt), "controller": "SONIC (frozen; encoder mode 0, semantic40)",
            "joint_names": list(R.JOINT_NAMES), "joint_lower": limits[:, 0].tolist(), "joint_upper": limits[:, 1].tolist(),
            "hand_joint_names": self.hand_names, "cameras": list(R.CAMERAS), "depth": self.include_depth,
            "sonic_sha256": dict(sonic.get("sha256", {})), "compatibility": self.compatibility,
            "scene_patches": self.scene_patches}

    # ------------------------------------------------------------- rendering
    def _render_without_physics(self, frames=1, synchronize=False):
        """Render without stepping physics. synchronize=True steps the replicator orchestrator
        (rt_subframes=frames) so camera render products hold the current frame: queued app
        renders alone can return an old or empty RTX buffer."""
        sim = self.env.sim
        before = (float(sim.current_time), int(sim.current_time_step_index))
        if synchronize:
            import omni.replicator.core as rep
            rep.orchestrator.step(rt_subframes=frames, delta_time=0.0, pause_timeline=False)
        else:
            for _ in range(frames):
                sim.render()
        if before != (float(sim.current_time), int(sim.current_time_step_index)):
            raise RuntimeError("Camera rendering unexpectedly advanced physics")

    def _camera_world(self):
        robot = self.env.scene["robot"].data
        body_p = robot.body_pos_w[0, self.camera_body].cpu().numpy()
        body_q = robot.body_quat_w[0, self.camera_body].cpu().numpy()
        return body_p + rotate(body_q, self.front_mount[0]), multiply(body_q, self.front_mount[1])

    def _agent_observation(self):
        """Policy inputs only: no world position, no object state."""
        torch = self.torch
        robot = self.env.scene["robot"].data
        self._render_without_physics(1)
        camera = self.env.scene["agent_front_camera"]
        camera_p, camera_q = self._camera_world()
        camera.set_world_poses(torch.tensor(camera_p[None], device=self.env.device, dtype=torch.float32),
                               torch.tensor(camera_q[None], device=self.env.device, dtype=torch.float32),
                               convention="world")
        self._render_without_physics(RENDER_SUBFRAMES, synchronize=True)
        # force_recompute alone does not invalidate a fresh sensor buffer; the synchronized
        # render above fills it, including the first frame after startup.
        camera.update(self.env.physics_dt, force_recompute=True)
        rgb = camera.data.output["rgb"][0, :, :, :3].cpu().numpy().astype(np.uint8)
        if not rgb.any():
            raise RuntimeError("The front camera returned an all-black frame after a synchronized render")
        root = robot.root_state_w[0, :7].cpu().numpy()
        root_q_inverse = inverse(root[3:7])
        observation = {
            "step_id": int(self.step_id), "sim_time_s": self.step_id * R.CONTROL_DT, "cam_front": rgb,
            "proprio": {
                "root_orientation_world_wxyz": root[3:7].tolist(),
                "joint_names_canonical29": list(R.JOINT_NAMES),
                "joint_positions_rad": robot.joint_pos[0, self.joint_index].cpu().numpy().tolist(),
                "joint_velocities_rad_s": robot.joint_vel[0, self.joint_index].cpu().numpy().tolist(),
                "hand_state": {
                    "joint_names": self.hand_names,
                    "positions_rad": robot.joint_pos[0, self.hand_index].cpu().numpy().tolist(),
                    "velocities_rad_s": robot.joint_vel[0, self.hand_index].cpu().numpy().tolist(),
                    "applied_torque_Nm": robot.applied_torque[0, self.hand_index].cpu().numpy().tolist()}},
        }
        if self.include_depth:
            if "distance_to_image_plane" not in camera.data.output:
                raise RuntimeError("RGB-D requested but the front depth sensor is unavailable; no RGB fallback")
            depth = camera.data.output["distance_to_image_plane"][0].cpu().numpy().reshape(rgb.shape[:2])
            observation["cam_front_depth_m"] = np.where(np.isfinite(depth) & (depth > 0), depth, 0).astype(np.float32)
            observation["camera_intrinsics"] = {"cam_front": {
                "matrix": camera.data.intrinsic_matrices[0].cpu().numpy().tolist(), "width": int(rgb.shape[1]),
                "height": int(rgb.shape[0]), "depth": "distance along the optical axis (m); 0 = invalid"}}
            observation["camera_poses"] = {"cam_front": {
                "frame": "pelvis", "position_m": rotate(root_q_inverse, camera_p - root[:3]).tolist(),
                "quaternion_wxyz": multiply(root_q_inverse, camera_q).tolist(),
                "axes": "camera X forward (optical axis), Y left, Z up"}}
        return observation

    # ----------------------------------------------------------------- episode
    def reset(self, seed, source="agent", policy_version="mcp"):
        require(self.episode_id is None, "One fresh episode per process; no reset")
        self.seed = int(seed)
        self.cfg.seed = self.seed
        self.cfg._episode_object_seed_counter = 0
        self.env.reset(seed=self.seed)
        self.cfg._episode_object_seed_counter = 0  # Arena's deterministic per-episode RNG counter.
        physics_restart = None
        self.task_adapter.reset()
        if self.task_adapter.needs_physics_restart():
            # Collision must follow upstream's written static prims (prim_reset.py): re-parse the
            # stage, then replay the identical seeded reset on the restarted physics.
            started = time.monotonic()
            self.env.sim.reset()
            self.cfg._episode_object_seed_counter = 0
            self.env.reset(seed=self.seed)
            self.cfg._episode_object_seed_counter = 0
            self.task_adapter.reset()
            physics_restart = round(time.monotonic() - started, 3)
            from services.humanoid.sim.prim_reset import measure_static_footprints
            measure_static_footprints(self.task_adapter.prim_repairs)
        self.provider.reset_reference()
        self.fall_detector.reset()
        self.step_id = 0
        self.terminated = self.truncated = self.success = self.fallen = False
        self.invalid_reason = self.finish_reason = None
        self.sim_start = float(self.env.sim.current_time)
        self.env.scene.write_data_to_sim()
        self.env.sim.forward()
        # Flush the robot parent's rendered transform before reading the camera mount.
        self._render_without_physics(1)
        if not hasattr(self, "front_mount"):
            front = self.env.scene["front_camera"]
            front.reset()
            robot = self.env.scene["robot"].data
            p = robot.body_pos_w[0, self.camera_body].cpu().numpy()
            q = robot.body_quat_w[0, self.camera_body].cpu().numpy()
            self.front_mount = (rotate(inverse(q), front.data.pos_w[0].cpu().numpy() - p),
                                multiply(inverse(q), front.data.quat_w_world[0].cpu().numpy()))
        self.last_command = None
        self.episode_id = f"{self.task}_{self.episode_mode}_{self.seed}_{uuid.uuid4().hex[:8]}"
        self.episode_dir = self.output / self.episode_id
        (self.episode_dir / "traces").mkdir(parents=True)
        layout = {"objects": self.task_adapter.layout_objects(),
                  "robot_root": plain(self.env.scene["robot"].data.root_state_w[0, :7].cpu().numpy())}
        if self.task_adapter.door is not None:
            layout["physical_door"] = self.task_adapter.door.fingerprint_state()
        write_json(self.episode_dir / "reset.json", {
            "episode_id": self.episode_id, "task": self.task, "mode": self.mode, "seed": self.seed,
            "source": source, "policy_version": policy_version, "native_step_limit": self.step_limit,
            "layout_fingerprint": digest(layout), "private_initial_layout": layout,
            "prim_reset_repairs": self.task_adapter.prim_repairs, "physics_restart_seconds": physics_restart,
            "started_at_unix_s": time.time()})
        self.obs = self._agent_observation()
        self._write_operator_state()
        return {"episode_id": self.episode_id, "step_id": 0, "native_step_limit": self.step_limit,
                "seed_kind": self.episode_mode}

    def _tick(self):
        """One 50 Hz control step through the frozen controller; returns its private trace row."""
        sim = self.env.sim
        before = float(sim.current_time)
        self.provider.execute_tick()
        if abs(float(sim.current_time) - before - R.CONTROL_DT) > 1e-6:
            raise RuntimeError("Unexpected physical control-step duration")
        from tools.get_reward import sync_reward_after_physics_step
        sync_reward_after_physics_step(self.env)
        self.step_id += 1
        physics_elapsed = float(sim.current_time) - self.sim_start
        if abs(self.step_id * R.CONTROL_DT - physics_elapsed) > 1e-4:
            raise RuntimeError(f"Episode tick clock differs from simulation time at step {self.step_id}")
        robot = self.env.scene["robot"].data
        measured = robot.joint_pos[0, self.joint_index].cpu().numpy()
        target = np.asarray(self.provider._latest_decoder_target, dtype=np.float64)
        self.success = bool(self.task_adapter.success())
        # Upstream's rule (fall.py): checked after success, confirmed over five steps.
        self.fallen, fall_state = self.fall_detector.update(robot)
        contacts = self.env.scene["contact_forces"]
        return {"step_id": self.step_id, "physics_time_s": physics_elapsed,
                "target_joint_positions_rad": target.tolist(), "measured_joint_positions_rad": measured.tolist(),
                "measured_root_world_m_wxyz": plain(robot.root_state_w[0, :7]),
                "contact_body_names": list(contacts.body_names),
                "contact_net_forces_world_N": plain(contacts.data.net_forces_w[0]),
                "root_orientation_projection_deg": float(getattr(self.provider, "_latest_root_projection_deg", 0.)),
                "hand_target_joint_positions_rad": [*self.provider._left_hand_target.tolist(),
                                                    *self.provider._right_hand_target.tolist()],
                "hand_measured_joint_positions_rad": robot.joint_pos[0, self.hand_index].cpu().numpy().tolist(),
                "hand_applied_torque_Nm": robot.applied_torque[0, self.hand_index].cpu().numpy().tolist(),
                "objects_root_world_m_wxyz": {name: plain(asset.data.root_state_w[0, :7])
                                              for kind in ("rigid_objects", "articulations")
                                              for name, asset in getattr(self.env.scene, kind, {}).items()
                                              if name != "robot"},
                "success": self.success, **fall_state}

    def execute(self, rows, *, kind, extra=None):
        """Run semantic40 rows; keep every FRAME_STRIDE-th frame plus the final one. Rows past the
        step limit are cut, not refused. Operator state is written once, at the end of the call."""
        require(self.episode_id is not None, "No active episode")
        require(not (self.terminated or self.truncated), "Episode already ended")
        rows = np.asarray(rows, dtype=np.float32)
        remaining = self.step_limit - self.step_id
        cut = max(0, len(rows) - remaining)
        rows = rows[:remaining]
        self.provider.command, self.provider.segment_step = rows, 0
        frames, steps, trace = [], [], []
        pending = False
        tick_seconds = render_seconds = 0.0
        for row in rows:
            observation_step = self.step_id
            started = time.monotonic()
            trace.append(self._tick())
            tick_seconds += time.monotonic() - started
            self.last_command = row
            steps.append({"step_id": self.step_id, "observation_step_id": observation_step,
                          "executed_action": row.tolist()})
            pending = True
            ended = self.success or self.fallen or self.step_id >= self.step_limit
            if self.step_id % R.FRAME_STRIDE == 0 or ended:
                started = time.monotonic()
                self.obs = self._agent_observation()
                render_seconds += time.monotonic() - started
                frames.append(self.obs)
                pending = False
            if ended:
                break
        if pending:
            started = time.monotonic()
            self.obs = self._agent_observation()
            render_seconds += time.monotonic() - started
            frames.append(self.obs)
        self.terminated = self.success or self.fallen
        self.truncated = not self.terminated and self.step_id >= self.step_limit
        trace_file = f"traces/step_{steps[0]['step_id']:06d}.json"
        (self.episode_dir / trace_file).write_text(json.dumps(
            {"kind": kind, "requested_rows": int(len(rows) + cut), "rows_cut": cut, "ticks": trace},
            allow_nan=False, default=lambda v: plain(v) if hasattr(v, "tolist") else str(v)))
        error = np.asarray(trace[-1]["measured_joint_positions_rad"]) - np.asarray(trace[-1]["target_joint_positions_rad"])
        feedback = {"ticks": len(steps), "rows_cut_at_step_limit": cut,
                    "joint_tracking_rmse_rad": float(np.sqrt(np.mean(error ** 2))),
                    "max_joint_tracking_error_rad": float(np.max(np.abs(error))),
                    "max_root_orientation_projection_deg": max(t["root_orientation_projection_deg"] for t in trace),
                    "tilt_deg": trace[-1]["tilt_deg"], "fallen": self.fallen,
                    "wall_seconds": {"control_steps": round(tick_seconds, 3), "observations": round(render_seconds, 3)}}
        ended = self.terminated or self.truncated
        if ended:
            self._write_summary("terminal")
        else:
            self._write_operator_state()
        result = {"episode_id": self.episode_id, "step_id": self.step_id, "kind": kind,
                  "observation_frequency_hz": R.CONTROL_HZ / R.FRAME_STRIDE, "control_frequency_hz": R.CONTROL_HZ,
                  "frame_stride": R.FRAME_STRIDE, "frame_count": len(frames), "frames": frames, "steps": steps,
                  "feedback": feedback, "terminated": self.terminated, "truncated": self.truncated,
                  **({"success": self.success} if ended else {}),
                  **({"end_reason": "fall" if self.fallen else "task_complete" if self.success else "step_limit"}
                     if ended else {})}
        result.update(extra or {})
        return result

    def chunk_step(self, actions):
        R.validate_actions(np.asarray(actions, dtype=np.float64).tolist())
        return self.execute(actions, kind="semantic40")

    # ---------------------------------------------------------- operator files
    def _write_operator_state(self, reason=None):
        if self.episode_id is None:
            return
        state = {
            "schema": SCHEMA, "simulator_pid": os.getpid(), "updated_at_unix_s": time.time(),
            "benchmark": "humanoid", "task": self.task, "episode_id": self.episode_id, "seed": self.seed,
            "episode_mode": self.episode_mode, "step_id": int(self.step_id),
            "native_reference_step_limit": self.step_limit, "step_limit_enforced": True,
            "episode_step_limit": self.step_limit,
            # Latest front-camera frame for the operator's live view (overwritten each call).
            "observation_file": f"{self.episode_id}/live.npz", "cameras": list(R.CAMERAS),
            "reason": reason, "fallen": bool(self.fallen),
            "reward": {"native_success": bool(self.success), "native_reward": 1.0 if self.success else 0.0,
                       "native_score_percent": 100.0 if self.success else 0.0,
                       "terminated": bool(self.terminated), "truncated": bool(self.truncated),
                       "valid_for_success_rate": bool((self.terminated or self.truncated) and not self.invalid_reason)},
        }
        live = self.episode_dir / "live.npz"
        with open(live.with_suffix(".tmp"), "wb") as stream:
            np.savez(stream, cam_front=self.obs["cam_front"])
        live.with_suffix(".tmp").replace(live)
        path = self.output / "operator_state.json"
        write_json(path, state)
        path.chmod(0o600)

    def _write_summary(self, reason):
        if self.episode_id is None:
            return
        reason = self.finish_reason or reason
        complete = self.terminated or self.truncated
        eligible = complete and not self.invalid_reason
        write_json(self.output / "evaluation_outcome.json", {
            "complete": complete, "valid_for_success_rate": eligible,
            "native_success": self.success if eligible else None,
            # Binary scoring: once closed, the outcome is decided unless the episode was invalidated
            # (e.g. a wall timeout); a controller that returned early scores what it reached.
            "native_score": None if self.invalid_reason else (1.0 if self.success else 0.0),
            "valid_for_score": not self.invalid_reason,
            "score_source": "humanoidarena_success", "invalid_reason": self.invalid_reason,
            "status": "native_completed" if eligible else "incomplete", "reason": reason,
            "end_reason": "fall" if self.fallen else "task_complete" if self.success else "step_limit" if self.truncated else None,
            "native_control_steps": self.step_id, "native_reference_step_limit": self.step_limit,
            "step_limit_enforced": True, "episode_step_limit": self.step_limit})
        write_json(self.output / "summary.json", {
            "episode_id": self.episode_id, "step_id": self.step_id, "success": self.success, "fallen": self.fallen,
            "terminated": self.terminated, "truncated": self.truncated, "complete": complete, "reason": reason,
            "control_dt": R.CONTROL_DT, "no_rollback": True, "seed": self.seed, "episode_mode": self.episode_mode})
        self._write_operator_state(reason=reason)

    def finish(self, reason):
        self.finish_reason = self.finish_reason or reason
        self._write_summary(reason)
        return {"episode_id": self.episode_id, "step_id": self.step_id, "reason": reason}

    def close(self):
        if self.provider is not None:
            self.provider.cleanup()
            self.provider = None
        if self.env is not None:
            self.env.close()
            self.env = None
        if getattr(self, "inference_guard", None) is not None:
            self.inference_guard.__exit__(None, None, None)
            self.inference_guard = None

    # --------------------------------------------------------------- dispatch
    def _check_identity(self, episode_id, step_id):
        require(self.episode_id is not None, "No active episode")
        reject_unless(episode_id == self.episode_id, "Stale episode")
        reject_unless(int(step_id) == self.step_id, f"Stale step: {step_id} != {self.step_id}")

    def dispatch(self, op, args):
        with action_execution_scope(self, op):
            return self._dispatch(op, args)

    def _dispatch(self, op, args):
        if op == "metadata":
            return self.metadata
        if op == "reset":
            return self.reset(**args)
        if op == "finish_pilot":
            require(args["episode_id"] == self.episode_id, "Stale episode")
            return self.finish(str(args.get("reason", "finish")))
        self._check_identity(args["episode_id"], args["step_id"])
        data = {k: v for k, v in args.items() if k not in ("episode_id", "step_id")}
        if op == "observe":
            return self.obs
        if op == "chunk_step":
            return self.chunk_step(data["actions"])
        if op in ("plan_arm", "execute_motion_plan"):
            from services.humanoid.sim.planner import dispatch_planner
            return dispatch_planner(self, op, data)
        raise RejectedBeforeAction(f"Unknown op: {op}")
