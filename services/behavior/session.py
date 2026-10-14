"""One BEHAVIOR-1K episode inside the OmniGibson process (runs in runtime/envs/behavior).

The official 2026 ``BatchedEvaluator`` (``num_envs=1``) owns environment creation, task
instance loading, settling, stepping and metrics, so this session reproduces the
organizers' evaluation exactly. The session adds the harness contract: identity-checked
RPC ops, challenge-track agent observations, start-relative odometry, and the trusted
operator files (``operator_state.json``, ``evaluation_outcome.json``, ``summary.json``,
``private_goal_report.json``)
that dashboards and formal scoring read. Nothing privileged is returned to the host
except inside those operator-only files.
"""

from __future__ import annotations

import json
import math
import os
import time
import traceback
import uuid
from pathlib import Path

import numpy as np

from services.simulator.io import RejectedBeforeAction, action_execution_scope, reject_unless, require, write_json
from services.simulator.rpc import error_text
from services.behavior import robot as R

SCHEMA = "behavior_operator_state_v1"


class BehaviorSession:
    def __init__(self, evaluator, output, task, *, mode, include_depth, instruction=None):
        import torch as th

        self.th = th
        self.evaluator = evaluator
        self.env = evaluator.env
        self.state = evaluator.instance_eval_states[0]
        self.robot = self.state.env_accessor.robot
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.task = task
        self.mode = mode
        # Harness episode kind, as RoboDojo records it: held-out test instances are formal;
        # train instances serve exploration and rehearsal.
        self.episode_mode = "formal" if mode == "public_test" else "exploration"
        self.include_depth = bool(include_depth)
        self.episode_id = None
        self.episode_dir = None
        self.step_id = 0
        self.terminated = self.truncated = self.success = False
        self.invalid_reason = None  # set only for a concrete reason, e.g. episode_wall_timeout
        self.finish_reason = None
        self.instance_id = None
        self.odometry = R.Odometry()
        self.grippers = [1.0, 1.0]
        self.plans = {}
        self.truth_log = None
        self._check_action_layout()
        self.lower, self.upper = self._action_limits()
        self.step_limit = int(self.env.task._termination_conditions["timeout"]._max_steps)
        # Pinhole intrinsics from the live sensors' USD attributes (after the evaluator's
        # head-aperture override). VisionSensor.intrinsic_matrix needs a completed render.
        self.intrinsics = {}
        for cam, role in R.CAMERA_ROLES.items():
            sensor = self.robot.sensors[evaluator.robot_camera_names[role].split("::", 1)[1]]
            width, height = int(sensor.image_width), int(sensor.image_height)
            focal = float(sensor.focal_length) * width / float(sensor.horizontal_aperture)
            self.intrinsics[cam] = {"matrix": [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
                                    "width": width, "height": height}
        self.metadata = {
            "task": task,
            "instruction": instruction,
            "benchmark": "behavior",
            "robot": self.robot.model,
            "mode": mode,
            "control_frequency_hz": R.CONTROL_HZ,
            "control_dt": R.CONTROL_DT,
            "native_step_limit": self.step_limit,
            "cameras": list(R.CAMERAS),
            "depth": self.include_depth,
            "action_layout": R.action_layout(),
            "action_lower": self.lower.tolist(),
            "action_upper": self.upper.tolist(),
            "max_action_rows": R.MAX_ACTION_ROWS,
            "frame_stride": R.FRAME_STRIDE,
        }

    # ------------------------------------------------------------------ setup
    def _check_action_layout(self):
        """Fail fast if the robot's controller order differs from the published contract."""
        index = {name: self.robot.controller_action_idx[name] for name in self.robot.controller_order}
        for name, s in R.ACTION_SLICES.items():
            got = [int(i) for i in index[name]]
            require(got == list(range(s.start, s.stop)), f"R1Pro controller {name} occupies {got}, expected {s}")
        require(int(self.robot.action_dim) == R.ACTION_DIM, f"R1Pro action_dim {self.robot.action_dim}")

    def dof(self, name):
        """Robot joint indices driven by controller ``name`` (controllers are batched views)."""
        from omnigibson.controllers.controller_view import ControllerView
        return ControllerView.get_dof_idx(self.robot.controllers[name][0]).cpu().numpy()

    def _action_limits(self):
        lower = np.full(R.ACTION_DIM, -1.0)
        upper = np.full(R.ACTION_DIM, 1.0)
        joint_lower = self.robot.joint_lower_limits.cpu().numpy()
        joint_upper = self.robot.joint_upper_limits.cpu().numpy()
        for name in ("trunk", "arm_left", "arm_right"):
            dof = self.dof(name)
            lower[R.ACTION_SLICES[name]] = joint_lower[dof]
            upper[R.ACTION_SLICES[name]] = joint_upper[dof]
        return lower, upper

    # ------------------------------------------------------------- observation
    def _flat(self):
        return self.state.obs

    def _camera_key(self, cam):
        return self.evaluator.robot_camera_names[R.CAMERA_ROLES[cam]]

    def _gripper_opening(self, side):
        """Measured finger opening mapped onto [-1, 1] over the finger joint range, the
        default output range of the smooth multi-finger gripper controller."""
        dof = self.dof(f"gripper_{side}")
        q = self.robot.get_joint_positions()[dof].cpu().numpy().astype(np.float64)
        low = self.robot.joint_lower_limits.cpu().numpy()[dof]
        high = self.robot.joint_upper_limits.cpu().numpy()[dof]
        value = 2.0 * (q.mean() - low.mean()) / max(1e-9, high.mean() - low.mean()) - 1.0
        return float(np.clip(value, -1.0, 1.0))

    def _agent_observation(self):
        flat = self._flat()
        obs = {}
        for cam in R.CAMERAS:
            key = self._camera_key(cam)
            rgb = flat[key + "::rgb"]
            rgb = rgb.detach().cpu().numpy() if hasattr(rgb, "detach") else np.asarray(rgb)
            obs[cam] = np.ascontiguousarray(rgb[..., :3]).astype(np.uint8)
            if self.include_depth and key + "::depth_linear" in flat:
                depth = flat[key + "::depth_linear"]
                depth = depth.detach().cpu().numpy() if hasattr(depth, "detach") else np.asarray(depth)
                obs[f"{cam}_depth_m"] = depth.astype(np.float32)
        proprio = flat[f"{self.robot.name}::proprio"]
        proprio = proprio.detach().cpu().numpy() if hasattr(proprio, "detach") else np.asarray(proprio)
        obs["proprio"] = proprio.astype(np.float64).reshape(-1).tolist()
        obs["proprio_fields"] = R.decode_proprio(proprio)
        measured = R.hold_action(obs["proprio_fields"], self.grippers)
        obs["joint_limit_violations"] = R.joint_limit_violations([measured], self.lower, self.upper)
        obs["odometry"] = self.odometry.pose()
        obs["gripper_commands"] = {"left": self.grippers[0], "right": self.grippers[1]}
        obs["gripper_openings"] = {side: self._gripper_opening(side) for side in ("left", "right")}
        if self.include_depth:
            rel = flat.get(f"{self.robot.name}::cam_rel_poses")
            if rel is not None:
                rel = rel.detach().cpu().numpy() if hasattr(rel, "detach") else np.asarray(rel)
                roles = list(self.evaluator.robot_camera_names)
                poses = {}
                for i, role in enumerate(roles):
                    cam = next(c for c, r in R.CAMERA_ROLES.items() if r == role)
                    chunk = rel.reshape(-1)[7 * i:7 * i + 7]
                    poses[cam] = {"position_m": chunk[:3].tolist(), "quaternion_xyzw": chunk[3:].tolist(),
                                  "frame": "robot_base"}
                obs["camera_poses"] = poses
            obs["camera_intrinsics"] = self.intrinsics
        obs["instruction"] = self.metadata.get("instruction")
        obs["step_id"] = int(self.step_id)
        return obs

    # ----------------------------------------------------------------- episode
    def reset(self, instance_id, source="agent", policy_version="mcp"):
        require(self.episode_id is None, "One fresh episode per process; no reset")
        self.instance_id = int(instance_id)
        self.evaluator.load_batch({0: self.instance_id})
        # Opaque, as RoboDojo's: the agent sees this ID, and the split and instance stay in the
        # operator files (reset.json, summary.json).
        self.episode_id = uuid.uuid4().hex
        self.episode_dir = self.output / self.episode_id
        (self.episode_dir / "observations").mkdir(parents=True)
        self.step_id = 0
        self.odometry.reset()
        self.grippers = [self._gripper_opening("left"), self._gripper_opening("right")]
        self.truth_log = (self.output / "private_base_truth.jsonl").open("a", encoding="utf-8")
        self.video_path = self.output / "sensors.mp4"
        from omnigibson.eval.utils.obs_utils import create_video_writer
        self.evaluator._set_video_writer(self.state, create_video_writer(
            fpath=str(self.video_path), resolution=(448, 672), rate=R.CONTROL_HZ))
        self.evaluator._write_video(self.state)
        self.start_pose = self._world_pose()
        self.initial_orientations = self._object_orientations()
        self.obs = self._agent_observation()
        self._save_observation()
        write_json(self.output / "reset.json", {
            "episode_id": self.episode_id, "task": self.task, "mode": self.mode,
            "instance_id": self.instance_id, "source": source, "policy_version": policy_version,
            "native_step_limit": self.step_limit, "started_at_unix_s": time.time()})
        self._write_operator_state()
        return {"episode_id": self.episode_id, "step_id": 0, "native_step_limit": self.step_limit}

    def _world_pose(self):
        position, orientation = self.robot.get_position_orientation()
        x, y, z, w = (float(v) for v in orientation)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return [float(position[0]), float(position[1]), yaw]

    def _save_observation(self):
        # Images are kept in the published frame sequences and sensors.mp4; saving full
        # RGB-D here too would cost ~0.2 s per frame and ~1 GB per episode.
        odometry = self.obs["odometry"]
        np.savez(self.episode_dir / "observations" / f"{self.step_id:06d}.npz",
                 proprio=np.asarray(self.obs["proprio"]),
                 odometry=np.array([odometry["x_m"], odometry["y_m"], odometry["yaw_rad"]]))

    def _step_row(self, row):
        th = self.th
        terminated, truncated, info = self.evaluator._apply_actions(
            th.as_tensor(row, dtype=th.float32).reshape(1, -1), [0])
        self.step_id += 1
        self.grippers = [float(row[R.ACTION_SLICES["gripper_left"]][0]),
                         float(row[R.ACTION_SLICES["gripper_right"]][0])]
        proprio = self._flat()[f"{self.robot.name}::proprio"]
        proprio = proprio.detach().cpu().numpy() if hasattr(proprio, "detach") else np.asarray(proprio)
        self.odometry.update(proprio.reshape(-1)[R.PROPRIO_SLICES["base_qvel"]])
        self.evaluator._write_video(self.state)
        pose = self._world_pose()
        self.truth_log.write(json.dumps({"step_id": self.step_id, "world_base_xy_yaw": pose,
                                         "odometry": [self.odometry.x, self.odometry.y, self.odometry.yaw]}) + "\n")
        self.terminated = bool(terminated[0])
        self.truncated = bool(truncated[0])
        self.success = bool(self.env.task.success[0])
        return self.terminated or self.truncated

    def _execute(self, rows, *, kind, extra=None):
        """Run rows from an iterable (possibly closed-loop); keep every FRAME_STRIDE-th
        frame plus the final one. Operator state is written once, at the end of the call,
        or when the call fails after rows ran."""
        require(not (self.terminated or self.truncated), "Episode already ended")
        frames, steps = [], []
        pending = None
        start = self.step_id
        try:
            for row in rows:
                ended = self._step_row(row)
                steps.append({"step_id": self.step_id, "executed_action": np.asarray(row, dtype=np.float32).tolist(),
                              "terminated": self.terminated, "truncated": self.truncated,
                              **({"success": self.success} if ended else {})})
                write_json(self.episode_dir / f"action_{self.step_id - 1:06d}.json", steps[-1])
                pending = True
                if self.step_id % R.FRAME_STRIDE == 0:
                    self.obs = self._agent_observation()
                    frames.append(self.obs)
                    self._save_observation()
                    pending = False
                if ended:
                    break
        except Exception:
            if self.step_id != start:
                self._record_after_failure()
            raise
        if pending:
            self.obs = self._agent_observation()
            frames.append(self.obs)
            self._save_observation()
        if self.terminated or self.truncated:
            self._write_summary("terminal")
        else:
            self._write_operator_state()
        result = {"episode_id": self.episode_id, "step_id": self.step_id, "kind": kind,
                  "observation_frequency_hz": R.CONTROL_HZ / R.FRAME_STRIDE,
                  "control_frequency_hz": R.CONTROL_HZ, "frame_stride": R.FRAME_STRIDE,
                  "frame_count": len(frames), "frames": frames, "steps": steps,
                  "terminated": self.terminated, "truncated": self.truncated,
                  **({"success": self.success} if self.terminated or self.truncated else {})}
        result.update(extra or {})
        return result

    def _record_after_failure(self):
        """A call failed after rows ran: refresh the observation and the operator record, so
        the next observe, the next hold action and the evaluation all start from this step.
        The caller re-raises the original failure; a failure here is only logged."""
        try:
            self.obs = self._agent_observation()
            self._save_observation()
        except Exception:  # noqa: BLE001 - logged; the call's own error is what the agent gets
            traceback.print_exc()
        try:
            if self.terminated or self.truncated:
                self._write_summary("terminal")
            else:
                self._write_operator_state()
        except Exception:  # noqa: BLE001
            traceback.print_exc()

    def chunk_step(self, actions):
        corrections = {}
        matrix = R.validate_actions(actions, self.lower, self.upper, diagnostics=corrections)
        return self._execute(matrix, kind="joint_actions",
                             extra={"joint_target_clamping": corrections} if corrections else None)

    def base_move(self, dx, dy, dyaw, *, max_linear=0.5, max_angular=0.8,
                  position_tolerance_m=0.03, yaw_tolerance_rad=0.03, max_steps=600):
        """Closed-loop planar move relative to the current odometry pose. Uses only the
        integrated proprioceptive base velocity: no map, obstacles or world pose."""
        for name, value in (("dx", dx), ("dy", dy), ("dyaw", dyaw), ("max_linear", max_linear),
                            ("max_angular", max_angular), ("position_tolerance_m", position_tolerance_m),
                            ("yaw_tolerance_rad", yaw_tolerance_rad)):
            reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool)
                          and math.isfinite(value), f"{name} must be a finite number")
        reject_unless(math.hypot(dx, dy) <= 5.0 and abs(dyaw) <= math.pi, "move at most 5 m and pi rad per call")
        reject_unless(0.05 <= max_linear <= 0.75 and 0.05 <= max_angular <= 1.0, "speed limits out of range")
        reject_unless(0.005 <= position_tolerance_m <= 0.5 and 0.005 <= yaw_tolerance_rad <= 0.5,
                      "tolerances out of range")
        reject_unless(isinstance(max_steps, int) and 1 <= max_steps <= 4 * R.MAX_ACTION_ROWS,
                      f"max_steps must be an integer in 1..{4 * R.MAX_ACTION_ROWS}")
        require(not (self.terminated or self.truncated), "Episode already ended")
        goal = R.relative_goal(self.odometry, float(dx), float(dy), float(dyaw))
        corrections = {}
        hold = R.bounded_hold_action(self.obs["proprio_fields"], self.grippers,
                                     self.lower, self.upper, diagnostics=corrections)
        status = {"stalled": False}

        def rows():
            settled, history = 0, []
            for _ in range(max_steps):
                command, (position_error, yaw_error) = R.base_command(
                    self.odometry, goal, max_linear=float(max_linear), max_angular=float(max_angular))
                if position_error <= position_tolerance_m and yaw_error <= yaw_tolerance_rad:
                    settled += 1
                    if settled > 3:
                        return
                    command = np.zeros(3)
                else:
                    settled = 0
                # Stall: commanded motion but under 1 cm / 0.01 rad progress over one second.
                history.append((self.odometry.x, self.odometry.y, self.odometry.yaw))
                if settled == 0 and len(history) > R.CONTROL_HZ:
                    ox, oy, oyaw = history[-R.CONTROL_HZ - 1]
                    moved = math.hypot(self.odometry.x - ox, self.odometry.y - oy)
                    turned = abs(math.atan2(math.sin(self.odometry.yaw - oyaw), math.cos(self.odometry.yaw - oyaw)))
                    if moved < 0.01 and turned < 0.01:
                        status["stalled"] = True
                        return
                row = hold.copy()
                row[R.ACTION_SLICES["base"]] = np.clip(command, -1.0, 1.0)
                yield row

        result = self._execute(rows(), kind="base_move",
                               extra={"joint_target_clamping": corrections} if corrections else None)
        position_error, yaw_error = R.base_command(self.odometry, goal)[1]
        result.update(goal_odometry={"x_m": goal[0], "y_m": goal[1], "yaw_rad": goal[2]},
                      final_position_error_m=position_error, final_yaw_error_rad=yaw_error,
                      reached=bool(position_error <= position_tolerance_m and yaw_error <= yaw_tolerance_rad),
                      stalled=status["stalled"])
        return result

    # ---------------------------------------------------------- operator files
    def _q_score(self):
        from omnigibson.metrics.task_metric import compute_q_score, TaskMetric
        metric = next(m for m in self.state.metrics if isinstance(m, TaskMetric))
        try:
            now = self.env.task.get_goal_option_satisfaction(0)
            q = compute_q_score(self.success, now, metric.initial_predicate_states)
            satisfied = [sum(map(bool, option)) for option in now]
            totals = [len(option) for option in now]
            return float(q), {"satisfied": satisfied, "total": totals}
        except Exception as exc:  # noqa: BLE001 - scoring must not break control
            error = error_text(exc)
            if error != getattr(self, "_q_score_error", None):  # Logged once per distinct failure.
                self._q_score_error = error
                traceback.print_exc()
            return None, {"error": error}

    def _write_operator_state(self, reason=None):
        if self.episode_id is None:
            return
        q, predicates = self._q_score()
        state = {
            "schema": SCHEMA,
            "simulator_pid": os.getpid(),
            "updated_at_unix_s": time.time(),
            "benchmark": "behavior",
            "task": self.task,
            "instruction": self.metadata.get("instruction"),
            "episode_id": self.episode_id,
            "instance_id": self.instance_id,
            "instance_mode": self.mode,
            "episode_mode": self.episode_mode,
            "step_id": int(self.step_id),
            "native_reference_step_limit": self.step_limit,
            "step_limit_enforced": True,
            "episode_step_limit": self.step_limit,
            # Latest RGB frames for the operator's live view (overwritten; history is in the
            # published frame sequences and sensors.mp4).
            "observation_file": f"{self.episode_id}/live.npz",
            "cameras": list(R.CAMERAS),
            "reason": reason,
            "reward": {
                "native_success": bool(self.success),
                "native_reward": 1.0 if self.success else 0.0,
                "native_score_percent": None if q is None else 100.0 * q,
                "q_score": q,
                "goal_predicates": predicates,
                "terminated": bool(self.terminated),
                "truncated": bool(self.truncated),
                "valid_for_success_rate": bool((self.terminated or self.truncated) and not self.invalid_reason),
            },
        }
        live = self.episode_dir / "live.npz"
        with open(live.with_suffix(".tmp"), "wb") as stream:
            np.savez(stream, **{cam: self.obs[cam] for cam in R.CAMERAS if cam in self.obs})
        live.with_suffix(".tmp").replace(live)
        path = self.output / "operator_state.json"
        write_json(path, state)
        path.chmod(0o600)

    def _write_summary(self, reason):
        if self.episode_id is None:
            return
        if self.finish_reason is not None:
            reason = self.finish_reason
        if self.truth_log is not None:
            self.truth_log.flush()
        complete = self.terminated or self.truncated
        eligible = complete and not self.invalid_reason
        q, _ = self._q_score()
        write_json(self.output / "evaluation_outcome.json", {
            "complete": complete,
            "valid_for_success_rate": eligible,
            "native_success": self.success if eligible else None,
            "native_score": q,
            "valid_for_score": q is not None,
            "score_source": "behavior_q_score",
            "invalid_reason": self.invalid_reason,
            "status": "native_completed" if eligible else "incomplete",
            "reason": reason,
            "native_control_steps": self.step_id,
            "native_reference_step_limit": self.step_limit,
            "step_limit_enforced": True,
            "episode_step_limit": self.step_limit,
        })
        write_json(self.output / "summary.json", {
            "episode_id": self.episode_id, "step_id": self.step_id, "success": self.success,
            "terminated": self.terminated, "truncated": self.truncated, "complete": complete,
            "reason": reason, "control_dt": R.CONTROL_DT, "no_rollback": True,
            "instance_id": self.instance_id, "instance_mode": self.mode, "episode_mode": self.episode_mode})
        self._write_operator_state(reason=reason)
        self._write_goal_report(reason)

    def _object_orientations(self):
        """Task objects' orientations at reset, so the review can tell what tipped over."""
        try:
            task = self.env.task
            scope = task.object_scopes[0] if isinstance(task.object_scopes, (list, tuple)) else task.object_scopes
            return {name: [float(v) for v in obj.get_position_orientation()[1]]
                    for name, obj in scope.items() if obj is not None and getattr(obj, "initialized", True)
                    and hasattr(obj, "get_position_orientation")}
        except Exception as exc:  # noqa: BLE001 - the review must never break the episode
            traceback.print_exc()
            self.initial_orientations_error = error_text(exc)
            return {}

    def _write_goal_report(self, reason):
        """Ground truth for the operator-side episode review: every grounded goal condition
        with its initial and final truth, and the poses and extents of the objects involved."""
        try:
            from omnigibson.metrics.task_metric import TaskMetric
            task = self.env.task
            metric = next(m for m in self.state.metrics if isinstance(m, TaskMetric))
            final = task.get_goal_option_satisfaction(0)
            initial = metric.initial_predicate_states
            strip = lambda body: [strip(t) for t in body] if isinstance(body, list) else str(body).lstrip("?")
            options = [[{"condition": strip(head.body), "initial": bool(was), "final": bool(now)}
                        for head, was, now in zip(option, initial_option, final_option)]
                       for option, initial_option, final_option in zip(task.ground_goal_state_options, initial, final)]
            scope = task.object_scopes[0] if isinstance(task.object_scopes, (list, tuple)) else task.object_scopes
            objects = {}
            for name, obj in scope.items():
                if obj is None or not getattr(obj, "initialized", True):
                    objects[name] = {"exists": False}
                    continue
                entry = {"exists": True, "scene_name": getattr(obj, "name", None),
                         "category": getattr(obj, "category", None),
                         "rooms": list(getattr(obj, "in_rooms", None) or [])}
                try:
                    lo, hi = obj.aabb
                    entry["aabb"] = [[float(v) for v in lo], [float(v) for v in hi]]
                    position, orientation = obj.get_position_orientation()
                    entry["position"] = [float(v) for v in position]
                    entry["orientation_xyzw"] = [float(v) for v in orientation]
                    if name in getattr(self, "initial_orientations", {}):
                        entry["initial_orientation_xyzw"] = self.initial_orientations[name]
                    if getattr(obj, "fixed_base", False):  # Supports: per-link extents for the relation geometry.
                        entry["links"] = {link_name: [[float(v) for v in link.aabb[0]], [float(v) for v in link.aabb[1]]]
                                          for link_name, link in obj.links.items() if link.collision_meshes}
                except Exception as exc:  # noqa: BLE001 - systems/particles have no rigid pose
                    entry["pose_error"] = f"{type(exc).__name__}: {exc}"
                objects[name] = entry
            report = {"schema": "behavior_goal_report_v1", "episode_id": self.episode_id, "task": self.task,
                      "instance_id": self.instance_id, "episode_mode": self.episode_mode, "reason": reason,
                      "step_id": int(self.step_id), "step_limit": self.step_limit, "success": bool(self.success),
                      "natural_language_goal": list(task.activity_natural_language_goal_conditions or []),
                      "options": options, "objects": objects, "robot_world_pose": self._world_pose()}
            if getattr(self, "initial_orientations_error", None):
                report["initial_orientations_error"] = self.initial_orientations_error
        except Exception as exc:  # noqa: BLE001 - the review must never break scoring or control
            traceback.print_exc()
            report = {"schema": "behavior_goal_report_v1", "episode_id": self.episode_id,
                      "error": f"{type(exc).__name__}: {exc}", "traceback": error_text(exc)}
        path = self.output / "private_goal_report.json"
        write_json(path, report)
        path.chmod(0o600)

    def finish(self, reason):
        self.finish_reason = self.finish_reason or reason
        self._write_summary(reason)
        self.evaluator._set_video_writer(self.state, None)
        if self.truth_log is not None:
            self.truth_log.close()
            self.truth_log = None
        return {"episode_id": self.episode_id, "step_id": self.step_id, "reason": reason}

    # --------------------------------------------------------------- dispatch
    def _check_identity(self, episode_id, step_id):
        require(self.episode_id is not None, "No active episode")
        reject_unless(episode_id == self.episode_id, "Stale episode")
        reject_unless(int(step_id) == self.step_id, f"Stale step: {step_id} != {self.step_id}")

    def dispatch(self, op, args):
        # Closed-loop row generation and final observation happen within one RPC.
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
        if op == "base_move":
            return self.base_move(**data)
        if op in ("plan_arm", "execute_motion_plan", "eef_move", "motion_plan_rows"):
            from services.behavior.planner import dispatch_planner
            return dispatch_planner(self, op, data)
        raise RejectedBeforeAction(f"Unknown op: {op}")
