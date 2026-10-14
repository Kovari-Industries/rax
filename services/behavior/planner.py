"""cuRobo free-space arm planning inside the OmniGibson process (runs in runtime/envs/behavior).

Challenge-compliant: the planner's world collision checker is created empty and never
updated (``skip_obstacle_update=True``), so it avoids self-collision only and never reads
scene meshes. Targets arrive in the robot base frame (the frame of the proprioceptive EEF
poses). The wrapper plans relative to the holonomic base's virtual root link and refreshes
the locked joints (virtual base, and the trunk for ``arm_no_torso``) from the current state,
so targets are converted to world coordinates here, inside the simulator; the world pose
never leaves this process. Plans become native 23-D rows at the control rate and run
through the ordinary episode step path.
"""

from __future__ import annotations

import uuid

import numpy as np

from services.simulator.io import RejectedBeforeAction, reject_unless, require
from services.behavior import robot as R

SETTLE_STEPS = 10
DLS_DAMPING = 0.05
MAX_COMMAND_LEAD_RAD = 0.2  # Commanded-minus-measured joint lead; bounds wind-up on contact.
MAX_PLAN_ROWS = 20 * R.CONTROL_HZ


class ArmPlanner:
    def __init__(self, session):
        self.session = session
        self.robot = session.robot
        self._generators = {}
        self.plans = {}
        self.retired = {}  # Why an earlier motion_plan_id can no longer run, for its rejection.

    def generator(self, use_trunk):
        """cuRobo's ARM embodiment (trunk and both arms) or its arms-only variant (trunk locked
        at its current joints by the wrapper's per-plan locked-joint refresh).

        On sm_120 GPUs the arms-only variant's finetune trajopt would launch cuRobo's fused
        L-BFGS kernel (history 27) with 392 threads x 168 registers, over the per-block
        register limit ("too many resources requested for launch"; NVlabs/curobo#716). The
        trunk-enabled variant already uses cuRobo's PyTorch L-BFGS step there, so the
        arms-only finetune solver is configured the same way."""
        key = "arm" if use_trunk else "arm_no_torso"
        if key not in self._generators:
            # One generator per process: with both variants built, replaying either one's
            # captured CUDA graphs crashed the simulator (CUDA errors or a segfault).
            self.release()
            from curobo.util_file import get_task_configs_path, join_path, load_yaml
            from omnigibson.action_primitives.curobo import CuRoboEmbodimentSelection, CuRoboMotionGenerator
            self.emb = CuRoboEmbodimentSelection.ARM
            path = self.robot.curobo_path[self.emb]
            options = {}
            if not use_trunk:
                path = path.replace("_curobo_arm.yaml", "_curobo_arm_no_torso.yaml")
                finetune = load_yaml(join_path(get_task_configs_path(), "finetune_trajopt.yml"))
                finetune["lbfgs"]["use_cuda_kernel"] = False
                options["motion_cfg_kwargs"] = {"finetune_trajopt_file": finetune}
            self._generators[key] = CuRoboMotionGenerator(
                self.robot, robot_cfg_path={self.emb: path}, batch_size=2, use_cuda_graph=True, **options)
        return self._generators[key]

    def release(self):
        """Free the current generator, its CUDA graphs and their memory pools."""
        if not self._generators:
            return
        import gc
        import torch
        torch.cuda.synchronize()
        self._generators.clear()
        gc.collect()
        torch.cuda.empty_cache()

    def _targets(self, generator, arm, position, quaternion):
        th = self.session.th
        import omnigibson.utils.transform_utils as TU
        base_pos, base_quat = self.robot.get_position_orientation()
        ee = {side: link for side, link in (("left", "left_eef_link"), ("right", "right_eef_link"))}
        positions, quaternions = {}, {}
        for side, link in ee.items():
            if side == arm:
                # Robot base frame -> world, inside the simulator only.
                p, q = TU.pose_transform(base_pos, base_quat, th.tensor(position, dtype=th.float32),
                                         th.tensor(quaternion, dtype=th.float32))
            else:
                # Hold the other hand where it is.
                p, q = self.robot.eef_links[side].get_position_orientation()
            positions[link], quaternions[link] = p, q
        known = {generator.ee_link[self.emb], *generator.additional_links[self.emb]}
        require(set(positions) <= known, f"cuRobo links {sorted(known)} lack {sorted(set(positions) - known)}")
        return positions, quaternions

    def plan(self, arm, position, quaternion_xyzw, use_trunk=False, preview_only=False):
        session = self.session
        require(not (session.terminated or session.truncated), "Episode already ended")
        reject_unless(arm in ("left", "right"), "arm must be left or right")
        generator = self.generator(use_trunk)
        positions, quaternions = self._targets(generator, arm, position, quaternion_xyzw)
        successes, paths = generator.compute_trajectories(
            target_pos=positions, target_quat=quaternions, is_local=False, skip_obstacle_update=True,
            max_attempts=4, timeout=30.0, emb_sel=self.emb)
        if not bool(successes[0]):
            return {"status": "Planning_Failed", "executed": False, "episode_id": session.episode_id,
                    "step_id": session.step_id, "frames": [], "steps": [], "frame_count": 0,
                    "reason": "cuRobo found no self-collision-free plan to the target"}
        q = generator.path_to_joint_trajectory(paths[0], get_full_js=True, emb_sel=self.emb).cpu().float()
        rows = self._rows(q)
        trunk = rows[:, R.ACTION_SLICES["trunk"]]
        trunk_motion = float(np.abs(trunk - trunk[0]).max()) if len(rows) else 0.0
        require(use_trunk or trunk_motion < 1e-3, f"trunk moved {trunk_motion:.4f} rad in an arms-only plan")
        summary = {"status": "Success", "arm": arm, "use_trunk": bool(use_trunk), "rows": int(len(rows)),
                   "duration_s": len(rows) / R.CONTROL_HZ, "trunk_motion_rad": trunk_motion,
                   "collision_model": "self-collision only; scene objects unknown"}
        if preview_only:
            plan_id = uuid.uuid4().hex[:16]
            self._store(plan_id, (session.episode_id, session.step_id, rows))
            return {**summary, "motion_plan_id": plan_id, "executed": False, "episode_id": session.episode_id,
                    "step_id": session.step_id, "frames": [], "steps": [], "frame_count": 0,
                    "planned_joint_targets": rows[:, :].tolist()[:: max(1, len(rows) // 10)]}
        result = session._execute(rows, kind="free_space_move")
        result.update(summary, executed=True)
        return result

    def _rows(self, q):
        """Full-robot joint waypoints (at the sim step) -> 23-D rows at the control rate."""
        session = self.session
        dt = float(__import__("omnigibson").sim.get_sim_step_dt())
        q = q.numpy()
        if abs(dt - R.CONTROL_DT) > 1e-6 and len(q) > 1:
            times = np.arange(len(q)) * dt
            grid = np.arange(0.0, times[-1] + 1e-9, R.CONTROL_DT)
            q = np.stack([np.interp(grid, times, q[:, j]) for j in range(q.shape[1])], axis=1)
        hold = R.hold_action(session.obs["proprio_fields"], session.grippers)
        rows = np.repeat(hold[None], len(q) + SETTLE_STEPS, axis=0)
        for name in ("trunk", "arm_left", "arm_right"):
            dof = session.dof(name)
            block = q[:, dof]
            rows[: len(q), R.ACTION_SLICES[name]] = block
            rows[len(q):, R.ACTION_SLICES[name]] = block[-1]
        require(len(rows) <= MAX_PLAN_ROWS, f"plan too long ({len(rows)} rows)")
        rows = np.clip(rows, session.lower, session.upper)
        return rows.astype(np.float32)

    def eef_move(self, arm, position, quaternion_xyzw, *, max_steps=150, position_tolerance_m=0.005,
                 orientation_tolerance_rad=0.03, max_joint_step_rad=0.05):
        """Closed-loop damped least-squares IK for one arm toward a robot-base-frame pose: the
        trunk, base and other arm hold their measured joints. Uses the robot's relative
        Jacobian (base-body frame) each control step. Local and not collision-aware, like
        RoboDojo's EEF stepping."""
        import math
        session, robot, th = self.session, self.robot, self.session.th
        import omnigibson.utils.transform_utils as TU
        from omnigibson.utils.usd_utils import ControllableObjectViewAPI
        reject_unless(arm in ("left", "right"), "arm must be left or right")
        reject_unless(isinstance(max_steps, int) and 1 <= max_steps <= 4 * R.MAX_ACTION_ROWS, "invalid max_steps")
        for name, value, low, high in (("position_tolerance_m", position_tolerance_m, 0.001, 0.1),
                                       ("orientation_tolerance_rad", orientation_tolerance_rad, 0.005, 0.5),
                                       ("max_joint_step_rad", max_joint_step_rad, 0.005, 0.2)):
            reject_unless(isinstance(value, (int, float)) and low <= value <= high, f"{name} must be in [{low}, {high}]")
        require(not (session.terminated or session.truncated), "Episode already ended")
        goal_pos = th.tensor(position, dtype=th.float32)
        goal_mat = TU.quat2mat(th.tensor(quaternion_xyzw, dtype=th.float32))
        dof = session.dof(f"arm_{arm}")
        link = robot.eef_links[arm].prim_path.split("/")[-1]
        row_index = ControllableObjectViewAPI.get_link_index(robot.articulation_root_path, link) - 1
        lower = th.as_tensor(session.lower[R.ACTION_SLICES[f"arm_{arm}"]], dtype=th.float32)
        upper = th.as_tensor(session.upper[R.ACTION_SLICES[f"arm_{arm}"]], dtype=th.float32)
        corrections = {}
        hold = R.bounded_hold_action(session.obs["proprio_fields"], session.grippers,
                                     session.lower, session.upper, diagnostics=corrections)
        status = {"position_error_m": None, "orientation_error_rad": None}

        def errors():
            pos, quat = robot.get_relative_eef_pose(arm=arm)
            pos_err = goal_pos - pos
            ori_err = TU.orientation_error(goal_mat.unsqueeze(0), TU.quat2mat(quat).unsqueeze(0))[0]
            return pos_err, ori_err

        def rows():
            # Damped least squares on the measured Jacobian, accumulated into the commanded
            # joint target (the position controllers lag the command) and kept within limits.
            settled = 0
            q_cmd = robot.get_joint_positions()[th.as_tensor(dof)].clone()
            for _ in range(max_steps):
                pos_err, ori_err = errors()
                if float(pos_err.norm()) <= position_tolerance_m and float(ori_err.norm()) <= orientation_tolerance_rad:
                    settled += 1
                    if settled >= 3:
                        return
                else:
                    settled = 0
                jac = robot.get_relative_jacobian()
                offset = jac.shape[-1] - robot.n_dof
                j_eef = jac[row_index][:, th.as_tensor(dof) + offset].double()
                error = th.cat([pos_err, ori_err]).double()
                damping = th.eye(6, dtype=th.float64) * DLS_DAMPING ** 2
                q = robot.get_joint_positions()[th.as_tensor(dof)]
                # Joints held at a limit and pushed outward leave the solve; otherwise their
                # clamped share of the step turns the others' motion in the wrong direction.
                free = th.ones(len(dof), dtype=th.bool)
                for _ in range(len(dof)):
                    j_free = j_eef * free.double()
                    delta = (j_free.T @ th.linalg.solve(j_free @ j_free.T + damping, error)).float()
                    blocked = free & (((q_cmd <= lower + 1e-3) & (delta < 0)) | ((q_cmd >= upper - 1e-3) & (delta > 0)))
                    if not blocked.any():
                        break
                    free &= ~blocked
                delta = delta.clamp(-max_joint_step_rad, max_joint_step_rad)
                q_cmd = (q_cmd + delta).clamp(lower, upper)
                q_cmd = (q + (q_cmd - q).clamp(-MAX_COMMAND_LEAD_RAD, MAX_COMMAND_LEAD_RAD)).clamp(lower, upper)
                row = hold.copy()
                row[R.ACTION_SLICES[f"arm_{arm}"]] = q_cmd.numpy()
                yield row

        result = session._execute(rows(), kind="eef_move",
                                  extra={"joint_target_clamping": corrections} if corrections else None)
        pos_err, ori_err = errors()
        status.update(position_error_m=float(pos_err.norm()), orientation_error_rad=float(ori_err.norm()))
        result.update(arm=arm, reached=bool(status["position_error_m"] <= position_tolerance_m
                                            and status["orientation_error_rad"] <= orientation_tolerance_rad),
                      final_position_error_m=status["position_error_m"],
                      final_orientation_error_rad=status["orientation_error_rad"], collision_model="none")
        if not math.isfinite(status["position_error_m"]):
            session.invalid_reason = "non_finite_end_effector_pose"
            raise RuntimeError(f"eef_move on the {arm} arm ran to step {session.step_id} but the end-effector "
                               f"pose is no longer finite (position error {status['position_error_m']}); "
                               "the physics state is invalid, so this episode no longer counts for the success rate")
        return result

    def _store(self, plan_id, entry):
        """Keep only the newest preview plan; the ones it replaces say so when used."""
        for old in self.plans:
            self.retired[old] = f"replaced by the newer plan {plan_id}; only the latest preview_only plan is kept"
        self.plans = {plan_id: entry}

    def _entry(self, motion_plan_id, *, pop=False):
        entry = self.plans.pop(motion_plan_id, None) if pop else self.plans.get(motion_plan_id)
        if entry is None:
            reason = self.retired.get(motion_plan_id, "no preview_only plan with this id was made in this episode")
            raise RejectedBeforeAction(f"motion_plan_id {motion_plan_id} cannot run: {reason}")
        return entry

    def plan_rows(self, motion_plan_id):
        """A cached plan's native rows, for the robot-only preview; the plan stays executable."""
        session = self.session
        episode, step, rows = self._entry(motion_plan_id)
        reject_unless(episode == session.episode_id and step == session.step_id,
                      "Plan_Stale: the robot moved since planning; plan again")
        return {"motion_plan_id": motion_plan_id, "rows": rows}

    def execute(self, motion_plan_id):
        session = self.session
        episode, step, rows = self._entry(motion_plan_id, pop=True)
        reject_unless(episode == session.episode_id and step == session.step_id,
                      "Plan_Stale: the robot moved since planning; plan again")
        self.retired[motion_plan_id] = f"already executed from step {step}"
        result = session._execute(rows, kind="free_space_move")
        result.update(status="Success", executed=True, motion_plan_id=motion_plan_id)
        return result


def dispatch_planner(session, op, data):
    planner = getattr(session, "_planner", None)
    if planner is None:
        planner = session._planner = ArmPlanner(session)
    if op == "plan_arm":
        return planner.plan(data["arm"], data["position"], data["quaternion_xyzw"],
                            bool(data.get("use_trunk", False)), bool(data.get("preview_only", False)))
    if op == "eef_move":
        options = {k: data[k] for k in ("max_steps", "position_tolerance_m", "orientation_tolerance_rad",
                                        "max_joint_step_rad") if k in data}
        return planner.eef_move(data["arm"], data["position"], data["quaternion_xyzw"], **options)
    if op == "execute_motion_plan":
        return planner.execute(str(data["motion_plan_id"]))
    if op == "motion_plan_rows":
        return planner.plan_rows(str(data["motion_plan_id"]))
    raise RejectedBeforeAction(f"Unknown planner op: {op}")
