"""humanoid_free_space_move: cuRobo arm plans executed as semantic40 frames.

The plan starts at the measured arm joints and avoids self-collision only (the planner knows
no scene objects). Each 50 Hz trajectory sample becomes one semantic40 frame: the planned arm
(and, with use_waist, waist) joints; every other joint, both hands, the root rotation and root
height held at the last commanded frame (at the measured state before any command). The legs
therefore keep their references and the robot does not step.
"""
from __future__ import annotations
import uuid

import numpy as np

from services.humanoid import robot as R
from services.humanoid.sim.quat import inverse, multiply
from services.simulator.io import reject_unless, require

JOINTS = R.ACTION_SLICES["joints_canonical29_rad"]


def _rot6d(q_wxyz):
    w, x, y, z = (float(v) for v in q_wxyz)
    matrix = np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                       [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                       [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])
    return matrix[:, :2].reshape(-1)  # first two columns, row-major


def held_frame(session):
    """The frame the plan holds: the last commanded one, or one built from the measured state."""
    if session.last_command is not None:
        return np.asarray(session.last_command, dtype=np.float64).copy()
    robot = session.env.scene["robot"].data
    root = robot.root_state_w[0, :7].cpu().numpy()
    heading = session.provider._lerobot_vla_runtime._episode_ref_to_world_heading_quat_wxyz
    frame = np.zeros(R.ACTION_DIM)
    frame[R.ACTION_SLICES["root_height_m"]] = root[2]
    frame[R.ACTION_SLICES["root_rot6d"]] = _rot6d(multiply(inverse(np.asarray(heading, dtype=np.float64)), root[3:7]))
    frame[JOINTS] = robot.joint_pos[0, session.joint_index].cpu().numpy()
    return frame


def plan(session, *, arm, position, quaternion_wxyz=None, frame="pelvis", end_effector="palm", use_waist=False,
         time_scale=1.0, preview_only=False):
    require(not (session.terminated or session.truncated), "Episode already ended")
    from services.humanoid import g1
    robot = session.env.scene["robot"].data
    measured = dict(zip(R.JOINT_NAMES, robot.joint_pos[0, session.joint_index].cpu().numpy().tolist()))
    hands = dict(zip(session.hand_names, robot.joint_pos[0, session.hand_index].cpu().numpy().tolist()))
    # The session runs under torch.inference_mode(); cuRobo's IK seeds optimise with autograd.
    torch = session.torch
    with torch.inference_mode(False), torch.enable_grad():
        result = g1.plan_arm_motion(arm, position, quaternion_wxyz, current_joints=measured, frame=frame,
                                    use_waist=use_waist, end_effector=end_effector, control_dt_s=R.CONTROL_DT,
                                    time_scale=time_scale, hand_joints=hands)
    summary = {key: result.get(key) for key in (
        "success", "status", "joint_names", "frame", "end_effector", "position_only", "planning_time_s",
        "duration_s", "position_error_m", "rotation_error_rad", "achieved_pose", "max_abs_velocity_rad_s",
        "limit_violations")}
    if not result["success"] or not result.get("trajectory_canonical29"):
        return {"episode_id": session.episode_id, "step_id": session.step_id, "kind": "free_space_plan",
                "planned": False, "plan": summary, "executed_rows": 0}
    base = held_frame(session)
    solved = [R.JOINT_NAMES.index(name) for name in result["joint_names"]]
    rows = np.repeat(base[None], len(result["trajectory_canonical29"]), axis=0)
    trajectory = np.asarray(result["trajectory_canonical29"], dtype=np.float64)
    rows[:, JOINTS.start + np.asarray(solved)] = trajectory[:, solved]
    rows = rows.astype(np.float32)
    plan_id = uuid.uuid4().hex[:16]
    session.plans = {plan_id: {"episode_id": session.episode_id, "step_id": session.step_id, "rows": rows,
                               "summary": summary}}
    summary.update(motion_plan_id=plan_id, rows=int(len(rows)), seconds=round(len(rows) * R.CONTROL_DT, 3),
                   held="other joints, hands and root rotation at the last commanded frame")
    if preview_only:
        return {"episode_id": session.episode_id, "step_id": session.step_id, "kind": "free_space_plan",
                "planned": True, "plan": summary, "executed_rows": 0}
    return session.execute(rows, kind="free_space_move", extra={"plan": summary})


def execute(session, *, motion_plan_id):
    entry = getattr(session, "plans", {}).get(motion_plan_id)
    reject_unless(entry is not None, "Unknown or expired motion_plan_id; plan again")
    reject_unless(entry["episode_id"] == session.episode_id and entry["step_id"] == session.step_id,
                  "Plan_Stale: the robot moved since this plan was made; plan again from the current state")
    session.plans = {}
    return session.execute(entry["rows"], kind="free_space_move", extra={"plan": entry["summary"]})


def dispatch_planner(session, op, data):
    if op == "plan_arm":
        return plan(session, **data)
    if op == "execute_motion_plan":
        return execute(session, **data)
    raise ValueError(f"Unknown planner op: {op}")


