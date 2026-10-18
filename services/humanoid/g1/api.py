"""Public functions of :mod:`services.humanoid.g1`.

Frames: ``"pelvis"`` (articulation root, default) or ``"torso"`` (``torso_link``), both taken
at the joint configuration supplied with the call. Units: metres, radians, wxyz quaternions.
"""
from __future__ import annotations

import math
import time

import numpy as np

from . import model as g1

FRAMES = ("pelvis", "torso")


# ----------------------------------------------------------------------------- helpers
def _joints_with_defaults(current_joints, hand_joints=None):
    joints = g1.as_joint_dict(current_joints)
    joints.update(g1.as_joint_dict(hand_joints))
    defaulted = [name for name in g1.CANONICAL29 if name not in joints]
    return joints, defaulted


def _base_T_frame(frame, joints):
    """pelvis_T_frame for frame in FRAMES at ``joints``."""
    if frame == "pelvis":
        return np.eye(4)
    if frame == "torso":
        return g1.model().link_transform("torso_link", joints)
    raise ValueError(f"frame must be one of {FRAMES}")


def _target_matrix(position, quaternion):
    position = np.asarray(position, dtype=float).reshape(-1)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("target_position_m must be 3 finite numbers (metres)")
    if quaternion is None:
        return g1.pose_matrix(position, (1.0, 0.0, 0.0, 0.0))
    quaternion = np.asarray(quaternion, dtype=float).reshape(-1)
    if quaternion.shape != (4,):
        raise ValueError("target_quaternion_wxyz must have 4 entries [w, x, y, z]")
    return g1.pose_matrix(position, g1.quat_normalize(quaternion))


def _canonical_vector(joints, solution):
    merged = dict(joints)
    merged.update(solution)
    return [float(merged.get(name, 0.0)) for name in g1.CANONICAL29]


def _limit_violations(solution, tolerance=1e-4):
    limits = g1.joint_limits(list(solution))
    return {name: value for name, value in solution.items()
            if value < limits[name][0] - tolerance or value > limits[name][1] + tolerance}


# ----------------------------------------------------------------------------- FK
def forward_kinematics(joints, frames=None, hand_joints=None, backend="numpy"):
    """Poses of named frames for one joint configuration.

    Args:
        joints: ``{joint_name: rad}`` (missing joints are taken as 0 and listed in
            ``defaulted_joints``) or a 29-vector in :data:`CANONICAL29` order.
        frames: names from :data:`NAMED_FRAMES` (default: all of them).
        hand_joints: optional Dex3 finger angles (no named frame depends on them; accepted for
            symmetry with the collision functions).
        backend: ``"numpy"`` (CPU, no CUDA) or ``"curobo"`` (cuRobo GPU kinematics, same URDF).

    Returns:
        ``{"pelvis": {frame: {"position_m", "quaternion_wxyz"}}, "torso": {...},
        "defaulted_joints": [...], "backend": ...}``
    """
    joint_map, defaulted = _joints_with_defaults(joints, hand_joints)
    names = list(g1.NAMED_FRAMES) if frames is None else list(frames)
    unknown = [name for name in names if name not in g1.NAMED_FRAMES]
    if unknown:
        raise ValueError(f"Unknown frames {unknown}; choose from {list(g1.NAMED_FRAMES)}")
    if backend == "numpy":
        matrices = {name: g1.model().frame_transform(name, joint_map) for name in names}
        torso = g1.model().link_transform("torso_link", joint_map)
    elif backend == "curobo":
        from . import solvers
        matrices, torso = solvers.curobo_frames(joint_map, names)
    else:
        raise ValueError("backend must be 'numpy' or 'curobo'")
    torso_inv = np.linalg.inv(torso)
    return {
        "pelvis": {name: g1.matrix_pose(m) for name, m in matrices.items()},
        "torso": {name: g1.matrix_pose(torso_inv @ m) for name, m in matrices.items()},
        "defaulted_joints": defaulted,
        "backend": backend,
        "units": "metres, wxyz quaternions; poses of each frame in the pelvis / torso_link frame",
    }


def transform_pose(position_m, quaternion_wxyz=None, from_frame="camera_optical", to_frame="pelvis",
                   joints=None):
    """Re-express a point/pose given in one named frame in another, at ``joints``.

    Example: a point measured from the depth image in the ROS optical frame of the front
    camera -> pelvis frame: ``transform_pose(p_cam, from_frame="camera_optical")``.
    """
    joint_map, defaulted = _joints_with_defaults(joints)
    model = g1.model()
    world_T_from = model.frame_transform(from_frame, joint_map)
    world_T_to = model.frame_transform(to_frame, joint_map)
    pose = _target_matrix(position_m, quaternion_wxyz)
    out = np.linalg.inv(world_T_to) @ world_T_from @ pose
    result = g1.matrix_pose(out)
    if quaternion_wxyz is None:
        result.pop("quaternion_wxyz")
    result["defaulted_joints"] = defaulted
    return result


# ----------------------------------------------------------------------------- IK
def solve_arm_ik(side, target_position_m, target_quaternion_wxyz=None, frame="pelvis",
                 current_joints=None, use_waist=False, seeds=32, end_effector="palm",
                 obstacles=None, hand_joints=None):
    """Collision-aware IK for one arm (cuRobo ``InverseKinematics``).

    Args:
        side: ``"left"`` or ``"right"``.
        target_position_m: target of the end-effector origin in ``frame`` (metres).
        target_quaternion_wxyz: target orientation of the end-effector frame in ``frame``;
            ``None`` solves for position only.
        frame: ``"pelvis"`` or ``"torso"`` (``torso_link`` at ``current_joints``).
        current_joints: measured joints ``{name: rad}`` or a CANONICAL29 vector. Every joint not
            being solved for (other arm, legs, waist unless ``use_waist``) is held here; the arm's
            own values seed the solver. Missing joints are taken as 0 (see ``defaulted_joints``).
        use_waist: also solve the three waist joints, kept within yaw +-0.6, roll +-0.25 and
            pitch +-0.35 rad (``solvers.WAIST_LIMITS_RAD``).
        seeds: parallel optimisation seeds (cuRobo ``num_seeds``); each value builds a solver.
        end_effector: ``"palm"`` (``*_hand_palm_link``, default), ``"wrist"``
            (``*_wrist_yaw_link``) or ``"hand_center"`` (point between the open fingers and thumb).
        obstacles: cuboids in ``frame``: ``[{"cuboid": {"dims": [x, y, z],
            "pose_wxyz": [x, y, z, qw, qx, qy, qz]}}]`` (box centre pose, full edge lengths).
        hand_joints: optional Dex3 finger angles for the collision model (default open hand).

    Returns:
        dict with ``success`` (cuRobo: converged within 5 mm / 0.05 rad, within joint limits,
        no self or obstacle collision), ``joints`` (solved joints), ``joints_canonical29``
        (current joints with the solution substituted, CANONICAL29 order), ``position_error_m``,
        ``rotation_error_rad`` (None for position-only; both recomputed with the NumPy FK of the
        returned joints), ``achieved_pose`` (in ``frame``), ``limit_violations``,
        ``solve_time_s`` (wall time of the cuRobo solve), ``call_time_s``,
        ``solver_build_time_s`` (non-zero on the call that built this solver variant),
        ``held_links_excluded_from_collision`` and ``moving_links_touching_obstacles_at_start``.
    """
    from . import solvers
    started = time.perf_counter()
    joints, defaulted = _joints_with_defaults(current_joints, hand_joints)
    base_T_frame = _base_T_frame(frame, joints)
    target = base_T_frame @ _target_matrix(target_position_m, target_quaternion_wxyz)
    position_only = target_quaternion_wxyz is None
    ctx = solvers.context("ik", side, use_waist, end_effector, seeds, position_only)
    built = ctx.build_time_s if not getattr(ctx, "_reported", False) else 0.0
    ctx._reported = True
    raw = solvers.solve_ik(ctx, joints, target, position_only, obstacles, base_T_frame)
    solution = raw["solution"]
    achieved = g1.model().frame_transform(_ee_frame(side, end_effector), {**joints, **solution})
    position_error = float(np.linalg.norm(achieved[:3, 3] - target[:3, 3]))
    rotation_error = None if position_only else g1.rotation_angle_between(
        g1.matrix_to_quat(achieved[:3, :3]), g1.matrix_to_quat(target[:3, :3]))
    return {
        "success": bool(raw["success"]),
        "joints": solution,
        "joints_canonical29": _canonical_vector(joints, solution),
        "position_error_m": position_error,
        "rotation_error_rad": rotation_error,
        "achieved_pose": g1.matrix_pose(np.linalg.inv(base_T_frame) @ achieved),
        "frame": frame,
        "end_effector": end_effector,
        "tool_link": ctx.tool_link,
        "position_only": position_only,
        "limit_violations": _limit_violations(solution),
        "solve_time_s": raw["solve_time_s"],
        "call_time_s": time.perf_counter() - started,
        "solver_build_time_s": built,
        "defaulted_joints": defaulted,
        "curobo_position_error_m": raw.get("position_error_m"),
        "curobo_rotation_error_rad": raw.get("rotation_error_rad"),
        "held_links_excluded_from_collision": raw.get("held_links_excluded_from_collision", []),
        "moving_links_touching_obstacles_at_start": raw.get("moving_links_touching_obstacles_at_start", []),
    }


def _ee_frame(side, end_effector):
    return f"{side}_{end_effector}"


# ----------------------------------------------------------------------------- planning
def resample_trajectory(times_s, positions, dt_s=0.02):
    """Linearly resample a joint trajectory onto a uniform grid ``0, dt, 2dt, ..., T``."""
    times = np.asarray(times_s, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if times.ndim != 1 or positions.shape[0] != times.shape[0]:
        raise ValueError("times_s must be (T,) and positions (T, n)")
    end = float(times[-1])
    count = int(math.floor(end / dt_s + 1e-9)) + 1
    grid = np.arange(count) * dt_s
    if grid[-1] < end - 1e-9:
        grid = np.append(grid, end)
    out = np.stack([np.interp(grid, times, positions[:, j]) for j in range(positions.shape[1])], axis=1)
    return grid, out


def plan_arm_motion(side, target_position_m, target_quaternion_wxyz=None, current_joints=None,
                    obstacles=None, frame="pelvis", use_waist=False, end_effector="palm",
                    control_dt_s=0.02, time_scale=1.0, max_attempts=3, seeds=32, hand_joints=None):
    """Collision-free, time-parameterised arm trajectory (cuRobo ``MotionPlanner``).

    Same targets/obstacles/frames/end effectors as :func:`solve_arm_ik`. The trajectory starts
    at the arm joints in ``current_joints`` (all other joints held there), ends at the target
    (5 mm / 0.05 rad), and respects joint limits, self-collision and the cuboid obstacles
    (1 cm margin) with acceleration <= 6 rad/s^2 and jerk <= 100 rad/s^3 per joint before
    ``time_scale`` (typical reaches peak at 1-1.5 rad/s).

    Args:
        control_dt_s: output sample period (default 0.02 s = the 50 Hz control tick).
        time_scale: >= 1 retimes the same path uniformly slower (2.0 = twice as long, half speed).
        max_attempts: cuRobo planning attempts.

    Returns:
        dict with ``success``, ``status``, ``joint_names`` (solved joints), ``times_s``,
        ``positions`` (one row per sample, rad), ``velocities`` (rad/s), ``dt_s``,
        ``duration_s``, ``final_joints``, ``trajectory_canonical29`` (one 29-vector per sample in
        CANONICAL29 order; held joints from ``current_joints``), ``position_error_m``,
        ``rotation_error_rad``, ``achieved_pose``, ``max_abs_velocity_rad_s``,
        ``limit_violations`` and timing fields.
    """
    from . import solvers
    if time_scale < 1.0:
        raise ValueError("time_scale must be >= 1 (only slowing down keeps the planned limits)")
    started = time.perf_counter()
    joints, defaulted = _joints_with_defaults(current_joints, hand_joints)
    base_T_frame = _base_T_frame(frame, joints)
    target = base_T_frame @ _target_matrix(target_position_m, target_quaternion_wxyz)
    position_only = target_quaternion_wxyz is None
    ctx = solvers.context("plan", side, use_waist, end_effector, seeds, position_only)
    built = ctx.build_time_s if not getattr(ctx, "_reported", False) else 0.0
    ctx._reported = True
    raw = solvers.plan(ctx, joints, target, position_only, obstacles, base_T_frame, max_attempts)
    out = {
        "success": bool(raw["success"]), "status": raw["status"], "joint_names": list(ctx.active),
        "frame": frame, "end_effector": end_effector, "tool_link": ctx.tool_link,
        "position_only": position_only, "planning_time_s": raw["planning_time_s"],
        "call_time_s": None, "solver_build_time_s": built, "defaulted_joints": defaulted,
        "held_links_excluded_from_collision": raw.get("held_links_excluded_from_collision", []),
        "moving_links_touching_obstacles_at_start": raw.get("moving_links_touching_obstacles_at_start", []),
    }
    if raw["positions"] is None:
        out["call_time_s"] = time.perf_counter() - started
        return out
    native_dt = raw["dt_s"] * time_scale
    native_times = np.arange(raw["positions"].shape[0]) * native_dt
    times, positions = resample_trajectory(native_times, raw["positions"], control_dt_s)
    velocities = np.gradient(positions, times, axis=0) if len(times) > 1 else np.zeros_like(positions)
    final = {name: float(v) for name, v in zip(ctx.active, positions[-1])}
    achieved = g1.model().frame_transform(_ee_frame(side, end_effector), {**joints, **final})
    canonical = [_canonical_vector(joints, dict(zip(ctx.active, row))) for row in positions]
    out.update({
        "dt_s": control_dt_s, "times_s": times.tolist(), "positions": positions.tolist(),
        "velocities": velocities.tolist(), "duration_s": float(times[-1]),
        "native_dt_s": native_dt, "time_scale": time_scale,
        "final_joints": final, "trajectory_canonical29": canonical,
        "position_error_m": float(np.linalg.norm(achieved[:3, 3] - target[:3, 3])),
        "rotation_error_rad": None if position_only else g1.rotation_angle_between(
            g1.matrix_to_quat(achieved[:3, :3]), g1.matrix_to_quat(target[:3, :3])),
        "achieved_pose": g1.matrix_pose(np.linalg.inv(base_T_frame) @ achieved),
        "max_abs_velocity_rad_s": float(np.abs(velocities).max()),
        "limit_violations": {
            **_limit_violations({n: float(v) for n, v in zip(ctx.active, positions.min(axis=0))}),
            **_limit_violations({n: float(v) for n, v in zip(ctx.active, positions.max(axis=0))})},
    })
    out["call_time_s"] = time.perf_counter() - started
    return out


# ----------------------------------------------------------------------------- collision
def check_collision(joints, obstacles=None, frame="pelvis", hand_joints=None):
    """Self- and obstacle-collision check of one full-body configuration (cuRobo spheres).

    Uses the same sphere model as the solvers (obstacles get the 1 cm margin; link pairs at most
    two joints apart are not checked against each other). Returns ``{"self_collision": bool,
    "obstacle_collision": bool, "self_penetration_m": deepest sphere-pair overlap,
    "obstacle_penetration_sum_m": summed sphere penetration into obstacles,
    "links_touching_obstacles": {link: depth_m}}``.
    """
    from . import solvers
    joint_map, defaulted = _joints_with_defaults(joints, hand_joints)
    base_T_frame = _base_T_frame(frame, joint_map)
    out = solvers.collision(joint_map, obstacles, base_T_frame)
    out["links_touching_obstacles"] = g1.obstacle_contacts(
        joint_map, obstacles, margin_m=solvers.WORLD_COLLISION_MARGIN_M, base_T_frame=base_T_frame)
    out["defaulted_joints"] = defaulted
    return out


def warmup(sides=("left", "right"), kinds=("ik",), end_effectors=("palm",), use_waist=(False,),
           position_only=(False,), seeds=32):
    """Build (and run once) the requested solvers so later calls in this process only pay solve
    time. Each (kind, side, end effector, waist, position-only) combination is its own solver."""
    report = []
    neutral = {"left_shoulder_roll_joint": 0.3, "right_shoulder_roll_joint": -0.3,
               "left_elbow_joint": 1.0, "right_elbow_joint": 1.0}
    for kind in kinds:
        for side in sides:
            for ee in end_effectors:
                for waist in use_waist:
                    for pos_only in position_only:
                        started = time.perf_counter()
                        pose = g1.model().frame_transform(_ee_frame(side, ee), neutral)
                        quat = None if pos_only else g1.matrix_to_quat(pose[:3, :3])
                        if kind == "ik":
                            result = solve_arm_ik(side, pose[:3, 3], quat, current_joints=neutral,
                                                  end_effector=ee, use_waist=waist, seeds=seeds)
                        elif kind == "plan":
                            result = plan_arm_motion(side, pose[:3, 3] + np.array([0.05, 0.0, 0.05]), quat,
                                                     current_joints=neutral, end_effector=ee,
                                                     use_waist=waist, seeds=seeds)
                        else:
                            raise ValueError("kinds are 'ik' and 'plan'")
                        report.append({"kind": kind, "side": side, "end_effector": ee, "use_waist": waist,
                                       "position_only": pos_only, "success": result["success"],
                                       "seconds": round(time.perf_counter() - started, 3)})
    return report
