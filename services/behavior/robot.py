"""R1Pro action/observation contract for the 2026 BEHAVIOR challenge (dependency-light).

Layouts mirror ``omnigibson/eval/utils/eval_utils.py`` (ACTION_QPOS_INDICES and
PROPRIOCEPTION_INDICES for R1Pro) and ``omnigibson/eval/r1pro.yaml``. Only challenge-track
inputs are exposed: RGB, depth and the 61-D proprioception, plus odometry integrated from
the proprioceptive base velocity. No world pose, object state or segmentation.
"""

from __future__ import annotations

import math

import numpy as np

from services.simulator.io import reject_unless

CONTROL_HZ = 30
CONTROL_DT = 1.0 / CONTROL_HZ
ACTION_DIM = 23
# Rows per motion call (5 s at 30 Hz). The native horizon still applies.
MAX_ACTION_ROWS = 150
# Accommodate rounding in explicit joint targets without sending illegal targets.
# The radio rehearsals reached 9.90e-6 rad; 1e-4 rad is about 0.006 degrees.
JOINT_LIMIT_TOLERANCE_RAD = 1e-4
# Saved frame sequences keep every FRAME_STRIDE-th step (5 Hz) plus the final step;
# full-rate saving would be ~10 GB per episode at the official camera resolution.
FRAME_STRIDE = 6

ACTION_SLICES = {
    "base": slice(0, 3),
    "trunk": slice(3, 7),
    "arm_left": slice(7, 14),
    "gripper_left": slice(14, 15),
    "arm_right": slice(15, 22),
    "gripper_right": slice(22, 23),
}
# Base: normalized [vx, vy, wz] in the robot frame; [-1, 1] maps to +-0.75 m/s and +-1 rad/s.
BASE_VELOCITY_SCALE = (0.75, 0.75, 1.0)
NORMALIZED = ("base", "gripper_left", "gripper_right")

PROPRIO_SLICES = {
    "base_qvel": slice(0, 3),
    "arm_left_qpos": slice(3, 10),
    "arm_left_qvel": slice(10, 17),
    "eef_left_pos": slice(17, 20),
    "eef_left_quat": slice(20, 24),
    "gripper_left_qpos": slice(24, 26),
    "gripper_left_qvel": slice(26, 28),
    "arm_right_qpos": slice(28, 35),
    "arm_right_qvel": slice(35, 42),
    "eef_right_pos": slice(42, 45),
    "eef_right_quat": slice(45, 49),
    "gripper_right_qpos": slice(49, 51),
    "gripper_right_qvel": slice(51, 53),
    "trunk_qpos": slice(53, 57),
    "trunk_qvel": slice(57, 61),
}
PROPRIO_DIM = 61

# Agent-facing camera names -> eval camera roles (r1pro.yaml eval.camera_sensor_names).
CAMERAS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
CAMERA_ROLES = {"cam_head": "head", "cam_left_wrist": "left_wrist", "cam_right_wrist": "right_wrist"}


def action_layout():
    """Agent-facing description of one 23-D action row."""
    return {
        "dimension": ACTION_DIM,
        "frequency_hz": CONTROL_HZ,
        "fields": [
            {"name": name, "start": s.start, "stop": s.stop,
             "units": ("normalized [-1, 1]" if name in NORMALIZED else "absolute joint radians")}
            for name, s in ACTION_SLICES.items()
        ],
        "base": ("Robot-frame velocity command [vx, vy, wz], each in [-1, 1]; scaled to "
                 "+-0.75 m/s (x, y) and +-1 rad/s (yaw). Zero holds the base still."),
        "grippers": "Normalized opening command in [-1, 1].",
        "holding": "Arms and trunk are absolute position targets: repeat measured joints, bounded by action_limits, to hold still.",
        "joint_limit_tolerance_rad": JOINT_LIMIT_TOLERANCE_RAD,
        "joint_limit_policy": "Explicit targets within tolerance are clamped and reported; larger violations reject before action. Internally generated base/EEF hold targets always clamp measured joint drift and report it.",
    }


def joint_limit_violations(actions, lower, upper):
    """Describe absolute joints outside the actual bounds; do not alter measurements."""
    matrix = np.asarray(actions, dtype=np.float64)
    lower, upper = np.asarray(lower), np.asarray(upper)
    violations = []
    for name in ("trunk", "arm_left", "arm_right"):
        s = ACTION_SLICES[name]
        block, low, high = matrix[:, s], lower[s], upper[s]
        for row, joint in np.argwhere((block < low) | (block > high)):
            value = float(block[row, joint])
            bound = float(np.clip(value, low[joint], high[joint]))
            violations.append({"part": name, "row_index": int(row), "joint_index": int(joint),
                               "action_index": s.start + int(joint), "value_rad": value,
                               "lower_rad": float(low[joint]), "upper_rad": float(high[joint]),
                               "clamped_rad": bound, "excess_rad": abs(value - bound)})
    return violations


def validate_actions(actions, lower=None, upper=None, *, max_rows=MAX_ACTION_ROWS, diagnostics=None):
    """Validate the entire batch before physics; clamp only epsilon-scale excursions.

    ``diagnostics`` receives a bounded correction report for the public reply.
    The input is never modified, including when a later row is rejected.
    """
    matrix = np.asarray(actions, dtype=np.float64)
    reject_unless(matrix.ndim == 2 and matrix.shape[1] == ACTION_DIM,
                  f"actions must be an N x {ACTION_DIM} matrix")
    reject_unless(1 <= matrix.shape[0] <= max_rows, f"actions must have 1..{max_rows} rows")
    reject_unless(bool(np.isfinite(matrix).all()), "actions must be finite")
    for name in NORMALIZED:
        block = matrix[:, ACTION_SLICES[name]]
        reject_unless(bool((np.abs(block) <= 1.0 + 1e-9).all()), f"{name} commands must be in [-1, 1]")
    if lower is not None and upper is not None:
        lower, upper = np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)
        violations = joint_limit_violations(matrix, lower, upper)
        for item in violations:
            reject_unless(item["excess_rad"] <= JOINT_LIMIT_TOLERANCE_RAD,
                          f'{item["part"]} targets exceed joint limits: row {item["row_index"]}, '
                          f'joint {item["joint_index"]} (action[{item["action_index"]}]) = {item["value_rad"]:.12g} rad; '
                          f'bounds [{item["lower_rad"]:.12g}, {item["upper_rad"]:.12g}], '
                          f'excess {item["excess_rad"]:.12g} rad > tolerance {JOINT_LIMIT_TOLERANCE_RAD:g} rad')
        matrix = matrix.copy()
        for name in ("trunk", "arm_left", "arm_right"):
            s = ACTION_SLICES[name]
            matrix[:, s] = np.clip(matrix[:, s], lower[s], upper[s])
        if diagnostics is not None and violations:
            diagnostics.update(count=len(violations), tolerance_rad=JOINT_LIMIT_TOLERANCE_RAD,
                               max_excess_rad=max(v["excess_rad"] for v in violations),
                               corrections=violations[:20], corrections_truncated=len(violations) > 20)
    return matrix.astype(np.float32)


def decode_proprio(vector):
    """Split the official 61-D proprioception into named fields."""
    vector = np.asarray(vector, dtype=np.float64).reshape(-1)
    if vector.shape[0] != PROPRIO_DIM:
        raise ValueError(f"expected {PROPRIO_DIM}-D proprioception, got {vector.shape[0]}")
    return {name: vector[s].tolist() for name, s in PROPRIO_SLICES.items()}


def hold_action(proprio, grippers):
    """Action row that keeps the base still and holds the measured arm/trunk joints.

    ``grippers`` is the last commanded (left, right) gripper value: the measured finger
    opening is not a command, so holding repeats the previous command.
    """
    fields = proprio if isinstance(proprio, dict) else decode_proprio(proprio)
    row = np.zeros(ACTION_DIM, dtype=np.float64)
    row[ACTION_SLICES["trunk"]] = fields["trunk_qpos"]
    row[ACTION_SLICES["arm_left"]] = fields["arm_left_qpos"]
    row[ACTION_SLICES["arm_right"]] = fields["arm_right_qpos"]
    row[ACTION_SLICES["gripper_left"]] = grippers[0]
    row[ACTION_SLICES["gripper_right"]] = grippers[1]
    return row


def bounded_hold_action(proprio, grippers, lower, upper, *, diagnostics=None):
    """Clamp an internally generated measured hold, regardless of excursion size.

    Measurements remain raw. Explicit agent targets use validate_actions instead.
    Finiteness and normalized command checks still apply to these generated rows.
    """
    row = hold_action(proprio, grippers)
    validate_actions([row])
    violations = joint_limit_violations([row], lower, upper)
    for name in ("trunk", "arm_left", "arm_right"):
        s = ACTION_SLICES[name]
        row[s] = np.clip(row[s], lower[s], upper[s])
    if diagnostics is not None and violations:
        diagnostics.update(count=len(violations), policy="clamp_measured_hold",
                           max_excess_rad=max(v["excess_rad"] for v in violations),
                           corrections=violations[:20], corrections_truncated=len(violations) > 20)
    return row.astype(np.float32)


class Odometry:
    """Planar pose relative to the episode start, integrated from robot-frame base velocity.

    Only the proprioceptive ``base_qvel`` ([vx, vy, wz] in the robot frame) is used, which
    every challenge-track policy receives. Trapezoidal integration over one control period.
    """

    def __init__(self, dt=CONTROL_DT):
        self.dt = float(dt)
        self.reset()

    def reset(self):
        self.x = self.y = self.yaw = 0.0
        self._previous = None
        self.distance = 0.0

    def update(self, base_qvel):
        v = np.asarray(base_qvel, dtype=np.float64).reshape(3)
        previous = v if self._previous is None else self._previous
        vx, vy, wz = (previous + v) / 2.0
        yaw_mid = self.yaw + wz * self.dt / 2.0
        c, s = math.cos(yaw_mid), math.sin(yaw_mid)
        dx, dy = (c * vx - s * vy) * self.dt, (s * vx + c * vy) * self.dt
        self.x += dx
        self.y += dy
        self.yaw = math.atan2(math.sin(self.yaw + wz * self.dt), math.cos(self.yaw + wz * self.dt))
        self.distance += math.hypot(dx, dy)
        self._previous = v

    def pose(self):
        return {"x_m": self.x, "y_m": self.y, "yaw_rad": self.yaw,
                "frame": "episode_start_robot_frame", "distance_travelled_m": self.distance,
                "source": "integrated proprioceptive base_qvel"}


def relative_goal(odometry, dx, dy, dyaw):
    """Goal pose expressed in the odometry frame for a move relative to the current pose."""
    c, s = math.cos(odometry.yaw), math.sin(odometry.yaw)
    return (odometry.x + c * dx - s * dy, odometry.y + s * dx + c * dy,
            math.atan2(math.sin(odometry.yaw + dyaw), math.cos(odometry.yaw + dyaw)))


def base_command(odometry, goal, *, max_linear=0.5, max_angular=0.8, gain=2.0):
    """Normalized base velocity toward ``goal`` (x, y, yaw) in the odometry frame."""
    gx, gy, gyaw = goal
    ex, ey = gx - odometry.x, gy - odometry.y
    c, s = math.cos(odometry.yaw), math.sin(odometry.yaw)
    # Error in the current robot frame.
    fx, fy = c * ex + s * ey, -s * ex + c * ey
    eyaw = math.atan2(math.sin(gyaw - odometry.yaw), math.cos(gyaw - odometry.yaw))
    vx, vy, wz = gain * fx, gain * fy, gain * eyaw
    linear = math.hypot(vx, vy)
    if linear > max_linear:
        vx, vy = vx * max_linear / linear, vy * max_linear / linear
    wz = max(-max_angular, min(max_angular, wz))
    return np.array([vx / BASE_VELOCITY_SCALE[0], vy / BASE_VELOCITY_SCALE[1], wz / BASE_VELOCITY_SCALE[2]]), (
        math.hypot(ex, ey), abs(eyaw))
