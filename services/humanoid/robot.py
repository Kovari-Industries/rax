"""G1 + Dex3 semantic40 action and observation contract (dependency-light).

The action is HumanoidArena's canonical 40-value whole-body frame at 50 Hz, tracked by the
frozen SONIC whole-body controller (encoder mode 0). Layout and semantics follow the pinned
upstream converter (``isaaclab_twist2_g1/action_provider``): the controller receives only the
29 joint angles [9:38], their tick-to-tick change, and the root rotation [3:9] relative to the
measured pelvis. Root XY [0:2] and height [2] are part of the layout but do not move the robot:
it translates only by stepping, through the leg joint references.

Observations carry only HumanoidArena's policy inputs: the front camera, the 29 joint states,
the root orientation and the Dex3 finger state. No world position and no object state.
"""

from __future__ import annotations

import math

from services.simulator.io import reject_unless

CONTROL_HZ = 50
CONTROL_DT = 1.0 / CONTROL_HZ
ACTION_DIM = 40
# Rows per motion call: 2 s at 50 Hz. The native horizon still applies.
MAX_ACTION_ROWS = 100
# Saved frame sequences keep every FRAME_STRIDE-th step (5 Hz) plus the final step. A render is
# needed for each saved frame; rendering every 50 Hz tick would dominate the episode's wall time.
FRAME_STRIDE = 10

ACTION_SLICES = {
    "root_xy_delta_m": slice(0, 2),
    "root_height_m": slice(2, 3),
    "root_rot6d": slice(3, 9),
    "joints_canonical29_rad": slice(9, 38),
    "hands_closed": slice(38, 40),
}
# Yaw-free identity rotation: the first two columns of the 3x3 identity, row-major.
IDENTITY_ROT6D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)

# Upstream CANONICAL_G1_JOINT_NAMES_29 (Isaac articulation order of the 29 body joints).
JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
)
assert len(JOINT_NAMES) == ACTION_SLICES["joints_canonical29_rad"].stop - ACTION_SLICES["joints_canonical29_rad"].start

# Agent-facing camera: HumanoidArena's front (head) camera, 640x480, re-posed every
# observation from the measured torso and the fixed upstream mount.
CAMERAS = ("cam_front",)
IMAGE_SIZE = (640, 480)


def action_layout():
    return {name: [s.start, s.stop] for name, s in ACTION_SLICES.items()}


def validate_actions(actions):
    """Shape and value checks of semantic40 rows before any RPC; raises RejectedBeforeAction."""
    reject_unless(isinstance(actions, list) and 1 <= len(actions) <= MAX_ACTION_ROWS,
                  f"actions must be a list of 1..{MAX_ACTION_ROWS} semantic40 rows")
    hands = ACTION_SLICES["hands_closed"]
    for index, row in enumerate(actions):
        reject_unless(isinstance(row, list) and len(row) == ACTION_DIM,
                      f"actions[{index}] must be a list of {ACTION_DIM} numbers")
        for value in row:
            reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                          f"actions[{index}] must contain finite numbers")
        reject_unless(all(row[i] in (0, 1) for i in range(hands.start, hands.stop)),
                      f"actions[{index}][38:40] (hand commands) must be 0 (open) or 1 (closed)")


def rot6d_from_yaw(yaw_rad):
    """The [3:9] block for a pure heading change, relative to the episode's initial heading."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [c, -s, s, c, 0.0, 0.0]
