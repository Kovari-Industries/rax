"""G1 + Dex3 kinematics (numpy) and cuRobo, arm IK and motion planning, used by humanoid_free_space_move.

Body-frame only. Every pose is expressed in the ``pelvis`` frame (the articulation root,
default) or in the ``torso_link`` frame (``frame="torso"``), evaluated at the joint
configuration you pass. Metres, radians, quaternions ``[w, x, y, z]``. Joint names are the
simulator's canonical names (:data:`CANONICAL29`).

Quick reference::

    from services.humanoid import g1
    fk = g1.forward_kinematics(joints)                 # named frame poses, CPU only
    table = {"cuboid": {"dims": [0.5, 1.2, 0.04], "pose_wxyz": [0.55, 0, -0.02, 1, 0, 0, 0]}}
    ik = g1.solve_arm_ik("right", [0.35, -0.2, 0.08], [0.7071068, -0.7071068, 0, 0],
                         current_joints=joints, end_effector="hand_center", obstacles=[table])
    plan = g1.plan_arm_motion("right", [0.35, -0.2, 0.08], [0.7071068, -0.7071068, 0, 0],
                              current_joints=joints, end_effector="hand_center", obstacles=[table])

CUDA is only initialised by the IK / planning / collision functions (lazily, cached per process).
"""
from __future__ import annotations

from .api import (check_collision, forward_kinematics, plan_arm_motion, resample_trajectory,
                  solve_arm_ik, transform_pose, warmup)
from .model import (ALL_JOINTS, ARM_JOINTS, CANONICAL29, END_EFFECTORS, HAND_JOINTS,
                    LEG_JOINTS, NAMED_FRAMES, WAIST_JOINTS, joint_limits)

__version__ = "0.1.0"

__all__ = [
    "CANONICAL29", "ARM_JOINTS", "WAIST_JOINTS", "LEG_JOINTS", "HAND_JOINTS", "ALL_JOINTS",
    "NAMED_FRAMES", "END_EFFECTORS", "joint_limits", "forward_kinematics", "transform_pose",
    "solve_arm_ik", "plan_arm_motion", "resample_trajectory", "check_collision", "warmup",
]
