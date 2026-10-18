"""Forward kinematics of the released G1 29-DoF MuJoCo tree, for the expert episodes' derived
virtual points (scripts/setup_humanoid_demos.py). NumPy only; no simulator.

The tree is parsed from the released SONIC MuJoCo description (pinned in dependencies.lock); no
mesh, inertial or collision property is used. Frames and units: the pelvis frame, meters and
scalar-first quaternions ``[w, x, y, z]``.
"""
import math
import re
import xml.etree.ElementTree as ET

import numpy as np

# IsaacLab joint order of the released G1 29-DoF model (gear_sonic joint_utils).
ISAACLAB_JOINTS = [
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
]

# Released SONIC virtual tracking points: body plus a rigid local offset.
# These are controller tracking points, not calibrated Dex3 gripper TCPs.
VIRTUAL_POINTS = {
    "left_hand": ("left_wrist_yaw_link", np.array([0.18, -0.025, 0.0])),
    "right_hand": ("right_wrist_yaw_link", np.array([0.18, 0.025, 0.0])),
    "head": ("torso_link", np.array([0.0, 0.0, 0.35])),
}


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def inverse(q):
    return np.asarray(q) * [1, -1, -1, -1]


def rotate(q, p):
    return multiply(multiply(q, np.r_[0, p]), inverse(q))[1:]


def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / norm
    half = 0.5 * angle
    return np.r_[math.cos(half), math.sin(half) * axis]


def _floats(text):
    return [float(v) for v in re.split(r"\s+", text.strip()) if v]


class Body:
    __slots__ = ("name", "parent", "pos", "quat", "joint", "axis", "joint_pos")

    def __init__(self, name, parent, pos, quat):
        self.name, self.parent = name, parent
        self.pos, self.quat = pos, quat
        self.joint = self.axis = self.joint_pos = None


class G1Kinematics:
    """Pelvis-rooted forward kinematics for the released G1 29-DoF MuJoCo tree."""

    def __init__(self, mjcf_path):
        root = ET.parse(mjcf_path).getroot()
        pelvis = root.find("./worldbody/body")
        if pelvis is None or pelvis.get("name") != "pelvis":
            raise ValueError("Expected a pelvis-rooted G1 MuJoCo description")
        self.bodies = {}
        self.order = []
        self._parse(pelvis, None)
        joints = {body.joint for body in self.bodies.values() if body.joint}
        missing = [name for name in ISAACLAB_JOINTS if name not in joints]
        if missing:
            raise ValueError(f"MuJoCo description is missing released joints: {missing}")

    def _parse(self, element, parent):
        name = element.get("name")
        body = Body(name, parent, np.asarray(_floats(element.get("pos", "0 0 0"))),
                    np.asarray(_floats(element.get("quat", "1 0 0 0"))))
        for joint in element.findall("joint"):
            if joint.get("type") == "free":
                continue
            if body.joint is not None:
                raise ValueError(f"Multiple hinge joints on body {name}; unsupported")
            body.joint = joint.get("name")
            body.axis = np.asarray(_floats(joint.get("axis", "0 0 1")))
            body.joint_pos = np.asarray(_floats(joint.get("pos", "0 0 0")))
        self.bodies[name] = body
        self.order.append(name)
        for child in element.findall("body"):
            self._parse(child, name)

    def link_poses(self, joints_isaaclab):
        """Body poses in the pelvis frame for a 29-vector in IsaacLab joint order."""
        angles = {name: float(joints_isaaclab[i]) for i, name in enumerate(ISAACLAB_JOINTS)}
        poses = {}
        for name in self.order:
            body = self.bodies[name]
            if body.parent is None:
                position, quaternion = np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
            else:
                parent_p, parent_q = poses[body.parent]
                position = parent_p + rotate(parent_q, body.pos)
                quaternion = multiply(parent_q, body.quat)
            if body.joint is not None:
                rotation = quat_from_axis_angle(body.axis, angles.get(body.joint, 0.0))
                pivot = position + rotate(quaternion, body.joint_pos)
                quaternion = multiply(quaternion, rotation)
                position = pivot - rotate(quaternion, body.joint_pos)
            poses[name] = (position, quaternion)
        return poses

    def virtual_points(self, joints_isaaclab):
        """The three SONIC virtual tracking points in the pelvis frame."""
        poses = self.link_poses(joints_isaaclab)
        out = {}
        for label, (body, offset) in VIRTUAL_POINTS.items():
            position, quaternion = poses[body]
            out[label] = (position + rotate(quaternion, offset), quaternion)
        return out
