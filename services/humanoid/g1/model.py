"""Unitree G1 29-DoF + Dex3 model data and a dependency-light forward kinematics.

Everything here is CPU-only NumPy and never touches CUDA. The URDF is Unitree's
``g1_29dof_with_hand_rev_1_0.urdf`` (BSD-3-Clause, see ``robot/LICENSE.unitree``),
the source the simulator's ``g1_29dof_with_dex3_rev_1_0.usd`` was converted from.
The cuRobo solvers in :mod:`services.humanoid.g1.solvers` load the same file.

Conventions: metres, radians, quaternions ``[w, x, y, z]``, poses expressed in the
``pelvis`` body frame (the articulation root) unless a function says otherwise.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np

ROBOT_DIR = Path(__file__).resolve().parent / "robot"
URDF_PATH = ROBOT_DIR / "g1_29dof_with_hand_rev_1_0.urdf"
CUROBO_SPHERES_PATH = ROBOT_DIR / "g1_dex3_spheres.yml"

#: Isaac Lab joint order of the simulated G1 (``observation.proprioception.joint_names_canonical29``).
CANONICAL29 = [
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

WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
ARM_JOINTS = {
    side: [f"{side}_{name}_joint" for name in (
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
        "wrist_roll", "wrist_pitch", "wrist_yaw")]
    for side in ("left", "right")
}
LEG_JOINTS = [f"{side}_{name}_joint" for side in ("left", "right") for name in (
    "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
#: Dex3 finger joints. They are not part of the 29-DoF action; zero is the open, straight hand.
HAND_JOINTS = {
    side: [f"{side}_hand_{name}_joint" for name in (
        "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")]
    for side in ("left", "right")
}
ALL_JOINTS = CANONICAL29 + HAND_JOINTS["left"] + HAND_JOINTS["right"]

#: Rotation from the camera body frame (x forward along the optical axis, y left, z up) to the
#: ROS optical frame (x right, y down, z forward). HumanoidArena mounts ``front_cam`` on
#: ``d435_link`` with this rotation (``convention="ros"``, zero translation).
CAMERA_OPTICAL_FROM_BODY_WXYZ = (0.5, -0.5, 0.5, -0.5)

FRAMES_JSON = ROBOT_DIR / "g1_dex3_frames.json"


def _derived_points():
    """Mesh-derived convenience points written by ``build_robot_config.py``."""
    import json
    if not FRAMES_JSON.is_file():
        return {"left": (0.0, 0.0, 0.0), "right": (0.0, 0.0, 0.0)}, (0.0, 0.0, 0.0)
    data = json.loads(FRAMES_JSON.read_text())
    hands = {side: tuple(float(v) for v in data["hand_center_in_palm_m"][side]) for side in ("left", "right")}
    return hands, tuple(float(v) for v in data["head_center_in_head_link_m"])


#: Hand centre in the palm-link frame (m): midpoint between the open distal finger links and the
#: open distal thumb link, from the Dex3 meshes. A geometric convenience point, not a calibrated
#: grasp or contact point. Head centre in the ``head_link`` frame (mesh bounding-box centre);
#: ``head_link`` itself has its origin at the waist (Unitree convention).
HAND_CENTER_IN_PALM_M, HEAD_CENTER_IN_HEAD_LINK_M = _derived_points()

#: Named frames: name -> (urdf link, position offset in that link (m), rotation offset wxyz).
NAMED_FRAMES = {
    "pelvis": ("pelvis", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    "torso": ("torso_link", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    "head_link": ("head_link", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    "head_center": ("head_link", HEAD_CENTER_IN_HEAD_LINK_M, (1.0, 0.0, 0.0, 0.0)),
    "camera": ("d435_link", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    "camera_optical": ("d435_link", (0.0, 0.0, 0.0), CAMERA_OPTICAL_FROM_BODY_WXYZ),
}
for _side in ("left", "right"):
    NAMED_FRAMES[f"{_side}_wrist"] = (f"{_side}_wrist_yaw_link", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    NAMED_FRAMES[f"{_side}_palm"] = (f"{_side}_hand_palm_link", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    NAMED_FRAMES[f"{_side}_hand_center"] = (
        f"{_side}_hand_palm_link", HAND_CENTER_IN_PALM_M[_side], (1.0, 0.0, 0.0, 0.0))

#: End-effector choices for IK/planning: name -> named frame, per side.
END_EFFECTORS = ("wrist", "palm", "hand_center")


# ----------------------------------------------------------------------------- quaternion math
def quat_multiply(a, b):
    """Hamilton product of wxyz quaternions."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conjugate(q):
    return np.asarray(q, dtype=float) * np.array([1.0, -1.0, -1.0, -1.0])


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError(f"Invalid quaternion {q!r}")
    q = q / norm
    return q if q[0] >= 0 else -q


def quat_to_matrix(q):
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(m):
    """Rotation matrix -> wxyz quaternion with w >= 0."""
    m = np.asarray(m, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    return quat_normalize(q)


def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rotation_angle_between(q_a, q_b):
    """Geodesic angle (rad) between two wxyz quaternions."""
    d = abs(float(np.dot(quat_normalize(q_a), quat_normalize(q_b))))
    return 2.0 * math.acos(min(1.0, d))


def pose_matrix(position, quaternion_wxyz):
    out = np.eye(4)
    out[:3, :3] = quat_to_matrix(quaternion_wxyz)
    out[:3, 3] = np.asarray(position, dtype=float)
    return out


def matrix_pose(matrix):
    return {"position_m": [float(v) for v in matrix[:3, 3]],
            "quaternion_wxyz": [float(v) for v in matrix_to_quat(matrix[:3, :3])]}


# ----------------------------------------------------------------------------- URDF model
class UrdfJoint:
    __slots__ = ("name", "type", "parent", "child", "origin", "axis", "lower", "upper", "velocity")

    def __init__(self, element):
        self.name = element.get("name")
        self.type = element.get("type")
        self.parent = element.find("parent").get("link")
        self.child = element.find("child").get("link")
        origin = element.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        self.origin = np.eye(4)
        self.origin[:3, :3] = rpy_to_matrix(*rpy)
        self.origin[:3, 3] = xyz
        axis = element.find("axis")
        self.axis = np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else None
        limit = element.find("limit")
        self.lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else None
        self.upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else None
        self.velocity = float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None

    def transform(self, q=0.0):
        """parent_T_child at joint angle ``q`` (rad)."""
        if self.type in ("revolute", "continuous"):
            motion = np.eye(4)
            motion[:3, :3] = axis_angle_matrix(self.axis, q)
            return self.origin @ motion
        if self.type == "fixed":
            return self.origin
        raise ValueError(f"Unsupported joint type {self.type} for {self.name}")


class G1Model:
    """Pelvis-rooted kinematic tree parsed from the package URDF."""

    def __init__(self, urdf_path=URDF_PATH):
        root = ET.parse(str(urdf_path)).getroot()
        self.joints = {j.get("name"): UrdfJoint(j) for j in root.findall("joint")}
        self.child_joint = {j.child: j for j in self.joints.values()}
        self.links = [link.get("name") for link in root.findall("link")]
        missing = [name for name in ALL_JOINTS if name not in self.joints]
        if missing:
            raise ValueError(f"URDF lacks joints {missing}")

    def limits(self, names=CANONICAL29):
        return {name: (self.joints[name].lower, self.joints[name].upper) for name in names}

    def chain(self, link, root="pelvis"):
        """Joints from ``root`` down to ``link``."""
        out = []
        while link != root:
            joint = self.child_joint.get(link)
            if joint is None:
                raise ValueError(f"{link} is not below {root}")
            out.append(joint)
            link = joint.parent
        return out[::-1]

    def link_transform(self, link, joints, root="pelvis"):
        """root_T_link (4x4) for a joint dict; absent joints are taken as 0."""
        out = np.eye(4)
        for joint in self.chain(link, root):
            out = out @ joint.transform(float(joints.get(joint.name, 0.0)))
        return out

    def frame_transform(self, frame, joints):
        link, offset, rotation = NAMED_FRAMES[frame]
        return self.link_transform(link, joints) @ pose_matrix(offset, rotation)


@lru_cache(maxsize=1)
def model() -> G1Model:
    return G1Model()


def joint_limits(names=None):
    """URDF position limits ``{joint: (lower_rad, upper_rad)}`` (default: CANONICAL29)."""
    return model().limits(CANONICAL29 if names is None else names)


def as_joint_dict(joints, *, allow_hand=True):
    """Validate a joint mapping (or a 29-vector in CANONICAL29 order) into ``{name: float}``."""
    if joints is None:
        return {}
    if not isinstance(joints, dict):
        values = [float(v) for v in joints]
        if len(values) != len(CANONICAL29):
            raise ValueError("A joint vector must have 29 entries in CANONICAL29 order")
        joints = dict(zip(CANONICAL29, values))
    known = set(ALL_JOINTS if allow_hand else CANONICAL29)
    out = {}
    for name, value in joints.items():
        if name not in known:
            raise ValueError(f"Unknown joint name {name!r}; see services.humanoid.g1.CANONICAL29")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"Joint {name} is not finite: {value}")
        out[name] = value
    return out


# ----------------------------------------------------------------------------- sphere model (CPU)
@lru_cache(maxsize=1)
def collision_spheres():
    """``{link: (centers (n,3) in link frame, radii (n,))}`` of the shipped cuRobo sphere model."""
    import yaml
    data = yaml.safe_load(CUROBO_SPHERES_PATH.read_text())
    return {link: (np.array([s["center"] for s in spheres], dtype=float),
                   np.array([s["radius"] for s in spheres], dtype=float))
            for link, spheres in data["collision_spheres"].items()}


def cuboid_matrix(cuboid):
    """(pelvis_T_box 4x4, half extents) of ``{"dims": [...], "pose_wxyz": [...]}``."""
    pose = [float(v) for v in cuboid["pose_wxyz"]]
    return pose_matrix(pose[:3], pose[3:]), 0.5 * np.asarray(cuboid["dims"], dtype=float)


def obstacle_contacts(joints, obstacles, margin_m=0.0, base_T_frame=None):
    """Links whose spheres come within ``margin_m`` of (or penetrate) any cuboid obstacle.

    CPU sphere-vs-box test on the same sphere model the solvers use. Returns
    ``{link: deepest penetration (m), positive inside}``.
    """
    out = {}
    boxes = []
    for item in obstacles or []:
        transform, half = cuboid_matrix(item["cuboid"])
        if base_T_frame is not None:
            transform = base_T_frame @ transform
        boxes.append((np.linalg.inv(transform), half))
    if not boxes:
        return out
    for link, (centers, radii) in collision_spheres().items():
        transform = model().link_transform(link, joints)
        world = centers @ transform[:3, :3].T + transform[:3, 3]
        for box_T_world, half in boxes:
            local = world @ box_T_world[:3, :3].T + box_T_world[:3, 3]
            q = np.abs(local) - half
            outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
            inside = np.minimum(q.max(axis=1), 0.0)
            depth = radii + margin_m - (outside + inside)
            if depth.max() > 0.0:
                out[link] = max(out.get(link, 0.0), float(depth.max()))
    return out
