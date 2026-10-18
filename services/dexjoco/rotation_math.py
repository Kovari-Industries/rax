"""Rotation and pose arithmetic for DexJoCo action rows: the ``dexjoco_rotation_math`` tool.

Pure computation: no episode, simulator or scene state. Safe to import in the host harness
interpreter; validation is plain Python and scipy loads on the first computation.

Rotation vectors and quaternions are converted with scipy's ``Rotation`` exactly as the official
evaluation wrapper converts rows (dexjoco_openpi_client/dexjoco_openpi_env.py): a row's rotation
vector becomes the environment's target quaternion with
``Rotation.from_rotvec(v).as_quat(scalar_first=True)`` (``_process_action``), and a measured
quaternion becomes a hold row's rotation vector with
``Rotation.from_quat(q, scalar_first=True).as_rotvec()`` (``stay``). Every reported
``quaternion_wxyz`` is the official conversion of the reported ``rotation_vector``.

A pose is a world-frame position (metres) and an orientation. ``compose_pose`` is pose_a followed
by pose_b expressed in pose_a's frame (world_T_b = world_T_a * a_T_b); ``relative_pose`` is
pose_b expressed in pose_a's frame (a_T_b = world_T_a^-1 * world_T_b).
"""

from __future__ import annotations

import math

from services.simulator.io import reject_unless

TOOL = "dexjoco_rotation_math"
OPERATIONS = ("convert_rotation", "compose_pose", "relative_pose", "flange_target", "row_from_state")
REPRESENTATIONS = ("quaternion_wxyz", "rotation_vector", "matrix", "axis_angle")
# Official state vector -> hold row (DexJoCoOpenPIEnv.stay): the state widths it accepts.
STATE_DIMS = (23, 46)
ARGUMENTS = {
    "convert_rotation": ({"rotation"}, {"near_rotation_vector"}),
    "compose_pose": ({"pose_a", "pose_b"}, {"near_rotation_vector"}),
    "relative_pose": ({"pose_a", "pose_b"}, {"near_rotation_vector"}),
    "flange_target": ({"point_target_m", "point_offset_m", "rotation"}, {"near_rotation_vector"}),
    "row_from_state": ({"state"}, set()),
}


# ----------------------------------------------------------------- validation
def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, size, name):
    reject_unless(isinstance(value, list) and len(value) == size and all(_finite(v) for v in value),
                  f"{name} must be a list of {size} finite numbers")


def _rotation(value, name):
    reject_unless(isinstance(value, dict) and len(value) == 1 and set(value) <= set(REPRESENTATIONS),
                  f"{name} must have exactly one of {', '.join(REPRESENTATIONS)}")
    (kind, item), = value.items()
    if kind == "quaternion_wxyz":
        _vector(item, 4, f"{name}.quaternion_wxyz")
        reject_unless(math.hypot(*item) > 1e-9, f"{name}.quaternion_wxyz must be non-zero")
    elif kind == "rotation_vector":
        _vector(item, 3, f"{name}.rotation_vector")
    elif kind == "matrix":
        reject_unless(isinstance(item, list) and len(item) == 3, f"{name}.matrix must be 3 rows of 3 numbers")
        for row in item:
            _vector(row, 3, f"{name}.matrix row")
    else:
        reject_unless(isinstance(item, dict) and set(item) == {"axis", "angle_rad"},
                      f"{name}.axis_angle needs exactly axis and angle_rad")
        _vector(item["axis"], 3, f"{name}.axis_angle.axis")
        reject_unless(math.hypot(*item["axis"]) > 1e-9, f"{name}.axis_angle.axis must be non-zero")
        reject_unless(_finite(item["angle_rad"]), f"{name}.axis_angle.angle_rad must be a finite number")


def _pose(value, name):
    reject_unless(isinstance(value, dict) and value and set(value) <= {"position_m", "rotation"},
                  f"{name} must have position_m, rotation or both")
    if "position_m" in value:
        _vector(value["position_m"], 3, f"{name}.position_m")
    if "rotation" in value:
        _rotation(value["rotation"], f"{name}.rotation")


def validate(arguments):
    """Complete validation before computation; raises RejectedBeforeAction."""
    reject_unless(isinstance(arguments, dict), "arguments must be an object")
    operation = arguments.get("operation")
    reject_unless(operation in OPERATIONS, f"operation must be one of {', '.join(OPERATIONS)}")
    required, optional = ARGUMENTS[operation]
    present = set(arguments) - {"operation"}
    reject_unless(required <= present, f"{operation} needs {', '.join(sorted(required))}")
    reject_unless(present <= required | optional,
                  f"{operation} does not take {', '.join(sorted(present - required - optional))}")
    for key in present & {"rotation"}:
        _rotation(arguments[key], key)
    for key in present & {"pose_a", "pose_b"}:
        _pose(arguments[key], key)
    for key in present & {"near_rotation_vector", "point_target_m", "point_offset_m"}:
        _vector(arguments[key], 3, key)
    if "state" in present:
        state = arguments["state"]
        reject_unless(isinstance(state, list) and len(state) in STATE_DIMS and all(_finite(v) for v in state),
                      "state must be the official state vector: 23 (single arm) or 46 (dual arm) finite numbers")


# --------------------------------------------------------------- computation
def _from(value):
    import numpy as np
    from scipy.spatial.transform import Rotation

    (kind, item), = value.items()
    if kind == "quaternion_wxyz":
        return Rotation.from_quat(np.asarray(item, dtype=np.float64), scalar_first=True)
    if kind == "rotation_vector":
        return Rotation.from_rotvec(np.asarray(item, dtype=np.float64))
    if kind == "matrix":
        matrix = np.asarray(item, dtype=np.float64)
        reject_unless(np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-3) and np.linalg.det(matrix) > 0,
                      "matrix must be a rotation matrix (orthonormal rows, determinant +1)")
        return Rotation.from_matrix(matrix)
    axis = np.asarray(item["axis"], dtype=np.float64)
    return Rotation.from_rotvec(axis / np.linalg.norm(axis) * float(item["angle_rad"]))


def _nearest(rotvec, near):
    """The rotation vector of the same rotation closest to ``near``: angle t about n equals
    angle t - 2 pi about n, so consecutive rows need not jump across the pi boundary."""
    import numpy as np

    near = np.asarray(near, dtype=np.float64)
    angle = float(np.linalg.norm(rotvec))
    axis = rotvec / angle if angle > 1e-12 else (near / np.linalg.norm(near) if np.linalg.norm(near) > 1e-12
                                                  else np.zeros(3))
    candidates = [rotvec + k * 2 * np.pi * axis for k in (0, -1, 1)]
    return min(candidates, key=lambda v: float(np.linalg.norm(v - near)))


def _describe(rotation, near=None):
    import numpy as np
    from scipy.spatial.transform import Rotation

    rotvec = rotation.as_rotvec()
    if near is not None:
        rotvec = _nearest(rotvec, near)
    return {"quaternion_wxyz": Rotation.from_rotvec(rotvec).as_quat(scalar_first=True).tolist(),
            "rotation_vector": rotvec.tolist(), "angle_rad": float(np.linalg.norm(rotvec)),
            "matrix": rotation.as_matrix().tolist()}


def _pose_of(value):
    import numpy as np
    from scipy.spatial.transform import Rotation

    position = np.asarray(value.get("position_m", [0.0, 0.0, 0.0]), dtype=np.float64)
    rotation = _from(value["rotation"]) if "rotation" in value else Rotation.identity()
    return position, rotation


def _pose_reply(position, rotation, near=None):
    described = _describe(rotation, near)
    return {"position_m": position.tolist(), **described,
            "row_prefix": [*position.tolist(), *described["rotation_vector"]]}


def rotation_math(arguments):
    """The reply of one dexjoco_rotation_math call."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    validate(arguments)
    operation = arguments["operation"]
    near = arguments.get("near_rotation_vector")
    if operation == "convert_rotation":
        return {"operation": operation, "rotation": _describe(_from(arguments["rotation"]), near)}
    if operation in ("compose_pose", "relative_pose"):
        (pa, ra), (pb, rb) = _pose_of(arguments["pose_a"]), _pose_of(arguments["pose_b"])
        if operation == "compose_pose":
            position, rotation = pa + ra.apply(pb), ra * rb
        else:
            position, rotation = ra.inv().apply(pb - pa), ra.inv() * rb
        return {"operation": operation, "pose": _pose_reply(position, rotation, near)}
    if operation == "flange_target":
        rotation = _from(arguments["rotation"])
        offset = np.asarray(arguments["point_offset_m"], dtype=np.float64)
        position = np.asarray(arguments["point_target_m"], dtype=np.float64) - rotation.apply(offset)
        return {"operation": operation, "flange_pose": _pose_reply(position, rotation, near),
                "note": "row_prefix is the flange part of a row: position (m) then rotation vector (rad)"}
    state = np.asarray(arguments["state"], dtype=np.float64)

    def arm(pose):  # As DexJoCoOpenPIEnv.stay(): xyz, then the quaternion as a rotation vector.
        return [*pose[:3], *Rotation.from_quat(pose[3:7], scalar_first=True).as_rotvec()]

    if len(state) == 23:
        row = [*arm(state[:7]), *state[7:23]]
    else:
        row = [*arm(state[:7]), *state[14:30], *arm(state[7:14]), *state[30:46]]
    return {"operation": operation, "row": [float(v) for v in row], "row_dim": len(row),
            "note": "the row the official hold builds from this state: it targets the measured pose and joints"}
