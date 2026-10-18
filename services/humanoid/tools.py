"""HumanoidArena MCP tool schemas and host-side request validation.

Pure Python + numpy: safe to import in the host harness interpreter. Validation runs before any RPC
so a malformed request never touches the simulator.
"""

from __future__ import annotations

import math

import numpy as np

from services.simulator.io import RejectedBeforeAction, reject_unless
from services.humanoid import robot as R
from services.benchmarks import HUMANOID_TOOLS

PREFIX = "humanoid_"
STATUS, OBSERVE, STEP = "humanoid_status", "humanoid_observe", "humanoid_step"
FREE_SPACE_MOVE, EXECUTE_PLAN = "humanoid_free_space_move", "humanoid_execute_motion_plan"
PIXEL_TO_POSITION = "humanoid_pixel_to_position"

ROBOT_TOOLS = frozenset({STATUS, OBSERVE, STEP, FREE_SPACE_MOVE, EXECUTE_PLAN, PIXEL_TO_POSITION})
assert ROBOT_TOOLS == HUMANOID_TOOLS, "services/benchmarks.py and the HumanoidArena tool set disagree"
GEOMETRY_TOOLS = frozenset({PIXEL_TO_POSITION})
MUTATING = frozenset({STEP, FREE_SPACE_MOVE, EXECUTE_PLAN})
PREVIEWABLE = frozenset({FREE_SPACE_MOVE})
# Geometry-only fields removed by the RGB-only profile.
GEOMETRY_KEYS = frozenset({"cam_front_depth_m", "camera_intrinsics", "camera_poses"})
# Body frames a target may be given in (both rigidly attached to the robot; no world frame exists).
BODY_FRAMES = ("pelvis", "torso")

_number = {"type": "number"}
_vector = lambda n: {"type": "array", "items": _number, "minItems": n, "maxItems": n}  # noqa: E731


def tool_definitions():
    layout = ", ".join(f"[{s.start}:{s.stop}] {name}" for name, s in R.ACTION_SLICES.items())
    return [
        {"name": STATUS, "description": (
            "Episode status, step budget, control mode, the semantic40 action layout, joint names and limits. "
            "No physics."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": OBSERVE, "description": (
            "Current front-camera RGB/depth attachment, calibration relative to the pelvis, measured root "
            "orientation, the 29 joint positions and velocities in canonical order, and the Dex3 finger state. "
            "No world position and no object state. No physics; the observation is cached after the last motion."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": STEP, "description": (
            f"Execute 1..{R.MAX_ACTION_ROWS} semantic40 reference frames, one per {R.CONTROL_HZ} Hz tick "
            f"(at most {R.MAX_ACTION_ROWS / R.CONTROL_HZ:g} s): {layout}. The frozen SONIC whole-body controller "
            "tracks only the 29 joint angles (canonical order, radians), their tick-to-tick change and the root "
            "rotation [3:9] (first two rotation-matrix columns, row-major, relative to the episode's initial heading; "
            "identity [1,0,0,1,0,0]). Root XY [0:2] and height [2] are required by the layout but do not move the "
            "robot: it translates only by stepping. Hands [38:40] are 0 (open) or 1 (closed). References are not "
            "motor targets: read the measured feedback. A fall or task completion ends the episode; frames past "
            "the remaining step budget are cut, not refused. Returns the final frame and saves every "
            f"{R.FRAME_STRIDE}th frame."),
         "inputSchema": {"type": "object", "required": ["actions"], "additionalProperties": False, "properties": {
             "actions": {"type": "array", "minItems": 1, "maxItems": R.MAX_ACTION_ROWS, "items": _vector(R.ACTION_DIM)}}}},
        {"name": FREE_SPACE_MOVE, "description": (
            "cuRobo motion plan of one arm (use_waist=true: the three waist joints too) to a hand pose in a body "
            "frame (pelvis or torso; metres, quaternion wxyz, or position only when the quaternion is omitted). "
            "end_effector selects the palm, the hand centre or the wrist. Avoids self-collision only: scene objects are "
            "unknown to the planner. Executes the plan as 50 Hz semantic40 frames with the other joints, the hands "
            "and the root rotation held at their last commanded values; legs do not step. preview_only=true "
            "returns the plan and a motion_plan_id without moving."),
         "inputSchema": {"type": "object", "required": ["arm", "position"], "additionalProperties": False,
             "properties": {
                 "arm": {"type": "string", "enum": ["left", "right"]},
                 "position": _vector(3), "quaternion_wxyz": _vector(4),
                 "frame": {"type": "string", "enum": list(BODY_FRAMES), "default": "pelvis"},
                 "end_effector": {"type": "string", "enum": ["palm", "wrist", "hand_center"], "default": "palm"},
                 "use_waist": {"type": "boolean", "default": False},
                 "time_scale": {"type": "number", "minimum": 1.0, "maximum": 4.0, "default": 1.0},
                 "preview_only": {"type": "boolean", "default": False}}}},
        {"name": EXECUTE_PLAN, "description": "Execute a previewed motion plan if the robot has not moved since planning.",
         "inputSchema": {"type": "object", "required": ["motion_plan_id"], "additionalProperties": False,
             "properties": {"motion_plan_id": {"type": "string", "minLength": 1, "maxLength": 64}}}},
        {"name": PIXEL_TO_POSITION, "description": (
            "Convert front-camera depth pixels of the current observation into 3-D points in the pelvis frame and in "
            "the level frame (origin at the pelvis, z up, x along the measured heading). No world position. Pass "
            "the observation's step_id; stale selections are rejected."),
         "inputSchema": {"type": "object", "required": ["observation_step_id", "selections"], "additionalProperties": False,
             "properties": {
                 "observation_step_id": {"type": "integer", "minimum": 0},
                 "selections": {"type": "array", "minItems": 1, "maxItems": 32, "items": {
                     "type": "object", "required": ["u", "v"], "additionalProperties": False, "properties": {
                         "u": {"type": "number"}, "v": {"type": "number"}}}},
                 "depth_window_radius": {"type": "integer", "minimum": 0, "maximum": 5, "default": 1}}}},
    ]


def _finite(value, name):
    reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                  f"{name} must be a finite number")


def _finite_vector(value, n, name):
    reject_unless(isinstance(value, list) and len(value) == n, f"{name} must be a list of {n} numbers")
    for item in value:
        _finite(item, name)


def _only(arguments, allowed):
    unknown = set(arguments) - set(allowed)
    reject_unless(not unknown, f"Unknown arguments: {sorted(unknown)}")


def validate_request(name, arguments):
    """Complete schema/shape validation before dispatch; raises RejectedBeforeAction."""
    reject_unless(isinstance(arguments, dict), "arguments must be an object")
    if name in (STATUS, OBSERVE):
        _only(arguments, ())
    elif name == STEP:
        _only(arguments, ("actions",))
        R.validate_actions(arguments.get("actions"))
    elif name == FREE_SPACE_MOVE:
        _only(arguments, ("arm", "position", "quaternion_wxyz", "frame", "end_effector", "use_waist",
                          "time_scale", "preview_only"))
        reject_unless(arguments.get("arm") in ("left", "right"), "arm must be left or right")
        _finite_vector(arguments.get("position"), 3, "position")
        if "quaternion_wxyz" in arguments:
            _finite_vector(arguments["quaternion_wxyz"], 4, "quaternion_wxyz")
            norm = math.sqrt(sum(float(v) ** 2 for v in arguments["quaternion_wxyz"]))
            reject_unless(abs(norm - 1.0) < 1e-3, "quaternion_wxyz must be unit length")
        reject_unless(arguments.get("frame", "pelvis") in BODY_FRAMES, f"frame must be one of {list(BODY_FRAMES)}")
        reject_unless(arguments.get("end_effector", "palm") in ("palm", "wrist", "hand_center"),
                      "end_effector must be palm, wrist or hand_center")
        for key in ("use_waist", "preview_only"):
            reject_unless(type(arguments.get(key, False)) is bool, f"{key} must be a boolean")
        scale = arguments.get("time_scale", 1.0)
        _finite(scale, "time_scale")
        reject_unless(1.0 <= scale <= 4.0, "time_scale must be in [1, 4]")
    elif name == EXECUTE_PLAN:
        _only(arguments, ("motion_plan_id",))
        plan = arguments.get("motion_plan_id")
        reject_unless(isinstance(plan, str) and 0 < len(plan) <= 64, "motion_plan_id must be a string")
    elif name == PIXEL_TO_POSITION:
        _only(arguments, ("observation_step_id", "selections", "depth_window_radius"))
        reject_unless(type(arguments.get("observation_step_id")) is int, "observation_step_id must be an integer")
        selections = arguments.get("selections")
        reject_unless(isinstance(selections, list) and 1 <= len(selections) <= 32, "selections must have 1..32 items")
        for item in selections:
            reject_unless(isinstance(item, dict) and set(item) == {"u", "v"}, "each selection needs exactly u and v")
            _finite_vector([item["u"], item["v"]], 2, "pixel")
        radius = arguments.get("depth_window_radius", 1)
        reject_unless(type(radius) is int and 0 <= radius <= 5, "depth_window_radius must be 0..5")
    else:
        raise RejectedBeforeAction(f"Unknown HumanoidArena tool: {name}")


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v - x*i - y*j - z*k, w*i + x*v + y*k - z*j, w*j - x*k + y*v + z*i, w*k + x*j - y*i + z*v])


def _rotate(q, p):
    conjugate = np.asarray(q, dtype=np.float64) * [1, -1, -1, -1]
    return _multiply(_multiply(q, np.r_[0.0, p]), conjugate)[1:]


def level_from_pelvis(point, root_orientation_wxyz):
    """Pelvis-frame point -> level frame (origin at the pelvis, z up, x along the measured heading)."""
    q = np.asarray(root_orientation_wxyz, dtype=np.float64)
    w, x, y, z = q / np.linalg.norm(q)
    heading = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    world_axes = _rotate((w, x, y, z), np.asarray(point, dtype=np.float64))
    return _rotate((math.cos(heading / 2), 0.0, 0.0, -math.sin(heading / 2)), world_axes)


def pixel_to_pelvis(depth_m, intrinsics, pose, u, v, radius=1):
    """Back-project one front-camera pixel into the pelvis frame.

    Depth is distance along the optical axis; the camera pose uses Isaac Lab's world convention
    (X forward along the optical axis, Y left, Z up). Uses the median valid depth in a
    (2r+1)^2 window.
    """
    depth = np.asarray(depth_m, dtype=np.float64)
    height, width = depth.shape[:2]
    reject_unless(0 <= u < width and 0 <= v < height, f"pixel ({u}, {v}) outside {width}x{height}")
    iu, iv = int(round(u)), int(round(v))
    window = depth[max(0, iv - radius):iv + radius + 1, max(0, iu - radius):iu + radius + 1]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size == 0:
        return {"valid": False, "reason": "no valid depth near the pixel"}
    distance = float(np.median(valid))
    K = np.asarray(intrinsics["matrix"], dtype=np.float64)
    x_cv = (u - K[0, 2]) / K[0, 0] * distance
    y_cv = (v - K[1, 2]) / K[1, 1] * distance
    point_camera = np.array([distance, -x_cv, -y_cv])
    point = _rotate(pose["quaternion_wxyz"], point_camera) + np.asarray(pose["position_m"], dtype=np.float64)
    return {"valid": True, "position_pelvis_m": point.tolist(), "depth_m": distance,
            "depth_spread_m": float(valid.max() - valid.min())}
