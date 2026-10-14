"""BEHAVIOR MCP tool schemas, host-side request validation and depth-pixel geometry.

Pure Python + numpy: safe to import in the host harness interpreter. Validation runs before
any RPC so a malformed request never touches the simulator.
"""

from __future__ import annotations

import math

import numpy as np

from services.simulator.io import RejectedBeforeAction, reject_unless
from services.behavior import robot as R
from services.benchmarks import BEHAVIOR_TOOLS

STATUS, OBSERVE = "behavior_status", "behavior_observe"
STEP, BASE_MOVE, STEP_EEF = "behavior_step", "behavior_base_move", "behavior_step_eef"
FREE_SPACE_MOVE, EXECUTE_PLAN = "behavior_free_space_move", "behavior_execute_motion_plan"
PIXEL_TO_POSITION = "behavior_pixel_to_position"
PREVIEW_DESCRIBE, PREVIEW_RUN = "behavior_preview_describe", "behavior_preview_run"
PREVIEW_CAMERAS = ("preview_front", "preview_side", "preview_top")
# Robot-only preview: never advances the episode or counts against its step limit.
PREVIEW_TOOLS = frozenset({PREVIEW_DESCRIBE, PREVIEW_RUN})
INITIAL_STATE_FIELDS = {"trunk": 4, "arm_left": 7, "arm_right": 7, "grippers": 2}

ROBOT_TOOLS = frozenset({STATUS, OBSERVE, STEP, BASE_MOVE, STEP_EEF, FREE_SPACE_MOVE, EXECUTE_PLAN, PIXEL_TO_POSITION,
                         *PREVIEW_TOOLS})
assert ROBOT_TOOLS == BEHAVIOR_TOOLS, "services/benchmarks.py and the BEHAVIOR tool set disagree"

_number = {"type": "number"}
_vector = lambda n: {"type": "array", "items": _number, "minItems": n, "maxItems": n}  # noqa: E731


def tool_definitions():
    layout = ", ".join(f"[{s.start}:{s.stop}] {name}" for name, s in R.ACTION_SLICES.items())
    return [
        {"name": STATUS, "description": "Episode status, step budget and action/observation contract. No physics.",
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": OBSERVE, "description": (
            "Current head and wrist RGB/depth attachments, calibration, named 61-D R1Pro proprioception and "
            "start-relative odometry. No physics; the observation is cached after the last motion."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": STEP, "description": (
            f"Execute 1..{R.MAX_ACTION_ROWS} rows of the native 23-D R1Pro action at {R.CONTROL_HZ} Hz: {layout}. "
            "Base is a normalized robot-frame velocity [vx, vy, wz] in [-1, 1] (+-0.75 m/s, +-1 rad/s); trunk and "
            "arms are absolute joint radians within joint limits; grippers are normalized [-1, 1]. Returns the final "
            f"frame and saves every {R.FRAME_STRIDE}th frame; each row is one native step against the episode limit."),
         "inputSchema": {"type": "object", "required": ["actions"], "additionalProperties": False, "properties": {
             "actions": {"type": "array", "minItems": 1, "maxItems": R.MAX_ACTION_ROWS, "items": _vector(R.ACTION_DIM)}}}},
        {"name": BASE_MOVE, "description": (
            "Closed-loop planar base move relative to the current pose (robot frame: +x forward, +y left, yaw CCW), "
            "driven only by integrated proprioceptive odometry. Arms and trunk hold their measured joints. No map "
            "or obstacle knowledge: check the cameras first. Reports reached/stalled and the executed native rows."),
         "inputSchema": {"type": "object", "required": ["dx", "dy", "dyaw"], "additionalProperties": False, "properties": {
             "dx": {"type": "number", "minimum": -5, "maximum": 5}, "dy": {"type": "number", "minimum": -5, "maximum": 5},
             "dyaw": {"type": "number", "minimum": -math.pi, "maximum": math.pi},
             "max_linear": {"type": "number", "minimum": 0.05, "maximum": 0.75, "default": 0.5},
             "max_angular": {"type": "number", "minimum": 0.05, "maximum": 1.0, "default": 0.8},
             "position_tolerance_m": {"type": "number", "minimum": 0.005, "maximum": 0.5, "default": 0.03},
             "yaw_tolerance_rad": {"type": "number", "minimum": 0.005, "maximum": 0.5, "default": 0.03},
             "max_steps": {"type": "integer", "minimum": 1, "maximum": 4 * R.MAX_ACTION_ROWS, "default": 600}}}},
        {"name": STEP_EEF, "description": (
            "Move one end effector toward a pose in the robot base frame (metres, quaternion xyzw) by closed-loop "
            "inverse kinematics of that arm only; the trunk, base and other arm hold still. Local and not "
            "collision-aware: use it for approaches, contact and small corrections. Reports reached and the final error."),
         "inputSchema": {"type": "object", "required": ["arm", "position", "quaternion_xyzw"], "additionalProperties": False,
             "properties": {
                 "arm": {"type": "string", "enum": ["left", "right"]},
                 "position": _vector(3), "quaternion_xyzw": _vector(4),
                 "max_steps": {"type": "integer", "minimum": 1, "maximum": 4 * R.MAX_ACTION_ROWS, "default": 150},
                 "position_tolerance_m": {"type": "number", "minimum": 0.001, "maximum": 0.1, "default": 0.005},
                 "orientation_tolerance_rad": {"type": "number", "minimum": 0.005, "maximum": 0.5, "default": 0.03},
                 "max_joint_step_rad": {"type": "number", "minimum": 0.005, "maximum": 0.2, "default": 0.05}}}},
        {"name": FREE_SPACE_MOVE, "description": (
            "cuRobo motion plan of one arm (use_trunk=true: the trunk too) to an end-effector pose in the robot base "
            "frame (metres, quaternion xyzw); the other hand holds its pose. Avoids self-collision only: scene "
            "objects are unknown to the planner. With the trunk, reaches go farther but the trunk may move "
            "substantially. preview_only=true returns the plan and a motion_plan_id without moving; only the latest "
            "such plan is kept, so a new one replaces it."),
         "inputSchema": {"type": "object", "required": ["arm", "position", "quaternion_xyzw"], "additionalProperties": False,
             "properties": {
                 "arm": {"type": "string", "enum": ["left", "right"]},
                 "position": _vector(3), "quaternion_xyzw": _vector(4),
                 "use_trunk": {"type": "boolean", "default": False},
                 "preview_only": {"type": "boolean", "default": False}}}},
        {"name": EXECUTE_PLAN, "description": "Execute a previewed motion plan if the robot has not moved since planning.",
         "inputSchema": {"type": "object", "required": ["motion_plan_id"], "additionalProperties": False,
             "properties": {"motion_plan_id": {"type": "string", "minLength": 1, "maxLength": 64}}}},
        {"name": PIXEL_TO_POSITION, "description": (
            "Convert depth pixels from the current observation into 3-D points in the robot base frame. "
            "Pass the observation's step_id; stale selections are rejected."),
         "inputSchema": {"type": "object", "required": ["observation_step_id", "selections"], "additionalProperties": False,
             "properties": {
                 "observation_step_id": {"type": "integer", "minimum": 0},
                 "selections": {"type": "array", "minItems": 1, "maxItems": 32, "items": {
                     "type": "object", "required": ["camera", "u", "v"], "additionalProperties": False, "properties": {
                         "camera": {"type": "string", "enum": list(R.CAMERAS)},
                         "u": {"type": "number"}, "v": {"type": "number"}}}},
                 "depth_window_radius": {"type": "integer", "minimum": 0, "maximum": 5, "default": 1}}}},
        {"name": PREVIEW_DESCRIBE, "description": (
            "Contract of the robot-only motion preview: cameras, inputs, outputs and limits. Starts nothing."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": PREVIEW_RUN, "description": (
            "Dry-run R1Pro motion in a robot-only simulator (same robot, controllers and 30 Hz rows; a floor and "
            "nothing else) and watch it from fixed third-person cameras. Pass either actions (behavior_step rows) or "
            "the motion_plan_id of a behavior_free_space_move preview_only plan. The start state defaults to the "
            "current episode's measured joints; initial_state overrides it. The base starts at the preview origin. "
            "Returns the measured end state and base motion, saves every "
            f"{R.FRAME_STRIDE}th frame and attaches the final frames. Never moves the episode or spends its steps."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "actions": {"type": "array", "minItems": 1, "maxItems": R.MAX_ACTION_ROWS, "items": _vector(R.ACTION_DIM)},
             "motion_plan_id": {"type": "string", "minLength": 1, "maxLength": 64},
             "initial_state": {"type": "object", "additionalProperties": False,
                               "required": list(INITIAL_STATE_FIELDS),
                               "description": "Joint radians (trunk, arms) and gripper commands in [-1, 1].",
                               "properties": {k: _vector(n) for k, n in INITIAL_STATE_FIELDS.items()}}}}},
    ]


def _finite_vector(value, n, name):
    reject_unless(isinstance(value, list) and len(value) == n, f"{name} must be a list of {n} numbers")
    for item in value:
        reject_unless(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item),
                      f"{name} must contain finite numbers")


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
    elif name == BASE_MOVE:
        _only(arguments, ("dx", "dy", "dyaw", "max_linear", "max_angular", "position_tolerance_m",
                          "yaw_tolerance_rad", "max_steps"))
        for key in ("dx", "dy", "dyaw"):
            reject_unless(key in arguments, f"{key} is required")
        for key, value in arguments.items():
            if key == "max_steps":
                reject_unless(type(value) is int and 1 <= value <= 4 * R.MAX_ACTION_ROWS, "invalid max_steps")
            else:
                reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                              f"{key} must be a finite number")
    elif name in (FREE_SPACE_MOVE, STEP_EEF):
        options = (("use_trunk", "preview_only") if name == FREE_SPACE_MOVE else
                   ("max_steps", "position_tolerance_m", "orientation_tolerance_rad", "max_joint_step_rad"))
        _only(arguments, ("arm", "position", "quaternion_xyzw", *options))
        reject_unless(arguments.get("arm") in ("left", "right"), "arm must be left or right")
        _finite_vector(arguments.get("position"), 3, "position")
        _finite_vector(arguments.get("quaternion_xyzw"), 4, "quaternion_xyzw")
        norm = float(np.linalg.norm(arguments["quaternion_xyzw"]))
        reject_unless(abs(norm - 1.0) < 1e-3, "quaternion_xyzw must be unit length")
        if name == FREE_SPACE_MOVE:
            for key in options:
                reject_unless(type(arguments.get(key, False)) is bool, f"{key} must be a boolean")
        else:
            reject_unless(type(arguments.get("max_steps", 150)) is int, "max_steps must be an integer")
            for key in options[1:]:
                value = arguments.get(key, 0.01)
                reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                              f"{key} must be a finite number")
    elif name == EXECUTE_PLAN:
        _only(arguments, ("motion_plan_id",))
        plan = arguments.get("motion_plan_id")
        reject_unless(isinstance(plan, str) and 0 < len(plan) <= 64, "motion_plan_id must be a string")
    elif name == PREVIEW_DESCRIBE:
        _only(arguments, ())
    elif name == PREVIEW_RUN:
        _only(arguments, ("actions", "motion_plan_id", "initial_state"))
        reject_unless(("actions" in arguments) != ("motion_plan_id" in arguments),
                      "Pass exactly one of actions or motion_plan_id")
        if "actions" in arguments:
            R.validate_actions(arguments["actions"])
        else:
            plan = arguments["motion_plan_id"]
            reject_unless(isinstance(plan, str) and 0 < len(plan) <= 64, "motion_plan_id must be a string")
        if "initial_state" in arguments:
            state = arguments["initial_state"]
            reject_unless(isinstance(state, dict) and set(state) == set(INITIAL_STATE_FIELDS),
                          f"initial_state needs exactly {sorted(INITIAL_STATE_FIELDS)}")
            for key, n in INITIAL_STATE_FIELDS.items():
                _finite_vector(state[key], n, f"initial_state.{key}")
            reject_unless(all(abs(g) <= 1 for g in state["grippers"]), "initial_state.grippers must be in [-1, 1]")
    elif name == PIXEL_TO_POSITION:
        _only(arguments, ("observation_step_id", "selections", "depth_window_radius"))
        reject_unless(type(arguments.get("observation_step_id")) is int, "observation_step_id must be an integer")
        selections = arguments.get("selections")
        reject_unless(isinstance(selections, list) and 1 <= len(selections) <= 32, "selections must have 1..32 items")
        for item in selections:
            reject_unless(isinstance(item, dict) and set(item) == {"camera", "u", "v"}, "selection needs camera, u, v")
            reject_unless(item["camera"] in R.CAMERAS, "unknown camera")
            _finite_vector([item["u"], item["v"]], 2, "pixel")
        radius = arguments.get("depth_window_radius", 1)
        reject_unless(type(radius) is int and 0 <= radius <= 5, "depth_window_radius must be 0..5")
    else:
        raise RejectedBeforeAction(f"Unknown BEHAVIOR tool: {name}")


def quaternion_matrix(q_xyzw):
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pixel_to_base(depth_m, intrinsics, pose, u, v, radius=1):
    """Back-project one pixel of a linear-depth image into the robot base frame.

    OmniGibson cameras follow the USD convention (looking down -Z, +Y up); pixel rays are
    formed in the OpenCV convention and converted. ``depth_linear`` is distance along the
    optical axis. Uses the median valid depth in a (2r+1)^2 window.
    """
    depth = np.asarray(depth_m, dtype=np.float64)
    height, width = depth.shape[:2]
    reject_unless(0 <= u < width and 0 <= v < height, f"pixel ({u}, {v}) outside {width}x{height}")
    iu, iv = int(round(u)), int(round(v))
    window = depth[max(0, iv - radius):iv + radius + 1, max(0, iu - radius):iu + radius + 1]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size == 0:
        return {"valid": False, "reason": "no valid depth near the pixel"}
    z = float(np.median(valid))
    K = np.asarray(intrinsics["matrix"], dtype=np.float64)
    x_cv = (u - K[0, 2]) / K[0, 0] * z
    y_cv = (v - K[1, 2]) / K[1, 1] * z
    point_camera = np.array([x_cv, -y_cv, -z])
    rotation = quaternion_matrix(pose["quaternion_xyzw"])
    point = rotation @ point_camera + np.asarray(pose["position_m"], dtype=np.float64)
    return {"valid": True, "position_m": point.tolist(), "depth_m": z, "frame": "robot_base",
            "depth_spread_m": float(valid.max() - valid.min())}


def base_to_pixel(point, intrinsics, pose):
    """Project a robot-base-frame point into a camera (inverse of pixel_to_base)."""
    rotation = quaternion_matrix(pose["quaternion_xyzw"])
    local = rotation.T @ (np.asarray(point, dtype=np.float64) - np.asarray(pose["position_m"], dtype=np.float64))
    x_cv, y_cv, z = local[0], -local[1], -local[2]
    if z <= 0:
        return None
    K = np.asarray(intrinsics["matrix"], dtype=np.float64)
    return float(K[0, 0] * x_cv / z + K[0, 2]), float(K[1, 1] * y_cv / z + K[1, 2]), float(z)
