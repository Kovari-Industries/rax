"""Robot-only BEHAVIOR motion preview inside OmniGibson (runs in runtime/envs/behavior).

The R1Pro of the official evaluation (``omnigibson/eval/r1pro.yaml``: controllers, base
mass, 30 Hz actions over 120 Hz physics) stands alone on a floor plane, watched by fixed
third-person cameras. Each run starts from the joint state the caller supplies, executes
native 23-D rows exactly as an episode would, and returns third-person frames and the
measured end state. There is no scene, object or task: nothing here reads a live episode.
"""

from __future__ import annotations

import math
import time

import numpy as np

from services.simulator.io import RejectedBeforeAction, reject_unless, require
from services.behavior import robot as R

CAMERA_SIZE = 480
SETTLE_STEPS = 10
# Fixed views (eye, look-at) in the start-relative robot frame: +x forward, +y left, metres.
# They frame about 2.3 m around the start pose: the robot fills most of the view and
# base moves of about half a metre stay in it.
CAMERAS = {
    "preview_front": ((1.7, 1.0, 1.3), (0.3, 0.0, 0.7)),
    "preview_side": ((0.3, -1.9, 0.95), (0.3, 0.0, 0.7)),
    "preview_top": ((0.3, 0.0, 2.6), (0.3, 0.0, 0.0)),
}


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Quaternion (xyzw) of a USD camera (looks along -Z, +Y up) at ``eye`` facing ``target``."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    forward = target - eye
    forward /= np.linalg.norm(forward)
    up = np.asarray(up, float)
    if abs(float(forward @ up)) > 0.99:  # Looking straight down: image up is robot forward.
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    m = np.stack([right, true_up, -forward], axis=1)
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(m).as_quat().tolist()


def environment_config(robot_cfg):
    sensors = [{
        "sensor_type": "VisionSensor", "name": name, "relative_prim_path": f"/{name}",
        "modalities": ["rgb"], "include_in_obs": False,
        "sensor_kwargs": {"image_height": CAMERA_SIZE, "image_width": CAMERA_SIZE},
        "position": list(eye), "orientation": look_at(eye, target), "pose_frame": "scene",
    } for name, (eye, target) in CAMERAS.items()]
    return {
        "env": {"action_frequency": R.CONTROL_HZ, "rendering_frequency": R.CONTROL_HZ,
                "physics_frequency": 120, "external_sensors": sensors},
        "scene": {"type": "Scene", "use_floor_plane": True, "floor_plane_visible": True,
                  "floor_plane_color": (0.2, 0.22, 0.25)},
        "robots": [robot_cfg],
        "task": {"type": "DummyTask"},
    }


def robot_config():
    """The evaluator's R1Pro config (``BatchedEvaluator._build_robot_config``), at the origin,
    with only proprioception: the preview shows third-person views, not the robot cameras."""
    from omegaconf import OmegaConf
    from omnigibson.eval.evaluator import DEFAULT_ROBOT_CONFIG_PATH, _to_plain_dict
    cfg = _to_plain_dict(OmegaConf.load(DEFAULT_ROBOT_CONFIG_PATH))
    cfg.pop("eval", None)
    cfg["model"] = cfg["model"].lower()
    cfg.update(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0], obs_modalities=["proprio"])
    return cfg


class PreviewSession:
    def __init__(self):
        import omnigibson as og
        import torch as th
        from omnigibson.eval.evaluator import EVAL_BASE_LINK_MASS
        from omnigibson.macros import gm
        from services.behavior.server import pin_render_gpu

        pin_render_gpu()
        gm.HEADLESS = True
        gm.RENDER_VIEWER_CAMERA = False
        self.og, self.th = og, th
        self.env = og.Environment(configs=environment_config(robot_config()))
        self.robot = self.env.scenes[0].robots[0]
        og.sim.stop()
        self.robot.base_footprint_link.mass = EVAL_BASE_LINK_MASS
        og.sim.play()
        self.env.reset()
        self.sensors = self.env._external_sensors[0]
        self.home = self.robot.get_joint_positions().clone()
        self.lower, self.upper = self._action_limits()
        self.terminated = self.truncated = False  # services.simulator.rpc.serve contract
        self.finish_reason = None
        self.metadata = {
            "benchmark": "behavior", "robot": self.robot.model, "control_frequency_hz": R.CONTROL_HZ,
            "frame_stride": R.FRAME_STRIDE, "max_action_rows": R.MAX_ACTION_ROWS,
            "action_lower": self.lower.tolist(), "action_upper": self.upper.tolist(),
            "cameras": {name: {"position_m": list(eye), "look_at_m": list(target), "width": CAMERA_SIZE,
                               "height": CAMERA_SIZE, "frame": "preview_start_robot_frame"}
                        for name, (eye, target) in CAMERAS.items()},
            "scene": "floor plane only", "task_environment_loaded": False,
        }

    def dof(self, name):
        from omnigibson.controllers.controller_view import ControllerView
        return ControllerView.get_dof_idx(self.robot.controllers[name][0]).cpu().numpy()

    def _action_limits(self):
        lower, upper = np.full(R.ACTION_DIM, -1.0), np.full(R.ACTION_DIM, 1.0)
        joint_lower = self.robot.joint_lower_limits.cpu().numpy()
        joint_upper = self.robot.joint_upper_limits.cpu().numpy()
        for name in ("trunk", "arm_left", "arm_right"):
            dof = self.dof(name)
            lower[R.ACTION_SLICES[name]] = joint_lower[dof]
            upper[R.ACTION_SLICES[name]] = joint_upper[dof]
        return lower, upper

    # ------------------------------------------------------------------ state
    def _proprio(self):
        from omnigibson.eval.utils.eval_utils import flatten_obs_dict
        obs, _ = self.env.get_obs()
        obs = obs[0] if isinstance(obs, (list, tuple)) else obs
        proprio = flatten_obs_dict(obs)[f"{self.robot.name}::proprio"]
        return np.asarray(proprio.detach().cpu().numpy() if hasattr(proprio, "detach") else proprio,
                          dtype=np.float64).reshape(-1)

    def _planar_pose(self):
        position, quaternion = self.robot.get_position_orientation()
        x, y, z, w = (float(v) for v in quaternion)
        return float(position[0]), float(position[1]), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _frame(self, step):
        images = {}
        for name, sensor in self.sensors.items():
            obs, _ = sensor.get_obs()
            rgb = obs["rgb"]
            rgb = rgb.detach().cpu().numpy() if hasattr(rgb, "detach") else np.asarray(rgb)
            images[name] = np.ascontiguousarray(rgb[..., :3]).astype(np.uint8)
        return {"step": int(step), "images": images}

    def _set_state(self, state):
        th = self.th
        q = self.home.clone()
        for name in ("trunk", "arm_left", "arm_right"):
            q[th.as_tensor(self.dof(name))] = th.as_tensor(state[name], dtype=q.dtype)
        lower, upper = self.robot.joint_lower_limits, self.robot.joint_upper_limits
        for index, side in enumerate(("left", "right")):
            dof = th.as_tensor(self.dof(f"gripper_{side}"))
            fingers = state.get(f"gripper_{side}_qpos")
            if fingers is None:  # Opening implied by the command (-1 closed, +1 open).
                fingers = lower[dof] + (state["grippers"][index] + 1.0) / 2.0 * (upper[dof] - lower[dof])
            q[dof] = th.as_tensor(fingers, dtype=q.dtype)
        self.robot.set_position_orientation(position=th.zeros(3), orientation=th.tensor([0.0, 0.0, 0.0, 1.0]))
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(th.zeros_like(q))
        self.robot.keep_still()
        hold = np.zeros(R.ACTION_DIM)
        for name in ("trunk", "arm_left", "arm_right"):
            hold[R.ACTION_SLICES[name]] = state[name]
        hold[R.ACTION_SLICES["gripper_left"]] = state["grippers"][0]
        hold[R.ACTION_SLICES["gripper_right"]] = state["grippers"][1]
        for _ in range(SETTLE_STEPS):
            self.env.step(th.as_tensor(hold, dtype=th.float32).reshape(1, -1))
        return hold

    # ------------------------------------------------------------------- run
    def run(self, initial_state, actions, measured=False):
        """``measured``: the start state is the live episode's measured joints, not caller input,
        so joint drift past a limit is clamped and reported like any measured hold."""
        began = time.monotonic()
        corrections = {}
        rows = R.validate_actions(actions, self.lower, self.upper, diagnostics=corrections)
        state = {name: np.asarray(initial_state[name], dtype=np.float64) for name in ("trunk", "arm_left", "arm_right")}
        for name in ("gripper_left_qpos", "gripper_right_qpos"):  # Measured fingers, when known.
            if initial_state.get(name) is not None:
                state[name] = np.asarray(initial_state[name], dtype=np.float64)
        state["grippers"] = [float(v) for v in initial_state["grippers"]]
        for name in ("trunk", "arm_left", "arm_right"):
            s = R.ACTION_SLICES[name]
            reject_unless(state[name].shape == (s.stop - s.start,), f"initial_state.{name} has the wrong length")
        for side in ("left", "right"):
            if f"gripper_{side}_qpos" in state:
                reject_unless(state[f"gripper_{side}_qpos"].shape == (2,), f"gripper_{side}_qpos needs 2 values")
        reject_unless(len(state["grippers"]) == 2 and all(abs(g) <= 1 for g in state["grippers"]),
                      "initial_state.grippers must be two commands in [-1, 1]")
        initial_corrections = {}
        joints = {f"{name}_qpos": state[name] for name in ("trunk", "arm_left", "arm_right")}
        if measured:
            initial_row = R.bounded_hold_action(joints, state["grippers"], self.lower, self.upper,
                                                diagnostics=initial_corrections)
        else:
            try:
                initial_row = R.validate_actions([R.hold_action(joints, state["grippers"])], self.lower, self.upper,
                                                 diagnostics=initial_corrections)[0]
            except RejectedBeforeAction as exc:
                raise RejectedBeforeAction(f"initial_state: {exc}") from exc
        for name in ("trunk", "arm_left", "arm_right"):
            state[name] = initial_row[R.ACTION_SLICES[name]]
        self._set_state(state)
        origin = self._planar_pose()
        start = R.decode_proprio(self._proprio())
        frames, distance, previous = [self._frame(0)], 0.0, origin
        for index, row in enumerate(rows, 1):
            self.env.step(self.th.as_tensor(row, dtype=self.th.float32).reshape(1, -1))
            pose = self._planar_pose()
            distance += math.hypot(pose[0] - previous[0], pose[1] - previous[1])
            previous = pose
            if index % R.FRAME_STRIDE == 0 or index == len(rows):
                frames.append(self._frame(index))
        end = R.decode_proprio(self._proprio())
        # The base pose itself, not integrated base_qvel: on the bare floor plane the
        # reported base joint velocity drifts while the base stands still.
        dx, dy = previous[0] - origin[0], previous[1] - origin[1]
        c, s = math.cos(origin[2]), math.sin(origin[2])
        base_motion = {"x_m": c * dx + s * dy, "y_m": -s * dx + c * dy,
                       "yaw_rad": math.remainder(previous[2] - origin[2], math.tau),
                       "frame": "preview_start_robot_frame", "distance_travelled_m": distance,
                       "source": "preview simulator base pose"}
        return {"status": "Success", "rows": int(len(rows)), "frames": frames,
                "initial_state": {name: np.asarray(state[name]).tolist()
                                  for name in ("trunk", "arm_left", "arm_right", "grippers")},
                **({"joint_target_clamping": corrections} if corrections else {}),
                **({"initial_state_joint_clamping": initial_corrections} if initial_corrections else {}),
                "start_proprio": start, "end_proprio": end, "base_motion": base_motion,
                "seconds": round(time.monotonic() - began, 2)}

    def dispatch(self, op, args):
        if op == "metadata":
            return self.metadata
        if op == "run":
            return self.run(args["initial_state"], args["actions"], measured=bool(args.get("measured", False)))
        raise RejectedBeforeAction(f"Unknown preview op: {op}")

    def _write_summary(self, reason):  # services.simulator.rpc.serve contract; nothing to record.
        self.finish_reason = reason
