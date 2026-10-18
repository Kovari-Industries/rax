"""Robot-only DexJoCo motion preview inside MuJoCo (runs in runtime/envs/dexjoco).

The task's own official robot (the Panda + Allegro MJCF that the task's official arena includes:
one arm, or the bimanual pair) stands alone on a floor plane, watched by fixed third-person
cameras. No arena, table, object, layout, seed or episode state is loaded or read.

Control is the official one. Each row is converted by the official code: the evaluation
wrapper's rotation-vector conversion (``DexJoCoOpenPIEnv._process_action``) and the policy
wrapper's split into the environment action (``SingleArmPolicyWrapper`` /
``DualArmPolicyWrapper``). The environment's step loop is then replayed, as in every official
``panda_*_env.py`` ``step()`` (e.g. ``panda_click_mouse_env.py`` lines 565-590,
``panda_bimanual_hanoi_env.py`` lines 613-645): ``n_substeps`` times [the official ``opspace()``
per arm -> arm torques, the hand joint targets -> the Allegro position actuators,
``mj_step``]. The keyword arguments of the ``opspace()`` calls (gains, damping, null-space
posture, gravity compensation) are read from the task env's own ``step()`` source rather than
copied; ``n_substeps``, the physics timestep and the home posture come from the env class
(``mujoco_gym_env.py`` lines 34-36, ``_PANDA_HOME``). Without objects the replay is the official
free-space dynamics; the environment's wall-clock pacing (``time.sleep``) is not replayed.

Start state. The official state vector has flange poses and hand joints but no arm joint
angles, so each run rebuilds its start: arm joints at the env's home posture (as ``reset()``
sets them), hand joints held at the start state's values, then the same per-step control drives
each flange from its home pose to the start pose: a ramp, then holds until the measured flange
is within ``SETTLE_POSITION_M`` / ``SETTLE_ROTATION_RAD`` of the start pose and the arm is
still. The official controller holds a pose with a small steady offset (about 0.3 mm and 6 mrad
were measured), so once the arm is nearly still the target is corrected by a fraction of the
remaining error each step until the measured pose, not the target, matches. The elbow
posture this yields follows the controller's null-space pull toward the home posture; after
motion the live arm's elbow sits differently (tenths of a radian per joint were seen), which
shifts the flange path slightly and matters near joint limits or elbow contact. The arm starts
at rest. The settle phase is reported (``settle``) and is not part of the frames.
"""

from __future__ import annotations

import ast
import inspect
import math
import sys
import textwrap
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from services.dexjoco import tasks as T
from services.dexjoco.tools import FRAME_STRIDE, MAX_ACTION_ROWS, PREVIEW_CAMERAS
from services.simulator.io import RejectedBeforeAction, reject_unless, require

CAMERA_SIZE = 480
CAMERA_FOVY = 45.0
# Fixed views (eye, look-at) in the official world frame (+x across the table, +z up), around the
# arm bases at x = -0.8 m and the home flange at x = -0.44 m, z = 1.52 m.
CAMERAS = {
    "single": {"preview_front": ((0.75, 0.95, 1.75), (-0.4, 0.0, 1.15)),
               "preview_side": ((-0.4, -1.8, 1.3), (-0.4, 0.0, 1.15)),
               "preview_top": ((-0.4, 0.0, 2.9), (-0.4, 0.0, 1.0))},
    "dual": {"preview_front": ((1.1, 1.6, 2.1), (-0.45, 0.0, 1.1)),
             "preview_side": ((-0.4, -2.3, 1.35), (-0.4, 0.0, 1.15)),
             "preview_top": ((-0.45, 0.0, 3.4), (-0.45, 0.0, 1.0))},
}
assert all(tuple(views) == PREVIEW_CAMERAS for views in CAMERAS.values())
# Start: the flange target moves at most this far per control step from the home pose to the
# start pose, then holds until settled (or SETTLE_MAX_STEPS).
RAMP_METRES_PER_STEP = 0.01
RAMP_RADIANS_PER_STEP = 0.05
SETTLE_MAX_STEPS = 600
SETTLE_POSITION_M = 0.0002
SETTLE_ROTATION_RAD = 0.001
SETTLE_JOINT_SPEED = 0.01  # rad/s
SETTLE_CORRECTION = 0.05  # Share of the remaining flange error added to the target per hold step.
# The correction only removes the controller's small holding offset; an unreachable start pose is
# reported (converged: false), not chased.
SETTLE_CORRECTION_MAX_M = 0.005
SETTLE_CORRECTION_MAX_RAD = 0.05
# opspace() arguments that differ per arm and per step; every other keyword is the env's own.
PER_CALL = frozenset({"model", "data", "site_id", "dof_ids", "pos", "ori"})
FINGERS = ("ff", "mf", "rf", "th")
LEFT_FINGERS = ("rf", "mf", "ff", "th")  # The official left hand's mirrored order.


def official_env(env_name):
    """The official env class and module behind a DexJoCo task (its TaskConfig builds it)."""
    from dexjoco.sim.mujoco_gym_env import MujocoGymEnv
    from dexjoco.tasks import CONFIG_MAPPING

    module = sys.modules[CONFIG_MAPPING[env_name].__module__]
    classes = [value for value in vars(module).values()
               if isinstance(value, type) and issubclass(value, MujocoGymEnv) and value is not MujocoGymEnv]
    require(len(classes) == 1, f"{module.__name__} must import exactly one DexJoCo env class, found {classes}")
    return classes[0], sys.modules[classes[0].__module__]


def robot_file(env_module):
    """The robot MJCF the task's official arena includes (panda_allegro_*.xml)."""
    import xml.etree.ElementTree as ET

    arena = Path(env_module._XML_PATH)
    robots = [element.get("file") for element in ET.parse(arena).getroot().iter("include")
              if Path(element.get("file", "")).name.startswith("panda_allegro")]
    require(len(robots) == 1, f"{arena.name} must include exactly one panda_allegro robot, found {robots}")
    return arena.parent / robots[0]


def opspace_keywords(step_source, resolve, owner="step()"):
    """Keyword arguments of the opspace() calls in an env's step() source, minus the per-arm data
    arguments: literals evaluated, names resolved with ``resolve`` (joint=_PANDA_HOME). Every
    arm's call must agree."""
    tree = ast.parse(textwrap.dedent(step_source))
    calls = [node for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "opspace"]
    require(calls, f"{owner} has no opspace() call")
    found = []
    for call in calls:
        require(not call.args, f"{owner} passes positional opspace() arguments")
        found.append({keyword.arg: (resolve(keyword.value.id) if isinstance(keyword.value, ast.Name)
                                    else ast.literal_eval(keyword.value))
                      for keyword in call.keywords if keyword.arg not in PER_CALL})
    for other in found[1:]:
        require(other.keys() == found[0].keys() and all(np.array_equal(np.asarray(other[k]), np.asarray(found[0][k]))
                                                         for k in other),
                f"{owner} calls opspace() with different arguments per arm")
    return found[0]


def official_opspace_arguments(env_class):
    """The opspace() keyword arguments of the official env's own step()."""
    module = sys.modules[env_class.__module__]
    return opspace_keywords(inspect.getsource(env_class.step), lambda name: getattr(module, name),
                            f"{env_class.__name__}.step()")


def look_at(eye, target):
    """MuJoCo camera quaternion (wxyz; the camera looks along its -Z with +Y up) at eye facing target."""
    import mujoco

    eye, target = np.asarray(eye, float), np.asarray(target, float)
    forward = (target - eye) / np.linalg.norm(target - eye)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(forward @ up)) > 0.99:  # Looking straight down: image up is +x (across the table).
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    matrix = np.stack([right, np.cross(right, forward), -forward], axis=1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, matrix.reshape(-1))
    return quat


def build_model(robot_xml, cameras, size=CAMERA_SIZE):
    """The official robot MJCF, unchanged, plus a floor, a light and fixed cameras (as the arenas
    add theirs; the visual settings are the arenas' own)."""
    import mujoco

    spec = mujoco.MjSpec.from_file(str(robot_xml))
    spec.visual.global_.offwidth = spec.visual.global_.offheight = size
    spec.visual.headlight.diffuse = [0.4, 0.4, 0.4]
    spec.visual.headlight.ambient = [0.5, 0.5, 0.5]
    spec.visual.quality.offsamples = 8
    spec.add_texture(name="preview_grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=512, height=512,
                     rgb1=[0.1, 0.2, 0.3], rgb2=[0.2, 0.3, 0.4])
    spec.add_texture(name="preview_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, width=32, height=512,
                     rgb1=[0.3, 0.5, 0.7], rgb2=[0.0, 0.0, 0.0])
    material = spec.add_material(name="preview_grid", texrepeat=[2, 2], texuniform=True)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "preview_grid"
    spec.worldbody.add_geom(name="preview_floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.01],
                            material="preview_grid")
    spec.worldbody.add_light(name="preview_light", pos=[0, 0, 3], dir=[0, 0, -1], diffuse=[1, 1, 1],
                             specular=[0.3, 0.3, 0.3])
    for name, (eye, target) in cameras.items():
        spec.worldbody.add_camera(name=name, pos=list(eye), quat=look_at(eye, target).tolist(), fovy=CAMERA_FOVY)
    return spec.compile()


class _Arm(SimpleNamespace):
    """One Panda + Allegro: the ids the official env resolves for it (joints, actuators, site, sensors)."""


def arms(model, dual):
    """Per arm, the names the official env uses: joint{i}{suffix} (joint ids double as dof ids,
    as in the env), actuator{i}{suffix}, attachment_site{suffix}, the Allegro joints and position
    actuators in the official order (the left hand mirrored), and the flange and hand sensors."""
    sides = (("_right", "allegro_right", FINGERS), ("_left", "allegro_left", LEFT_FINGERS)) if dual else \
        (("", "allegro_right", FINGERS),)
    result = []
    for suffix, prefix, fingers in sides:
        joints = [f"{finger}j{i}" for finger in fingers for i in range(4)]
        arm = _Arm(
            dof_ids=np.asarray([model.joint(f"joint{i}{suffix}").id for i in range(1, 8)]),
            ctrl_ids=np.asarray([model.actuator(f"actuator{i}{suffix}").id for i in range(1, 8)]),
            site_id=model.site(f"attachment_site{suffix}").id,
            hand_qpos=np.asarray([int(model.joint(f"{j}{suffix}").qposadr[0]) for j in joints]),
            hand_joint_ids=np.asarray([model.joint(f"{j}{suffix}").id for j in joints]),
            hand_names=[f"{j}{suffix}" for j in joints],
            hand_ctrl=np.asarray([model.actuator(f"{finger}a{i}{suffix}").id for finger in fingers for i in range(4)]),
            flange=(f"franka/flange_pos{suffix}", f"franka/flange_quat{suffix}"),
            hand_sensors=[f"{prefix}/{j}_pos" for j in joints])
        # The env indexes qpos and qvel with joint ids; that holds while the robot comes first.
        require(np.array_equal(model.jnt_dofadr[arm.dof_ids], arm.dof_ids)
                and np.array_equal(model.jnt_qposadr[arm.dof_ids], arm.dof_ids), "robot joints are not first")
        result.append(arm)
    return result


class PreviewSession:
    def __init__(self, task):
        import mujoco
        from dexjoco.sim.controllers import opspace
        from dexjoco.sim.rendering import MujocoRenderer
        from dexjoco.tasks.policy_wrappers import DualArmPolicyWrapper, SingleArmPolicyWrapper
        from dexjoco_openpi_client.dexjoco_openpi_env import DexJoCoOpenPIEnv

        self.mujoco, self.opspace = mujoco, opspace
        self.info = T.task_info(task)
        self.dual = self.info["dual_arm"]
        env_class, env_module = official_env(self.info["env_name"])
        self.home = np.asarray(env_module._PANDA_HOME, dtype=np.float64)
        self.gains = official_opspace_arguments(env_class)
        defaults = inspect.signature(env_class.__init__).parameters
        control_dt, physics_dt = defaults["control_dt"].default, defaults["physics_dt"].default
        self.n_substeps = int(control_dt // physics_dt)  # As MujocoGymEnv.__init__.
        self.robot_xml = robot_file(env_module)
        self.cameras = CAMERAS["dual" if self.dual else "single"]
        self.model = build_model(self.robot_xml, self.cameras)
        self.model.opt.timestep = physics_dt  # As MujocoGymEnv.__init__.
        self.data = mujoco.MjData(self.model)
        self.arms = arms(self.model, self.dual)
        self.camera_ids = {name: int(self.model.camera(name).id) for name in self.cameras}
        self.renderer = MujocoRenderer(self.model, self.data, width=CAMERA_SIZE, height=CAMERA_SIZE)
        # The official row conversion: the evaluation wrapper's rotvec -> quaternion, then the
        # policy wrapper's environment action (one 23-D pose + hand row per arm).
        wrapper = SimpleNamespace(dual_arm=self.dual)
        split = DualArmPolicyWrapper.action if self.dual else SingleArmPolicyWrapper.action
        self._convert = lambda row: split(None, DexJoCoOpenPIEnv._process_action(wrapper, row))
        # The Allegro position actuators clip targets to their control ranges (as in the episode).
        self.hand_range = np.concatenate([self.model.actuator_ctrlrange[arm.hand_ctrl] for arm in self.arms])
        self.terminated = self.truncated = False  # services.simulator.rpc.serve contract
        self.finish_reason = None
        self.metadata = {
            "benchmark": "dexjoco", "task": task, "env_name": self.info["env_name"], "dual_arm": self.dual,
            "robot_model": self.robot_xml.name, "official_env": f"{env_class.__module__}.{env_class.__name__}",
            "control_frequency_hz": T.CONTROL_HZ, "physics_substeps": self.n_substeps, "physics_dt_s": physics_dt,
            "opspace_arguments": {k: np.asarray(v).tolist() for k, v in self.gains.items()},
            "home_arm_joints_rad": self.home.tolist(), "frame_stride": FRAME_STRIDE,
            "max_action_rows": MAX_ACTION_ROWS, "action_dim": T.action_dim(task), "state_dim": T.state_dim(task),
            "cameras": {name: {"position_m": list(eye), "look_at_m": list(target), "fovy_deg": CAMERA_FOVY,
                               "width": CAMERA_SIZE, "height": CAMERA_SIZE, "frame": "world"}
                        for name, (eye, target) in self.cameras.items()},
            "scene": "floor plane only", "task_environment_loaded": False,
        }

    # ------------------------------------------------------------------ state
    def state(self):
        """The official state vector, read as the env's _compute_observation() reads it: flange
        sensors, then the hand joint sensors (which the env stores as float32)."""
        sensor = lambda name: np.asarray(self.data.sensor(name).data, dtype=np.float64)  # noqa: E731
        poses = [np.concatenate([sensor(arm.flange[0]), sensor(arm.flange[1])]) for arm in self.arms]
        hands = [np.asarray([sensor(name)[0] for name in arm.hand_sensors], dtype=np.float32) for arm in self.arms]
        return np.concatenate([*poses, *hands]).astype(np.float64)

    def _split_state(self, state):
        """Official state vector -> per arm (flange position, unit quaternion wxyz, hand joints)."""
        n = len(self.arms)
        result = []
        for index in range(n):
            pose = state[7 * index:7 * index + 7]
            quat = pose[3:7] / np.linalg.norm(pose[3:7])
            hand = state[7 * n + 16 * index:7 * n + 16 * index + 16]
            result.append((pose[:3].copy(), quat, hand.copy()))
        return result

    def _targets(self, row):
        """One official row -> per arm (target position, target quaternion wxyz, hand targets),
        through the official conversion and as the env's step() reads its action."""
        action = self._convert(np.asarray(row, dtype=np.float64))
        per_arm = [action["right"], action["left"]] if self.dual else [action]
        return [(np.asarray(a[0:3], dtype=np.float64), np.asarray(a[3:7], dtype=np.float64),
                 np.asarray(a[7:23], dtype=np.float64)) for a in per_arm]

    def _control_step(self, targets):
        """One official environment step's control and physics (see the module docstring)."""
        model, data = self.model, self.data
        for _ in range(self.n_substeps):
            for arm, (position, quat, _) in zip(self.arms, targets):
                data.ctrl[arm.ctrl_ids] = self.opspace(model=model, data=data, site_id=arm.site_id,
                                                       dof_ids=arm.dof_ids, pos=position, ori=quat, **self.gains)
            for arm, (_, _, hand) in zip(self.arms, targets):
                data.ctrl[arm.hand_ctrl] = hand
            self.mujoco.mj_step(model, data)

    def _errors(self, start):
        from scipy.spatial.transform import Rotation

        current = self._split_state(self.state())
        position = max(float(np.linalg.norm(c[0] - s[0])) for c, s in zip(current, start))
        rotation = max(float((Rotation.from_quat(c[1], scalar_first=True).inv()
                              * Rotation.from_quat(s[1], scalar_first=True)).magnitude())
                       for c, s in zip(current, start))
        speed = float(np.abs(self.data.qvel[np.concatenate([arm.dof_ids for arm in self.arms])]).max())
        return position, rotation, speed

    def _start(self, start):
        """Home posture + start hand joints, then the official control drives the flange(s) to the
        start pose: a ramp from the home flange pose, then holds until settled. The hand joints are
        held at the start values meanwhile, so the rows start from the requested hand state."""
        from scipy.spatial.transform import Rotation, Slerp

        mujoco, model, data = self.mujoco, self.model, self.data
        mujoco.mj_resetData(model, data)
        clipped, hands = {}, []
        for arm, (_, _, hand) in zip(self.arms, start):
            data.qpos[arm.dof_ids] = self.home  # As reset().
            low, high = model.jnt_range[arm.hand_joint_ids].T
            inside = np.clip(hand, low, high)
            clipped.update({name: float(value - kept) for name, value, kept in zip(arm.hand_names, hand, inside)
                            if value != kept})
            hands.append(inside)
        hand_qpos = np.concatenate([arm.hand_qpos for arm in self.arms])
        hand_dofs = model.jnt_dofadr[np.concatenate([arm.hand_joint_ids for arm in self.arms])]
        data.qpos[hand_qpos] = np.concatenate(hands)
        mujoco.mj_forward(model, data)
        home = self._split_state(self.state())
        ramps = []
        for (h_pos, h_quat, _), (s_pos, s_quat, _) in zip(home, start):
            angle = float((Rotation.from_quat(h_quat, scalar_first=True).inv()
                           * Rotation.from_quat(s_quat, scalar_first=True)).magnitude())
            ramps.append(max(1, math.ceil(max(np.linalg.norm(s_pos - h_pos) / RAMP_METRES_PER_STEP,
                                              angle / RAMP_RADIANS_PER_STEP))))
        slerps = [Slerp([0.0, 1.0], Rotation.from_quat(np.stack([h[1], s[1]]), scalar_first=True))
                  for h, s in zip(home, start)]
        ramp, steps = max(ramps), 0
        corrections = [(np.zeros(3), Rotation.identity()) for _ in start]
        settled = lambda: (position <= SETTLE_POSITION_M and rotation <= SETTLE_ROTATION_RAD  # noqa: E731
                           and speed <= SETTLE_JOINT_SPEED)
        while True:
            steps += 1
            if steps <= ramp:
                alpha = steps / ramp
                targets = [((1 - alpha) * h[0] + alpha * s[0], slerp([alpha]).as_quat(scalar_first=True)[0], hand)
                           for h, s, slerp, hand in zip(home, start, slerps, hands)]
            else:
                targets = [(s[0] + shift, (turn * Rotation.from_quat(s[1], scalar_first=True)).as_quat(scalar_first=True),
                            hand) for s, (shift, turn), hand in zip(start, corrections, hands)]
            self._control_step(targets)
            data.qpos[hand_qpos], data.qvel[hand_dofs] = np.concatenate(hands), 0.0
            if steps >= ramp:
                mujoco.mj_forward(model, data)
                position, rotation, speed = self._errors(start)
                if settled() or steps >= ramp + SETTLE_MAX_STEPS:
                    break
                if speed > 5 * SETTLE_JOINT_SPEED:  # Correct the steady offset only, not the transient.
                    continue
                for index, (current, wanted) in enumerate(zip(self._split_state(self.state()), start)):
                    shift, turn = corrections[index]
                    error = (Rotation.from_quat(wanted[1], scalar_first=True)
                             * Rotation.from_quat(current[1], scalar_first=True).inv()).as_rotvec()
                    shift = shift + SETTLE_CORRECTION * (wanted[0] - current[0])
                    turn = (Rotation.from_rotvec(SETTLE_CORRECTION * error) * turn).as_rotvec()
                    shift *= min(1.0, SETTLE_CORRECTION_MAX_M / max(float(np.linalg.norm(shift)), 1e-12))
                    turn *= min(1.0, SETTLE_CORRECTION_MAX_RAD / max(float(np.linalg.norm(turn)), 1e-12))
                    corrections[index] = (shift, Rotation.from_rotvec(turn))
        report = {"converged": bool(settled()), "control_steps": steps, "ramp_steps": ramp,
                  "flange_position_error_m": position, "flange_rotation_error_rad": rotation,
                  "arm_joint_speed_max_rad_s": speed,
                  "tolerance": {"position_m": SETTLE_POSITION_M, "rotation_rad": SETTLE_ROTATION_RAD,
                                "joint_speed_rad_s": SETTLE_JOINT_SPEED},
                  "target_correction": [{"position_m": shift.tolist(), "rotation_rad": float(turn.magnitude())}
                                        for shift, turn in corrections],
                  # The preview's own elbow posture: the episode reports no arm joints to copy.
                  "preview_arm_joints_rad": [data.qpos[arm.dof_ids].tolist() for arm in self.arms]}
        if clipped:
            report["hand_joints_clipped_to_range_rad"] = clipped
        return report

    def _frame(self, row):
        images = {}
        for name, camera_id in self.camera_ids.items():
            image = self.renderer.render("rgb_array", camera_id=camera_id)
            images[name] = np.ascontiguousarray(np.asarray(image)[..., :3]).astype(np.uint8)
        return {"step": int(row), "state": self.state().tolist(), "images": images}

    # ------------------------------------------------------------------- run
    def run(self, initial_state, actions):
        began = time.monotonic()
        rows = np.asarray(actions, dtype=np.float64)
        dim, state_dim = T.action_dim(self.info["task"]), T.state_dim(self.info["task"])
        reject_unless(rows.ndim == 2 and rows.shape[1] == dim and 1 <= len(rows) <= MAX_ACTION_ROWS,
                      f"actions must be 1..{MAX_ACTION_ROWS} rows of {dim} numbers")
        reject_unless(bool(np.isfinite(rows).all()), "actions must be finite")
        state = np.asarray(initial_state, dtype=np.float64)
        reject_unless(state.shape == (state_dim,) and bool(np.isfinite(state).all()),
                      f"initial_state must be {state_dim} finite numbers")
        reject_unless(all(np.linalg.norm(state[7 * i + 3:7 * i + 7]) > 1e-6 for i in range(len(self.arms))),
                      "initial_state flange quaternions must be non-zero")
        start = self._split_state(state)
        settle = self._start(start)
        frames = [self._frame(0)]
        start_state = self.state().tolist()
        hand_low, hand_high = self.hand_range.T
        clipped_rows = 0
        for index, row in enumerate(rows, 1):
            targets = self._targets(row)
            hands = np.concatenate([t[2] for t in targets])
            clipped_rows += bool(((hands < hand_low) | (hands > hand_high)).any())
            self._control_step(targets)
            if index % FRAME_STRIDE == 0 or index == len(rows):
                frames.append(self._frame(index))
        # MuJoCo's own warnings (e.g. an unstable acceleration it reset) since the start was rebuilt.
        warnings = {self.mujoco.mjtWarning(index).name: int(item.number)
                    for index, item in enumerate(self.data.warning) if item.number}
        return {"status": "Success", "rows": int(len(rows)), "frames": frames, "settle": settle,
                "start_state": start_state, "end_state": self.state().tolist(),
                "rows_with_clipped_hand_targets": clipped_rows,
                **({"physics_warnings": warnings} if warnings else {}),
                "seconds": round(time.monotonic() - began, 2)}

    def dispatch(self, op, args):
        if op == "metadata":
            return self.metadata
        if op == "run":
            return self.run(args["initial_state"], args["actions"])
        raise RejectedBeforeAction(f"Unknown preview op: {op}")

    def _write_summary(self, reason):  # services.simulator.rpc.serve contract; nothing to record.
        self.finish_reason = reason

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
