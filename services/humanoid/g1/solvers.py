"""cuRobo-backed arm IK, motion planning and collision queries for the G1 + Dex3.

CUDA, torch and cuRobo are imported on the first call only; built solvers are cached for
the life of the process (one per kind / side / waist option / end effector / position-only),
so repeated calls only pay the solve time.

Locked joints. Every joint that is not being solved for (the other arm, the legs, the
waist unless ``use_waist=True`` and the Dex3 fingers) is a *locked* joint in the cuRobo
model, held at the caller's measured ``current_joints``. cuRobo turns a locked joint into a
fixed transform ``parent_T_child = origin * Rot(axis, q)``; this module rewrites those
transforms in place before every call (the cuRobo-supported way to change locked-joint
state without rebuilding, see ``Kinematics.update_kinematics_config``), so the same CUDA
graphs serve any posture. ``self_check()`` verifies this against a freshly built model.
"""
from __future__ import annotations

import copy
import os
import threading
import time
from functools import lru_cache

import numpy as np

from . import model as g1

_LOCK = threading.RLock()
_CACHE: dict = {}

#: Planning limits for the active joints. Velocity stays at the URDF limit (cuRobo's B-spline
#: trajectory optimiser bounds the trajectory duration, so a low velocity cap makes ordinary
#: reaches infeasible); acceleration and jerk are capped well below cuRobo's defaults (10, 500)
#: so planned reaches peak around 1-1.5 rad/s. Slow further with ``time_scale`` (retiming).
DEFAULT_MAX_ACCELERATION_RAD_S2 = 6.0
DEFAULT_MAX_JERK_RAD_S3 = 100.0
#: With ``use_waist=True`` the solvers keep the waist inside this conservative envelope (rad), a
#: subset of the URDF limits (yaw +-2.618, roll/pitch +-0.52): large waist excursions would ask the
#: balance controller for postures it was not shown to hold.
WAIST_LIMITS_RAD = {"waist_yaw_joint": 0.6, "waist_roll_joint": 0.25, "waist_pitch_joint": 0.35}
#: Extra clearance added to every robot sphere for obstacle (not self) collision, metres.
WORLD_COLLISION_MARGIN_M = 0.01
#: Maximum number of cuboid obstacles per call (collision cache size).
MAX_CUBOIDS = 16


# ----------------------------------------------------------------------------- runtime
@lru_cache(maxsize=1)
def runtime():
    """Import torch/cuRobo once, after installing the persistent kernel caches."""
    from . import kernel_cache
    kernel_cache.install()
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("The G1 planner needs a CUDA GPU (torch.cuda.is_available() is False)")
    from curobo.types import DeviceCfg
    return {"torch": torch, "device_cfg": DeviceCfg(device=torch.device("cuda", 0))}


@lru_cache(maxsize=1)
def sphere_model():
    import yaml
    return yaml.safe_load(g1.CUROBO_SPHERES_PATH.read_text())


def _chain_joints(links):
    """Movable joints between the pelvis and any of ``links``."""
    model = g1.model()
    out = []
    for link in links:
        for joint in model.chain(link):
            if joint.type != "fixed" and joint.name not in out:
                out.append(joint.name)
    return out


def _extra_links():
    """Fixed convenience frames (hand centres, camera optical frame) as cuRobo extra links."""
    out = {}
    for name in ("left_hand_center", "right_hand_center", "head_center", "camera_optical"):
        parent, offset, rotation = g1.NAMED_FRAMES[name]
        out[name] = {"parent_link_name": parent, "link_name": name, "joint_name": f"{name}_fixed",
                     "joint_type": "FIXED", "fixed_transform": [*offset, *rotation]}
    return out


#: Link pairs separated by at most this many movable joints are never checked against each other:
#: the sphere model of compact neighbouring parts (wrist roll/pitch/yaw, palm and thumb base,
#: pelvis and hip) overlaps by millimetres in ordinary postures although the joint limits keep
#: the real parts apart.
NEIGHBOUR_JOINT_DISTANCE = 2


def self_collision_ignore(active_joints):
    """Shipped ignore matrix + close kinematic neighbours + pairs that cannot move relatively.

    With ``active_joints`` fixed for the solve, two links whose chains contain the same active
    joints keep a constant relative pose (it only depends on locked joints), so they are ignored:
    a self-contact among held joints (e.g. legs vs pelvis at the measured posture) must not make
    every arm solution infeasible.
    """
    spheres = sphere_model()
    links = list(spheres["collision_link_names"])
    model = g1.model()
    chains = {link: [j.name for j in model.chain(link) if j.type != "fixed"] for link in links}
    active = set(active_joints)
    ignore = {k: set(v) for k, v in spheres["self_collision_ignore"].items()}
    for i, a in enumerate(links):
        for b in links[i + 1:]:
            ca, cb = chains[a], chains[b]
            separating = set(ca) ^ set(cb)
            fixed = (set(ca) & active) == (set(cb) & active)
            if len(separating) <= NEIGHBOUR_JOINT_DISTANCE or fixed:
                ignore.setdefault(a, set()).add(b)
    return {k: sorted(v) for k, v in ignore.items() if v}


def _tool_link(side, end_effector):
    if end_effector not in g1.END_EFFECTORS:
        raise ValueError(f"end_effector must be one of {g1.END_EFFECTORS}")
    if end_effector == "hand_center":
        return f"{side}_hand_center"
    return g1.NAMED_FRAMES[f"{side}_{end_effector}"][0]


def _limit_clip(joint):
    """Symmetric shrink of the URDF range that leaves +-WAIST_LIMITS_RAD for the waist joints."""
    if joint not in WAIST_LIMITS_RAD:
        return 0.0
    upper = g1.model().joints[joint].upper
    return max(0.0, upper - WAIST_LIMITS_RAD[joint])


def robot_dict(active_joints, tool_frames, collision=True, clip_waist=True):
    """cuRobo robot configuration with ``active_joints`` free and every other joint locked."""
    spheres = sphere_model()
    collision_links = list(spheres["collision_link_names"]) if collision else []
    extra = _extra_links()
    parents = [extra[t]["parent_link_name"] if t in extra else t for t in tool_frames]
    chain = _chain_joints(collision_links + parents)
    for joint in active_joints:
        if joint not in chain:
            raise ValueError(f"{joint} does not move any tool frame or collision link")
    locked = {j: 0.0 for j in chain if j not in active_joints}
    n = len(active_joints)
    kinematics = {
        "urdf_path": str(g1.URDF_PATH),
        "asset_root_path": str(g1.ROBOT_DIR),
        "base_link": "pelvis",
        "tool_frames": list(tool_frames),
        "extra_links": extra,
        "lock_joints": locked,
        "load_meshes": False,
        "cspace": {
            "joint_names": list(active_joints),
            "default_joint_position": [0.0] * n,
            "cspace_distance_weight": [1.0] * n,
            "null_space_weight": [1.0] * n,
            "max_acceleration": [DEFAULT_MAX_ACCELERATION_RAD_S2] * n,
            "max_jerk": [DEFAULT_MAX_JERK_RAD_S3] * n,
            "velocity_scale": [1.0] * n,
            # cuRobo v0.8.0 does not reorder position_limit_clip with the cspace (other cspace
            # arrays are reordered), so a per-joint list must already be in cuRobo's joint order;
            # _Context verifies that order after building.
            "position_limit_clip": ([_limit_clip(j) for j in active_joints]
                                    if clip_waist and any(_limit_clip(j) for j in active_joints) else 0.0),
        },
    }
    if collision:
        kinematics.update({
            "collision_link_names": collision_links,
            "collision_spheres": spheres["collision_spheres"],
            "collision_sphere_buffer": WORLD_COLLISION_MARGIN_M,
            "self_collision_ignore": self_collision_ignore(active_joints),
            "self_collision_buffer": {link: -WORLD_COLLISION_MARGIN_M for link in collision_links},
        })
    return {"kinematics": kinematics}


# ----------------------------------------------------------------------------- locked joints
def _kinematics_params(root):
    """All cuRobo KinematicsParams objects reachable from ``root`` (deduplicated)."""
    from curobo._src.robot.types.kinematics_params import KinematicsParams
    torch = runtime()["torch"]
    found, seen, stack = {}, set(), [root]
    while stack:
        obj = stack.pop()
        if id(obj) in seen or obj is None or isinstance(obj, (str, bytes, int, float, bool, torch.Tensor,
                                                               np.ndarray, type)):
            continue
        seen.add(id(obj))
        if isinstance(obj, KinematicsParams):
            found[id(obj)] = obj
            continue
        if isinstance(obj, dict):
            stack.extend(obj.values())
        elif isinstance(obj, (list, tuple, set)):
            stack.extend(obj)
        elif type(obj).__module__.startswith("curobo"):
            stack.extend(getattr(obj, "__dict__", {}).values())
            for slot in getattr(type(obj), "__slots__", ()):
                stack.append(getattr(obj, slot, None))
    return list(found.values())


class LockedJoints:
    """Rewrites locked-joint fixed transforms in every KinematicsParams of a solver."""

    def __init__(self, solver, locked_names):
        torch = runtime()["torch"]
        model = g1.model()
        self.names = list(locked_names)
        self.params = _kinematics_params(solver)
        if not self.params:
            raise RuntimeError("No cuRobo kinematics parameters found in solver")
        self.entries = []
        for params in self.params:
            index = params.link_name_to_idx_map
            rows = []
            for name in self.names:
                joint = model.joints[name]
                if joint.child not in index:
                    raise RuntimeError(f"Locked joint {name}: link {joint.child} not in cuRobo model")
                rows.append(index[joint.child])
            self.entries.append((params, torch.as_tensor(rows, device=params.fixed_transforms.device,
                                                         dtype=torch.long)))
        self.current = None

    def update(self, joints):
        values = tuple(round(float(joints.get(name, 0.0)), 7) for name in self.names)
        if values == self.current:
            return
        torch = runtime()["torch"]
        model = g1.model()
        blocks = np.stack([model.joints[name].transform(value)[:3, :4]
                           for name, value in zip(self.names, values)])
        for params, rows in self.entries:
            data = torch.as_tensor(blocks, device=params.fixed_transforms.device,
                                   dtype=params.fixed_transforms.dtype)
            params.fixed_transforms[rows] = data
            if params.lock_jointstate is not None:
                lock_names = list(params.lock_jointstate.joint_names)
                position = params.lock_jointstate.position
                for name, value in zip(self.names, values):
                    if name in lock_names:
                        position.view(-1)[lock_names.index(name)] = value
        self.current = values


# ----------------------------------------------------------------------------- obstacles
def _scene(obstacles, base_T_frame=None):
    """List of {'cuboid': {'dims': [x,y,z], 'pose_wxyz': [x,y,z,qw,qx,qy,qz]}} -> cuRobo Scene."""
    from curobo.scene import Cuboid, Scene
    cuboids = []
    for i, item in enumerate(obstacles or []):
        if set(item) != {"cuboid"}:
            raise ValueError("Each obstacle must be {'cuboid': {'dims': [...], 'pose_wxyz': [...]}}")
        spec = item["cuboid"]
        dims = [float(v) for v in spec["dims"]]
        pose = [float(v) for v in spec["pose_wxyz"]]
        if len(dims) != 3 or min(dims) <= 0 or len(pose) != 7 or not np.all(np.isfinite(pose)):
            raise ValueError(f"Bad cuboid obstacle {i}: {spec}")
        matrix = g1.pose_matrix(pose[:3], pose[3:])
        if base_T_frame is not None:
            matrix = base_T_frame @ matrix
        p = g1.matrix_pose(matrix)
        cuboids.append(Cuboid(name=spec.get("name", f"cuboid_{i}"), dims=dims,
                              pose=p["position_m"] + p["quaternion_wxyz"]))
    if len(cuboids) > MAX_CUBOIDS:
        raise ValueError(f"At most {MAX_CUBOIDS} cuboid obstacles are supported per call")
    if not cuboids:
        # Keep the collision buffers allocated with a far-away placeholder.
        cuboids = [Cuboid(name="placeholder_far", dims=[0.01, 0.01, 0.01], pose=[50.0, 50.0, 50.0, 1, 0, 0, 0])]
    return Scene(cuboid=cuboids)


def _empty_scene_dict():
    return {"cuboid": {"placeholder_far": {"dims": [0.01, 0.01, 0.01], "pose": [50.0, 50.0, 50.0, 1, 0, 0, 0]}}}


# ----------------------------------------------------------------------------- contexts
class _Context:
    kind = "ik"

    def __init__(self, side, use_waist, end_effector, seeds, position_only=False):
        self.side, self.use_waist, self.end_effector = side, use_waist, end_effector
        self.active = (list(g1.WAIST_JOINTS) if use_waist else []) + list(g1.ARM_JOINTS[side])
        self.tool_link = _tool_link(side, end_effector)
        self.robot = robot_dict(self.active, [self.tool_link])
        self.locked_names = list(self.robot["kinematics"]["lock_joints"])
        started = time.perf_counter()
        self.solver = self._build(seeds)
        order = list(self.solver.kinematics.joint_names)
        if order != self.active:
            raise RuntimeError(f"cuRobo joint order {order} differs from {self.active}; "
                               "position_limit_clip would be misapplied")
        self.position_only = None
        # Pose criteria are fixed per solver (a separate solver serves position-only goals) so
        # that captured CUDA graphs never depend on a criteria switch.
        self._set_position_only(position_only)
        self.locks = LockedJoints(self.solver, self.locked_names)
        self.build_time_s = time.perf_counter() - started
        self.obstacles_key = None
        model = g1.model()
        active = set(self.active)
        #: Collision links that the solved joints cannot move (their pose is fixed by held joints).
        self.held_links = [link for link in sphere_model()["collision_link_names"]
                           if not active & {j.name for j in model.chain(link)}]
        self.disabled = set()

    def set_state(self, joints, obstacles, base_T_frame):
        """Held joints, obstacles, and held links already touching an obstacle.

        A held link that already intersects an obstacle at ``joints`` (e.g. the other hand resting
        inside a table model) would make every solution infeasible although the solve cannot
        change it; its spheres are disabled for this call and reported to the caller.
        """
        self.locks.update(joints)
        key = repr((obstacles, None if base_T_frame is None else base_T_frame.round(9).tolist()))
        if key != self.obstacles_key:
            self.solver.update_world(_scene(obstacles, base_T_frame))
            self.obstacles_key = key
        contacts = g1.obstacle_contacts(joints, obstacles, margin_m=2 * WORLD_COLLISION_MARGIN_M,
                                        base_T_frame=base_T_frame)
        disable = {link for link in contacts if link in self.held_links}
        for params in self.locks.params:
            for link in self.disabled - disable:
                params.enable_link_spheres(link)
            for link in disable - self.disabled:
                params.disable_link_spheres(link)
        self.disabled = disable
        return {"held_links_excluded_from_collision": sorted(disable),
                "moving_links_touching_obstacles_at_start": sorted(set(contacts) - set(self.held_links))}

    def _set_position_only(self, position_only):
        if not position_only:
            self.position_only = False
            return
        from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
        self.solver.update_tool_pose_criteria(
            {self.tool_link: ToolPoseCriteria(terminal_pose_axes_weight_factor=[1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                                              device_cfg=runtime()["device_cfg"])})
        self.position_only = True


class _IKContext(_Context):
    def _build(self, seeds):
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
        config = InverseKinematicsCfg.create(
            robot=self.robot, scene_model=_empty_scene_dict(), collision_cache={"cuboid": MAX_CUBOIDS},
            self_collision_check=True, num_seeds=seeds, use_cuda_graph=_use_cuda_graph(),
            device_cfg=runtime()["device_cfg"])
        return InverseKinematics(config)


class _PlanContext(_Context):
    kind = "plan"

    def _build(self, seeds):
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        config = MotionPlannerCfg.create(
            robot=self.robot, scene_model=_empty_scene_dict(), collision_cache={"cuboid": MAX_CUBOIDS},
            self_collision_check=True, num_ik_seeds=seeds, num_trajopt_seeds=4,
            use_cuda_graph=_use_cuda_graph(), device_cfg=runtime()["device_cfg"])
        planner = MotionPlanner(config)
        return planner


def _use_cuda_graph():
    """CUDA graphs (default on): slower first call per solver, then ~20-30x faster solves."""
    return os.environ.get("G1_CUROBO_CUDA_GRAPH", "1") != "0"


def context(kind, side, use_waist=False, end_effector="palm", seeds=32, position_only=False):
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    runtime()  # kernel caches and warp cache directory before any cuRobo import
    key = (kind, side, bool(use_waist), end_effector, int(seeds), bool(position_only))
    with _LOCK:
        if key not in _CACHE:
            cls = _IKContext if kind == "ik" else _PlanContext
            _CACHE[key] = cls(side, bool(use_waist), end_effector, int(seeds), bool(position_only))
        return _CACHE[key]


# ----------------------------------------------------------------------------- solving
def _tensor(values):
    rt = runtime()
    return rt["torch"].as_tensor(np.asarray(values, dtype=np.float32), device=rt["device_cfg"].device)


def _goal(ctx, target):
    from curobo.types import GoalToolPose, Pose
    pose = Pose(position=_tensor(target[:3, 3]).view(1, 3),
                quaternion=_tensor(g1.matrix_to_quat(target[:3, :3])).view(1, 4))
    return GoalToolPose.from_poses({ctx.tool_link: pose}, ordered_tool_frames=[ctx.tool_link], num_goalset=1)


def _active_state(ctx, joints):
    from curobo.types import JointState
    position = _tensor([joints.get(name, 0.0) for name in ctx.active]).view(1, -1)
    return JointState.from_position(position, joint_names=list(ctx.active))


def _first(value):
    if value is None:
        return None
    return float(value.reshape(-1)[0].item())


def solve_ik(ctx, joints, target, position_only, obstacles, base_T_frame):
    with _LOCK:
        notes = ctx.set_state(joints, obstacles, base_T_frame)
        started = time.perf_counter()
        result = ctx.solver.solve_pose(_goal(ctx, target), current_state=_active_state(ctx, joints))
        success = bool(result.success.reshape(-1)[0].item())  # synchronises the GPU
        elapsed = time.perf_counter() - started
        js = result.js_solution
        names = list(js.joint_names)
        position = js.position.reshape(-1, len(names))[0].detach().cpu().numpy()
        solution = {name: float(position[names.index(name)]) for name in ctx.active}
        return {"success": success, "solution": solution, "solve_time_s": elapsed, **notes,
                "position_error_m": _first(result.position_error),
                "rotation_error_rad": _first(result.rotation_error)}


def plan(ctx, joints, target, position_only, obstacles, base_T_frame, max_attempts):
    with _LOCK:
        notes = ctx.set_state(joints, obstacles, base_T_frame)
        started = time.perf_counter()
        result = ctx.solver.plan_pose(_goal(ctx, target), _active_state(ctx, joints),
                                      max_attempts=int(max_attempts))
        elapsed = time.perf_counter() - started
        if result is None or not bool(result.success.reshape(-1).any().item()):
            status = "planning_failed" if result is None else str(getattr(result, "status", "failed"))
            return {"success": False, "status": status, "positions": None, "dt_s": None,
                    "planning_time_s": elapsed, **notes}
        plan_js = result.get_interpolated_plan()
        positions = plan_js.position.reshape(-1, plan_js.position.shape[-1]).detach().cpu().numpy()
        names = list(plan_js.joint_names) if plan_js.joint_names else list(ctx.active)
        order = [names.index(name) for name in ctx.active]
        dt = ctx.solver.trajopt_solver.config.interpolation_dt
        if getattr(plan_js, "dt", None) is not None:
            dt = float(plan_js.dt.reshape(-1)[0].item())
        positions = positions[:, order]  # get_interpolated_plan() is already trimmed
        return {"success": True, "status": "success", "positions": positions, "dt_s": float(dt),
                "planning_time_s": elapsed, **notes}


# ----------------------------------------------------------------------------- FK / collision
class _FullBody:
    """All body and finger joints active: plain FK and full-configuration collision checks."""

    def __init__(self):
        from curobo.kinematics import Kinematics, KinematicsCfg
        started = time.perf_counter()
        frames = sorted({g1.NAMED_FRAMES[n][0] if n not in _extra_links() else n for n in g1.NAMED_FRAMES}
                        | {"torso_link"})
        self.frames = frames
        extra = _extra_links()
        parents = [extra[f]["parent_link_name"] if f in extra else f for f in frames]
        chain = _chain_joints(list(sphere_model()["collision_link_names"]) + parents)
        self.joint_names = [j for j in g1.ALL_JOINTS if j in chain]
        robot = robot_dict(self.joint_names, frames, collision=True, clip_waist=False)
        robot["kinematics"]["lock_joints"] = None
        self.robot = robot
        rt = runtime()
        cfg = KinematicsCfg.from_data_dict(dict(robot["kinematics"]), device_cfg=rt["device_cfg"])
        self.kinematics = Kinematics(cfg)
        self.order = list(self.kinematics.joint_names)
        self.checker = None
        self.obstacles_key = None
        self.build_time_s = time.perf_counter() - started

    def collision_checker(self):
        """cuRobo RobotCollisionChecker with all joints active (built on first use)."""
        if self.checker is None:
            from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
            rt = runtime()
            cfg = RobotCollisionCheckerCfg.load_from_config(
                robot_config=dict(self.robot), scene_model=_empty_scene_dict(), device_cfg=rt["device_cfg"],
                n_cuboids=MAX_CUBOIDS, n_meshes=1, collision_activation_distance=0.0,
                self_collision_activation_distance=0.0)
            self.checker = RobotCollisionChecker(cfg)
            self.checker_order = list(self.checker.kinematics.joint_names)
            # v0.8.0's RobotCollisionChecker leaves the scene-cost sphere count at 0; set it.
            spheres = int(self.checker.kinematics.kinematics_config.total_spheres)
            for cost in (self.checker.collision_cost, self.checker.collision_constraint):
                if cost is not None:
                    cost.update_num_spheres(spheres, 1, 1)
            self.checker.setup_batch_tensors(1, 1)
        return self.checker

    def vector(self, joints, order):
        return _tensor([joints.get(name, 0.0) for name in order]).view(1, -1)


def full_body():
    runtime()
    with _LOCK:
        if "full" not in _CACHE:
            _CACHE["full"] = _FullBody()
        return _CACHE["full"]


def curobo_frames(joints, names):
    from curobo.types import JointState
    body = full_body()
    with _LOCK:
        state = body.kinematics.compute_kinematics(
            JointState.from_position(body.vector(joints, body.order), joint_names=body.order))
        out = {}
        poses = {}
        for link in body.frames:
            pose = state.tool_poses.get_link_pose(link)
            matrix = np.eye(4)
            matrix[:3, :3] = g1.quat_to_matrix(pose.quaternion.reshape(-1, 4)[0].cpu().numpy())
            matrix[:3, 3] = pose.position.reshape(-1, 3)[0].cpu().numpy()
            poses[link] = matrix
        for name in names:
            link, offset, rotation = g1.NAMED_FRAMES[name]
            if name in _extra_links():
                out[name] = poses[name]
            else:
                out[name] = poses[link] @ g1.pose_matrix(offset, rotation)
        return out, poses["torso_link"]


def collision(joints, obstacles, base_T_frame):
    body = full_body()
    with _LOCK:
        checker = body.collision_checker()
        key = repr((obstacles, base_T_frame.round(9).tolist()))
        if key != body.obstacles_key:
            checker.update_world(_scene(obstacles, base_T_frame))
            body.obstacles_key = key
        q = body.vector(joints, body.checker_order).view(1, 1, -1)
        state = copy.copy(checker.get_kinematics(q))
        spheres = state.robot_spheres.view(1, 1, -1, 4)
        state.robot_spheres = spheres
        self_value = float(checker.get_self_collision(spheres).reshape(-1).max().item())
        # cuRobo v0.8.0's scene-collision cost takes the kinematics state, not the sphere tensor.
        scene_value = float(checker.collision_constraint.forward(state).reshape(-1).max().item())
        return {"self_collision": self_value > 0.0, "obstacle_collision": scene_value > 0.0,
                "self_penetration_m": self_value, "obstacle_penetration_sum_m": scene_value}


def self_check(samples=5, seed=0):
    """Locked-joint rewrite == freshly built model (max abs fixed-transform / FK difference)."""
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.types import JointState
    rng = np.random.default_rng(seed)
    ctx = context("ik", "right", False, "palm", 32, False)
    limits = g1.joint_limits(g1.ALL_JOINTS)
    worst = 0.0
    for _ in range(samples):
        joints = {name: float(rng.uniform(lo, hi)) for name, (lo, hi) in limits.items()}
        ctx.locks.update(joints)
        fresh = robot_dict(ctx.active, [ctx.tool_link])
        fresh["kinematics"]["lock_joints"] = {n: joints[n] for n in ctx.locked_names}
        cfg = KinematicsCfg.from_data_dict(dict(fresh["kinematics"]), device_cfg=runtime()["device_cfg"])
        reference = cfg.kinematics_config.fixed_transforms
        for params in ctx.locks.params:
            worst = max(worst, float((params.fixed_transforms - reference).abs().max().item()))
        kin = Kinematics(cfg)
        q = _tensor([joints[n] for n in kin.joint_names]).view(1, -1)
        ref_pose = kin.compute_kinematics(JointState.from_position(q, joint_names=list(kin.joint_names)))
        mine = ctx.solver.compute_kinematics(JointState.from_position(q, joint_names=list(kin.joint_names)))
        worst = max(worst, float((ref_pose.tool_poses.position - mine.tool_poses.position).abs().max().item()))
    return {"samples": samples, "max_abs_difference": worst, "kinematics_params_instances": len(ctx.locks.params)}
