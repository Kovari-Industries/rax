"""Simulator-side checks of the official DexJoCo environment (run in runtime/envs/dexjoco).

``python -m services.dexjoco.checks --task pick_bucket`` prints one JSON report:

- layout determinism: the official evaluation's episode k of seed S (reset k+1 times, with
  policy steps in between) equals k+1 resets with no steps in between, which is how the
  harness reaches a formal layout in a fresh process;
- timing of environment construction, reset, step and depth rendering;
- EGL rendering on the selected device.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def build(task, seed):
    from services.dexjoco import tasks as T
    from services.dexjoco.session import _Tap
    from dexjoco_openpi_client.dexjoco_openpi_env import DexJoCoOpenPIEnv
    from dexjoco_openpi_client.eval_dexjoco_openpi import _set_seed

    info, config = T.task_info(task), T.official_config(task)
    _set_seed(seed)
    env = DexJoCoOpenPIEnv(env_name=info["env_name"], camera_mapping=config["camera_mapping"], seed=seed,
                           rand_full=info["regime"] == "rand_full", randomize_dynamics=False,
                           dual_arm=info["dual_arm"], prompt=config["prompt"], render_mode="rgb_array",
                           password=config.get("password", None))
    env.start()
    tap = _Tap(env.env)
    env.env = tap
    return env, tap


def layout_signature(env, tap):
    """The full official state (privileged tail included) plus scene randomization state."""
    raw = tap.unwrapped
    model = raw._model
    return {"state": np.asarray(tap.last_obs["state"], dtype=np.float64).tolist(),
            "cam_pos": np.asarray(model.cam_pos).round(9).tolist(),
            "cam_quat": np.asarray(model.cam_quat).round(9).tolist(),
            "light_pos": np.asarray(model.light_pos).round(9).tolist(),
            "light_diffuse": np.asarray(model.light_diffuse).round(9).tolist(),
            "geom_matid": np.asarray(model.geom_matid).tolist()}


def determinism(task, seed, episodes, steps_between):
    with_steps, without_steps = [], []
    env, tap = build(task, seed)
    for _ in range(episodes):
        env.reset()
        with_steps.append(layout_signature(env, tap))
        for i in range(steps_between):
            env.stay(continue_stay=i > 0)
            if env.is_done:
                break
    env.close()
    env, tap = build(task, seed)
    for _ in range(episodes):
        env.reset()
        without_steps.append(layout_signature(env, tap))
    env.close()
    equal = [a == b for a, b in zip(with_steps, without_steps)]
    distinct = len({json.dumps(s["state"]) for s in without_steps})
    return {"seed": seed, "episodes": episodes, "steps_between": steps_between, "identical": all(equal),
            "per_episode_identical": equal, "distinct_layouts": distinct}


def timing(task, seed):
    from dexjoco.data.depth_capture import collect_depth_frames

    started = time.monotonic()
    env, tap = build(task, seed)
    built = time.monotonic() - started
    started = time.monotonic()
    env.reset()
    reset = time.monotonic() - started
    started = time.monotonic()
    for i in range(30):
        env.stay(continue_stay=i > 0)
    step = (time.monotonic() - started) / 30
    keys = [k for k in tap.last_obs if k != "state"]
    started = time.monotonic()
    depth = collect_depth_frames(tap, keys)
    depth_seconds = time.monotonic() - started
    images = env.get_raw_images()
    report = {"build_seconds": round(built, 3), "reset_seconds": round(reset, 3),
              "step_seconds": round(step, 4), "depth_render_seconds": round(depth_seconds, 4),
              "image_shapes": {k: list(np.asarray(v).shape) for k, v in images.items()},
              "image_means": {k: float(np.asarray(v).mean()) for k, v in images.items()},
              "depth_cameras": sorted(depth),
              "depth_finite_fraction": {k: float(np.isfinite(v).mean()) for k, v in depth.items()},
              "depth_median_m": {k: float(np.median(v[np.isfinite(v)])) for k, v in depth.items()}}
    env.close()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--steps-between", type=int, default=40)
    parser.add_argument("--describe", action="store_true", help="print robot facts only")
    parser.add_argument("--characterize", action="store_true", help="print the control response only")
    args = parser.parse_args(argv)
    os.environ.setdefault("MUJOCO_GL", "egl")
    if args.characterize:
        print(json.dumps(characterize(args.task, args.seed)), flush=True)
        return
    if args.describe:
        print(json.dumps(describe(args.task, args.seed)), flush=True)
        return
    report = {"task": args.task, "mujoco_gl": os.environ.get("MUJOCO_GL"),
              "egl_device": os.environ.get("MUJOCO_EGL_DEVICE_ID"),
              "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES")}
    report["timing"] = timing(args.task, args.seed)
    report["determinism"] = determinism(args.task, args.seed, args.episodes, args.steps_between)
    print(json.dumps(report), flush=True)



def describe(task, seed=0):
    """Robot facts at the first reset of `seed`: the official state, hand joint ranges and gains,
    robot base placement, table height and camera calibration (operator reference for skills)."""
    import mujoco
    from services.dexjoco import tasks as T

    env, tap = build(task, seed)
    env.reset()
    raw = tap.unwrapped
    model, data = raw._model, raw._data
    dim = T.state_dim(task)
    state = np.asarray(tap.last_obs["state"], dtype=np.float64)
    joints = {}
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        if any(name.startswith(p) for p in ("ff", "mf", "rf", "th", "joint")):
            joints[name] = [round(float(v), 4) for v in model.jnt_range[j]]
    actuators = {}
    for a in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
        actuators[name] = {"gain": round(float(model.actuator_gainprm[a][0]), 4),
                           "ctrlrange": [round(float(v), 4) for v in model.actuator_ctrlrange[a]]}
    bodies = {}
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if name.startswith(("base", "link0", "table", "attachment", "allegro_palm")):
            bodies[name] = [round(float(v), 4) for v in data.xpos[b]]
    sites = {}
    for s in range(model.nsite):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, s) or ""
        if name.startswith("attachment_site"):
            mat = np.asarray(data.site_xmat[s]).reshape(3, 3)
            sites[name] = {"pos": [round(float(v), 4) for v in data.site_xpos[s]],
                           "x_axis": mat[:, 0].round(3).tolist(), "y_axis": mat[:, 1].round(3).tolist(),
                           "z_axis": mat[:, 2].round(3).tolist()}
    report = {"task": task, "official_state": state[:dim].round(5).tolist(), "privileged_tail_len": int(len(state) - dim),
              "hand_and_arm_joint_ranges": joints, "actuators": actuators, "bodies": bodies, "flange_sites": sites,
              "timestep": float(model.opt.timestep), "image_keys": [k for k in tap.last_obs if k != "state"]}
    env.close()
    return report


def characterize(task, seed=0):
    """Official control response from the start pose: a 10 cm flange target change (OSC) and a hand
    closing command (position actuators), measured step by step through the official step()."""
    import mujoco
    from scipy.spatial.transform import Rotation
    from services.dexjoco import tasks as T

    env, tap = build(task, seed)
    env.reset()
    raw = tap.unwrapped
    model, data = raw._model, raw._data
    dual = T.task_info(task)["dual_arm"]
    state = np.asarray(env.obs["state"], dtype=np.float64)

    def tips():
        out = {}
        for g in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if name.startswith(("fingertip", "thumbtip")):
                out[name] = [round(float(v), 4) for v in data.geom_xpos[g]]
        return out

    start_tips = tips()
    if dual:
        r_rotvec = Rotation.from_quat(state[3:7], scalar_first=True).as_rotvec()
        l_rotvec = Rotation.from_quat(state[10:14], scalar_first=True).as_rotvec()
        base = np.concatenate([state[0:3], r_rotvec, state[14:30], state[7:10], l_rotvec, state[30:46]])
    else:
        rotvec = Rotation.from_quat(state[3:7], scalar_first=True).as_rotvec()
        base = np.concatenate([state[0:3], rotvec, state[7:23]])
    target = base.copy()
    target[2] -= 0.10  # (right) flange 10 cm down
    reach = []
    for i in range(45):
        env.step(target)
        s = np.asarray(env.obs["state"], dtype=np.float64)
        reach.append(round(float(np.linalg.norm(s[0:3] - target[0:3])), 4))
    closed = target.copy()
    flex = [1, 2, 3, 5, 6, 7, 9, 10, 11]  # proximal..distal of ff, mf, rf
    for j in flex:
        closed[6 + j] = 1.2
    closed[6 + 12] = 1.2  # thumb rotation
    hand = []
    for i in range(45):
        env.step(closed)
        s = np.asarray(env.obs["state"], dtype=np.float64)
        q = s[14:30] if dual else s[7:23]
        hand.append(round(float(np.abs(q[flex] - 1.2).max()), 4))
    report = {"task": task, "rotvec_at_start": base[3:6].round(4).tolist(), "fingertips_at_start": start_tips,
              "flange_error_m_after_steps": {n: reach[n - 1] for n in (1, 3, 5, 10, 15, 20, 30, 45)},
              "finger_flex_error_rad_after_steps": {n: hand[n - 1] for n in (1, 3, 5, 10, 15, 20, 30, 45)}}
    env.close()
    return report


if __name__ == "__main__":
    main()
