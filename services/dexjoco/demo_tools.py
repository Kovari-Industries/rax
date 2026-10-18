"""Simulator-side processing of official DexJoCo raw demonstrations (runs in runtime/envs/dexjoco).

``python -m services.dexjoco.demo_tools --task pick_bucket --demo DIR [--demo DIR ...]`` prints one
JSON line per demo after:

- exporting ``trajectory.npz`` (``action_rotvec``, ``action``, ``state``, ``timestamp``) from the
  official ``replay.zarr`` and keyframes of the evaluation base camera from ``videos/<camera>.mp4``;
- replaying the recorded raw actions in the official environment from the recorded initial state,
  as the official ``scripts/replay_demos_zarr.py`` does (holding the last action up to
  ``--extend-steps`` more steps, the known end-of-recording offset);
- comparing the recorded initial layout with every official evaluation layout (formal), so an
  expert episode never shows a held-out layout.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

KEYFRAMES = 8


def export(demo, out, camera):
    import imageio
    import zarr

    root = zarr.open(str(demo / "replay.zarr"), mode="r")
    data = root["data"]
    arrays = {name: np.asarray(data[name]) for name in ("action_rotvec", "action", "state", "timestamp") if name in data}
    np.savez_compressed(out / "trajectory.npz", **arrays)
    video = demo / "videos" / f"{camera}.mp4"
    frames_dir = out / "frames"
    frames_dir.mkdir(exist_ok=True)
    reader = imageio.get_reader(str(video))
    images = [np.asarray(frame) for frame in reader]
    reader.close()
    picks = sorted({int(round(i * (len(images) - 1) / (KEYFRAMES - 1))) for i in range(KEYFRAMES)})
    index = []
    for step in picks:
        name = f"frames/frame_{step:06d}.jpg"
        imageio.imwrite(out / name, images[step][..., :3], quality=92)
        index.append({"step": step, "file": name})
    (frames_dir / "index.json").write_text(json.dumps({"camera": camera, "frames": index}, indent=2) + "\n")
    return {"steps": int(len(arrays["action"])), "video_frames": len(images), "keyframes": len(index)}


def replay(task, demo, extend_steps):
    import zarr
    from dexjoco.tasks import CONFIG_MAPPING
    from dexjoco.tasks.state_restorers import restore_initial_state
    from services.dexjoco import tasks as T

    info = T.task_info(task)
    root = zarr.open(str(demo / "replay.zarr"), mode="r")
    actions = np.asarray(root["data"]["action"])
    initial = np.asarray(root["data"]["state"][0]).ravel()
    config = CONFIG_MAPPING[info["env_name"]]()
    env = config.get_environment(policy_mode=True, render_mode="rgb_array", randomize=info["regime"] == "rand_full",
                                 randomize_dynamics=False, seed=0)
    try:
        env.reset()
        restore_initial_state(env, info["env_name"], config, initial)
        raw = env.unwrapped
        succeeded_at = None
        for index in range(len(actions) + extend_steps):
            row = np.asarray(actions[min(index, len(actions) - 1)], dtype=np.float64)
            action = {"right": row[:23], "left": row[23:46]} if row.shape == (46,) else row
            _obs, _reward, done, _truncated, step_info = raw.step(action)
            if step_info.get("succeed", False):
                succeeded_at = index + 1
                break
            if done:
                break
    finally:
        env.close()
    return {"replay_success": succeeded_at is not None, "replay_success_step": succeeded_at,
            "recorded_steps": int(len(actions)), "held_last_action_steps": max(0, (succeeded_at or 0) - len(actions))}


def formal_tails(task):
    """The privileged state tail (initial object poses, table height) of every official layout."""
    from services.dexjoco import tasks as T
    from services.dexjoco.checks import build

    dim = T.state_dim(task)
    tails = []
    for seed in T.OFFICIAL_SEEDS:
        env, tap = build(task, seed)
        for _ in range(T.OFFICIAL_EPISODES_PER_SEED):
            env.reset()
            tails.append(np.asarray(tap.last_obs["state"], dtype=np.float64)[dim:])
        env.close()
    return np.stack(tails)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--demo", action="append", required=True, help="raw demo directory (replay.zarr, videos/)")
    parser.add_argument("--out", action="append", required=True, help="export directory, one per --demo")
    parser.add_argument("--camera", required=True, help="evaluation base camera video to take keyframes from")
    parser.add_argument("--extend-steps", type=int, default=40)
    args = parser.parse_args(argv)
    os.environ.setdefault("MUJOCO_GL", "egl")
    from services.dexjoco import tasks as T

    dim = T.state_dim(args.task)
    tails = formal_tails(args.task)
    import zarr
    for demo, out in zip(map(Path, args.demo), map(Path, args.out), strict=True):
        out.mkdir(parents=True, exist_ok=True)
        report = {"demo": demo.name}
        try:
            report.update(export(demo, out, args.camera))
            initial = np.asarray(zarr.open(str(demo / "replay.zarr"), mode="r")["data"]["state"][0]).ravel()[dim:]
            distance = float(np.abs(tails - initial).max(axis=1).min()) if len(initial) == tails.shape[1] else None
            report["formal_min_distance"] = distance
            report["formal_overlap"] = distance is None or distance < 1e-3
            report.update(replay(args.task, demo, args.extend_steps))
        except Exception as exc:  # noqa: BLE001 - report each demo
            report["error"] = f"{type(exc).__name__}: {exc}"
        print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
