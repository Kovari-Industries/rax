"""DexJoCo doctor: one real official simulator per task through the host process manager.

For each task: simulator startup, the observation contract (official state, evaluation
cameras at 640x640, depth and calibration), a hold and an official action chunk through
``chunk_step``, depth back-projection of the measured flange into the overview camera, the
operator files (operator_state, live frame, sensors.mp4, outcome), and the simulator-side
checks (official layout determinism and timing). Writes a JSON report.

  runtime/envs/host/bin/python scripts/dexjoco_doctor.py --gpu GPU-... --task pick_bucket
"""
import argparse
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))


def rotvec(q_wxyz):
    w, *xyz = (float(v) for v in q_wxyz)
    axis = np.asarray(xyz)
    norm = float(np.linalg.norm(axis))
    return (axis / norm * 2 * np.arctan2(norm, w)) if norm > 1e-12 else np.zeros(3)


def hold_row(state):
    s = np.asarray(state, dtype=np.float64)
    if len(s) == 46:
        return np.concatenate([s[0:3], rotvec(s[3:7]), s[14:30], s[7:10], rotvec(s[10:14]), s[30:46]])
    return np.concatenate([s[0:3], rotvec(s[3:7]), s[7:23]])


def doctor_task(task, gpu, root, collection):
    from services.dexjoco import tasks as T
    from services.dexjoco.process import DexjocoProcess, dexjoco_python, simulator_environment
    from services.dexjoco.tools import pixel_to_world, world_to_pixel

    report = {"task": task, "collection": collection}
    sim = DexjocoProcess(output_root=root / "results")
    run_id = f"{task}_doctor_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{uuid.uuid4().hex[:6]}"
    started = time.monotonic()
    reset = sim.start(task=task, collection=collection, layout_id=0, run_id=run_id, sim_gpu=gpu,
                      startup_timeout=T.STARTUP_SECONDS, include_depth=True, episode_timeout_seconds=1800)
    report["startup_seconds"] = round(time.monotonic() - started, 2)
    report["reset"] = reset
    report["metadata"] = sim.metadata
    try:
        obs = sim.request("observe")
        cameras = sim.metadata["cameras"]
        report["observation"] = {
            "state_dim": len(obs["state"]), "cameras": cameras,
            "image_shapes": {c: list(np.asarray(obs[c]).shape) for c in cameras},
            "depth": {c: {"finite": float(np.isfinite(obs[f"{c}_depth_m"]).mean()),
                          "median_m": float(np.median(obs[f"{c}_depth_m"][np.isfinite(obs[f"{c}_depth_m"])]))}
                      for c in cameras if f"{c}_depth_m" in obs},
            "instruction": obs.get("instruction")}
        assert len(obs["state"]) == T.state_dim(task), obs["state"]
        # The measured flange projected into the overview camera and back-projected from depth.
        overview = cameras[0]
        flange = np.asarray(obs["state"][0:3])
        pixel = world_to_pixel(flange, obs["camera_intrinsics"][overview], obs["camera_poses"][overview])
        report["flange_projection"] = {"camera": overview, "pixel": pixel}
        if pixel is not None:
            back = pixel_to_world(obs[f"{overview}_depth_m"], obs["camera_intrinsics"][overview],
                                  obs["camera_poses"][overview], pixel[0], pixel[1], 0)
            if back.get("valid"):
                # The flange lies inside the hand; the visible surface at its pixel is a few cm in front.
                report["flange_projection"].update(
                    back_projected_m=back["position_m"], depth_m=back["depth_m"], projected_depth_m=pixel[2],
                    distance_to_flange_m=round(float(np.linalg.norm(np.asarray(back["position_m"]) - flange)), 4))
        # The first object's true initial position (operator-only state tail) against its depth
        # back-projection in every camera that sees it: the surface lies within the object's size.
        private = json.loads((sim.output_dir / "sim" / "private_initial_state.json").read_text())
        target = np.asarray(private["state"][T.state_dim(task):T.state_dim(task) + 3])
        checks = {}
        for camera in cameras:
            pixel = world_to_pixel(target, obs["camera_intrinsics"][camera], obs["camera_poses"][camera])
            if pixel is None or not (0 <= pixel[0] < 640 and 0 <= pixel[1] < 640):
                checks[camera] = {"visible": False}
                continue
            back = pixel_to_world(obs[f"{camera}_depth_m"], obs["camera_intrinsics"][camera],
                                  obs["camera_poses"][camera], pixel[0], pixel[1], 1)
            checks[camera] = {"pixel": [round(pixel[0], 1), round(pixel[1], 1)], "valid": back.get("valid"),
                              "distance_m": round(float(np.linalg.norm(np.asarray(back["position_m"]) - target)), 4)
                              if back.get("valid") else None}
        report["object_projection"] = {"object_position_m": target.round(4).tolist(), "cameras": checks}
        started = time.monotonic()
        held = sim.request("hold", steps=30)
        report["hold"] = {"step_id": held["step_id"], "frames": held["frame_count"],
                          "seconds_per_step": round((time.monotonic() - started) / 30, 4)}
        row = hold_row(held["frames"][-1]["state"])
        target = row.copy()
        target[2] -= 0.05
        chunk = np.stack([row + (target - row) * (i + 1) / 30 for i in range(30)] + [target] * 15)
        started = time.monotonic()
        moved = sim.request("chunk_step", actions=chunk)
        report["chunk"] = {"step_id": moved["step_id"], "frames": moved["frame_count"],
                           "seconds_per_step": round((time.monotonic() - started) / len(chunk), 4),
                           "flange_z_drop_m": round(float(np.asarray(held["frames"][-1]["state"])[2]
                                                          - np.asarray(moved["frames"][-1]["state"])[2]), 4),
                           "terminated": moved["terminated"]}
        sim.finish("doctor")
        sim_dir = sim.output_dir / "sim"
        state = json.loads((sim_dir / "operator_state.json").read_text())
        outcome = json.loads((sim_dir / "evaluation_outcome.json").read_text())
        video = sim_dir / "sensors.mp4"
        report["operator_files"] = {
            "operator_state": {k: state.get(k) for k in ("episode_mode", "step_id", "episode_step_limit", "cameras",
                                                         "official_seed", "official_episode_index")},
            "reward": state.get("reward"), "outcome": {k: outcome.get(k) for k in ("native_score", "native_success",
                                                                                   "status", "reason")},
            "live_npz": (sim_dir / state["observation_file"]).is_file(),
            "sensors_mp4_bytes": video.stat().st_size if video.is_file() else 0,
            "goal_report": (sim_dir / "private_goal_report.json").is_file()}
    finally:
        sim.stop()
    env = simulator_environment(gpu)
    checks = subprocess.run([dexjoco_python(), "-m", "services.dexjoco.checks", "--task", task, "--episodes", "3"],
                            env=env, cwd=PROJECT, capture_output=True, text=True, timeout=1800)
    lines = [line for line in checks.stdout.splitlines() if line.startswith("{")]
    report["simulator_checks"] = json.loads(lines[-1]) if lines else {"returncode": checks.returncode,
                                                                      "stderr": checks.stderr[-2000:]}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gpu", required=True, help="Simulator GPU UUID or index")
    parser.add_argument("--task", action="append", required=True)
    parser.add_argument("--collection", type=int, default=1, choices=(0, 1))
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    root = Path(args.output or PROJECT / "runtime" / "diagnostics" / f"dexjoco-doctor-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}")
    root.mkdir(parents=True, exist_ok=True)
    reports = []
    for task in args.task:
        try:
            reports.append(doctor_task(task, args.gpu, root, args.collection))
        except Exception as exc:  # noqa: BLE001 - report every task, then fail
            reports.append({"task": task, "error": f"{type(exc).__name__}: {exc}"})
    (root / "doctor-report.json").write_text(json.dumps(reports, indent=2, default=str) + "\n")
    print(json.dumps(reports, indent=2, default=str))
    if any("error" in r for r in reports):
        sys.exit(1)


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
