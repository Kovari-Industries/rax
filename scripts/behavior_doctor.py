#!/usr/bin/env python3
"""BEHAVIOR-1K backend feasibility and calibration check (M0). Uses one GPU.

Runs the real host-side client (services.behavior.process) against one simulator process
on a train instance and reports: startup seconds, stepping rate, peak GPU memory, camera
intrinsics vs the published constants, projection consistency of the proprioceptive EEF
into the cameras, gripper command sign, odometry drift against the private true base pose,
and (unless --no-planner) a cuRobo arm plan. Writes a JSON report next to the run.

Usage: runtime/envs/robodojo/bin/python scripts/behavior_doctor.py --gpu 0 [--task turning_on_radio]
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from services.behavior import robot as R  # noqa: E402
from services.behavior import tools as T  # noqa: E402
from services.behavior.process import BehaviorProcess, runtime_root  # noqa: E402

PUBLISHED_INTRINSICS = {"cam_head": (306.0, 360.0), "cam_left_wrist": (388.6639, 240.0),
                        "cam_right_wrist": (388.6639, 240.0)}


def gpu_memory_mb(gpu):
    out = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid,memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True, check=True).stdout
    for line in out.strip().splitlines():
        index, uuid, used = (x.strip() for x in line.split(","))
        if str(gpu) in (index, uuid):
            return int(used)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gpu", required=True, help="GPU index or UUID for the simulator")
    parser.add_argument("--task", default="turning_on_radio")
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--no-planner", action="store_true")
    args = parser.parse_args()

    out = runtime_root() / "diagnostics" / "behavior" / f"doctor_{time.strftime('%Y%m%dT%H%M%S')}"
    report = {"task": args.task, "instance": args.instance, "gpu": args.gpu, "checks": {}}
    checks = report["checks"]
    baseline = gpu_memory_mb(args.gpu)
    sim = BehaviorProcess(output_root=out)
    try:
        started = time.monotonic()
        reset = sim.start(task=args.task, collection=0, instance_id=args.instance, run_id="doctor",
                          sim_gpu=args.gpu, startup_timeout=1800)
        checks["startup_seconds"] = round(time.monotonic() - started, 1)
        checks["native_step_limit"] = reset["native_step_limit"]
        checks["gpu_memory_mb_after_load"] = (gpu_memory_mb(args.gpu) or 0) - (baseline or 0)
        obs = sim.request("observe")
        checks["cameras"] = {c: list(np.asarray(obs[c]).shape) for c in R.CAMERAS}
        checks["depth"] = {c: list(np.asarray(obs[f"{c}_depth_m"]).shape) for c in R.CAMERAS if f"{c}_depth_m" in obs}
        checks["intrinsics"] = {c: {"live": obs["camera_intrinsics"][c], "published_f_c": PUBLISHED_INTRINSICS[c]}
                                for c in R.CAMERAS}
        # Projection consistency: each EEF (base frame) should project into the head camera
        # with depth close to its camera-frame distance.
        projection = {}
        for side in ("left", "right"):
            point = obs["proprio_fields"][f"eef_{side}_pos"]
            pixel = T.base_to_pixel(point, obs["camera_intrinsics"]["cam_head"], obs["camera_poses"]["cam_head"])
            entry = {"eef_base_m": point, "head_pixel": pixel}
            if pixel is not None:
                u, v, z = pixel
                depth = np.asarray(obs["cam_head_depth_m"])
                if 0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]:
                    entry["depth_at_pixel_m"] = float(depth[int(v), int(u)])
                    entry["projected_depth_m"] = z
                    back = T.pixel_to_base(depth, obs["camera_intrinsics"]["cam_head"], obs["camera_poses"]["cam_head"], u, v, 1)
                    entry["back_projected_m"] = back.get("position_m")
            projection[side] = entry
        checks["eef_head_projection"] = projection

        # Stepping rate with a hold action.
        hold = R.hold_action(obs["proprio_fields"], [obs["gripper_openings"]["left"], obs["gripper_openings"]["right"]])
        rows = np.repeat(hold[None], 60, axis=0).astype(np.float32)
        t = time.monotonic()
        result = sim.request("chunk_step", actions=rows)
        elapsed = time.monotonic() - t
        checks["hold_60_steps_seconds"] = round(elapsed, 2)
        checks["steps_per_second_with_frames"] = round(60 / elapsed, 1)
        checks["frames_returned"] = len(result["frames"])
        after = result["frames"][-1]
        drift = {k: float(np.abs(np.asarray(after["proprio_fields"][k]) - np.asarray(obs["proprio_fields"][k])).max())
                 for k in ("arm_left_qpos", "arm_right_qpos", "trunk_qpos")}
        checks["hold_joint_drift_rad"] = drift

        # Gripper sign: command -1 on the left gripper for 30 steps and watch the opening.
        grip = np.repeat(hold[None], 30, axis=0).astype(np.float32)
        grip[:, R.ACTION_SLICES["gripper_left"]] = -1.0
        result = sim.request("chunk_step", actions=grip)
        checks["gripper_left_after_minus_one"] = {
            "finger_qpos": result["frames"][-1]["proprio_fields"]["gripper_left_qpos"],
            "normalized_opening": result["frames"][-1]["gripper_openings"]["left"]}

        # Base move and odometry drift vs private truth.
        t = time.monotonic()
        move = sim.request("base_move", dx=0.5, dy=0.0, dyaw=0.5)
        checks["base_move"] = {k: move.get(k) for k in ("reached", "stalled", "final_position_error_m", "final_yaw_error_rad")}
        checks["base_move"]["steps"] = len(move["steps"])
        checks["base_move"]["seconds"] = round(time.monotonic() - t, 1)
        truth = [json.loads(line) for line in (out / "doctor" / "sim" / "private_base_truth.jsonl").read_text().splitlines()]
        if truth:
            first, last = truth[0], truth[-1]
            x0, y0, yaw0 = first["world_base_xy_yaw"]
            x1, y1, yaw1 = last["world_base_xy_yaw"]
            c, s = math.cos(yaw0), math.sin(yaw0)
            true_rel = (c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0),
                        math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
            # Odometry starts at the episode start; truth log starts after the first step.
            checks["odometry_vs_truth"] = {"odometry": last["odometry"], "truth_relative_to_first_logged": true_rel}

        if not args.no_planner:
            fields = sim.request("observe")["proprio_fields"]
            target = list(np.asarray(fields["eef_right_pos"]) + np.array([0.05, 0.0, 0.05]))
            t = time.monotonic()
            try:
                plan = sim.request("plan_arm", arm="right", position=target,
                                   quaternion_xyzw=fields["eef_right_quat"], preview_only=False)
                final = plan["frames"][-1]["proprio_fields"] if plan.get("frames") else sim.request("observe")["proprio_fields"]
                checks["planner"] = {"seconds": round(time.monotonic() - t, 1), "status": plan.get("status"),
                                     "target": target, "reached": final["eef_right_pos"],
                                     "error_m": float(np.linalg.norm(np.asarray(final["eef_right_pos"]) - target))}
            except Exception as exc:  # noqa: BLE001 - report, do not hide
                checks["planner"] = {"error": f"{type(exc).__name__}: {exc}", "seconds": round(time.monotonic() - t, 1)}
            fields = sim.request("observe")["proprio_fields"]
            goal = list(np.asarray(fields["eef_left_pos"]) + np.array([0.08, 0.0, 0.06]))
            move = sim.request("eef_move", arm="left", position=goal, quaternion_xyzw=fields["eef_left_quat"])
            checks["eef_move"] = {k: move.get(k) for k in ("reached", "final_position_error_m", "final_orientation_error_rad")}
            checks["eef_move"]["steps"] = len(move["steps"])
        checks["gpu_memory_mb_peak_delta"] = (gpu_memory_mb(args.gpu) or 0) - (baseline or 0)
        state = json.loads((out / "doctor" / "sim" / "operator_state.json").read_text())
        checks["operator_q_score"] = state["reward"]["q_score"]
        checks["operator_predicates"] = state["reward"]["goal_predicates"]
    finally:
        try:
            if sim.active:
                sim.finish("doctor")
        finally:
            sim.stop()
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.json").write_text(json.dumps(report, indent=2, default=str))
        print(json.dumps(report, indent=2, default=str))
        print(f"report: {out / 'report.json'}")


if __name__ == "__main__":
    main()
