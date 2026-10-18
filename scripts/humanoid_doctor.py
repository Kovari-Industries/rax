#!/usr/bin/env python3
"""HumanoidArena backend feasibility and calibration check. Uses one GPU.

Runs the real host-side client (services.humanoid.process) against one simulator process on
an exploration layout and reports: startup seconds, the observation contract (camera, depth,
joints, Dex3 fingers), a standing hold through the frozen SONIC controller (tracking error,
tilt, wall seconds per 50 Hz step), the depth-pixel geometry, peak GPU memory and, unless
--no-planner, a cuRobo arm plan (preview, then execution). Writes a JSON report next to the run.

Usage: runtime/envs/host/bin/python scripts/humanoid_doctor.py --gpu 0 [--task opendoor]
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from services.humanoid import robot as R  # noqa: E402
from services.humanoid import tools as T  # noqa: E402
from services.humanoid.process import HumanoidProcess, runtime_root  # noqa: E402
from services.humanoid.tasks import EXPLORATION_COLLECTION, layout_seed  # noqa: E402


def gpu_memory_mb(gpu):
    out = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid,memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True, check=True).stdout
    for line in out.strip().splitlines():
        index, uuid, used = (x.strip() for x in line.split(","))
        if str(gpu) in (index, uuid):
            return int(used)
    return None


def hold_frame(observation):
    """A standing reference: measured joints, identity heading change, hands open (root height
    is part of the layout but ignored by the controller)."""
    frame = [0.0] * R.ACTION_DIM
    frame[R.ACTION_SLICES["root_height_m"]] = [0.74]
    frame[R.ACTION_SLICES["root_rot6d"]] = list(R.IDENTITY_ROT6D)
    frame[R.ACTION_SLICES["joints_canonical29_rad"]] = observation["proprio"]["joint_positions_rad"]
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gpu", required=True, help="GPU index or UUID for the simulator")
    parser.add_argument("--task", default="opendoor")
    parser.add_argument("--layout", type=int, default=0)
    parser.add_argument("--hold-steps", type=int, default=100)
    parser.add_argument("--no-planner", action="store_true")
    parser.add_argument("--no-depth", action="store_true")
    args = parser.parse_args()

    out = runtime_root() / "diagnostics" / "humanoid" / f"doctor_{time.strftime('%Y%m%dT%H%M%S')}"
    seed = layout_seed(args.task, EXPLORATION_COLLECTION, args.layout)
    report = {"task": args.task, "layout": args.layout, "seed": seed, "gpu": args.gpu, "checks": {}}
    checks = report["checks"]
    baseline = gpu_memory_mb(args.gpu)
    sim = HumanoidProcess(output_root=out)
    failed = None
    try:
        started = time.monotonic()
        reset = sim.start(task=args.task, collection=EXPLORATION_COLLECTION, seed=seed, run_id="doctor",
                          sim_gpu=args.gpu, startup_timeout=1800, include_depth=not args.no_depth)
        checks["startup_seconds"] = round(time.monotonic() - started, 1)
        checks["reset"] = reset
        report["metadata"] = sim.metadata
        observation = sim.request("observe")
        rgb = np.asarray(observation["cam_front"])
        proprio = observation["proprio"]
        checks["observation"] = {
            "rgb_shape": list(rgb.shape), "rgb_std": round(float(rgb.std()), 2),
            "joint_names_match": proprio["joint_names_canonical29"] == list(R.JOINT_NAMES),
            "hand_joints": len(proprio["hand_state"]["positions_rad"]),
            "root_orientation_world_wxyz": proprio["root_orientation_world_wxyz"],
            "keys": sorted(observation)}
        if not args.no_depth:
            depth = np.asarray(observation["cam_front_depth_m"])
            valid = depth[depth > 0]
            checks["depth"] = {"shape": list(depth.shape), "valid_fraction": round(float(valid.size / depth.size), 3),
                               "median_m": round(float(np.median(valid)), 3) if valid.size else None,
                               "intrinsics": observation["camera_intrinsics"]["cam_front"]["matrix"],
                               "camera_pose_pelvis": observation["camera_poses"]["cam_front"]}
            centre = T.pixel_to_pelvis(depth, observation["camera_intrinsics"]["cam_front"],
                                       observation["camera_poses"]["cam_front"], rgb.shape[1] / 2, rgb.shape[0] * 0.9)
            checks["depth"]["lower_centre_pixel_pelvis_m"] = centre.get("position_pelvis_m")
        # A standing hold: the measured posture as the reference, in 2 s chunks.
        frame, steps, wall = hold_frame(observation), 0, 0.0
        feedback = []
        while steps < args.hold_steps:
            rows = [frame] * min(R.MAX_ACTION_ROWS, args.hold_steps - steps)
            tick = time.monotonic()
            result = sim.request("chunk_step", actions=np.asarray(rows, dtype=np.float32))
            wall += time.monotonic() - tick
            steps += result["feedback"]["ticks"]
            feedback.append(result["feedback"])
            if result["terminated"] or result["truncated"]:
                checks["hold_ended"] = result.get("end_reason")
                break
        checks["hold"] = {"steps": steps, "wall_seconds_per_step": round(wall / max(steps, 1), 4),
                          "real_time_factor": round(steps * R.CONTROL_DT / max(wall, 1e-9), 3),
                          "final_tilt_deg": feedback[-1]["tilt_deg"], "fallen": feedback[-1]["fallen"],
                          "wall_seconds_split": {key: round(sum(f["wall_seconds"][key] for f in feedback), 2)
                                                 for key in ("control_steps", "observations")},
                          "joint_tracking_rmse_rad": [round(f["joint_tracking_rmse_rad"], 4) for f in feedback]}
        if not args.no_planner:
            target = {"arm": "right", "position": [0.30, -0.20, 0.15], "frame": "pelvis", "preview_only": True}
            tick = time.monotonic()
            plan = sim.request("plan_arm", **target)
            checks["planner_preview"] = {**plan["plan"], "call_seconds": round(time.monotonic() - tick, 2)}
            # A second plan from the same state measures the warm planner (solvers and graphs built).
            tick = time.monotonic()
            warm = sim.request("plan_arm", **target)
            checks["planner_warm"] = {"call_seconds": round(time.monotonic() - tick, 2), "success": warm["plan"]["success"],
                                      "planning_time_s": warm["plan"]["planning_time_s"]}
            plan = warm
            if plan.get("planned"):
                tick = time.monotonic()
                moved = sim.request("execute_motion_plan", motion_plan_id=plan["plan"]["motion_plan_id"])
                checks["planner_execute"] = {"call_seconds": round(time.monotonic() - tick, 2),
                                             "ticks": moved["feedback"]["ticks"], "tilt_deg": moved["feedback"]["tilt_deg"],
                                             "fallen": moved["feedback"]["fallen"],
                                             "joint_tracking_rmse_rad": moved["feedback"]["joint_tracking_rmse_rad"]}
        used = gpu_memory_mb(args.gpu)
        checks["gpu_memory_mb"] = {"baseline": baseline, "with_simulator": used}
        state = json.loads((sim.output_dir / "sim" / "operator_state.json").read_text())
        checks["operator_state"] = {"episode_id": state["episode_id"], "step_id": state["step_id"],
                                    "native_success": state["reward"]["native_success"]}
    except Exception as exc:  # noqa: BLE001 - the report records the whole failure
        import traceback
        failed = traceback.format_exc()
        report["error"] = failed
    finally:
        try:
            sim.finish("doctor") if sim.active else None
        except Exception as exc:  # noqa: BLE001
            report["finish_error"] = repr(exc)
        sim.stop()
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.json").write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(json.dumps(report, indent=2, default=str))
        print(f"report: {out / 'report.json'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
