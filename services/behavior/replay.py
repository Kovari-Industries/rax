"""Replay of a finished BEHAVIOR episode from its recorded actions, for its camera frames.

Runs only in ``runtime/envs/behavior``, like ``server.py``. It resets the episode's task
instance under the same evaluator configuration, steps every recorded
``<episode>/action_NNNNNN.json`` row in order, and saves the head and wrist RGB frames at the
requested steps and at the end. Physics need not repeat exactly, so every replayed base pose
is compared with the run's recorded one (``private_base_truth.jsonl``), and the replay's final
goal conditions with the run's goal report. ``replay.json`` records both comparisons and the
saved frames.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# A replayed base further than this from the recorded one no longer shows the run.
DIVERGED_M = 0.05


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", required=True, help="the run's simulator directory (holds reset.json)")
    parser.add_argument("--output", required=True)
    parser.add_argument("--steps", default="", help="comma-separated steps whose frames to save")
    return parser.parse_args(argv)


def conditions(report):
    """Each goal option's final truth values, from a goal report."""
    return [[bool(c["final"]) for c in option] for option in report.get("options", [])]


def base_error(replayed, recorded):
    dx, dy = replayed[0] - recorded[0], replayed[1] - recorded[1]
    yaw = (replayed[2] - recorded[2] + math.pi) % (2 * math.pi) - math.pi
    return math.hypot(dx, dy), abs(yaw)


def main(argv=None):
    args = parse_args(argv)
    sim, output = Path(args.sim), Path(args.output)
    (output / "frames").mkdir(parents=True, exist_ok=True)
    reset = json.loads((sim / "reset.json").read_text())
    recorded_report = json.loads((sim / "private_goal_report.json").read_text())
    actions = sorted((sim / reset["episode_id"]).glob("action_*.json"))
    truth = {}
    with (sim / "private_base_truth.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            truth[row["step_id"]] = row["world_base_xy_yaw"]
    keys = {int(step) for step in args.steps.split(",") if step}

    from services.behavior.server import build_evaluator, preload_planner_libraries
    preload_planner_libraries()
    evaluator = build_evaluator(reset["task"], reset["mode"])
    import numpy as np
    from PIL import Image
    from services.behavior import robot as R
    from services.behavior.session import BehaviorSession

    session = BehaviorSession(evaluator, output / "sim", reset["task"], mode=reset["mode"], include_depth=False)
    session.reset(reset["instance_id"], source="replay", policy_version="replay")
    # Only the requested frames are kept, so no video is encoded at every step. The video only
    # reads observations and never touches physics.
    evaluator._write_video = lambda state: None
    frames = []

    def save(label):
        observation = session._agent_observation()
        for cam in R.CAMERAS:
            path = output / "frames" / f"{label}_{cam}.png"
            Image.fromarray(observation[cam]).save(path)
            frames.append({"step": session.step_id, "camera": cam, "path": str(path.relative_to(output))})

    record = {"episode_id": reset["episode_id"], "task": reset["task"], "instance_id": reset["instance_id"],
              "recorded_steps": len(actions), "started_at": time.time()}
    if 0 in keys:
        save("step-000000")
    worst = {"position_m": 0.0, "yaw_rad": 0.0, "step": None}
    diverged, compared = None, 0
    for expected, path in enumerate(actions, start=1):
        row = json.loads(path.read_text())
        if row["step_id"] != expected:
            raise RuntimeError(f"{path.name} records step {row['step_id']}, expected {expected}")
        ended = session._step_row(np.asarray(row["executed_action"], dtype=np.float32))
        recorded = truth.get(session.step_id)
        if recorded is not None:
            position, yaw = base_error(session._world_pose(), recorded)
            compared += 1
            if position > worst["position_m"]:
                worst.update(position_m=position, yaw_rad=yaw, step=session.step_id)
            if diverged is None and position > DIVERGED_M:
                diverged = session.step_id
        if session.step_id in keys:
            save(f"step-{session.step_id:06d}")
        if ended:
            break
    save("final")
    session.finish("replay")
    replayed_report = json.loads((output / "sim" / "private_goal_report.json").read_text())
    expected_truth, replayed_truth = conditions(recorded_report), conditions(replayed_report)
    record.update({
        "replayed_steps": session.step_id, "compared_steps": compared, "worst_base_error": worst,
        "diverged_at_step": diverged,
        "diverged_threshold_m": DIVERGED_M,
        "goal_conditions_match": expected_truth == replayed_truth,
        "recorded_goal_conditions": expected_truth, "replayed_goal_conditions": replayed_truth,
        "recorded_success": recorded_report.get("success"), "replayed_success": replayed_report.get("success"),
        "frames": frames, "finished_at": time.time()})
    (output / "replay.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"event": "replayed", **{k: record[k] for k in (
        "replayed_steps", "recorded_steps", "compared_steps", "diverged_at_step",
        "goal_conditions_match")}}), flush=True)
    import omnigibson as og
    evaluator.env.close()
    og.shutdown()


if __name__ == "__main__":
    main()
