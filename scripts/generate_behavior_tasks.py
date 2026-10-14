#!/usr/bin/env python3
"""Regenerate the BEHAVIOR task catalog and task rules for all 2026-challenge tasks.

Reads the pinned 2026 task-instance metadata ($OMNIGIBSON_DATA_PATH/2026-challenge-task-instances),
the upstream challenge gallery (BEHAVIOR-1K/docs/challenge/task_data.json), the BDDL definitions
and the official 2025 challenge results (BEHAVIOR-1K/docs/challenge_submissions, standard track,
public test set). Writes services/behavior/tasks.json and, for each task without one,
services/behavior/task_rules/<task>.md; existing (reviewed) rule files are kept. Stdlib only.
"""
import argparse
import csv
import json
import os
import re
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def results_2025(source):
    """Per-task official 2025 results over the standard-track public-test submissions."""
    submissions = [json.loads(p.read_text()) for p in (source / "docs" / "challenge_submissions").glob("*.json")]
    standard = [s for s in submissions if s["track"] == "standard" and s["testset"] == "public"]
    tasks = {}
    for submission in standard:
        scores = submission["per_task_scores"]
        for task, q in scores["q_score"].items():
            entry = tasks.setdefault(task, {"q": [], "sr": []})
            entry["q"].append(q)
            entry["sr"].append(scores.get("task_sr", {}).get(task, 0.0))
    return {task: {"teams": len(v["q"]), "best_q_score": round(max(v["q"]), 4),
                   "top4_mean_q_score": round(statistics.mean(sorted(v["q"])[-4:]), 4),
                   "best_success_rate": round(max(v["sr"]), 4)}
            for task, v in tasks.items()}


def rules_text(task, instruction, bddl):
    goal = bddl[bddl.index("(:goal"):].rstrip().rstrip(")").rstrip()
    init = bddl[bddl.index("(:init"):bddl.index("(:goal")].rstrip()
    lead = (f"**Goal:** {instruction}\n\n" if instruction else
            "**Goal:** satisfy every condition of the BDDL goal below (the official task definition).\n\n")
    return (lead + "**Success:** every goal condition below holds at the same time. The episode then ends with\n"
            "`task_complete: true`. Nothing else is scored for formal success; partial progress is not\n"
            "reported to you.\n\n"
            f"**BDDL goal** (the official task definition, `problem0.bddl`):\n\n```lisp\n{goal}\n```\n\n"
            f"**BDDL initial conditions:**\n\n```lisp\n{init}\n```\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default=os.environ.get("OMNIGIBSON_DATA_PATH", ROOT / "runtime" / "behavior-data"))
    parser.add_argument("--source", default=ROOT / "BEHAVIOR-1K")
    parser.add_argument("--output", default=ROOT / "services" / "behavior" / "tasks.json")
    parser.add_argument("--rules", default=ROOT / "services" / "behavior" / "task_rules")
    args = parser.parse_args()
    source, rules = Path(args.source), Path(args.rules)
    instances = Path(args.data) / "2026-challenge-task-instances"
    meta = instances / "metadata"
    lengths = {row["task_name"]: row for row in map(json.loads, (meta / "task.jsonl").read_text().splitlines())}
    with (meta / "B100_task_misc.csv").open() as stream:
        misc = list(csv.DictReader(stream))
    gallery = json.loads((source / "docs" / "challenge" / "task_data.json").read_text())
    gallery = {row["id"]: row for row in (gallery if isinstance(gallery, list) else gallery["tasks"])}
    official = results_2025(source)
    tasks, written = {}, 0
    for row in misc:
        task = row["Task"]
        scene = gallery[task]["scene_model"]
        folder = instances / "scenes" / scene / "json" / f"{scene}_task_{task}_instances"
        train = sorted(int(m.group(1)) for p in folder.glob("*-tro_state.json")
                       for m in [re.search(r"_0_(\d+)_template-tro_state\.json$", p.name)] if m)
        if train != list(range(train[0], train[-1] + 1)):
            raise SystemExit(f"{task}: train instances are not contiguous")
        instruction = gallery[task].get("instruction") or None
        tasks[task] = {
            "task_id": int(row["Task ID"]),
            "title": gallery[task]["name"],
            "scene_model": scene,
            "rooms": row["Rooms to inlcude"].split(),
            "instruction": instruction,
            "gallery_duration_seconds": gallery[task].get("duration"),
            "gallery_video": gallery[task].get("video"),
            "human_mean_length_steps": lengths[task]["length"],
            "human_distance_traveled_m": lengths[task]["distance_traveled"],
            # Contiguous in every 2026 task (0-299 or 1-300): stored as an inclusive range.
            "train_instance_range": [train[0], train[-1]],
            # Official 2025 challenge results (standard track, public test); null for tasks new in 2026.
            "results_2025": official.get(task),
        }
        path = rules / f"{task}.md"
        if not path.exists():
            bddl = (source / "bddl3" / "bddl" / "activity_definitions" / task / "problem0.bddl").read_text()
            path.write_text(rules_text(task, instruction, bddl))
            written += 1
    catalog = {"schema": "behavior_task_catalog_v2", "source_commit": os.environ.get("BEHAVIOR_COMMIT"),
               "challenge": "BEHAVIOR 2026 (100 tasks)", "tasks": tasks}
    Path(args.output).write_text(json.dumps(catalog, indent=2, sort_keys=True) + "\n")
    write_table(tasks, ROOT / "docs" / "BEHAVIOR_TASKS.md")
    print(f"wrote {len(tasks)} tasks to {args.output}; {written} new task rule files")


def write_table(tasks, path):
    """Operator reference: every task with its limits and the official 2025 results."""
    def pct(value):
        return "" if value is None else f"{100 * value:.0f}"
    lines = ["# BEHAVIOR 2026 challenge tasks", "",
             "Generated by `scripts/generate_behavior_tasks.py`; do not edit by hand.", "",
             "- **Step limit**: the official timeout, 1.5 x the mean human demonstration length (30 Hz).",
             "- **2025 results**: official BEHAVIOR Challenge 2025 per-task scores on the public test set,",
             "  over the standard-track submissions (RGB-D + proprioception policies): best q-score,",
             "  mean q-score of the four best submissions on that task, and best success rate, in percent.",
             "  Tasks 50-99 are new in 2026 and have no official results yet (the 2026 leaderboard",
             "  publishes verified results after the submission deadline).",
             "- q-score is the fraction of BDDL goal predicates newly satisfied at episode end (1 on success);",
             "  it is the 2026 ranking metric.", "",
             "| ID | Task | Scene | Human demo | Step limit | 2025 best q | 2025 top-4 q | 2025 best success |",
             "|---|---|---|---|---|---|---|---|"]
    for task, info in sorted(tasks.items(), key=lambda item: item[1]["task_id"]):
        r = info["results_2025"] or {}
        lines.append(f"| {info['task_id']} | `{task}` | {info['scene_model']} | "
                     f"{info['human_mean_length_steps'] / 30 / 60:.1f} min | {int(info['human_mean_length_steps'] * 1.5)} | "
                     f"{pct(r.get('best_q_score'))} | {pct(r.get('top4_mean_q_score'))} | {pct(r.get('best_success_rate'))} |")
    path.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
