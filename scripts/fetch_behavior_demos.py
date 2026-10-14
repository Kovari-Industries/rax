"""Cache a few official BEHAVIOR 2026 teleoperation demos per task as head-camera keyframes.

Runs in the BEHAVIOR env (huggingface_hub, pandas, PyAV); use setup_behavior_demos.sh.
Only the head RGB segment of each selected episode is streamed (HTTP range reads), never
the 3.27 TB dataset. Keyframes are the episode start, the end of every annotated skill
and fillers so no gap exceeds MAX_GAP_FRAMES. Images only reach agent workspaces; the
demo's actions, states and annotations stay in this operator cache.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import av
import pandas as pd
from huggingface_hub import HfApi, hf_hub_download, hf_hub_url

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from services.behavior.tasks import task_names  # noqa: E402

REPO = "behavior-1k/2026-challenge-demos"
HEAD_VIDEO = "videos/observation.rgb.zed_link_camera_0"
MAX_GAP_FRAMES = 300  # 10 s at 30 Hz
MAX_FRAMES = 48
JPEG_QUALITY = 88


def download(path, revision):
    for attempt in range(5):
        try:
            return Path(hf_hub_download(REPO, path, repo_type="dataset", revision=revision))
        except Exception:
            if attempt == 4:
                raise
            time.sleep(5 * (attempt + 1))


def task_index_map(revision):
    rows = [json.loads(line) for line in download("meta/tasks.jsonl", revision).read_text().splitlines() if line]
    return {row["task_name"]: row["task_index"] for row in rows}


def select_episodes(episodes, count):
    """Episodes nearest the median length, preferring distinct task instances (layouts)."""
    median = episodes["length"].median()
    ranked = episodes.assign(gap=(episodes["length"] - median).abs()).sort_values(["gap", "episode_index"])
    chosen, instances = [], set()
    for _, row in ranked.iterrows():
        if row["task_instance_id"] not in instances:
            chosen.append(row)
            instances.add(row["task_instance_id"])
        if len(chosen) == count:
            return chosen
    rest = [row for _, row in ranked.iterrows() if all(row["episode_index"] != c["episode_index"] for c in chosen)]
    return chosen + rest[:count - len(chosen)]


def keyframes(length, skills):
    """Start, each skill's last frame, the final frame, and evenly spaced fillers."""
    marks = {0: "start", length - 1: "end"}
    for skill in skills:
        # frame_duration is [start, end], or a list of such spans for an interrupted skill.
        spans = skill["frame_duration"]
        last = max(span[1] for span in spans) if isinstance(spans[0], list) else spans[1]
        end = min(max(int(last) - 1, 0), length - 1)
        marks.setdefault(end, " / ".join(skill["skill_description"]))
    ordered = sorted(marks)
    frames = []
    for a, b in zip(ordered, ordered[1:]):
        frames.append(a)
        pieces = -(-(b - a) // MAX_GAP_FRAMES)
        frames += [a + (b - a) * k // pieces for k in range(1, pieces)]
    frames.append(ordered[-1])
    if len(frames) > MAX_FRAMES:  # very long demos: keep an even subset, always the final frame
        frames = [frames[round(k * (len(frames) - 1) / (MAX_FRAMES - 1))] for k in range(MAX_FRAMES)]
    return [(frame, marks.get(frame)) for frame in frames]


def decode(url, start, wanted, fps, target):
    """Write the wanted episode-relative frames of a concatenated LeRobot video."""
    pending, written = set(wanted), {}
    for attempt in range(4):
        try:
            with av.open(url, timeout=60) as container:
                stream = container.streams.video[0]
                container.seek(int(start / stream.time_base), stream=stream, backward=True)
                for frame in container.decode(stream):
                    index = round((float(frame.pts * stream.time_base) - start) * fps)
                    if index in pending:
                        name = f"frame_{index:06d}.jpg"
                        frame.to_image().save(target / name, quality=JPEG_QUALITY)
                        written[index] = name
                        pending.discard(index)
                    if not pending or index > max(wanted):
                        break
            break
        except Exception:
            if attempt == 3:
                raise
            time.sleep(5 * (attempt + 1))
    if pending:
        raise RuntimeError(f"Missing frames {sorted(pending)[:5]} in {url}")
    return written


def fetch_task(task, task_index, root, count, revision):
    destination = root / task
    if (destination / "demos.json").is_file():
        return f"{task}: cached"
    chunk = f"chunk-{task_index:03d}"
    files = sorted(f.path for f in HfApi().list_repo_tree(REPO, path_in_repo=f"meta/episodes/{chunk}",
                                                         repo_type="dataset", revision=revision))
    episodes = pd.concat([pd.read_parquet(download(path, revision)) for path in files])
    staging = root / f".{task}.{uuid.uuid4().hex}.tmp"
    staging.mkdir(parents=True)
    try:
        demos = []
        for number, row in enumerate(select_episodes(episodes, count), 1):
            annotation = json.loads(download(row["annotation_path"], revision).read_text())
            length = int(row["length"])
            frames = keyframes(length, annotation.get("skill_annotation", []))
            directory = f"demo-{number}"
            (staging / directory).mkdir()
            video = (f"{HEAD_VIDEO}/chunk-{int(row[f'{HEAD_VIDEO}/chunk_index']):03d}/"
                     f"file-{int(row[f'{HEAD_VIDEO}/file_index']):03d}.mp4")
            written = decode(hf_hub_url(REPO, video, repo_type="dataset", revision=revision),
                             float(row[f"{HEAD_VIDEO}/from_timestamp"]), [f for f, _ in frames], 30, staging / directory)
            demos.append({
                "directory": directory, "episode_index": int(row["episode_index"]),
                "raw_episode_id": int(row["raw_episode_id"]), "task_instance_id": int(row["task_instance_id"]),
                "length": length, "fps": 30, "camera": "cam_head",
                "frames": [{"image": f"{directory}/{written[f]}", "frame_index": f, "skill_end": skill}
                           for f, skill in frames],
            })
        index = {"schema_version": 1, "task": task, "task_index": task_index, "dataset": REPO,
                 "revision": revision, "demos": demos}
        (staging / "demos.json").write_text(json.dumps(index, indent=2) + "\n")
        if destination.exists():
            shutil.rmtree(destination)
        os.replace(staging, destination)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return f"{task}: {', '.join(str(len(d['frames'])) for d in demos)} keyframes"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--root", type=Path, default=PROJECT / "runtime/reference-demos/behavior")
    parser.add_argument("--episodes", type=int, default=2, choices=(1, 2, 3))
    parser.add_argument("--tasks", default="", help="comma-separated subset (default: all 100)")
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    tasks = [t for t in args.tasks.split(",") if t] or list(task_names())
    indices = task_index_map(args.revision)
    missing = [t for t in tasks if t not in indices]
    if missing:
        raise SystemExit(f"Not in the demo dataset: {missing}")
    args.root.mkdir(parents=True, exist_ok=True)
    failures = []
    with ThreadPoolExecutor(args.workers) as pool:
        futures = {t: pool.submit(fetch_task, t, indices[t], args.root, args.episodes, args.revision) for t in tasks}
        for task, future in futures.items():
            try:
                print(future.result(), flush=True)
            except Exception as error:
                failures.append(task)
                print(f"{task}: FAILED {error}", flush=True)
    if failures:
        raise SystemExit(f"{len(failures)} task(s) failed; rerun to resume: {','.join(failures)}")


if __name__ == "__main__":
    main()
