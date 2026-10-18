"""Install official DexJoCo demonstrations for the harness (images and expert episodes).

For each task, downloads candidate raw demonstrations from the official Hugging Face dataset
(``DexJoCo/DexJoCo-Datasets-Raw``: the rand_obj set for ``<task>``, the rand_full set for
``<task>_random``) at a pinned revision, then runs ``services.dexjoco.demo_tools`` in the official
environment (needs a GPU for rendering) to export trajectories and keyframes, replay each demo
and compare its layout with the 150 official evaluation layouts. The first three candidates that
replay to success and show no evaluation layout become the task's expert episodes, copied with
sha256 manifests to runtime/reference-demos/dexjoco/<task>/experts/.

  runtime/envs/host/bin/python scripts/setup_dexjoco_demos.py --gpu GPU-... --task pick_bucket
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

REPO = "DexJoCo/DexJoCo-Datasets-Raw"
API = f"https://huggingface.co/api/datasets/{REPO}"
SCHEMA = "dexjoco_demos_v1"
EXPERTS = 3


def locked_revision():
    for line in (PROJECT / "dependencies.lock").read_text().splitlines():
        if line.startswith("DEXJOCO_DEMOS_REVISION="):
            return line.split("=", 1)[1].strip()
    raise SystemExit("DEXJOCO_DEMOS_REVISION missing from dependencies.lock")


def get_json(url):
    with urllib.request.urlopen(url, timeout=120) as response:
        return json.loads(response.read())


def tree(path, rev, recursive=False):
    query = "?recursive=true" if recursive else ""
    return get_json(f"{API}/tree/{rev}/{urllib.parse.quote(path)}{query}")


def download(path, target, rev):
    url = f"https://huggingface.co/datasets/{REPO}/resolve/{rev}/{urllib.parse.quote(path)}"
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(target.suffix + ".partial")
    with urllib.request.urlopen(url, timeout=600) as response, partial.open("wb") as stream:
        shutil.copyfileobj(response, stream)
    partial.replace(target)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def file_hashes(directory):
    return {p.relative_to(directory).as_posix(): sha256(p) for p in sorted(directory.rglob("*")) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gpu", required=True, help="GPU index or UUID for the official environment (rendering)")
    parser.add_argument("--task", action="append", required=True)
    parser.add_argument("--candidates", type=int, default=8)
    args = parser.parse_args()
    from services.dexjoco import tasks as T
    from services.dexjoco.demonstrations import demos_root
    from services.dexjoco.process import dexjoco_python, simulator_environment

    rev = locked_revision()
    for task in args.task:
        name, regime = T.official_task(task)
        base_camera = T.official_config(task)["camera_mapping"]["base"]
        prefix = "dexjoco_raw_datasets" + ("_rand_full" if regime == "rand_full" else "")
        listing = sorted(item["path"] for item in tree(f"{prefix}/{name}", rev) if item["type"] == "directory")
        task_root = demos_root() / task
        candidates_dir = task_root / "candidates"
        chosen = listing[:args.candidates]
        sources = []
        for path in chosen:
            demo = candidates_dir / Path(path).name / "official"
            if not (demo / "replay.zarr").is_dir():
                for item in tree(path, rev, recursive=True):
                    if item["type"] == "file":
                        download(item["path"], demo / Path(item["path"]).relative_to(path), rev)
            sources.append(demo)
        command = [dexjoco_python(), "-m", "services.dexjoco.demo_tools", "--task", task, "--camera", base_camera]
        for demo in sources:
            command += ["--demo", str(demo), "--out", str(demo.parent / "export")]
        result = subprocess.run(command, cwd=PROJECT, env=simulator_environment(args.gpu), capture_output=True,
                                text=True, timeout=7200)
        reports = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
        if result.returncode != 0 or len(reports) != len(sources):
            raise SystemExit(f"demo_tools failed for {task} (exit {result.returncode}):\n{result.stderr[-4000:]}")
        for report, demo in zip(reports, sources, strict=True):
            report["demo"] = demo.parent.name  # the official episode directory name
        experts = [r["demo"] for r in reports
                   if r.get("replay_success") and r.get("formal_overlap") is False and "error" not in r][:EXPERTS]
        if len(experts) < EXPERTS:
            raise SystemExit(f"Only {len(experts)} verified demos for {task}; rerun with more --candidates:\n"
                             + json.dumps(reports, indent=2))
        experts_dir = task_root / "experts"
        if experts_dir.exists():
            shutil.rmtree(experts_dir)
        for demo_name in experts:
            source = candidates_dir / demo_name
            target = experts_dir / demo_name
            shutil.copytree(source / "official", target)
            for item in ("trajectory.npz", "frames"):
                (shutil.copytree if (source / "export" / item).is_dir() else shutil.copy2)(
                    source / "export" / item, target / item)
            report = next(r for r in reports if r["demo"] == demo_name)
            (target / "episode.json").write_text(json.dumps({
                "task": task, "source": f"{REPO}@{rev}/{prefix}/{name}/{demo_name}",
                "steps": report["steps"], "replay_success_step": report["replay_success_step"],
                "verification": "replayed to success in the official environment from its recorded initial state; "
                                "its layout is none of the official evaluation layouts"}, indent=2) + "\n")
            for path in [target, *target.rglob("*")]:
                path.chmod(0o555 if path.is_dir() else 0o444)
        manifest = {"schema": SCHEMA, "task": task, "regime": regime, "dataset": REPO, "revision": rev,
                    "base_camera": base_camera, "candidates": reports, "experts": experts,
                    "files": {name: file_hashes(experts_dir / name) for name in experts}}
        (task_root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps({"task": task, "experts": experts,
                          "candidates": [{k: r.get(k) for k in ("demo", "replay_success", "formal_overlap",
                                                                 "formal_min_distance", "error")} for r in reports]}))


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
