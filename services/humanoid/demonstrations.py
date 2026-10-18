"""Host-side HumanoidArena demonstrations: three complete SONIC expert episodes per task.

scripts/setup_humanoid_demos.py installs each task's three released teleoperation episodes,
recorded through the same frozen SONIC controller on other layouts, under
runtime/reference-demos/humanoid/<task>/experts/ with a sha256 manifest. With
``expert_demonstrations`` the whole set is shown read-only at runtime/experts
(exploration and agent formal copies; never in isolated rehearsal or formal runs).
With ``expert_content: images`` the mount is instead the camera-only set derived from them
(``setup_humanoid_demos.py --images``): each episode's front-camera video, previews and depth,
without commanded references, measured motion, object states or layout.
Independently, ``demonstration_context`` copies only preview images of the same episodes into
runtime/demonstrations, like the other benchmarks.
"""
from __future__ import annotations

import json
import shutil
import uuid
from pathlib import Path

from services.demonstration_io import (
    DEMONSTRATION_CONTEXTS, DemonstrationError, _replace_directory, _validate_jpeg,
)

from .tasks import task_info

PROJECT = Path(__file__).resolve().parents[2]
SCHEMA = "humanoid_experts_v1"
IMAGE_SCHEMA = "humanoid_expert_images_v1"
# The files of one camera-only episode, all derived from the complete episode.
IMAGE_FILES = ("README.md", "metadata.json", "front_rgb.mp4", "depth.npz", "frames/index.json")
EXPERTS = 3


def demos_root():
    return PROJECT / "runtime" / "reference-demos" / "humanoid"


def _load_manifest(directory, task, schema=SCHEMA):
    try:
        manifest = json.loads((directory / "manifest.json").read_text())
    except (OSError, ValueError) as error:
        setup = "scripts/setup_humanoid_demos.py --images" if schema == IMAGE_SCHEMA else "scripts/setup_humanoid_demos.py"
        raise DemonstrationError(f"HumanoidArena expert episodes for {task} are not installed; run {setup}") from error
    if manifest.get("schema") != schema or manifest.get("task") != task:
        raise DemonstrationError(f"Invalid HumanoidArena expert manifest for {task}")
    if len(manifest.get("demos", ())) != EXPERTS:
        raise DemonstrationError(f"HumanoidArena expert set for {task} must hold {EXPERTS} episodes")
    return manifest


def cached_experts(cache_root, task):
    """The installed expert episode directories of one task, in order (light check; the
    setup script verified every file hash)."""
    task_info(task)
    directory = (Path(cache_root).expanduser() / task / "experts").resolve()
    manifest = _load_manifest(directory, task)
    demos = []
    for demo in manifest["demos"]:
        path = (directory / demo["directory"]).resolve()
        if not path.is_relative_to(directory) or not (path / "trajectory.jsonl").is_file():
            raise DemonstrationError(f"Incomplete HumanoidArena expert episode {demo['directory']} for {task}")
        demos.append(path)
    return demos


def cached_expert_images(cache_root, task):
    """The camera-only episode directories of one task, in order (light check)."""
    task_info(task)
    directory = (Path(cache_root).expanduser() / task / "expert-images").resolve()
    manifest = _load_manifest(directory, task, IMAGE_SCHEMA)
    demos = []
    for demo in manifest["demos"]:
        path = (directory / demo["directory"]).resolve()
        if not path.is_relative_to(directory) or not all((path / name).is_file() for name in IMAGE_FILES):
            raise DemonstrationError(f"Incomplete HumanoidArena camera-only episode {demo['directory']} for {task}")
        demos.append(path)
    return demos


def expert_root(task, content="full"):
    """Host directory the agent reads at runtime/experts: the complete episodes, or
    their camera-only set (content 'images')."""
    if content == "images":
        cached_expert_images(demos_root(), task)
        return (demos_root() / task / "expert-images").resolve()
    cached_experts(demos_root(), task)
    return (demos_root() / task / "experts").resolve()


def preview_images(demo):
    """The episode's uniformly spaced front-camera previews, in time order."""
    frames = json.loads((demo / "frames" / "index.json").read_text())
    images = []
    for frame in sorted(frames, key=lambda item: item["control_frame"]):
        path = (demo / frame["file"]).resolve()
        if not path.is_relative_to(demo):
            raise DemonstrationError("HumanoidArena preview image escaped its episode directory")
        _validate_jpeg(path)
        images.append(path)
    if not images:
        raise DemonstrationError(f"No preview images in {demo.name}")
    return images


def provision_trial_demonstration(*, cache_root, target, task, context_kind):
    """Expose the selected preview images in the trial runtime."""
    if context_kind not in DEMONSTRATION_CONTEXTS:
        raise DemonstrationError("demonstration_context must be one of: " + ", ".join(DEMONSTRATION_CONTEXTS))
    demos = [preview_images(demo) for demo in cached_experts(cache_root, task)] if context_kind != "none" else []
    target = Path(target).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    target = target.parent.resolve() / target.name
    # Stage in the trusted trial root, outside the agent-writable runtime.
    staging = target.parent.parent / f".{target.name}.{uuid.uuid4().hex}.tmp"
    staging.mkdir(mode=0o700)
    try:
        if context_kind == "none":
            _replace_directory(staging, target)
            return {"kind": "none", "image_count": 0, "root": "runtime/demonstrations", "manifest_path": None}
        sequences = []
        for number, images in enumerate(demos, 1):
            chosen = images if context_kind == "completion_sequence" else images[-1:]
            (staging / f"demo-{number}").mkdir(mode=0o700)
            names = []
            for order, source in enumerate(chosen):
                relative = (f"demo-{number}/frame_{order:03d}.jpg" if context_kind == "completion_sequence"
                            else f"demo-{number}/terminal.jpg")
                shutil.copyfile(source, staging / relative)
                (staging / relative).chmod(0o600)
                names.append(relative)
            sequences.append(names)
        ordered_images = [name for names in sequences for name in names]
        manifest = {
            "schema_version": 1, "task": task, "context_kind": context_kind, "demonstrator": "robot",
            "source": "humanoidarena_sonic_teleoperation_demos", "camera": "cam_front",
            "image_count": len(ordered_images), "ordered_images": ordered_images,
            # Independent demonstrations, each in a different layout; order applies within one.
            "demonstrations": [{"ordered_images": names} for names in sequences],
            "contains": ["rgb_images"],
        }
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (staging / "manifest.json").chmod(0o600)
        _replace_directory(staging, target)
        return {"kind": context_kind, "image_count": len(ordered_images), "root": "runtime/demonstrations",
                "manifest_path": "runtime/demonstrations/manifest.json"}
    finally:
        if staging.exists():
            shutil.rmtree(staging)
