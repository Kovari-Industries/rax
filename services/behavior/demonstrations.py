"""Host-side BEHAVIOR visual demonstrations: official teleoperation keyframes, images only.

scripts/setup_behavior_demos.sh caches a few demos per task as head-camera keyframes under
runtime/reference-demos/behavior/<task>/. Workspaces receive only the selected images and
a manifest; the cache's episode ids, annotations, actions and states stay with the operator.
"""
from __future__ import annotations

import json
import shutil
import uuid
from pathlib import Path

from services.robodojo.demonstrations import (
    DEMONSTRATION_CONTEXTS, DemonstrationError, _replace_directory, _validate_jpeg,
)

from .tasks import task_info


def cached_demos(cache_root, task):
    """The validated demo list of one task, or DemonstrationError when not installed."""
    task_info(task)
    cache_root = Path(cache_root).expanduser().resolve()
    index_path = cache_root / task / "demos.json"
    try:
        index = json.loads(index_path.read_text())
    except (OSError, ValueError) as error:
        raise DemonstrationError(f"BEHAVIOR demonstrations for {task} are not installed; "
                                 "run scripts/setup_behavior_demos.sh") from error
    if index.get("task") != task or not index.get("demos"):
        raise DemonstrationError(f"Invalid BEHAVIOR demonstration index for {task}")
    demos = []
    for demo in index["demos"]:
        images = []
        for frame in demo["frames"]:
            path = (cache_root / task / frame["image"]).resolve()
            if not path.is_relative_to(cache_root / task):
                raise DemonstrationError("BEHAVIOR demonstration image escaped its task cache")
            _validate_jpeg(path)
            images.append(path)
        if not images:
            raise DemonstrationError(f"Empty BEHAVIOR demonstration for {task}")
        demos.append(images)
    return demos


def provision_trial_demonstration(*, cache_root, target, task, context_kind):
    """Expose the selected demo images in the trial runtime, like the RoboDojo provider."""
    if context_kind not in DEMONSTRATION_CONTEXTS:
        raise DemonstrationError("demonstration_context must be one of: " + ", ".join(DEMONSTRATION_CONTEXTS))
    demos = cached_demos(cache_root, task) if context_kind != "none" else []
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
            "source": "official_behavior_2026_teleoperation_demos", "camera": "cam_head",
            "image_count": len(ordered_images), "ordered_images": ordered_images,
            # Independent demonstrations, each in a different layout; order applies within one.
            "demonstrations": [{"ordered_images": names} for names in sequences],
            "contains": ["rgb_images"],
        }
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (staging / "manifest.json").chmod(0o600)
        _replace_directory(staging, target)
        return {"kind": context_kind, "image_count": len(ordered_images), "demonstration_count": len(sequences),
                "root": "runtime/demonstrations", "manifest_path": "runtime/demonstrations/manifest.json"}
    finally:
        if staging.exists():
            shutil.rmtree(staging)
