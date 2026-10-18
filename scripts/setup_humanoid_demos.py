#!/usr/bin/env python3
"""Install three complete SONIC expert episodes per HumanoidArena task from the public raw release.

    python scripts/setup_humanoid_demos.py [--task TASK ...] [--root DIR]
    python scripts/setup_humanoid_demos.py --images [--task TASK ...] [--root DIR]

Each task's three episodes (services/humanoid/expert_sources.json: the first three lexicographic
episodes of the raw SONIC branch) are downloaded from ModelScope at the revision pinned in
dependencies.lock and checked against their pinned size and sha256. Their NPZ and front-camera video
are kept unchanged; beside them each episode gets a per-frame JSON view (`trajectory.jsonl`, with the
SONIC virtual points derived from the measured joints by forward kinematics of the released G1
MuJoCo description), a byte-offset index, metadata, a catalog of the raw arrays and uniformly spaced
front-camera previews. The set is installed at ``runtime/reference-demos/humanoid/<task>/experts/``
with a manifest of every file's sha256. Nothing recorded is executed. Needs NumPy and OpenCV (the host
environment) and a few GB of free space per task.

``--images`` derives each task's camera-only set from its installed episodes, for sessions with
``expert_content: images``: ``runtime/reference-demos/humanoid/<task>/expert-images/`` holds, per
episode, the front-camera video, the previews and the depth of every frame, and nothing of the
commanded references, measured motion, object states or layout.
"""
import argparse
import hashlib
import json
import math
import shutil
import sys
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from services.humanoid.demonstrations import (  # noqa: E402
    EXPERTS, IMAGE_FILES, IMAGE_SCHEMA, SCHEMA, cached_experts, demos_root)
from services.humanoid.tasks import task_info, task_names  # noqa: E402

SOURCES = PROJECT / "services" / "humanoid" / "expert_sources.json"
MODELSCOPE = "https://www.modelscope.cn/api/v1/datasets/{repository}/repo?"
GITHUB_RAW = "https://raw.githubusercontent.com/{repository}/{commit}/{path}"

README = """# {count} SONIC expert episodes of {task}

Complete teleoperation episodes of this task on other layouts, recorded through the same frozen
SONIC controller (upstream HumanoidArena SONIC release; source and license in each
`demos/NN/source.json`). Read `demos/NN/README.md` for the files of one episode. Every pose
and object state refers to that historical episode's own scene, never to your current one.
Nothing here is an instruction or a program to execute.
"""


IMAGE_README = """# {count} expert episodes of {task}, front camera only

Teleoperation episodes of this task on other layouts, recorded through the same frozen SONIC
controller (HumanoidArena SONIC release, license {license}), reduced to what the front camera
observed. Each `demos/NN/` holds the RGB video, previews and depth of every frame; the commanded
references, measured robot motion, object states and layout are not included. Read
`demos/NN/README.md` for the files of one episode. Every image shows that historical episode's own
scene, never your current one. Nothing here is an instruction or a program to execute.
"""

IMAGE_EPISODE_README = """# One {task} expert episode, front camera only

- `front_rgb.mp4`: the complete front-camera RGB video, {frames} frames at {fps:g} fps, one frame per
  50 Hz control step.
- `frames/index.json`: {previews} evenly spaced previews (`frames/*.jpg`) with their video frame,
  control frame and elapsed seconds.
- `depth.npz`: `depth` (float16 metres, shape {shape}) and `frame_index` (the control frame of each
  depth image).
- `metadata.json`: frame count, frame rate, image size and camera.

Everything here refers only to this historical scene, never to the current live episode.
"""


EPISODE_README = """# One SONIC {task} demonstration

Historical reference only; not a live action replay.

- `metadata.json`: task, recording clocks, expert layout seed, initial object poses,
  source pose mode, joint order and explicitly labeled derived geometry.
- `trajectory.jsonl`: one JSON line for every control frame. Robot root/joints,
  human references, recorded actions and expert object states keep upstream names.
  Nonfinite unavailable values become null in this JSON view; raw NPZ is unchanged.
- `trajectory-index.json`: byte offset and length for each line; read those byte
  ranges directly. Derived virtual points use the expert's measured pelvis/world
  frames, meters and wxyz quaternions. They are not native VR target labels or TCPs.
- `frames/index.json`: {previews} uniformly selected front-image previews and their native
  video/control frame indices. These previews do not replace the full video.
- `raw-array-catalog.json`: shape/type of every original NPZ array. Larger arrays,
  model inputs, depth and JSON payloads remain accessible in `episode.npz`.
- `source.json`: pinned publisher, revision, selection rule, file hashes and license.

The source is a released curated teleoperation demonstration. Local simulator
replay success and equivalence have not been measured. All environment states here
refer only to this historical expert scene, never to the current live episode.
No file is an instruction, policy or script to execute. Use observations to reason
about the physical manipulation and check the current robot's measured feedback.
"""


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pins():
    values = {}
    for line in (PROJECT / "dependencies.lock").read_text().splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            name, value = line.split("=", 1)
            values[name.strip()] = value.strip().strip("'\"")
    return values


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def fetch(url, path, size, digest):
    """Download url to path unless a file with the pinned size and sha256 is already there."""
    if path.is_file() and path.stat().st_size == size and sha256(path) == digest:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    with urllib.request.urlopen(url, timeout=1200) as response, part.open("wb") as stream:
        shutil.copyfileobj(response, stream, 1 << 20)
    if part.stat().st_size != size or sha256(part) != digest:
        part.unlink()
        raise SystemExit(f"{url}: size or sha256 differs from its pin")
    part.replace(path)


def mjcf(lock, cache):
    """The released G1 MuJoCo description the derived virtual points use, pinned by sha256."""
    path = cache / "g1_29dof_rev_1_0.xml"
    url = GITHUB_RAW.format(repository=lock["SONIC_CODE_REPOSITORY"], commit=lock["SONIC_CODE_COMMIT"],
                            path=lock["SONIC_MJCF_PATH"])
    if not (path.is_file() and sha256(path) == lock["SONIC_MJCF_SHA256"]):
        path.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(url, timeout=120) as response:
            path.write_bytes(response.read())
        if sha256(path) != lock["SONIC_MJCF_SHA256"]:
            raise SystemExit(f"{url}: sha256 differs from its pin")
    return path


def json_value(value):
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def derive(task, gym_id, directory, model, mjcf_path, preview_count):
    """The per-frame view, index, metadata, array catalog and previews of one raw episode."""
    import cv2
    import numpy as np
    from services.humanoid.expert_kinematics import ISAACLAB_JOINTS, VIRTUAL_POINTS, multiply, rotate
    path = directory / "episode.npz"
    fields = {}
    with zipfile.ZipFile(path) as archive:
        for name in archive.namelist():
            with archive.open(name) as handle:
                header = {(1, 0): np.lib.format.read_array_header_1_0, (2, 0): np.lib.format.read_array_header_2_0}
                shape, _, dtype = header[np.lib.format.read_magic(handle)](handle)
            fields[name[:-4]] = {"shape": list(shape), "dtype": str(dtype),
                                "object_dtype": dtype.hasobject, "itemsize": dtype.itemsize}
    # Numeric and string arrays only; the original complete NPZ is kept and never unpickled.
    arrays, metadata = {}, {}
    with np.load(path, allow_pickle=False) as archive:
        count = int(archive["num_frames"])
        if str(archive["task"]) != gym_id or count <= 0:
            raise SystemExit(f"{task}: {directory.name} is not a {gym_id} recording")
        for name, item in fields.items():
            shape = item["shape"]
            if item["object_dtype"]:
                continue
            if not shape and item["itemsize"] < 64000:
                metadata[name] = json_value(archive[name].tolist())
            elif shape and shape[0] == count and math.prod(shape[1:]) <= 128 and item["itemsize"] <= 16:
                arrays[name] = archive[name]
            elif name.startswith("episode_init_env_obj_"):
                metadata[name] = json_value(archive[name].tolist())
        vision_indices = archive["vision_frame_indices"]
        # Some releases keep the complete front-camera stream inside the NPZ instead of a video.
        video_name = str(archive["vision_rgb_video_path"]) if "vision_rgb_video_path" in archive.files else None
        embedded = archive["vision_rgb"] if video_name is None else None
    metadata["joint_names_canonical29"] = ISAACLAB_JOINTS
    metadata["derived_geometry"] = {
        "source": "Recorded measured robot_qpos_before_decimation in SONIC IsaacLab joint order",
        "mjcf_sha256": sha256(mjcf_path), "units": "meters, radians, wxyz quaternions",
        "frame": "expert measured pelvis; world uses expert recorded root pose",
        "points": {name: {"body": body, "offset_m": offset.tolist()} for name, (body, offset) in VIRTUAL_POINTS.items()},
        "limitation": "Forward-kinematic SONIC virtual points, not observed contact/TCPs or VR action labels"}
    offsets = []
    with (directory / "trajectory.jsonl").open("wb") as stream:
        for index in range(count):
            row = {"frame": index, "elapsed_control_s": index * float(metadata["meta_control_dt"]),
                   **{name: json_value(array[index].tolist()) for name, array in arrays.items()}}
            joints = arrays["robot_qpos_before_decimation"][index]
            if not np.isfinite(joints).all():
                raise SystemExit(f"{task}: nonfinite measured joints in {directory.name}")
            points = model.virtual_points(joints)
            root_position = arrays["robot_root_position"][index]
            root_orientation = arrays["robot_root_orientation"][index]
            row["derived_measured_virtual_points_pelvis"] = {
                name: {"position_m": pos.tolist(), "quaternion_wxyz": quat.tolist()}
                for name, (pos, quat) in points.items()}
            row["derived_measured_virtual_points_world"] = {
                name: {"position_m": (root_position + rotate(root_orientation, pos)).tolist(),
                       "quaternion_wxyz": multiply(root_orientation, quat).tolist()}
                for name, (pos, quat) in points.items()}
            data = (json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n").encode()
            offsets.append({"frame": index, "offset": stream.tell(), "length": len(data)})
            stream.write(data)
    write_json(directory / "trajectory-index.json", offsets)
    write_json(directory / "metadata.json", metadata)
    write_json(directory / "raw-array-catalog.json", fields)
    (directory / "frames").mkdir()
    if video_name is not None:
        video = cv2.VideoCapture(str(directory / video_name))
        total = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(metadata["vision_rgb_video_fps"])
    else:
        video, total = None, len(embedded)
        fps = 1 / float(metadata["meta_control_dt"])
    if total != len(vision_indices) or total <= 0:
        raise SystemExit(f"{task}: {directory.name} video and observation frames do not align")
    selected = sorted(set(np.linspace(0, total - 1, preview_count, dtype=int).tolist()))
    catalog = []
    for frame in selected:
        if video is not None:
            video.set(cv2.CAP_PROP_POS_FRAMES, frame)
            ok, image = video.read()
        else:  # Embedded arrays are RGB; OpenCV writes BGR.
            ok, image = True, cv2.cvtColor(np.ascontiguousarray(embedded[frame]), cv2.COLOR_RGB2BGR)
        name = f"frames/{frame:08d}.jpg"
        if not ok or not cv2.imwrite(str(directory / name), image, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise SystemExit(f"{task}: cannot write preview {name} of {directory.name}")
        catalog.append({"file": name, "video_frame": frame, "control_frame": int(vision_indices[frame]),
                        "elapsed_s": frame / fps})
    if video is not None:
        video.release()
    write_json(directory / "frames" / "index.json", catalog)
    return len(catalog)


def install(task, root):
    lock = pins()
    gym_id = task_info(task).gym_id
    sources = json.loads(SOURCES.read_text())
    episodes = sources["tasks"][task]
    if len(episodes) != EXPERTS:
        raise SystemExit(f"{task}: {SOURCES.name} pins {len(episodes)} episodes, not {EXPERTS}")
    from services.humanoid.expert_kinematics import G1Kinematics
    mjcf_path = mjcf(lock, root / ".cache")
    model = G1Kinematics(mjcf_path)
    provenance = {"repository": lock["HUMANOID_EXPERTS_REPOSITORY"], "revision": lock["HUMANOID_EXPERTS_REVISION"],
                  "source_url": sources["source_url"], "license": sources["license"]}
    target = root / task / "experts"
    staging = target.with_name(".experts.staging")
    downloads = root / ".cache" / "raw"
    if staging.exists():
        for path in [staging, *staging.rglob("*")]:
            path.chmod(0o755 if path.is_dir() else 0o644)
        shutil.rmtree(staging)
    demos = []
    for number, episode in enumerate(episodes):
        directory = staging / "demos" / f"{number:02d}"
        for item in episode["files"]:
            query = urllib.parse.urlencode({"Revision": provenance["revision"], "FilePath": item["remote_path"]})
            cached = downloads / episode["trajectory_id"] / item["local_path"]
            fetch(MODELSCOPE.format(repository=provenance["repository"]) + query, cached, item["bytes"], item["sha256"])
            (directory / item["local_path"]).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(cached, directory / item["local_path"])
        previews = derive(task, gym_id, directory, model, mjcf_path, episode.get("preview_count", 24))
        (directory / "README.md").write_text(EPISODE_README.format(task=task, previews=previews))
        write_json(directory / "source.json", {**provenance, "task": gym_id, "trajectory_id": episode["trajectory_id"],
                                               "selection": sources["selection"], "files": episode["files"]})
        files = {path.relative_to(directory).as_posix(): sha256(path)
                 for path in sorted(directory.rglob("*")) if path.is_file()}
        demos.append({"directory": f"demos/{number:02d}", "trajectory_id": episode["trajectory_id"], "files": files})
        print(f"{task} demos/{number:02d}: {episode['trajectory_id']}, {previews} previews", flush=True)
    (staging / "README.md").write_text(README.format(count=EXPERTS, task=task))
    write_json(staging / "manifest.json", {"schema": SCHEMA, "task": task, "gym_id": gym_id, "demos": demos,
                                           "source": {**provenance, "selection": sources["selection"]}})
    target.parent.mkdir(parents=True, exist_ok=True)
    _replace(staging, target)
    shutil.rmtree(downloads, ignore_errors=True)
    size = sum(p.stat().st_size for p in target.rglob("*") if p.is_file())
    print(f"{task}: {EXPERTS} episodes ({size / 1e9:.2f} GB)")


def _replace(staging, target):
    for path in staging.rglob("*"):
        path.chmod(0o555 if path.is_dir() else 0o444)
    staging.chmod(0o555)
    if target.exists():
        for path in [target, *target.rglob("*")]:
            path.chmod(0o755 if path.is_dir() else 0o644)
        shutil.rmtree(target)
    staging.rename(target)


def image_episode(task, gym_id, demo, destination):
    """One camera-only episode: video, previews and depth of every frame, copied or extracted
    from the complete episode; nothing else of it. Episodes recorded without a video file keep
    their RGB stream in the NPZ (`vision_rgb`); it is encoded to H.264 here."""
    import numpy as np
    videos = sorted((demo / "videos").glob("*.mp4")) if (demo / "videos").is_dir() else []
    if len(videos) > 1:
        raise SystemExit(f"{task}: {demo.name} holds {len(videos)} front-camera videos")
    metadata = json.loads((demo / "metadata.json").read_text())
    source = json.loads((demo / "source.json").read_text())
    previews = json.loads((demo / "frames" / "index.json").read_text())
    with np.load(demo / "episode.npz", allow_pickle=False) as arrays:
        depth = arrays["vision_depth"]
        frame_index = arrays["vision_frame_indices"].astype(np.int64)
        rgb = None if videos else arrays["vision_rgb"]
    if depth.ndim == 4 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    frames = len(depth)
    if (depth.ndim != 3 or frame_index.shape != (frames,)
            or metadata.get("vision_rgb_video_num_frames") not in (None, frames)
            or (rgb is not None and rgb.shape != (*depth.shape, 3))):
        raise SystemExit(f"{task}: {demo.name} depth {depth.shape} does not match its video frames")
    (destination / "frames").mkdir(parents=True)
    fps = float(metadata.get("vision_rgb_video_fps") or 50.0)  # One frame per control step.
    if videos:
        shutil.copyfile(videos[0], destination / "front_rgb.mp4")
    else:
        import imageio.v2 as imageio
        with imageio.get_writer(destination / "front_rgb.mp4", fps=fps, codec="libx264", quality=8,
                                pixelformat="yuv420p", macro_block_size=16) as writer:
            for image in rgb:
                writer.append_data(image)
    for preview in previews:
        name = Path(preview["file"])
        if name.parent != Path("frames") or name.suffix != ".jpg":
            raise SystemExit(f"{task}: unexpected preview path {preview['file']}")
        shutil.copyfile(demo / name, destination / name)
    (destination / "frames" / "index.json").write_text(json.dumps(previews, indent=2) + "\n")
    np.savez_compressed(destination / "depth.npz", depth=depth.astype(np.float16), frame_index=frame_index)
    height, width = depth.shape[1:]
    (destination / "metadata.json").write_text(json.dumps({
        "task": task, "gym_id": gym_id, "camera": "cam_front", "frames": frames, "fps": fps, "control_hz": 50,
        "image": {"height": int(height), "width": int(width)},
        "depth": {"units": "metres", "dtype": "float16", "shape": [frames, int(height), int(width)]},
        "license": source.get("license"),
        "source": {"repository": source.get("repository"), "revision": source.get("revision"),
                   "trajectory_id": source.get("trajectory_id")}}, indent=2) + "\n")
    (destination / "README.md").write_text(IMAGE_EPISODE_README.format(
        task=task, frames=frames, fps=fps, previews=len(previews), shape=[frames, int(height), int(width)]))
    finite = depth[::50][np.isfinite(depth[::50])]
    return source.get("license"), (float(finite.min()), float(finite.max())) if finite.size else None


def install_images(task, root):
    gym_id = task_info(task).gym_id
    experts = root / task / "experts"
    episodes = cached_experts(root, task)
    manifest = json.loads((experts / "manifest.json").read_text())
    target = root / task / "expert-images"
    staging = target.with_name(".expert-images.staging")
    if staging.exists():
        for path in [staging, *staging.rglob("*")]:
            path.chmod(0o755 if path.is_dir() else 0o644)
        shutil.rmtree(staging)
    demos, licenses = [], set()
    for entry, demo in zip(manifest["demos"], episodes):
        destination = staging / entry["directory"]
        license, depth_range = image_episode(task, gym_id, demo, destination)
        licenses.add(license)
        files = {str(path.relative_to(destination)): sha256(path) for path in sorted(destination.rglob("*"))
                 if path.is_file()}
        if not set(IMAGE_FILES) <= set(files):
            raise SystemExit(f"{task}: {entry['directory']} lacks {sorted(set(IMAGE_FILES) - set(files))}")
        demos.append({"directory": entry["directory"], "trajectory_id": entry["trajectory_id"], "files": files})
        print(f"{task} {entry['directory']}: depth {depth_range} m")
    (staging / "README.md").write_text(IMAGE_README.format(count=EXPERTS, task=task, license=", ".join(sorted(map(str, licenses)))))
    (staging / "manifest.json").write_text(json.dumps({
        "schema": IMAGE_SCHEMA, "task": task, "gym_id": gym_id, "demos": demos,
        "source": {"experts_manifest_sha256": sha256(experts / "manifest.json")}}, indent=2) + "\n")
    _replace(staging, target)
    size = sum(p.stat().st_size for p in target.rglob("*") if p.is_file())
    print(f"{task}: {EXPERTS} camera-only episodes ({size / 1e9:.2f} GB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", action="store_true",
                        help="Derive the camera-only sets from the installed episodes instead of installing")
    parser.add_argument("--task", action="append", choices=task_names())
    parser.add_argument("--root", type=Path, default=demos_root(),
                        help="Install root (default runtime/reference-demos/humanoid)")
    args = parser.parse_args()
    for task in args.task or task_names():
        install_images(task, args.root) if args.images else install(task, args.root)


if __name__ == "__main__":
    main()
