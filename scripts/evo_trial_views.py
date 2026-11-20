"""Render head RGB and both wrist RGB-D captures from one Evo trial phase."""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import cv2
import numpy as np


def artifact(directory, name):
    path = (directory / name).resolve()
    if not path.is_relative_to(directory.resolve()):
        raise ValueError(f"Artifact outside trial: {name}")
    return path


def wrist(directory, entry):
    raw = artifact(directory, entry["path"]).read_bytes()
    if hashlib.sha256(raw).hexdigest() != entry["sha256"]:
        raise ValueError("Wrist frame hash mismatch")
    magic, meta_size, rgb_size, depth_size = struct.unpack_from("<4I", raw)
    offset = 16 + meta_size
    if magic != 0x31525645 or len(raw) != offset + rgb_size + depth_size:
        raise ValueError("Incomplete EVR1 wrist frame")
    meta = json.loads(raw[16:offset])
    height, width = meta["height"], meta["width"]
    if rgb_size != 2 * height * width or depth_size != rgb_size:
        raise ValueError("Unexpected EVR1 plane dimensions")
    if meta["sequence"] != entry["sequence"] or meta["capture_ns"] != entry["capture_ns"]:
        raise ValueError("Wrist frame identity mismatch")
    rgb = np.frombuffer(raw, np.uint8, rgb_size, offset).reshape(height, width, 2)
    depth = np.frombuffer(raw, "<u2", height * width, offset + rgb_size).reshape(height, width)
    return cv2.cvtColor(rgb, cv2.COLOR_YUV2BGR_YUY2), depth * meta["depth_scale_m"]


def panel(image, title, width):
    height = width * 5 // 8
    canvas = np.zeros((height + 29, width, 3), np.uint8)
    scale = min(width / image.shape[1], height / image.shape[0])
    scaled = cv2.resize(image, (max(1, round(image.shape[1] * scale)),
                                max(1, round(image.shape[0] * scale))), interpolation=cv2.INTER_AREA)
    y = (height - scaled.shape[0]) // 2
    x = (width - scaled.shape[1]) // 2
    canvas[y:y + scaled.shape[0], x:x + scaled.shape[1]] = scaled
    cv2.putText(canvas, title, (6, height + 19), cv2.FONT_HERSHEY_SIMPLEX,
                .48, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def render(directory, phase, width):
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    annotation, = [e for e in events if e.get("outcome") == "wrist_" + phase]
    refs = json.loads(annotation["note"])
    head_ids = {refs[side]["head_observation_id"] for side in ("left", "right")}
    if len(head_ids) != 1:
        raise ValueError("Wrist frames reference different head observations")
    head_id = head_ids.pop()
    journals = [(directory, events)]
    feedback = directory / "feedback"
    if (feedback / "events.jsonl").exists():
        journals.append((feedback, [json.loads(s) for s in (feedback / "events.jsonl").read_text().splitlines()]))
    heads = [(folder, e["value"]["cameras"]["base"]) for folder, stream in journals for e in stream
             if e.get("kind") == "result" and isinstance(e.get("value"), dict)
             and e["value"].get("observation_id") == head_id and "cameras" in e["value"]]
    if not heads and (directory / "recovered-feedback.json").is_file():
        recovery = json.loads((directory / "recovered-feedback.json").read_text())
        if recovery["observation_id"] == head_id:
            heads.append((directory, recovery["camera"]))
    if not heads:
        raise ValueError(f"Missing referenced head observation {head_id}")
    folder, camera = heads[0]
    head_data = artifact(folder, camera["path"]).read_bytes()
    digest = camera.get("trial_sha256") or camera.get("sha256")
    if digest and hashlib.sha256(head_data).hexdigest() != digest:
        raise ValueError("Head frame hash mismatch")
    head = cv2.imdecode(np.frombuffer(head_data, np.uint8), cv2.IMREAD_COLOR)
    if head is None:
        raise ValueError("Cannot decode head RGB")
    colors = [panel(head, f"Head   {head_id}", width)]
    legend = np.zeros((width * 5 // 8, width, 3), np.uint8)
    ramp = np.tile(np.linspace(0, 255, width - 32, dtype=np.uint8), (18, 1))
    legend[32:50, 16:-16] = cv2.applyColorMap(ramp, cv2.COLORMAP_VIRIDIS)
    cv2.putText(legend, "0.10 m", (16, 73), 0, .55, (255, 255, 255), 1)
    cv2.putText(legend, "0.80 m", (width - 82, 73), 0, .55, (255, 255, 255), 1)
    gap_ms = abs(refs["left"]["capture_ns"] - refs["right"]["capture_ns"]) / 1e6
    depths = [panel(legend, f"Wrist capture separation   {gap_ms:.0f} ms", width)]
    for side in ("left", "right"):
        ref = refs[side]
        color, depth = wrist(directory, ref)
        colors.append(panel(color, f"{side.title()} RGB   frame {ref['sequence']}", width))
        values = np.clip((depth - .10) * (255 / .70), 0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(values, cv2.COLORMAP_VIRIDIS)
        colored[depth == 0] = 0
        label = "Registered depth" if ref["registered"] else "Unregistered depth"
        depths.append(panel(colored, f"{side.title()}   {label} (black = missing)", width))
    return np.vstack((np.hstack(colors), np.hstack(depths)))


def attach_joint_panel(board, directory, phase, sensors):
    from evo_trial_joints import at_capture

    events = (directory / "events.jsonl").read_text().splitlines()
    annotations = (json.loads(line) for line in events)
    ref = next(e for e in annotations if e.get("outcome") == "wrist_" + phase)
    frames = json.loads(ref["note"])
    summary = {side: at_capture(sensors, frames[side]["capture_ns"]) for side in ("left", "right")}
    strip = np.full((184, board.shape[1], 3), 245, np.uint8)
    for index, (side, sample) in enumerate(summary.items()):
        x = index * (board.shape[1] // 2) + 8
        metrics = sample["arms"][side]
        lines = [f"{side.title()} wrist   frame minus joint sample {sample['capture_minus_joint_ms']:+.1f} ms",
                 "                 J1      J2      J3      J4      J5      J6      J7"]
        for name, key in [("q (rad)", "q_rad"), ("dq (rad/s)", "dq_rad_s"), ("torque (Nm)", "torque_nm")]:
            values = ["--" if v is None else f"{v:.2f}" for v in metrics[key]]
            lines.append(f"{name:13}" + " ".join(f"{v:>6}" for v in values))
        for row, line in enumerate(lines):
            font_scale = min(.53, (board.shape[1] // 2 - 16) / max(cv2.getTextSize(line, 0, 1, 1)[0][0], 1))
            cv2.putText(strip, line, (x, 25 + 30 * row), 0, font_scale, (35, 35, 35), 1, cv2.LINE_AA)
    return np.vstack((board, strip)), summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--phase", choices=["before", "after"], default="after")
    parser.add_argument("--width", type=int, default=400, help="Pixel width per view (200–1280)")
    parser.add_argument("--sensors", type=Path, help="Pickup sensors.jsonl containing /joint_states")
    parser.add_argument("--joint-output", type=Path, help="Save the two matched joint samples as JSON")
    args = parser.parse_args()
    if args.joint_output and not args.sensors:
        parser.error("--joint-output requires --sensors")
    if not 200 <= args.width <= 1280:
        parser.error("Choose a panel width between 200 and 1280 pixels")
    board = render(args.trial, args.phase, args.width)
    if args.sensors:
        board, joints = attach_joint_panel(board, args.trial, args.phase, args.sensors)
        if args.joint_output:
            args.joint_output.write_text(json.dumps(joints, indent=2, allow_nan=False) + "\n")
    if not cv2.imwrite(str(args.output), board):
        raise OSError(f"Cannot write {args.output}")


if __name__ == "__main__":
    main()
