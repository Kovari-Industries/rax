"""Attach a before or after pair of Evo EVR1 wrist frames to a RAX TrialContext."""
import hashlib
import json
import os
from pathlib import Path
import struct
import time


def record_wrist_pair(trial, phase, *, left, right, head_observation_id):
    """Read each atomic camera snapshot once. Journal the identity of those exact bytes."""
    if phase not in ("before", "after") or not head_observation_id:
        raise ValueError("Supply a trial phase and the paired head observation ID")
    frames, payloads = {}, {}
    for side, source in (("left", left), ("right", right)):
        raw = Path(source).read_bytes()
        received = time.monotonic_ns()
        if len(raw) < 16:
            raise ValueError(f"Incomplete {side} EVR1 header")
        magic, size, color, depth = struct.unpack_from("<4I", raw)
        if magic != 0x31525645 or not 0 < size < 65536 or len(raw) != 16 + size + color + depth:
            raise ValueError(f"Incomplete {side} EVR1 frame")
        meta = json.loads(raw[16:16 + size])
        if color != depth or color != 2 * meta["width"] * meta["height"]:
            raise ValueError(f"Unexpected {side} plane dimensions")
        name = f"wrist_{phase}-{side}.evr"
        frame = {k: meta[k] for k in ("serial", "sequence", "capture_ns", "registered", "color_us", "depth_us")}
        frames[side] = dict(frame, path=name, sha256=hashlib.sha256(raw).hexdigest(),
                            receipt_monotonic_ns=received, age_ns=received - meta["capture_ns"],
                            head_observation_id=head_observation_id)
        payloads[name] = raw
    created = []
    try:
        for name, raw in payloads.items():
            path = trial.directory / name
            with path.open("xb") as output:
                created.append(path)
                output.write(raw)
                output.flush()
                os.fsync(output.fileno())
    except OSError:
        for path in created:
            path.unlink()
        raise
    trial.annotate("wrist_" + phase, json.dumps(frames))
    return frames
