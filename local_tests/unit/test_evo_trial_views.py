import hashlib
import json
from pathlib import Path
import struct
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from evo_trial_joints import at_capture
from evo_trial_views import render


def test_frame_identity_and_corruption(tmp_path):
    head = cv2.imencode(".jpg", np.zeros((8, 12, 3), np.uint8))[1].tobytes()
    (tmp_path / "head.jpg").write_bytes(head)
    camera = {"path": "head.jpg", "trial_sha256": hashlib.sha256(head).hexdigest()}
    result = {"kind": "result", "value": {"observation_id": "o1", "cameras": {"base": camera}}}
    refs = {}
    for i, side in enumerate(("left", "right")):
        meta = {"sequence": i, "capture_ns": 1000 + i, "height": 4, "width": 6, "depth_scale_m": .001, "registered": True}
        header = json.dumps(meta).encode()
        raw = struct.pack("<4I", 0x31525645, len(header), 48, 48) + header + bytes(96)
        name = side + ".evr"
        (tmp_path / name).write_bytes(raw)
        refs[side] = dict(meta, path=name, head_observation_id="o1", sha256=hashlib.sha256(raw).hexdigest())
    note = {"kind": "annotation", "outcome": "wrist_after", "note": json.dumps(refs)}
    (tmp_path / "events.jsonl").write_text("\n".join(map(json.dumps, [result, note])))
    board = render(tmp_path, "after", 320)
    assert board.shape[1] == 960
    (tmp_path / "right.evr").write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash mismatch"):
        render(tmp_path, "after", 320)


def test_proprioception_uses_acquisition_time(tmp_path):
    path = tmp_path / "sensors.jsonl"
    names = [f"{s}_joint{i}" for s in ("left", "right") for i in range(1, 8)]
    base = 9_000_000_000
    row = dict(source="/joint_states", names=names, position=[0.] * 14, velocity=[], effort=list(range(14)))
    with path.open("w") as stream:
        for i in range(1800):
            t = base + i * 1_000_000
            measured = dict(row, receipt_ns=t + 5_000_000, wall_ns=t + 105_000_000, acquired_ns=t + 100_000_000)
            print(json.dumps(measured), file=stream)
        stream.write('{"partial"')
    sample = at_capture(path, base + 876_100_000)
    assert sample["capture_minus_joint_ms"] == pytest.approx(.1)
    assert sample["arms"]["right"]["torque_nm"] == list(range(7, 14))
    assert sample["arms"]["left"]["dq_rad_s"] == [None] * 7
    with pytest.raises(ValueError, match="No measured"):
        at_capture(path, base - 500_000_000)


def test_invalid_second_wrist_leaves_trial_untouched(tmp_path):
    from auto_research_agent.api.evo_wrist_capture import record_wrist_pair
    from types import SimpleNamespace

    directory = tmp_path / "trial"
    directory.mkdir()
    trial = SimpleNamespace(directory=directory, annotate=lambda *args: pytest.fail("unexpected annotation"))
    left, right = tmp_path / "left", tmp_path / "right"
    meta = dict(width=2, height=2, serial="left", sequence=1, capture_ns=1, registered=True, color_us=1, depth_us=2)
    header = json.dumps(meta).encode()
    left.write_bytes(struct.pack("<4I", 0x31525645, len(header), 8, 8) + header + bytes(16))
    right.write_bytes(b"incomplete")
    with pytest.raises(ValueError, match="right EVR1 header"):
        record_wrist_pair(trial, "after", left=left, right=right, head_observation_id="o1")
    assert not list(directory.iterdir())
