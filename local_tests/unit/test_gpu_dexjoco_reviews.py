"""The real DexJoCo simulator records the ground truth episode reviews read (GPU, a few seconds per task).

For each official task, exploration episodes on the real official environment, closed early:
- after a short motion and a hold, the goal report holds every live measurement at reset and at
  the end, its state tail agrees with MuJoCo at reset (position and orientation), fixed objects do
  not move, and the report reads;
- with the robot holding still, untouched objects settle no more than the report says.

Run with: DEXJOCO_TEST_GPU=0 runtime/envs/host/bin/python -m pytest -m gpu local_tests/unit/test_gpu_dexjoco_reviews.py
"""
import json
import math
import os
import time
import uuid

import numpy as np
import pytest

from services.dexjoco import analysis as A
from services.dexjoco import objects as O

pytestmark = pytest.mark.gpu
GPU = os.environ.get("DEXJOCO_TEST_GPU")
FIXED = {"glasses case", "plant", "stand", "tower base", "microwave"}  # Fixed in the scene (body poses).


def rotvec(q):
    w, x, y, z = q
    n = math.sqrt(x * x + y * y + z * z)
    angle = 2 * math.atan2(n, w)
    return [x / n * angle, y / n * angle, z / n * angle] if n > 1e-12 else [0.0, 0.0, 0.0]


def lowered(state, drop):
    """The official row that holds the hands and moves each flange ``drop`` metres down."""
    if len(state) == 46:
        return [state[0], state[1], state[2] - drop, *rotvec(state[3:7]), *state[14:30],
                state[7], state[8], state[9] - drop, *rotvec(state[10:14]), *state[30:46]]
    return [state[0], state[1], state[2] - drop, *rotvec(state[3:7]), *state[7:23]]


def episode(tmp_path, task, act):
    """One closed-early exploration episode on layout 0; its sim directory and raw goal report."""
    from services.dexjoco.process import DexjocoProcess
    from services.dexjoco.tasks import STARTUP_SECONDS
    sim = DexjocoProcess(output_root=tmp_path / "results")
    run_id = f"{task}_reviews_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{uuid.uuid4().hex[:6]}"
    try:
        sim.start(task=task, collection=0, layout_id=0, run_id=run_id, sim_gpu=GPU, startup_timeout=STARTUP_SECONDS,
                  include_depth=False, episode_timeout_seconds=1800)
        act(sim)
        sim.finish("close")
    finally:
        sim.stop()
    raw = json.loads((sim.output_dir / "sim" / "private_goal_report.json").read_text())
    assert "error" not in raw, raw.get("error")
    return sim.output_dir / "sim", raw


@pytest.mark.skipif(GPU is None, reason="set DEXJOCO_TEST_GPU to run the real simulator")
@pytest.mark.parametrize("task", sorted(O.TAIL))
def test_the_simulator_records_what_reviews_read(tmp_path, task):
    def act(sim):
        state = sim.request("observe")["state"]
        sim.request("chunk_step", actions=[lowered(state, 0.002 * i) for i in range(1, 31)])
        sim.request("hold", steps=5)
    directory, raw = episode(tmp_path, task, act)
    sources = O.live_sources(task)
    for when in ("initial_live", "live"):
        assert set(raw[when]) == set(sources)
        failed = {label: value for label, value in raw[when].items() if not isinstance(value, list)}
        assert not failed, failed
    assert raw["step_id"] == 35 and raw["terminated"] is False and raw["reason"] == "close"
    report = A.ground_truth_report(directory)
    assert report["warnings"] == []  # Tail and MuJoCo agree at reset; every condition measured and read.
    for line in report["movement"]:
        label, moved = line[2:].split(": ", 1)
        if label in FIXED:
            assert moved == "did not move", line
    text = A.report_text(report)
    assert text.startswith("Outcome: no success after 35 of ") and "(the agent closed it with `finish`" in text
    print(f"\n--- {task}\n{text}")


@pytest.mark.skipif(GPU is None, reason="set DEXJOCO_TEST_GPU to run the real simulator")
@pytest.mark.parametrize("task", sorted(O.TAIL))
def test_untouched_objects_settle_within_what_the_report_says(tmp_path, task):
    directory, raw = episode(tmp_path, task, lambda sim: sim.request("hold", steps=60))
    episode_ = A.Episode(raw)
    for label, size, source in O.TAIL[task]:
        if source is None:
            continue
        start, end = np.asarray(episode_.start[label]), np.asarray(episode_.live[label])
        shift = end[:3] - start[:3]
        assert np.hypot(shift[0], shift[1]) <= A.SETTLING_M and abs(shift[2]) <= A.SETTLING_M, (label, shift)
        if size == 7:
            assert max(abs(v) for v in A.rotation_change(start[3:7], end[3:7])) <= A.SETTLING_DEG, label
