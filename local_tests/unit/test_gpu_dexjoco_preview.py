"""Robot-only DexJoCo preview against the real official episode (GPU, ~1-2 min per task).

The backend runs the real episode simulator and the real preview simulator; the same rows run in
both, and the preview's states must follow the episode's. From the reset state the arm starts in
the official home posture in both, so they agree to numerical precision; after earlier motion the
preview rebuilds the elbow posture from home, so they agree to the settle tolerance.

Run with: DEXJOCO_TEST_GPU=0 runtime/envs/host/bin/python -m pytest -m gpu \
    local_tests/unit/test_gpu_dexjoco_preview.py
"""
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest

from services.dexjoco import tasks as TK
from services.dexjoco import tools as T

pytestmark = pytest.mark.gpu
GPU = os.environ.get("DEXJOCO_TEST_GPU")
# (dx, dz, yaw, finger flexion) per motion: a reach with a wrist turn and a partial curl, then back.
MOTIONS = ((0.08, -0.05, 0.3, 0.4), (-0.05, 0.08, -0.5, -0.3))


def rows_from(state, dual, dx, dz, yaw, curl, ramp=40, hold=20):
    """A smooth flange move with a wrist turn and finger flexion, then a hold (mirrored on the left arm)."""
    from scipy.spatial.transform import Rotation

    def arm(pose, hand, sign):
        rotation = Rotation.from_quat(pose[3:7], scalar_first=True)
        result = []
        for k in range(1, ramp + hold + 1):
            a = min(1.0, k / ramp)
            fingers = hand.copy()
            fingers[[1, 2, 5, 6, 9, 10]] += a * curl
            result.append(np.concatenate([pose[:3] + a * np.array([dx, 0.0, dz]),
                                          (Rotation.from_rotvec([0, 0, sign * a * yaw]) * rotation).as_rotvec(),
                                          fingers]))
        return result

    state = np.asarray(state, dtype=np.float64)
    if not dual:
        return [r.tolist() for r in arm(state[:7], state[7:23], 1)]
    return [np.concatenate([r, l]).tolist()
            for r, l in zip(arm(state[:7], state[14:30], 1), arm(state[7:14], state[30:46], -1))]


def difference(a, b, dual):
    from scipy.spatial.transform import Rotation
    a, b, arms = np.asarray(a), np.asarray(b), 2 if dual else 1
    flange = max(np.linalg.norm(a[7 * i:7 * i + 3] - b[7 * i:7 * i + 3]) for i in range(arms))
    rotation = max((Rotation.from_quat(a[7 * i + 3:7 * i + 7], scalar_first=True).inv()
                    * Rotation.from_quat(b[7 * i + 3:7 * i + 7], scalar_first=True)).magnitude() for i in range(arms))
    return float(flange), float(rotation), float(np.abs(a[7 * arms:] - b[7 * arms:]).max())


def manifest(backend, value):
    return json.loads((backend.frame_workspace / value["frame_sequence"]["manifest_path"]).read_text())


@pytest.mark.skipif(GPU is None, reason="set DEXJOCO_TEST_GPU to run the real simulators")
@pytest.mark.parametrize("task", ["pick_bucket", "bimanual_hanoi"])
def test_preview_follows_the_live_episode(tmp_path, task):
    from PIL import Image
    from services.dexjoco.backend import DexjocoBackend
    config = SimpleNamespace(root=tmp_path, task=task, benchmark="dexjoco", observation_profile="rgbd",
                             exploration_envs=1, subagents=0, eval_seed=0, formal_collection=1, sim_gpu=GPU,
                             formal=SimpleNamespace(wall_seconds=3600))
    dual = TK.task_info(task)["dual_arm"]
    backend = DexjocoBackend(config)
    report = {}
    try:
        start, _ = backend.start(formal=False, seed=0)
        description, images = backend.call(T.PREVIEW_DESCRIBE, {})
        assert images == [] and not backend.preview.active  # Describing starts nothing.
        assert description["counts_toward_episode_steps"] is False
        state = start["state"]
        for phase, motion in enumerate(MOTIONS):
            rows = rows_from(state, dual, *motion)
            before = backend.sim.step_id
            preview, images = backend.call(T.PREVIEW_RUN, {"actions": rows})
            assert backend.sim.step_id == before and preview["live_step_id"] == before and "step_id" not in preview
            assert len(images) == len(T.PREVIEW_CAMERAS) and preview["start_source"] == "episode_measured_state"
            assert preview["requested_start_state"] == state
            assert preview["start_settle"]["converged"], preview["start_settle"]
            frames = manifest(backend, preview)["frames"]
            assert [f["preview_row"] for f in frames] == [0, *range(10, len(rows) + 1, 10)]
            for item in frames[-1]["files"]:
                image = np.asarray(Image.open(backend.frame_workspace / item["path"]))
                assert image.shape == (480, 480, 3) and image.std() > 5, item["camera"]
            live, _ = backend.call(T.STEP, {"actions": rows})
            assert live["step_id"] == before + len(rows) and not live["transition"]["episode_ended"]
            end = difference(preview["end_state"], live["state"], dual)
            # Every saved frame of both sequences: the preview's row k against the episode's step before + k.
            live_states = {f["step_id"] - before: f["state"] for f in manifest(backend, live)["frames"]}
            path = [difference(f["state"], live_states[f["preview_row"]], dual) for f in frames[1:]]
            worst = tuple(max(p[i] for p in path) for i in range(3))
            report[phase] = {"end": end, "path_max": worst, "settle_steps": preview["start_settle"]["control_steps"],
                             "preview_seconds": preview["preview_seconds"]}
            # From reset the arm starts in the home posture in both simulators: the same dynamics.
            # After motion the preview rebuilds the elbow posture: within the settle tolerance.
            limit = (1e-5, 1e-4, 1e-4) if phase == 0 else (1e-3, 5e-3, 1e-3)
            assert all(value <= bound for value, bound in zip(worst, limit)), report
            state = live["state"]

        # An explicit start state: the flange 5 cm higher than measured, no episode state used.
        explicit = list(state)
        explicit[2] += 0.05
        value, _ = backend.call(T.PREVIEW_RUN, {"actions": rows_from(explicit, dual, 0, 0, 0, 0, 1, 1),
                                                "initial_state": explicit})
        assert value["start_source"] == "initial_state" and value["start_settle"]["converged"]
        assert difference(value["start_state"], explicit, dual)[0] <= 1e-3

        # Through the gateway: the frames are published, the episode's recorded steps do not move.
        from services.controller.gateway import Gateway
        from services.controller.recording import RunRecording
        recording = RunRecording(tmp_path / "published", "operate-" + "b" * 32)
        gateway = Gateway(backend, None, contract=backend.contract, audit=recording.event, charge=lambda *_: None,
                          refund=lambda *_: None, budget=lambda: {},
                          publish_sequence=lambda value: recording.sequence(value, backend.frame_workspace))
        reply = gateway.handle(gateway.acquire(), {"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {
            "name": T.PREVIEW_RUN, "arguments": {"actions": rows[:12]}}})
        assert len(reply["result"]["content"]) == 1 + len(T.PREVIEW_CAMERAS)
        published = json.loads(reply["result"]["content"][0]["text"])
        assert published["frame_sequence"]["frame_count"] == 3 and not recording.errors
        assert recording.end_step is None and published["live_step_id"] == backend.sim.step_id
        print(task, json.dumps(report))
    finally:
        backend.close()
    assert not backend.preview.active
    # One persistent preview process served every run, and it logged no error.
    logs = sorted((tmp_path / "native" / "preview").glob("preview_*.log"))
    assert len(logs) == 1 and "Traceback" not in logs[0].read_text(), logs
    print(logs[0].read_text()[-2000:])
