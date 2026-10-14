"""Robot-only motion preview against the real BEHAVIOR-1K episode (GPU, SSD runtime, ~3-6 min).

Run with: BEHAVIOR_TEST_GPU=0 runtime/envs/robodojo/bin/python -m pytest -m gpu local_tests/unit/test_gpu_behavior_preview.py
"""
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest

from services.behavior import robot as R
from services.behavior import tools as T

pytestmark = pytest.mark.gpu
GPU = os.environ.get("BEHAVIOR_TEST_GPU")


def sequence(value):
    """Trunk tilt, a left-arm move, then a short forward drive, from the measured state."""
    fields, grippers = value["proprio"], value["gripper_commands"]
    hold = R.hold_action(fields, (grippers["left"], grippers["right"]))
    rows = []
    for index in range(60):
        row = hold.copy()
        tilt = min(1.0, (index + 1) / 20)
        row[R.ACTION_SLICES["trunk"]] = np.asarray(fields["trunk_qpos"]) + tilt * np.array([0.15, 0.15, 0.0, 0.0])
        if index >= 20:
            reach = min(1.0, (index - 19) / 20)
            row[R.ACTION_SLICES["arm_left"]] = (1 - reach) * np.asarray(fields["arm_left_qpos"]) + \
                reach * np.array([0.3, 0.6, 0.0, -1.2, 0.0, 0.4, 0.0])
        if 40 <= index < 55:
            row[0] = 0.3  # normalized: about 0.22 m/s forward
        rows.append(row.tolist())
    return rows


def assert_frames(backend, value):
    sequence = value["frame_sequence"]
    manifest = json.loads((backend.frame_workspace / sequence["manifest_path"]).read_text())
    from PIL import Image
    for item in manifest["frames"][-1]["files"]:
        image = np.asarray(Image.open(backend.frame_workspace / item["path"]))
        assert image.shape == (480, 480, 3) and image.std() > 5, item["camera"]


@pytest.mark.skipif(GPU is None, reason="set BEHAVIOR_TEST_GPU to run the real simulator")
def test_preview_matches_the_live_episode(tmp_path):
    from services.behavior.backend import BehaviorBackend
    config = SimpleNamespace(root=tmp_path, task="turning_on_radio", benchmark="behavior", observation_profile="rgbd",
                             exploration_envs=1, subagents=0, eval_seed=0, formal_collection=1, sim_gpu=GPU,
                             formal=SimpleNamespace(wall_seconds=None))  # BEHAVIOR: no wall limit
    backend = BehaviorBackend(config)
    try:
        start, _ = backend.start(formal=False, seed=1)
        rows = sequence(start)
        preview, images = backend.call(T.PREVIEW_RUN, {"actions": rows})
        assert len(images) == len(T.PREVIEW_CAMERAS) and preview["live_step_id"] == 0
        assert backend.sim.step_id == 0
        assert_frames(backend, preview)
        live, _ = backend.call("behavior_step", {"actions": rows})
        end, measured = preview["end_state"], live["proprio"]
        for name in ("trunk_qpos", "arm_left_qpos", "arm_right_qpos"):
            assert np.abs(np.asarray(end[name]) - measured[name]).max() <= 0.01, name
        for name in ("eef_left_pos", "eef_right_pos"):
            assert np.linalg.norm(np.asarray(end[name]) - measured[name]) <= 0.005, name
        motion, odometry = preview["base_motion"], live["odometry"]
        assert np.hypot(motion["x_m"] - odometry["x_m"], motion["y_m"] - odometry["y_m"]) <= 0.01
        assert abs(motion["yaw_rad"] - odometry["yaw_rad"]) <= 0.01
        assert odometry["x_m"] > 0.1

        # A trunk-and-arm cuRobo candidate: watch it, then execute the same plan.
        target = (np.asarray(measured["eef_right_pos"]) + [0.1, 0.0, 0.15]).tolist()
        plan, _ = backend.call("behavior_free_space_move", {
            "arm": "right", "position": target, "quaternion_xyzw": measured["eef_right_quat"],
            "use_trunk": True, "preview_only": True})
        assert plan["transition"]["status"] == "Success", plan["transition"]
        plan_id = plan["transition"]["motion_plan_id"]
        watched, _ = backend.call(T.PREVIEW_RUN, {"motion_plan_id": plan_id})
        assert_frames(backend, watched)
        assert np.linalg.norm(np.asarray(watched["end_state"]["eef_right_pos"]) - target) <= 0.01
        executed, _ = backend.call("behavior_execute_motion_plan", {"motion_plan_id": plan_id})
        assert np.linalg.norm(np.asarray(executed["proprio"]["eef_right_pos"]) - target) < 0.01
        print("preview frames:", backend.frame_workspace / watched["frame_sequence"]["manifest_path"])
    finally:
        backend.close()
    assert not backend.preview.active
