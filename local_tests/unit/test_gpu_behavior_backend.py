"""Real BEHAVIOR-1K simulator through the host BehaviorBackend (GPU, SSD runtime, ~2-6 min).

Run with: BEHAVIOR_TEST_GPU=0 runtime/envs/robodojo/bin/python -m pytest -m gpu local_tests/unit/test_gpu_behavior_backend.py
"""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services.behavior import robot as R

pytestmark = pytest.mark.gpu
GPU = os.environ.get("BEHAVIOR_TEST_GPU")
PRIVATE = ("q_score", "native_score", "native_success", "goal_predicates", "world_base", "robot_pos",
           "joint_qpos", "base_qpos", "instance_id")


def rows_from(value, count):
    fields = value["proprio"]
    grippers = value["gripper_commands"]
    return [R.hold_action(fields, (grippers["left"], grippers["right"])).tolist()] * count


@pytest.mark.skipif(GPU is None, reason="set BEHAVIOR_TEST_GPU to run the real simulator")
@pytest.mark.parametrize("profile", ["rgbd"])  # BEHAVIOR sessions are RGB-D only
def test_real_simulator_through_backend(tmp_path, profile):
    from services.behavior.backend import BehaviorBackend
    config = SimpleNamespace(root=tmp_path, task="turning_on_radio", benchmark="behavior", observation_profile=profile,
                             exploration_envs=1, subagents=0, eval_seed=0, formal_collection=1, sim_gpu=GPU,
                             formal=SimpleNamespace(wall_seconds=None))  # BEHAVIOR: no wall limit
    backend = BehaviorBackend(config)
    try:
        value, images = backend.start(formal=False, seed=0)
        assert value["environment_spec"]["episode_step_limit"] > 1000
        # No wall limit: the simulator arms no episode timer.
        command = Path(f"/proc/{backend.sim.process.pid}/cmdline").read_bytes().split(b"\0")
        assert command[command.index(b"--episode-timeout-seconds") + 1] == b"0"
        assert len(images) == (6 if profile == "rgbd" else 3)
        status, _ = backend.call("behavior_status", {})
        assert len(status["action_limits"]["lower"]) == R.ACTION_DIM
        value, images = backend.call("behavior_step", {"actions": rows_from(value, 13)})
        assert value["step_id"] == 13 and value["frame_sequence"]["frame_count"] == 3
        manifest = backend.frame_workspace / value["frame_sequence"]["manifest_path"]
        assert [f["step_id"] for f in json.loads(manifest.read_text())["frames"]] == [6, 12, 13]
        moved, _ = backend.call("behavior_base_move", {"dx": 0.3, "dy": 0.0, "dyaw": 0.3})
        assert moved["transition"]["reached"] is True
        assert abs(moved["odometry"]["x_m"]) > 0.2
        if profile == "rgbd":
            point, _ = backend.call("behavior_pixel_to_position", {
                "observation_step_id": moved["step_id"], "selections": [{"camera": "cam_head", "u": 360, "v": 700}]})
            assert point["selections"][0]["valid"] and abs(point["selections"][0]["position_m"][2]) < 0.05  # floor
        # Bend the left arm away from its joint limits, then arms-only closed-loop IK.
        rows = rows_from(moved, 60)
        for row in rows:
            row[7:14] = [0.3, 0.6, 0.0, -1.2, 0.0, 0.4, 0.0]
        bent, _ = backend.call("behavior_step", {"actions": rows})
        goal = (np.asarray(bent["proprio"]["eef_left_pos"]) + [0.06, 0.04, -0.05]).tolist()
        reached, _ = backend.call("behavior_step_eef", {
            "arm": "left", "position": goal, "quaternion_xyzw": bent["proprio"]["eef_left_quat"]})
        transition = reached["transition"]
        assert transition["reached"] is True, {k: transition.get(k) for k in ("final_position_error_m", "final_orientation_error_rad")}
        assert np.linalg.norm(np.asarray(reached["proprio"]["eef_left_pos"]) - goal) < 0.01
        assert np.abs(np.asarray(reached["proprio"]["trunk_qpos"]) - bent["proprio"]["trunk_qpos"]).max() < 0.01
        assert np.linalg.norm(np.asarray(reached["proprio"]["eef_right_pos"]) - bent["proprio"]["eef_right_pos"]) < 0.01
        # Arms-only cuRobo plan (trunk locked): the finetune L-BFGS workaround for sm_120.
        target = (np.asarray(reached["proprio"]["eef_right_pos"]) + [0.08, 0.0, 0.08]).tolist()
        arms_only, _ = backend.call("behavior_free_space_move", {
            "arm": "right", "position": target, "quaternion_xyzw": reached["proprio"]["eef_right_quat"]})
        assert arms_only["transition"]["status"] == "Success", arms_only["transition"]
        assert np.linalg.norm(np.asarray(arms_only["proprio"]["eef_right_pos"]) - target) < 0.01
        assert np.abs(np.asarray(arms_only["proprio"]["trunk_qpos"]) - reached["proprio"]["trunk_qpos"]).max() < 0.01
        # Trunk-and-arm plan, previewed then executed.
        target = (np.asarray(arms_only["proprio"]["eef_left_pos"]) + [0.15, 0.0, 0.2]).tolist()
        plan, _ = backend.call("behavior_free_space_move", {
            "arm": "left", "position": target, "quaternion_xyzw": arms_only["proprio"]["eef_left_quat"],
            "use_trunk": True, "preview_only": True})
        assert plan["transition"]["status"] == "Success" and plan["transition"]["executed"] is False
        executed, _ = backend.call("behavior_execute_motion_plan", {"motion_plan_id": plan["transition"]["motion_plan_id"]})
        assert np.linalg.norm(np.asarray(executed["proprio"]["eef_left_pos"]) - target) < 0.01
        evaluation = backend.evaluate()
        assert evaluation["task_complete"] is False
        for reply in (value, moved, reached, plan, executed, arms_only, evaluation):
            text = json.dumps(reply)
            assert not any(word in text for word in PRIVATE)
        state = json.loads((backend.sim.output_dir / "sim" / "operator_state.json").read_text())
        assert state["reward"]["q_score"] == 0.0 and state["step_id"] == executed["step_id"]
        assert state["episode_mode"] == "exploration" and state["instance_mode"] == "train"
    finally:
        backend.close()
    outcome = json.loads((backend.sim.output_dir / "sim" / "evaluation_outcome.json").read_text())
    assert outcome["native_control_steps"] > 50 and outcome["score_source"] == "behavior_q_score"
    assert (backend.sim.output_dir / "sim" / "sensors.mp4").stat().st_size > 0


@pytest.mark.skipif(GPU is None, reason="set BEHAVIOR_TEST_GPU to run the real simulator")
def test_two_exploration_environments_run_concurrently(tmp_path):
    """The parallel-exploration pool routes BEHAVIOR tools to two live simulators."""
    from concurrent.futures import ThreadPoolExecutor
    from services.behavior.tasks import task_timeouts
    from services.exploration.pool import ExplorationPool
    config = {"task": "turning_on_radio", "benchmark": "behavior", "sim_gpu": GPU, "eval_seed": 0,
              "episode_seconds": task_timeouts("turning_on_radio")["episode_seconds"],
              "published_root": str(tmp_path / "published")}
    pool = ExplorationPool(tmp_path / "pool", count=2, seeds=[0, 1], worker_config=config)

    def value(reply):
        assert not reply.get("isError"), reply
        return json.loads(reply["content"][0]["text"])
    try:
        # Workers exit with the thread that started them (parent-death signal), as in the
        # dispatcher's long-lived slot threads: keep every call inside the executor.
        with ThreadPoolExecutor(max_workers=2) as executor:
            started = [value(r) for r in executor.map(pool.start, [0, 1])]
            assert started[0]["episode_id"] != started[1]["episode_id"]
            replies = list(executor.map(lambda i: value(pool.call(i, "behavior_step", {"actions": rows_from(started[i], 4)})), [0, 1]))
            assert [r["step_id"] for r in replies] == [4, 4]
            pool.call(0, "behavior_step", {"actions": rows_from(started[0], 2)})
            assert value(pool.call(1, "behavior_status", {}))["step_id"] == 4  # environments are independent
            list(executor.map(pool.finish, [0, 1]))
        assert pool.status()["exploration_complete"]
    finally:
        pool.close()
