"""HumanoidArena host modules: catalog, official seeds, tool validation, geometry and TASK.md.

Imports every host-side module with Isaac Sim, Isaac Lab and the upstream task code blocked;
nothing here starts a simulator.
"""
import builtins
import hashlib
import importlib
import sys

import numpy as np
import pytest

BLOCKED = ("isaacsim", "isaaclab", "isaaclab_tasks", "omni", "carb", "pxr", "tasks", "action_provider",
           "common_env_objects", "curobo", "onnxruntime")


@pytest.fixture
def host_only(monkeypatch):
    real_import = builtins.__import__

    def guarded(name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0 and name.split(".")[0] in BLOCKED:  # Relative imports are package-internal.
            raise ImportError(f"host must not import {name}")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded)
    for module in [m for m in sys.modules if m.startswith("services.humanoid")]:
        monkeypatch.delitem(sys.modules, module)


def test_host_modules_import_without_simulator(host_only):
    for name in ("benchmark", "backend", "demonstrations", "process", "robot", "task_rules", "tasks", "tools"):
        importlib.import_module(f"services.humanoid.{name}")


def test_registry_entry_matches_adapter():
    from services.benchmarks import BENCHMARKS, for_task
    from services.humanoid import tasks, tools
    bench = BENCHMARKS["humanoid"]
    assert for_task("doubledesk") is bench
    assert bench.task_names() == tasks.task_names()
    assert {t["name"] for t in bench.definitions()} == tools.ROBOT_TOOLS == bench.robot_tools
    assert bench.mutating == tools.MUTATING and bench.geometry_tools == tools.GEOMETRY_TOOLS


def test_official_seeds_follow_upstream_formula():
    from services.humanoid import tasks
    # eval_vla_suite._derive_episode_seed: sha256("<gym id>|<group>|<repeat>"), last 4 bytes, int31.
    gym = tasks.task_info("doubledesk").gym_id
    expected = int.from_bytes(hashlib.sha256(f"{gym}|1|7".encode()).digest()[-4:], "big") & 0x7FFFFFFF
    seeds = tasks.official_seeds("doubledesk")
    assert len(seeds) == len(set(seeds)) == tasks.FORMAL_EPISODES == 60
    assert seeds[27] == expected  # group-major: 20 repeats per group seed
    assert tasks.layout_seed("doubledesk", tasks.FORMAL_COLLECTION, 27) == expected
    with pytest.raises(ValueError):
        tasks.layout_seed("doubledesk", tasks.FORMAL_COLLECTION, 60)


@pytest.mark.parametrize("task", ["opendoor", "boxing_bag", "pickplace_box", "doubledesk", "sit_sofa",
                                  "football_single", "warehouse_navigation"])
def test_exploration_never_uses_official_seeds(task):
    from services.humanoid import tasks
    explore = [tasks.layout_seed(task, tasks.EXPLORATION_COLLECTION, i) for i in range(1000)]
    assert len(set(explore)) == 1000
    assert not set(explore) & set(tasks.official_seeds(task))
    assert explore[:5] == [tasks.exploration_seed(task, i) for i in range(5)]  # deterministic


def test_step_validation():
    from services.humanoid import tools
    good = [[0.0] * 38 + [0, 1]]
    tools.validate_request("humanoid_step", {"actions": good})
    bad = [{"actions": []}, {"actions": good * 101}, {"actions": [[0.0] * 38 + [0.5, 0]]},
           {"actions": [[0.0] * 39]}, {"actions": [[float("nan")] * 40]}, {"actions": good, "mode": "vr3"}]
    for arguments in bad:
        with pytest.raises(tools.RejectedBeforeAction):
            tools.validate_request("humanoid_step", arguments)


def test_free_space_move_validation():
    from services.humanoid import tools
    tools.validate_request("humanoid_free_space_move", {"arm": "left", "position": [0.3, 0.2, 0.1]})
    tools.validate_request("humanoid_free_space_move", {"arm": "right", "position": [0.3, -0.2, 0.1],
                                                        "quaternion_wxyz": [1, 0, 0, 0], "frame": "torso",
                                                        "preview_only": True})
    for arguments in ({"arm": "both", "position": [0, 0, 0]}, {"arm": "left", "position": [0, 0]},
                      {"arm": "left", "position": [0, 0, 0], "quaternion_wxyz": [2, 0, 0, 0]},
                      {"arm": "left", "position": [0, 0, 0], "frame": "world"},
                      {"arm": "left", "position": [0, 0, 0], "gains": [1]}):
        with pytest.raises(tools.RejectedBeforeAction):
            tools.validate_request("humanoid_free_space_move", arguments)


def test_pixel_geometry():
    from services.humanoid import tools
    K = {"matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]]}
    camera = {"position_m": [0.1, 0.0, 0.5], "quaternion_wxyz": [1, 0, 0, 0]}
    point = tools.pixel_to_pelvis(np.full((480, 640), 2.0), K, camera, 320, 240)
    assert np.allclose(point["position_pelvis_m"], [2.1, 0.0, 0.5])
    left = tools.pixel_to_pelvis(np.full((480, 640), 2.0), K, camera, 70, 240)  # 250 px left of centre
    assert np.allclose(left["position_pelvis_m"], [2.1, 1.0, 0.5])
    assert not tools.pixel_to_pelvis(np.zeros((480, 640)), K, camera, 1, 1)["valid"]
    # A yawed pelvis keeps level coordinates; a pitched pelvis does not.
    yaw = [np.cos(0.4), 0, 0, np.sin(0.4)]
    assert np.allclose(tools.level_from_pelvis([1, 0, 0], yaw), [1, 0, 0])
    pitch = [np.cos(np.pi / 8), 0, np.sin(np.pi / 8), 0]  # 45 degrees nose down
    assert np.allclose(tools.level_from_pelvis([1, 0, 0], pitch), [np.sqrt(0.5), 0, -np.sqrt(0.5)])


def test_task_context_sections():
    from services.humanoid import task_rules, tasks
    for task in tasks.task_names():
        text = task_rules.task_context(task, expert_demonstrations=3)
        assert text.startswith("# Task: ") and "## Episode" in text and "## Success" in text
        assert f"{tasks.native_step_limit(task)} control steps at 50 Hz" in text
        assert "runtime/experts/" in text
        assert "runtime/experts/" not in task_rules.task_context(task)
        camera = task_rules.task_context(task, expert_demonstrations=3, expert_content="images")
        assert "front-camera observations only" in camera and "commanded references, measured" not in camera
        assert "Wall-clock" not in task_rules.task_context(task, live_formal=True)
        for word in ("owner", "brief", "reflection", "scene_facts.py"):
            assert word not in text


def test_semantic40_layout():
    from services.humanoid import robot
    assert robot.ACTION_DIM == 40 and len(robot.JOINT_NAMES) == 29
    assert sum(s.stop - s.start for s in robot.ACTION_SLICES.values()) == 40
    assert robot.rot6d_from_yaw(0.0) == list(robot.IDENTITY_ROT6D)


def test_planner_rot6d_matches_the_semantic40_heading_convention(host_only):
    from services.humanoid import robot
    from services.humanoid.sim.planner import _rot6d
    for yaw in (0.0, 0.4, -1.2, 3.0):
        q = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        assert np.allclose(_rot6d(q), robot.rot6d_from_yaw(yaw))


def test_g1_forward_kinematics_is_host_safe(host_only):
    from services.humanoid import g1
    fk = g1.forward_kinematics({name: 0.0 for name in g1.CANONICAL29})
    assert fk["backend"] == "numpy" and fk["defaulted_joints"] == []
    left, right = (np.asarray(fk["pelvis"][f"{side}_palm"]["position_m"]) for side in ("left", "right"))
    # The zero pose is mirror-symmetric about the sagittal plane, to the URDF's own left/right
    # differences: 10 um at the wrists, 0.1 mm between the Dex3 palm links.
    wrists = [np.asarray(fk["pelvis"][f"{side}_wrist"]["position_m"]) for side in ("left", "right")]
    assert np.allclose(wrists[0] * [1, -1, 1], wrists[1], atol=1e-4) and wrists[0][1] > 0.1
    assert np.allclose(left * [1, -1, 1], right, atol=1e-3)
    assert np.allclose(fk["pelvis"]["pelvis"]["position_m"], 0.0)


def test_humanoid_frames_pass_recording_publication():
    from services.controller.recording import IMAGE_NAME
    for name in ("frame_000010_cam_front_rgb.png", "frame_000010_cam_front_depth.png"):
        assert IMAGE_NAME.fullmatch(name)
    assert IMAGE_NAME.fullmatch("initial_preview_top_rgb.png")  # the robot-only preview cameras still pass
    assert not IMAGE_NAME.fullmatch("frame_000010_cam_rear_rgb.png")
