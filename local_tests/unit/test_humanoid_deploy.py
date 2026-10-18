"""HumanoidArena deployment through the real launcher path: workspace, TASK.md and mounts."""
import importlib.util
import json
from pathlib import Path

import numpy as np

import pytest

from harness.codex_cli.auto_research import deploy, readable
from services.controller.config import Configuration, Limits

ROBOT_SKILLS = ("robot-interface", "free-space-move", "controller-tracking", "controller-field-guide",
                "object-localization")
METHOD_SKILLS = ("keep-trying", "spend-the-budget", "measure-success", "verify-committal-actions",
                 "match-precision-to-error", "evidence-and-regression", "avoid-overfitting", "prioritize-by-funnel",
                 "learn-from-incidents", "engineer-the-controller", "change-discipline", "rehearse-the-controller")
OTHER_ROBOTS = ("policies", "pose-math", "action-preview", "action-search", "wrist-camera-inspection",
                "base-navigation", "motion-preview")


def humanoid_configuration(tmp_path, experts, content="full"):
    limits = Limits(60, 512, 1, 32, 32, 4 * 1024 * 1024)
    return Configuration(tmp_path / "private", "opendoor", (0, 1, 2), 0, 0,
                         "GPU-aaaa", limits, limits, limits, benchmark="humanoid",
                         formal_eval_seed=1, formal_episodes=60, expert_demonstrations=experts,
                         expert_content=content)


@pytest.fixture
def expert_cache(tmp_path, monkeypatch):
    """Three minimal installed expert episodes, as setup_humanoid_demos.py writes them."""
    from services.humanoid import demonstrations
    root = tmp_path / "reference-demos" / "humanoid"
    experts = root / "opendoor" / "experts"
    demos = []
    for number in range(3):
        directory = experts / "demos" / f"{number:02d}"
        directory.mkdir(parents=True)
        (directory / "trajectory.jsonl").write_text('{"frame": 0}\n')
        demos.append({"directory": f"demos/{number:02d}", "trajectory_id": f"sonic_{number}", "files": {}})
    (experts / "manifest.json").write_text(json.dumps({"schema": demonstrations.SCHEMA, "task": "opendoor",
                                                       "demos": demos}))
    monkeypatch.setattr(demonstrations, "demos_root", lambda: root)
    return experts


@pytest.fixture
def expert_image_cache(tmp_path, expert_cache):
    """The camera-only set beside the complete episodes, as setup_humanoid_demos.py --images writes it."""
    from services.humanoid import demonstrations
    images = expert_cache.parent / "expert-images"
    demos = []
    for number in range(3):
        directory = images / "demos" / f"{number:02d}"
        (directory / "frames").mkdir(parents=True)
        for name in demonstrations.IMAGE_FILES:
            (directory / name).write_text("[]\n" if name.endswith(".json") else "x")
        demos.append({"directory": f"demos/{number:02d}", "trajectory_id": f"sonic_{number}", "files": {}})
    (images / "manifest.json").write_text(json.dumps({"schema": demonstrations.IMAGE_SCHEMA, "task": "opendoor",
                                                      "demos": demos}))
    return images


def deployed_workspace(tmp_path, monkeypatch, config):
    provider = tmp_path / "provider"
    provider.mkdir()
    (provider / "openrouter.config.toml").write_text(
        'model_provider="fixture"\n[model_providers.fixture]\nname="Fixture"\nbase_url="http://127.0.0.1:1"\nwire_api="responses"\n'
        '[rax]\nkey_file="model-key"\n')
    (provider / "model-key").write_text("sk-fixture\n")
    (provider / "model-key").chmod(0o600)
    return deploy(config, provider_home=provider, codex="/usr/bin/true")


@pytest.mark.parametrize("experts, content", [(3, "full"), (0, "full"), (3, "images")])
def test_humanoid_deploy(tmp_path, monkeypatch, expert_image_cache, experts, content):
    expert_cache = expert_image_cache if content == "images" else expert_image_cache.parent / "experts"
    config = humanoid_configuration(tmp_path, experts, content)
    deployed = deployed_workspace(tmp_path, monkeypatch, config)
    workspace = deployed["workspace"]
    skills = workspace / ".agents/skills"
    for name in (*ROBOT_SKILLS, *METHOD_SKILLS, "experience-memory", "autonomous-control", "manual-robot-control",
                 "gemini"):
        assert (skills / name / "SKILL.md").is_file(), name
    for name in OTHER_ROBOTS:
        assert not (skills / name).exists(), name
    assert (skills / "learn-from-experts").exists() == bool(experts)
    if experts:
        guide = (skills / "learn-from-experts/SKILL.md").read_text()
        assert ("front_rgb.mp4" in guide) == (content == "images")
        assert ("trajectory.jsonl" in guide) == (content == "full")
    assert not (skills / "in-context-action-learning").exists()  # no image demonstration
    # The robot model (camera calibration included) ships with RGB-D only.
    assert (skills / "robot-interface/scripts/robot.json").is_file()

    task = (workspace / "TASK.md").read_text()
    assert task.startswith("# Task: Open the door")
    assert "1800 control steps at 50 Hz" in task and "## Success" in task
    assert ("runtime/experts/" in task) == bool(experts)
    assert ("front-camera observations only" in task) == (content == "images")
    session = (workspace / "MCP_SESSION.md").read_text()
    assert "humanoid_step" in session
    agents = (workspace / "AGENTS.md").read_text()
    assert "engineer-the-controller" in agents and "wrist-camera-inspection" not in agents

    # The expert episodes appear at runtime/experts, linked to the installed set the sandbox reads.
    link = workspace / "runtime/experts"
    assert link.is_symlink() == bool(experts)
    if experts:
        assert link.resolve() == expert_cache.resolve()
    from harness.claude_cli.auto_research import readable as claude_readable
    assert readable(config, deployed)["references"] == ([expert_cache.resolve()] if experts else [])
    assert (expert_cache.resolve() in claude_readable(config, deployed)) == bool(experts)
    assert config.root / "published" in claude_readable(config, deployed)


def test_expert_episodes_must_be_installed(tmp_path, monkeypatch):
    from services.benchmarks import benchmark
    from services.humanoid import demonstrations
    from services.demonstration_io import DemonstrationError
    monkeypatch.setattr(demonstrations, "demos_root", lambda: tmp_path / "missing")
    with pytest.raises(DemonstrationError, match="not installed"):
        benchmark("humanoid").reference_mounts(humanoid_configuration(tmp_path, 3))
    assert benchmark("humanoid").reference_mounts(humanoid_configuration(tmp_path, 0)) == []


def test_camera_only_experts_need_expert_episodes(tmp_path):
    with pytest.raises(ValueError, match="only with expert_demonstrations"):
        humanoid_configuration(tmp_path, 0, "images")
    with pytest.raises(ValueError, match="expert_content must be one of"):
        humanoid_configuration(tmp_path, 3, "video")


def test_camera_only_set_holds_no_recording(tmp_path, monkeypatch):
    """setup_humanoid_demos.py --images keeps the video, previews and depth, and nothing else."""
    from services.humanoid import demonstrations
    root = tmp_path / "reference-demos" / "humanoid"
    experts = root / "opendoor" / "experts"
    demos = []
    for number in range(3):
        directory = experts / "demos" / f"{number:02d}"
        (directory / "frames").mkdir(parents=True)
        (directory / "trajectory.jsonl").write_text('{"vla_action": [0.1]}\n')
        if number < 2:  # The last episode keeps its RGB stream in the NPZ only.
            (directory / "videos").mkdir()
            (directory / "videos" / "front_rgb_source.mp4").write_bytes(b"mp4")
        (directory / "frames" / "00000000.jpg").write_bytes(b"jpg")
        (directory / "frames" / "index.json").write_text(json.dumps(
            [{"file": "frames/00000000.jpg", "video_frame": 0, "control_frame": 0, "elapsed_s": 0.0}]))
        (directory / "metadata.json").write_text(json.dumps(
            {"vision_rgb_video_num_frames": 5, "vision_rgb_video_fps": 50.0,
             "episode_init_env_obj_door_position": [1, 2, 3]}))
        (directory / "source.json").write_text(json.dumps(
            {"license": "CC-BY-NC-4.0", "repository": "repo", "revision": "abc", "trajectory_id": f"sonic_{number}"}))
        np.savez(directory / "episode.npz", vision_depth=np.full((5, 16, 32, 1), 1.5, np.float32),
                 vision_rgb=np.full((5, 16, 32, 3), 100, np.uint8),
                 vision_frame_indices=np.arange(5), vla_action=np.zeros((5, 3)))
        demos.append({"directory": f"demos/{number:02d}", "trajectory_id": f"sonic_{number}", "files": {}})
    (experts / "manifest.json").write_text(json.dumps({"schema": demonstrations.SCHEMA, "task": "opendoor",
                                                       "demos": demos}))
    spec = importlib.util.spec_from_file_location(
        "setup_humanoid_demos", Path(__file__).resolve().parents[2] / "scripts/setup_humanoid_demos.py")
    setup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(setup)
    monkeypatch.setattr(demonstrations, "demos_root", lambda: root)
    setup.install_images("opendoor", root)
    episodes = demonstrations.cached_expert_images(root, "opendoor")
    assert len(episodes) == 3
    for episode in episodes:
        files = {str(path.relative_to(episode)) for path in episode.rglob("*") if path.is_file()}
        assert files == {*demonstrations.IMAGE_FILES, "frames/00000000.jpg"}
        with np.load(episode / "depth.npz") as arrays:
            assert sorted(arrays.files) == ["depth", "frame_index"]
            assert arrays["depth"].shape == (5, 16, 32) and arrays["depth"].dtype == np.float16
        text = " ".join((episode / name).read_text() for name in ("README.md", "metadata.json"))
        assert "vla_action" not in text and "env_obj" not in text and "trajectory.jsonl" not in text
    assert (episodes[0] / "front_rgb.mp4").read_bytes() == b"mp4"
    import imageio.v2 as imageio
    with imageio.get_reader(episodes[2] / "front_rgb.mp4") as reader:
        assert sum(1 for _ in reader) == 5
    assert demonstrations.expert_root("opendoor", "images") == (root / "opendoor" / "expert-images").resolve()


def test_raw_episode_derivation_writes_the_reference_views(tmp_path):
    """setup_humanoid_demos.py derives the JSON view, index, catalog and previews of one raw episode."""
    from services.humanoid.expert_kinematics import ISAACLAB_JOINTS, G1Kinematics
    spec = importlib.util.spec_from_file_location(
        "setup_humanoid_demos", Path(__file__).resolve().parents[2] / "scripts/setup_humanoid_demos.py")
    setup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(setup)
    # A pelvis-rooted chain carrying every released joint and the virtual points' bodies.
    links = {"left_wrist_yaw_joint": "left_wrist_yaw_link", "right_wrist_yaw_joint": "right_wrist_yaw_link",
             "waist_pitch_joint": "torso_link"}
    bodies = "".join(f'<body name="{links.get(n, n + "_link")}" pos="0 0 0.1"><joint name="{n}" axis="0 1 0"/>'
                     for n in ISAACLAB_JOINTS) + "</body>" * len(ISAACLAB_JOINTS)
    mjcf = tmp_path / "g1.xml"
    mjcf.write_text(f'<mujoco><worldbody><body name="pelvis">{bodies}</body></worldbody></mujoco>')
    episode = tmp_path / "demo"
    episode.mkdir()
    count = 6
    np.savez(episode / "episode.npz", num_frames=np.array(count), task=np.array("Isaac-Move-Open-Door-G129-Dex3-Wholebody"),
             meta_control_dt=np.array(0.02), robot_qpos_before_decimation=np.zeros((count, 29)),
             robot_root_position=np.zeros((count, 3)), robot_root_orientation=np.tile([1.0, 0, 0, 0], (count, 1)),
             vision_frame_indices=np.arange(count), vision_rgb=np.full((count, 8, 12, 3), 90, np.uint8),
             episode_init_env_obj_door_position=np.zeros((1, 3)))
    previews = setup.derive("opendoor", "Isaac-Move-Open-Door-G129-Dex3-Wholebody", episode, G1Kinematics(mjcf), mjcf, 3)
    assert previews == 3 and len(list((episode / "frames").glob("*.jpg"))) == 3
    lines = (episode / "trajectory.jsonl").read_bytes()
    for entry in json.loads((episode / "trajectory-index.json").read_text()):
        row = json.loads(lines[entry["offset"]:entry["offset"] + entry["length"]])
        assert row["frame"] == entry["frame"] and set(row["derived_measured_virtual_points_world"]) == {
            "left_hand", "right_hand", "head"}
    metadata = json.loads((episode / "metadata.json").read_text())
    assert metadata["episode_init_env_obj_door_position"] == [[0.0, 0.0, 0.0]]
    assert metadata["derived_geometry"]["mjcf_sha256"] == setup.sha256(mjcf)
    assert "vision_rgb" in json.loads((episode / "raw-array-catalog.json").read_text())
