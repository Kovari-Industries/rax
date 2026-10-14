"""Capabilities must agree across discovery, dispatch, files and workspace."""
import copy
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
import pytest

from services.mcp_contract import Contract, ObservationProfile
from services.mcp_workspace import compose, render
from services.controller.gateway import Gateway
from services.controller.recording import RunRecording
from services.robodojo.mcp_server import RoboDojoMCP
from services.robot_preview.mcp_server import RobotPreviewMCP

PROJECT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("profile", ["rgbd", "rgb-only"])
@pytest.mark.parametrize("count", [1, 2])
def test_contract_matrix(profile, count):
    c = Contract(observation_profile=profile, environments=count)
    definitions = c.robot_definitions()
    assert {t["name"] for t in definitions} == c.robot_names
    for t in definitions:
        expected = count > 1 and t["name"] != "robodojo_pose_math"
        assert ("env_id" in t["inputSchema"].get("properties", {})) == expected
        assert ("env_id" in t["inputSchema"].get("required", [])) == expected
    args = {"actions": [[0.0] * 14]}
    if count > 1:
        with pytest.raises(ValueError, match="env_id"):
            c.require_robot("robodojo_step", args)
        c.require_robot("robodojo_step", {**args, "env_id": count - 1})
        for bad in (True, -1, count, "0", None):
            with pytest.raises(ValueError, match="env_id"):
                c.require_robot("robodojo_step", {**args, "env_id": bad})
    else:
        c.require_robot("robodojo_step", args)
        with pytest.raises(ValueError, match="env_id"):
            c.require_robot("robodojo_step", {**args, "env_id": 0})
    with pytest.raises(ValueError, match="env_id"):
        c.require_robot("robodojo_pose_math", {"env_id": 0})
    if profile == "rgb-only":
        with pytest.raises(PermissionError):
            c.require_robot("robodojo_pixel_to_position", {})
    with pytest.raises(ValueError, match="Multiple environments"):
        Contract(phase="isolated", observation_profile=profile, environments=2)


def test_isolated_gateway_discovery_and_forged_calls():
    c = Contract(phase="isolated", observation_profile="rgb-only")
    calls = []
    backend = SimpleNamespace(
        tools=c.robot_definitions,
        call=lambda name, args: (calls.append((name, args)) or {"step_id": 1}, []))
    gateway = Gateway(backend, None, contract=c, audit=lambda *_: None,
                      charge=lambda *_: None, refund=lambda *_: None, budget=lambda: {})
    names = {t["name"] for t in gateway.definitions()}
    assert "gemini_generate" in names
    assert not {"start_episode", "rehearse", "submit", "robodojo_pixel_to_position"} & names
    token = gateway.acquire()
    for name, args in [("robodojo_pixel_to_position", {}),
                       ("robodojo_step", {"env_id": 0, "actions": [[0] * 14]}),
                       ("start_episode", {})]:
        response = gateway.handle(token, {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": name, "arguments": args}})
        assert "error" in response or response["result"].get("isError")
    assert calls == [] and not gateway.terminal
    for params in (None, [1], 'robodojo_step'):  # A malformed request is an error reply, not a crash.
        response = gateway.handle(token, {"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": params})
        assert response["error"]["data"]["type"] == "TypeError"
    assert calls == []


def frame():
    return {"cam_high": np.zeros((3, 4, 3), dtype=np.uint8),
            "cam_high_depth_m": np.ones((3, 4), dtype=np.float32),
            "camera_parameters": {"cam_high": {"intrinsic_matrix": [[1]]}},
            "states": np.zeros(14), "eef_positions": np.zeros((2, 3)),
            "eef_quaternions_wxyz": np.zeros((2, 4))}


@pytest.mark.parametrize("profile,images", [("rgbd", 2), ("rgb-only", 1)])
def test_native_and_preview_images_and_saved_files(tmp_path, profile, images):
    server = RoboDojoMCP.__new__(RoboDojoMCP)
    server.observation_profile = profile
    server.sim = SimpleNamespace(episode_id="episode", step_id=2, task="make_kong",
                                 run_id="run", metadata={"control_frequency_hz": 25})
    server.frame_root = tmp_path / "runtime/frames"
    packet, content = server._observation_sequence_packet([frame(), frame()],
        transition={"steps": [{"step_id": 1}, {"step_id": 2}]})
    assert len(content) == images
    assert [a["content_index"] for a in packet["attachments"]] == list(range(1, images + 1))
    manifest_path = tmp_path / packet["frame_sequence"]["manifest_path"]
    manifest = json.loads(manifest_path.read_text())
    assert ("camera_parameters" in manifest["frames"][0]) == (profile == "rgbd")
    assert ("depth" in packet) == (profile == "rgbd")
    assert len(list(manifest_path.parent.glob("*.png"))) == 2 * images
    preview = RobotPreviewMCP.__new__(RobotPreviewMCP)
    preview.observation_profile = profile
    preview.frame_root = tmp_path / "preview"
    blocks = []
    encoded = preview._encode_observation(frame(), blocks)
    assert len(blocks) == images
    _, path = preview._save_sequence(encoded, [{**encoded, "frame_index": 0}],
        [blocks, blocks], frequency=25, execution={})
    saved = json.loads((preview.frame_root / "robot_preview" / Path(path).parent.name / "manifest.json").read_text())
    assert ("camera_parameters" in saved["initial_observation"]) == (profile == "rgbd")
    assert ("camera calibration" in preview.description(profile)["outputs"]) == (profile == "rgbd")


def test_rgb_recording_filters_before_copying_depth(tmp_path):
    native = tmp_path / "native"
    folder = native / "runtime/frames/seq"
    folder.mkdir(parents=True)
    rgb = "frame_000000_cam_high_rgb.png"
    (folder / rgb).write_bytes(b"rgb")
    # Deliberately absent depth: publication must not even try to read it.
    manifest = {"frames": [{"camera_parameters": {"private": True}, "depth": {"cam_high": {}},
        "files": [{"kind": kind, "path": "runtime/frames/seq/" + name}
                  for kind, name in [("rgb", rgb), ("depth", "frame_000000_cam_high_depth.png")]]}]}
    (folder / "manifest.json").write_text(json.dumps(manifest))
    recording = RunRecording(tmp_path / "published", "interactive-" + "a" * 32, observation_profile="rgb-only")
    value = {"frame_sequence": {"manifest_path": "runtime/frames/seq/manifest.json",
             "streams": [{"kind": "rgb"}, {"kind": "depth"}]}}
    published = recording.sequence(value, native)
    assert published["streams"] == [{"kind": "rgb"}]
    exported = next(recording.root.rglob("manifest.json"))
    assert "depth" not in exported.read_text()
    assert "camera_parameters" not in exported.read_text()
    assert len(list(recording.root.rglob("*.png"))) == 1


@pytest.mark.parametrize("profile", ["rgbd", "rgb-only"])
def test_workspace_composition_and_references(tmp_path, profile):
    c = Contract(observation_profile=profile)
    target = tmp_path / "workspace"
    shutil.copytree(PROJECT / "auto_research_agent", target,
                    ignore=shutil.ignore_patterns("runtime", "__pycache__"))
    compose(target, c, task="make_kong")
    written = json.loads((target / "MCP_CONTRACT.json").read_text())
    assert {k: written[k] for k in c.manifest()} == c.manifest()
    assert written['tools'] == sorted(c.tool_names) and written['role'] == 'exploration'
    calibration = target / ".agents/skills/object-localization"
    assert calibration.exists()  # One localization skill per observation mode.
    assert ("robodojo_pixel_to_position" in (calibration / "SKILL.md").read_text()) == c.observation.geometry
    # RGB-only gets no calibration: no camera model, table priors or triangulation helper.
    assert not list(target.rglob("CALIBRATION.md")) and not list(target.rglob("triangulation.py"))
    files = list(target.glob("*.md")) + list((target / ".agents").rglob("*.md"))
    for path in files:
        text = path.read_text()
        assert "<!-- capability:" not in text
        if not c.observation.geometry:
            assert "robodojo_pixel_to_position" not in text
    interface = (target / ".agents/skills/robot-interface/SKILL.md").read_text()
    assert "robodojo_step_eef" in interface and "free_space_move" in interface
    for path in [target / "MCP_SESSION.md",
                 target / ".agents/skills/manual-robot-control/SKILL.md",
                 target / ".agents/skills/wrist-camera-inspection/SKILL.md"]:
        assert "object-localization/SKILL.md" in path.read_text()


def test_contracts_do_not_mutate_shared_schemas_or_each_other():
    baseline = RoboDojoMCP.tool_definitions()
    original = copy.deepcopy(baseline)
    a = Contract(observation_profile="rgb-only", environments=3)
    once = a.select(baseline)
    assert a.select(once) == once
    assert baseline == original
    assert "robodojo_pixel_to_position" in Contract().robot_names
    with pytest.raises(ValueError, match="Unknown workspace template name"):
        render("{% if unknown %}\na\n{% endif %}\n", a)
    with pytest.raises(ValueError, match="markup survived"):
        render("<!-- capability:camera-geometry -->\na\n<!-- otherwise -->\nb\n<!-- end-capability -->", a)


def test_routing_is_pure_and_rejects_global_or_unavailable_tools():
    contract = Contract(observation_profile="rgb-only", environments=3)
    args = {"env_id": 2, "actions": [[0.0] * 14]}
    before = copy.deepcopy(args)
    index, payload = contract.route("robodojo_step", args)
    assert index == 2 and payload == {"actions": args["actions"]} and args == before
    for name in ("gemini_generate", "robodojo_pose_math", "robodojo_pixel_to_position", "submit"):
        with pytest.raises(PermissionError):
            contract.route(name, args)
    with pytest.raises(ValueError):
        Contract().route("robodojo_step", args)
