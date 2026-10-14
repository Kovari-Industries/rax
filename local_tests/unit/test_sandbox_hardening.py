from __future__ import annotations

import base64
import json
import os
from types import SimpleNamespace

import pytest

from services.artifact_io import write_artifacts
from services.robodojo.mcp_server import RoboDojoMCP
from services.robot_preview.mcp_server import RobotPreviewMCP


@pytest.mark.parametrize("service", ["robodojo", "robot_preview"])
@pytest.mark.parametrize("attack", [False, True])
def test_frame_writers_reject_symlink_ancestors(tmp_path, service, attack):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    private = tmp_path / "private"
    private.mkdir()
    if attack:
        (runtime / "frames").symlink_to(private, target_is_directory=True)
    record = {
        "frame_index": 0,
        "attachments": [{"camera": "cam_high", "kind": "rgb"}],
        "depth": {},
    }
    content = [{"data": base64.b64encode(b"fixture image").decode()}]
    if service == "robodojo":
        bridge = RoboDojoMCP.__new__(RoboDojoMCP)
        bridge.sim = SimpleNamespace(run_id="run_demo", step_id=1)
        invoke = lambda: bridge._save_frame_sequence(
            [record], [content], frequency=25, transition={"native_success": True}
        )
    else:
        bridge = RobotPreviewMCP.__new__(RobotPreviewMCP)
        invoke = lambda: bridge._save_sequence(
            record, [], [content], frequency=25, execution={}
        )
    bridge.frame_root = runtime / "frames"
    if attack:
        with pytest.raises(OSError):
            invoke()
        assert not list(private.iterdir())
    else:
        _, manifest = invoke()
        path = tmp_path / manifest
        assert path.is_file()
        assert next(path.parent.glob("*.png")).read_bytes() == b"fixture image"
        if service == "robodojo":
            assert json.loads(path.read_text())["transition"] == {}


def test_directory_replacement_during_write_does_not_redirect(tmp_path, monkeypatch):
    output = tmp_path / "runtime" / "seq"
    output.parent.mkdir()
    private = tmp_path / "private"
    private.mkdir()
    real_open = os.open

    def race_open(path, flags, *args, **kwargs):
        fd = real_open(path, flags, *args, **kwargs)
        if path == "seq" and flags & os.O_DIRECTORY:
            output.rename(output.with_name("moved"))
            output.symlink_to(private, target_is_directory=True)
        return fd

    monkeypatch.setattr(os, "open", race_open)
    write_artifacts(output, {"manifest.json": b"{}"})
    assert not list(private.iterdir())
    assert (output.with_name("moved") / "manifest.json").read_bytes() == b"{}"


@pytest.mark.parametrize("link_type", ["symlink", "hardlink"])
def test_raced_existing_file_is_never_overwritten(tmp_path, monkeypatch, link_type):
    victim = tmp_path / "private.json"
    victim.write_bytes(b"unchanged")
    output = tmp_path / "seq"
    real_open = os.open

    def race_open(path, flags, *args, **kwargs):
        if path == "manifest.json":
            if link_type == "symlink":
                (output / path).symlink_to(victim)
            else:
                os.link(victim, output / path)
        return real_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(os, "open", race_open)
    with pytest.raises(FileExistsError):
        write_artifacts(output, {"manifest.json": b"overwritten"})
    assert victim.read_bytes() == b"unchanged"


def test_demo_decodes_outside_agent_writable_runtime(tmp_path, monkeypatch):
    from services.robodojo import demonstrations as demos

    root = tmp_path / "trial"
    runtime = root / "runtime"
    runtime.mkdir(parents=True)
    observed = []
    original_replace = demos._replace_directory

    def check_staging(staging, target):
        observed.append(staging)
        assert staging.parent == root
        assert not staging.is_relative_to(runtime)
        original_replace(staging, target)

    monkeypatch.setattr(demos, "_replace_directory", check_staging)
    result = demos.provision_trial_demonstration(
        video_path=None, target=runtime / "demonstrations",
        task="make_kong", context_kind="none",
    )
    assert result["kind"] == "none"
    assert (runtime / "demonstrations").is_dir()
    assert observed and not observed[0].exists()
