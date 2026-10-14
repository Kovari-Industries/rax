"""Real free-space parity, deliberately separate from mocked unit coverage."""

import base64
import copy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np
import pytest
from PIL import Image

from services.robodojo.mcp_server import SimulatorProcess, _png_data
from services.robot_preview.mcp_server import PreviewProcess, _rgb_png


@pytest.fixture
def gpu_runtime(host_artifact_root):
    root = Path(__file__).resolve().parents[2]
    python = Path(os.environ.get("ROBODOJO_PYTHON", root / "runtime/envs/robodojo/bin/python"))
    if not python.is_file() or not (root / "RoboDojo/task/RoboDojo/config/make_kong.yml").is_file():
        pytest.skip("GPU integration needs the bootstrapped RoboDojo runtime and source")
    if not shutil.which("nvidia-smi"):
        pytest.skip("NVIDIA driver unavailable")
    try:
        probe = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,uuid,memory.free,utilization.gpu", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=15,
        )
        if probe.returncode:
            pytest.skip("NVIDIA driver inaccessible (a sandbox may hide host GPUs)")
        visible = os.environ.get("CUDA_VISIBLE_DEVICES")
        allowed = None if visible is None else set(visible.split(","))
        candidates = []
        for line in probe.stdout.splitlines():
            index, uuid, free, utilization = map(str.strip, line.split(","))
            if allowed is not None and index not in allowed and uuid not in allowed:
                continue
            if int(free) >= 24 * 1024 and int(utilization) < 20:
                candidates.append((int(free), uuid))
        if not candidates:
            pytest.skip("No visible idle GPU with >=24 GiB free for two Isaac services")
        gpu = max(candidates)[1]
        dependency = subprocess.run(
            [str(python), "-c", "import importlib.util; import torch; "
             "assert importlib.util.find_spec('isaaclab'); "
             "assert importlib.util.find_spec('isaacsim'); assert torch.cuda.is_available()"],
            env={**os.environ, "CUDA_VISIBLE_DEVICES": gpu},
            capture_output=True, text=True, timeout=30,
        )
        if dependency.returncode:
            pytest.skip("Native Python needs Isaac Lab, Isaac Sim, and usable CUDA: " + dependency.stderr[-500:])
    except (OSError, subprocess.TimeoutExpired) as exc:
        pytest.skip(f"GPU preflight unavailable: {exc}")
    runtime = (root / "runtime").resolve()
    if not runtime.is_relative_to(host_artifact_root) or not os.access(runtime, os.W_OK):
        pytest.skip(f"GPU artifacts require a writable runtime/ under the artifact root {host_artifact_root}")
    output = runtime / "integration-tests"
    output.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="preview-parity-", dir=output))
    print(f"GPU integration artifacts: {output}")
    return root, str(python), gpu, output


def _check_images(observation, encode):
    for camera in ("cam_high", "cam_left_wrist", "cam_right_wrist"):
        rgb = np.asarray(observation[camera])[..., :3]
        assert rgb.dtype == np.uint8 and rgb.ndim == 3 and rgb.shape[-1] == 3
        assert rgb.std() > 1, f"Blank render: {camera}"
        decoded = np.asarray(Image.open(io.BytesIO(base64.b64decode(encode(rgb)))))
        np.testing.assert_array_equal(decoded, rgb)
        depth = np.asarray(observation[f"{camera}_depth_m"])
        assert depth.shape[:2] == rgb.shape[:2]
        assert np.any(np.isfinite(depth) & (depth > 0)), f"No valid depth: {camera}"


def _errors(actual, predicted):
    qa = np.asarray(actual["eef_quaternions_wxyz"], dtype=float)
    qp = np.asarray(predicted["eef_quaternions_wxyz"], dtype=float)
    qa /= np.linalg.norm(qa, axis=-1, keepdims=True)
    qp /= np.linalg.norm(qp, axis=-1, keepdims=True)
    joints = [*range(6), *range(7, 13)]
    return {
        "joint_rad": float(np.max(np.abs(np.asarray(actual["states"])[joints] - np.asarray(predicted["states"])[joints]))),
        "gripper": float(np.max(np.abs(np.asarray(actual["states"])[[6, 13]] - np.asarray(predicted["states"])[[6, 13]]))),
        "eef_m": float(np.max(np.linalg.norm(np.asarray(actual["eef_positions"]) - np.asarray(predicted["eef_positions"]), axis=-1))),
        "eef_rad": float(np.max(2 * np.arccos(np.clip(np.abs(np.sum(qa * qp, axis=-1)), 0, 1)))),
    }


def _check_rendered_table_calibration(observation):
    """Check real depth against make_kong's table, not another calibration API.

    These two empty tabletop patches avoid tiles/arms in the seed-0 fixture.
    The old high-camera fy produced >1 cm median height error here.
    """
    camera = observation["camera_parameters"]["cam_high"]
    intrinsic = np.asarray(camera["intrinsic_matrix"])
    transform = np.asarray(camera["camera_to_world_usd"])
    depth = np.asarray(observation["cam_high_depth_m"])
    mask = np.zeros(depth.shape, dtype=bool)
    mask[120:230, 190:240] = True
    mask[295:390, 230:410] = True
    v, u = np.where(mask)
    rays = np.stack(((u - intrinsic[0, 2]) / intrinsic[0, 0],
                     -(v - intrinsic[1, 2]) / intrinsic[1, 1],
                     -np.ones_like(u)), axis=1)
    world = (rays * depth[mask, None]) @ transform[:3, :3].T + transform[:3, 3]
    error = np.abs(world[:, 2] - 0.765)
    assert np.percentile(error, 95) < 0.002, np.percentile(error, [50, 95])


@pytest.mark.gpu
def test_real_environment_smoke_and_preview_free_space_accuracy(gpu_runtime):
    root, python, gpu, output = gpu_runtime
    actual = SimulatorProcess(project_root=root, python=python, output_root=output / "native")
    preview = PreviewProcess(root, python, output / "preview", 0, 240, sim_gpu=gpu)
    try:
        actual.start(task="make_kong", seed=0, eval_seed=0, run_id="smoke",
                     sim_gpu=gpu, sim_port=0, startup_timeout=240,
                     enable_depth=True, enable_camera_parameters=True)
        initial = actual.request("teacher_observation")
        # Settle at the untouched home pose before comparing a state-only preview.
        settled = actual.request("chunk_step", actions=[initial["states"].tolist()] * 15)
        actual.step_id = settled["step_id"]
        assert len(settled["frames"]) == 15, "Native episode ended during settling"
        initial = settled["frames"][-1]
        _check_images(initial, _png_data)
        _check_rendered_table_calibration(initial)
        # Negative control: ensure this fixture actually detects the original
        # aperture mismatch rather than just comparing two identical services.
        incorrect = copy.deepcopy(initial)
        incorrect["camera_parameters"]["cam_high"]["intrinsic_matrix"][1][1] = 336.46432
        with pytest.raises(AssertionError):
            _check_rendered_table_calibration(incorrect)
        # Small, symmetric wrist rotations at home avoid table/object contacts.
        actions = np.repeat(np.asarray(initial["states"])[None, :], 12, axis=0)
        actions[:, 5] += np.linspace(0, 0.06, len(actions))
        actions[:, 12] -= np.linspace(0, 0.06, len(actions))
        predicted = preview.request(robot_model="dual_arx_x5", camera_model="robodojo_gemini_wrist_v1",
                                    initial_state=initial["states"].tolist(), actions=actions.tolist())
        executed = actual.request("chunk_step", actions=actions.tolist())
        actual.step_id = executed["step_id"]
        observed_frames = executed["frames"]
        predicted_frames = predicted["execution"]["frames"]
        assert len(observed_frames) == len(predicted_frames) == len(actions)
        # Matching stationary frames would not establish execution accuracy.
        displacement = np.asarray(observed_frames[-1]["states"]) - initial["states"]
        assert displacement[5] > 0.02 and displacement[12] < -0.02
        _check_images(predicted_frames[-1], _rgb_png)
        _check_images(observed_frames[-1], _png_data)
        errors = [_errors(a, p) for a, p in zip(observed_frames, predicted_frames, strict=True)]
        maxima = {key: max(row[key] for row in errors) for key in errors[0]}
        # Explicit smoke tolerances, not a certification for contacts or arbitrary motion.
        limits = {"joint_rad": 0.01, "gripper": 0.02, "eef_m": 0.005, "eef_rad": 0.02}
        (output / "accuracy.json").write_text(json.dumps({"max_errors": maxima, "limits": limits, "per_frame": errors}, indent=2))
        for key, limit in limits.items():
            assert maxima[key] <= limit, f"{key}: {maxima[key]} > {limit}; see {output}"
        for camera in ("cam_high", "cam_left_wrist", "cam_right_wrist"):
            native = observed_frames[-1]["camera_parameters"][camera]
            model = predicted_frames[-1]["camera_parameters"][camera]
            assert model["resolution"] == native["resolution"]
            np.testing.assert_allclose(model["intrinsic_matrix"], native["intrinsic_matrix"], atol=1e-4, rtol=0)
            np.testing.assert_allclose(model["camera_to_world_usd"], native["camera_to_world_usd"], atol=0.02, rtol=0)
    finally:
        try:
            preview.close()
        finally:
            actual.stop()
