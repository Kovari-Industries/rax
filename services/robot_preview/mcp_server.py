"""The robot preview tools over the object-free robot-model Isaac preview."""

from __future__ import annotations

import base64
import io
import json
import os
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress
from pathlib import Path

from services.json_io import jsonable
from services.artifact_io import write_artifacts
from services.mcp_contract import observation_for
from services.simulator.io import RejectedBeforeAction, SimulatorError, exit_report

CAMERAS = ("cam_high", "cam_left_wrist", "cam_right_wrist")


_jsonable = jsonable


def _rgb_png(value):
    from PIL import Image

    array = value
    if array.shape[-1] > 3:
        array = array[..., :3]
    stream = io.BytesIO()
    Image.fromarray(array).save(stream, format="PNG")
    return base64.b64encode(stream.getvalue()).decode("ascii")


def _depth_png(value):
    import numpy as np
    from PIL import Image

    depth = np.asarray(value, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    valid = np.isfinite(depth) & (depth > 0)
    encoded = np.zeros(depth.shape, dtype=np.uint16)
    encoded[valid] = np.rint(np.clip(depth[valid] * 1000.0, 0, 65535)).astype(np.uint16)
    stream = io.BytesIO()
    Image.fromarray(encoded).save(stream, format="PNG")
    values = depth[valid]
    return base64.b64encode(stream.getvalue()).decode("ascii"), {
        "shape": [int(depth.shape[1]), int(depth.shape[0])],
        "encoding": "uint16_png",
        "unit": "millimeters",
        "meters_per_unit": 0.001,
        "invalid_value": 0,
        "invalid_pixels": int((~valid).sum()),
        "min_meters": float(values.min()) if values.size else None,
        "max_meters": float(values.max()) if values.size else None,
    }


class PreviewProcess:
    def __init__(self, root, python, output_root, port, startup_timeout, sim_gpu=None):
        self.root = root
        self.python = python
        self.output_root = output_root
        self.port = port
        self.startup_timeout = startup_timeout
        self.sim_gpu = sim_gpu
        self.process = self.rpc = None
        self.log_path = None
        self.ready = threading.Event()
        self.metadata = None
        self.bound_port = None

    def _read_log(self, stream, log):
        try:
            for line in iter(stream.readline, ""):
                log.write(line)
                log.flush()
                if '"event": "ready"' not in line:
                    continue
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(packet, dict) and packet.get("event") == "ready":
                    self.bound_port = int(packet["port"])
                    self.ready.set()
        finally:
            stream.close()
            log.close()

    def start(self):
        if self.rpc is not None:
            return
        output = self.output_root / "robot_model_preview"
        output.mkdir(parents=True, exist_ok=True)
        log_path = (
            output / f"service_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.log"
        )
        self.log_path = log_path
        self.ready.clear()
        self.bound_port = None
        environment = dict(os.environ)
        environment["PYTHONUNBUFFERED"] = "1"
        if self.sim_gpu is not None:
            environment["CUDA_VISIBLE_DEVICES"] = str(self.sim_gpu)
        environment["OMNI_KIT_ACCEPT_EULA"] = environment.get(
            "OMNI_KIT_ACCEPT_EULA", "Y"
        )
        xdg_runtime = self.output_root.parent / "preview_xdg_runtime"
        xdg_runtime.mkdir(parents=True, exist_ok=True)
        xdg_runtime.chmod(0o700)
        environment["XDG_RUNTIME_DIR"] = str(xdg_runtime)
        source = self.root / "RoboDojo"
        environment["PYTHONPATH"] = os.pathsep.join(
            str(path)
            for path in (
                self.root,
                source,
                source / "XPolicyLab",
                source / "third_party" / "curobo",
            )
        ) + (
            os.pathsep + environment["PYTHONPATH"]
            if environment.get("PYTHONPATH")
            else ""
        )
        command = [
            self.python,
            "-m",
            "services.robot_preview.server",
            "--robodojo-root",
            str(source),
            "--robot-model",
            "dual_arx_x5",
            "--camera-model",
            "robodojo_gemini_wrist_v1",
            "--port",
            str(self.port),
        ]
        log = log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(
                command,
                cwd=output,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except Exception:
            log.close()
            raise
        if self.process.stdout is None:
            log.close()
            self.close()
            raise RuntimeError("Robot preview stdout pipe was not created")
        threading.Thread(
            target=self._read_log,
            args=(self.process.stdout, log),
            name="robot-preview-log",
            daemon=True,
        ).start()
        deadline = time.monotonic() + self.startup_timeout
        try:
            while not self.ready.wait(
                timeout=min(1.0, max(0.0, deadline - time.monotonic()))
            ):
                if self.process.poll() is not None:
                    time.sleep(1)  # Let the log reader drain the pipe.
                    raise RuntimeError("Robot preview Isaac service exited during startup: "
                                       + exit_report(self.process, log_path))
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Robot preview Isaac service did not become ready within "
                                       f"{self.startup_timeout:g}s; " + exit_report(self.process, log_path))
            if str(self.root) not in sys.path:
                sys.path.insert(0, str(self.root))
            from services.simulator.protocol import RPCClient

            if self.bound_port is None:
                raise RuntimeError("Robot preview service did not report its RPC port")
            self.rpc = RPCClient(
                "127.0.0.1", self.bound_port, timeout=self.startup_timeout
            )
            self.metadata = self._request("metadata")
        except BaseException:
            # A preview that never became usable is stopped now; the next call starts a new one.
            self.close(grace=0)
            raise

    def _request(self, op, **arguments):
        try:
            return self.rpc.request(op, **arguments)
        except (SimulatorError, RejectedBeforeAction):
            raise  # The preview answered with an error and is still usable.
        except Exception as exc:
            # The connection is gone. Report how the preview ended, and stop it so the next
            # call starts a fresh one instead of reusing the closed client.
            process = self.process
            if process is not None:
                with suppress(subprocess.TimeoutExpired):
                    process.wait(timeout=5)
            report = exit_report(process, self.log_path)
            self.close(grace=0)
            raise RuntimeError(f"Robot preview {op} failed with {type(exc).__name__}: {exc}; {report}; "
                               "the next robot_preview_run starts a new preview") from exc

    def request(self, **arguments):
        self.start()
        return self._request("preview", **arguments)

    def close(self, grace=30):
        if self.rpc is not None:
            with suppress(OSError, RuntimeError):
                self.rpc.close()
            self.rpc = None
        if self.process is not None:
            try:
                self.process.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=10)
                except (OSError, subprocess.TimeoutExpired):
                    with suppress(OSError):
                        os.killpg(self.process.pid, signal.SIGKILL)
                    with suppress(subprocess.TimeoutExpired):
                        self.process.wait(timeout=5)
            self.process = None


class RobotPreviewMCP:
    def __init__(self):
        root = Path(
            os.environ.get("ROBODOJO_PROJECT_ROOT", Path(__file__).resolve().parents[2])
        ).resolve()
        runtime = Path(
            os.environ.get("ROBODOJO_RUNTIME_ROOT", root / "runtime")
        ).resolve()
        python = os.environ.get(
            "ROBODOJO_PYTHON", str(runtime / "envs" / "robodojo" / "bin" / "python")
        )
        workspace = Path(
            os.environ.get("ROBODOJO_AGENT_WORKSPACE", runtime / "agent-workspace")
        ).resolve()
        self.frame_root = (workspace / "runtime").resolve() / "frames"
        self.native = PreviewProcess(
            root, python, runtime / "results", 0, 900.0, os.environ.get("ROBODOJO_TRIAL_GPU")
        )

    @staticmethod
    def tools():
        vector14 = {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 14,
            "maxItems": 14,
            "description": "[left joints 1..6, left gripper, right joints 1..6, right gripper]; joints are radians and grippers use 0 closed/1 open.",
        }
        from services.robodojo.trajectory import MAX_TRAJECTORY_ACTIONS

        run_properties = {
            "robot_model": {"type": "string", "enum": ["dual_arx_x5"]},
            "camera_model": {"type": "string", "enum": ["robodojo_gemini_wrist_v1"]},
            "initial_state": vector14,
            "actions": {
                "type": "array", "items": vector14, "minItems": 1, "maxItems": MAX_TRAJECTORY_ACTIONS,
                "description": "Complete 25Hz absolute joint targets, one row per step. Use motion_plan.trajectory_preview.actions or transition.steps[*].executed_action.",
            },
        }
        return [
            {
                "name": "robot_preview_describe",
                "description": "Describe the separate object-free robot-model preview contract without starting Isaac Sim.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "robot_preview_run",
                "description": "Render 25Hz joint targets in a separate robot-only scene using native RoboDojo interpolation. Supply initial_state and actions; there are no EEF, dense-control, or input_type fields. No task objects, rewards, or privileged object state.",
                "inputSchema": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": [
                        "robot_model",
                        "camera_model",
                        "initial_state",
                        "actions",
                    ],
                    "properties": run_properties,
                },
            },
        ]

    @staticmethod
    def description(observation_profile="rgbd"):
        run_schema = next(
            tool["inputSchema"]
            for tool in RobotPreviewMCP.tools()
            if tool["name"] == "robot_preview_run"
        )
        return {
            "service": "robot_model_preview",
            "robot_models": ["dual_arx_x5"],
            "camera_models": ["robodojo_gemini_wrist_v1"],
            "inputs": [
                "model IDs",
                "initial 14-D robot state",
                "25Hz absolute 14D joint targets",
            ],
            "robot_preview_run_input_schema": run_schema,
            "required_top_level_fields": run_schema["required"],
            "outputs": [
                *(["robot-only RGB/depth", "camera calibration"] if observation_profile == "rgbd" else ["robot-only RGB"]),
                "measured joint/EEF state",
                "final 25 Hz frame attached; complete sequence saved in the agent workspace",
            ],
            "not_available": [
                "task scene",
                "task objects or poses",
                "reward/success",
                "object contacts or occlusion prediction",
            ],
            "native_service_started": False,
        }

    def _encode_observation(self, observation, content):
        observation = observation_for(self).clean(observation)
        images, depth, attachments = [], {}, []
        for camera in CAMERAS:
            if camera not in observation:
                continue
            index = len(content) + 1
            content.append(
                {
                    "type": "image",
                    "data": _rgb_png(observation[camera]),
                    "mimeType": "image/png",
                }
            )
            images.append(camera)
            attachments.append(
                {"kind": "rgb", "camera": camera, "content_index": index}
            )
        for camera in CAMERAS:
            key = f"{camera}_depth_m"
            if key not in observation:
                continue
            data, stats = _depth_png(observation[key])
            index = len(content) + 1
            content.append({"type": "image", "data": data, "mimeType": "image/png"})
            depth[camera] = dict(stats, content_index=index)
            attachments.append(
                {"kind": "depth", "camera": camera, "content_index": index}
            )
        return {
            "states": _jsonable(observation["states"]),
            "eef_positions": _jsonable(observation["eef_positions"]),
            "eef_quaternions_wxyz": _jsonable(observation["eef_quaternions_wxyz"]),
            "images": images,
            "depth": depth,
            "camera_parameters": _jsonable(observation.get("camera_parameters", {})),
            "attachments": attachments,
        }

    def _save_sequence(self, initial, frames, contents, *, frequency, execution):
        sequence_id = f"seq_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{os.urandom(4).hex()}"
        logical_dir = Path("runtime") / "frames" / "robot_preview" / sequence_id
        output_dir = self.frame_root / "robot_preview" / sequence_id
        artifacts: dict[str, bytes] = {}

        records = [
            ("initial", initial, contents[0]),
            *[
                (f"frame_{int(frame['frame_index']):06d}", frame, content)
                for frame, content in zip(frames, contents[1:], strict=True)
            ],
        ]
        saved = []
        for stem, record, content in records:
            files = []
            for attachment, item in zip(record["attachments"], content, strict=True):
                filename = f"{stem}_{attachment['camera']}_{attachment['kind']}.png"
                artifacts[filename] = base64.b64decode(item["data"])
                files.append(
                    {
                        "kind": attachment["kind"],
                        "camera": attachment["camera"],
                        "path": (logical_dir / filename).as_posix(),
                    }
                )
            item = {
                key: value
                for key, value in record.items()
                if key not in {"attachments", "depth"}
            }
            item["depth"] = {
                camera: {
                    key: value
                    for key, value in values.items()
                    if key != "content_index"
                }
                for camera, values in record["depth"].items()
            }
            item["files"] = files
            saved.append(item)

        manifest = {
            "sequence_id": sequence_id,
            "source": "robot_preview",
            "frequency_hz": frequency,
            "frame_count": len(frames),
            "execution": _jsonable(execution),
            "initial_observation": saved[0],
            "frames": saved[1:],
        }
        artifacts["manifest.json"] = (
            json.dumps(observation_for(self).clean(_jsonable(manifest)), ensure_ascii=False, indent=2) + "\n"
        ).encode("utf-8")
        write_artifacts(output_dir, artifacts)
        return sequence_id, (logical_dir / "manifest.json").as_posix()

    def preview(self, arguments):
        if not isinstance(arguments, dict):
            raise TypeError("robot_preview_run arguments must be an object")
        required = {"robot_model", "camera_model", "initial_state", "actions"}
        missing = required - set(arguments)
        if missing:
            raise ValueError(f"Missing required fields: {sorted(missing)}")
        unknown = set(arguments) - required
        if unknown:
            raise ValueError(f"Unknown robot_preview_run fields: {sorted(unknown)}")
        from services.robot_preview.session import validate_preview_inputs

        validate_preview_inputs(**arguments)
        result = self.native.request(**arguments)
        initial_content = []
        initial = self._encode_observation(
            result["initial_observation"], initial_content
        )
        execution = dict(result["execution"])
        raw_frames = execution.pop("frames")
        frequency = execution["observation_frequency_hz"]
        frames = []
        frame_contents = []
        for index, observation in enumerate(raw_frames):
            local_content = []
            frame = self._encode_observation(observation, local_content)
            frame.update(
                frame_index=index,
                time_from_execution_start_s=(index + 1) / frequency,
            )
            frames.append(frame)
            frame_contents.append(local_content)
        sequence_id, manifest_path = self._save_sequence(
            initial,
            frames,
            [initial_content, *frame_contents],
            frequency=frequency,
            execution=execution,
        )
        sequence_dir = Path(manifest_path).parent.as_posix()
        final = frames[-1] if frames else initial
        final_content = frame_contents[-1] if frame_contents else initial_content
        packet = {
            "status": result["status"],
            "final_observation": final,
            "execution": _jsonable(execution),
            "frame_sequence": {
                "sequence_id": sequence_id,
                "frequency_hz": frequency,
                "frame_count": len(frames),
                "manifest_path": manifest_path,
                "initial_frame_path_pattern": (
                    f"{sequence_dir}/initial_{{camera}}_{{kind}}.png"
                ),
                "frame_path_pattern": (
                    f"{sequence_dir}/frame_{{frame_index:06d}}_{{camera}}_{{kind}}.png"
                ),
                "frame_index_range": ([0, len(frames) - 1] if frames else None),
                "streams": [
                    {
                        "camera": item["camera"],
                        "kind": item["kind"],
                    }
                    for item in final["attachments"]
                ],
                "initial_frame_saved": True,
                "returned_images": "final_frame_only",
                "inspection": (
                    "If the final frame is insufficient, choose an index in "
                    "frame_index_range, substitute frame_path_pattern, and use the "
                    "image viewer. No script or interpreter is needed."
                ),
            },
            "contract": result["contract"],
            "metadata": _jsonable(self.native.metadata),
        }
        return observation_for(self).clean(packet), final_content

    def close(self):
        self.native.close()

