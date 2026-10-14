"""Host-side owner of one BEHAVIOR-1K simulator subprocess (stdlib + msgpack + numpy only).

The subprocess runs ``python -m services.behavior.server`` in ``runtime/envs/behavior``;
this module never imports OmniGibson. Each process leases a private OmniGibson appdata
slot (kit caches, shader and texture caches, warp kernels): concurrent OmniGibson
processes must not share one, while sequential reuse keeps caches warm.
"""

from __future__ import annotations

import fcntl
import json
import os
import re
import signal
import subprocess
import threading
import time
from contextlib import suppress
from pathlib import Path
from typing import Any

from services.behavior.tasks import task_info
from services.simulator.io import exit_report, log_size, simulator_request

RUN_ID = re.compile(r"^[A-Za-z0-9_.-]+$")
# One per live simulator and per preview, machine-wide: 4-env pool sessions take 8 each.
APPDATA_SLOTS = 32
# Scene loads take 150-300 s on an RTX 4090 per upstream; first launch also builds caches.
DEFAULT_STARTUP_SECONDS = 900


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def runtime_root() -> Path:
    base = os.environ.get("BEHAVIOR_RUNTIME_ROOT") or os.environ.get("ROBODOJO_RUNTIME_ROOT")
    return Path(base or project_root() / "runtime").resolve()


def behavior_python() -> str:
    return os.environ.get("BEHAVIOR_PYTHON") or str(runtime_root() / "envs" / "behavior" / "bin" / "python")


def data_path() -> Path:
    return Path(os.environ.get("OMNIGIBSON_DATA_PATH") or runtime_root() / "behavior-data")


class AppdataSlot:
    """Exclusive lease of one appdata directory, held by an open flock."""

    def __init__(self, root: Path, slots: int = APPDATA_SLOTS):
        self.root, self.slots = root, slots
        self.path: Path | None = None
        self._handle = None

    def acquire(self) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        # Highest first: processes started under the old 16-slot cap scan only the lowest 16.
        for index in reversed(range(self.slots)):
            path = self.root / f"slot-{index:02d}"
            path.mkdir(exist_ok=True)
            handle = (self.root / f"slot-{index:02d}.lock").open("a")
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                continue
            self.path, self._handle = path, handle
            return path
        raise RuntimeError(f"All {self.slots} BEHAVIOR appdata slots in {self.root} are in use")

    def release(self) -> None:
        if self._handle is not None:
            with suppress(OSError):
                fcntl.flock(self._handle, fcntl.LOCK_UN)
            self._handle.close()
        self._handle = self.path = None


def physical_gpu_index(sim_gpu) -> str:
    """GPU ordinal or UUID -> nvidia-smi (PCI-ordered) index, which Kit's renderer uses."""
    selector = str(sim_gpu)
    rows = subprocess.check_output(["nvidia-smi", "--query-gpu=index,uuid", "--format=csv,noheader"],
                                   text=True)
    for row in rows.splitlines():
        index, uuid = (part.strip() for part in row.split(","))
        if selector in (index, uuid):
            return index
    raise ValueError(f"Unknown GPU {selector!r}")


def simulator_environment(sim_gpu, appdata: Path, base=None) -> dict[str, str]:
    """Environment for the simulator subprocess, mirroring scripts/behavior_env.sh.

    CUDA sees only the simulator GPU (physics device 0), but Vulkan still enumerates every
    GPU, so the renderer needs that GPU's physical index (ROBODOJO_RENDER_GPU)."""
    env = dict(os.environ if base is None else base)
    env_root = Path(behavior_python()).resolve().parents[1]
    runtime = runtime_root()
    for name in ("EXP_PATH", "CARB_APP_PATH", "ISAAC_PATH", "PYTHONHOME", "CONDA_PREFIX",
                 "ROBODOJO_AGENT_TRIAL_ID"):
        env.pop(name, None)
    env.update({
        "CUDA_VISIBLE_DEVICES": str(sim_gpu),
        "OMNIGIBSON_GPU_ID": "0",
        "ROBODOJO_RENDER_GPU": physical_gpu_index(sim_gpu),
        "OMNIGIBSON_HEADLESS": "1",
        "OMNIGIBSON_NO_OMNI_LOGS": "1",
        "OMNIGIBSON_DATA_PATH": str(data_path()),
        "OMNIGIBSON_APPDATA_PATH": str(appdata),
        "OMNI_KIT_ACCEPT_EULA": "YES",
        "CUDA_HOME": str(env_root),
        "PATH": f"{env_root / 'bin'}:{env.get('PATH', '/usr/bin:/bin')}",
        "PYTHONPATH": str(project_root()),
        "PYTHONUNBUFFERED": "1",
        "XDG_RUNTIME_DIR": str(runtime / "xdg-runtime"),
    })
    if Path("/usr/share/vulkan/icd.d/nvidia_icd.json").is_file():
        env.setdefault("VK_ICD_FILENAMES", "/usr/share/vulkan/icd.d/nvidia_icd.json")
    if Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json").is_file():
        env.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    return env


class BehaviorProcess:
    """One simulator process, its RPC client and its identity-checked requests."""

    def __init__(self, *, output_root: Path, python: str | None = None):
        self.output_root = Path(output_root)
        self.python = python or behavior_python()
        self.process: subprocess.Popen[str] | None = None
        self.rpc = None
        self.ready = threading.Event()
        self.bound_port: int | None = None
        self.log_path: Path | None = None
        self.output_dir: Path | None = None
        self.run_id: str | None = None
        self.task: str | None = None
        self.episode_id: str | None = None
        self.step_id = 0
        self.metadata: dict[str, Any] | None = None
        self.finished = False
        self.log_mark = 0
        self.slot: AppdataSlot | None = None
        self.startup_seconds: float | None = None

    @property
    def active(self) -> bool:
        return self.process is not None and self.rpc is not None and not self.finished

    def _read_log(self, stream, log_file) -> None:
        try:
            for line in iter(stream.readline, ""):
                log_file.write(line)
                log_file.flush()
                if '"event": "ready"' not in line:
                    continue
                with suppress(json.JSONDecodeError):
                    packet = json.loads(line)
                    if packet.get("event") == "ready":
                        self.bound_port = int(packet["port"])
                        self.ready.set()
        finally:
            with suppress(OSError):
                stream.close()
            with suppress(OSError):
                log_file.close()

    def start(self, *, task: str, collection: int, instance_id: int, run_id: str, sim_gpu,
              startup_timeout: float = DEFAULT_STARTUP_SECONDS, include_depth: bool = True,
              episode_timeout_seconds: int = 0) -> dict[str, Any]:
        if self.active:
            raise RuntimeError("A BEHAVIOR episode is already active; finish it before starting another")
        info = task_info(task)
        if collection not in (0, 1):
            raise ValueError("collection must be 0 (train instances) or 1 (held-out public test)")
        if not RUN_ID.fullmatch(run_id):
            raise ValueError("run_id must contain only letters, numbers, dots, underscores and hyphens")
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.output_dir = (self.output_root / run_id).resolve()
        if self.output_dir.exists():
            raise ValueError(f"Artifact identifier already exists: {run_id}")
        self.output_dir.mkdir(parents=True)
        (self.output_dir / "sim").mkdir()
        (self.output_dir / "native_runtime").mkdir()
        self.run_id, self.log_path = run_id, self.output_dir / "sim.log"
        self.slot = AppdataSlot(runtime_root() / "behavior-cache" / "appdata")
        appdata = self.slot.acquire()
        env = simulator_environment(sim_gpu, appdata)
        Path(env["XDG_RUNTIME_DIR"]).mkdir(mode=0o700, parents=True, exist_ok=True)
        command = [self.python, "-m", "services.behavior.server", "--task", task,
                   "--output", str(self.output_dir / "sim"), "--port", "0",
                   "--collection", str(int(collection)),
                   "--episode-timeout-seconds", str(int(episode_timeout_seconds)),
                   "--depth" if include_depth else "--no-depth"]
        if info.get("instruction"):
            command += ["--instruction", info["instruction"]]
        started = time.monotonic()
        log_handle = self.log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(command, cwd=self.output_dir / "native_runtime", env=env,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, bufsize=1, start_new_session=True)
        except Exception:
            log_handle.close()
            self.slot.release()
            raise
        self.ready.clear()
        threading.Thread(target=self._read_log, args=(self.process.stdout, log_handle),
                         name="behavior-sim-log", daemon=True).start()
        deadline = started + startup_timeout
        try:
            while not self.ready.wait(timeout=min(1.0, max(0.0, deadline - time.monotonic()))):
                if self.process.poll() is not None:
                    time.sleep(1)  # Let the log reader drain the pipe.
                    raise RuntimeError(f"BEHAVIOR simulator exited during startup: {exit_report(self.process, self.log_path)}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"BEHAVIOR simulator did not become ready within {startup_timeout:g}s; "
                                       + exit_report(self.process, self.log_path))
            from services.simulator.protocol import RPCClient
            self.rpc = RPCClient("127.0.0.1", self.bound_port, timeout=max(900.0, startup_timeout))
            # A simulator that dies while loading the scene reports its exit code and the
            # errors it logged, not only the closed connection.
            self.episode_id, self.step_id, self.finished, self.log_mark = None, 0, False, 0
            self.metadata = simulator_request(self, "metadata")
            if self.metadata.get("task") != task:
                raise RuntimeError(f"Simulator task mismatch: expected {task!r}, got {self.metadata.get('task')!r}")
            reset = simulator_request(self, "reset", instance_id=int(instance_id), source="agent",
                                      policy_version="behavior-mcp")
            self.episode_id, self.step_id = reset["episode_id"], int(reset["step_id"])
            self.task, self.finished = task, False
            self.log_mark = log_size(self.log_path)  # Startup output is not an episode error.
            self.startup_seconds = time.monotonic() - started
            return reset
        except Exception:
            self.stop()
            raise

    def request(self, op: str, **kwargs: Any) -> Any:
        if self.rpc is None or self.episode_id is None or self.finished:
            raise RuntimeError("No active BEHAVIOR episode")
        result = simulator_request(self, op, episode_id=self.episode_id, step_id=self.step_id, **kwargs)
        if isinstance(result, dict) and result.get("step_id") is not None and result.get("episode_id") == self.episode_id:
            self.step_id = int(result["step_id"])
        return result

    def finish(self, reason: str) -> Any:
        if self.rpc is None or self.episode_id is None:
            raise RuntimeError("No active BEHAVIOR episode")
        if self.finished:
            return {"episode_id": self.episode_id, "step_id": self.step_id, "reason": reason}
        # A simulator that died since the last call reports its exit code and log.
        result = simulator_request(self, "finish_pilot", episode_id=self.episode_id, step_id=self.step_id,
                                   reason=reason)
        self.finished = True
        return result

    def stop(self) -> None:
        rpc, process = self.rpc, self.process
        self.rpc = self.process = None
        self.finished = True
        if rpc is not None:
            with suppress(Exception):
                rpc.close()
        if process is not None:
            try:
                # og.shutdown() can take a while; the scene teardown is not interruptible.
                process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
                except (OSError, subprocess.TimeoutExpired):
                    with suppress(OSError):
                        os.killpg(process.pid, signal.SIGKILL)
            except OSError:
                pass
        if self.slot is not None:
            self.slot.release()
            self.slot = None
