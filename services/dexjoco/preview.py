"""Host-side owner of the DexJoCo robot-only preview process (stdlib + msgpack + numpy only).

One persistent ``python -m services.dexjoco.preview_server`` per backend, started on the first
preview run on the simulator GPU and reused until the backend closes. It holds only the task's
official robot on a floor plane; it never sees a live episode.
"""

from __future__ import annotations

import subprocess
import threading
import time
from contextlib import suppress
from pathlib import Path

from services.dexjoco.process import DexjocoProcess, simulator_environment
from services.simulator.io import exit_report, log_size

PREVIEW_STARTUP_SECONDS = 300


class PreviewProcess(DexjocoProcess):
    def __init__(self, *, output_root: Path, sim_gpu, task: str, python: str | None = None):
        super().__init__(output_root=output_root, python=python)
        self.sim_gpu, self.preview_task = sim_gpu, task
        self.lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self.process is not None and self.rpc is not None and self.process.poll() is None

    def ensure(self, startup_timeout: float = PREVIEW_STARTUP_SECONDS) -> dict:
        if self.active:
            return self.metadata
        self.stop()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.log_path = self.output_root / f"preview_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.log"
        env = simulator_environment(self.sim_gpu)
        Path(env["XDG_RUNTIME_DIR"]).mkdir(mode=0o700, parents=True, exist_ok=True)
        log_handle = self.log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen([self.python, "-m", "services.dexjoco.preview_server",
                                             "--task", self.preview_task, "--port", "0",
                                             "--output", str(self.output_root.resolve())],
                                            cwd=self.output_root, env=env, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
        except Exception:
            log_handle.close()
            raise
        self.ready.clear()
        threading.Thread(target=self._read_log, args=(self.process.stdout, log_handle),
                         name="dexjoco-preview-log", daemon=True).start()
        deadline = time.monotonic() + startup_timeout
        try:
            while not self.ready.wait(timeout=min(1.0, max(0.0, deadline - time.monotonic()))):
                if self.process.poll() is not None:
                    time.sleep(1)  # Let the log reader drain the pipe.
                    raise RuntimeError(f"DexJoCo preview simulator exited during startup: "
                                       f"{exit_report(self.process, self.log_path)}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"DexJoCo preview simulator did not start within {startup_timeout:g}s")
            from services.simulator.protocol import RPCClient
            self.rpc = RPCClient("127.0.0.1", self.bound_port, timeout=startup_timeout)
            self.metadata = self.rpc.request("metadata")
            self.finished = False
            return self.metadata
        except Exception:
            self.stop()
            raise

    def run(self, initial_state, actions):
        with self.lock:
            self.ensure()
            since = log_size(self.log_path)
            try:
                return self.rpc.request("run", initial_state=initial_state, actions=actions)
            except (ConnectionError, EOFError, OSError) as exc:
                with suppress(Exception):
                    self.process.wait(timeout=5)
                report = exit_report(self.process, self.log_path, since=since)
                self.stop()  # A dead preview restarts on the next call.
                raise RuntimeError(f"Preview simulator failed with {type(exc).__name__}: {exc}; {report}; "
                                   "it restarts on the next preview call") from exc
