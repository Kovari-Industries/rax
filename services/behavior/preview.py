"""Host-side owner of the BEHAVIOR robot-only preview process (stdlib + msgpack + numpy only).

One persistent ``python -m services.behavior.preview_server`` per backend, started on the
first preview run and reused until the backend closes. It holds an R1Pro on a floor
plane with fixed third-person cameras; it never sees a live episode.
"""

from __future__ import annotations

import subprocess
import threading
import time
from contextlib import suppress
from pathlib import Path

from services.behavior.process import AppdataSlot, BehaviorProcess, runtime_root, simulator_environment
from services.simulator.io import exit_report, log_size

PREVIEW_STARTUP_SECONDS = 600


class PreviewProcess(BehaviorProcess):
    def __init__(self, *, output_root: Path, sim_gpu, python: str | None = None):
        super().__init__(output_root=output_root, python=python)
        self.sim_gpu = sim_gpu
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
        self.slot = AppdataSlot(runtime_root() / "behavior-cache" / "appdata")
        env = simulator_environment(self.sim_gpu, self.slot.acquire())
        Path(env["XDG_RUNTIME_DIR"]).mkdir(mode=0o700, parents=True, exist_ok=True)
        log_handle = self.log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen([self.python, "-m", "services.behavior.preview_server",
                                             "--output", str(self.output_root.resolve()), "--port", "0"],
                                            cwd=self.output_root, env=env, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
        except Exception:
            log_handle.close()
            self.slot.release()
            raise
        self.ready.clear()
        threading.Thread(target=self._read_log, args=(self.process.stdout, log_handle),
                         name="behavior-preview-log", daemon=True).start()
        deadline = time.monotonic() + startup_timeout
        try:
            while not self.ready.wait(timeout=min(1.0, max(0.0, deadline - time.monotonic()))):
                if self.process.poll() is not None:
                    time.sleep(1)  # Let the log reader drain the pipe.
                    raise RuntimeError(f"BEHAVIOR preview simulator exited during startup: "
                                       f"{exit_report(self.process, self.log_path)}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"BEHAVIOR preview simulator did not start within {startup_timeout:g}s; "
                                       + exit_report(self.process, self.log_path))
            from services.simulator.protocol import RPCClient
            self.rpc = RPCClient("127.0.0.1", self.bound_port, timeout=startup_timeout)
            try:
                self.metadata = self.rpc.request("metadata")
            except (ConnectionError, EOFError, OSError) as exc:
                with suppress(Exception):
                    self.process.wait(timeout=5)
                raise RuntimeError(f"BEHAVIOR preview simulator failed during startup with {type(exc).__name__}: "
                                   f"{exc}; {exit_report(self.process, self.log_path)}") from exc
            self.finished = False
            return self.metadata
        except Exception:
            self.stop()
            raise

    def run(self, initial_state, actions, measured=False):
        with self.lock:
            self.ensure()
            since = log_size(self.log_path)
            try:
                return self.rpc.request("run", initial_state=initial_state, actions=actions, measured=measured)
            except (ConnectionError, EOFError, OSError) as exc:
                with suppress(Exception):
                    self.process.wait(timeout=5)
                report = exit_report(self.process, self.log_path, since=since)
                self.stop()  # A dead preview restarts on the next call.
                raise RuntimeError(f"Preview simulator failed with {type(exc).__name__}: {exc}; {report}; "
                                   "it restarts on the next preview call") from exc
