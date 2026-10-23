"""Python wrapper for Evo elevator services and bounded drive pulses.

Evo owns perception, model queries, and actuator execution. This adapter uses
the existing Evo session for every actuator command. An expired pulse cannot be extended by a delayed call.
"""
import threading
import time
import uuid


class ElevatorRuntime:
    def __init__(self, backend):
        self.backend = backend
        self._lock = threading.Lock()
        self._active = None
        self._results = {}
        self._thread = None

    def rpc(self, method, params, timeout=0.5):
        return self.backend._rpc("elevator", {"operation": method, "params": params}, timeout=timeout)

    @property
    def active(self):
        with self._lock:
            return self._active is not None

    def start(self, arguments):
        with self._lock:
            if self._active is not None:
                raise RuntimeError("RAX base goal is already active")
            goal = self.rpc("begin", arguments)["goal_id"]
            ident = "rax-base-" + uuid.uuid4().hex
            cancelled = threading.Event()
            self._active = (ident, cancelled)
            self._results[ident] = {"action_id": ident, "state": "accepted"}
            self._thread = threading.Thread(target=self._run, args=(ident, goal, cancelled), daemon=True)
            self._thread.start()
            return dict(self._results[ident])

    def result(self, ident):
        with self._lock:
            return self._result_unlocked(ident)

    def cancel(self, ident=None):
        with self._lock:
            if self._active is None:
                if ident is not None:
                    return self._result_unlocked(ident)
                raise ValueError("no active RAX base action")
            active, event = self._active
            if ident is not None and ident != active:
                raise ValueError("RAX base action is not active")
            event.set()  # Set before I/O, including while a pulse request is pending.
            self._results[active] = {"action_id": active, "state": "stopping"}
        self.backend._rpc("hold", {}, timeout=0.2)
        return self.result(active)

    def _result_unlocked(self, ident):
        if ident not in self._results:
            raise ValueError("unknown RAX base action")
        return dict(self._results[ident])

    def _wait_pulse(self, action_id, cancelled):
        deadline = time.monotonic() + 3
        stop_sent = False
        while time.monotonic() < deadline:
            if cancelled.is_set() and not stop_sent:
                self.backend._rpc("hold", {}, timeout=0.2)
                stop_sent = True
            result = self.backend._rpc("action_status", {"action_id": action_id}, timeout=0.2)
            if result.get("state") in ("succeeded", "failed", "cancelled"):
                return result
            time.sleep(0.02)
        raise TimeoutError("Evo pulse stop verification timed out")

    def _run(self, ident, goal, cancelled):
        stopped, pulses = False, []
        try:
            after_us = 0
            with self._lock:
                if not cancelled.is_set():
                    self._results[ident] = {"action_id": ident, "state": "running"}
            while True:
                if cancelled.is_set():
                    raise RuntimeError("caller cancelled")
                command = self.rpc("step", {"goal_id": goal, "after_us": after_us})
                if command.get("waiting"):
                    cancelled.wait(0.02)
                    continue
                if command.get("done"):
                    with self._lock:
                        if cancelled.is_set():
                            raise RuntimeError("caller cancelled")
                        self._results[ident] = {"action_id": ident, "state": "succeeded", "physical_stop_verified": True,
                                               "base_result": command["base_result"], "pulse_actions": pulses}
                        self._active = None
                        self._trim_results()
                    return
                if cancelled.is_set():
                    raise RuntimeError("caller cancelled")
                # Do not preview and then refresh this deadline. It belongs to the
                # measured scene that the Evo planner checked for this exact pulse.
                if set(command) != {"velocity", "duration_s", "valid_until_us"}:
                    raise RuntimeError("invalid drive pulse")
                stopped = False
                accepted = self.backend._rpc("drive", command, timeout=0.5)
                if accepted.get("state") != "accepted" or not isinstance(accepted.get("action_id"), str):
                    raise RuntimeError("invalid Evo pulse acceptance")
                pulses.append(accepted["action_id"])
                pulse = self._wait_pulse(accepted["action_id"], cancelled)
                stopped = pulse.get("physical_stop_verified") is True
                if pulse.get("state") != "succeeded" or not stopped:
                    raise RuntimeError("Evo pulse failed: " + str(pulse.get("reason", pulse.get("state"))))
                after_us = pulse.get("final_state", {}).get("source_us", 0)
                if type(after_us) is not int or after_us <= 0:
                    raise RuntimeError("pulse lacks measured stop timestamp")
        except Exception as error:
            try:
                # Reconcile any in-flight dispatch before publishing a terminal
                # result. The absolute Evo deadline also bounds a lost response.
                hold = self.backend._rpc("hold", {}, timeout=0.2)
                stop = self._wait_pulse(hold["action_id"], threading.Event())
                stopped = stop.get("physical_stop_verified") is True
            except Exception:
                stopped = False
            if not stopped:
                self.backend._failure = "RAX base stop is unconfirmed"
                self.backend._stop.set()  # Evo's independent lease watchdog stops.
            result = {"action_id": ident, "state": "cancelled" if cancelled.is_set() and stopped else "failed",
                      "reason": str(error), "physical_stop_verified": stopped, "pulse_actions": pulses}
        finally:
            try:
                self.rpc("abort", {"goal_id": goal})
            except Exception:
                pass  # The worker can already have completed the goal; no motion RPC.
        with self._lock:
            self._results[ident] = result
            self._active = None
            self._trim_results()

    def _trim_results(self):
        while len(self._results) > 256:
            self._results.pop(next(iter(self._results)))

    def close(self):
        if self.active:
            try:
                self.cancel()
            except Exception:
                self.backend._stop.set()
            if self._thread:
                self._thread.join(timeout=4)
