"""Task trial journal and strict offline replay. No robot connection in replay."""
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def normalized(value):
    return json.loads(json.dumps(value, allow_nan=False))


class TrialContext:
    """Wrap Context.value. Each request is durable before dispatch; errors latch closed.

    Callers retain ownership of the underlying context. Use a new directory per trial.
    This log supplements the Evo recorder; it does not replace it.
    """

    def __init__(self, context, directory, *, workspace="."):
        self.context = context
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.workspace = Path(workspace).resolve()
        self.stream = (self.directory / "events.jsonl").open("x")
        self.lock = threading.Lock()
        self.failed = False
        self.sequence = 0

    def _write(self, event):
        try:
            self.stream.write(
                json.dumps({"time_ns": time.time_ns(), **event}, allow_nan=False) + "\n"
            )
            self.stream.flush()
            os.fsync(self.stream.fileno())
        except BaseException:
            self.failed = True
            raise

    def value(self, tool, **arguments):
        with self.lock:
            if self.failed:
                raise RuntimeError("trial is closed after an error")
            self.sequence += 1
            self._write(
                {
                    "kind": "request",
                    "id": self.sequence,
                    "tool": tool,
                    "arguments": arguments,
                }
            )
            try:
                result = self.context.value(tool, **arguments)
                saved = copy.deepcopy(result)
                if tool == "evo_observe":
                    for index, camera in enumerate(saved["cameras"].values()):
                        source = (self.workspace / camera["path"]).resolve()
                        if not source.is_relative_to(self.workspace):
                            raise ValueError("capture is outside workspace")
                        data = source.read_bytes()
                        name = f"capture-{self.sequence}-{index}.jpg"
                        with (self.directory / name).open("xb") as image:
                            image.write(data)
                            image.flush()
                            os.fsync(image.fileno())
                        camera["path"] = name
                        camera["trial_sha256"] = hashlib.sha256(data).hexdigest()
                self._write({"kind": "result", "id": self.sequence, "value": saved})
                return result
            except BaseException as exc:
                self.failed = True
                self._write({"kind": "error", "id": self.sequence, "error": str(exc)})
                raise

    def annotate(self, outcome, note=""):
        with self.lock:
            self._write({"kind": "annotation", "outcome": outcome, "note": note})

    def close(self):
        self.failed = True
        self.stream.close()


class ReplayContext:
    """Exact recorded call replay. Never opens a socket or falls back to live tools."""

    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        events = [
            json.loads(line)
            for line in (self.directory / "events.jsonl").read_text().splitlines()
        ]
        self.annotations = [e for e in events if e["kind"] == "annotation"]
        events = [e for e in events if e["kind"] != "annotation"]
        if len(events) % 2:
            raise ValueError("incomplete trial")
        self.calls = []
        for request, result in zip(events[::2], events[1::2]):
            if (
                request["kind"] != "request"
                or result["kind"] not in {"result", "error"}
                or request["id"] != result["id"]
            ):
                raise ValueError("invalid trial ordering")
            self.calls.append((request, result))
        self.index = 0

    def value(self, tool, **arguments):
        if self.index == len(self.calls):
            raise ValueError("replay exhausted")
        request, result = self.calls[self.index]
        if request["tool"] != tool or request["arguments"] != normalized(arguments):
            raise ValueError("replay call differs from recorded request")
        self.index += 1
        if result["kind"] == "error":
            raise RuntimeError("recorded error: " + result["error"])
        value = copy.deepcopy(result["value"])
        if tool == "evo_observe":
            for camera in value["cameras"].values():
                path = (self.directory / camera["path"]).resolve()
                if not path.is_relative_to(self.directory) or hashlib.sha256(
                    path.read_bytes()
                ).hexdigest() != camera.pop("trial_sha256"):
                    raise ValueError("recorded image is missing or changed")
                camera["path"] = str(path)
        return value
