"""Thin RAX client for Evo's exclusive agent backend.

Evo owns arming checks, motion preparation, limits, execution, and measured stops.
The client never opens a sensor, controller, ROS publisher, or hardware SDK.
"""
import base64
import copy
import json
import math
import os
from pathlib import Path
import socket
import threading
import uuid

TOOLS = json.loads(Path(__file__).with_name("tools.json").read_text())
BY_NAME = {tool["name"]: tool for tool in TOOLS}
MOTION = frozenset(
    "evo_" + name
    for name in (
        "move_joints",
        "move_eef",
        "gripper",
        "move_carriage",
        "move_neck",
        "drive",
        "move_base",
        "move_body",
        "follow_path",
        "move_pair",
        "set_grip",
    )
)
DEFAULTS = {
    "control_socket": str(Path.home() / ".local/evo/agent/agent.sock"),
    "socket_timeout_s": 5.0,
}
MAX_RESPONSE = 40 * 1024 * 1024


def validate_value(value, schema):
    """Validate the restricted, checked-in tool schema without a runtime dependency."""
    if "oneOf" in schema:
        matches = 0
        for choice in schema["oneOf"]:
            try:
                validate_value(value, choice)
                matches += 1
            except ValueError:
                pass
        if matches != 1:
            raise ValueError("arguments must match exactly one action schema")
        return
    if "const" in schema and value != schema["const"]:
        raise ValueError("invalid action kind")
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError("value is not an allowed choice")
    kind = schema.get("type")
    if kind == "object":
        if not isinstance(value, dict):
            raise ValueError("expected an object")
        properties = schema.get("properties", {})
        if set(schema.get("required", ())) - value.keys():
            raise ValueError("missing required arguments")
        extra = schema.get("additionalProperties", True)
        if extra is False and value.keys() - properties.keys():
            raise ValueError("unknown arguments")
        if (
            not schema.get("minProperties", 0)
            <= len(value)
            <= schema.get("maxProperties", 128)
        ):
            raise ValueError("invalid object size")
        for key, item in value.items():
            if not isinstance(key, str) or len(key) > 128:
                raise ValueError("invalid field name")
            validate_value(
                item, properties.get(key, extra if isinstance(extra, dict) else {})
            )
    elif kind == "array":
        if not isinstance(value, (list, tuple)) or not schema.get("minItems", 0) <= len(
            value
        ) <= schema.get("maxItems", 128):
            raise ValueError("invalid array size")
        for item in value:
            validate_value(item, schema["items"])
    elif kind == "number":
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError("expected a finite number")
        if "minimum" in schema and value < schema["minimum"]:
            raise ValueError("number is below minimum")
        if "maximum" in schema and value > schema["maximum"]:
            raise ValueError("number exceeds maximum")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            raise ValueError("number is below minimum")
    elif kind == "boolean":
        if not isinstance(value, bool):
            raise ValueError("expected a boolean")
    elif kind == "string":
        if not isinstance(value, str) or not value or len(value) > 1024:
            raise ValueError("expected a nonempty bounded string")


class EvoRefused(ValueError):
    """A validated server response confirms that the request was refused."""


class EvoV3Backend:
    stop_tools = frozenset({"evo_cancel", "evo_hold"})
    mutating = MOTION
    priority_tools = frozenset(
        {"evo_cancel", "evo_hold", "evo_action_status", "evo_status"}
    )

    def __init__(self, config):
        unknown = set(config.backend_options) - DEFAULTS.keys()
        if unknown:
            raise ValueError(f"unknown Evo options: {sorted(unknown)}")
        self.options = {**DEFAULTS, **config.backend_options}
        path = self.options["control_socket"]
        timeout = self.options["socket_timeout_s"]
        if not isinstance(path, str) or not Path(path).is_absolute():
            raise ValueError("control_socket must be an absolute operator path")
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or not 0 < timeout <= 30
        ):
            raise ValueError("invalid socket_timeout_s")
        self.config = config
        self.session_id = None
        self.closed = False
        self._session_lock = threading.Lock()
        self._stop = threading.Event()
        self._heartbeat_thread = None
        self._failure = None

    def tools(self):
        return copy.deepcopy(TOOLS)

    def validate(self, name, arguments):
        if name not in BY_NAME or not isinstance(arguments, dict):
            raise ValueError("unknown Evo tool or arguments")
        validate_value(arguments, BY_NAME[name]["inputSchema"])
        if self.closed:
            raise RuntimeError("Evo adapter closed")

    def _rpc(self, method, params, *, timeout=None):
        request_id = uuid.uuid4().hex
        request = {
            "version": 1,
            "id": request_id,
            "session_id": self.session_id,
            "method": method,
            "params": params,
        }
        encoded = (json.dumps(request, allow_nan=False) + "\n").encode()
        if len(encoded) > 64 * 1024:
            raise ValueError("Evo request too large")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout or self.options["socket_timeout_s"])
            connection.connect(self.options["control_socket"])
            connection.sendall(encoded)
            with connection.makefile("rb") as reader:
                line = reader.readline(MAX_RESPONSE + 1)
        if len(line) > MAX_RESPONSE or not line.endswith(b"\n"):
            raise ConnectionError("Evo response is incomplete or too large")
        value = json.loads(
            line,
            parse_constant=lambda _: (_ for _ in ()).throw(
                ValueError("nonfinite response")
            ),
        )
        if (
            not isinstance(value, dict)
            or value.get("version") != 1
            or not isinstance(value.get("version"), int)
            or isinstance(value.get("version"), bool)
            or value.get("id") != request_id
            or not isinstance(value.get("ok"), bool)
        ):
            raise ValueError("invalid Evo response identity or version")
        expected = {"version", "id", "ok", "result" if value["ok"] else "error"}
        if set(value) != expected:
            raise ValueError("invalid Evo response fields")
        if not value["ok"]:
            error = value.get("error")
            if (
                not isinstance(error, dict)
                or not isinstance(error.get("code"), str)
                or not isinstance(error.get("message"), str)
                or "result" in value
            ):
                raise ValueError("invalid Evo error response")
            raise EvoRefused("Evo refused operation: " + json.dumps(value.get("error")))
        if not isinstance(value.get("result"), dict):
            raise ValueError("invalid Evo result")
        return value["result"]

    def _ensure_session(self):
        with self._session_lock:
            if self._failure:
                raise ConnectionError(
                    f"Evo session lost; start a new session: {self._failure}"
                )
            if self.session_id:
                return
            operator = self.config.root / "operator"
            operator.mkdir(mode=0o700, exist_ok=True)
            ident = uuid.uuid4().hex
            value = self._rpc(
                "open_session",
                {"session_id": ident, "directory": str(self.config.root.resolve())},
            )
            if value.get("session_id") != ident or value.get("lease_s") != 0.5:
                raise ValueError("unexpected Evo session contract")
            self.session_id = ident
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat, name="evo-session-lease", daemon=True
            )
            self._heartbeat_thread.start()

    def _heartbeat(self):
        sequence = 0
        while not self._stop.is_set():
            sequence += 1
            try:
                self._rpc("heartbeat", {"sequence": sequence}, timeout=0.2)
            except Exception as error:
                self._failure = str(error)
                self._stop.set()
                return  # Evo's own lease watchdog withdraws motion.
            self._stop.wait(0.1)

    def status(self):
        return self._rpc("status", {})

    def _save_observation(self, value):
        value = copy.deepcopy(value)
        root = (
            self.config.root
            / "agent-workspace"
            / "runtime"
            / "frames"
            / uuid.uuid4().hex
        )
        root.mkdir(parents=True, exist_ok=False)
        images = []
        for camera, metadata in value.get("cameras", {}).items():
            if camera not in ("base", "left_arm", "right_arm") or not isinstance(
                metadata, dict
            ):
                raise ValueError("invalid camera identity")
            encoded = metadata.pop("jpeg")
            jpeg = base64.b64decode(encoded, validate=True)
            if (
                not 4 <= len(jpeg) <= 8 * 1024 * 1024
                or not jpeg.startswith(b"\xff\xd8")
                or not jpeg.endswith(b"\xff\xd9")
            ):
                raise ValueError("invalid camera JPEG")
            target = root / (camera + ".jpg")
            temporary = target.with_suffix(".tmp")
            temporary.write_bytes(jpeg)
            os.replace(temporary, target)
            metadata["path"] = str(
                target.relative_to(self.config.root / "agent-workspace")
            )
            images.append({"type": "image", "mimeType": "image/jpeg", "data": encoded})
        value["frame_id"] = root.name
        temporary = root / "meta.tmp"
        temporary.write_text(json.dumps(value, allow_nan=False, indent=2))
        os.replace(temporary, root / "meta.json")
        return value, images

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name in MOTION or name in ("evo_action_status", "evo_cancel", "evo_hold"):
            if not (name in ("evo_cancel", "evo_hold") and self.session_id):
                self._ensure_session()
        try:
            value = self._rpc(
                name.removeprefix("evo_"),
                arguments,
                **({"timeout": 0.2} if name in self.stop_tools else {}),
            )
            if name in MOTION and (
                value.get("state") != "accepted"
                or not isinstance(value.get("action_id"), str)
                or not value["action_id"]
            ):
                raise ValueError("invalid Evo action acceptance")
        except EvoRefused as error:
            if name in self.stop_tools:
                self._failure = str(error)
                self._stop.set()
            raise
        except Exception as error:
            if name in MOTION or name in self.stop_tools:
                self._failure = str(error)
                self._stop.set()  # No retry of an action with an unknown result.
            raise
        return (
            self._save_observation(value)
            if name in {"evo_observe", "evo_detect_buttons"}
            or (name == "evo_history" and "observation_id" in arguments)
            else (value, [])
        )

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.session_id:
                self._rpc("close_session", {}, timeout=0.2)
        except Exception as error:
            self._failure = self._failure or str(error)
        finally:
            self._stop.set()
            if self._heartbeat_thread:
                self._heartbeat_thread.join(timeout=0.5)
