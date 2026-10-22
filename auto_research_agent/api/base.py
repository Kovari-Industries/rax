"""Measured base primitives through Evo's sole motion owner.

The runtime supplies lidar pose and calibrated point-cloud geometry. A duration
or command acknowledgement is never treated as an arrival measurement.
"""
from dataclasses import dataclass, field
import math
import time

from .evo import Evo


def vector(value, size=3):
    value = tuple(value)
    if len(value) != size or any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"expected {size} finite coordinates")
    return value


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def dot(a, b):
    return sum(x*y for x, y in zip(a, b, strict=True))


def add(a, b):
    return tuple(x+y for x, y in zip(a, b, strict=True))


def scale(a, k):
    return tuple(x*k for x in a)


def unit(a):
    a = vector(a)
    length = math.sqrt(dot(a, a))
    if length < 1e-8:
        raise ValueError("zero direction")
    return scale(a, 1/length)


class PrimitiveError(RuntimeError):
    """The requested postcondition was not established. No recovery is implied."""
    def __init__(self, reason, evidence=()):
        super().__init__(reason)
        self.evidence = list(evidence)


@dataclass(frozen=True)
class PrimitiveResult:
    stage: str
    outcome: str
    evidence: tuple = ()
    measured: dict = field(default_factory=dict)


class Base:
    def __init__(self, context, *, cancelled=None):
        self.context = context
        self.evo = Evo(context)
        self.cancelled = cancelled or (lambda: False)
        self.capabilities = context.value("evo_capabilities")
        scene = self.capabilities.get("scene", {})
        if not scene.get("configured") or not isinstance(scene.get("settings"), dict):
            raise PrimitiveError("calibrated point-cloud base control is not configured")
        self.settings = scene["settings"]
        self.evidence = []

    def check_cancel(self):
        if self.cancelled():
            raise PrimitiveError("cancelled", self.evidence)

    def observe_scene(self):
        self.check_cancel()
        value = self.context.value("evo_scene")
        stamp = value.get("capture_us")
        if type(stamp) is not int or not 0 <= time.time() - stamp/1e6 <= self.settings["max_age_s"]:
            raise PrimitiveError("stale or invalid point-cloud capture", self.evidence)
        if value.get("frame") != "base_link" or value.get("heading_source") != "lidar_fixed_reference":
            raise PrimitiveError("unqualified scene frame or heading source", self.evidence)
        vector(value["pose"])
        if not value.get("observation_id"):
            raise PrimitiveError("scene lacks observation identity", self.evidence)
        self.evidence.append({"scene_id": value["observation_id"], "capture_us": stamp})
        return value

    def _move(self, delta, doorway_id=None):
        self.check_cancel()
        delta = vector(delta)
        if not any(delta):
            return PrimitiveResult("base", "already_aligned", measured=self.observe_scene())
        args = {"delta": list(delta)}
        if doorway_id is not None:
            args["doorway_id"] = doorway_id
        result = self.evo.execute("evo_move_base", wait_timeout_s=95,
                                  cancelled=self.cancelled, **args)
        measured = result.get("base_result")
        if not isinstance(measured, dict) or "residual" not in measured:
            raise PrimitiveError("base action lacks measured residual", self.evidence)
        e = vector(measured["residual"])
        if math.hypot(*e[:2]) > self.settings["position_tolerance_m"] or abs(e[2]) > self.settings["angle_tolerance_rad"]:
            raise PrimitiveError("base target was not reached", self.evidence)
        self.evidence.append({"action_id": result["action_id"], "base_result": measured})
        return PrimitiveResult("base", "succeeded", tuple(self.evidence), measured)

    def rotate_base(self, angle_rad, *, doorway_id=None):
        return self._move((0, 0, vector((angle_rad,), 1)[0]), doorway_id)

    def translate_base(self, dx_m, dy_m, *, doorway_id=None):
        return self._move((dx_m, dy_m, 0), doorway_id)

    def target(self, kind="panel", doorway_id=None):
        scene = self.observe_scene()
        target = scene.get(kind)
        if not isinstance(target, dict):
            raise PrimitiveError(f"{kind} geometry is unavailable", self.evidence)
        if kind == "doorway" and (target.get("state") != "open" or target.get("stable_samples", 0) < 3
                                  or doorway_id is not None and target.get("id") != doorway_id):
            raise PrimitiveError("doorway is not stably open or identity changed", self.evidence)
        point = vector(target["point" if kind == "panel" else "center"])
        normal = unit(target["normal"])
        if abs(normal[2]) > 0.05:
            raise PrimitiveError("surface is not vertical", self.evidence)
        # Face the aperture from the current side, including an exit trip.
        if kind == "doorway" and dot(point, normal) < 0:
            normal = scale(normal, -1)
        return scene, target, point, normal

    def _align(self, mode, target_value, *, kind="panel", doorway_id=None, max_travel_m=1.0, max_rotation_rad=math.pi):
        vector((target_value,), 1)
        positive(max_travel_m, "max_travel_m")
        positive(max_rotation_rad, "max_rotation_rad")
        travelled = rotated = 0.0
        for _ in range(16):
            _, target, point, normal = self.target(kind, doorway_id)
            door = target["id"] if kind == "doorway" else None
            if mode == "angle":
                error = math.atan2(normal[1], normal[0]) - target_value
                error = math.atan2(math.sin(error), math.cos(error))
                if abs(error) <= self.settings["angle_tolerance_rad"]:
                    return PrimitiveResult("align_base_angle", "succeeded", tuple(self.evidence), {"error_rad": error})
                step = max(-self.settings["max_turn_rad"], min(self.settings["max_turn_rad"], error))
                if rotated + abs(step) > max_rotation_rad:
                    raise PrimitiveError("alignment angle budget exceeded", self.evidence)
                result = self.rotate_base(step, doorway_id=door)
                rotated += result.measured["angle_travelled_rad"]
            else:
                axis = normal if mode == "distance" else (-normal[1], normal[0], 0)
                error = dot(point, axis) - target_value
                if abs(error) <= self.settings["position_tolerance_m"]:
                    return PrimitiveResult("align_base_"+mode, "succeeded", tuple(self.evidence), {"error_m": error})
                step = max(-0.15, min(0.15, error))
                if travelled + abs(step) > max_travel_m:
                    raise PrimitiveError("alignment travel budget exceeded", self.evidence)
                result = self.translate_base(step*axis[0], step*axis[1], doorway_id=door)
                travelled += result.measured["distance_travelled_m"]
        raise PrimitiveError("base alignment did not converge", self.evidence)

    def align_base_angle(self, angle_rad=0.0, **kwargs):
        return self._align("angle", angle_rad, **kwargs)

    def align_base_distance(self, standoff_m, **kwargs):
        positive(standoff_m, "standoff_m")
        return self._align("distance", standoff_m, **kwargs)

    def align_base_lateral(self, offset_m=0.0, **kwargs):
        return self._align("lateral", offset_m, **kwargs)

    def wait_for_door_open(self, doorway_id=None, timeout_s=30):
        positive(timeout_s, "timeout_s")
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            scene = self.observe_scene()
            door = scene.get("doorway")
            if door and (doorway_id is None or door.get("id") == doorway_id) and door.get("state") == "open" and door.get("stable_samples", 0) >= 3:
                return door
            time.sleep(0.05)
        raise PrimitiveError("doorway did not become open and clear before timeout", self.evidence)

    def align_to_doorway(self, doorway_id=None, *, staging_gap_m):
        positive(staging_gap_m, "staging_gap_m")
        minimum = self.settings["footprint"][0]+self.settings["margin_m"]
        if staging_gap_m <= minimum:
            raise ValueError("staging gap must keep the complete base before the sill")
        door = self.wait_for_door_open(doorway_id)
        args = {"kind": "doorway", "doorway_id": door["id"]}
        self.align_base_angle(**args)
        self.align_base_lateral(**args)
        self.align_base_distance(staging_gap_m, **args)
        return self.target(**args)[1]
