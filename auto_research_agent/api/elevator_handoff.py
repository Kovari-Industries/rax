"""Read-only admission after teleoperated entry using Evo's status fields.

No motion, lease acquisition, stop command, or inferred sensor defaults.
The native controller remains responsible for lease enforcement at dispatch.
"""
from dataclasses import dataclass
import math
import time
from .base import PrimitiveError, vector


@dataclass(frozen=True)
class CabinEntryHandoff:
    owner: str
    action_id: str
    expected_pose: tuple

    def validate(self):
        if any(not isinstance(v, str) or not v for v in (self.owner, self.action_id)):
            raise ValueError("handoff needs the receiving owner and final action ID")
        vector(self.expected_pose)


def check_handoff_status(status, handoff, travel, tolerances, *, now_us=None):
    """Require native-format fresh feedback and the original strict stop cap."""
    handoff.validate()
    if not isinstance(status, dict):
        raise PrimitiveError("handoff status is unavailable")
    if (status.get("owner") != handoff.owner or status.get("armed") is not True
            or "active_action" not in status or status["active_action"] is not None
            or status.get("feedback_fresh") is not True or status.get("motion_blocked") is not False):
        raise PrimitiveError("handoff requires armed ownership, idle control and fresh feedback")
    last = status.get("last_action", {})
    if (not isinstance(last, dict) or last.get("action_id") != handoff.action_id
            or last.get("state") != "succeeded" or last.get("physical_stop_verified") is not True):
        raise PrimitiveError("handoff final native action and stop are unconfirmed")
    state = status.get("state", {})
    if not isinstance(state, dict) or "fault" not in state or state["fault"] is not None:
        raise PrimitiveError("handoff measured state has a fault or missing fault field")
    now = time.time_ns()//1000 if now_us is None else now_us
    stamps = []
    for key in ("source_us", "receipt_us", "odometry_source_us", "odometry_receipt_us"):
        stamp = state.get(key)
        if type(stamp) is not int or not 0 <= now-stamp <= 200_000:
            raise PrimitiveError("handoff feedback exceeds native 0.2 second freshness bound")
        stamps.append(stamp)
    positions, speeds = state.get("positions"), state.get("velocities")
    if (not isinstance(positions, dict) or not isinstance(speeds, dict) or not positions
            or positions.keys() != speeds.keys() or not travel
            or not set(travel) <= positions.keys()):
        raise PrimitiveError("handoff joint feedback is incomplete")
    if any(type(v) not in (int, float) or not math.isfinite(v) for values in (positions, speeds) for v in values.values()):
        raise PrimitiveError("handoff joint feedback is nonfinite")
    if any(abs(v) >= .005 for v in speeds.values()):
        raise PrimitiveError("handoff joints have not stopped")
    odometry = state.get("odometry")
    if not isinstance(odometry, dict):
        raise PrimitiveError("handoff odometry is unavailable")
    velocity = vector(odometry.get("velocity", ()))
    if any(abs(v) >= .005 for v in velocity):
        raise PrimitiveError("handoff base has not stopped")
    for name, target in travel.items():
        tolerance = tolerances.get(name, {}).get("tolerance")
        if (type(target) not in (int, float) or not math.isfinite(target)
                or type(tolerance) not in (int, float) or not math.isfinite(tolerance) or tolerance <= 0
                or abs(positions[name]-target) > tolerance):
            raise PrimitiveError("handoff travel posture is unconfirmed: "+name)
    return {"source_us": stamps[0], "odometry_source_us": stamps[2],
            "owner": handoff.owner, "action_id": handoff.action_id,
            "positions": dict(positions), "velocities": dict(speeds), "base_velocity": list(velocity)}
