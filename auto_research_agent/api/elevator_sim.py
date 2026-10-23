"""Explicit privileged simulation baseline for the existing elevator routines.

This class uses measured simulator state. It does not test image recognition,
calibration, or transfer to a physical robot. It refuses a live context.
"""
import math
import time
from .base import PrimitiveError, PrimitiveResult, vector, unit
from .elevator import ButtonObservation, Elevator


class SimulationElevator(Elevator):
    def __init__(self, context, settings, **kwargs):
        caps = context.value("evo_capabilities")
        if caps.get("simulated") is not True or caps.get("scene", {}).get("observation_source") != "simulator_ground_truth":
            raise PrimitiveError("SimulationElevator requires explicit simulator ground truth")
        kwargs.setdefault("clock", context.simulation_clock)
        super().__init__(context, settings, **kwargs)

    def observe_button(self, label, side):
        self._profile(side)
        if label not in self.settings.press_offsets:
            raise PrimitiveError("unknown panel control")
        value = self.context.value("sim_button_observation", label=label, side=side)
        if value.get("source") != "simulator_ground_truth" or value.get("label") != label:
            raise PrimitiveError("unexpected simulation observation source or label")
        stamp = value.get("capture_us")
        if type(stamp) is not int or stamp <= self._last_stop_us or not 0 <= time.time()-stamp/1e6 <= self.settings.max_observation_age_s:
            raise PrimitiveError("stale simulation observation")
        uncertainty = value.get("uncertainty_m")
        if type(uncertainty) not in (int, float) or not math.isfinite(uncertainty) or not 0 < uncertainty <= self.settings.max_geometry_uncertainty_m:
            raise PrimitiveError("invalid simulation uncertainty")
        observation = ButtonObservation(
            label, self.settings.panel_id, side+"_arm", value["observation_id"], stamp,
            vector(value["position"]), unit(value["normal"]), uncertainty, 0.,
            vector(value["scene_pose"]), 1.,
        )
        self.evidence.append({"observation_id": observation.observation_id,
                              "source": "simulator_ground_truth", "label": label})
        return observation

    def verify_button_activation(self, label, side):
        self.observe_button(label, side)
        value = self.context.value("sim_button_activation", label=label)
        valid = (value.get("source") == "simulator_ground_truth"
                 and value.get("target") == label
                 and value.get("single_correct_press") is True and value.get("released") is True)
        return PrimitiveResult("verify_button_activation", "succeeded" if valid else "unknown",
                               tuple(self.evidence), value)
