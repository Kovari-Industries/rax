"""Opt-in cabin ride with checked turning, interior contact and landing sensors.

The scene provider supplies calibrated geometry or explicit simulator geometry.
Landing identity must come from image OCR. A simulator floor state is not an OCR
observation. This module does not provide a hardware perception qualification.
"""
from dataclasses import asdict, dataclass
import math
import time

from .base import PrimitiveError, PrimitiveResult, positive, vector


@dataclass(frozen=True)
class CabinRideSettings:
    origin_floor: str
    floor_heights_m: dict
    entry_depth_m: float
    exit_depth_m: float
    turn_rad: float = math.pi
    hall_call_label: str = "CALL"
    cabin_staging_lateral_m: float = 0.0
    arrival_timeout_s: float = 90.0
    floor_max_age_s: float = 3.0
    floor_max_skew_s: float = 0.1
    min_ocr_confidence: float = 0.8
    min_image_luma: float = 0.05
    max_geometry_uncertainty_m: float = 0.02
    max_pressure_uncertainty_pa: float = 2.0
    max_pressure_span_pa: float = 2.0
    height_tolerance_m: float = 0.75
    air_temperature_k: float = 293.15

    def validate(self):
        if not isinstance(self.origin_floor, str) or self.origin_floor not in self.floor_heights_m:
            raise ValueError("origin floor must have a configured height")
        if not isinstance(self.hall_call_label, str) or not self.hall_call_label:
            raise ValueError("hall call label must be an exact nonempty string")
        if len(self.floor_heights_m) < 2 or any(not isinstance(k, str) or not k for k in self.floor_heights_m):
            raise ValueError("configure at least two exact floor labels")
        heights = sorted(vector(self.floor_heights_m.values(), len(self.floor_heights_m)))
        for name in ("entry_depth_m", "exit_depth_m", "arrival_timeout_s", "floor_max_age_s",
                     "floor_max_skew_s", "max_geometry_uncertainty_m", "max_pressure_uncertainty_pa",
                     "max_pressure_span_pa", "height_tolerance_m"):
            positive(getattr(self, name), name)
        vector((self.cabin_staging_lateral_m,), 1)
        if abs(abs(vector((self.turn_rad,), 1)[0])-math.pi) > 1e-12:
            raise ValueError("cabin turn must be exactly 180 degrees")
        if not 0 < self.min_ocr_confidence <= 1 or not 0 < self.min_image_luma < 1:
            raise ValueError("invalid OCR or image brightness threshold")
        if not 263 <= self.air_temperature_k <= 313:
            raise ValueError("air temperature is outside the configured indoor range")
        if min(b-a for a, b in zip(heights, heights[1:])) <= 2*self.height_tolerance_m:
            raise ValueError("floor height acceptance bands must not overlap")


class CabinRide:
    """Compose an existing Elevator or SimulationElevator without changing it."""

    def __init__(self, task, settings, *, hall_task=None):
        settings.validate()
        self.task, self.settings, self.hall_task = task, settings, hall_task
        if hall_task is not None:
            if hall_task.settings.panel_id == task.settings.panel_id:
                raise ValueError("hall call and cabin destination panels must be distinct")
            if set(hall_task.settings.press_offsets) != {settings.hall_call_label}:
                raise ValueError("hall panel must have exactly one call button")
        self.base, self.context = task.base, task.context
        if set(task.profiles) != {"left", "right"}:
            raise ValueError("cabin arm selection requires both calibrated arms")
        if task.settings.search_turns_rad:
            raise ValueError("cabin button search must not turn the base")
        self.evidence = []
        self._cabin_id = None
        self._scene_id = None
        self._light_configuration_id = None
        self._light_receipt = None
        self._doorway_id = None
        self._last_motion_us = 0
        self._baseline = None
        self._turn_pose = None
        self._used = False

    def _fresh(self, stamp, max_age_s, *, post_motion=False):
        if type(stamp) is not int or not 0 <= time.time()-stamp/1e6 <= max_age_s:
            raise PrimitiveError("stale or invalid cabin sensor capture", self.evidence)
        if post_motion and stamp <= max(self._last_motion_us, self.task._last_stop_us):
            raise PrimitiveError("cabin capture predates the last motion", self.evidence)

    @staticmethod
    def _image_hash(value):
        return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)

    def _scene(self):
        scene = self.base.observe_scene()
        self._fresh(scene["capture_us"], self.base.settings["max_age_s"], post_motion=True)
        return scene

    def _cabin(self, scene, *, turn_clearance=False, allow_unverified_image=False, center_offset_m=(0., 0.)):
        cabin = scene.get("cabin")
        if not isinstance(cabin, dict) or not isinstance(cabin.get("id"), str) or not cabin["id"]:
            raise PrimitiveError("cabin geometry is unavailable", self.evidence)
        if self._cabin_id is not None and cabin["id"] != self._cabin_id:
            raise PrimitiveError("cabin identity changed", self.evidence)
        if cabin.get("doorway_id") != self._doorway_id or cabin.get("panel_id") != self.task.settings.panel_id:
            raise PrimitiveError("cabin doorway or panel identity differs", self.evidence)
        uncertainty = positive(cabin.get("geometry_uncertainty_m"), "cabin geometry uncertainty")
        if uncertainty > self.settings.max_geometry_uncertainty_m:
            raise PrimitiveError("cabin geometry is too uncertain", self.evidence)
        polygon = [vector(p, 2) for p in cabin.get("polygon_m", ())]
        if len(polygon) < 3:
            raise PrimitiveError("cabin polygon is unavailable", self.evidence)
        area = sum(a[0]*b[1]-a[1]*b[0] for a, b in zip(polygon, polygon[1:]+polygon[:1]))
        if abs(area) < 1e-8:
            raise PrimitiveError("cabin polygon has no usable area", self.evidence)
        sign = 1 if area > 0 else -1
        planes = []
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            length = math.dist(a, b)
            if length < 1e-8:
                raise PrimitiveError("cabin polygon has a zero edge", self.evidence)
            inward = (-sign*(b[1]-a[1])/length, sign*(b[0]-a[0])/length)
            if min((p[0]-a[0])*inward[0]+(p[1]-a[1])*inward[1] for p in polygon) < -1e-8:
                raise PrimitiveError("cabin polygon must be convex", self.evidence)
            planes.append((a, inward))
        lighting = cabin.get("lighting", {})
        scene_id, configuration_id = cabin.get("scene_id"), lighting.get("configuration_id")
        if not isinstance(scene_id, str) or not scene_id or not isinstance(configuration_id, str) or not configuration_id:
            raise PrimitiveError("cabin scene or lighting configuration identity is unavailable", self.evidence)
        if (self._scene_id is not None and scene_id != self._scene_id
                or self._light_configuration_id is not None and configuration_id != self._light_configuration_id):
            raise PrimitiveError("cabin scene or lighting configuration changed", self.evidence)
        if (lighting.get("enabled") is not True
                or lighting.get("source") not in {"authored_simulation", "camera_observation"}
                or lighting.get("source") == "authored_simulation" and self.base.capabilities.get("simulated") is not True):
            raise PrimitiveError("cabin lighting is not continuously enabled", self.evidence)
        if self._light_receipt is None and not allow_unverified_image:
            raise PrimitiveError("cabin admission image has not been verified", self.evidence)
        self._scene_id, self._light_configuration_id = scene_id, configuration_id
        hx, hy, height = vector(self.base.settings["footprint"])
        margin = positive(self.base.settings["margin_m"], "base margin")
        if min(hx, hy, height) <= 0:
            raise PrimitiveError("full robot envelope is unavailable", self.evidence)
        ceiling = positive(cabin.get("ceiling_height_m"), "cabin ceiling height")
        if ceiling-height < margin+uncertainty:
            raise PrimitiveError("robot has insufficient cabin ceiling clearance", self.evidence)
        # The circle contains every yaw of the complete source travel envelope.
        reserve = margin+uncertainty+self.base.settings["position_tolerance_m"]
        radius = math.hypot(hx, hy)
        center = vector(center_offset_m, 2)
        clearances = [(center[0]-a[0])*n[0]+(center[1]-a[1])*n[1]
                      -(radius if turn_clearance else abs(n[0])*hx+abs(n[1])*hy)-reserve
                      for a, n in planes]
        if turn_clearance and min(clearances) < 0:
            raise PrimitiveError("complete robot rotation envelope is not inside the cabin", self.evidence)
        self._cabin_id = cabin["id"]
        return cabin, planes, min(clearances)

    def _verify_lighting_receipt(self, scene, receipt, purpose, *, minimum_capture_us=0):
        cabin, _, _ = self._cabin(scene, allow_unverified_image=True)
        if (receipt.get("cabin_id") != cabin["id"] or receipt.get("scene_id") != self._scene_id
                or receipt.get("configuration_id") != self._light_configuration_id
                or receipt.get("enabled") is not True
                or receipt.get("source") != cabin["lighting"]["source"]
                or not self._image_hash(receipt.get("image_sha256"))):
            raise PrimitiveError("cabin image receipt does not match the scene and lighting", self.evidence)
        self._fresh(receipt.get("capture_us"), self.settings.floor_max_age_s, post_motion=True)
        if receipt["capture_us"] < minimum_capture_us:
            raise PrimitiveError("cabin image predates the decision capture", self.evidence)
        if abs(receipt["capture_us"]-scene["capture_us"])/1e6 > self.settings.floor_max_skew_s:
            raise PrimitiveError("cabin image and geometry are not synchronized", self.evidence)
        luma = vector((receipt.get("mean_luma"),), 1)[0]
        if not self.settings.min_image_luma <= luma <= 1:
            raise PrimitiveError("cabin image is too dark", self.evidence)
        self._light_receipt = dict(receipt)
        self.evidence.append({"stage": "lighting_verified", "purpose": purpose,
                              "minimum_capture_us": minimum_capture_us, "receipt": dict(receipt)})

    def _refresh_lighting(self, purpose, *, minimum_capture_us=0):
        scene = self._scene()
        cabin, _, _ = self._cabin(scene, allow_unverified_image=True)
        self.base.check_cancel()
        receipt = self.context.value("rax_cabin_image", doorway_id=self._doorway_id, cabin_id=cabin["id"])
        self._verify_lighting_receipt(self._scene(), receipt, purpose, minimum_capture_us=minimum_capture_us)

    def _inside(self, scene):
        cabin, planes, clearance = self._cabin(scene)
        if clearance < 0:
            raise PrimitiveError("complete robot is not inside the cabin", self.evidence)
        return cabin, planes, clearance

    def stage_inside(self):
        if self.task._contact is not None:
            raise PrimitiveError("retract before cabin staging")
        self.task.stow_arms()
        scene = self._scene()
        self._inside(scene)
        start = vector(scene["pose"])
        lateral = self.settings.cabin_staging_lateral_m
        if abs(lateral) > self.base.settings["max_step_m"]:
            raise PrimitiveError("cabin staging exceeds the existing translation bound", self.evidence)
        # The cabin polygon is convex. Containment of the current footprint and
        # the target rotation circle contains the straight translation sweep.
        _, _, target_clearance = self._cabin(scene, turn_clearance=True, center_offset_m=(0., lateral))
        result = self.base.translate_base(0., lateral)
        self._last_motion_us = time.time_ns()//1000
        final_scene = self._scene()
        _, _, final_clearance = self._cabin(final_scene, turn_clearance=True)
        final = vector(final_scene["pose"])
        expected = (start[0]-math.sin(start[2])*lateral, start[1]+math.cos(start[2])*lateral)
        yaw_error = math.atan2(math.sin(final[2]-start[2]), math.cos(final[2]-start[2]))
        if math.dist(final[:2], expected) > self.base.settings["position_tolerance_m"] or abs(yaw_error) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("measured inside cabin staging endpoint was not reached", self.evidence)
        measured = {"lateral_m": lateral, "start_pose": start, "final_pose": final,
                    "minimum_target_turn_clearance_m": min(target_clearance, final_clearance),
                    "base_result": result.measured}
        self.evidence.append({"stage": "cabin_staging", **measured})
        return PrimitiveResult("stage_inside_cabin", "succeeded", tuple(self.evidence+self.base.evidence), measured)

    def turn_inside(self):
        if self.task._contact is not None:
            raise PrimitiveError("retract before the cabin turn")
        self.task.stow_arms()
        scene = self._scene()
        start = vector(scene["pose"])
        cabin, _, clearance = self._cabin(scene, turn_clearance=True)
        max_turn = positive(self.base.settings["max_turn_rad"], "maximum turn")
        remaining = self.settings.turn_rad
        turns = []
        while abs(remaining) > 1e-10:
            self._cabin(self._scene(), turn_clearance=True)
            step = math.copysign(min(abs(remaining), max_turn), remaining)
            result = self.base.rotate_base(step)
            self._last_motion_us = time.time_ns()//1000
            turns.append(asdict(result))
            remaining -= step
        final_scene = self._scene()
        _, _, final_clearance = self._cabin(final_scene, turn_clearance=True)
        final = vector(final_scene["pose"])
        yaw_error = math.atan2(math.sin(final[2]-start[2]-self.settings.turn_rad),
                               math.cos(final[2]-start[2]-self.settings.turn_rad))
        if math.dist(start[:2], final[:2]) > self.base.settings["position_tolerance_m"] or abs(yaw_error) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("measured cabin turnaround endpoint was not reached", self.evidence)
        self._turn_pose = final
        result = PrimitiveResult("turn_inside_cabin", "succeeded", tuple(self.evidence+self.base.evidence),
                                 {"cabin_id": cabin["id"], "angle_rad": self.settings.turn_rad,
                                  "minimum_envelope_clearance_m": min(clearance, final_clearance),
                                  "start_pose": start, "final_pose": final, "turns": turns})
        self.evidence.append(asdict(result))
        return result

    def _stationary_panel(self):
        scene = self._scene()
        _, planes, _ = self._inside(scene)
        if self._turn_pose is None:
            raise PrimitiveError("a checked inside turn must precede the cabin press")
        pose = vector(scene["pose"])
        error = math.atan2(math.sin(pose[2]-self._turn_pose[2]), math.cos(pose[2]-self._turn_pose[2]))
        if math.dist(pose[:2], self._turn_pose[:2]) > self.base.settings["position_tolerance_m"] or abs(error) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("base moved after the cabin turn", self.evidence)
        panel = scene.get("panel", {})
        if panel.get("location") != "inside_cabin" or panel.get("cabin_id") != self._cabin_id or panel.get("id") != self.task.settings.panel_id:
            raise PrimitiveError("button panel is not the identified interior panel", self.evidence)
        point = vector(panel["point"])
        uncertainty = self.settings.max_geometry_uncertainty_m
        if any((point[0]-a[0])*n[0]+(point[1]-a[1])*n[1] < -uncertainty for a, n in planes):
            raise PrimitiveError("button panel lies outside the cabin", self.evidence)
        return scene

    def press_cabin_button(self, label):
        # Reuse each detector's just-captured RGB evidence. The image tool can
        # return that receipt without another render or OCR inference. Geometry
        # queries between captures remain cheap and retain their original age gate.
        original = self.task.observe_button
        had_override = "observe_button" in self.task.__dict__
        previous_override = self.task.__dict__.get("observe_button")

        def checked_observation(*args, **kwargs):
            observation = original(*args, **kwargs)
            self._refresh_lighting("panel_capture", minimum_capture_us=observation.capture_us if observation else 0)
            return observation

        self.task.observe_button = checked_observation
        try:
            return self._press_cabin_button(label)
        finally:
            if had_override:
                self.task.observe_button = previous_override
            else:
                del self.task.observe_button

    def _press_cabin_button(self, label):
        if label not in self.task.settings.press_offsets:
            raise PrimitiveError("unknown calibrated cabin press point")
        self._stationary_panel()
        observations = self.task.search_button(label, "auto")
        self._stationary_panel()
        selection = self.task.select_press_arm(observations, "auto")
        self.evidence.append({"stage": "select_cabin_arm", "selection": asdict(selection)})
        if selection.side is None:
            raise PrimitiveError("neither arm has a checked cabin approach", selection.candidates)
        side = selection.side
        self.task.prepare_press_arm(selection)
        if not self.task.settings.direct_press_alignment:
            self.task.align_arm_to_button(label, side)
        self._stationary_panel()
        self.task.approach_button(label, side)
        self.task.press_button(label, side)
        self.task.retract_arm(side)
        activation = self.task.verify_button_activation(label, side)
        self.task.stow_arms()
        self._stationary_panel()
        if activation.outcome != "succeeded":
            raise PrimitiveError("interior button activation and release are unconfirmed", self.evidence)
        return PrimitiveResult("press_cabin_button", "succeeded", tuple(self.evidence+self.task.evidence),
                               {"arm": side, "activation": activation.outcome,
                                "arm_candidates": selection.candidates})

    def _floor_observation(self):
        self.base.check_cancel()
        value = self.context.value("rax_floor_observation", doorway_id=self._doorway_id)
        if not isinstance(value, dict) or not value.get("observation_id"):
            raise PrimitiveError("floor observation identity is unavailable", self.evidence)
        self._fresh(value.get("capture_us"), self.settings.floor_max_age_s, post_motion=True)
        ocr, baro = value.get("ocr", {}), value.get("barometer", {})
        if (ocr.get("source") != "image_ocr" or ocr.get("text_readable") is not True
                or not isinstance(ocr.get("text"), str) or not ocr["text"]
                or not self._image_hash(ocr.get("image_sha256"))
                or ocr.get("location") != "landing_door_jamb" or ocr.get("doorway_id") != self._doorway_id):
            raise PrimitiveError("fresh landing-jamb image OCR is unavailable", self.evidence)
        score = vector((ocr.get("confidence"),), 1)[0]
        if not self.settings.min_ocr_confidence <= score <= 1:
            raise PrimitiveError("floor OCR confidence is insufficient", self.evidence)
        expected_source = "simulated_barometer" if self.base.capabilities.get("simulated") is True else "barometer"
        if baro.get("source") != expected_source or baro.get("stable") is not True or type(baro.get("sample_count")) is not int or baro["sample_count"] < 3:
            raise PrimitiveError("stable barometer samples are unavailable", self.evidence)
        for sensor in (ocr, baro):
            self._fresh(sensor.get("capture_us"), self.settings.floor_max_age_s, post_motion=True)
            if abs(sensor["capture_us"]-value["capture_us"])/1e6 > self.settings.floor_max_skew_s:
                raise PrimitiveError("floor sensor captures are not synchronized", self.evidence)
        pressure = positive(baro.get("pressure_pa"), "barometer pressure")
        uncertainty = positive(baro.get("uncertainty_pa"), "barometer uncertainty")
        span = vector((baro.get("span_pa"),), 1)[0]
        if not 50000 <= pressure <= 120000 or uncertainty > self.settings.max_pressure_uncertainty_pa or not 0 <= span <= self.settings.max_pressure_span_pa:
            raise PrimitiveError("barometer pressure or stability uncertainty is outside bounds", self.evidence)
        scene = value.get("scene")
        if scene is None:
            scene = self._scene()
        else:
            self._fresh(scene.get("capture_us"), self.settings.floor_max_age_s, post_motion=True)
            if scene.get("frame") != "base_link" or scene.get("heading_source") != self.base.heading_source or not scene.get("observation_id"):
                raise PrimitiveError("floor scene frame is unqualified", self.evidence)
            vector(scene["pose"])
        if abs(scene["capture_us"]-value["capture_us"])/1e6 > self.settings.floor_max_skew_s:
            raise PrimitiveError("floor sensors and door state are not synchronized", self.evidence)
        if self._light_receipt is not None:
            # The floor tool already captured an image. Require its matching
            # brightness receipt; never render or run OCR in ordinary scene checks.
            cabin = scene.get("cabin", {})
            receipt = {**cabin.get("lighting", {}), "cabin_id": cabin.get("id"),
                       "scene_id": cabin.get("scene_id")}
            self._verify_lighting_receipt(scene, receipt, "floor_capture", minimum_capture_us=ocr["capture_us"])
        return value, scene

    def _open(self, scene):
        door = scene.get("doorway", {})
        return door.get("id") == self._doorway_id and door.get("state") == "open" and door.get("stable_samples", 0) >= 3

    def capture_origin(self):
        value, scene = self._floor_observation()
        if (value["ocr"]["text"] != self.settings.origin_floor
                or scene.get("doorway", {}).get("id") != self._doorway_id):
            raise PrimitiveError("origin floor OCR and doorway identity are unconfirmed", self.evidence)
        self._baseline = value
        self.evidence.append({"stage": "origin_verified", "observation": value})
        return PrimitiveResult("origin_verified", "succeeded", measured=value)

    def call_elevator(self):
        if self.hall_task is None:
            raise PrimitiveError("closed elevator needs a separate physical hall call task", self.evidence)
        result = self.hall_task.press_elevator_button(self.settings.hall_call_label)
        self._last_motion_us = time.time_ns()//1000
        if result.outcome != "succeeded" or result.measured.get("activation") != "succeeded":
            raise PrimitiveError("hall call activation, release and stow are unconfirmed", self.evidence)
        return result

    def _height(self, value):
        initial = self._baseline["barometer"]
        current = value["barometer"]
        scale = 287.05*self.settings.air_temperature_k/9.80665
        measured = scale*math.log(initial["pressure_pa"]/current["pressure_pa"])
        low = scale*math.log((initial["pressure_pa"]-initial["uncertainty_pa"])/(current["pressure_pa"]+current["uncertainty_pa"]))
        high = scale*math.log((initial["pressure_pa"]+initial["uncertainty_pa"])/(current["pressure_pa"]-current["uncertainty_pa"]))
        return measured, (low, high)

    def wait_for_destination(self, floor):
        if self._baseline is None:
            raise PrimitiveError("origin barometer baseline is unavailable")
        expected = self.settings.floor_heights_m[floor]-self.settings.floor_heights_m[self.settings.origin_floor]
        deadline = self.base.clock.monotonic()+self.settings.arrival_timeout_s
        last = None
        while self.base.clock.monotonic() < deadline:
            self.base.check_cancel()
            # A closed door has no visible landing label. Wait without inventing OCR.
            scene = self._scene()
            self._inside(scene)
            if self._open(scene):
                value, observed_scene = self._floor_observation()
                height, interval = self._height(value)
                matches = (value["ocr"]["text"] == floor
                           and max(abs(x-expected) for x in interval) <= self.settings.height_tolerance_m)
                last = {"observation": value, "expected_height_delta_m": expected,
                        "barometric_height_delta_m": height, "barometric_height_interval_m": interval,
                        "accepted": matches and self._open(observed_scene)}
                self.evidence.append({"stage": "destination_observation", **last})
                if last["accepted"]:
                    self._inside(observed_scene)
                    return PrimitiveResult("destination_verified", "succeeded", tuple(self.evidence), last)
            self.base.clock.sleep(0.05)
        raise PrimitiveError("destination OCR, barometer and open doorway did not agree before timeout", self.evidence)

    def exit_forward(self, floor):
        # Read both sensors again after any callback or pause before the crossing.
        destination = self.wait_for_destination(floor)
        scene = self._scene()
        self._inside(scene)
        _, door, center, normal = self.base.target("doorway", self._doorway_id)
        if abs(math.atan2(normal[1], normal[0])) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("robot is not facing the exit after the cabin turn", self.evidence)
        extent = self.base.settings["footprint"][0]+self.base.settings["margin_m"]
        if self.settings.exit_depth_m <= extent:
            raise ValueError("exit goal must clear the complete robot")
        goal = (center[0]+normal[0]*self.settings.exit_depth_m,
                center[1]+normal[1]*self.settings.exit_depth_m)
        if goal[0] <= 0:
            raise PrimitiveError("cabin exit must be forward", self.evidence)
        result = self.base.translate_base(*goal, doorway_id=door["id"])
        self._last_motion_us = time.time_ns()//1000
        if result.measured.get("crossing_state") != "after":
            raise PrimitiveError("complete forward exit clearance is unconfirmed", self.base.evidence)
        return PrimitiveResult("exit_cabin_forward", "succeeded", tuple(self.evidence+self.base.evidence),
                               {"floor": floor, "location": "outside", "destination": destination.measured,
                                **result.measured})

    def run(self, destination_floor, doorway_id=None, *, on_stage=None):
        if self._used:
            raise PrimitiveError("a cabin ride instance cannot repeat a press or resume an uncertain ride")
        if destination_floor not in self.settings.floor_heights_m or destination_floor == self.settings.origin_floor:
            raise ValueError("destination must be a different configured floor")
        if destination_floor not in self.task.settings.press_offsets:
            raise ValueError("destination has no calibrated cabin button")
        self._used = True
        door = self._scene().get("doorway", {})
        if not isinstance(door.get("id"), str) or not door["id"] or doorway_id is not None and door["id"] != doorway_id:
            raise PrimitiveError("origin elevator doorway identity is unavailable", self.evidence)
        self._doorway_id = door["id"]
        results = {}

        def stage(name, function):
            if on_stage:
                on_stage({"stage": name, "event": "started"})
            try:
                result = function()
            except Exception as error:
                if on_stage:
                    on_stage({"stage": name, "event": "failed", "reason": str(error)})
                raise
            results[name] = asdict(result)
            if on_stage:
                on_stage({"stage": name, "event": "finished", "result": asdict(result)})
            return result

        stage("origin", self.capture_origin)
        if not self._open(self._scene()):
            stage("hall_call", self.call_elevator)
        self.base.wait_for_door_open(self._doorway_id, self.settings.arrival_timeout_s)
        self._refresh_lighting("admission")  # One fresh RGB check before entry.
        stage("entry", lambda: self.task.navigate_into_elevator(self._doorway_id, goal_depth_m=self.settings.entry_depth_m))
        self._last_motion_us = time.time_ns()//1000
        if self.settings.cabin_staging_lateral_m:
            stage("cabin_staging", self.stage_inside)
        stage("turn", self.turn_inside)
        stage("button", lambda: self.press_cabin_button(destination_floor))
        stage("destination", lambda: self.wait_for_destination(destination_floor))
        stage("exit", lambda: self.exit_forward(destination_floor))
        return PrimitiveResult("ride_elevator", "succeeded", tuple(self.evidence+self.task.evidence+self.base.evidence),
                               {"origin_floor": self.settings.origin_floor, "destination_floor": destination_floor,
                                "location": "outside", "stages": results})
