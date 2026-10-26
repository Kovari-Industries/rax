"""Elevator primitives using calibrated wrist views and measured point clouds.

Arm motion uses Evo tools; base goals use RAX tools backed by Evo actions. Configure each arm's measured viewing posture,
pressing-tip transform, and panel offsets before physical use. This module never
arms the robot, changes limits, or guesses missing calibration.
"""
from copy import deepcopy
from dataclasses import dataclass
import math
import time

from .base import Base, PrimitiveError, PrimitiveResult, vector, positive, dot, add, scale, unit
from .evo import Evo, EvoActionFailed


def rotate_vector(v, q):
    v, q = vector(v), vector(q, 4)
    if abs(dot(q, q)-1) > 1e-3:
        raise ValueError("orientation must be a unit xyzw quaternion")
    t = (2*(q[1]*v[2]-q[2]*v[1]), 2*(q[2]*v[0]-q[0]*v[2]), 2*(q[0]*v[1]-q[1]*v[0]))
    return (v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1],
            v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2],
            v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0])


@dataclass(frozen=True)
class ArmProfile:
    side: str
    view_joints: dict
    press_quaternion_xyzw: tuple
    tip_in_palm_m: tuple
    calibration_id: str
    elbow_joint: str
    minimum_elbow_bend_rad: float
    lateral_offset_m: float = 0.0
    search_joints: tuple = ()

    def validate(self):
        if self.side not in {"left", "right"} or not self.calibration_id:
            raise ValueError("arm profile needs a side and calibration identity")
        required = {f"{self.side}_joint{i}" for i in range(1, 8)}
        if self.elbow_joint not in required:
            raise ValueError("elbow joint must belong to this arm")
        positive(self.minimum_elbow_bend_rad, "minimum_elbow_bend_rad")
        for pose in (self.view_joints, *self.search_joints):
            if set(pose) != required:
                raise ValueError("viewing posture must name exactly the arm joints")
            vector(pose.values(), 7)
            if abs(pose[self.elbow_joint]) < self.minimum_elbow_bend_rad:
                raise ValueError("viewing posture does not bend the elbow enough")
        rotate_vector(self.tip_in_palm_m, self.press_quaternion_xyzw)
        vector((self.lateral_offset_m,), 1)


@dataclass(frozen=True)
class FinalContactPolicy:
    """Explicit optical qualification for a single guarded contact transaction."""
    visible_gap_m: float
    max_inward_travel_m: float
    occlusion_qualification_id: str | dict[str, str]

    def validate(self, settings):
        positive(self.visible_gap_m, "visible_gap_m")
        positive(self.max_inward_travel_m, "max_inward_travel_m")
        identity = self.occlusion_qualification_id
        if isinstance(identity, dict):
            if (not identity or set(identity)-{"left", "right"}
                    or any(not isinstance(v, str) or not v for v in identity.values())):
                raise ValueError("guarded qualification mapping requires explicit left/right identities")
        elif not isinstance(identity, str) or not identity:
            raise ValueError("guarded contact requires an optical qualification identity")
        if self.visible_gap_m <= settings.prepress_gap_m+settings.max_geometry_uncertainty_m:
            raise ValueError("visible gap must exceed the uncertain prepress gap")
        if not self.visible_gap_m+settings.stroke_m <= self.max_inward_travel_m <= settings.approach_budget_m:
            raise ValueError("guarded inward travel must cover the stroke within the existing approach budget")


@dataclass(frozen=True)
class ElevatorSettings:
    arms: tuple
    panel_id: str
    # Physical press point relative to label-box center, in box widths/heights.
    press_offsets: dict
    viewing_carriage_m: float
    panel_standoff_m: float
    prepress_gap_m: float = 0.04
    approach_step_m: float = 0.02
    approach_budget_m: float = 0.4
    retract_gap_m: float = 0.08
    stroke_m: float = 0.01
    dwell_s: float = 0.25
    segment_s: float = 3.0
    position_tolerance_m: float = 0.006
    max_geometry_uncertainty_m: float = 0.02
    max_observation_age_s: float = 1.0
    min_score: float = 0.6
    search_turns_rad: tuple = ()
    max_search_rotation_rad: float = math.pi
    tangent_approach_correction: bool = False
    direct_doorway_staging: bool = False
    reverse_exit: bool = False
    synchronized_view: bool = False
    direct_press_alignment: bool = False
    final_contact: FinalContactPolicy | None = None
    coarse_panel_lateral_offset_m: float | None = None
    retract_segment_s: float | None = None

    def validate(self):
        if self.retract_segment_s is not None and positive(self.retract_segment_s,"retract_segment_s")>10:
            raise ValueError("retract segment exceeds supported duration")
        for name in ("direct_doorway_staging", "reverse_exit", "synchronized_view", "direct_press_alignment", "tangent_approach_correction"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        if not self.arms or len({p.side for p in self.arms}) != len(self.arms) or not self.panel_id:
            raise ValueError("specify unique arm profiles and the panel identity")
        for arm in self.arms:
            arm.validate()
        for label, offset in self.press_offsets.items():
            if not isinstance(label, str) or not label:
                raise ValueError("panel offset labels must be exact nonempty strings")
            vector(offset, 2)
        vector((self.viewing_carriage_m,), 1)
        for name in ("panel_standoff_m", "prepress_gap_m", "approach_step_m", "approach_budget_m",
                     "retract_gap_m", "stroke_m", "dwell_s", "segment_s", "position_tolerance_m",
                     "max_geometry_uncertainty_m", "max_observation_age_s", "max_search_rotation_rad"):
            positive(getattr(self, name), name)
        if self.stroke_m > 0.03 or self.dwell_s > 2 or self.segment_s > 10 or not 0 < self.min_score <= 1:
            raise ValueError("press settings exceed supported bounds")
        if self.prepress_gap_m <= self.max_geometry_uncertainty_m or self.retract_gap_m <= self.prepress_gap_m:
            raise ValueError("press and retract gaps must exceed measurement uncertainty")
        if self.coarse_panel_lateral_offset_m is not None:
            vector((self.coarse_panel_lateral_offset_m,), 1)
        if self.final_contact is not None:
            if not isinstance(self.final_contact, FinalContactPolicy):
                raise ValueError("final_contact must be an explicit FinalContactPolicy")
            self.final_contact.validate(self)
        vector(self.search_turns_rad, len(self.search_turns_rad))
        if sum(abs(x) for x in self.search_turns_rad) > self.max_search_rotation_rad:
            raise ValueError("search turns exceed their total angle budget")


@dataclass(frozen=True)
class ButtonObservation:
    label: str
    panel_id: str
    camera: str
    observation_id: str
    capture_us: int
    position: tuple
    normal: tuple
    uncertainty_m: float
    image_error: float
    scene_pose: tuple
    score: float
    receipt_id: str | None = None
    image_sha256: str | None = None
    surface_face: dict | None = None


@dataclass(frozen=True)
class ArmSelection:
    side: str | None
    outcome: str
    candidates: tuple
    observation: ButtonObservation | None = None
    carriage_target_m: float | None = None


class Elevator:
    def __init__(self, context, settings, *, cancelled=None, clock=time):
        settings.validate()
        self.context, self.settings = context, settings
        self.base = Base(context, cancelled=cancelled, clock=clock)
        self.evo = Evo(context)
        self.profiles = {p.side: p for p in settings.arms}
        self.evidence = []
        self._selected = None
        self._contact = None
        self._last_stop_us = 0
        self._guarded_attempted = False
        self._guarded_sequence_started = False
        if not self.base.capabilities.get("scene", {}).get("button_detection"):
            raise PrimitiveError("button detector is not configured")
        tools = {t["name"] for t in self.base.capabilities.get("tools", [])}
        if "evo_follow_path" not in tools:
            raise PrimitiveError("checked Cartesian paths are unavailable")
        if settings.final_contact is not None:
            self._guarded_require(capabilities=self.base.capabilities)

    def _guarded_eligible(self, capabilities=None):
        """Resolve only explicitly configured panel/camera qualifications."""
        policy = self.settings.final_contact
        if policy is None:
            return {}
        capabilities = self.context.value("evo_capabilities") if capabilities is None else capabilities
        scene = capabilities.get("scene", {})
        tools = {t["name"] for t in capabilities.get("tools", ())}
        records = scene.get("guarded_contact_profiles")
        if (scene.get("guarded_contact") is not True or "evo_guarded_contact" not in tools
                or not isinstance(records, list) or not records):
            raise PrimitiveError("guarded contact requires explicit runtime panel/camera qualification admission", self.evidence)
        triples = set()
        for record in records:
            if (not isinstance(record, dict) or set(record) != {"panel_id", "camera", "occlusion_qualification_id"}
                    or any(not isinstance(v, str) or not v for v in record.values())
                    or record["camera"] not in {"left_arm", "right_arm"}):
                raise PrimitiveError("guarded qualification advertisement is malformed", self.evidence)
            triple = (record["panel_id"], record["camera"], record["occlusion_qualification_id"])
            if triple in triples:
                raise PrimitiveError("guarded qualification advertisement has duplicate records", self.evidence)
            triples.add(triple)
        identity = policy.occlusion_qualification_id
        configured = identity if isinstance(identity, dict) else {side: identity for side in self.profiles}
        return {side: configured[side] for side in self.profiles if side in configured and
                (self.settings.panel_id, side+"_arm", configured[side]) in triples}

    def _guarded_require(self, sides=None, *, capabilities=None):
        eligible = self._guarded_eligible(capabilities)
        if not eligible or (sides is not None and any(side not in eligible for side in sides)):
            raise PrimitiveError("requested arm has no matching guarded panel/camera qualification", self.evidence)
        return eligible

    def _guarded_search_sides(self, arm):
        sides = tuple(self.profiles) if arm == "auto" else (self._profile(arm).side,)
        if self.settings.final_contact is not None:
            eligible = self._guarded_require(None if arm == "auto" else sides)
            sides = tuple(side for side in sides if side in eligible)
        return sides

    def _run(self, tool, *, guarded_sides=None, **args):
        self.base.check_cancel()
        callback = None
        if guarded_sides is not None and self.settings.final_contact is not None:
            self._guarded_require(guarded_sides)
            callback = lambda: self._guarded_require(guarded_sides)
        result = self.evo.execute(tool, cancelled=self.base.cancelled, before_dispatch=callback, **args)
        self._last_stop_us = time.time_ns()//1000
        self.evidence.append({"tool": tool, "action_id": result["action_id"],
                              "physical_stop_verified": result["physical_stop_verified"]})
        return result

    def _state(self):
        state = self.context.value("evo_status")
        if not state.get("feedback_fresh") or state.get("motion_blocked"):
            raise PrimitiveError("measured robot state is unavailable", self.evidence)
        return state["state"]

    def _profile(self, side):
        if side not in self.profiles:
            raise ValueError("requested arm has no configured profile")
        return self.profiles[side]

    def _palm(self, side, joints=None):
        args = {"links": [side+"_palm"]}
        if joints is not None:
            args["joints"] = joints
        pose = self.context.value("evo_fk", **args)["poses"][side+"_palm"]
        vector(pose["position"])
        rotate_vector((0, 0, 0), pose["quaternion_xyzw"])
        return pose

    def _tip(self, side, joints=None):
        pose = self._palm(side, joints)
        return add(vector(pose["position"]), rotate_vector(self._profile(side).tip_in_palm_m, pose["quaternion_xyzw"]))

    def _pose_for_tip(self, side, tip):
        profile = self._profile(side)
        offset = rotate_vector(profile.tip_in_palm_m, profile.press_quaternion_xyzw)
        return {"position": list(add(tip, scale(offset, -1))),
                "quaternion_xyzw": list(profile.press_quaternion_xyzw)}

    def _path(self, side, tip, *, duration_s=None):
        pose = self._pose_for_tip(side, vector(tip))
        self._run("evo_follow_path", guarded_sides=(side,), frame="base_link", waypoints=[{
            "duration_s": duration_s or self.settings.segment_s, side: pose}])
        measured = self._tip(side)
        if math.dist(measured, tip) > self.settings.position_tolerance_m:
            raise PrimitiveError("pressing tip did not reach the measured target", self.evidence)
        return measured

    def stow_arms(self):
        if self._contact is not None:
            raise PrimitiveError("retract before changing the travel posture", self.evidence)
        self._run("evo_move_joints", targets=dict(self.base.settings["travel_joints"]))
        measured = self._state()["positions"]
        for joint, target in self.base.settings["travel_joints"].items():
            if abs(measured[joint]-target) > self.base.capabilities["joint_limits"][joint]["tolerance"]:
                raise PrimitiveError("travel posture was not reached", self.evidence)
        self._selected = None
        return PrimitiveResult("stow_arms", "succeeded", tuple(self.evidence))

    def orient_base_to_panel(self, panel_hint=None):
        if self._contact is not None:
            raise PrimitiveError("retract the arm before a base turn", self.evidence)
        self.stow_arms()
        if panel_hint is not None:
            # A caller may supply a known bearing for coarse acquisition. It is
            # only a rotation request; the runtime still checks the swept path.
            self.base.rotate_base(vector((panel_hint,), 1)[0])
        turns = iter(self.settings.search_turns_rad)
        while self.base.observe_scene().get("panel") is None:
            try:
                angle = next(turns)
            except StopIteration:
                raise PrimitiveError("panel not found within the configured base search", self.evidence) from None
            self.base.rotate_base(angle)
        return self.base.align_base_angle()

    def prepare_button_view(self, arms=None):
        if self._contact is not None:
            raise PrimitiveError("retract before changing the viewing posture", self.evidence)
        sides = tuple(arms or self.profiles)
        if not sides or any(s not in self.profiles for s in sides):
            raise ValueError("unknown viewing arm")
        if self.settings.final_contact is not None:
            eligible = self._guarded_require(sides if arms is not None else None)
            sides = tuple(side for side in sides if side in eligible)
        if self.settings.synchronized_view:
            targets = {"carriage_joint": self.settings.viewing_carriage_m}
            for side in sides:
                targets.update(self._profile(side).view_joints)
            # One normal preview and action covers the complete joint path.
            # A refused preview or unverified stop must not trigger a fallback.
            self._run("evo_move_joints", guarded_sides=sides, targets=targets)
            measured = self._state()["positions"]["carriage_joint"]
            tolerance = self.base.capabilities["joint_limits"]["carriage_joint"]["tolerance"]
            if abs(measured-self.settings.viewing_carriage_m) > tolerance:
                raise PrimitiveError("viewing carriage height not reached", self.evidence)
            for side in sides:
                self._verify_view(side, self._profile(side).view_joints)
        else:
            self._run("evo_move_carriage", guarded_sides=sides, position_m=self.settings.viewing_carriage_m)
            for side in sides:
                profile = self._profile(side)
                self._run("evo_move_joints", guarded_sides=(side,), targets=dict(profile.view_joints))
                self._verify_view(side, profile.view_joints)
        return PrimitiveResult("prepare_button_view", "succeeded", tuple(self.evidence))

    def _verify_view(self, side, targets):
        state = self._state()["positions"]
        profile = self._profile(side)
        if abs(state[profile.elbow_joint]) < profile.minimum_elbow_bend_rad:
            raise PrimitiveError("measured elbow is not in a viewing posture", self.evidence)
        for joint, value in targets.items():
            if abs(state[joint]-value) > self.base.capabilities["joint_limits"][joint]["tolerance"]:
                raise PrimitiveError("viewing posture not reached", self.evidence)

    def _fresh_button(self, button):
        self.base.check_cancel()
        if button.panel_id != self.settings.panel_id or not 0 <= time.time()-button.capture_us/1e6 <= self.settings.max_observation_age_s:
            raise PrimitiveError("button observation is stale or belongs to another panel", self.evidence)
        if button.capture_us <= self._last_stop_us:
            raise PrimitiveError("button capture predates the last motion", self.evidence)
        scene = self.base.observe_scene()
        a, b = vector(button.scene_pose), vector(scene["pose"])
        if math.dist(a[:2], b[:2]) > self.settings.position_tolerance_m or abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2]))) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("base moved after button observation", self.evidence)

    def observe_button(self, label, side):
        profile = self._profile(side)
        if label not in self.settings.press_offsets:
            raise PrimitiveError("the panel has no calibrated press offset for this label")
        self.base.check_cancel()
        observation = self.context.value("rax_detect_buttons", camera=side+"_arm", score_threshold=self.settings.min_score)
        self.base.check_cancel()
        camera = observation.get("cameras", {}).get(side+"_arm", {})
        stamp = camera.get("capture_us")
        if type(stamp) is not int or stamp <= self._last_stop_us or not 0 <= time.time()-stamp/1e6 <= self.settings.max_observation_age_s:
            raise PrimitiveError("detector did not return a fresh post-motion wrist capture", self.evidence)
        if camera.get("geometry_validated") is not True:
            raise PrimitiveError("wrist-camera calibration is unavailable", self.evidence)
        scene = observation.get("scene", {})
        if scene.get("frame") != "base_link" or abs(scene.get("capture_us", 0)-stamp)/1e6 > self.base.settings["max_skew_s"]:
            raise PrimitiveError("button and point cloud are not from the same robot state", self.evidence)
        plane = scene.get("panel")
        if not plane:
            raise PrimitiveError("panel plane is unavailable", self.evidence)
        uncertainty = plane["uncertainty_m"]
        if not 0 < uncertainty <= self.settings.max_geometry_uncertainty_m:
            raise PrimitiveError("panel geometry is too uncertain", self.evidence)
        candidates = [d for d in observation.get("detections", []) if d.get("text") == label
                      and d.get("text_readable") is True and d.get("score", 0) >= self.settings.min_score]
        if len(candidates) != 1:
            return None  # Missing and ambiguous labels never become press targets.
        detection = candidates[0]
        x0, y0, x1, y1 = vector(detection["box"], 4)
        width, height = camera["width"], camera["height"]
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise PrimitiveError("invalid detected label box", self.evidence)
        du, dv = self.settings.press_offsets[label]
        pixel = ((x0+x1)/2+du*(x1-x0), (y0+y1)/2+dv*(y1-y0))
        ray = self.context.value("rax_camera_ray", observation_id=observation["observation_id"], camera=side+"_arm", pixel=list(pixel))
        if ray.get("frame") != "base_link" or ray.get("capture_us") != stamp:
            raise PrimitiveError("ray does not identify the detected capture", self.evidence)
        origin, direction, normal = vector(ray["origin"]), unit(ray["direction"]), unit(plane["normal"])
        denominator = dot(direction, normal)
        if denominator < 0.2:
            raise PrimitiveError("wrist view is too oblique to the panel", self.evidence)
        distance = dot(add(vector(plane["point"]), scale(origin, -1)), normal)/denominator
        if distance <= 0:
            raise PrimitiveError("button plane is behind the wrist camera", self.evidence)
        point = add(origin, scale(direction, distance))
        surface = ray.get("surface", {})
        if surface.get("capture_us") != scene["capture_us"] or surface.get("support_points", 0) < 6:
            raise PrimitiveError("press pixel has no synchronized local depth support", self.evidence)
        if math.dist(vector(surface["position"]), point) > self.settings.position_tolerance_m:
            raise PrimitiveError("local depth and the panel projection disagree", self.evidence)
        button = ButtonObservation(label, self.settings.panel_id, side+"_arm", observation["observation_id"], stamp,
                                   point, normal, uncertainty, math.hypot(pixel[0]/width-.5, pixel[1]/height-.5),
                                   vector(scene["pose"]), detection["score"],
                                   receipt_id=observation.get("model_receipt"),
                                   image_sha256=camera.get("image_sha256", observation.get("image_sha256")),
                                   surface_face=deepcopy(surface.get("face")))
        self.evidence.append({"observation_id": button.observation_id, "camera": button.camera,
                              "label": label, "calibration_id": profile.calibration_id, "point": point})
        return button

    def search_button(self, label, arm="auto"):
        sides = self._guarded_search_sides(arm)
        if label not in self.settings.press_offsets:
            raise PrimitiveError("unknown panel press offset")
        self.prepare_button_view(sides)
        turns = iter(self.settings.search_turns_rad)
        while True:
            stations = max((len(self.profiles[s].search_joints) for s in sides), default=0)
            for index in range(stations+1):
                # Finish all arm motion before taking either capture.
                for side in sides:
                    profile = self.profiles[side]
                    if index:
                        if index > len(profile.search_joints):
                            continue
                        posture = profile.search_joints[index-1]
                        self._run("evo_move_joints", guarded_sides=(side,), targets=dict(posture))
                        self._verify_view(side, posture)
                found = []
                for side in sides:
                    button = self.observe_button(label, side)
                    if button is not None:
                        found.append(button)
                if found:
                    return tuple(found)
            try:
                angle = next(turns)
            except StopIteration:
                raise PrimitiveError("button not found within the configured viewing search", self.evidence) from None
            self.stow_arms()
            self.base.rotate_base(angle)
            self.prepare_button_view(sides)

    def _carriage_target(self, button, side):
        state = self._state()["positions"]
        current = state["carriage_joint"]
        tip = self._tip(side)
        limits = self.base.capabilities["model"]["limits"]["carriage_joint"]
        probe = 0.005 if current+0.005 <= limits["upper"] else -0.005
        shifted = self._tip(side, {"carriage_joint": current+probe})
        derivative = (shifted[2]-tip[2])/probe
        if abs(derivative) < 0.5:
            raise PrimitiveError("carriage height direction is not established", self.evidence)
        target = current+(button.position[2]-tip[2])/derivative
        if not limits["lower"] <= target <= limits["upper"]:
            raise PrimitiveError("button is outside carriage travel", self.evidence)
        return target

    def select_press_arm(self, observations, preference="auto"):
        if preference != "auto":
            self._profile(preference)
        candidates, feasible = [], []
        eligible = self._guarded_eligible() if self.settings.final_contact is not None else self.profiles
        for side in self.profiles:
            if preference not in ("auto", side):
                continue
            if side not in eligible:
                candidates.append({"side": side, "feasible": False,
                                   "reason": "no matching guarded panel/camera qualification"})
                continue
            button = next((b for b in observations if b.camera == side+"_arm"), None)
            if button is None:
                candidates.append({"side": side, "feasible": False, "reason": "no readable wrist detection"})
                continue
            try:
                self._fresh_button(button)
                carriage = self._carriage_target(button, side)
                pre = add(button.position, scale(button.normal, -self.settings.prepress_gap_m))
                tip_targets = (pre,
                               add(button.position, scale(button.normal, self.settings.stroke_m)),
                               add(button.position, scale(button.normal, -self.settings.retract_gap_m)))
                preview = self.context.value("evo_preview", action={"kind": "follow_path", "frame": "base_link",
                    "waypoints": [{"duration_s": self.settings.segment_s,
                                   side: self._pose_for_tip(side, tip), "carriage_position_m": carriage}
                                  for tip in tip_targets]})
                if preview.get("valid") is not True:
                    raise PrimitiveError("approach preview refused")
                points = preview.get("plan", {}).get("points", [])
                joints = [p.get("positions", {}) for p in points]
                limits = self.base.capabilities["model"]["limits"]
                margins = [min(q-limits[n]["lower"], limits[n]["upper"]-q) for sample in joints for n, q in sample.items()
                           if n.startswith(side+"_joint") and n in limits]
                if not margins or min(margins) <= 0:
                    raise PrimitiveError("joint-limit margin is unavailable")
                travel = math.dist(self._tip(side), pre)
                candidate = {"side": side, "feasible": True, "joint_margin_rad": min(margins),
                             "tip_travel_m": travel, "image_error": button.image_error,
                             "carriage_target_m": carriage}
                candidates.append(candidate)
                feasible.append(((travel, -min(margins), button.image_error, side), button, carriage))
            except (ValueError, RuntimeError) as error:
                candidates.append({"side": side, "feasible": False, "reason": str(error)})
        if not feasible:
            return ArmSelection(None, "reposition_required", tuple(candidates))
        _, button, carriage = min(feasible, key=lambda x: x[0])
        side = button.camera.removesuffix("_arm")
        return ArmSelection(side, "selected", tuple(candidates), button, carriage)

    def _look(self, label, side):
        button = self.observe_button(label, side)
        if button is None:
            raise PrimitiveError("selected button was lost or became ambiguous", self.evidence)
        return button

    def prepare_press_arm(self, selection):
        if selection.side is None:
            raise PrimitiveError("no arm has a feasible approach", selection.candidates)
        if self._contact is not None:
            raise PrimitiveError("retract before selecting another arm", self.evidence)
        side = selection.side
        # Previews can take time. Acquire a new image before movement.
        button = self._look(selection.observation.label, side)
        if math.dist(button.position, selection.observation.position) > button.uncertainty_m+self.settings.position_tolerance_m:
            raise PrimitiveError("button changed after arm selection", self.evidence)
        self._fresh_button(button)
        target = self._preparation_tip(button, side)
        if self.settings.direct_press_alignment:
            # Keep the original gap/sweep check before any carriage motion.
            # Height correction still uses bounded steps and fresh observations.
            button = self.align_carriage_height(button.label, side)
            self._fresh_button(button)
            target = self._preparation_tip(button, side)
            gap = dot(add(button.position, scale(target, -1)), button.normal)
            target = add(button.position, scale(button.normal, -gap))
        self._path(side, target)
        self._selected = side
        return self._look(button.label, side)

    def _preparation_tip(self, button, side):
        # Bound the actual shortest-arc orientation sweep. An unchanged palm
        # orientation has no tool-offset sweep. Keep this check for both routes.
        tip = self._tip(side)
        current_q = self._palm(side)["quaternion_xyzw"]
        profile = self._profile(side)
        radius = math.sqrt(dot(profile.tip_in_palm_m, profile.tip_in_palm_m))
        cosine = min(1.0, abs(dot(current_q, profile.press_quaternion_xyzw)))
        sweep = 2*radius*math.sqrt(max(0.0, 1-cosine*cosine))
        if dot(add(button.position, scale(tip, -1)), button.normal) <= self.settings.prepress_gap_m+button.uncertainty_m+sweep:
            raise PrimitiveError("insufficient gap for press-posture preparation", self.evidence)
        return tip

    def align_carriage_height(self, label, side):
        if self._contact is not None:
            raise PrimitiveError("retract before moving the carriage", self.evidence)
        for _ in range(8):
            button = self._look(label, side)
            if abs(button.position[2]-self._tip(side)[2]) <= self.settings.position_tolerance_m:
                return button
            target = self._carriage_target(button, side)
            current = self._state()["positions"]["carriage_joint"]
            target = current+max(-0.04, min(0.04, target-current))
            self._run("evo_move_carriage", guarded_sides=(side,), position_m=target)
        raise PrimitiveError("carriage height did not converge", self.evidence)

    def align_arm_to_button(self, label, side):
        button = self.align_carriage_height(label, side)
        tip = self._tip(side)
        gap = dot(add(button.position, scale(tip, -1)), button.normal)
        if gap <= self.settings.prepress_gap_m+button.uncertainty_m:
            raise PrimitiveError("insufficient gap for aim correction", self.evidence)
        target = add(button.position, scale(button.normal, -gap))
        self._path(side, target)
        return self._look(label, side)

    def _correct_approach_aim(self, button, side, remaining_budget_m):
        """Correct tangential image drift without moving carriage or going inward."""
        if not self.settings.tangent_approach_correction:
            raise PrimitiveError("tangential approach correction is not enabled", self.evidence)
        contact=self._contact_state(side)
        if contact.get("stage")!="approaching" or contact.get("label")!=button.label:
            raise PrimitiveError("aim correction requires the same unpressed approach", self.evidence)
        self._fresh_button(button)
        if math.dist(contact["normal"],button.normal)>self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("panel normal changed during approach", self.evidence)
        # Preserve the existing tool-offset orientation sweep and gap check.
        before=self._preparation_tip(button,side)
        gap=dot(add(button.position,scale(before,-1)),button.normal)
        target=add(button.position,scale(button.normal,-gap))
        distance=math.dist(before,target)
        if distance>min(self.settings.approach_step_m,remaining_budget_m):
            raise PrimitiveError("approach aim correction exceeds the existing step or travel budget", self.evidence)
        self._path(side,target)
        travelled=math.dist(before,self._tip(side))
        self.evidence.append(dict(stage="tangent_approach_correction",gap_m=gap,
                                  requested_m=distance,measured_travel_m=travelled,carriage_held=True))
        # Keep the recovery reference until this checked arm path succeeds.
        self._contact={**contact,"point":button.position,"normal":button.normal,
                       "pose":button.scene_pose}
        return travelled

    def approach_button(self, label, side, standoff_m=None):
        if self._selected != side:
            raise PrimitiveError("prepare the selected arm before approach")
        gap_target = self.settings.prepress_gap_m if standoff_m is None else positive(standoff_m, "standoff_m")
        travelled = 0.0
        for _ in range(40):
            button = self._look(label, side)
            tip = self._tip(side)
            gap = dot(add(button.position, scale(tip, -1)), button.normal)
            if gap < gap_target-button.uncertainty_m:
                raise PrimitiveError("arm is closer to the panel than the approach target", self.evidence)
            lateral = add(add(button.position, scale(tip, -1)), scale(button.normal, -gap))
            if math.sqrt(dot(lateral, lateral)) > self.settings.position_tolerance_m:
                if self._contact is not None and self.settings.tangent_approach_correction:
                    travelled+=self._correct_approach_aim(button,side,self.settings.approach_budget_m-travelled)
                    if travelled>self.settings.approach_budget_m:
                        raise PrimitiveError("arm approach travel budget exceeded", self.evidence)
                else:
                    self.align_arm_to_button(label, side)
                continue
            # Target gap includes uncertainty. No minimum inward step can overshoot it.
            remaining = gap-gap_target-button.uncertainty_m
            if remaining <= self.settings.position_tolerance_m:
                self._contact = {"side": side, "label": label, "point": button.position,
                                 "normal": button.normal, "pose": button.scene_pose,
                                 "carriage": self._state()["positions"]["carriage_joint"], "stage": "prepress"}
                return button
            step = min(self.settings.approach_step_m, remaining)
            if travelled+step > self.settings.approach_budget_m:
                raise PrimitiveError("arm approach travel budget exceeded", self.evidence)
            before = tip
            # Preserve a retraction reference if an inward action fails part way.
            self._contact = {"side": side, "label": label, "point": button.position,
                             "normal": button.normal, "pose": button.scene_pose,
                             "carriage": self._state()["positions"]["carriage_joint"], "stage": "approaching"}
            self._path(side, add(tip, scale(button.normal, step)))
            travelled += math.dist(before, self._tip(side))
        raise PrimitiveError("button approach did not converge", self.evidence)

    def _contact_state(self, side):
        contact = self._contact
        if contact is None or contact["side"] != side:
            raise PrimitiveError("no verified approach exists for this arm")
        state = self._state()
        pose = vector(self.base.observe_scene()["pose"])
        if math.dist(pose[:2], contact["pose"][:2]) > self.settings.position_tolerance_m or abs(math.atan2(math.sin(pose[2]-contact["pose"][2]), math.cos(pose[2]-contact["pose"][2]))) > self.base.settings["angle_tolerance_rad"]:
            raise PrimitiveError("base moved during contact stage", self.evidence)
        if abs(state["positions"]["carriage_joint"]-contact["carriage"]) > self.settings.position_tolerance_m:
            raise PrimitiveError("carriage moved during contact stage", self.evidence)
        return contact

    def press_button(self, label, side, *, stroke_m=None, dwell_s=None):
        if self.settings.final_contact is not None:
            raise PrimitiveError("configured guarded contact must use its single transaction")
        contact = self._contact_state(side)
        if contact["label"] != label or contact["stage"] != "prepress":
            raise PrimitiveError("press must follow a fresh approach and cannot be repeated automatically")
        stroke = self.settings.stroke_m if stroke_m is None else positive(stroke_m, "stroke_m")
        dwell = self.settings.dwell_s if dwell_s is None else positive(dwell_s, "dwell_s")
        if stroke > self.settings.stroke_m or dwell > self.settings.dwell_s:
            raise ValueError("press exceeds configured stroke or dwell")
        button = self._look(label, side)
        self._fresh_button(button)
        if math.dist(button.position, contact["point"]) > button.uncertainty_m+self.settings.position_tolerance_m:
            raise PrimitiveError("button geometry changed before pressing")
        # The requested stroke is beyond the measured button surface. The total
        # movement also includes the verified positive approach gap.
        target = add(button.position, scale(button.normal, stroke))
        contact["stage"] = "press_attempted"  # Never retry after an uncertain action.
        self._path(side, target)
        until = self.base.clock.monotonic()+dwell
        while self.base.clock.monotonic() < until:
            self.base.check_cancel()
            self._contact_state(side)
            self.base.clock.sleep(min(0.05, max(0, until-self.base.clock.monotonic())))
        return PrimitiveResult("press_button", "stroke_complete_unconfirmed", tuple(self.evidence))

    def retract_arm(self, side, clearance_m=None):
        if self._contact is not None and self._contact.get("guarded"):
            raise PrimitiveError("guarded contact requires explicit verified recovery; generic retraction cannot clear it")
        contact = self._contact_state(side)
        gap = self.settings.retract_gap_m if clearance_m is None else positive(clearance_m, "clearance_m")
        if gap < self.settings.retract_gap_m:
            raise ValueError("retraction cannot reduce the configured clearance")
        tip = self._tip(side)
        current_gap = dot(add(contact["point"], scale(tip, -1)), contact["normal"])
        target = add(tip, scale(contact["normal"], -max(0, gap-current_gap)))
        self._path(side, target, duration_s=self.settings.retract_segment_s)
        self._contact = None
        return PrimitiveResult("retract_arm", "succeeded", tuple(self.evidence), {"clearance_m": gap})

    def verify_button_activation(self, label, side):
        # The shipped detector has no activation field. Preserve this explicit
        # outcome until a measured light-state or elevator signal is integrated.
        button = self.observe_button(label, side)
        return PrimitiveResult("verify_button_activation", "unknown", tuple(self.evidence),
                               {"button_visible": button is not None})

    def _finite_contact_target(self, button):
        face = button.surface_face
        if not isinstance(face, dict) or not isinstance(face.get("source"), str) or not face["source"]:
            raise PrimitiveError("guarded contact requires a finite qualified panel face", self.evidence)
        center = vector(face.get("center"))
        horizontal, vertical = (unit(face.get(name)) for name in ("horizontal_axis", "vertical_axis"))
        if max(abs(dot(horizontal, vertical)), abs(dot(horizontal, button.normal)),
               abs(dot(vertical, button.normal))) > 1e-6:
            raise PrimitiveError("guarded panel axes are not orthogonal", self.evidence)
        offset = add(button.position, scale(center, -1))
        if abs(dot(offset, button.normal)) > button.uncertainty_m:
            raise PrimitiveError("guarded target is not on the captured panel face", self.evidence)
        for axis, size in ((horizontal, "width_m"), (vertical, "height_m")):
            if abs(dot(offset, axis))+button.uncertainty_m > positive(face.get(size), size)/2:
                raise PrimitiveError("guarded target uncertainty extends outside the finite panel", self.evidence)
        identity = button.receipt_id
        image_hash = button.image_sha256
        valid_hash = (isinstance(image_hash, str) and len(image_hash) == 64
                      and all(c in "0123456789abcdef" for c in image_hash))
        if not (isinstance(identity, str) and identity or valid_hash):
            raise PrimitiveError("guarded target lacks an immutable image or receipt identity", self.evidence)
        if image_hash is not None and not valid_hash:
            raise PrimitiveError("guarded target image hash is invalid", self.evidence)

    def guarded_press_button(self, selection):
        """Use one fresh target for an explicitly guarded occluded transaction."""
        if self.settings.final_contact is None:
            raise PrimitiveError("guarded contact is disabled", self.evidence)
        if self._guarded_sequence_started or self._guarded_attempted or self._contact is not None:
            raise PrimitiveError("guarded contact cannot repeat or replace an uncertain contact", self.evidence)
        if selection.side is None:
            raise PrimitiveError("guarded contact requires a selected feasible arm", self.evidence)
        self._guarded_require((selection.side,))
        self._guarded_sequence_started = True
        record = {"stage": "guarded_contact", "state": "preparing", "side": selection.side,
                  "label": selection.observation.label}
        self.evidence.append(record)
        try:
            return self._guarded_press_once(selection, record)
        except BaseException as error:
            record.update(state="failed", reason=str(error))
            if isinstance(error, EvoActionFailed):
                record["result"] = deepcopy(error.result)
            if self._contact is None:
                self._contact = {"side": selection.side, "label": selection.observation.label,
                                 "stage": "guarded_preparation_failed", "guarded": True}
            else:
                self._contact["guarded"] = True
            raise

    def _guarded_press_once(self, selection, record):
        policy = self.settings.final_contact
        if policy is None:
            raise PrimitiveError("guarded contact is disabled", self.evidence)
        if self._guarded_attempted or self._contact is not None:
            raise PrimitiveError("guarded contact cannot repeat or replace an uncertain contact", self.evidence)
        if selection.side is None:
            raise PrimitiveError("guarded contact requires a selected feasible arm", self.evidence)
        side, label = selection.side, selection.observation.label
        profile = self._profile(side)
        button = self._look(label, side)
        self._fresh_button(button)
        self._finite_contact_target(button)
        if math.dist(button.position, selection.observation.position) > button.uncertainty_m+self.settings.position_tolerance_m:
            raise PrimitiveError("button changed after arm selection", self.evidence)
        tip = self._preparation_tip(button, side)
        if dot(add(button.position, scale(tip, -1)), button.normal) < policy.visible_gap_m+button.uncertainty_m:
            raise PrimitiveError("guarded preparation starts inside the qualified visible gap", self.evidence)
        # Rotate at the current tip/gap. Never use the old preparation route's
        # inward aim. Finish all carriage and tangential alignment while visible.
        self._path(side, tip)
        button = self.align_carriage_height(label, side)
        tip = self._tip(side)
        gap = dot(add(button.position, scale(tip, -1)), button.normal)
        if gap < policy.visible_gap_m+button.uncertainty_m:
            raise PrimitiveError("guarded alignment lost its qualified visible gap", self.evidence)
        if gap+self.settings.stroke_m > self.settings.approach_budget_m:
            raise PrimitiveError("visible approach and guarded stroke exceed the existing approach budget", self.evidence)
        self._path(side, add(button.position, scale(button.normal, -gap)))
        self._selected = side
        button = self.approach_button(label, side, standoff_m=policy.visible_gap_m)
        self._finite_contact_target(button)
        self._fresh_button(button)
        point, normal = button.position, button.normal
        gap = dot(add(point, scale(self._tip(side), -1)), normal)
        if gap < policy.visible_gap_m or gap+self.settings.stroke_m > policy.max_inward_travel_m:
            raise PrimitiveError("guarded final motion exceeds its visible gap or travel budget", self.evidence)
        carriage = vector((self._state()["positions"]["carriage_joint"],), 1)[0]
        args = dict(frame="base_link", side=side, observation=dict(
            observation_id=button.observation_id, camera=button.camera, capture_us=button.capture_us,
            panel_id=button.panel_id, label=label, receipt_id=button.receipt_id,
            image_sha256=button.image_sha256, position=list(point), normal=list(normal),
            uncertainty_m=button.uncertainty_m, scene_pose=list(button.scene_pose),
            carriage_position_m=carriage, surface_face=deepcopy(button.surface_face)),
            tip_in_palm_m=list(profile.tip_in_palm_m), press_quaternion_xyzw=list(profile.press_quaternion_xyzw),
            occlusion_qualification_id=self._guarded_require((side,))[side],
            visible_gap_m=policy.visible_gap_m, max_inward_travel_m=policy.max_inward_travel_m,
            prepress_gap_m=self.settings.prepress_gap_m, approach_step_m=self.settings.approach_step_m,
            segment_s=self.settings.segment_s, stroke_m=self.settings.stroke_m, dwell_s=self.settings.dwell_s,
            retract_gap_m=max(self.settings.retract_gap_m, policy.visible_gap_m),
            position_tolerance_m=min(self.settings.position_tolerance_m, .004),
            angle_tolerance_rad=min(self.base.settings["angle_tolerance_rad"], .01))
        record.update(state="prepared", request=deepcopy(args))
        self._contact.update(stage="guarded_prepared", guarded=True)

        def before_dispatch():
            if self._guarded_require((side,))[side] != args["occlusion_qualification_id"]:
                raise PrimitiveError("guarded qualification changed before dispatch", self.evidence)
            self._fresh_button(button)  # Original stamp, after potentially slow preview.
            self._guarded_fixed_state(args)
            # Recheck age after the state/FK queries as well; do not retimestamp.
            if not 0 <= time.time()-button.capture_us/1e6 <= self.settings.max_observation_age_s:
                raise PrimitiveError("guarded target expired before dispatch", self.evidence)
            self._guarded_attempted = True
            self._contact["stage"] = "guarded_attempted"
            record.update(state="attempted", dispatch_wall_us=time.time_ns()//1000)

        try:
            result = self.evo.execute("evo_guarded_contact", cancelled=self.base.cancelled,
                                      before_dispatch=before_dispatch, **deepcopy(args))
            self._last_stop_us = time.time_ns()//1000
            record["result"] = deepcopy(result)
            receipt = result.get("guarded_contact", {})
            if (receipt.get("observation_id") != button.observation_id or receipt.get("capture_us") != button.capture_us
                    or receipt.get("side") != side or receipt.get("target") != label
                    or any(receipt.get(name) is not True for name in (
                        "prepress_stop_verified", "press_stop_verified", "withdrawal_stop_verified",
                        "source_limits_verified", "fixed_geometry_verified", "single_correct_press", "released"))):
                raise PrimitiveError("guarded contact result lacks matching measured completion", self.evidence)
            self._verify_guarded_withdrawal(args)
            observed = self._look(label, side)
            self._fresh_button(observed)
            self._finite_contact_target(observed)
            if (math.dist(observed.position, point) > observed.uncertainty_m+args["position_tolerance_m"]
                    or math.acos(max(-1., min(1., dot(observed.normal, normal)))) > args["angle_tolerance_rad"]):
                raise PrimitiveError("postwithdraw image does not confirm the frozen target", self.evidence)
            self._verify_guarded_withdrawal(args)
            if not 0 <= time.time()-observed.capture_us/1e6 <= self.settings.max_observation_age_s:
                raise PrimitiveError("postwithdraw image expired during final verification", self.evidence)
            record.update(state="verified", postwithdraw_observation_id=observed.observation_id,
                          postwithdraw_capture_us=observed.capture_us)
            self._contact = None
            return PrimitiveResult("guarded_press_button", "succeeded", tuple(self.evidence),
                                   {"arm": side, "activation": "succeeded", "guarded_contact": receipt})
        except BaseException as error:
            record.update(state="failed", reason=str(error))
            if isinstance(error, EvoActionFailed):
                record["result"] = deepcopy(error.result)
            # An attempted or refused transaction stays visible in contact state.
            # No automatic recovery, second press, or stow follows this failure.
            raise

    def _guarded_fixed_state(self, args):
        state = self._state()["positions"]
        carriage = vector((state["carriage_joint"],), 1)[0]
        scene = self.base.observe_scene()
        pose, captured = vector(scene["pose"]), args["observation"]["scene_pose"]
        yaw = math.atan2(math.sin(pose[2]-captured[2]), math.cos(pose[2]-captured[2]))
        if (math.dist(pose[:2], captured[:2]) > args["position_tolerance_m"]
                or abs(yaw) > args["angle_tolerance_rad"]
                or abs(carriage-args["observation"]["carriage_position_m"]) > args["position_tolerance_m"]):
            raise PrimitiveError("base or carriage moved during guarded contact", self.evidence)
        panel = scene.get("panel", {})
        if panel.get("id") != self.settings.panel_id:
            raise PrimitiveError("guarded panel identity changed", self.evidence)
        face, captured_face = panel.get("face"), args["observation"]["surface_face"]
        uncertainty = args["observation"]["uncertainty_m"]
        if (not isinstance(face, dict) or face.get("source") != captured_face["source"]
                or any(abs(positive(face.get(size), size)-captured_face[size]) > uncertainty
                       for size in ("width_m", "height_m"))
                or math.dist(vector(face.get("center")), captured_face["center"]) > uncertainty+args["position_tolerance_m"]
                or any(math.acos(max(-1., min(1., dot(unit(face.get(axis)), unit(captured_face[axis])))))
                       > args["angle_tolerance_rad"] for axis in ("horizontal_axis", "vertical_axis"))):
            raise PrimitiveError("guarded finite panel geometry changed", self.evidence)

    def _verify_guarded_withdrawal(self, args):
        self._guarded_fixed_state(args)
        palm = self._palm(args["side"])
        tip = add(vector(palm["position"]), rotate_vector(args["tip_in_palm_m"], palm["quaternion_xyzw"]))
        expected = add(args["observation"]["position"],
                       scale(args["observation"]["normal"], -args["retract_gap_m"]))
        if math.dist(tip, expected) > args["position_tolerance_m"]:
            raise PrimitiveError("guarded withdrawal did not reach the visible endpoint", self.evidence)
        angle = 2*math.acos(min(1., abs(dot(vector(palm["quaternion_xyzw"], 4), args["press_quaternion_xyzw"]))))
        if angle > args["angle_tolerance_rad"]:
            raise PrimitiveError("guarded withdrawal orientation was not reached", self.evidence)

    def _coarse_panel_stage(self):
        offset = self.settings.coarse_panel_lateral_offset_m
        if offset is None:
            return
        if self._contact is not None:
            raise PrimitiveError("retract before coarse panel staging", self.evidence)
        state = self._state()["positions"]
        if any(abs(vector((state[n],), 1)[0]-v) > self.base.capabilities["joint_limits"][n]["tolerance"]
               for n, v in self.base.settings["travel_joints"].items()):
            raise PrimitiveError("coarse panel staging requires compact measured arms", self.evidence)
        scene, panel, point, normal = self.base.target()
        if panel.get("id") != self.settings.panel_id or not isinstance(panel.get("face"), dict):
            raise PrimitiveError("coarse staging requires identified finite panel geometry", self.evidence)
        face = panel["face"]
        vector(face.get("center"))
        positive(face.get("width_m"), "panel width")
        positive(face.get("height_m"), "panel height")
        tangent = (-normal[1], normal[0], 0.)
        desired = dot(point, tangent)-offset
        if abs(desired) > self.base.settings["max_step_m"]:
            raise PrimitiveError("coarse panel staging exceeds the existing base step bound", self.evidence)
        result = self.base.translate_base(desired*tangent[0], desired*tangent[1])
        self._last_stop_us = time.time_ns()//1000
        self.evidence.append({"stage": "coarse_panel_staging", "offset_m": offset,
                              "source": "configured panel frame; no observed label or button center",
                              "base_result": result.measured})

    def press_elevator_button(self, label, arm="auto", *, panel_bearing_rad=None):
        if label not in self.settings.press_offsets:
            raise PrimitiveError("unknown calibrated press point")
        if self.settings.final_contact is not None:
            self._guarded_search_sides(arm)  # Refuse unmatched requests before any task motion.
        self.orient_base_to_panel(panel_bearing_rad)
        self.base.align_base_distance(self.settings.panel_standoff_m)
        self._coarse_panel_stage()
        observations = self.search_button(label, arm)
        selection = self.select_press_arm(observations, arm)
        if selection.side is None:
            raise PrimitiveError("base or arm repositioning is required", selection.candidates)
        side = selection.side
        # Lateral base adjustment requires compact arms, then fresh wrist search.
        offset = self.profiles[side].lateral_offset_m
        scene, _, point, normal = self.base.target()
        desired = dot(selection.observation.position, (-normal[1], normal[0], 0))-offset
        if self.settings.coarse_panel_lateral_offset_m is None and abs(desired) > self.base.settings["position_tolerance_m"]:
            self.stow_arms()
            tangent = (-normal[1], normal[0], 0)
            self.base.translate_base(desired*tangent[0], desired*tangent[1])
            observations = self.search_button(label, side)
            selection = self.select_press_arm(observations, side)
        if self.settings.final_contact is not None:
            activation = self.guarded_press_button(selection)
            self.stow_arms()
            return PrimitiveResult("press_elevator_button", "succeeded", tuple(self.evidence+self.base.evidence),
                                   {"arm": side, "activation": activation.measured["activation"],
                                    "guarded_contact": activation.measured["guarded_contact"]})
        self.prepare_press_arm(selection)
        if not self.settings.direct_press_alignment:
            self.align_arm_to_button(label, side)
        self.approach_button(label, side)
        self.press_button(label, side)
        self.retract_arm(side)
        activation = self.verify_button_activation(label, side)
        self.stow_arms()
        return PrimitiveResult("press_elevator_button",
                               "succeeded" if activation.outcome == "succeeded" else "stroke_complete_unconfirmed",
                               tuple(self.evidence+self.base.evidence), {"arm": side, "activation": activation.outcome})

    def press_with_deadline(self, label, arm="auto", *, button_timeout_s=20.):
        """Press, release and stow under the full button-action deadline.

        The deadline is checked after the complete action. A blocking adapter
        does not provide in-flight cancellation through this check.
        """
        timeout=positive(button_timeout_s,"button_timeout_s")
        if timeout>20.:raise ValueError("Full button action must remain under 20 seconds")
        start=self.base.clock.monotonic()
        if not math.isfinite(start):raise PrimitiveError("Invalid button start clock",self.evidence)
        record=dict(stage="hall_button_deadline",start_s=start,limit_s=timeout,passed=False)
        self.evidence.append(record)
        try:
            button=self.press_elevator_button(label,arm)
            if button.outcome!="succeeded" or button.measured.get("activation")!="succeeded":
                raise PrimitiveError("Hall activation, release and stow are unconfirmed",self.evidence)
        except BaseException as error:
            record["error"]=repr(error)
            raise
        finally:
            end=self.base.clock.monotonic()
            duration=end-start
            valid=math.isfinite(duration) and 0<=duration<timeout
            record.update(end_s=end if math.isfinite(end) else repr(end),
                          duration_s=duration if math.isfinite(duration) else repr(duration),
                          passed=valid and "error" not in record)
        if not record["passed"]:
            raise PrimitiveError("Full hall button action exceeded its deadline or clock is invalid",self.evidence)
        return PrimitiveResult("press_with_deadline","succeeded",tuple(self.evidence+self.base.evidence),
                               {**button.measured,"button_timing":dict(record)})

    def press_and_enter(self, label, arm="auto", *, doorway_id=None,
                        goal_depth_m=.9, button_timeout_s=20.):
        """Use the checked button action, then the normal fresh-door entry."""
        positive(goal_depth_m,"goal_depth_m")
        if goal_depth_m<=self.base.settings["footprint"][0]+self.base.settings["margin_m"]:
            raise ValueError("Entry goal must clear the complete robot before pressing")
        button=self.press_with_deadline(label,arm,button_timeout_s=button_timeout_s)
        entry=self.navigate_into_elevator(doorway_id,goal_depth_m=goal_depth_m)
        if entry.outcome!="succeeded":
            raise PrimitiveError("Timed entry was not confirmed",self.evidence)
        return PrimitiveResult("press_and_enter","succeeded",tuple(self.evidence+self.base.evidence),
                               {"button":button.measured,"button_timing":button.measured["button_timing"],"entry":entry.measured})

    def observe_doorway(self):
        return self.base.observe_scene().get("doorway")

    def wait_for_door_open(self, doorway_id=None, timeout_s=30):
        return self.base.wait_for_door_open(doorway_id, timeout_s)

    def align_to_doorway(self, doorway_id=None, *, staging_gap_m):
        self.stow_arms()
        return self.base.align_to_doorway(doorway_id, staging_gap_m=staging_gap_m, direct=self.settings.direct_doorway_staging)

    def _traverse(self, direction, doorway_id, goal_depth_m):
        if self._contact is not None:
            raise PrimitiveError("retract before navigation")
        positive(goal_depth_m, "goal_depth_m")
        extent = self.base.settings["footprint"][0]+self.base.settings["margin_m"]
        if goal_depth_m <= extent:
            raise ValueError("goal must put the complete robot beyond the doorway")
        self.stow_arms()
        door = self.base.wait_for_door_open(doorway_id)
        door = self.base.align_to_doorway(door["id"], staging_gap_m=extent+0.15,
                                          direct=self.settings.direct_doorway_staging,
                                          reverse=direction == "out_of" and self.settings.reverse_exit)
        _, door, center, normal = self.base.target("doorway", door["id"])
        goal = add(center, scale(normal, goal_depth_m))
        result = self.base.translate_base(goal[0], goal[1], doorway_id=door["id"])
        if result.measured.get("crossing_state") != "after":
            raise PrimitiveError("full robot clearance of the threshold is unconfirmed", self.base.evidence)
        return PrimitiveResult("navigate_"+direction+"_elevator", "succeeded",
                               tuple(self.evidence+self.base.evidence),
                               {"location": "inside" if direction == "into" else "outside", **result.measured})

    def navigate_into_elevator(self, doorway_id=None, *, goal_depth_m):
        return self._traverse("into", doorway_id, goal_depth_m)

    def navigate_out_of_elevator(self, doorway_id=None, *, goal_depth_m):
        return self._traverse("out_of", doorway_id, goal_depth_m)
