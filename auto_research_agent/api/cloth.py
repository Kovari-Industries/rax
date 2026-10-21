"""Measured cloth routines built from the public Evo tools.

A perception callable receives (saved Evo observation, Check) and returns Scene.
It can use the agent's vision, calibrated stereo, or a task-specific estimator.
Unknown cloth state is a normal outcome; robot completion never proves cloth success.
"""
from dataclasses import dataclass, field, replace
import math
import time
from .evo import Evo


def _vector(value, size=3):
    value = tuple(value)
    if len(value) != size or any(
        type(v) not in (int, float) or not math.isfinite(v) for v in value
    ):
        raise ValueError(f"expected {size} finite coordinates")
    return value


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _scale(a, factor):
    return tuple(x * factor for x in a)


def _norm(a):
    return math.sqrt(sum(x * x for x in a))


def _unit(a):
    length = _norm(a)
    if length < 1e-8:
        raise ValueError("direction has zero length")
    return _scale(a, 1 / length)


def _mean(points):
    return tuple(sum(p[i] for p in points) / len(points) for i in range(3))


def _pose(position, quaternion):
    position, quaternion = _vector(position), _vector(quaternion, 4)
    if abs(_norm(quaternion) - 1) > 1e-3:
        raise ValueError("orientation must be a unit xyzw quaternion")
    return {"position": list(position), "quaternion_xyzw": list(quaternion)}


def _yaw_pose(pose, origin, target, yaw, dz):
    x, y, z = _sub(pose["position"], origin)
    c, s = math.cos(yaw), math.sin(yaw)
    qx, qy, qz, qw = pose["quaternion_xyzw"]
    a, b = math.sin(yaw / 2), math.cos(yaw / 2)
    return _pose(
        (target[0] + c * x - s * y, target[1] + s * x + c * y, origin[2] + z + dz),
        (b * qx - a * qy, a * qx + b * qy, b * qz + a * qw, b * qw - a * qz),
    )


@dataclass(frozen=True)
class Grasp:
    ident: str
    position: tuple
    quaternion_xyzw: tuple
    feature: str = "surface"
    uncertainty_m: float = 0.0
    score: float = 1.0
    feature_position: tuple | None = None  # cloth point; position is the palm target

    def point(self):
        return _vector(
            self.feature_position
            if self.feature_position is not None
            else self.position
        )

    def pose(self):
        if not self.ident or self.feature not in {
            "surface",
            "edge",
            "corner",
            "packet",
        }:
            raise ValueError("invalid grasp identity or feature")
        if not 0 <= self.uncertainty_m <= 0.01 or not 0 <= self.score <= 1:
            raise ValueError("grasp uncertainty or score outside supported range")
        self.point()
        return _pose(self.position, self.quaternion_xyzw)


@dataclass(frozen=True)
class Pile:
    """Freshly measured top surface and available rectangular placement region."""

    center: tuple
    size_xy: tuple
    yaw_rad: float = 0.0
    uncertainty_m: float = 0.0

    def validate(self):
        _vector(self.center)
        if any(v <= 0 for v in _vector(self.size_xy, 2)) or not math.isfinite(
            self.yaw_rad
        ):
            raise ValueError("invalid pile footprint")
        if not 0 <= self.uncertainty_m <= 0.01:
            raise ValueError("pile height is too uncertain")


@dataclass(frozen=True)
class Scene:
    observation_id: str
    state: str  # crumpled, spread, folded, stacked, unknown
    geometry_source: str  # calibrated_triangulation, measured_surface, simulation
    grasps: tuple = ()
    corners: tuple = ()  # ordered boundary corners for a visible flat rectangle
    held: dict = field(default_factory=dict)  # per hand: verified, lost, uncertain
    layers: dict = field(default_factory=dict)  # per hand: single, packet, unknown
    stable: bool = False
    supported: bool = False
    surface_z: float | None = None
    packet_bottom_z: float | None = None
    pile: Pile | None = None
    alignment_error_m: float | None = None
    note: str = ""

    @classmethod
    def from_dict(cls, value):
        """Decode the metric result of an agent or task perception program."""
        fields = dict(value)
        fields["grasps"] = tuple(Grasp(**g) for g in fields.get("grasps", ()))
        fields["corners"] = tuple(tuple(p) for p in fields.get("corners", ()))
        if fields.get("pile") is not None:
            fields["pile"] = Pile(**fields["pile"])
        return cls(**fields)


@dataclass(frozen=True)
class Check:
    stage: str
    hands: tuple = ()
    expected: str = ""


class ClothUncertain(RuntimeError):
    """No further motion is issued. The caller can inspect evidence and recover."""

    def __init__(self, reason, evidence=()):
        super().__init__(reason)
        self.evidence = list(evidence)


@dataclass(frozen=True)
class ClothSettings:
    open_rad: dict
    close_rad: dict
    approach_clearance_m: float = 0.06
    test_lift_m: float = 0.03
    inspection_lift_m: float = 0.12
    fold_clearance_m: float = 0.12
    transport_clearance_m: float = 0.15
    release_gap_m: float = 0.01
    max_span_m: float = 0.7
    segment_s: float = 3.0
    max_observation_age_s: float = 5.0
    max_unfold_attempts: int = 4
    alignment_tolerance_m: float = 0.025

    def validate(self):
        for positions in (self.open_rad, self.close_rad):
            if set(positions) != {"left", "right"}:
                raise ValueError("gripper angles must specify both hands")
            _vector(positions.values(), 2)
        for value in (
            self.approach_clearance_m,
            self.test_lift_m,
            self.inspection_lift_m,
            self.fold_clearance_m,
            self.transport_clearance_m,
            self.release_gap_m,
            self.max_span_m,
            self.segment_s,
            self.max_observation_age_s,
            self.alignment_tolerance_m,
        ):
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError("cloth settings must be finite and positive")
        if (
            not isinstance(self.max_unfold_attempts, int)
            or isinstance(self.max_unfold_attempts, bool)
        ) or not 1 <= self.max_unfold_attempts <= 8:
            raise ValueError("unfold attempts must be bounded")
        if self.segment_s > 6:
            raise ValueError("segment_s exceeds the bounded multi-segment path budget")


class Cloth:
    """Routines for towels and sheets. No controller, ROS, or camera SDK access."""

    def __init__(self, context, perceive, settings):
        settings.validate()
        self.context = context
        self.evo = Evo(context)
        self.perceive = perceive
        self.settings = settings
        self.evidence = []
        self._last_scene = None
        self._last_capture_us = 0
        self._base = None
        self._simulated = bool(context.value("evo_capabilities")["simulated"])

    def _uncertain(self, reason):
        raise ClothUncertain(reason, self.evidence)

    def observe(self, stage, hands=(), expected=""):
        self._check_base()
        observation = self.context.value("evo_observe")
        cameras = observation.get("cameras", {})
        if not cameras:
            self._uncertain("no camera observation")
        stamp = min(c["capture_us"] for c in cameras.values())
        scene = self.perceive(observation, Check(stage, tuple(hands), expected))
        if (
            not isinstance(scene, Scene)
            or scene.observation_id != observation["observation_id"]
        ):
            self._uncertain("perception must identify the current observation")
        if scene.state not in {"crumpled", "spread", "folded", "stacked", "unknown"}:
            self._uncertain("unknown cloth state label")
        if scene.geometry_source not in {
            "calibrated_triangulation",
            "measured_surface",
            "simulation",
        }:
            self._uncertain("metric cloth geometry has no supported source")
        if scene.geometry_source == "simulation" and not self._simulated:
            self._uncertain("simulation geometry is forbidden on a physical robot")
        if scene.geometry_source != "simulation" and not any(
            c.get("geometry_validated") for c in cameras.values()
        ):
            self._uncertain("calibrated image geometry is unavailable")
        if scene.geometry_source == "measured_surface" and (
            scene.state == "crumpled"
            or any(v == "verified" for v in scene.held.values())
            or scene.packet_bottom_z is not None
        ):
            self._uncertain(
                "surface projection cannot locate crumpled or suspended cloth"
            )
        if len({g.ident for g in scene.grasps}) != len(scene.grasps):
            self._uncertain("grasp identifiers must be unique")
        if any(v not in {"verified", "lost", "uncertain"} for v in scene.held.values()):
            self._uncertain("invalid grasp evidence")
        if not isinstance(scene.stable, bool) or not isinstance(scene.supported, bool):
            self._uncertain("support evidence must be explicit")
        for point in scene.corners:
            _vector(point)
        for grasp in scene.grasps:
            grasp.pose()
            if not self._simulated and grasp.feature_position is None:
                self._uncertain(
                    "cloth point and palm target must be measured separately"
                )
        for value in (scene.surface_z, scene.packet_bottom_z, scene.alignment_error_m):
            if value is not None and not math.isfinite(value):
                self._uncertain("nonfinite measured geometry")
        if scene.alignment_error_m is not None and scene.alignment_error_m < 0:
            self._uncertain("alignment error must be nonnegative")
        if scene.pile:
            scene.pile.validate()
        self._last_scene, self._last_capture_us = scene, stamp
        self.evidence.append(
            {
                "stage": stage,
                "observation_id": scene.observation_id,
                "state": scene.state,
                "held": dict(scene.held),
                "layers": dict(scene.layers),
                "note": scene.note,
            }
        )
        self._fresh()
        self._check_base()
        return scene

    def _fresh(self):
        age = time.time() - self._last_capture_us / 1e6
        if not 0 <= age <= self.settings.max_observation_age_s:
            self._uncertain("cloth observation expired; observe again")

    def _check_base(self):
        state = self.context.value("evo_status")["state"]
        base = state.get("odometry")
        if not base:
            self._uncertain("base pose is unavailable")
        if self._base is None:
            self._base = base
        elif _norm(_sub(base["position"], self._base["position"])) > 0.005 or abs(
            sum(
                a * b
                for a, b in zip(base["quaternion_xyzw"], self._base["quaternion_xyzw"])
            )
        ) < math.cos(0.005):
            self._uncertain(
                "base moved; cloth and surface coordinates must be measured again"
            )

    def _execute(self, tool, **arguments):
        self._check_base()
        result = self.evo.execute(tool, **arguments)
        self.evidence.append(
            {"tool": tool, "action_id": result["action_id"], "result": result}
        )
        return result

    def _poses(self):
        return self.context.value("evo_fk", links=["left_palm", "right_palm"])["poses"]

    def _path(self, points):
        if not 1 <= len(points) <= 8:
            raise ValueError("cloth path needs 1..8 segments")
        return self._execute(
            "evo_follow_path",
            frame="base_link",
            waypoints=[
                {"duration_s": self.settings.segment_s, **point} for point in points
            ],
        )

    def _paired_path(self, points):
        return self._execute(
            "evo_move_pair",
            frame="base_link",
            waypoints=[
                {
                    "duration_s": self.settings.segment_s,
                    "midpoint": _mean([p["left"]["position"], p["right"]["position"]]),
                    "separation": _sub(p["left"]["position"], p["right"]["position"]),
                    "left_quaternion_xyzw": p["left"]["quaternion_xyzw"],
                    "right_quaternion_xyzw": p["right"]["quaternion_xyzw"],
                }
                for p in points
            ],
        )

    def _grip(self, side, retain):
        return self._execute(
            "evo_set_grip",
            side=side,
            retain=retain,
            position_rad=(
                self.settings.close_rad if retain else self.settings.open_rad
            )[side],
        )

    def _held(self, scene, hands, packet=False):
        self._fresh()
        if scene is None or scene is not self._last_scene:
            self._uncertain("grasp evidence is not current")
        for side in hands:
            if scene.held.get(side) != "verified":
                self._uncertain(f"{side} grasp is not verified")
            if packet and scene.layers.get(side) != "packet":
                self._uncertain(
                    f"{side} does not have a verified complete folded packet"
                )

    def _targets(self, grasps):
        self._fresh()
        if not grasps or set(grasps) - {"left", "right"}:
            raise ValueError("select left and/or right grasps")
        current = {g.ident: g for g in self._last_scene.grasps}
        for grasp in grasps.values():
            if current.get(grasp.ident) != grasp:
                self._uncertain("grasp candidate is not from the current observation")
        return {side: grasp.pose() for side, grasp in grasps.items()}

    def pinch_and_check(self, grasps, packet=False):
        targets = self._targets(grasps)
        for side in targets:
            self._grip(side, False)
        above = {
            s: _pose(
                _add(p["position"], (0, 0, self.settings.approach_clearance_m)),
                p["quaternion_xyzw"],
            )
            for s, p in targets.items()
        }
        self._path([above, targets])
        for side in targets:
            self._grip(side, True)
        lifted = {
            s: _pose(
                _add(p["position"], (0, 0, self.settings.test_lift_m)),
                p["quaternion_xyzw"],
            )
            for s, p in targets.items()
        }
        self._path([lifted])
        scene = self.observe(
            "pinch_check", targets, "packet" if packet else "cloth follows the hands"
        )
        self._held(scene, targets, packet)
        return scene

    def lift_and_inspect(self, hands):
        self._held(self._last_scene, hands)
        poses = self._poses()
        self._path(
            [
                {
                    s: _pose(
                        _add(
                            poses[s + "_palm"]["position"],
                            (0, 0, self.settings.inspection_lift_m),
                        ),
                        poses[s + "_palm"]["quaternion_xyzw"],
                    )
                    for s in hands
                }
            ]
        )
        scene = self.observe(
            "lift_inspect", hands, "visible boundary and grasp candidates"
        )
        self._held(scene, hands)
        return scene

    def spread_to_span(self, span_m):
        if not math.isfinite(span_m) or not 0 < span_m <= self.settings.max_span_m:
            raise ValueError("requested span exceeds the cloth limit")
        self._held(self._last_scene, ("left", "right"))
        poses = self._poses()
        left, right = poses["left_palm"], poses["right_palm"]
        midpoint = _mean([left["position"], right["position"]])
        separation = _sub(left["position"], right["position"])
        start = _norm(separation)
        direction = _unit(separation)
        # Inspect each bounded increase; hand separation is not a tension measurement.
        count = max(1, math.ceil(abs(span_m - start) / 0.04))
        if count > 20:
            raise ValueError("spread needs too many steps")
        for i in range(1, count + 1):
            span = start + (span_m - start) * i / count
            self._execute(
                "evo_move_pair",
                frame="base_link",
                waypoints=[
                    {
                        "duration_s": self.settings.segment_s,
                        "midpoint": midpoint,
                        "separation": _scale(direction, span),
                        "left_quaternion_xyzw": left["quaternion_xyzw"],
                        "right_quaternion_xyzw": right["quaternion_xyzw"],
                    }
                ],
            )
            scene = self.observe(
                "spread",
                ("left", "right"),
                "retained grasps without unexpected deformation",
            )
            self._held(scene, ("left", "right"))
            if not scene.stable:
                self._uncertain("cloth changed unexpectedly during spreading")
        return scene

    def align_held_edge(self, midpoint, separation):
        _vector(midpoint)
        _vector(separation)
        if not 0 < _norm(separation) <= self.settings.max_span_m:
            raise ValueError("edge span exceeds the cloth limit")
        self._held(self._last_scene, ("left", "right"))
        poses = self._poses()
        self._execute(
            "evo_move_pair",
            frame="base_link",
            waypoints=[
                {
                    "duration_s": self.settings.segment_s,
                    "midpoint": midpoint,
                    "separation": separation,
                    "left_quaternion_xyzw": poses["left_palm"]["quaternion_xyzw"],
                    "right_quaternion_xyzw": poses["right_palm"]["quaternion_xyzw"],
                }
            ],
        )
        scene = self.observe("align_edge", ("left", "right"), "edge aligned")
        self._held(scene, ("left", "right"))
        return scene

    def anchor_and_pull(self, anchor, moving, offset):
        if {anchor, moving} != {"left", "right"} or _norm(_vector(offset)) > 0.05:
            raise ValueError("select different hands and a bounded pull")
        self._held(self._last_scene, (anchor, moving))
        poses = self._poses()
        self._path(
            [
                {
                    anchor: poses[anchor + "_palm"],
                    moving: _pose(
                        _add(poses[moving + "_palm"]["position"], offset),
                        poses[moving + "_palm"]["quaternion_xyzw"],
                    ),
                }
            ]
        )
        scene = self.observe("anchor_pull", (anchor, moving), "new boundary exposed")
        self._held(scene, (anchor, moving))
        return scene

    def regrasp(self, source, receiver, candidate, release_source=True):
        if {source, receiver} != {"left", "right"}:
            raise ValueError("regrasp needs different hands")
        self._held(self._last_scene, (source,))
        scene = self.pinch_and_check({receiver: candidate})
        self._held(scene, (source, receiver))
        if release_source:
            self._grip(source, False)
            scene = self.observe("handover", (receiver,), "receiver retains cloth")
            self._held(scene, (receiver,))
        return scene

    def lay_down(self, landing_poses, release=True):
        self._fresh()
        if self._last_scene.surface_z is None:
            self._uncertain("supporting surface has not been measured")
        self._held(self._last_scene, landing_poses)
        poses = {
            s: _pose(p["position"], p["quaternion_xyzw"])
            for s, p in landing_poses.items()
        }
        if any(p["position"][2] < self._last_scene.surface_z for p in poses.values()):
            raise ValueError("landing palm is below the measured surface")
        above = {
            s: _pose(
                _add(p["position"], (0, 0, self.settings.approach_clearance_m)),
                p["quaternion_xyzw"],
            )
            for s, p in poses.items()
        }
        self._path([above, poses])
        scene = self.observe("supported", poses, "cloth rests on the surface")
        if not scene.stable or not scene.supported:
            self._uncertain("cloth support is not verified; retained grips remain")
        if release:
            for side in poses:
                self._grip(side, False)
            scene = self.observe("laid_down", expected="cloth placement")
        return scene

    def fold_over_line(self, line, grasps, *, packet=False):
        targets = self._targets(grasps)
        a, b = (_vector(p) for p in line)
        direction = _unit((b[0] - a[0], b[1] - a[1], 0))
        normal = (-direction[1], direction[0], 0)
        self.pinch_and_check(grasps, packet=packet)
        start = self._poses()
        points = []
        for fraction in (0.25, 0.5, 0.75, 1.0):
            item = {}
            for side in targets:
                p = start[side + "_palm"]
                distance = sum(
                    (grasps[side].point()[i] - a[i]) * normal[i] for i in range(3)
                )
                location = _add(p["position"], _scale(normal, -2 * distance * fraction))
                location = _add(
                    location,
                    (
                        0,
                        0,
                        4 * fraction * (1 - fraction) * self.settings.fold_clearance_m,
                    ),
                )
                item[side] = _pose(location, p["quaternion_xyzw"])
            points.append(item)
        self._path(points)
        # Remove the test-lift offset only after the complete transfer.
        land = {
            s: _pose(
                _sub(
                    p["position"],
                    (0, 0, self.settings.test_lift_m - self.settings.release_gap_m),
                ),
                p["quaternion_xyzw"],
            )
            for s, p in points[-1].items()
        }
        self.observe("fold_before_landing", targets, "surface and retained grasps")
        scene = self.lay_down(land)
        if (
            scene.state != "folded"
            or not scene.stable
            or not scene.supported
            or scene.alignment_error_m is None
            or scene.alignment_error_m > self.settings.alignment_tolerance_m
        ):
            self._uncertain("fold alignment is not verified")
        return scene

    @staticmethod
    def _edge_grasps(scene, corners=None):
        candidates = sorted(scene.grasps, key=lambda g: g.score, reverse=True)
        if corners:
            selected = []
            for point in corners:
                usable = [
                    g for g in candidates if g.ident not in {s.ident for s in selected}
                ]
                if not usable:
                    raise ClothUncertain("not enough distinct grasp candidates")
                nearest = min(usable, key=lambda g: _norm(_sub(g.point(), point)))
                if _norm(_sub(nearest.point(), point)) > 0.04:
                    raise ClothUncertain("no grasp candidate near a fold corner")
                selected.append(nearest)
        else:
            selected = candidates[:2]
        if (
            len(selected) != 2
            or _norm(_sub(selected[0].position, selected[1].position)) < 0.04
        ):
            raise ClothUncertain("two separated grasp candidates are required")
        selected.sort(key=lambda g: g.position[1], reverse=True)
        return dict(zip(("left", "right"), selected))

    def unfold(self, landing_poses, span_m):
        """Bounded pick, inspect, regrasp, spread and lay-down loop for crumpled cloth."""
        for _ in range(self.settings.max_unfold_attempts):
            scene = self.observe("unfold_select", expected="visible grasp candidates")
            if (
                scene.state == "spread"
                and scene.stable
                and scene.supported
                and len(scene.corners) == 4
            ):
                self._rectangle(scene)
                return scene
            if not scene.grasps:
                self._uncertain(
                    "no verified metric grasp candidate on the crumpled cloth"
                )
            candidate = max(scene.grasps, key=lambda g: g.score)
            source = "left" if candidate.position[1] >= 0 else "right"
            receiver = "right" if source == "left" else "left"
            self.pinch_and_check({source: candidate})
            scene = self.lift_and_inspect((source,))
            source_pose = self._poses()[source + "_palm"]["position"]
            available = [
                g
                for g in scene.grasps
                if g.feature in {"edge", "corner"}
                and _norm(_sub(g.position, source_pose)) >= 0.06
            ]
            if not available:
                self._uncertain(
                    "no visible second edge grasp; change the view or recover"
                )
            self.regrasp(
                source,
                receiver,
                max(available, key=lambda g: g.score),
                release_source=False,
            )
            self.spread_to_span(span_m)
            scene = self.lay_down(landing_poses)
            if (
                scene.state == "spread"
                and scene.stable
                and scene.supported
                and len(scene.corners) == 4
            ):
                self._rectangle(scene)
                return scene
        self._uncertain("unfold attempt budget exhausted")

    def _rectangle(self, scene):
        if len(scene.corners) != 4:
            self._uncertain("rectangle corners are unavailable")
        edges = [_sub(scene.corners[(i + 1) % 4], scene.corners[i]) for i in range(4)]
        lengths = [_norm(e) for e in edges]
        if (
            min(lengths) < 0.02
            or max(p[2] for p in scene.corners) - min(p[2] for p in scene.corners)
            > 0.02
        ):
            self._uncertain("footprint is not a flat rectangle")
        for i in range(4):
            if (
                abs(sum(a * b for a, b in zip(edges[i], edges[(i + 1) % 4])))
                > 0.15 * lengths[i] * lengths[(i + 1) % 4]
            ):
                self._uncertain(
                    "footprint corners are not in rectangular boundary order"
                )
        if any(_norm(_add(edges[i], edges[i + 2])) > 0.02 for i in range(2)):
            self._uncertain("opposite footprint edges do not match")

    def fold(self, axes=("v", "u")):
        """Fold a measured rectangular towel in half along each selected local axis."""
        if not 1 <= len(axes) <= 4 or any(axis not in {"u", "v"} for axis in axes):
            raise ValueError("fold pattern must contain 1..4 u/v half-folds")
        for axis in axes:
            scene = self.observe(
                "fold_select", expected="ordered rectangle corners and corner grasps"
            )
            if (
                scene.state not in {"spread", "folded"}
                or not scene.stable
                or not scene.supported
                or len(scene.corners) != 4
            ):
                self._uncertain("fold requires a visible flat rectangular footprint")
            self._rectangle(scene)
            corners = list(scene.corners)
            if axis == "u":
                corners = corners[1:] + corners[:1]
            line = (_mean([corners[0], corners[3]]), _mean([corners[1], corners[2]]))
            if scene.state == "folded":
                candidates = replace(
                    scene,
                    grasps=tuple(g for g in scene.grasps if g.feature == "packet"),
                )
            else:
                candidates = scene
            grasps = self._edge_grasps(candidates, corners[:2])
            self.fold_over_line(line, grasps, packet=scene.state == "folded")
        return self._last_scene

    def stack_folded(self):
        """Pick a complete folded packet and place it on a freshly measured pile."""
        scene = self.observe(
            "stack_select",
            expected="folded packet footprint, packet grasps and pile top",
        )
        if (
            scene.state != "folded"
            or not scene.stable
            or not scene.supported
            or len(scene.corners) != 4
            or not scene.pile
        ):
            self._uncertain("stacking needs a folded packet and measured pile")
        self._rectangle(scene)
        edge = _sub(scene.corners[1], scene.corners[0])
        size = (_norm(edge), _norm(_sub(scene.corners[2], scene.corners[1])))
        if any(
            a + 2 * self.settings.alignment_tolerance_m > b
            for a, b in zip(size, scene.pile.size_xy)
        ):
            self._uncertain(
                "folded packet does not fit the placement region with margin"
            )
        grasps = self._edge_grasps(
            replace(
                scene, grasps=tuple(g for g in scene.grasps if g.feature == "packet")
            )
        )
        self.pinch_and_check(grasps, packet=True)
        scene = self.observe(
            "stack_transport",
            grasps,
            "complete packet held, packet bottom and current pile top",
        )
        self._held(scene, grasps, packet=True)
        if scene.packet_bottom_z is None or not scene.pile:
            self._uncertain("packet bottom or pile height is unknown")
        self._rectangle(scene)
        source_center = _mean(scene.corners)
        edge = _sub(scene.corners[1], scene.corners[0])
        size = (_norm(edge), _norm(_sub(scene.corners[2], scene.corners[1])))
        source_yaw = math.atan2(edge[1], edge[0])
        pile = scene.pile
        if any(
            a + 2 * self.settings.alignment_tolerance_m > b
            for a, b in zip(size, pile.size_xy)
        ):
            self._uncertain("updated pile footprint is too small")
        poses = self._poses()
        dz = pile.center[2] + self.settings.release_gap_m - scene.packet_bottom_z
        destination = {
            s: _yaw_pose(
                poses[s + "_palm"],
                source_center,
                pile.center,
                pile.yaw_rad - source_yaw,
                dz,
            )
            for s in grasps
        }
        # Lift vertically before translating. Never drag the packet across the pile.
        clearance = max(
            self.settings.transport_clearance_m,
            dz + self.settings.transport_clearance_m,
        )
        raised = {
            s: _pose(
                _add(poses[s + "_palm"]["position"], (0, 0, clearance)),
                poses[s + "_palm"]["quaternion_xyzw"],
            )
            for s in grasps
        }
        above = {
            s: _pose(_add(p["position"], (0, 0, clearance - dz)), p["quaternion_xyzw"])
            for s, p in destination.items()
        }
        self._paired_path([raised, above, destination])
        scene = self.observe(
            "stack_supported", grasps, "packet supported on pile and placement aligned"
        )
        if (
            not scene.stable
            or not scene.supported
            or scene.alignment_error_m is None
            or scene.alignment_error_m > self.settings.alignment_tolerance_m
        ):
            self._uncertain("pile placement is not verified; grips remain retained")
        for side in grasps:
            self._grip(side, False)
        self._path([above])
        scene = self.observe(
            "stack_complete", expected="packet released and pile stable"
        )
        if scene.state != "stacked" or not scene.stable or not scene.supported:
            self._uncertain("stack result is not verified")
        return scene

    def crumpled_to_pile(self, landing_poses, span_m, axes=("v", "u")):
        self.unfold(landing_poses, span_m)
        self.fold(axes)
        return self.stack_folded()
