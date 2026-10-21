"""Cloth workflow tests use explicit perception fixtures, not cloth physics."""
from dataclasses import replace
import copy
import time

import pytest
from auto_research_agent.api.cloth import (
    Cloth,
    ClothSettings,
    ClothUncertain,
    Grasp,
    Scene,
    Pile,
)
from auto_research_agent.api.cloth_geometry import triangulate, on_measured_plane
from auto_research_agent.api.evo import Evo, EvoActionFailed

Q = (0, 0, 0, 1)
CORNERS = ((0.4, 0.15, 0.5), (0.4, -0.15, 0.5), (0.6, -0.15, 0.5), (0.6, 0.15, 0.5))
SETTINGS = ClothSettings(
    open_rad={"left": 0.8, "right": 0.8}, close_rad={"left": 0.1, "right": 0.1}
)
LANDING = {
    s: {"position": list(CORNERS[i]), "quaternion_xyzw": list(Q)}
    for i, s in enumerate(("left", "right"))
}


class Context:
    """Immediate measured controller fixture with inspectable public calls."""

    def __init__(self):
        self.calls = []
        self.poses = {s + "_palm": copy.deepcopy(p) for s, p in LANDING.items()}
        self.number = 0
        self.base = [0, 0]
        self.capture_age_s = 0
        self.failed = False
        self.simulated = True

    def value(self, tool, **args):
        self.calls.append((tool, copy.deepcopy(args)))
        if tool == "evo_capabilities":
            return {"simulated": self.simulated}
        if tool == "evo_status":
            return {
                "state": {
                    "odometry": {
                        "position": list(self.base),
                        "quaternion_xyzw": list(Q),
                    }
                }
            }
        if tool == "evo_observe":
            self.number += 1
            return {
                "observation_id": str(self.number),
                "cameras": {
                    "base": {
                        "capture_us": int((time.time() - self.capture_age_s) * 1e6),
                        "geometry_validated": False,
                    }
                },
            }
        if tool == "evo_fk":
            return {"poses": copy.deepcopy(self.poses)}
        if tool == "evo_preview":
            return {"valid": True}
        if tool == "evo_action_status":
            return {
                "action_id": args["action_id"],
                "state": "failed" if self.failed else "succeeded",
                "physical_stop_verified": True,
            }
        if tool == "evo_follow_path":
            for s in ("left", "right"):
                if s in args["waypoints"][-1]:
                    self.poses[s + "_palm"] = copy.deepcopy(args["waypoints"][-1][s])
        if tool == "evo_move_pair":
            w = args["waypoints"][-1]
            for s, sign in (("left", 1), ("right", -1)):
                self.poses[s + "_palm"] = {
                    "position": [
                        a + sign * b / 2 for a, b in zip(w["midpoint"], w["separation"])
                    ],
                    "quaternion_xyzw": w[s + "_quaternion_xyzw"],
                }
        return {"action_id": f"a-{len(self.calls)}", "state": "accepted"}


class Perception:
    def __init__(self, state="crumpled"):
        self.state = state
        self.checks = []
        self.folding = False
        self.stacking = False
        self.change = lambda scene, check: scene

    def __call__(self, observation, check):
        self.checks.append(check)
        if check.stage == "fold_select":
            self.folding = True
        if check.stage == "stack_select":
            self.stacking = True
        if check.stage == "laid_down":
            self.state = "folded" if self.folding else "spread"
        if check.stage == "stack_complete":
            self.state = "stacked"
        grasps = tuple(
            Grasp(
                str(i),
                p,
                Q,
                "packet" if self.stacking or self.state == "folded" else "corner",
            )
            for i, p in enumerate(CORNERS)
        )
        scene = Scene(
            observation["observation_id"],
            self.state,
            "simulation",
            grasps=grasps,
            corners=CORNERS,
            held={"left": "verified", "right": "verified"},
            layers={"left": "packet", "right": "packet"},
            stable=True,
            supported=True,
            surface_z=0.48,
            packet_bottom_z=0.52,
            pile=Pile((0.8, 0, 0.65), (0.5, 0.5)),
            alignment_error_m=0.01,
        )
        return self.change(scene, check)


def fixture(state="crumpled"):
    context, perceive = Context(), Perception(state)
    return context, perceive, Cloth(context, perceive, SETTINGS)


def motions(context):
    return [
        (t, a)
        for t, a in context.calls
        if t in {"evo_follow_path", "evo_move_pair", "evo_set_grip"}
    ]


def test_crumpled_towel_to_folded_pile_uses_public_actions_and_verifies_stages():
    context, perceive, cloth = fixture()
    scene = cloth.crumpled_to_pile(LANDING, 0.35)
    assert scene.state == "stacked"
    stages = [c.stage for c in perceive.checks]
    assert (
        stages.index("lift_inspect")
        < stages.index("spread")
        < stages.index("fold_select")
        < stages.index("stack_select")
    )
    assert stages.count("fold_select") == 2
    assert "stack_supported" in stages and stages[-1] == "stack_complete"
    assert all(c.expected for c in perceive.checks)
    assert sum(t == "evo_preview" for t, _ in context.calls) == len(motions(context))
    assert sum(t == "evo_action_status" for t, _ in context.calls) == len(
        motions(context)
    )
    # Both hands follow one path; all fold waypoints use a common duration.
    folds = [
        a
        for t, a in motions(context)
        if t == "evo_follow_path" and len(a["waypoints"]) == 4
    ]
    assert len(folds) == 2
    assert all(
        set(w) == {"left", "right", "duration_s"} for a in folds for w in a["waypoints"]
    )


def test_failed_handover_does_not_release_source_or_move_again():
    context, perceive, cloth = fixture()
    scene = cloth.observe("select")
    perceive.change = lambda scene, check: replace(
        scene, held={"left": "verified", "right": "lost"}
    )
    with pytest.raises(ClothUncertain, match="right grasp"):
        cloth.regrasp("left", "right", scene.grasps[1])
    assert not any(
        t == "evo_set_grip" and a["side"] == "left" for t, a in motions(context)
    )
    assert perceive.checks[-1].stage == "pinch_check"


def test_stack_requires_all_layers_before_transport():
    context, perceive, cloth = fixture("folded")
    perceive.change = lambda scene, check: replace(
        scene, layers={"left": "single", "right": "packet"}
    )
    with pytest.raises(ClothUncertain, match="complete folded packet"):
        cloth.stack_folded()
    assert "stack_transport" not in [c.stage for c in perceive.checks]
    assert not any(
        t == "evo_move_pair" and len(a["waypoints"]) == 3 for t, a in motions(context)
    )


def test_stack_uses_new_pile_height_and_packet_pose_after_pick():
    context, perceive, cloth = fixture("folded")

    def update(scene, check):
        if check.stage == "stack_transport":
            return replace(
                scene,
                pile=Pile((0.8, 0, 0.7), (0.5, 0.5)),
                packet_bottom_z=0.52,
                corners=tuple((x + 0.01, y, z + 0.03) for x, y, z in scene.corners),
            )
        return scene

    perceive.change = update
    cloth.stack_folded()
    transport = next(
        a
        for t, a in motions(context)
        if t == "evo_move_pair" and len(a["waypoints"]) == 3
    )
    # Actual palms after test lift: z=.53. Fresh measured bottom=.52, pile=.70.
    assert transport["waypoints"][-1]["midpoint"][2] == pytest.approx(0.72)
    assert transport["waypoints"][0]["midpoint"][:2] == (CORNERS[0][0], 0.0)


@pytest.mark.parametrize(
    "problem", ["missing_height", "too_small", "unsupported", "unstable", "misaligned"]
)
def test_uncertain_stack_stops_before_release(problem):
    context, perceive, cloth = fixture("folded")

    def update(scene, check):
        if check.stage == "stack_transport" and problem == "missing_height":
            return replace(scene, packet_bottom_z=None)
        if check.stage == "stack_transport" and problem == "too_small":
            return replace(scene, pile=Pile((0.8, 0, 0.7), (0.01, 0.01)))
        if check.stage == "stack_supported" and problem == "unsupported":
            return replace(scene, supported=False)
        if check.stage == "stack_supported" and problem == "unstable":
            return replace(scene, stable=False)
        if check.stage == "stack_supported" and problem == "misaligned":
            return replace(scene, alignment_error_m=0.1)
        return scene

    perceive.change = update
    with pytest.raises(ClothUncertain):
        cloth.stack_folded()
    # Initial opens are required for picking; no opens follow the closes.
    grip = [
        (a["side"], a["retain"]) for t, a in motions(context) if t == "evo_set_grip"
    ]
    assert grip == [("left", False), ("right", False), ("left", True), ("right", True)]


def test_stale_perception_wrong_capture_and_uncalibrated_geometry_never_move():
    for problem in ("stale", "capture", "calibration", "base"):
        context, perceive, cloth = fixture()
        if problem == "stale":
            context.capture_age_s = 20
        if problem == "capture":
            perceive.change = lambda s, c: replace(s, observation_id="old")
        if problem == "calibration":
            perceive.change = lambda s, c: replace(
                s, geometry_source="calibrated_triangulation"
            )
        if problem == "base":

            def changed(s, c):
                context.base[0] = 0.1
                return s

            perceive.change = changed
        with pytest.raises(ClothUncertain):
            cloth.unfold(LANDING, 0.35)
        assert motions(context) == []


def test_simulation_geometry_cannot_enable_physical_motion():
    context, perceive = Context(), Perception()
    context.simulated = False
    cloth = Cloth(context, perceive, SETTINGS)
    with pytest.raises(ClothUncertain, match="physical robot"):
        cloth.unfold(LANDING, 0.35)
    assert not motions(context)


def test_unfold_has_finite_attempt_budget():
    context, perceive, cloth = fixture()
    perceive.change = lambda s, c: replace(s, state="crumpled")
    with pytest.raises(ClothUncertain, match="budget exhausted"):
        cloth.unfold(LANDING, 0.35)
    assert [c.stage for c in perceive.checks].count(
        "unfold_select"
    ) == SETTINGS.max_unfold_attempts


def test_invalid_rectangle_and_oversized_packet_are_refused_before_motion():
    for corners, pile in (
        (CORNERS[::-2] + CORNERS[1::2], Pile((0.8, 0, 0.6), (0.5, 0.5))),
        (CORNERS, Pile((0.8, 0, 0.6), (0.1, 0.1))),
    ):
        context, perceive, cloth = fixture("folded")
        perceive.change = lambda s, c: replace(s, corners=corners, pile=pile)
        with pytest.raises(ClothUncertain):
            cloth.stack_folded()
        assert not motions(context)


def test_action_failure_does_not_start_next_cloth_stage():
    context, perceive, cloth = fixture()
    context.failed = True
    with pytest.raises(EvoActionFailed):
        cloth.unfold(LANDING, 0.35)
    assert len(motions(context)) == 1
    assert len(perceive.checks) == 1


def test_evo_execute_rejects_non_action_tools():
    context = Context()
    with pytest.raises(ValueError):
        Evo(context).execute("evo_hold")
    assert not context.calls


def ray(origin, direction, stamp=1_000_000):
    return {
        "frame": "base_link",
        "origin": origin,
        "direction": direction,
        "capture_us": stamp,
    }


def test_calibrated_triangulation_and_measured_surface_geometry():
    a, b = ray((-0.1, 0, 0), (0.1, 0, 1)), ray((0.1, 0, 0), (-0.1, 0, 1))
    result = triangulate(a, b)
    assert result["position"] == pytest.approx((0, 0, 1))
    assert result["residual_m"] < 1e-8
    assert on_measured_plane(a, (0, 0, 1), (0, 0, 1), verified_on_surface=True)[
        "position"
    ] == pytest.approx((0, 0, 1))
    with pytest.raises(ValueError, match="verified"):
        on_measured_plane(a, (0, 0, 1), (0, 0, 1), verified_on_surface=False)
    with pytest.raises(ValueError, match="angle"):
        triangulate(a, a)
    with pytest.raises(ValueError, match="synchronized"):
        triangulate(a, {**b, "capture_us": 2_000_000})
    with pytest.raises(ValueError, match="intersect"):
        triangulate(a, ray((0.1, 0.1, 0), (-0.1, 0, 1)))
    with pytest.raises(ValueError, match="behind"):
        triangulate(ray((-0.1, 0, 0), (-0.1, 0, 1)), ray((0.1, 0, 0), (0.1, 0, 1)))


def test_scene_json_conversion_preserves_measurements():
    value = Scene.from_dict(
        {
            "observation_id": "a",
            "state": "folded",
            "geometry_source": "calibrated_triangulation",
            "grasps": [
                {
                    "ident": "edge",
                    "position": [1, 2, 3],
                    "quaternion_xyzw": [0, 0, 0, 1],
                    "feature": "packet",
                }
            ],
            "pile": {"center": [1, 2, 3], "size_xy": [0.4, 0.5]},
        }
    )
    assert value.grasps[0].pose()["position"] == [1, 2, 3]
    value.pile.validate()


def test_invalid_preview_cannot_start_a_cloth_action():
    class InvalidPreview(Context):
        def value(self, tool, **args):
            if tool == "evo_preview":
                return {"valid": False}
            return super().value(tool, **args)

    context = InvalidPreview()
    with pytest.raises(ValueError, match="preview"):
        Evo(context).execute(
            "evo_move_body", targets={"left_joint1": 0.1}, duration_s=3
        )
    assert not motions(context)


def test_refolding_requires_all_packet_layers_before_fold_transfer():
    context, perceive, cloth = fixture("folded")
    perceive.change = lambda s, c: replace(
        s, layers={"left": "single", "right": "packet"}
    )
    with pytest.raises(ClothUncertain, match="complete folded packet"):
        cloth.fold(axes=("v",))
    assert not any(
        t == "evo_follow_path" and len(a["waypoints"]) == 4 for t, a in motions(context)
    )
