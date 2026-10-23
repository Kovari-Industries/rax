import copy
import json
import math
import time
from dataclasses import replace

import pytest

from auto_research_agent.api.cloth import ClothUncertain
from auto_research_agent.api.cloth_actions import ClothActions
from auto_research_agent.api.cloth_vision import ClothVision
from auto_research_agent.api.cloth_trial import TrialContext, ReplayContext
from auto_research_agent.api.towel_planner import plan_half_fold, area, placement_yaw
from local_tests.unit.test_cloth import CORNERS, Context, fixture, motions


def test_fold_contact_curve_respects_available_material_even_during_test_lift():
    plan = plan_half_fold(
        CORNERS,
        ((0.5, -0.2, 0.5), (0.5, 0.2, 0.5)),
        {"left": CORNERS[0], "right": CORNERS[1]},
        test_lift_m=0.2,
        release_gap_m=0.001,
    )
    assert area(plan.expected_corners) == pytest.approx(area(CORNERS) / 2)
    for step in (plan.lift_contacts, *plan.contacts):
        for p in step.values():
            # Distance along the support plane plus hanging height equals material length.
            assert abs(p[0] - 0.5) + p[2] - 0.5 == pytest.approx(0.1)
    assert max(p[2] for step in plan.contacts for p in step.values()) == pytest.approx(
        0.6
    )


def test_invalid_fold_line_rejected_before_pick():
    context, perceive, cloth = fixture("spread")
    scene = cloth.observe("select")
    with pytest.raises(ValueError, match="half-fold"):
        cloth.fold_over_line(
            ((0.49, -0.2, 0.5), (0.49, 0.2, 0.5)),
            {"left": scene.grasps[0], "right": scene.grasps[1]},
        )
    assert not motions(context)


@pytest.mark.parametrize("problem", ["alignment", "unchanged", "support", "unknown"])
def test_fold_checks_before_release_even_when_state_claims_success(problem):
    context, perceive, cloth = fixture("spread")

    def change(scene, check):
        if check.stage == "fold_supported":
            if problem == "alignment":
                return replace(scene, alignment_error_m=0.2)
            if problem == "unchanged":
                return replace(scene, corners=CORNERS)
            if problem == "support":
                return replace(scene, supported=False)
            return replace(scene, state="unknown")
        return scene

    perceive.change = change
    with pytest.raises(ClothUncertain):
        cloth.fold(axes=("v",))
    assert [a["retain"] for t, a in motions(context) if t == "evo_set_grip"] == [
        False,
        False,
        True,
        True,
    ]


def test_two_folds_reduce_measured_area_twice():
    context, perceive, cloth = fixture("spread")
    result = cloth.fold()
    assert area(result.corners) == pytest.approx(area(CORNERS) / 4)


@pytest.mark.parametrize("shift", range(4))
@pytest.mark.parametrize("reverse", [False, True])
def test_stack_symmetry_avoids_half_turn_for_any_corner_order(shift, reverse):
    corners = CORNERS[::-1] if reverse else CORNERS
    corners = corners[shift:] + corners[:shift]
    assert placement_yaw(corners, (0.5, 0.5), math.pi / 2, 0.01) == pytest.approx(
        0, abs=1e-8
    )
    context, perceive, cloth = fixture("folded")
    perceive.change = lambda s, c: replace(
        s, corners=corners, pile=replace(s.pile, yaw_rad=math.pi / 2)
    )
    cloth.stack_folded()
    transport = next(
        a
        for t, a in motions(context)
        if t == "evo_move_pair" and len(a["waypoints"]) == 3
    )
    assert transport["waypoints"][0]["separation"] == pytest.approx(
        transport["waypoints"][-1]["separation"]
    )


def vision_fixture(tmp_path):
    (tmp_path / "capture.jpg").write_bytes(b"\xff\xd8test\xff\xd9")
    observation = {
        "observation_id": "capture",
        "cameras": {
            "base": {
                "capture_us": int(time.time() * 1e6),
                "width": 640,
                "height": 480,
                "path": "capture.jpg",
            }
        },
    }
    data = {
        "observation_id": "capture",
        "state": "unknown",
        "checks": [{"name": "supported", "verdict": "unknown", "evidence": "occluded"}],
        "candidates": [],
    }

    class VisionContext(Context):
        def value(self, tool, **args):
            if tool == "gemini_generate":
                self.calls.append((tool, args))
                return {
                    "finish_reason": "stop",
                    "model": "test-model",
                    "message": {"content": json.dumps(data)},
                    "usage": {},
                }
            return super().value(tool, **args)

    context = VisionContext()
    return context, observation, data, ClothVision(context, tmp_path)


def test_vision_uses_shared_model_tool_and_unknown_blocks_release(tmp_path):
    context, observation, data, vision = vision_fixture(tmp_path)
    assessment = vision.inspect(observation, ["supported"], target={"state": "folded"})
    assert assessment.verdict("supported") == "unknown"
    assert context.calls[0][0] == "gemini_generate"
    assert context.calls[0][1]["messages"][0]["content"][-1]["type"] == "image"
    with pytest.raises(ValueError, match="did not verify"):
        ClothActions(context).release_checked(
            "left", 0.8, assessment, checks=["supported"]
        )
    assert not motions(context)
    observation["cameras"]["base"]["capture_us"] = 0
    assessment.require()  # Original caller cannot change the saved capture.


@pytest.mark.parametrize(
    "bad",
    [
        None,
        [],
        {"checks": None},
        {"observation_id": "old"},
        {"checks": [{"name": "supported", "verdict": "verified", "evidence": ""}]},
        {
            "candidates": [
                {
                    "id": "a",
                    "feature": "corner",
                    "confidence": 0.9,
                    "note": "visible",
                    "views": [{"camera": "base", "pixel": [640, 2]}],
                }
            ]
        },
    ],
)
def test_malformed_model_evidence_refused(tmp_path, bad):
    context, observation, data, vision = vision_fixture(tmp_path)
    if isinstance(bad, dict):
        data.update(bad)
    else:
        data["checks"] = bad
    with pytest.raises(ValueError):
        vision.inspect(observation, ["supported"])
    assert not motions(context)


def test_vision_no_implicit_depth_and_workspace_escape(tmp_path):
    context, observation, data, vision = vision_fixture(tmp_path)
    data["candidates"] = [
        {
            "id": "a",
            "feature": "corner",
            "confidence": 0.9,
            "note": "visible",
            "views": [{"camera": "base", "pixel": [20, 20]}],
        }
    ]
    assessment = vision.inspect(observation, ["supported"])
    with pytest.raises(ValueError, match="two corresponding"):
        assessment.locate(context, "a")
    assert [t for t, a in context.calls] == ["gemini_generate"]
    observation["cameras"]["base"]["path"] = "../outside.jpg"
    with pytest.raises(ValueError, match="outside"):
        vision.inspect(observation, ["supported"])
    assessment.observation["cameras"]["base"]["capture_us"] = 1
    with pytest.raises(ValueError, match="stale"):
        assessment.require()


def test_generic_actions_need_no_rectangle_and_enforce_increment_bounds():
    context = Context()
    actions = ClothActions(context)
    actions.lift(["left"], 0.02)
    actions.spread_increment(0.01)
    assert len(motions(context)) == 2
    before = len(context.calls)
    for value in (0, -1, 0.05, float("nan")):
        with pytest.raises(ValueError):
            actions.lift(["left"], value)
        with pytest.raises(ValueError):
            actions.spread_increment(value)
    assert len(context.calls) == before


def test_trial_replay_preserves_images_without_live_connection(tmp_path):
    context, observation, data, vision = vision_fixture(tmp_path)

    class Captures:
        def value(self, tool, **args):
            return copy.deepcopy(observation)

    trial = TrialContext(Captures(), tmp_path / "trial", workspace=tmp_path)
    assert trial.value("evo_observe") == observation
    trial.annotate("uncertain", "grasp occluded")
    trial.close()
    (tmp_path / "capture.jpg").unlink()
    replay = ReplayContext(tmp_path / "trial")
    with pytest.raises(ValueError, match="differs"):
        replay.value("evo_drive")
    saved = replay.value("evo_observe")
    assert replay.annotations[0]["outcome"] == "uncertain"
    assert saved["cameras"]["base"]["path"].endswith("capture-1-0.jpg")
    assert (
        tmp_path / "trial" / "capture-1-0.jpg"
    ).read_bytes() == b"\xff\xd8test\xff\xd9"
    with pytest.raises(ValueError, match="exhausted"):
        replay.value("evo_observe")


def test_trial_failure_latches_and_incomplete_log_refused(tmp_path):
    class Fails:
        def value(self, tool, **args):
            raise RuntimeError("controller failed")

    trial = TrialContext(Fails(), tmp_path / "trial")
    with pytest.raises(RuntimeError, match="controller failed"):
        trial.value("evo_follow_path")
    with pytest.raises(RuntimeError, match="closed"):
        trial.value("evo_follow_path")
    trial.close()
    replay = ReplayContext(tmp_path / "trial")
    with pytest.raises(RuntimeError, match="recorded error"):
        replay.value("evo_follow_path")
    log = tmp_path / "trial" / "events.jsonl"
    log.write_text(log.read_text().splitlines()[0] + "\n")
    with pytest.raises(ValueError, match="incomplete"):
        ReplayContext(tmp_path / "trial")


def test_journal_write_failure_prevents_dispatch(tmp_path):
    context = Context()
    trial = TrialContext(context, tmp_path / "trial")
    trial.stream.close()
    with pytest.raises(ValueError):
        trial.value("evo_drive")
    assert not context.calls


def test_vision_localization_uses_capture_bound_calibrated_rays(tmp_path):
    context, observation, data, vision = vision_fixture(tmp_path)
    observation["cameras"]["left_arm"] = copy.deepcopy(observation["cameras"]["base"])
    data["candidates"] = [
        {
            "id": "a",
            "feature": "corner",
            "confidence": 0.9,
            "note": "same point",
            "views": [
                {"camera": name, "pixel": [20, 20]} for name in observation["cameras"]
            ],
        }
    ]
    assessment = vision.inspect(observation, ["supported"])

    class Rays:
        def __init__(self):
            self.calls = []

        def value(self, tool, **arguments):
            self.calls.append((tool, arguments))
            x = -0.1 if arguments["camera"] == "base" else 0.1
            return {
                "frame": "base_link",
                "capture_us": observation["cameras"]["base"]["capture_us"],
                "origin": [x, 0, 0],
                "direction": [-x, 0, 1],
            }

    rays = Rays()
    assert assessment.locate(rays, "a")["position"] == pytest.approx((0, 0, 1))
    assert all(
        t == "evo_camera_ray" and a["observation_id"] == "capture"
        for t, a in rays.calls
    )

    class MissingCalibration:
        def value(self, tool, **arguments):
            raise RuntimeError("calibration unavailable")

    with pytest.raises(RuntimeError, match="calibration unavailable"):
        assessment.locate(MissingCalibration(), "a")


def test_trial_image_tampering_is_not_replayed(tmp_path):
    context, observation, data, vision = vision_fixture(tmp_path)

    class Captures:
        def value(self, tool, **args):
            return observation

    trial = TrialContext(Captures(), tmp_path / "trial", workspace=tmp_path)
    trial.value("evo_observe")
    trial.close()
    (tmp_path / "trial" / "capture-1-0.jpg").write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed"):
        ReplayContext(tmp_path / "trial").value("evo_observe")
