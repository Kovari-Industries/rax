"""Deterministic task tests. This context does not connect to robot hardware."""
from dataclasses import replace
import math
import time

import pytest

from auto_research_agent.api.base import Base, PrimitiveError
from auto_research_agent.api.elevator import ArmProfile, Elevator, ElevatorSettings
from auto_research_agent.api.evo import Evo, EvoActionFailed


def posture(side, bend=1.0):
    return {f"{side}_joint{i}": bend if i == 4 else 0.0 for i in range(1, 8)}


def settings(**changes):
    profiles = tuple(ArmProfile(side, posture(side), (0, 0, 0, 1), (0, 0, 0),
                                "synthetic-test-only", f"{side}_joint4", 0.5,
                                search_joints=(posture(side, 1.2),)) for side in ("left", "right"))
    return replace(ElevatorSettings(profiles, "test-panel", {"2": (0, 0)}, 0.0, 0.75,
                                    dwell_s=0.001, approach_budget_m=0.8), **changes)


class RobotContext:
    def __init__(self):
        self.calls = []
        self.pose = [0., 0., 0.]
        self.joints = {**posture("left", 0), **posture("right", 0), "carriage_joint": 0.0}
        self.palms = {s: {"position": [0.2, y, 1.0], "quaternion_xyzw": [0, 0, 0, 1]}
                      for s, y in (("left", 0.1), ("right", -0.1))}
        self.scene_cfg = {"max_age_s": 0.5, "max_skew_s": 0.05, "position_tolerance_m": 0.01,
                          "angle_tolerance_rad": 0.01, "max_turn_rad": math.pi, "max_step_m": 2.,
                          "footprint": [0.25, 0.2, 1.8], "margin_m": 0.05,
                          "travel_joints": self.joints.copy()}
        self.capabilities = {"scene": {"configured": True, "settings": self.scene_cfg, "button_detection": True},
                             "tools": [{"name": "evo_follow_path"}],
                             "model": {"limits": {n: {"lower": -3., "upper": 3.} for n in self.joints}},
                             "joint_limits": {n: {"tolerance": 0.01} for n in self.joints}}
        self.last_result = None
        self.door_state = "open"
        self.door_id = "door-1"
        self.present = {"left": True, "right": True}
        self.preview_refused = set()
        self.detect_count = 0
        self.fail_action = None
        self.capture = None
        self.stale = False
        self.ray_bad_stamp = False
        self.calibrated = True
        self.duplicate_label = False
        self.close_on_crossing = False
        self.base_calls = []

    def local(self, point):
        x, y = point[0]-self.pose[0], point[1]-self.pose[1]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        return [c*x+s*y, -s*x+c*y, point[2]]

    def scene(self, stamp=None):
        stamp = stamp or time.time_ns()//1000
        normal = [math.cos(self.pose[2]), -math.sin(self.pose[2]), 0.]
        center = self.local((1., 0., 0.))
        return {"observation_id": f"scene-{stamp}", "capture_us": stamp-(1_000_000 if self.stale else 0),
                "frame": "base_link", "heading_source": "lidar_fixed_reference", "pose": self.pose.copy(),
                "panel": {"point": center, "normal": normal, "uncertainty_m": 0.015},
                "doorway": {"id": self.door_id, "center": center, "normal": normal, "width_m": 1.2,
                            "state": self.door_state, "stable_samples": 4, "uncertainty_m": 0.015}}

    def value(self, tool, **args):
        self.calls.append((tool, args))
        if tool == "evo_capabilities":
            return self.capabilities
        if tool == "rax_scene":
            return self.scene()
        if tool == "evo_status":
            return {"feedback_fresh": True, "motion_blocked": False, "state": {"positions": self.joints.copy()}}
        if tool == "evo_fk":
            side = args["links"][0].removesuffix("_palm")
            p = self.palms[side]
            xyz = p["position"].copy()
            xyz[2] += args.get("joints", {}).get("carriage_joint", self.joints["carriage_joint"])-self.joints["carriage_joint"]
            return {"poses": {side+"_palm": {"position": xyz, "quaternion_xyzw": p["quaternion_xyzw"]}}}
        if tool == "rax_base_preview":
            return {"valid": True}
        if tool == "evo_preview":
            action = args["action"]
            if action["kind"] == "follow_path":
                if any(s in p for s in self.preview_refused for p in action["waypoints"]):
                    return {"valid": False}
                points = [{"positions": {**posture("left"), **posture("right")}} for _ in action["waypoints"]]
            else:
                points = []
            return {"valid": True, "plan": {"points": points}}
        if tool == "rax_detect_buttons":
            self.detect_count += 1
            side = args["camera"].removesuffix("_arm")
            stamp = time.time_ns()//1000
            camera = {"capture_us": stamp, "geometry_validated": self.calibrated, "width": 640, "height": 480,
                      "jpeg": "/9j/2Q=="}
            detection = {"text": "2", "text_readable": True, "score": .9, "box": [300, 220, 340, 260]}
            detections = [detection] * (2 if self.duplicate_label else 1) if self.present[side] else []
            self.capture = {"observation_id": f"detect-{self.detect_count}", "cameras": {args["camera"]: camera},
                            "scene": self.scene(stamp), "detections": detections}
            return self.capture
        if tool == "rax_camera_ray":
            assert args["observation_id"] == self.capture["observation_id"]
            camera = self.capture["cameras"][args["camera"]]
            point = self.local((1., 0., 1.))
            return {"capture_us": camera["capture_us"]-int(self.ray_bad_stamp), "frame": "base_link",
                    "origin": [0., 0., 1.], "direction": [point[0], point[1], 0.],
                    "surface": {"capture_us": self.capture["scene"]["capture_us"], "support_points": 8,
                                "position": point}}
        if tool == "evo_action_status":
            return self.last_result.copy()
        if tool == "evo_cancel":
            self.last_result.update(state="cancelled", reason="cancelled")
            return {"state": "stopping"}
        if tool.startswith("evo_move_") or tool in {"evo_follow_path", "rax_move_base"}:
            action_id = ("rax-base-" if tool == "rax_move_base" else "action-") + str(len(self.calls))
            self.last_result = {"action_id": action_id, "state": "succeeded", "physical_stop_verified": True}
            if self.fail_action == tool:
                self.last_result.update(state="failed", reason="injected failure", physical_stop_verified=False)
                return {"action_id": action_id, "state": "accepted"}
            if tool == "evo_move_joints":
                self.joints.update(args["targets"])
            if tool == "evo_move_carriage":
                delta = args["position_m"]-self.joints["carriage_joint"]
                self.joints["carriage_joint"] += delta
                for p in self.palms.values():
                    p["position"][2] += delta
            if tool == "evo_follow_path":
                for p in args["waypoints"]:
                    for side in self.palms:
                        if side in p:
                            self.palms[side] = {"position": list(p[side]["position"]),
                                                "quaternion_xyzw": list(p[side]["quaternion_xyzw"])}
            if tool == "rax_move_base":
                dx, dy, angle = args["delta"]
                self.base_calls.append(args)
                if self.close_on_crossing and math.hypot(dx, dy) > 0.6:
                    self.last_result.update(state="failed", reason="doorway closing")
                else:
                    c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
                    self.pose = [self.pose[0]+c*dx-s*dy, self.pose[1]+s*dx+c*dy, self.pose[2]+angle]
                self.last_result["base_result"] = {"residual": [0, 0, 0], "distance_travelled_m": math.hypot(dx, dy),
                                                   "angle_travelled_rad": abs(angle), "crossing_state": "after"}
            return {"action_id": action_id, "state": "accepted"}
        raise AssertionError((tool, args))


@pytest.mark.parametrize("optimized", [False, True])
def test_complete_press_uses_fresh_views_and_reports_unconfirmed_activation(optimized):
    context = RobotContext()
    task = Elevator(context, settings(synchronized_view=optimized, direct_press_alignment=optimized))
    result = task.press_elevator_button("2")
    assert result.outcome == "stroke_complete_unconfirmed"
    assert result.measured["activation"] == "unknown"
    assert task._contact is None
    assert context.detect_count > 4
    assert context.joints == context.scene_cfg["travel_joints"]
    side = result.measured["arm"]
    assert context.palms[side]["position"][0] == pytest.approx(0.75-settings().retract_gap_m)
    assert not any(t == "evo_drive" for t, _ in context.calls)


def test_arm_selection_checks_approach_press_and_retract_and_rejects_unreachable_arm():
    context = RobotContext()
    context.preview_refused.add("left")
    task = Elevator(context, settings())
    views = task.search_button("2")
    before = len(context.calls)
    choice = task.select_press_arm(views)
    assert choice.side == "right"
    previews = [a["action"] for t, a in context.calls[before:] if t == "evo_preview"]
    assert all(len(p["waypoints"]) == 3 for p in previews)
    assert not any(t.startswith("evo_move") for t, _ in context.calls[before:])
    context.preview_refused.add("right")
    assert task.select_press_arm(views).outcome == "reposition_required"


def test_wrist_search_finishes_both_postures_before_captures():
    context = RobotContext()
    task = Elevator(context, settings())
    original = task.observe_button
    initial = 0
    def find_after_first_station(label, side):
        nonlocal initial
        initial += 1
        return None if initial <= 2 else original(label, side)
    task.observe_button = find_after_first_station
    views = task.search_button("2")
    assert len(views) == 2
    assert all(v.capture_us > task._last_stop_us for v in views)
    assert task.select_press_arm(views).outcome == "selected"


@pytest.mark.parametrize("attribute", ["ray_bad_stamp", "duplicate_label"])
def test_invalid_or_ambiguous_capture_cannot_be_selected(attribute):
    context = RobotContext()
    setattr(context, attribute, True)
    task = Elevator(context, settings())
    if attribute == "ray_bad_stamp":
        with pytest.raises(PrimitiveError, match="ray"):
            task.observe_button("2", "left")
    else:
        assert task.observe_button("2", "left") is None


def test_missing_calibration_and_stale_cloud_refuse_before_motion():
    context = RobotContext()
    task = Elevator(context, settings())
    context.calibrated = False
    with pytest.raises(PrimitiveError, match="calibration"):
        task.observe_button("2", "left")
    context.stale = True
    with pytest.raises(PrimitiveError, match="stale"):
        task.base.align_base_angle()
    assert not context.base_calls


def test_no_arm_motion_or_second_press_is_allowed_in_contact_stage():
    context = RobotContext()
    task = Elevator(context, settings())
    choice = task.select_press_arm(task.search_button("2"))
    side = choice.side
    task.prepare_press_arm(choice)
    task.align_arm_to_button("2", side)
    task.approach_button("2", side)
    with pytest.raises(PrimitiveError, match="retract"):
        task.stow_arms()
    with pytest.raises(PrimitiveError, match="retract"):
        task.align_carriage_height("2", side)
    task.press_button("2", side)
    with pytest.raises(PrimitiveError, match="repeated"):
        task.press_button("2", side)
    task.retract_arm(side)
    assert task._contact is None


def test_target_lost_during_approach_stops_sequence():
    context = RobotContext()
    task = Elevator(context, settings())
    choice = task.select_press_arm(task.search_button("2"))
    task.prepare_press_arm(choice)
    context.present[choice.side] = False
    with pytest.raises(PrimitiveError, match="lost"):
        task.approach_button("2", choice.side)
    assert task._contact is None


@pytest.mark.parametrize("direct", [False, True])
def test_enter_and_exit_face_the_same_doorway_from_opposite_sides(direct):
    context = RobotContext()
    task = Elevator(context, replace(settings(), direct_doorway_staging=direct))
    assert task.navigate_into_elevator(goal_depth_m=.5).measured["location"] == "inside"
    assert context.pose[0] == pytest.approx(1.5)
    assert task.navigate_out_of_elevator(goal_depth_m=.5).measured["location"] == "outside"
    assert context.pose[0] == pytest.approx(.5)
    assert abs(context.pose[2]) == pytest.approx(math.pi)
    assert all(call.get("doorway_id") == "door-1" for call in context.base_calls)


def test_closed_door_never_dispatches_navigation():
    context = RobotContext()
    context.door_state = "closing"
    base = Base(context)
    with pytest.raises(PrimitiveError, match="timeout"):
        base.wait_for_door_open(timeout_s=.001)
    assert not context.base_calls


def test_late_closure_prevents_reported_arrival():
    context = RobotContext()
    context.close_on_crossing = True
    task = Elevator(context, settings())
    with pytest.raises(EvoActionFailed, match="closing"):
        task.navigate_into_elevator(goal_depth_m=.5)
    assert context.pose[0] < 1.


def test_stale_capture_after_base_motion_is_rejected():
    context = RobotContext()
    task = Elevator(context, settings())
    observation = task.observe_button("2", "left")
    context.pose[0] = .1
    with pytest.raises(PrimitiveError, match="base moved"):
        task._fresh_button(observation)


def test_unverified_stop_aborts_the_task():
    context = RobotContext()
    context.fail_action = "evo_move_joints"
    task = Elevator(context, settings())
    with pytest.raises(EvoActionFailed):
        task.press_elevator_button("2")
    assert context.detect_count == 0
    assert not context.base_calls


def test_cancellation_requires_verified_stop_even_if_action_just_finished():
    context = RobotContext()
    action = context.value("evo_move_joints", targets={})
    with pytest.raises(EvoActionFailed, match="cancelled") as error:
        Evo(context).wait(action, cancelled=lambda: True)
    assert error.value.result["physical_stop_verified"] is True
    assert not any(t.startswith("evo_move") for t, _ in context.calls[1:])


def test_cancel_before_preview_dispatches_nothing():
    context = RobotContext()
    base = Base(context, cancelled=lambda: True)
    context.calls.clear()
    with pytest.raises(PrimitiveError, match="cancelled"):
        base.translate_base(.1, 0)
    assert context.calls == []


def test_elbow_branch_and_invalid_profile_refused():
    profile = settings().arms[0]
    with pytest.raises(ValueError, match="bend"):
        replace(profile, view_joints=posture("left", 0)).validate()
    with pytest.raises(ValueError, match="quaternion"):
        replace(profile, press_quaternion_xyzw=(0, 0, 0, 0)).validate()


def test_cancel_active_action_sends_stop_and_waits_for_verification():
    context = RobotContext()
    action = context.value("evo_move_joints", targets={})
    context.last_result["state"] = "running"
    with pytest.raises(EvoActionFailed, match="cancelled"):
        Evo(context).wait(action, cancelled=lambda: True)
    assert any(t == "evo_cancel" for t, _ in context.calls)


def test_failed_approach_blocks_further_motion():
    context = RobotContext()
    task = Elevator(context, settings())
    choice = task.select_press_arm(task.search_button("2"))
    task.prepare_press_arm(choice)
    task.align_arm_to_button("2", choice.side)
    context.fail_action = "evo_follow_path"
    with pytest.raises(EvoActionFailed):
        task.approach_button("2", choice.side)
    assert task._contact["stage"] == "approaching"
    with pytest.raises(PrimitiveError, match="retract"):
        task.stow_arms()


def test_plane_without_local_press_depth_is_refused():
    context = RobotContext()
    original = context.value
    def missing_surface(tool, **args):
        result = original(tool, **args)
        if tool == "rax_camera_ray":
            result.pop("surface")
        return result
    context.value = missing_surface
    with pytest.raises(PrimitiveError, match="local depth"):
        Elevator(context, settings()).observe_button("2", "left")


def test_press_entry_and_exit_through_real_rax_frontend_and_adapter(tmp_path, monkeypatch):
    import socket
    original_connect = socket.socket.connect

    def local_only(connection, address):
        assert connection.family == socket.AF_UNIX, "task attempted a network connection"
        return original_connect(connection, address)

    monkeypatch.setattr(socket.socket, "connect", local_only)
    from auto_research_agent.api.runtime import Context
    from local_tests.unit.test_evo_v3_backend import service, frontend_socket

    robot = RobotContext()
    goal = {}
    def handle(request):
        method, args = request["method"], request["params"]
        if method == "elevator":
            operation, params = args["operation"], args["params"]
            if operation in {"scene", "detect", "ray", "preview"}:
                tool = {"scene":"rax_scene", "detect":"rax_detect_buttons", "ray":"rax_camera_ray",
                        "preview":"rax_base_preview"}[operation]
                return robot.value(tool, **params)
            if operation == "begin":
                goal.update(params=params, done=False)
                return {"goal_id": "evo-goal"}
            if operation == "step":
                if goal["done"]:
                    return {"done": True, "base_result": robot.last_result["base_result"]}
                return {"velocity": [.01, 0., 0.], "duration_s": .06,
                        "valid_until_us": time.time_ns()//1000 + 150_000}
            assert operation == "abort"
            return {"aborted": True}
        if method == "drive":
            accepted = robot.value("rax_move_base", **goal["params"])
            robot.last_result["final_state"] = {"source_us": time.time_ns()//1000}
            goal["done"] = True
            return accepted
        return robot.value("evo_" + method, **args)

    with service(tmp_path, handle) as (client, received):
        with frontend_socket(client, tmp_path) as socket_path, Context(str(socket_path)) as context:
            task = Elevator(context, settings())
            assert task.press_elevator_button("2").outcome == "stroke_complete_unconfirmed"
            assert task.navigate_into_elevator(goal_depth_m=.5).outcome == "succeeded"
            assert task.navigate_out_of_elevator(goal_depth_m=.5).outcome == "succeeded"
    assert any(r["method"] == "elevator" and r["params"]["operation"] == "detect" for r in received)
    assert any(r["method"] == "drive" and "valid_until_us" in r["params"] for r in received)
    import json
    frames = list((tmp_path / "agent-workspace/runtime/frames").iterdir())
    assert frames
    for frame in frames:
        metadata = json.loads((frame / "meta.json").read_text())
        assert metadata["observation_id"].startswith("detect-")
        for camera in metadata["cameras"]:
            assert (frame / (camera+".jpg")).read_bytes() == b"\xff\xd8\xff\xd9"


def test_elevator_api_and_scene_reads_reach_generated_workspace(tmp_path):
    from services.workspace import Setup, build, recipe_for
    import json

    workspace = tmp_path / "agent"
    build(recipe_for(), Setup("evo-v3", "elevator", "claude", "rgb-only"), workspace)
    assert (workspace / "api/elevator.py").is_file()
    assert (workspace / "api/base.py").is_file()
    contract = json.loads((workspace / "MCP_CONTRACT.json").read_text())
    from services.evo_v3.backend import CLIENT_TOOLS
    from services.workspace import ROBOT_FACTS

    names = {tool["name"] for tool in CLIENT_TOOLS}
    assert names == set(ROBOT_FACTS["evo-v3"]["tools"])
    assert names <= set(contract["tools"])
    assert {"rax_move_base", "rax_scene", "rax_detect_buttons", "evo_sam_result", "evo_sam_cancel"} <= names
    assert {"rax_scene", "rax_detect_buttons", "evo_sam_result", "evo_sam_cancel"} <= set(ROBOT_FACTS["evo-v3"]["read_tools"])
    interface = (workspace / ".agents/skills/robot-interface/SKILL.md").read_text()
    assert "## Elevator primitives" in interface and "## SAM perception" in interface


@pytest.mark.parametrize("timeout", [True, 0, -1, float("inf"), float("nan"), "1", None])
@pytest.mark.parametrize("method", ["execute", "wait"])
def test_invalid_wait_timeout_cannot_dispatch_or_query_motion(timeout, method):
    context = RobotContext()
    evo = Evo(context)
    with pytest.raises(ValueError, match="finite and positive"):
        if method == "execute":
            evo.execute("evo_move_joints", targets={"left_joint1": .1}, wait_timeout_s=timeout)
        else:
            evo.wait({"action_id": "existing"}, timeout_s=timeout)
    assert not context.calls


def test_direct_doorway_staging_uses_bounded_previewed_diagonal_moves():
    context = RobotContext()
    context.pose = [0., -.2, 0.]
    Base(context).align_to_doorway(staging_gap_m=.5, direct=True)
    assert context.pose == pytest.approx([.5, 0., 0.])
    assert all(math.hypot(*c["delta"][:2]) <= .15+1e-12 for c in context.base_calls)
    assert all(c["doorway_id"] == "door-1" for c in context.base_calls)
    assert context.base_calls[0]["delta"][0] > 0 and context.base_calls[0]["delta"][1] > 0
    assert len([t for t, _ in context.calls if t == "rax_base_preview"]) == len(context.base_calls)


def test_direct_staging_stops_on_clearance_refusal():
    class Blocked(RobotContext):
        def value(self, tool, **args):
            if tool == "rax_base_preview":
                return {"valid": False}
            return super().value(tool, **args)
    context = Blocked()
    context.pose = [0., -.2, 0.]
    with pytest.raises(PrimitiveError, match="preview refused"):
        Base(context).align_to_doorway(staging_gap_m=.5, direct=True)
    assert not context.base_calls


def test_direct_staging_retains_total_alignment_budget():
    context = RobotContext()
    context.pose = [-2., -.2, 0.]
    with pytest.raises(PrimitiveError, match="travel budget"):
        Base(context).align_to_doorway(staging_gap_m=.5, direct=True)
    assert sum(math.hypot(*c["delta"][:2]) for c in context.base_calls) <= 1.


@pytest.mark.parametrize("direct", [False, True])
def test_reverse_exit_retains_entry_heading_and_checked_rear_motion(direct):
    context = RobotContext()
    task = Elevator(context, settings(direct_doorway_staging=direct, reverse_exit=True))
    assert task.navigate_into_elevator(goal_depth_m=.5).measured["location"] == "inside"
    first_exit_call = len(context.base_calls)
    assert task.navigate_out_of_elevator(goal_depth_m=.5).measured["location"] == "outside"
    assert context.pose == pytest.approx([.5, 0., 0.])
    assert all(call["delta"][0] < 0 and abs(call["delta"][2]) < 1e-12
               for call in context.base_calls[first_exit_call:])
    assert all(call["doorway_id"] == "door-1" for call in context.base_calls)
    assert len([tool for tool, _ in context.calls if tool == "rax_base_preview"]) == len(context.base_calls)


def test_reverse_exit_refuses_missing_rear_clearance_before_motion():
    class RearBlocked(RobotContext):
        def value(self, tool, **args):
            if tool == "rax_base_preview" and args["delta"][0] < 0:
                return {"valid": False}
            return super().value(tool, **args)
    context = RearBlocked()
    context.pose = [1.5, 0., 0.]
    task = Elevator(context, settings(direct_doorway_staging=True, reverse_exit=True))
    with pytest.raises(PrimitiveError, match="preview refused"):
        task.navigate_out_of_elevator(goal_depth_m=.5)
    assert not context.base_calls


def test_reverse_exit_door_closure_cannot_report_arrival():
    context = RobotContext()
    context.pose = [1.5, 0., 0.]
    context.close_on_crossing = True
    task = Elevator(context, settings(direct_doorway_staging=True, reverse_exit=True))
    with pytest.raises(EvoActionFailed, match="closing"):
        task.navigate_out_of_elevator(goal_depth_m=.5)
    assert context.pose[0] > 1.


def test_reverse_exit_option_requires_boolean_before_motion():
    context = RobotContext()
    with pytest.raises(ValueError, match="boolean"):
        Elevator(context, settings(reverse_exit="yes"))
    with pytest.raises(ValueError, match="boolean"):
        Base(context).align_to_doorway(staging_gap_m=.5, reverse=1)
    assert not context.base_calls


@pytest.mark.parametrize("name", ["synchronized_view", "direct_press_alignment"])
def test_press_timing_options_require_booleans(name):
    with pytest.raises(ValueError, match="boolean"):
        settings(**{name: 1}).validate()


def test_synchronized_view_requires_one_preview_and_all_measured_endpoints():
    context = RobotContext()
    task = Elevator(context, settings(synchronized_view=True, viewing_carriage_m=.2))
    task.prepare_button_view()
    moves = [(t, a) for t, a in context.calls if t.startswith("evo_move_")]
    assert len(moves) == 1
    assert moves[0][0] == "evo_move_joints"
    assert set(moves[0][1]["targets"]) == set(context.joints)
    assert context.joints["carriage_joint"] == .2
    assert len([t for t, _ in context.calls if t == "evo_preview"]) == 1
    assert task.evidence[-1]["physical_stop_verified"]


@pytest.mark.parametrize("fault", ["preview", "stop", "carriage", "arm"])
def test_synchronized_view_refuses_without_capture_or_fallback(fault):
    context = RobotContext()
    original = context.value
    def value(tool, **args):
        result = original(tool, **args)
        if fault == "preview" and tool == "evo_preview":
            result["valid"] = False
        if tool == "evo_move_joints":
            if fault == "stop":
                context.last_result.update(state="failed", physical_stop_verified=False)
            elif fault == "carriage":
                context.joints["carriage_joint"] = .1
            elif fault == "arm":
                context.joints["left_joint4"] = 0.
        return result
    context.value = value
    task = Elevator(context, settings(synchronized_view=True, viewing_carriage_m=.2))
    with pytest.raises(ValueError if fault == "preview" else (PrimitiveError, EvoActionFailed)):
        task.search_button("2")
    assert context.detect_count == 0
    assert not any(t == "evo_move_carriage" for t, _ in context.calls)


@pytest.mark.parametrize("direct", [False, True])
def test_direct_press_alignment_preserves_gap_and_full_sequence(direct):
    context = RobotContext()
    task = Elevator(context, settings(direct_press_alignment=direct, synchronized_view=True))
    choice = task.select_press_arm(task.search_button("2"))
    gap_before = 1.-task._tip(choice.side)[0]
    before = len(context.calls)
    task.prepare_press_arm(choice)
    if not direct:
        task.align_arm_to_button("2", choice.side)
    paths = [a for t, a in context.calls[before:] if t == "evo_follow_path"]
    assert len(paths) == (1 if direct else 2)
    assert 1.-task._tip(choice.side)[0] == pytest.approx(gap_before)
    assert task._tip(choice.side)[1:] == pytest.approx((0., 1.))
    task.approach_button("2", choice.side)
    task.press_button("2", choice.side)
    task.retract_arm(choice.side)
    task.stow_arms()
    assert task._contact is None
    assert context.joints == context.scene_cfg["travel_joints"]


def test_direct_preparation_keeps_orientation_sweep_refusal_before_carriage():
    context = RobotContext()
    task = Elevator(context, settings(direct_press_alignment=True))
    choice = task.select_press_arm(task.search_button("2"))
    context.palms[choice.side]["position"][0] = .98
    before = len(context.calls)
    with pytest.raises(PrimitiveError, match="insufficient gap"):
        task.prepare_press_arm(choice)
    assert not any(t.startswith("evo_move_") or t == "evo_follow_path" for t, _ in context.calls[before:])


def test_direct_preparation_rechecks_gap_after_carriage_alignment():
    context = RobotContext()
    task = Elevator(context, settings(direct_press_alignment=True))
    choice = task.select_press_arm(task.search_button("2"))
    def aligned(label, side):
        context.palms[side]["position"][0] = .98
        return task._look(label, side)
    task.align_carriage_height = aligned
    before = len(context.calls)
    with pytest.raises(PrimitiveError, match="insufficient gap"):
        task.prepare_press_arm(choice)
    assert not any(t == "evo_follow_path" for t, _ in context.calls[before:])


def low_confidence_context(scores):
    context = RobotContext()
    original = context.value
    remaining = iter(scores)
    def value(tool, **arguments):
        result = original(tool, **arguments)
        if tool == 'rax_detect_buttons':
            for detection in result['detections']:
                detection['score'] = next(remaining)
        return result
    context.value = value
    return context


def test_one_stationary_reacquisition_keeps_confidence_and_no_motion():
    context = low_confidence_context([.52, .9, .52])
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    start = len(context.calls)
    assert task._look('2', 'left').score == .9
    assert context.detect_count == 2
    assert not any(tool in ('evo_move_joints','evo_follow_path','evo_preview') for tool, _ in context.calls[start:])
    assert next(e for e in task.evidence if e.get('stage') == 'low_confidence_reacquisition')['failed_capture']['score'] == .52
    with pytest.raises(PrimitiveError, match='lost'):
        task._look('2', 'left')
    assert context.detect_count == 3  # One retry total, not one per observation.


@pytest.mark.parametrize('mode', ['default', 'no_stop', 'press_attempted', 'guarded_attempted', 'missing', 'ambiguous'])
def test_stationary_reacquisition_never_overrides_existing_refusals(mode):
    context = low_confidence_context([.52, .52])
    task = Elevator(context, settings(reacquire_low_confidence_once=mode != 'default'))
    if mode != 'no_stop':
        task.prepare_button_view(['left'])
    if mode in ('press_attempted','guarded_attempted'):
        task._contact = dict(stage=mode, side='left', label='2')
    if mode == 'missing':context.present['left'] = False
    if mode == 'ambiguous':context.duplicate_label = True
    with pytest.raises(PrimitiveError, match='lost'):
        task._look('2', 'left')
    assert context.detect_count == 1


def test_reacquisition_rejects_reused_capture_identity():
    context = low_confidence_context([.52, .9])
    original = context.value
    def value(tool, **arguments):
        result = original(tool, **arguments)
        if tool == 'rax_detect_buttons':result['observation_id'] = 'same-capture'
        return result
    context.value = value
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    with pytest.raises(PrimitiveError, match='new capture'):
        task._look('2', 'left')


def test_reacquisition_does_not_catch_bad_geometry():
    context = low_confidence_context([.52, .9])
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    context.calibrated = False
    with pytest.raises(PrimitiveError, match='calibration'):
        task._look('2', 'left')
    assert context.detect_count == 1


@pytest.mark.parametrize('stage', ['approaching', 'prepress'])
def test_reacquisition_can_observe_unpressed_approach_without_moving(stage):
    context = low_confidence_context([.52, .9])
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    task._contact = dict(stage=stage, side='left', label='2', pose=context.pose.copy(), carriage=0.)
    before = len(context.calls)
    assert task._look('2', 'left').score == .9
    assert task._contact['stage'] == stage
    assert not any(tool in ('evo_move_joints','evo_follow_path','evo_preview') for tool, _ in context.calls[before:])


def test_reacquisition_refuses_if_base_moved_during_approach():
    context = low_confidence_context([.52, .9])
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    task._contact = dict(stage='approaching', side='left', label='2', pose=[.1,0,0], carriage=0.)
    with pytest.raises(PrimitiveError, match='base moved'):
        task._look('2', 'left')
    assert context.detect_count == 1


def test_second_low_confidence_capture_refuses_without_another_attempt():
    context = low_confidence_context([.52, .59])
    task = Elevator(context, settings(reacquire_low_confidence_once=True))
    task.prepare_button_view(['left'])
    with pytest.raises(PrimitiveError, match='lost'):
        task._look('2', 'left')
    assert context.detect_count == 2


@pytest.mark.parametrize('age_s, accepted', [(0.1, True), (0.6, False), (-0.1, False)])
def test_base_capture_freshness_uses_injected_clock(age_s, accepted):
    class CaptureClock:
        def time(self):
            return 10.0
    context = RobotContext()
    snapshot = context.scene(stamp=round((10.0-age_s)*1e6))
    context.scene = lambda: snapshot
    base = Base(context, clock=CaptureClock())
    if accepted:
        assert base.observe_scene()['capture_us'] == snapshot['capture_us']
    else:
        with pytest.raises(PrimitiveError, match='stale or invalid'):
            base.observe_scene()
    assert not context.base_calls
