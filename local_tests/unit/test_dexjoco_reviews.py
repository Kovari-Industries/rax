"""DexJoCo episode reviews: the ground-truth report from the simulator's goal report, the shared
review lifecycle on a DexJoCo session, the workspace text, and what the simulator records.

No simulator or network: goal reports are synthetic and Gemini is a fake router.
"""
import io
import json
import math
import os
import re
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from services.benchmarks import BENCHMARKS
from services.dexjoco import analysis as A
from services.dexjoco import objects as O
from services.episode_reviews import runner

PROJECT = Path(__file__).resolve().parents[2]
IDENTITY = [1.0, 0.0, 0.0, 0.0]
ROBOT = [0.0] * 23


def yaw(degrees):
    half = math.radians(degrees) / 2
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def box(lo, hi):
    """The eight corners of an axis-aligned box, bottom ones at even indices (as bucket_ref_0..7)."""
    return [v for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2]) for v in (x, y, z)]


def bucket_report(**changes):
    """A pick_bucket episode the agent closed early: the bucket was slid and barely lifted, the
    boxed food never reached it."""
    report = {
        "schema": "dexjoco_goal_report_v1", "episode_id": "pick_bucket_exploration_3_ab12cd34", "task": "pick_bucket",
        "env_name": "pick_bucket", "instruction": "Place the boxed food into the bucket and then lift the bucket.",
        "episode_mode": "exploration", "reason": "close", "step_id": 400, "step_limit": 1200, "success": False,
        "terminated": False, "env_info": {"succeed": False, "grasp_penalty": 0.0},
        "initial_env_info": {"succeed": False}, "info_history": {"ever_true": [], "sequences": {}},
        "state_dim": 23,
        "initial_state": ROBOT + [0.40, -0.10, 0.95, *IDENTITY] + [0.30, -0.10, 0.95, *yaw(10)] + [0.031],
        "initial_live": {"bucket": [0.40, -0.10, 0.95, *IDENTITY], "boxed food": [0.30, -0.10, 0.95, *yaw(10)],
                         "boxed food centre": [0.30, -0.10, 0.97],
                         "bucket corners": box((0.32, -0.20, 0.88), (0.48, 0.0, 1.08))},
        "live": {"bucket": [0.50, -0.10, 0.97, *yaw(20)], "boxed food": [0.30, -0.10, 0.95, *yaw(10)],
                 "boxed food centre": [0.30, -0.10, 0.97],
                 "bucket corners": box((0.42, -0.20, 0.90), (0.58, 0.0, 1.10))}}
    return {**report, **changes}


def sim_dir(root, report, name="exploration/episode-001", mode=None, task_rules="code-summary"):
    """A closed episode's simulator files; the session's TASK.md states the rules (code-summary) unless
    task_rules says otherwise."""
    if not (root / "configuration.json").exists():
        root.mkdir(parents=True, exist_ok=True)
        (root / "configuration.json").write_text(json.dumps({"benchmark": "dexjoco", "task_rules": task_rules}))
    sim = root / name / "native/results/pick_bucket_exploration_20261010T000000Z_ab12cd34/sim"
    sim.mkdir(parents=True)
    mode = mode or report.get("episode_mode", "exploration")
    (sim / "summary.json").write_text(json.dumps({"episode_id": report["episode_id"], "step_id": report.get("step_id"),
                                                  "success": report.get("success"), "reason": report.get("reason"),
                                                  "episode_mode": mode}))
    (sim / "operator_state.json").write_text(json.dumps({"episode_id": report["episode_id"], "episode_mode": mode}))
    (sim / "private_goal_report.json").write_text(json.dumps({**report, "episode_mode": mode}))
    return sim


# ------------------------------------------------------------------ report
def test_report_states_the_rules_conditions_with_what_was_measured(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path, bucket_report()))
    assert report["warnings"] == []
    text = A.report_text(report)
    assert report["summary"] == text  # What the operator's dashboard shows beside the agent's review.
    assert text.startswith("Outcome: no success after 400 of 1200 steps (the agent closed it with `finish` before "
                           "the environment ended it).")
    assert ("Success requires, at the same step:\n"
            "- The boxed food's centre is inside the bucket (within the box spanned by the bucket's corner markers, "
            "which turns with the bucket).\n"
            "  At the end: the boxed food's centre ended 12 cm outside the box spanned by the bucket's corner markers.\n"
            "- The bucket's bottom is raised a set height above where it started.\n"
            "  At the end: the bucket's bottom ended 2.0 cm higher than at the start (its lowest bottom corner).") in text
    assert (f"How the task objects moved, from their reset poses to the end ({A.SETTLING}):\n"
            "- bucket: 10 cm horizontally, 2.0 cm up, turned 20 degrees about the vertical\n"
            "- boxed food: did not move") in text
    # Relative measurements only: no coordinates, table height or layout identity.
    for private in ("0.4", "0.95", "0.031", "3.1 cm", "seed", "layout"):
        assert private not in text
    assert report["private"]["table_height_offset_m"] == pytest.approx(0.031)
    assert report["private"]["start_poses"]["bucket"][:3] == [0.40, -0.10, 0.95]


def test_public_sessions_keep_the_conditions_for_the_operator(tmp_path):
    """With task_rules public, TASK.md gives only the prompt: the agent's review and the analyst's
    input state the outcome and the movement, never the conditions; the operator still sees them."""
    report = A.ground_truth_report(sim_dir(tmp_path, bucket_report(), task_rules="public"))
    text = A.report_text(report)
    assert report["rules_disclosed"] is False and "Success requires" not in text and "corner markers" not in text
    assert text.startswith("Outcome: no success after 400 of 1200 steps") and "How the task objects moved" in text
    assert "Success requires, at the same step:" in report["summary"]


def test_a_centre_inside_the_markers_and_the_random_variant(tmp_path):
    raw = bucket_report(task="pick_bucket_random")
    raw["live"] = {**raw["live"], "boxed food centre": [0.50, -0.12, 0.95]}
    report = A.ground_truth_report(sim_dir(tmp_path, raw))
    inside = report["conditions"]["success"]["conditions"][0]["measured"]
    assert inside == ("the boxed food's centre ended inside the box spanned by the bucket's corner markers "
                      "(5.0 cm from its nearest side)")
    assert report["conditions"]["success"]["heading"] == "Success requires, at the same step"
    raw["live"] = {**raw["live"], "bucket corners": raw["initial_live"]["bucket corners"]}
    still = A.ground_truth_report(sim_dir(tmp_path / "still", raw))
    assert still["conditions"]["success"]["conditions"][1]["measured"] == "the bucket's bottom was not raised"


def test_failure_conditions_and_the_hanoi_towers(tmp_path):
    raw = {"schema": "dexjoco_goal_report_v1", "episode_id": "bimanual_hanoi_exploration_0_x", "task": "bimanual_hanoi",
           "env_name": "bimanual_hanoi", "episode_mode": "exploration", "reason": "terminal", "step_id": 300,
           "step_limit": 1500, "success": False, "terminated": True, "state_dim": 46,
           "initial_state": [0.0] * 46 + [-0.15, -0.1, 0.94, 0.02],
           "initial_live": {"tower base": [-0.15, -0.1, 0.94, *IDENTITY]},
           "live": {"tower base": [-0.15, -0.1, 0.94, *IDENTITY]},
           "initial_env_info": {"tower_state": {"A": ["hanoi_disk_small"], "B": ["hanoi_disk_medium"],
                                                "C": ["hanoi_disk_large"]}},
           "env_info": {"succeed": False, "illegal_state": True, "illegal_reason": "illegal_stack_C",
                        "tower_state": {"A": [], "B": [], "C": ["hanoi_disk_large", "hanoi_disk_small",
                                                                "hanoi_disk_medium"]},
                        "disk_assignment": {"hanoi_disk_large": "C", "hanoi_disk_small": "C", "hanoi_disk_medium": "C"},
                        "all_disks_static": True, "xy_error_max": 0.0, "height_error_max": 0.0},
           "info_history": {"ever_true": ["illegal_state"], "sequences": {}}}
    report = A.ground_truth_report(sim_dir(tmp_path, raw))
    text = A.report_text(report)
    assert "(a failure condition ended it)" in text
    assert ("  At the end: left peg: empty; middle peg: empty; right peg: large disk, small disk, medium "
            "disk (bottom to top); all disks still. At the start: left peg: small disk; middle peg: medium disk; "
            "right peg: large disk (bottom to top).") in text
    assert ("Fails the episode if, held for a number of consecutive steps while the disks are still:\n"
            "- A larger disk rests on a smaller one, or a disk lies on the table off the pegs.\n"
            "  This episode: a larger disk rested on a smaller one on the right peg, which ended the episode.") in text
    assert "- tower base: did not move" in text and "hanoi_disk" not in text


def test_a_wrong_ipad_digit_and_the_step_limit(tmp_path):
    ipad = {"episode_id": "bimanual_unlock_ipad_exploration_1_x", "task": "bimanual_unlock_ipad",
            "env_name": "bimanual_unlock_ipad", "episode_mode": "exploration", "reason": "close", "step_id": 200,
            "step_limit": 1200, "success": False, "terminated": True, "state_dim": 46,
            "initial_state": [0.0] * 46 + [-0.2, 0.0, 0.94, *IDENTITY, -0.2, 0.0, 1.0, *IDENTITY, 0.01],
            "live": {"stand": [-0.2, 0.0, 0.94, *IDENTITY], "iPad": [-0.1, 0.0, 1.10, *IDENTITY]},
            "env_info": {"succeed": False, "pressed_digits": [3]},
            "info_history": {"ever_true": [], "sequences": {"pressed_digits": [1, 3]}}}
    text = A.report_text(A.ground_truth_report(sim_dir(tmp_path, ipad)))
    assert "  At the end: digits pressed, in order: 1, 3." in text
    assert "  This episode: a wrong digit was pressed, which ended the episode." in text
    assert "- iPad: 10 cm horizontally, 10 cm up" in text and "- stand: did not move" in text

    photo = {"episode_id": "bimanual_photograph_exploration_2_x", "task": "bimanual_photograph_random",
             "env_name": "bimanual_photograph", "episode_mode": "exploration", "reason": "close", "step_id": 1000,
             "step_limit": 1000, "success": False, "terminated": True, "state_dim": 46,
             "initial_state": [0.0] * 46 + [0.1, 0.3, 0.95, *IDENTITY, 0.5, 0.0, 1.3, *IDENTITY, 0.02],
             "live": {"camera": [0.1, 0.3, 0.95, *IDENTITY]},
             "env_info": {"succeed": False, "region_pass": False, "angle_pass": True, "shutter_pressed": False,
                          "radial_dist": 0.21, "region_height_dist": 0.03, "view_angle_deg": 7.2},
             "info_history": {"ever_true": ["region_pass", "angle_pass"], "sequences": {}}}
    report = A.ground_truth_report(sim_dir(tmp_path, photo, name="exploration/episode-002"))
    text = A.report_text(report)
    assert "(the step limit ended it)" in text
    measured = [c["measured"] for c in report["conditions"]["success"]["conditions"]]
    assert measured == [
        "not met at the end; met at some earlier step (the lens was 21 cm from the region's axis and 3.0 cm from "
        "its middle along it)",
        "met at the end (the camera pointed 7 degrees away from facing the logo)",
        "not pressed at the end; never pressed"]
    assert "logo" not in "\n".join(report["movement"])  # Fixed on the wall: never a moved object.


def test_success_with_the_environments_info(tmp_path):
    raw = {"episode_id": "hammer_nail_exploration_0_x", "task": "hammer_nail", "env_name": "hammer_nail",
           "episode_mode": "exploration", "reason": "terminal", "step_id": 350, "step_limit": 1000, "success": True,
           "terminated": True, "state_dim": 23,
           "initial_state": ROBOT + [-0.3, -0.45, 0.95, *IDENTITY, -0.05, 0.05, 1.10, *IDENTITY, 0.02],
           "live": {"hammer": [-0.1, 0.0, 1.05, *yaw(90)], "nail": [-0.05, 0.05, 1.055, *IDENTITY]},
           "env_info": {"succeed": True, "nail_depth": 0.045, "hammer_hit": False},
           "info_history": {"ever_true": ["succeed", "hammer_hit"], "sequences": {}}}
    report = A.ground_truth_report(sim_dir(tmp_path, raw))
    text = A.report_text(report)
    assert text.startswith("Outcome: success after 350 of 1000 steps (ended on success).")
    assert ("  At the end: the nail was driven 4.5 cm into the board; the hammer touched the nail during the "
            "episode.") in text
    assert "- nail: 4.5 cm down" in text and "- hammer: 49 cm horizontally, 10 cm up, turned 90 degrees" in text


def multiply(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def test_turns_and_tilts_do_not_depend_on_the_objects_own_frame():
    """Objects whose model frame lies on its side (the real hammer, peg and watering can) once read
    as turned 150 degrees after settling a few degrees: rotations are measured in the world frame."""
    lying = [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0]  # 90 degrees about y
    tip = [math.cos(math.radians(10)), math.sin(math.radians(10)), 0.0, 0.0]  # 20 degrees about x
    for start in (IDENTITY, lying, [0.6836, -0.1809, 0.6836, 0.1809]):
        turned = A.rotation_change(start, multiply(yaw(30), start))
        assert turned == pytest.approx((0.0, 30.0), abs=1e-6)
        tilted = A.rotation_change(start, multiply(tip, start))
        assert tilted == pytest.approx((20.0, 0.0), abs=1e-6)
        assert A.movement([0, 0, 0, *start], [0, 0, 0, *multiply(tip, start)]) == "tilted 20 degrees"
    # The real hammer settling after reset (hold only): tilted, never turned.
    assert A.movement([-0.2673, -0.4925, 1.0327, 0.6836, -0.1809, 0.6836, 0.1809],
                      [-0.2874, -0.5044, 1.0118, 0.7711, -0.1546, 0.5828, 0.2045]) == (
        "2.3 cm horizontally, 2.1 cm down, tilted 16 degrees")


@pytest.mark.parametrize("env", sorted(O.TAIL))
def test_every_task_measures_each_condition_its_rules_state(tmp_path, env):
    """Every success and failure condition of task_rules/<task>.md has a measurement slot, for both
    variants, and a report from a bare goal report still reads."""
    for task in (env, env + "_random"):
        state_dim = 46 if env.startswith("bimanual") else 23
        raw = {"episode_id": f"{task}_exploration_0_x", "task": task, "env_name": env, "episode_mode": "exploration",
               "reason": "close", "step_id": 10, "step_limit": 1000, "success": False, "terminated": False,
               "state_dim": state_dim, "initial_state": [0.0] * (state_dim + O.tail_size(env))}
        report = A.ground_truth_report(sim_dir(tmp_path / task, raw))
        assert report["warnings"] == [] and report["conditions"]["success"]["conditions"]
        assert "Outcome: no success after 10 of 1000 steps" in A.report_text(report)


def test_anomalies_are_reported_to_the_operator(tmp_path):
    raw = bucket_report(initial_state=ROBOT + [0.0] * 5)
    raw["live"] = {**raw["live"], "boxed food centre": {"error": "KeyError: 'boxed_food_0_pos'",
                                                        "source": "sensor:boxed_food_0_pos"}}
    report = A.ground_truth_report(sim_dir(tmp_path, raw))
    assert report["warnings"] == ["at the end: boxed food centre not read: KeyError: 'boxed_food_0_pos'",
                                  "the state tail has 5 values; pick_bucket's proprio_keys give 15"]
    assert report["conditions"]["success"]["conditions"][0]["measured"] is None
    assert "- bucket: not recorded" in A.report_text(report)
    disagree = bucket_report()
    disagree["initial_live"] = {**disagree["initial_live"], "bucket": [0.45, -0.10, 0.95, *IDENTITY]}
    assert A.ground_truth_report(sim_dir(tmp_path / "b", disagree))["warnings"] == [
        "bucket: the state tail and MuJoCo disagree on its reset position by 5.0 cm"]


def test_missing_reports_and_simulator_errors(tmp_path):
    sim = sim_dir(tmp_path, bucket_report())
    (sim / "private_goal_report.json").unlink()
    assert A.ground_truth_report(sim) is None
    broken = A.ground_truth_report(sim_dir(tmp_path / "b", {"episode_id": "e", "error": "KeyError: x", "step_id": 5}))
    assert broken["summary"] == A.report_text(broken)
    assert "could not report the rest of its ground truth (KeyError)" in broken["summary"]


def test_agent_text_never_names_the_layout_or_the_models_internals(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path, bucket_report()))
    text = A.agent_review(report, "With seed 1003 (layout id 3) boxed_food_0 stayed put; hanoi_disk_small and "
                                  "industreal_round_peg_8mm too; bucket_ori_pose unchanged.", episode_number=3)
    assert "boxed food stayed put; small disk and peg too; [omitted] unchanged." in text
    assert "seed" not in text and "layout id" not in text and "1003" in text  # Numbers alone reveal nothing.
    assert text.startswith("# Exploration episode 3 review\n\nEpisode: `pick_bucket_exploration_3_ab12cd34`")
    assert "Ground truth" not in text and "Success requires" not in text
    detailed = A.agent_review(report, "ok", episode_number=3, detailed=True, run="runtime/autonomous_controller/runs/r")
    assert "## Ground truth at the end\n\nOutcome: no success" in detailed and "## Analyst review\n\nok" in detailed
    assert "; run directory `runtime/autonomous_controller/runs/r`" in detailed


def test_the_trace_keeps_each_steps_outcome(tmp_path):
    trace = tmp_path / "mcp.jsonl"
    trace.write_text("\n".join(json.dumps(e) for e in [
        {"time": 0, "event": "mcp_request", "tool": "dexjoco_step", "arguments": {"actions": [[0.0] * 22] * 30}},
        {"time": 2, "event": "mcp_result", "tool": "dexjoco_step", "returned": {"step_id": 30, "transition": {
            "kind": "official_actions", "terminated": False, "steps": [{"step_id": 1}], "frame_count": 4}}}]) + "\n")
    assert A.condensed_trace(trace) == [
        '     0s dexjoco_step {"actions":"<30 rows>"} -> step 30 {"kind":"official_actions","terminated":false}']


# ------------------------------------------------------------------ lifecycle
class FakeRouter:
    calls = []

    def __init__(self, text="The bucket was slid 10 cm (seed 1003) but the boxed_food_0 never moved."):
        self.text = text

    def prepare(self, arguments):
        FakeRouter.calls.append(arguments)
        return arguments, 10 ** 6

    def execute(self, payload, reservation):
        return {"message": {"content": self.text}, "cost_usd": 0.0125}, 1


def png():
    from PIL import Image
    buffer = io.BytesIO()
    Image.new("RGB", (8, 8), 100).save(buffer, format="PNG")
    return buffer.getvalue()


def session(tmp_path, *, mode="exploration", benchmark="dexjoco"):
    root = tmp_path / "20261010T000000Z_dexjoco"
    root.mkdir()
    (root / "configuration.json").write_text(json.dumps({"benchmark": benchmark, "task": "pick_bucket",
                                                         "task_rules": "code-summary"}))
    sim = sim_dir(root, bucket_report(), mode=mode)
    os.utime(sim / "summary.json", (time.time() + 1, time.time() + 1))  # Closed after reviews were enabled.
    logical = "runtime/autonomous_controller/runs/interactive-/x"
    run = root / "published/runs/interactive-/x"
    run.mkdir(parents=True)
    (run / "mcp.jsonl").write_text("\n".join(json.dumps(e) for e in [
        {"time": 0, "event": "mcp_request", "tool": "dexjoco_step", "arguments": {"actions": [[0.0] * 22] * 30}},
        {"time": 3, "event": "mcp_result", "tool": "dexjoco_step",
         "returned": {"step_id": 30, "transition": {"kind": "official_actions", "terminated": False}}}]) + "\n")
    (run / "image_01.png").write_bytes(png())
    (root / "exploration").mkdir(exist_ok=True)
    (root / "exploration/state.json").write_text(json.dumps({"episodes": [{
        "episode": 1, "episode_id": bucket_report()["episode_id"], "status": "closed",
        "artifacts": {"directory": logical, "mcp_trace": f"{logical}/mcp.jsonl",
                      "final_images": [{"path": f"{logical}/image_01.png"}]}}]}))
    return root


def review_closed(root):
    """What closing each finished episode does while reviews are on: review it now, in this thread."""
    if not runner.settings(root)["enabled"]:
        return []
    reviews = BENCHMARKS["dexjoco"].episode_reviewer(root)
    reviews.router_factory, reviews.wait_seconds, reviews.poll_seconds = FakeRouter, 0, 0
    try:
        return [record for row in runner.pending(root) if (record := reviews._review(row["episode_id"]))]
    finally:
        reviews.close()


def test_a_closed_dexjoco_episode_is_reviewed_once(tmp_path):
    root = session(tmp_path)
    assert review_closed(root) == []  # Reviews are off.
    runner.set_settings(root, enabled=True)
    os.utime(next(root.glob("**/summary.json")), (time.time() + 1, time.time() + 1))
    FakeRouter.calls.clear()
    [record] = review_closed(root)
    assert record["status"] == "ready" and record["detailed"] is False and record["mode"] == "exploration"
    [call] = FakeRouter.calls
    assert call["messages"][0]["content"] == A.EXPLAIN_PROMPT
    prompt = call["messages"][1]["content"]
    assert prompt[0]["text"].startswith("Task instruction: Place the boxed food into the bucket and then lift the bucket.")
    assert "Success requires, at the same step:" in prompt[0]["text"]
    assert '"kind":"official_actions"' in prompt[0]["text"] and [p["type"] for p in prompt[1:]] == ["image"]
    agent = (root / "published/runs/interactive-/x/review.md").read_text()
    assert agent == ("# Exploration episode 1 review\n\nEpisode: `pick_bucket_exploration_3_ab12cd34`; run directory "
                     "`runtime/autonomous_controller/runs/interactive-/x`\n\n"
                     "The bucket was slid 10 cm ([omitted] 1003) but the boxed food never moved.\n")
    path = runner.record_path(root, record["episode_id"])
    assert path.stat().st_mode & 0o077 == 0
    assert json.loads(path.read_text())["report"]["private"]["table_height_offset_m"] == pytest.approx(0.031)
    assert runner.spent(root) == pytest.approx(0.0125)
    assert review_closed(root) == []  # Reviewed once.


def test_detailed_dexjoco_reviews_open_with_the_ground_truth(tmp_path):
    root = session(tmp_path)
    runner.set_settings(root, enabled=True, detailed=True)
    os.utime(next(root.glob("**/summary.json")), (time.time() + 1, time.time() + 1))
    FakeRouter.calls.clear()
    assert [r["detailed"] for r in review_closed(root)] == [True]
    assert FakeRouter.calls[0]["messages"][0]["content"] == A.DETAILED_PROMPT
    agent = (root / "published/runs/interactive-/x/review.md").read_text()
    assert "## Ground truth at the end\n\nOutcome: no success after 400 of 1200 steps" in agent
    assert "Partial score" not in agent and "0.031" not in agent


def test_the_dashboard_shows_the_operator_the_reports_summary_and_warnings():
    source = (PROJECT / "services/dashboard/static/app.js").read_text()
    review = source[source.index("async function showReview"):source.index("async function refreshReviews")]
    assert "record.report?.summary" in review and "record.report?.warnings" in review
    assert "record.report?.missed" in review  # A report may keep a missed-condition list.


def test_formal_episodes_are_never_reviewed(tmp_path):
    root = session(tmp_path, mode="formal", benchmark="dexjoco")
    runner.set_settings(root, enabled=True)
    os.utime(next(root.glob("**/summary.json")), (time.time() + 1, time.time() + 1))
    reviews = runner.EpisodeReviews(root, router_factory=FakeRouter, wait_seconds=0, poll_seconds=0)
    try:
        assert reviews._review(bucket_report()["episode_id"]) is None
    finally:
        reviews.close()
    assert not (root / "published/runs/interactive-/x/review.md").exists()


def test_reviews_are_a_dexjoco_setup_option(tmp_path):
    from services.dashboard.sessions import DEFAULTS, ResearchSessions
    bench = BENCHMARKS["dexjoco"]
    assert bench.episode_reviews and "episode_reviews" not in bench.fixed and bench.review_analysis() is A
    research = ResearchSessions(tmp_path, lambda: ["pick_bucket"])
    values = research.settings({**DEFAULTS, "task": "pick_bucket", "episode_reviews": "detailed"})
    assert (values["benchmark"], values["episode_reviews"]) == ("dexjoco", "detailed")


def test_a_dexjoco_workspace_follows_the_review_setting(tmp_path):
    from harness.codex_cli.auto_research import refresh_reviews
    from services.dexjoco.task_rules import task_context
    root = session(tmp_path)
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    task = task_context("pick_bucket")
    (workspace / "TASK.md").write_text(task)
    agents = "# Agent\n\n## Start here\n\nx\n\n## Workflow\n\ny\n"
    (workspace / "AGENTS.md").write_text(agents)
    config = SimpleNamespace(root=root, benchmark="dexjoco", agent_formal=False)
    refresh_reviews(config, workspace)
    assert (workspace / "TASK.md").read_text() == task and (workspace / "AGENTS.md").read_text() == agents
    runner.set_settings(root, enabled=True)
    refresh_reviews(config, workspace)
    refresh_reviews(config, workspace)
    text = (workspace / "TASK.md").read_text()
    assert text.count("## Episode reviews") == 1 and text.startswith(task.rstrip("\n") + "\n\n## Episode reviews\n")
    section = text.split("## Episode reviews")[1]
    assert "`rehearse` reply named, under `runtime/autonomous_controller/runs/`. From the simulator's" in section
    assert "what the simulator measured\nfor each success condition" in section and "exact distances" not in section
    assert "partial" not in section.lower()
    assert (workspace / "AGENTS.md").read_text().count("`review.md`") == 1
    runner.set_settings(root, enabled=True, detailed=True)
    refresh_reviews(config, workspace)
    assert "quotes exact distances, angles and counts" in (workspace / "TASK.md").read_text()
    live_copy = SimpleNamespace(root=root, benchmark="dexjoco", agent_formal=True)
    refresh_reviews(live_copy, workspace)
    assert "rehears" not in (workspace / "TASK.md").read_text().split("## Episode reviews")[1]
    runner.set_settings(root, enabled=False)
    refresh_reviews(config, workspace)
    assert (workspace / "TASK.md").read_text() == task and (workspace / "AGENTS.md").read_text() == agents


# ------------------------------------------------------------------ simulator records
def test_the_tap_remembers_ever_true_flags_and_pressed_digits():
    import numpy as np
    from services.dexjoco.session import _Tap

    class Env:
        infos = iter([{"hammer_hit": False, "pressed_digits": []}, {"hammer_hit": np.bool_(True), "pressed_digits": [1]},
                      {"hammer_hit": False, "pressed_digits": [2], "nail_depth": 0.01}])

        def reset(self):
            return {"state": []}, {"succeed": False}

        def step(self, action):
            return {"state": []}, 0.0, False, False, next(self.infos)

    tap = _Tap(Env())
    tap.reset()
    for _ in range(3):
        tap.step(None)
    assert tap.history() == {"ever_true": ["hammer_hit"], "sequences": {"pressed_digits": [1, 2]}}
    assert tap.last_info["nail_depth"] == 0.01
    tap.reset()
    assert tap.history() == {"ever_true": [], "sequences": {}}


def test_live_measurements_are_read_only_and_errors_recorded():
    class View(SimpleNamespace):
        pass

    names = {"bucket_root": View(qpos=[0.5, -0.1, 0.97, 1.0, 0.0, 0.0, 0.0])}
    sites = {f"bucket_ref_{i}": View(xpos=[float(i), 0.0, 1.0]) for i in range(8)}
    data = SimpleNamespace(joint=names.__getitem__, site=sites.__getitem__,
                           sensor={"boxed_food_0_pos": View(data=[0.3, -0.1, 0.97])}.__getitem__,
                           body=lambda name: (_ for _ in ()).throw(KeyError(name)))
    values = O.read_all(data, "pick_bucket")
    assert values["bucket"] == [0.5, -0.1, 0.97, 1.0, 0.0, 0.0, 0.0]
    assert values["boxed food centre"] == [0.3, -0.1, 0.97] and values["bucket corners"][::3] == list(map(float, range(8)))
    assert values["boxed food"] == {"error": "KeyError: 'boxed_food_0_freejoint'",
                                    "source": "joint:boxed_food_0_freejoint"}
    with pytest.raises(ValueError, match="Unknown live source"):
        O.read_live(data, "actuator:x")


DEXJOCO = PROJECT / "DexJoCo/dexjoco/dexjoco"


@pytest.mark.skipif(not DEXJOCO.is_dir(), reason="the official DexJoCo checkout is not linked")
@pytest.mark.parametrize("env", sorted(O.TAIL))
def test_tail_layout_and_live_names_follow_the_official_task(env):
    """One tail object per official *_ori_pose(s) key, in order, and every live name in the XMLs."""
    config = (DEXJOCO / "tasks" / env / "config.py").read_text()
    keys = re.findall(r'"(\w+)"', re.search(r"proprio_keys = \[(.*?)\]", config, re.S)[1])
    assert keys[:2] == ["tcp_pose", "gripper_pose"] and keys[-1] == "table_delta_height"
    assert len(keys[2:-1]) == len(O.TAIL[env]) and all(re.fullmatch(r"\w+_ori_pose?", k) for k in keys[2:-1])
    assert [size for _, size, _ in O.TAIL[env]] == [3 if k.endswith("_pos") else 7 for k in keys[2:-1]]
    xml = "\n".join(p.read_text(errors="replace") for p in (DEXJOCO / "sim/envs/xmls").rglob("*.xml"))
    for source in O.live_sources(env).values():
        for name in source.split(":", 1)[1].split(","):
            assert f'name="{name}"' in xml, f"{env}: {source}"
