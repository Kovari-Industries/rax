"""Episode reviews: ground-truth classification, agent-safe text, runner modes and privacy.

No simulator or network: sessions are built on disk and Gemini is a fake router.
"""
import io
import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from services.behavior import analysis as A
from services.behavior import reviews as runner

SINK = {"exists": True, "scene_name": "multi_station_furniture_sink_upwldu_0", "category": "sink", "rooms": ["bathroom_1"],
        "aabb": [[23.6, 19.3, 0.0], [24.2, 20.4, 0.75]], "position": [23.9, 19.85, 0.59],
        "links": {"base_link": [[23.6, 19.3, 0.6], [24.2, 20.4, 0.75]]}}
BOTTLE_OUT = {"exists": True, "scene_name": "bottle_of_detergent_228", "aabb": [[23.45, 19.5, 0.0], [23.55, 19.6, 0.25]]}
BOTTLE_IN = {"exists": True, "scene_name": "bottle_of_detergent_227", "aabb": [[23.7, 19.5, 0.0], [23.8, 19.6, 0.25]]}
CUP = {"exists": True, "scene_name": "coffee_cup_220", "aabb": [[23.8, 19.9, 0.75], [23.88, 19.98, 0.85]]}
GOAL = {"schema": "behavior_goal_report_v1", "episode_id": "sorting_household_items_train_48_abcd", "task": "sorting_household_items",
        "instance_id": 48, "episode_mode": "exploration", "reason": "close", "step_id": 7387, "step_limit": 23711,
        "success": False, "natural_language_goal": ["the detergent is under the sink"],
        "options": [[
            {"condition": ["under", "bottle__of__detergent.n.01_1", "sink.n.01_1"], "initial": False, "final": False},
            {"condition": ["under", "bottle__of__detergent.n.01_2", "sink.n.01_1"], "initial": False, "final": True},
            {"condition": ["ontop", "cup.n.01_1", "sink.n.01_1"], "initial": True, "final": True},
            {"condition": ["not", ["toggled_on", "sink.n.01_1"]], "initial": True, "final": False},
        ]],
        "objects": {"sink.n.01_1": SINK, "bottle__of__detergent.n.01_1": BOTTLE_OUT,
                    "bottle__of__detergent.n.01_2": BOTTLE_IN, "cup.n.01_1": CUP},
        "robot_world_pose": [23.56, 22.5, 1.49]}


def sim_dir(root, mode="exploration", report=GOAL, name="exploration/episode-049"):
    sim = root / name / "native/results/run_20261002T114837Z_1/sim"
    sim.mkdir(parents=True)
    (sim / "summary.json").write_text(json.dumps({"episode_id": (report or GOAL)["episode_id"], "step_id": 7387,
                                                  "success": False, "reason": "close", "episode_mode": mode}))
    (sim / "operator_state.json").write_text(json.dumps({"episode_id": (report or GOAL)["episode_id"], "episode_mode": mode}))
    if report is not None:
        (sim / "private_goal_report.json").write_text(json.dumps({**report, "episode_mode": mode}))
    return sim


def png(mode="RGB"):
    from PIL import Image
    buffer = io.BytesIO()
    Image.new(mode, (8, 8), 100 if mode == "RGB" else None).save(buffer, format="PNG")
    return buffer.getvalue()


def session(tmp_path, *, benchmark="behavior", mode="exploration"):
    root = tmp_path / "20261002T030726Z_5c284fedb0"
    root.mkdir()
    (root / "configuration.json").write_text(json.dumps({"benchmark": benchmark, "task": "sorting_household_items"}))
    sim_dir(root, mode=mode)
    run = root / "published/runs/interactive-" / "x"
    run.mkdir(parents=True)
    trace = [{"time": 0, "event": "mcp_request", "tool": "behavior_base_move", "arguments": {"dx": 0.3}},
             {"time": 5, "event": "mcp_result", "tool": "behavior_base_move", "returned": {"step_id": 40, "transition": {"reached": True}}},
             {"time": 6, "event": "mcp_request", "tool": "behavior_step", "arguments": {"actions": [[0.0] * 23] * 9}},
             {"time": 8, "event": "mcp_result", "tool": "behavior_step", "returned": {"step_id": 49}}]
    (run / "mcp.jsonl").write_text("\n".join(map(json.dumps, trace)) + "\n")
    (run / "image_01.png").write_bytes(png())
    (run / "image_02.png").write_bytes(png("I;16"))
    logical = "runtime/autonomous_controller/runs/interactive-/x"
    (root / "exploration").mkdir(exist_ok=True)
    (root / "exploration/state.json").write_text(json.dumps({"episodes": [{
        "episode": 49, "episode_id": GOAL["episode_id"], "status": "closed",
        "artifacts": {"directory": logical, "mcp_trace": f"{logical}/mcp.jsonl",
                      "final_images": [{"path": f"{logical}/image_01.png"}, {"path": f"{logical}/image_02.png"}]}}]}))
    return root


def review_closed(root, router):
    """What closing each finished episode does: review it now, in the caller's thread."""
    reviews = runner.EpisodeReviews(root, router_factory=router, wait_seconds=0, poll_seconds=0)
    try:
        if not runner.settings(root)["enabled"]:
            return []
        return [record for row in runner.pending(root) if (record := reviews._review(row["episode_id"]))]
    finally:
        reviews.close()


class FakeRouter:
    calls = []

    def __init__(self, text="The robot put detergent bottle 1 (bottle_of_detergent_228) in front of the counter; q_score 0."):
        self.text = text

    def prepare(self, arguments):
        FakeRouter.calls.append(arguments)
        return arguments, 10 ** 6

    def execute(self, payload, reservation):
        return {"message": {"content": self.text}, "cost_usd": 0.0125}, 1


# ------------------------------------------------------------------ report
def test_task_names_drop_wordnet_suffixes():
    assert A.task_name("bottle__of__detergent.n.01_2") == "bottle of detergent 2"
    assert A.task_name("?sink.n.01_1") == "sink 1"


def test_report_classifies_conditions_with_relative_geometry(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path))
    assert [c["condition"] for c in report["missed"]] == ["under bottle of detergent 1 sink 1"]
    assert [c["condition"] for c in report["achieved"]] == ["under bottle of detergent 2 sink 1"]
    assert [c["condition"] for c in report["already_true"]] == ["ontop cup 1 sink 1"]
    assert [c["condition"] for c in report["undone"]] == ["not toggled_on sink 1"]
    geometry = report["missed"][0]["geometry"]
    assert geometry == "bottle of detergent 1's centre is 10 cm outside sink 1's footprint; no part of sink 1 is directly above it"
    inside = A.relation_geometry("under", "bottle__of__detergent.n.01_2", "sink.n.01_1", GOAL["objects"])
    assert "inside the edge" in inside and "(base_link) is 35 cm above bottle of detergent 2's top" in inside
    text = A.report_text(report)
    for private in ("23.", "upwldu", "bottle_of_detergent_228", "instance"):
        assert private not in text


def test_missing_report_and_simulator_errors(tmp_path):
    assert A.ground_truth_report(sim_dir(tmp_path, report=None)) is None
    broken = A.ground_truth_report(sim_dir(tmp_path, report={"episode_id": "e", "error": "KeyError: x"}, name="b"))
    assert broken["error"] and "could not report" in A.report_text(broken)


def test_trace_is_condensed_and_depth_images_skipped(tmp_path):
    root = session(tmp_path)
    run = root / "published/runs/interactive-/x"
    lines = A.condensed_trace(run / "mcp.jsonl")
    assert len(lines) == 2 and "<9 rows>" in lines[1] and "step 49" in lines[1]
    assert len(A.rgb_images([run / "image_01.png", run / "image_02.png"])) == 1


def test_agent_review_is_the_scrubbed_explanation_only(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path))
    text = A.agent_review(report, "bottle_of_detergent_228 sits in front; q_score 0; instance id 48.", episode_number=49)
    assert "bottle of detergent 1 sits in front" in text and "Exploration episode 49" in text
    assert "q_score" not in text and "instance id" not in text and "Missed" not in text


# ------------------------------------------------------------------ runner
def test_runner_off_does_nothing(tmp_path):
    root = session(tmp_path)
    assert review_closed(root, FakeRouter) == []
    assert not (root / "published/runs/interactive-/x/review.md").exists()


def test_runner_reviews_new_exploration_episodes_once(tmp_path):
    root = session(tmp_path)
    summary = next(root.glob("exploration/*/native/results/*/sim/summary.json"))
    runner.set_settings(root, enabled=True)
    os.utime(summary, (time.time() + 1, time.time() + 1))  # Closed after enabling.
    FakeRouter.calls.clear()
    done = review_closed(root, FakeRouter)
    assert [r["status"] for r in done] == ["ready"]
    prompt = FakeRouter.calls[0]["messages"][1]["content"]
    assert "under bottle of detergent 1 sink 1" in prompt[0]["text"] and "behavior_base_move" in prompt[0]["text"]
    assert [p["type"] for p in prompt[1:]] == ["image"]
    # Only in the episode's own run directory, beside its trace and frames.
    agent = (root / "published/runs/interactive-/x/review.md").read_text()
    assert "bottle of detergent 1" in agent and "bottle_of_detergent_228" not in agent and "q_score" not in agent
    assert not (root / "published/episode_reviews").exists()
    record = runner.record_path(root, GOAL["episode_id"])
    assert record.stat().st_mode & 0o077 == 0 and json.loads(record.read_text())["report"]["private"]["objects"]
    assert runner.spent(root) == pytest.approx(0.0125)
    assert review_closed(root, FakeRouter) == []


def test_episodes_closed_before_enabling_are_skipped(tmp_path):
    root = session(tmp_path)
    summary = next(root.glob("exploration/*/native/results/*/sim/summary.json"))
    os.utime(summary, (time.time() - 100, time.time() - 100))
    runner.set_settings(root, enabled=True)
    assert review_closed(root, FakeRouter) == []
    first = runner.settings(root)["enabled_at"]
    runner.set_settings(root, enabled=True)
    assert runner.settings(root)["enabled_at"] == first  # Re-saving On keeps the original start.


@pytest.mark.parametrize("mode,benchmark", [("formal", "behavior"), ("exploration", "robodojo")])
def test_formal_and_robodojo_episodes_are_never_reviewed(tmp_path, mode, benchmark):
    root = session(tmp_path, benchmark=benchmark, mode=mode)
    runner.set_settings(root, enabled=True)
    for summary in root.glob("**/summary.json"):
        os.utime(summary, (time.time() + 1, time.time() + 1))
    assert review_closed(root, FakeRouter) == []


def test_failed_explanations_retry_then_stop(tmp_path):
    root = session(tmp_path)
    runner.set_settings(root, enabled=True)
    for summary in root.glob("**/summary.json"):
        os.utime(summary, (time.time() + 1, time.time() + 1))
    class Broken(FakeRouter):
        def execute(self, payload, reservation):
            raise ValueError("provider down")
    # One immediate retry when the episode closes, then the error stands.
    assert [(r["status"], r["attempts"]) for r in review_closed(root, Broken)] == [("error", 2)]
    assert review_closed(root, Broken) == [] and not (root / "published/runs/interactive-/x/review.md").exists()


def test_status_never_carries_reviews(tmp_path):
    from services.controller.frontend import ResearchFrontend
    assert not hasattr(ResearchFrontend, "episode_reviews")


def test_task_text_points_to_each_episode_review(tmp_path):
    from services.behavior.task_rules import review_level, task_context, with_reviews
    root = session(tmp_path)
    assert review_level(root) is None
    runner.set_settings(root, enabled=True)
    assert review_level(root) == "on"
    runner.set_settings(root, enabled=True, detailed=True)
    assert review_level(root) == "detailed"
    text = task_context("sorting_household_items") + "\n## Demonstration\n\nd\n"
    on = with_reviews(text, "on")
    assert "`review.md` in that episode's own run directory" in on and "`rehearse`" in on
    assert "partial score" not in on.split("## Episode reviews")[1]
    detailed = with_reviews(on, "detailed")
    assert detailed.count("## Episode reviews") == 1 and "partial score" in detailed.split("## Episode reviews")[1]
    assert "## Demonstration" in detailed and with_reviews(detailed, None) == text
    assert "rehears" not in with_reviews(text, "on", rehearsals=False).split("## Episode reviews")[1]


def test_reviews_are_a_behavior_setup_option(tmp_path):
    from services.benchmarks import BENCHMARKS
    from services.dashboard.sessions import DEFAULTS, ResearchSessions
    assert DEFAULTS["episode_reviews"] == "off"
    assert BENCHMARKS["robodojo"].fixed["episode_reviews"][0] == "off"
    assert "episode_reviews" not in BENCHMARKS["behavior"].fixed
    project = tmp_path / "project"
    (project / "runtime/auto-research").mkdir(parents=True)
    root = session(project / "runtime/auto-research")
    sessions = ResearchSessions(project, lambda: [])
    assert sessions.reviews(root.name, history=True)["enabled"] is False
    runner.set_settings(root, enabled=True)  # What prepare does for episode_reviews=on.
    assert sessions.reviews(root.name, history=True)["enabled"] is True
    assert not hasattr(sessions, "set_reviews")  # No runtime switch: setup only.


def test_tipped_over_objects_are_named_only_when_they_started_upright():
    lying = [0.7071, 0.0, 0.0, 0.7071]
    objects = {**GOAL["objects"], "bottle__of__detergent.n.01_1": {
        **BOTTLE_OUT, "orientation_xyzw": lying, "initial_orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}}
    line = A.relation_geometry("under", "bottle__of__detergent.n.01_1", "sink.n.01_1", objects)
    assert line.endswith("bottle of detergent 1 has tipped over (90 degrees from upright; it started upright)")
    objects["bottle__of__detergent.n.01_1"]["initial_orientation_xyzw"] = lying  # Its model frame lies sideways.
    assert "tipped" not in A.relation_geometry("under", "bottle__of__detergent.n.01_1", "sink.n.01_1", objects)


def test_detailed_reviews_give_the_partial_score_and_ground_truth(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path))
    text = A.agent_review(report, "bottle_of_detergent_228 is 4 cm short; q_score 0.25; instance id 48.",
                          episode_number=49, detailed=True, score=0.25)
    assert "Partial score: 0.250 (" in text and "Goal conditions at the end (ground truth)" in text
    assert "Missed" in text and "bottle of detergent 1 is 4 cm short; q_score 0.25" in text
    assert "instance id" not in text and "bottle_of_detergent_228" not in text
    assert "Partial score: unavailable" in A.score_text(report, None)


def test_runner_writes_detailed_reviews_when_set(tmp_path):
    root = session(tmp_path)
    runner.set_settings(root, enabled=True, detailed=True)
    assert runner.settings(root)["detailed"] is True
    for summary in root.glob("**/summary.json"):
        os.utime(summary, (time.time() + 1, time.time() + 1))
    FakeRouter.calls.clear()
    assert [r["detailed"] for r in review_closed(root, FakeRouter)] == [True]
    assert FakeRouter.calls[0]["messages"][0]["content"] == A.DETAILED_PROMPT
    assert "Partial score:" in FakeRouter.calls[0]["messages"][1]["content"][0]["text"]
    agent = (root / "published/runs/interactive-/x/review.md").read_text()
    assert agent.startswith("# Exploration episode 49 review") and "Partial score:" in agent
    runner.set_settings(root, enabled=False, detailed=True)
    assert runner.settings(root)["detailed"] is False


def test_behavior_task_text_always_asks_for_maximum_partial_credit():
    from services.behavior.task_rules import task_context
    for text in (task_context("sorting_household_items"), task_context("sorting_household_items", live_formal=True),
                 task_context("sorting_household_items", evaluation=False)):
        assert "## Scoring" in text and "partial credit" in text


def test_a_resumed_workspace_follows_the_review_setting(tmp_path):
    from harness.codex_cli.auto_research import refresh_reviews
    root = session(tmp_path)
    workspace = tmp_path / "workspace"; workspace.mkdir()
    (workspace / "TASK.md").write_text("# Task\n\n## Scoring\n\ns\n")
    agents = "# Agent\n\n## Start here\n\nx\n\n## Workflow\n\ny\n"
    (workspace / "AGENTS.md").write_text(agents)
    config = SimpleNamespace(root=root, benchmark="behavior", agent_formal=False)
    refresh_reviews(config, workspace)
    assert "Episode reviews" not in (workspace / "TASK.md").read_text()
    assert (workspace / "AGENTS.md").read_text() == agents
    runner.set_settings(root, enabled=True, detailed=True)
    refresh_reviews(config, workspace)
    refresh_reviews(config, workspace)
    assert (workspace / "TASK.md").read_text().count("## Episode reviews") == 1
    text = (workspace / "AGENTS.md").read_text()
    assert text.count("`review.md`") == 1 and text.index("`review.md`") < text.index("## Workflow")
    runner.set_settings(root, enabled=False)
    refresh_reviews(config, workspace)
    assert (workspace / "AGENTS.md").read_text() == agents


def test_only_partial_credit_formal_copies_are_asked_for_the_most_score():
    text = (Path(__file__).parents[2] / "auto_research_agent/FORMAL_AGENTS.md").read_text()
    assert "{% if partial_credit %}\n\nMake this episode score as high as it can." in text
    from services.benchmarks import BENCHMARKS
    assert [name for name, b in BENCHMARKS.items() if b.partial_credit] == ["behavior"]


def test_review_names_rehearsals_their_run_and_a_plain_end_reason(tmp_path):
    report = A.ground_truth_report(sim_dir(tmp_path, mode="rehearsal"))
    text = A.agent_review(report, "ok", run="runtime/autonomous_controller/runs/rehearsal-1", detailed=True, score=0.0)
    assert text.startswith("# Rehearsal review") and "run directory `runtime/autonomous_controller/runs/rehearsal-1`" in text
    assert "(closed with finish)" in text and "highest-scoring goal option" in text


def test_reviews_run_in_the_session_mcp_service_not_the_dashboard():
    import inspect
    from services.controller import frontend
    from services.dashboard import server
    source = inspect.getsource(frontend.main)
    assert "supervisor.episode_closed = reviewer.closed" in source and "frontend.pool.episode_closed" in source
    assert "Review" not in inspect.getsource(server)


def test_a_runner_reviews_only_its_own_session(tmp_path):
    root = session(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    runner.set_settings(root, enabled=True)
    for summary in root.glob("**/summary.json"):
        os.utime(summary, (time.time() + 1, time.time() + 1))
    assert review_closed(other, FakeRouter) == []
    assert [r["status"] for r in review_closed(root, FakeRouter)] == ["ready"]



def test_closing_an_episode_starts_its_review_in_the_background(tmp_path):
    root = session(tmp_path)
    reviews = runner.EpisodeReviews(root, router_factory=FakeRouter, wait_seconds=5, poll_seconds=0.01)
    reviews.closed(GOAL["episode_id"])  # Reviews off: nothing happens.
    runner.set_settings(root, enabled=True)
    for summary in root.glob("**/summary.json"):
        os.utime(summary, (time.time() + 1, time.time() + 1))
    reviews.closed(GOAL["episode_id"])
    reviews.closed(GOAL["episode_id"])  # Reported twice, reviewed once.
    reviews.executor.shutdown(wait=True)
    assert json.loads(runner.record_path(root, GOAL["episode_id"]).read_text())["status"] == "ready"
    assert (root / "published/runs/interactive-/x/review.md").is_file()
    assert runner.spent(root) == pytest.approx(0.0125)


def test_episode_management_reports_each_close_once(tmp_path):
    from services.exploration.pool import ExplorationPool
    pool = ExplorationPool(tmp_path / "exploration", count=1, seeds=(0,), worker_config={})
    closed = []
    pool.episode_closed = closed.append
    row = {"episode": 1, "env_id": 0, "status": "active"}
    pool.state["episodes"].append(row)
    pool._update(row, {"state": {"status": "active", "episode_id": "e1"}})
    pool._update(row, {"state": {"status": "closed", "episode_id": "e1"}})
    pool._update(row, {"state": {"status": "closed", "episode_id": "e1"}})
    assert closed == ["e1"]
    from services.controller.supervisor import Supervisor
    calls = []
    fake = SimpleNamespace(episode_closed=calls.append)
    Supervisor._closed(fake, {"episode_id": "e2"})
    Supervisor._closed(fake, {"episode_id": None})
    assert calls == ["e2"]



def test_headless_configuration_sets_episode_reviews(monkeypatch, capsys):
    import sys as system
    sys_path = Path(__file__).parents[2] / "scripts"
    monkeypatch.syspath_prepend(str(sys_path))
    import configure_auto_research as configure
    monkeypatch.setattr(system, "argv", ["configure", "--task", "make_kong", "--sim-gpu", "0",
                                         "--episode-reviews", "on"])
    with pytest.raises(SystemExit):
        configure.main()
    assert "RoboDojo has none" in capsys.readouterr().err
    source = Path(configure.__file__).read_text()
    assert "set_settings(root, enabled=args.episode_reviews != 'off', detailed=args.episode_reviews == 'detailed')" in source


def test_a_success_is_reviewed_against_the_goal_option_that_was_satisfied(tmp_path):
    held = lambda name, initial, final: {"condition": ["ontop", name, "sink.n.01_1"], "initial": initial, "final": final}
    satisfied = [held("cup.n.01_1", True, True), held("cup.n.01_2", True, True),
                 held("bottle__of__detergent.n.01_1", False, True), held("bottle__of__detergent.n.01_2", False, True)]
    # More newly satisfied (3 of 4) but one missed: the best option only when nothing succeeded.
    closer = [held("cup.n.01_1", False, True), held("cup.n.01_2", False, True),
              held("bottle__of__detergent.n.01_1", False, True), held("bottle__of__detergent.n.01_2", False, False)]
    report = A.ground_truth_report(sim_dir(tmp_path, report={**GOAL, "success": True, "options": [closer, satisfied]}))
    assert report["best_option"] == 1 and report["missed"] == [] and len(report["achieved"]) == 2
    assert A.score_text(report, 1.0) == ("Partial score: 1.000 (2 newly satisfied of the 4 goal conditions in the "
                                         "goal option that was satisfied); 1.0 means the task succeeded.")
    failed = A.ground_truth_report(sim_dir(tmp_path, report={**GOAL, "options": [closer, satisfied[:3] + [
        held("bottle__of__detergent.n.01_2", False, False)]]}, name="failed"))
    assert failed["best_option"] == 0 and len(failed["missed"]) == 1
    assert "in the highest-scoring goal option" in A.score_text(failed, 0.75)
