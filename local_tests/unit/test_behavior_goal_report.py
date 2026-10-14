"""Scoring and goal-report failures inside the simulator keep their whole error for the operator."""
import json
import sys
from types import ModuleType, SimpleNamespace

import pytest

from services.behavior import analysis as A
from services.behavior.session import BehaviorSession


class TaskMetric:
    initial_predicate_states = [[False, True]]


@pytest.fixture
def session(tmp_path, monkeypatch):
    metrics = ModuleType("omnigibson.metrics.task_metric")
    metrics.TaskMetric = TaskMetric
    metrics.compute_q_score = lambda success, now, initial: 0.5
    for name in ("omnigibson", "omnigibson.metrics"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    monkeypatch.setitem(sys.modules, "omnigibson.metrics.task_metric", metrics)
    session = BehaviorSession.__new__(BehaviorSession)
    session.output, session.task, session.episode_id, session.instance_id = tmp_path, "turning_on_radio", "ep1", 7
    session.episode_mode, session.step_id, session.step_limit, session.success = "exploration", 9, 100, False
    session.state = SimpleNamespace(metrics=[TaskMetric()])
    session._world_pose = lambda: [0.0, 0.0, 0.0]
    return session


def scope_raising(error):
    class Task:
        @property
        def object_scopes(self):
            raise error
    return Task()


def test_a_q_score_failure_keeps_its_message_and_traceback(session, capsys):
    def unsatisfiable(_):
        raise RuntimeError("bddl predicate ontop(radio, table) failed to evaluate")
    session.env = SimpleNamespace(task=SimpleNamespace(get_goal_option_satisfaction=unsatisfiable))
    q, predicates = session._q_score()
    assert q is None
    assert predicates["error"].endswith("RuntimeError: bddl predicate ontop(radio, table) failed to evaluate")
    assert "in unsatisfiable" in predicates["error"]  # The traceback tail, innermost frame included.
    session._q_score()
    assert capsys.readouterr().err.count("Traceback") == 1  # Logged once, not on every operator write.


def test_a_failed_reset_orientation_snapshot_is_reported_not_dropped(session, capsys):
    session.env = SimpleNamespace(task=scope_raising(KeyError("radio_89.n.01_1")))
    assert session._object_orientations() == {}
    assert session.initial_orientations_error.endswith("KeyError: 'radio_89.n.01_1'")
    assert "KeyError" in capsys.readouterr().err


def test_goal_report_errors_keep_their_message_and_reach_the_operator_review(session, tmp_path):
    class Particles:
        initialized, name, category = True, "water", "water"

        @property
        def aabb(self):
            raise ValueError("water has no rigid body")

    class Head:
        body = ["filled", "?bottle.n.01_1", "?water.n.06_1"]
    task = SimpleNamespace(get_goal_option_satisfaction=lambda _: [[True, False]],
                           ground_goal_state_options=[[Head(), Head()]], object_scopes={"water.n.06_1": Particles()},
                           activity_natural_language_goal_conditions=["fill the bottle"])
    session.env = SimpleNamespace(task=task)
    session.initial_orientations_error = "KeyError: 'radio_89.n.01_1'"
    session._write_goal_report("close")
    raw = json.loads((tmp_path / "private_goal_report.json").read_text())
    assert raw["objects"]["water.n.06_1"]["pose_error"] == "ValueError: water has no rigid body"
    assert raw["initial_orientations_error"] == "KeyError: 'radio_89.n.01_1'"
    (tmp_path / "summary.json").write_text("{}")
    report = A.ground_truth_report(tmp_path)
    assert report["private"]["initial_orientations_error"] == "KeyError: 'radio_89.n.01_1'"
    assert report["private"]["objects"]["water.n.06_1"]["pose_error"] == "ValueError: water has no rigid body"


def test_a_goal_report_that_fails_keeps_its_traceback_for_the_operator_only(session, tmp_path):
    session.env = SimpleNamespace(task=SimpleNamespace(get_goal_option_satisfaction=lambda _: 1 / 0))
    session._write_goal_report("close")
    raw = json.loads((tmp_path / "private_goal_report.json").read_text())
    assert raw["error"] == "ZeroDivisionError: division by zero"
    assert raw["traceback"].endswith("ZeroDivisionError: division by zero") and "in <lambda>" in raw["traceback"]
    (tmp_path / "summary.json").write_text("{}")
    report = A.ground_truth_report(tmp_path)
    assert report["error"] == raw["error"] and report["traceback"] == raw["traceback"]
    assert A.report_text(report) == "The simulator could not report the goal conditions (ZeroDivisionError)."
