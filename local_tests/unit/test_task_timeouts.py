from dataclasses import asdict, replace
from pathlib import Path

import pytest

from services.robodojo.timeouts import task_timeouts


@pytest.mark.parametrize('task,steps', [('plug_in_charger',400), ('make_kong',600),
    ('pour_by_language',800), ('make_toast',1400), ('make_toast_random',1400)])
def test_task_scaled_budgets(task, steps):
    assert task_timeouts(task) == {'native_step_limit':steps, 'episode_seconds':6*steps}


def test_every_installed_task_has_unambiguous_native_horizon():
    root = Path(__file__).resolve().parents[2]/'RoboDojo/task/RoboDojo/config'
    for path in root.glob('*.yml'):
        if not path.stem.startswith('_'):
            assert task_timeouts(path.stem)['native_step_limit'] > 0


def test_saved_absolute_timeouts_cannot_override_selected_task(tmp_path):
    from services.dashboard.sessions import ResearchSessions
    value = ResearchSessions(tmp_path, lambda: ['make_toast']).settings({'task': 'make_toast', 'formal_seconds': 2400})
    assert value['formal_seconds'] == 8400


def test_research_timeouts_apply_to_both_phases(tmp_path):
    from local_tests.unit.test_controller_backend import configuration
    from services.controller.config import with_task_timeouts
    config = replace(configuration(tmp_path), task='make_toast')
    scaled = with_task_timeouts(config)
    assert scaled.formal.wall_seconds == scaled.development.wall_seconds == 8400
    assert scaled.formal.memory_mb == config.formal.memory_mb


def test_behavior_tasks_have_a_step_limit_and_no_wall_or_session_limit(tmp_path):
    from local_tests.unit.test_controller_backend import configuration
    from services.behavior.tasks import task_timeouts as behavior_timeouts
    from services.controller.config import with_task_timeouts
    timeouts = behavior_timeouts('loading_the_car')
    assert timeouts['episode_seconds'] is None and 'session_seconds' not in timeouts
    assert timeouts['native_step_limit'] > 0
    config = replace(configuration(tmp_path), task='loading_the_car',
                     benchmark='behavior', eval_seed=0, formal_eval_seed=1, observation_profile='rgbd')
    scaled = with_task_timeouts(config)
    assert scaled.formal.wall_seconds is None and scaled.development.wall_seconds is None


def test_a_behavior_session_saved_under_the_old_wall_runs_its_episodes_without_one(tmp_path, monkeypatch):
    """The frontend, which starts the episodes, takes the task's current limits as the launcher
    does, and the resumed trial and its exploration workers follow (the frontend once read the
    saved limit, so a resumed session kept it)."""
    import json
    from local_tests.unit.test_controller_backend import configuration
    from services.controller import frontend
    from services.controller.supervisor import Supervisor
    base = configuration(tmp_path)
    monkeypatch.setenv('ROBODOJO_ARTIFACT_ROOT', str(tmp_path))
    old = replace(base, root=tmp_path/'session', task='loading_the_car', benchmark='behavior', eval_seed=0,
                  formal_eval_seed=1, observation_profile='rgbd')
    old = replace(old, development=replace(old.development, wall_seconds=12653),
                  formal=replace(old.formal, wall_seconds=12653))
    path = old.root/'configuration.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(old), default=str))  # As the launcher saves it.
    Supervisor(old, lambda: None).close()  # The trial as saved before the change.
    config = frontend.load_configuration(path)
    assert config.development.wall_seconds is None and config.formal.wall_seconds is None
    supervisor = Supervisor(config, lambda: None)
    try:
        saved = json.loads(supervisor.state_path.read_text())['configuration']
        assert saved['development']['wall_seconds'] is None and saved['formal']['wall_seconds'] is None
        # What ParallelResearchFrontend hands its exploration workers as episode_seconds.
        assert supervisor.config.formal.wall_seconds is None
    finally:
        supervisor.close()
