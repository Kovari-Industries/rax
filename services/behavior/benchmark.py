"""BEHAVIOR-1K (R1Pro household) hooks for the shared harness; see services/benchmarks.py.

Import-light: the host never imports the simulator stack, and task, rule, review and
demonstration modules load inside the hooks.
"""
from pathlib import Path


def tool_definitions():
    from services.behavior.tools import tool_definitions
    return tool_definitions()


def backend(config):
    from services.behavior.backend import BehaviorBackend
    return BehaviorBackend(config)


def task_names():
    from services.behavior.tasks import task_names
    return task_names()


def task_timeouts(task):
    from services.behavior.tasks import task_timeouts
    return task_timeouts(task)


def validate_config(config):
    from services.behavior.tasks import EXPLORATION_COLLECTION, FORMAL_COLLECTION, task_info
    task_info(config.task)
    if (config.eval_seed, config.formal_collection) != (EXPLORATION_COLLECTION, FORMAL_COLLECTION):
        raise ValueError('BEHAVIOR explores train instances (collection 0) and evaluates the held-out '
                         'public test instances (collection 1)')
    if config.observation_profile != 'rgbd':
        raise ValueError('BEHAVIOR uses the challenge-track RGB-D observations (observation_profile rgbd)')
    if config.task_rules != 'public':
        raise ValueError('BEHAVIOR TASK.md always gives the official BDDL task definition; task_rules does not apply')
    if config.adaptation_source is not None or config.write_handoff:
        raise ValueError('Adaptation needs the same task on another layout collection; BEHAVIOR has one '
                         'train collection, so adaptation and handoff sources are unavailable')


def validate_settings(values):
    """Dashboard setup values (fixed fields already applied). Exploration cycles through the train
    instances, so any start layout is valid; formal uses the held-out public test instances."""
    from services.behavior.tasks import PUBLIC_TEST_INSTANCE_IDS
    if values['formal_seed'] + values['formal_episodes'] > len(PUBLIC_TEST_INSTANCE_IDS):
        raise ValueError(f'BEHAVIOR has {len(PUBLIC_TEST_INSTANCE_IDS)} held-out test instances per task: '
                         'first formal layout + formal episodes must be at most that')


def validate_layouts(config):
    from services.behavior.tasks import validate_layouts as behavior_layouts
    behavior_layouts(config.task, config.formal_collection, range(config.formal_seed, config.formal_seed + config.formal_episodes))
    behavior_layouts(config.task, config.eval_seed, config.exploration_seeds)


def exploration_seeds(task, collection, start, episodes, **_):
    """Layout ids wrap over the train instances, which never include the held-out test instances."""
    return tuple(range(start, start + episodes))


def task_context(config, demonstration, *, live_formal):
    from services.behavior.task_rules import task_context
    return task_context(config.task, live_formal=live_formal, evaluation=not config.agent_formal)


def refresh_workspace(config, workspace):
    """An exploring agent's TASK.md and AGENTS.md follow the current episode-review setting."""
    from services.behavior.task_rules import review_level, with_review_pointer, with_reviews
    level = review_level(config.root)
    for name, change in (('TASK.md', lambda t: with_reviews(t, level, rehearsals=not config.agent_formal)),
                         ('AGENTS.md', lambda t: with_review_pointer(t, level))):
        path = Path(workspace)/name
        if not path.is_file() or path.is_symlink():
            continue
        text = path.read_text()
        if change(text) != text:
            path.write_text(change(text))


def fetch_demonstration(demos_root, task, kind):
    if kind != 'none':
        from services.behavior.demonstrations import cached_demos
        cached_demos(Path(demos_root)/'behavior', task)  # installed by setup_behavior_demos.sh


def provision_demonstration(config, target, demos_root):
    from services.behavior.demonstrations import provision_trial_demonstration
    return provision_trial_demonstration(cache_root=Path(demos_root)/'behavior', target=target,
                                         task=config.task, context_kind=config.demonstration_context)


def episode_reviewer(root):
    """Closing an exploration episode or rehearsal starts its review while reviews are on."""
    from services.behavior.reviews import EpisodeReviews
    return EpisodeReviews(root)
