"""HumanoidArena (G1 + Dex3, frozen SONIC) hooks for the shared harness; see services/benchmarks.py.

Import-light: the host never imports Isaac Sim, Isaac Lab or the upstream task code; task,
rule and demonstration modules load inside the hooks.
"""
from pathlib import Path

# Complete expert episodes per task (`scripts/setup_humanoid_demos.py`), mounted read-only.
EXPERT_DEMONSTRATIONS = 3


def tool_definitions():
    from services.humanoid.tools import tool_definitions
    return tool_definitions()


def backend(config):
    from services.humanoid.backend import HumanoidBackend
    return HumanoidBackend(config)


def task_names():
    from services.humanoid.tasks import task_names
    return task_names()


def task_timeouts(task):
    from services.humanoid.tasks import task_timeouts
    return task_timeouts(task)


def validate_config(config):
    from services.humanoid.tasks import EXPLORATION_COLLECTION, FORMAL_COLLECTION, task_info
    task_info(config.task)
    if (config.eval_seed, config.formal_collection) != (EXPLORATION_COLLECTION, FORMAL_COLLECTION):
        raise ValueError('HumanoidArena explores collection 0 and evaluates the official test episodes '
                         '(collection 1)')
    if config.task_rules != 'public':
        raise ValueError('HumanoidArena TASK.md always gives the task facts; task_rules does not apply')
    if config.expert_demonstrations not in (0, EXPERT_DEMONSTRATIONS):
        raise ValueError(f'expert_demonstrations must be 0 or {EXPERT_DEMONSTRATIONS}')
    if config.adaptation_source is not None or config.write_handoff:
        raise ValueError('Adaptation needs the same task on another layout collection; HumanoidArena has one '
                         'exploration collection, so adaptation and handoff sources are unavailable')


def validate_settings(values):
    """Dashboard setup values (fixed fields already applied): formal layouts index the 60 official episodes."""
    from services.humanoid.tasks import FORMAL_EPISODES
    if values['formal_seed'] + values['formal_episodes'] > FORMAL_EPISODES:
        raise ValueError(f'HumanoidArena has {FORMAL_EPISODES} official test episodes per task: '
                         'first formal layout + formal episodes must be at most that')


def validate_layouts(config):
    from services.humanoid.tasks import validate_layouts
    validate_layouts(config.task, config.formal_collection,
                     range(config.formal_seed, config.formal_seed + config.formal_episodes))
    validate_layouts(config.task, config.eval_seed, config.exploration_seeds)


def exploration_seeds(task, collection, start, episodes, **_):
    """Exploration layout ids map to fresh seeds of the test distribution, never official ones."""
    return tuple(range(start, start + episodes))


def task_context(config, demonstration, *, live_formal):
    from services.humanoid.task_rules import task_context
    return task_context(config.task, live_formal=live_formal, evaluation=not config.agent_formal,
                        expert_demonstrations=config.expert_demonstrations, expert_content=config.expert_content)


def refresh_workspace(config, workspace):
    pass


def reference_mounts(config):
    """Read-only reference data for the agent container: the task's expert episodes (complete, or
    camera-only with expert_content 'images')."""
    if not config.expert_demonstrations:
        return []
    from services.humanoid.demonstrations import expert_root
    return [(expert_root(config.task, config.expert_content), '/workspace/runtime/experts')]


def fetch_demonstration(demos_root, task, kind):
    if kind != 'none':
        from services.humanoid.demonstrations import cached_experts
        cached_experts(Path(demos_root)/'humanoid', task)  # installed by setup_humanoid_demos.py


def provision_demonstration(config, target, demos_root):
    from services.humanoid.demonstrations import provision_trial_demonstration
    return provision_trial_demonstration(cache_root=Path(demos_root)/'humanoid', target=target,
                                         task=config.task, context_kind=config.demonstration_context)


def episode_reviewer(root):
    return None  # HumanoidArena exposes no ground truth for exploration reviews.


def review_analysis():
    return None  # No episode-review analysis: HumanoidArena exposes no ground-truth review.


def unscoped_call(name, arguments):
    raise ValueError(f'{name!r} is not a HumanoidArena unscoped tool')  # HumanoidArena has none.
