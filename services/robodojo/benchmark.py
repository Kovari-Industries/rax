"""RoboDojo (ARX X5 table-top) hooks for the shared harness; see services/benchmarks.py.

Import-light: simulator, rubric and demonstration modules load inside the hooks.
"""
from pathlib import Path

CONFIGS = Path(__file__).resolve().parents[2] / 'RoboDojo' / 'task' / 'RoboDojo' / 'config'


def tool_definitions():
    from services.robodojo.mcp_server import RoboDojoMCP
    from services.robot_preview.mcp_server import RobotPreviewMCP
    return RoboDojoMCP.tool_definitions() + RobotPreviewMCP.tools()


def backend(config):
    from services.controller.backend import NativeBackend
    return NativeBackend(config)


def task_names():
    """Tasks with a native configuration."""
    return sorted(p.stem for p in CONFIGS.glob('*.yml') if not p.stem.startswith('_'))


def task_timeouts(task):
    from services.robodojo.timeouts import task_timeouts
    return task_timeouts(task)


def validate_config(config):
    pass  # Every shared setting applies; layouts are checked against the assets by validate_layouts.


def validate_settings(values):
    pass


def validate_layouts(config):
    from services.controller.batch import layout_count
    count = layout_count(config.formal_collection, config.task)
    if config.formal_seed+config.formal_episodes > count:
        raise ValueError(f'Formal collection {config.formal_collection} has {count} layouts for {config.task}; requested range is unavailable')
    # A layout seed indexes the collection's sorted layouts (RoboDojo seed manager).
    count = layout_count(config.eval_seed, config.task)
    if max(config.exploration_seeds) >= count:
        raise ValueError(f'Exploration collection {config.eval_seed} has {count} layouts for {config.task}; '
                         f'exploration seeds reach {max(config.exploration_seeds)}')


def exploration_seeds(task, collection, start, episodes, *, formal_collection, formal_seed, formal_episodes):
    from services.controller.batch import exploration_layouts
    return exploration_layouts(task, collection, start, episodes, formal_collection=formal_collection,
                               formal_seed=formal_seed, formal_episodes=formal_episodes)


def task_context(config, demonstration, *, live_formal):
    from services.robodojo.task_rubrics import task_rubric_context
    return task_rubric_context(config.task, include_source_url=False, task_rules=config.task_rules,
                               live_formal=live_formal,
                               demonstrations=config.demonstration_context != 'none',
                               demonstration_images=demonstration.get('image_count', 0),
                               evaluation=not config.agent_formal)


def refresh_workspace(config, workspace):
    pass


def fetch_demonstration(demos_root, task, kind):
    """The official website demonstration (outside the offline container, before deploy)."""
    from services.controller.demonstrations import cache_demonstration
    cache_demonstration(Path(demos_root)/'website', task, kind)


def provision_demonstration(config, target, demos_root):
    from services.controller import demonstrations
    video, terminal = demonstrations.cache_demonstration(Path(demos_root)/'website', config.task,
                                                         config.demonstration_context)
    return demonstrations.provision_trial_demonstration(video_path=video, terminal_path=terminal, target=target,
                                                        task=config.task, context_kind=config.demonstration_context)


def episode_reviewer(root):
    return None  # No simulator ground truth to review episodes from.
