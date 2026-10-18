"""Simulator backends behind the shared harness: one registry entry per benchmark.

Pure configuration, like ``services.mcp_contract``: no simulator or heavy imports at module
load. Each benchmark is a peer: an entry here holds its data (robot tool names, robot facts,
setup defaults and fixed fields, capabilities), and its adapter module
(``services/<benchmark>/benchmark.py``) implements the hooks the shared harness calls. Shared
code dispatches through the entry and never compares benchmark names. The benchmark is fixed
at session creation; configurations and manifests without the field use DEFAULT.
"""
from dataclasses import dataclass, field
import importlib

DEFAULT = 'humanoid'  # Configurations and manifests without a benchmark field.

HUMANOID_TOOLS = frozenset({
    'humanoid_status', 'humanoid_observe', 'humanoid_step', 'humanoid_free_space_move',
    'humanoid_execute_motion_plan', 'humanoid_pixel_to_position',
})


@dataclass(frozen=True, eq=False)  # Identity: entries are singletons with dict fields.
class Benchmark:
    name: str
    label: str                      # Display name: 'HumanoidArena'.
    adapter: str                    # Module implementing HOOKS.
    robot_tools: frozenset
    geometry_tools: frozenset       # Removed by the RGB-only profile.
    unscoped_tools: frozenset       # Pure computation: not environment-scoped, served without an episode.
    mutating: frozenset             # Tools that may advance physics.
    previewable: frozenset          # preview_only=true makes these planning-only.
    status_tool: str
    observe_tool: str
    tool_prefixes: tuple
    episode_tool_prefixes: tuple    # Tools whose replies carry the live episode's step_id.
    simulator_modules: tuple        # Simulator server modules (the episode server first).
    startup_seconds: int            # Environment creation deadline (seconds).
    # Words shared workspace sources use for the simulated robot instead of branching on it.
    robot: dict = field(default_factory=dict)
    partial_credit: bool = False    # Episodes are scored with partial credit, which workspaces may name.
    # Formal episodes earn a graded native score (partial credit in the average); False: each
    # episode succeeds or fails and the result is the share of successes.
    graded_score: bool = True
    wall_limit: bool = True         # Episodes have a wall-clock limit (task_timeouts episode_seconds).
    episode_reviews: bool = False   # Simulator ground truth supports exploration episode reviews.
    # Planning and stepping sources (project-relative) an offline parallel formal batch
    # fingerprints, so a change to them during the batch stops it.
    planner_files: tuple = ()
    # Workspaces take the research-method skills (agent_skills/method/); off until each
    # benchmark's workspaces are reviewed with them.
    method_skills: bool = False
    # Regex of this benchmark's task nouns, which its workspaces' skills never name ('' for none):
    # skills describe the method and the robot, TASK.md the task.
    task_nouns: str = ''
    # Regex of this benchmark's own robot, camera, tool and skill names, which another
    # benchmark's workspace never mentions ('' for none).
    vocabulary: str = ''
    task_tag: str = ''              # Suffix of the task in the dashboard's task menu.
    score_label: str = 'Formal average score'
    layout_note: str = ''           # Dashboard note on the layouts exploration and formal use.
    # Setup values applied when the dashboard's task switches to this benchmark (editable).
    setup_defaults: dict = field(default_factory=dict)
    # configure_auto_research.py defaults for flags the operator did not pass.
    cli_defaults: dict = field(default_factory=dict)
    # Setup fields that do not apply: name -> (fixed value, reason).
    fixed: dict = field(default_factory=dict)

    def _hooks(self):
        return importlib.import_module(self.adapter)

    def __getattr__(self, name):
        # The adapter's hooks (HOOKS), loaded only when first called.
        if name not in HOOKS:
            raise AttributeError(name)
        return getattr(self._hooks(), name)

    def planning_only(self, name, arguments):
        return name in self.previewable and isinstance(arguments, dict) and arguments.get('preview_only') is True

    def definitions(self):
        return [t for t in self._hooks().tool_definitions() if t['name'] in self.robot_tools]

    def robot_facts(self, task=None):
        """The robot words for one task: the entry's, with the adapter's optional per-task
        overrides (``task_robot(task)``), e.g. a task's own overview camera."""
        hook = getattr(self._hooks(), 'task_robot', None) if task else None
        return {**self.robot, **(hook(task) if hook else {})}

    def apply_fixed(self, values):
        """values with this benchmark's fixed setup fields."""
        return {**values, **{name: value for name, (value, _) in self.fixed.items()}}


# What every adapter module implements (besides tool_definitions(), the robot tools' schemas):
#   backend(config)                        the controller backend of one environment
#   task_names(), task_timeouts(task)      task catalog and budgets
#   validate_config(config)                benchmark rules of a session configuration
#   validate_settings(values)              benchmark rules of dashboard setup values
#   validate_layouts(config)               formal and exploration layouts exist
#   exploration_seeds(task, collection, start, episodes, *, formal_collection, formal_seed,
#                     formal_episodes)     one layout per exploration episode
#   task_context(config, demonstration, *, live_formal)   TASK.md text
#   refresh_workspace(config, workspace)   update instructions on deploy and resume
#   fetch_demonstration(demos_root, task, kind)           cache visual context before deploy
#   provision_demonstration(config, target, demos_root)   copy it into a workspace
#   episode_reviewer(root)                 reviewer of closed episodes, or None
#   review_analysis()                      the episode-review analysis module (ground truth and
#                                          wording; services/episode_reviews), or None
#   reference_mounts(config)               read-only (host path, container path) reference data for
#                                          the agent container, such as expert episodes
#   unscoped_call(name, arguments)         the JSON reply of one of the entry's unscoped_tools: pure
#                                          computation, with or without an episode
HOOKS = frozenset({
    'backend', 'task_names', 'task_timeouts', 'validate_config', 'validate_settings', 'validate_layouts',
    'exploration_seeds', 'task_context', 'refresh_workspace', 'provision_demonstration',
    'fetch_demonstration', 'episode_reviewer', 'review_analysis', 'reference_mounts', 'unscoped_call',
})

BENCHMARKS = {
    'humanoid': Benchmark(
        name='humanoid', label='HumanoidArena', adapter='services.humanoid.benchmark',
        robot_tools=HUMANOID_TOOLS, geometry_tools=frozenset({'humanoid_pixel_to_position'}),
        unscoped_tools=frozenset(),
        mutating=frozenset({'humanoid_step', 'humanoid_free_space_move', 'humanoid_execute_motion_plan'}),
        previewable=frozenset({'humanoid_free_space_move'}), status_tool='humanoid_status',
        observe_tool='humanoid_observe', tool_prefixes=('humanoid_',), episode_tool_prefixes=('humanoid_',),
        simulator_modules=('services.humanoid.server',), startup_seconds=900,
        robot={'benchmark': 'HumanoidArena', 'status': 'humanoid_status', 'observe': 'humanoid_observe',
               'step': 'humanoid_step', 'plan': 'humanoid_free_space_move', 'pixel': 'humanoid_pixel_to_position',
               'hz': 50,
               'overview_camera': 'cam_front', 'overview': 'front-camera',
               'cameras': ('cam_front',),
               # The skill that characterizes how the controller tracks commands (method skills link it).
               'tracking_skill': 'controller-tracking'},
        graded_score=False,
        vocabulary=(r'humanoid_|\bcam_front\b|\bG1\b|\bDex3\b|\bSONIC\b|semantic40|canonical29'),
        method_skills=True,
        task_nouns=(r'\b(doors?|doorway|leaf|ball|football|soccer|kick|bag|punch(ing)?|boxing|sofa|couch|'
                    r'hammer|basket|crate|desks?|box(es)?|carton|cardboard|shel(f|ves)|warehouse|stairs?|upstairs|'
                    r'staircase|platform|plinth)\b'),
        task_tag='HumanoidArena', score_label='Formal success rate (official test episodes)',
        layout_note='HumanoidArena: exploration draws fresh layouts; formal uses the 60 official test episodes.',
        # One Isaac Sim process per episode environment; the official evaluation has 60 test episodes.
        planner_files=('services/humanoid/sim/planner.py', 'services/humanoid/sim/session.py',
                       'services/humanoid/robot.py', 'services/simulator/protocol.py'),
        setup_defaults={'formal_episodes': 60, 'formal_workers': 4, 'formal_eval_seed': 1, 'exploration_start_seed': 0,
                        'demonstration_context': 'none', 'expert_demonstrations': 0},
        cli_defaults={'exploration_start_seed': 0, 'demonstration_context': 'none', 'formal_episodes': 60,
                      'formal_workers': 4, 'expert_demonstrations': 0},
        fixed={
            'task_rules': ('public', 'TASK.md always gives the HumanoidArena task facts.'),
            'formal_eval_seed': (1, 'Formal always uses the official HumanoidArena test episodes.'),
            'write_handoff': ('off', 'Adaptation needs another layout collection; HumanoidArena has one exploration collection.'),
            'adaptation_source': ('', 'Adaptation needs another layout collection; HumanoidArena has one exploration collection.'),
            'adaptation_context': ('compact', 'Adaptation is unavailable for HumanoidArena.'),
            'episode_reviews': ('off', 'Episode reviews read simulator ground truth; HumanoidArena has none.'),
        },
    ),
}
ALL_ROBOT_TOOLS = frozenset().union(*(b.robot_tools for b in BENCHMARKS.values()))
ALL_MUTATING = frozenset().union(*(b.mutating for b in BENCHMARKS.values()))
ALL_UNSCOPED_TOOLS = frozenset().union(*(b.unscoped_tools for b in BENCHMARKS.values()))
ALL_TOOL_PREFIXES = tuple(p for b in BENCHMARKS.values() for p in b.tool_prefixes)
EPISODE_TOOL_PREFIXES = tuple(p for b in BENCHMARKS.values() for p in b.episode_tool_prefixes)
SIMULATOR_MODULES = tuple(m for b in BENCHMARKS.values() for m in b.simulator_modules)
EPISODE_SERVERS = tuple(b.simulator_modules[0] for b in BENCHMARKS.values())
STATUS_TOOLS = frozenset(b.status_tool for b in BENCHMARKS.values())
OBSERVE_TOOLS = frozenset(b.observe_tool for b in BENCHMARKS.values())


def benchmark(name=DEFAULT):
    try:
        return BENCHMARKS[name]
    except KeyError:
        raise ValueError(f"Unknown benchmark {name!r}; choose one of {', '.join(sorted(BENCHMARKS))}") from None


def for_config(config):
    return benchmark(getattr(config, 'benchmark', DEFAULT))


def for_task(task):
    """The benchmark whose catalog has the task; task names are unique across benchmarks."""
    owner = next((b for b in BENCHMARKS.values() if task in b.task_names()), None)
    if owner is None:
        raise ValueError(f'Unknown task {task!r}')
    return owner


def tool_benchmark(name):
    """The benchmark that owns a robot tool name, or None."""
    return next((b for b in BENCHMARKS.values() if name in b.robot_tools), None)


def planning_only(name, arguments):
    owner = tool_benchmark(name)
    return bool(owner and owner.planning_only(name, arguments))
