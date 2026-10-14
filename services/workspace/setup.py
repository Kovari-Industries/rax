"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'franka': {'name': 'Franka FR3', 'status': 'franka_status', 'observe': 'franka_observe',
               'move': 'franka_move_pose', 'gripper': 'franka_gripper',
               'pixel': 'franka_pixel_to_position',
               'tools': ('franka_status', 'franka_observe', 'franka_move_pose', 'franka_move_joints',
                         'franka_gripper', 'franka_pixel_to_position', 'franka_stream', 'franka_hold'),
               'overview_camera': 'left', 'overview': 'left exterior camera',
               'cameras': ('left', 'right', 'wrist')},
    'mock': {'name': 'mock robot', 'status': 'status', 'observe': 'mock_observe', 'move': 'mock_move',
             'gripper': 'mock_move', 'pixel': 'mock_observe', 'tools': ('mock_observe', 'mock_move'),
             'overview_camera': 'none', 'overview': 'no camera', 'cameras': ()},
}
CLIS = ('claude',)


@dataclass(frozen=True)
class Setup:
    robot: str = 'franka'
    task: str | None = None
    cli: str = 'claude'
    observation_profile: str = 'rgbd'
    subagents: int = 0
    gemini: bool = True

    def __post_init__(self):
        if self.robot not in ROBOT_FACTS:
            raise ValueError(f'Unknown robot {self.robot!r}')
        if self.cli not in CLIS:
            raise ValueError('Unknown agent CLI')
        if self.observation_profile not in ('rgbd', 'rgb-only'):
            raise ValueError('Unknown observation profile')

    @classmethod
    def from_config(cls, config, *, gemini=True):
        return cls(config.robot, config.task, 'claude', config.observation_profile, config.subagents, gemini)

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @property
    def tools(self):
        """Every MCP tool the session exposes, sorted."""
        return tuple(sorted({'status', 'request_operator', *ROBOT_FACTS[self.robot]['tools'],
                             *(('gemini_generate',) if self.gemini else ())}))

    def axes(self):
        """Which skill layers a workspace takes: the robot it controls and how it perceives."""
        return {'robot': self.robot, 'perception': 'depth' if self.camera_geometry else 'rgb-only'}

    def context(self):
        """Template variables: facts only, named for readability in the sources."""
        return {
            'agent_cli': self.cli,
            'task': self.task,
            'observation_profile': self.observation_profile,
            'camera_geometry': self.camera_geometry,
            'subagent_cap': self.subagents,
            'gemini': self.gemini,
            'robot': ROBOT_FACTS[self.robot],
        }
