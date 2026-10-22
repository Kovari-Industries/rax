"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass
from services.evo_v3.backend import TOOLS as EVO_TOOLS

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'rby1': {'name': 'Rainbow RB-Y1', 'status': 'rby1_status', 'observe': 'rby1_observe',
             'move': 'rby1_move_eef', 'gripper': 'rby1_gripper',
             'pixel': 'rby1_pixel_to_position',
             'tools': ('rby1_status', 'rby1_observe', 'rby1_pixel_to_position', 'rby1_look_at',
                       'rby1_move_joints', 'rby1_move_body', 'rby1_move_eef', 'rby1_base_move',
                       'rby1_gripper', 'rby1_stream', 'rby1_hold', 'rby1_fk', 'rby1_check', 'rby1_controller_info',
                       'rby1_cartesian', 'rby1_optimal_control', 'rby1_joint_impedance', 'rby1_joint_group',
                       'rby1_jog'),
             'overview_camera': 'head_camera', 'overview': 'head camera (ZED 2i on the 2-joint head)',
             'cameras': ('head_camera',),
             'body': 'mobile manipulator** (differential-drive base, 6-joint torso, two 7-DoF arms with '
                     'two-finger grippers) with a head camera',
             'move_joints': 'rby1_move_joints',
             'neutral': 'torso and both arms `ready`, head `zero` (`rby1_move_body`; from the arms-down '
                        'rest pose through the `half` presets first)',
             'motion_example': 'ctx.value("rby1_move_eef", arm="right", position=[x, y, z])',
             'grasp_example': 'ctx.value("rby1_gripper", side="right", action="close")',
             'stream': 'rby1_stream',
             'look_around': 'turn the chest (`torso_5`) with the head panned the same way. Tested in a '
                            'session: from the upright torso (all torso joints 0), with nothing held, chest '
                            '-1.9 and +1.9 rad with head pan -/+0.47 and tilt 0.95 see about 86-186 deg to each '
                            'side. That covers where the base\'s rear corners swing in an in-place turn. Split '
                            'each chest turn into calls of at most 1.15 rad (the backend caps a call at 1.2), '
                            'observe each side, then turn back. A tested implementation is `look_around_torso` '
                            'in the skill library\'s `bimanual-box-pnp` (`motion/navigation.py`). For a small '
                            'glance, pan the head (about ±0.47 rad) or use `rby1_look_at`. Turn the base in place '
                            '(`rby1_base_move` with `turn`, [base-navigation](../base-navigation/SKILL.md)) only '
                            'after the look-around shows the swept area clear',
             # Tools that never move anything: the only robot tools subagents get.
             'read_tools': ('rby1_status', 'rby1_observe', 'rby1_pixel_to_position', 'rby1_stream', 'rby1_fk'),
             'model_dir': 'robot-models/rby1a',  # under the artifact root: vendor URDF + meshes (rby1-sdk f3bb123)
             'model_note': 'the vendor\'s model A URDFs with their meshes (`model_v1.0.urdf` matches this robot). '
                           'Generic tools (pinocchio, yourdfpy, trimesh) need `model_v1.0.standard.urdf`: the same model '
                           'with each vendor collision capsule written as a cylinder plus two spheres (forward '
                           'kinematics identical to the vendor SDK\'s to 1e-9). The round add-on fingers are in neither',
             'first_reads': (
                 ('robot-interface', 'every tool, the limits and guards the backend enforces, what a refusal or '
                                     'stop means, the planning tools `rby1_fk` / `rby1_check` that check a move '
                                     'without making it, and the controller\'s own command modes (Cartesian, '
                                     'whole-body IK) with their measured accuracy'),
                 ('whole-body-control', 'torso height and lean, the posture library measured on this robot '
                                        '(squat to `lowest`, leaning at each height, arm stretches, work heights '
                                        'and reach) and the order of moves for reaching a target'),
             ),
             'safety': (
                 '**A fault stops everything.** If a reply or `rby1_status` reports a controller fault '
                 '(`MinorFault` / `MajorFault`), drives not ready or an emergency stop, the drives may have '
                 'dropped and the arms fallen; the session is disarmed. Make no motion and plan no recovery '
                 'motion on your own: call `request_operator` (kind `help`) with what you saw and the last '
                 'command, and wait. After the operator re-arms, call `rby1_status` and `rby1_observe` first and '
                 'recover in small, slow steps from the measured pose.',
                 '**Stay well inside the joint limits.** The real end stops can sit inside the model\'s limits '
                 '(torso_2 hit one at -2.55 rad, model -2.62; pushing on it faulted the robot). Keep routine '
                 'targets at least 0.2 rad inside `rby1_status` -> `limits`; the torso squats at most to '
                 'torso_1 = 1.2 rad (`lowest`, 69 deg); the forward lean is limited by the load check (about +0.7 rad '
                 'with the arms out at `ready` height, more with them tucked; at `low` / `lowest` the torso_3 limit '
                 'caps it at +0.42 / +0.27). Leans beyond +0.15 are not yet tested on the robot: lean in steps of 0.1 rad and read '
                 '`safety.highest_torque_load` after each. Use the tested '
                 'postures in [whole-body-control](.agents/skills/whole-body-control/SKILL.md) (posture library); '
                 'for low targets, drive closer and reach with the arm rather than squatting deeper.',
                 '**Never push.** A reply with `blocked_rad` / `warning`, or a stop for joint torque or wrist '
                 'force, means the robot pressed on something (an end stop, a table, the object, itself). Do '
                 'not resend that target or a further one: back off a few centimetres, observe, and replan. A '
                 'stop leaves the robot controllable: a move that holds or eases the load that stopped it is '
                 'allowed even while that joint is still over the limit.',
                 '**Keep loads light.** Postures with the arms stretched out and the torso leaning load the '
                 'joints towards their ratings (`rby1_status` -> `safety.highest_torque_load`). A joint over '
                 '150 % of its rating stops the motion; above about 100 %, ease the load first (bring it closer '
                 'to the body, set it down) and do not hold it there long: drives heat, and none moves above '
                 '65 C. Carry objects close to the chest.',
                 '**Spend as little time bent down and forward as you can.** The arm joints carry their '
                 'heaviest load when the torso is squatted or leaning and the hands are low and out in front, '
                 'more so when holding something (the wrist joint `left_arm_5` reached 131 % setting a box '
                 'down that way). Do all perception, planning and `rby1_check` before you bend down. '
                 'Then go down, grasp and stand back up as one quick sequence, without pausing to observe '
                 'or think while bent over, and rise with the load close to the body. Quick means no pauses; '
                 'the speed limits do not change, so never try to bypass them.',
                 '**Plan, then move.** `rby1_fk` and `rby1_check` answer posture, reach and '
                 '"would this move be accepted" without arming or moving; use them before any large or unfamiliar '
                 'motion instead of trying it on the robot ([robot-interface](.agents/skills/robot-interface/SKILL.md)).',
                 '**Inspect the fingers before trusting the model.** Before the first grasp, look at both '
                 'grippers in a camera image (hand in view of `rby1_observe`). If the fingertips are round, they '
                 'are the custom fingers, longer than the URDF fingers (`ee_finger_*`, pads 0.073 m beyond '
                 '`ee_*`). Plan with their measured lengths (closed ball 0.195 m, open tips ~0.20 m along -z), '
                 'which are in [robot-interface](.agents/skills/robot-interface/SKILL.md) (Custom fingers). If '
                 'they look different, measure them again and record it in `memory/TIPS.md`.',
                 '**Slow and in short steps.** Motions are slow by design (arms 0.2 rad/s, torso 0.1 rad/s); do '
                 'not try to speed them up with larger steps. Near objects or surfaces: at most 5-10 cm of hand '
                 'travel or 0.3 rad per joint per call, with an observation between, and approach surfaces '
                 'from above with clearance instead of pressing into them.',
             )},
    'evo-v3': {'name': 'Kovari Evo v3', 'status': 'evo_status', 'observe': 'evo_observe',
             'move': 'evo_move_eef', 'gripper': 'evo_gripper', 'pixel': 'evo_camera_ray',
             'tools': tuple(t['name'] for t in EVO_TOOLS),
             'overview_camera': 'base', 'overview': 'head RGB camera',
             'cameras': ('base', 'left_arm', 'right_arm'),
             'body': 'mobile two-arm robot with carriage and neck', 'move_joints': 'evo_move_joints',
             'neutral': 'read measured joints; no automatic home command',
             'motion_example': 'Use evo_preview, then an action tool, then evo_action_status until measured completion.',
             'grasp_example': 'Use evo_set_grip with side, position_rad and retain for cloth; verify grasp from fresh images.',
             'stream': 'evo_history', 'look_around': 'Use bounded neck moves and inspect fresh observations.',
             'safety': ('Evo owns motion. Operator arming and a live client lease are required. A stop request is not proof of physical stop.',),
             'first_reads': (('robot-interface', 'tools, frames, action results and model limits'), ('cloth-handling', 'towel routines, metric perception, grasp and pile checks')),
             'model_dir': None, 'model_note': 'Use Evo FK, IK and preview tools.',
             'read_tools': ('evo_capabilities', 'evo_status', 'evo_observe', 'evo_history', 'evo_fk', 'evo_ik', 'evo_preview', 'evo_camera_ray', 'evo_scene', 'evo_detect_buttons')},
    'mock': {'name': 'mock robot', 'status': 'status', 'observe': 'mock_observe', 'move': 'mock_move',
             'gripper': 'mock_move', 'pixel': 'mock_observe', 'tools': ('mock_observe', 'mock_move'),
             'overview_camera': 'none', 'overview': 'no camera', 'cameras': (),
             'body': 'arm** (simulated)', 'move_joints': 'mock_move', 'neutral': 'the mock at rest',
             'motion_example': 'ctx.value("mock_move", position=[x, y, z])', 'grasp_example': None,
             'stream': None, 'look_around': None, 'safety': (), 'first_reads': (), 'model_dir': None, 'read_tools': ('mock_observe',),
             'model_note': None},
}
CLIS = ('claude',)


@dataclass(frozen=True)
class Setup:
    robot: str = 'evo-v3'
    task: str | None = None
    cli: str = 'claude'
    observation_profile: str = 'rgb-only'
    subagents: int = 0
    gemini: bool = True

    def __post_init__(self):
        if self.robot not in ROBOT_FACTS:
            raise ValueError(f'Unknown robot {self.robot!r}')
        if self.robot == 'evo-v3' and self.observation_profile != 'rgb-only':
            raise ValueError('Evo v3 currently provides RGB-only observations')
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
        return tuple(sorted({'status', 'request_operator', 'skill_library_merge', *ROBOT_FACTS[self.robot]['tools'],
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
