"""Simulation privilege must remain explicit; activation needs target and release."""
import time
import pytest
from test_elevator import RobotContext, settings
from auto_research_agent.api.elevator_sim import SimulationElevator
from auto_research_agent.api.base import Base, PrimitiveError


class SimulationContext(RobotContext):
    def __init__(self):
        super().__init__()
        self.simulation_clock = time
        self.capabilities['simulated'] = True
        self.capabilities['scene']['observation_source'] = 'simulator_ground_truth'
        self.source = 'simulator_ground_truth'
        self.age = 0
        self.activation = dict(source=self.source, target='2', single_correct_press=True, released=True)

    def value(self, tool, **args):
        if tool == 'sim_button_observation':
            return dict(source=self.source, label=args['label'], position=[1, 0, 1], normal=[1, 0, 0],
                        scene_pose=[0, 0, 0], capture_us=time.time_ns()//1000-self.age,
                        observation_id='test-capture', uncertainty_m=0.001)
        if tool == 'sim_button_activation':
            return self.activation
        if tool == 'rax_scene':
            result = self.scene()
            result['heading_source'] = self.source
            return result
        return super().value(tool, **args)


def test_live_context_cannot_use_simulated_task():
    with pytest.raises(PrimitiveError, match='explicit simulator'):
        SimulationElevator(RobotContext(), settings())


def test_simulation_origin_is_required_on_every_observation():
    context = SimulationContext()
    task = SimulationElevator(context, settings())
    assert task.observe_button('2', 'right').position == (1, 0, 1)
    context.source = 'unknown'
    with pytest.raises(PrimitiveError, match='source'):
        task.observe_button('2', 'right')


def test_stale_simulation_capture_is_rejected():
    context = SimulationContext()
    task = SimulationElevator(context, settings())
    context.age = 2_000_000
    with pytest.raises(PrimitiveError, match='stale'):
        task.observe_button('2', 'right')


@pytest.mark.parametrize('changes', [{'released': False}, {'target': '1'}, {'single_correct_press': False}, {'source': 'unknown'}])
def test_unverified_activation_cannot_complete_task(changes):
    context = SimulationContext()
    task = SimulationElevator(context, settings())
    assert task.verify_button_activation('2', 'right').outcome == 'succeeded'
    context.activation.update(changes)
    assert task.verify_button_activation('2', 'right').outcome == 'unknown'


def test_live_base_rejects_simulator_heading():
    context = SimulationContext()
    context.capabilities.pop('simulated')
    with pytest.raises(PrimitiveError, match='heading'):
        Base(context).observe_scene()


def test_tool_offset_does_not_require_rotation_clearance_when_orientation_is_unchanged():
    from dataclasses import replace
    from auto_research_agent.api.elevator import ArmSelection
    context = SimulationContext()
    context.palms['right']['position'] = [.9, 0., .88]
    cfg = settings()
    cfg = replace(cfg, arms=tuple(replace(p, tip_in_palm_m=(0., 0., .12)) for p in cfg.arms))
    task = SimulationElevator(context, cfg)
    button = task.observe_button('2', 'right')
    choice = ArmSelection('right', 'selected', (), button, 0.)
    task.prepare_press_arm(choice)
    assert task._selected == 'right'


def test_tool_rotation_still_requires_sweep_clearance():
    from dataclasses import replace
    from auto_research_agent.api.elevator import ArmSelection
    context = SimulationContext()
    context.palms['right']['position'] = [.9, 0., .88]
    cfg = settings()
    cfg = replace(cfg, arms=tuple(replace(p, tip_in_palm_m=(0., 0., .12),
                                         press_quaternion_xyzw=(1., 0., 0., 0.)) for p in cfg.arms))
    task = SimulationElevator(context, cfg)
    button = task.observe_button('2', 'right')
    with pytest.raises(PrimitiveError, match='insufficient gap'):
        task.prepare_press_arm(ArmSelection('right', 'selected', (), button, 0.))


def test_v4_hip_is_part_of_measured_travel_posture():
    context = SimulationContext()
    context.joints['hip_pitch_joint'] = .2
    context.scene_cfg['travel_joints']['hip_pitch_joint'] = 0.
    context.capabilities['joint_limits']['hip_pitch_joint'] = {'tolerance': .01}
    task = SimulationElevator(context, settings())
    task.stow_arms()
    assert context.joints['hip_pitch_joint'] == 0.
    moves = [a for name, a in context.calls if name == 'evo_move_joints']
    assert moves[-1]['targets']['hip_pitch_joint'] == 0.


def test_v4_stow_refuses_a_hip_that_did_not_reach_target():
    class UnreachedHip(SimulationContext):
        def value(self, tool, **args):
            result = super().value(tool, **args)
            if tool == 'evo_status':
                result['state']['positions']['hip_pitch_joint'] = .2
            return result
    context = UnreachedHip()
    context.joints['hip_pitch_joint'] = .2
    context.scene_cfg['travel_joints']['hip_pitch_joint'] = 0.
    context.capabilities['joint_limits']['hip_pitch_joint'] = {'tolerance': .01}
    task = SimulationElevator(context, settings())
    with pytest.raises(PrimitiveError, match='travel posture'):
        task.stow_arms()
    assert not context.base_calls


def test_image_simulation_behavior_refuses_real_adapter():
    from auto_research_agent.api.elevator_sim import ImageSimulationElevator
    with pytest.raises(PrimitiveError, match='simulated image'):
        ImageSimulationElevator(RobotContext(), settings())


@pytest.mark.parametrize('event,expected', [
    ({'source': 'simulator_ground_truth', 'target': '2', 'single_correct_press': True, 'released': True}, 'succeeded'),
    ({'source': 'unqualified', 'target': '2', 'single_correct_press': True, 'released': True}, 'unknown'),
    ({'source': 'simulator_ground_truth', 'target': '2', 'single_correct_press': True, 'released': False}, 'unknown'),
    ({'source': 'simulator_ground_truth', 'target': '1', 'single_correct_press': True, 'released': True}, 'unknown'),
])
def test_image_activation_decision_is_in_rax_and_requires_source_and_release(event, expected):
    from auto_research_agent.api.elevator_sim import ImageSimulationElevator
    context = SimulationContext()
    context.capabilities['scene']['button_observation_source'] = 'image_panel_recognition'
    context.activation = event
    task = ImageSimulationElevator(context, settings())
    task.observe_button = lambda label, side: object()
    assert task.verify_button_activation('2', 'right').outcome == expected
    task.observe_button = lambda label, side: None
    assert task.verify_button_activation('2', 'right').outcome == 'unknown'
