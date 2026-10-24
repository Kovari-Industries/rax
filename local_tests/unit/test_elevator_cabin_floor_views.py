"""CPU-only floor-view sequencing; no rendered images or physical qualification."""
from dataclasses import replace
import math

import pytest

from test_elevator_cabin import CabinContext, ride
from auto_research_agent.api.base import PrimitiveError
from auto_research_agent.api.elevator_cabin import CabinRide
from auto_research_agent.api.evo import EvoActionFailed


REST = {'neck_yaw_joint': 0., 'neck_pitch_joint': 0.}
VIEWS = ({'neck_yaw_joint': -.8, 'neck_pitch_joint': -.3},
         {'neck_yaw_joint': .8, 'neck_pitch_joint': -.3})


class FloorViewContext(CabinContext):
    def __init__(self):
        super().__init__()
        self.joints.update(REST)
        self.capabilities['model']['limits'].update({
            'neck_yaw_joint': {'lower': -1.5708, 'upper': 1.5708},
            'neck_pitch_joint': {'lower': -.31, 'upper': .456}})
        self.capabilities['joint_limits'].update({name: {'tolerance': .01} for name in REST})
        self.head_actions = []
        self.capture_positions = []
        self.unreadable = set()
        self.low_confidence = set()
        self.fault = None
        self.preview_number = 0
        self.head_motion_number = 0

    def value(self, tool, **args):
        neck_action = (tool == 'evo_preview' and args['action'].get('kind') == 'move_joints'
                       and set(args['action'].get('targets', {})) == set(REST))
        if neck_action:
            self.preview_number += 1
            if self.fault == ('preview', self.preview_number):
                self.calls.append((tool, args))
                return {'valid': False, 'reason': 'synthetic source-envelope refusal'}
        if tool == 'evo_move_joints' and set(args['targets']) == set(REST):
            self.head_motion_number += 1
            self.head_actions.append(dict(args['targets']))
            if self.fault == ('dispatch', self.head_motion_number):
                raise RuntimeError('dispatch outcome unknown')
            value = super().value(tool, **args)
            if self.fault == ('stop', self.head_motion_number):
                self.last_result['physical_stop_verified'] = False
            if self.fault == ('stopped_failure', self.head_motion_number):
                self.last_result.update(state='failed', reason='measured stopped failure')
            if self.fault == ('endpoint', self.head_motion_number):
                self.joints['neck_yaw_joint'] += .05
            return value
        if tool == 'rax_floor_observation':
            self.capture_positions.append({name: self.joints[name] for name in REST})
            if self.fault == 'sensor_exception':
                raise RuntimeError('camera adapter failed')
            result = super().value(tool, **args)
            if self.floor_reads in self.unreadable:
                result['ocr'].update(text=None, text_readable=False, confidence=0.)
            if self.floor_reads in self.low_confidence:
                result['ocr']['confidence'] = .3
            return result
        return super().value(tool, **args)


def configured(context=None, **changes):
    context = context or FloorViewContext()
    _, existing = ride(context)
    settings = replace(existing.settings, origin_floor_views=VIEWS,
                       destination_floor_views=tuple(dict(v, neck_yaw_joint=v['neck_yaw_joint']*.75) for v in VIEWS),
                       floor_rest_joints=REST, **changes)
    task = CabinRide(existing.task, settings)
    task._doorway_id = context.door_id
    return context, task


def test_default_floor_capture_has_no_new_motion():
    context, task = ride()
    task._doorway_id = context.door_id
    assert task.capture_origin().outcome == 'succeeded'
    assert not any(tool in ('evo_preview', 'evo_move_joints') for tool, _ in context.calls)


def test_ordered_no_read_then_success_restores_after_capture():
    context, task = configured()
    context.unreadable.add(1)
    before_travel = dict(context.scene_cfg['travel_joints'])
    assert task.capture_origin().outcome == 'succeeded'
    assert context.head_actions == [*VIEWS, REST]
    assert context.capture_positions == list(VIEWS)
    assert context.scene_cfg['travel_joints'] == before_travel
    assert {n: context.joints[n] for n in REST} == REST
    assert all(args == {'doorway_id': context.door_id}
               for tool, args in context.calls if tool == 'rax_floor_observation')
    records = [v for v in task.evidence if v['stage'] == 'floor_view_action']
    assert all(v['preview']['valid'] and v['result']['physical_stop_verified'] for v in records)
    assert [v['view_index'] for v in records] == [0, 1, 'rest']
    assert next(v for v in task.evidence if v['stage'] == 'floor_view_scan')['restoration'] == 'verified'


def test_low_confidence_can_use_next_preset_but_never_lower_threshold():
    context, task = configured()
    context.low_confidence.add(1)
    task.capture_origin()
    assert context.floor_reads == 2
    assert task.settings.min_ocr_confidence == .8


@pytest.mark.parametrize('fault', ['sensor_exception', 'stale', 'skew', 'barometer'])
def test_sensor_failure_restores_without_retrying_arbitrary_tool_errors(fault):
    context, task = configured()
    if fault == 'sensor_exception':
        context.fault = fault
    elif fault == 'stale':
        context.floor_edit = lambda v: v.update(capture_us=1)
    elif fault == 'skew':
        context.floor_edit = lambda v: v['ocr'].update(capture_us=v['capture_us']-200000)
    else:
        context.floor_edit = lambda v: v['barometer'].update(uncertainty_pa=3.)
    with pytest.raises((PrimitiveError, RuntimeError)):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0], REST]
    assert len(context.capture_positions) == 1


def test_all_no_read_uses_each_view_once_then_restores_and_fails():
    context, task = configured()
    context.unreadable.update({1, 2})
    with pytest.raises(PrimitiveError, match='did not produce qualified'):
        task.capture_origin()
    assert context.head_actions == [*VIEWS, REST]
    assert context.floor_reads == 2
    assert task._baseline is None


@pytest.mark.parametrize('fault', ['stop', 'dispatch'])
def test_uncertain_motion_never_captures_or_blindly_restores(fault):
    context, task = configured()
    context.fault = (fault, 1)
    with pytest.raises((EvoActionFailed, RuntimeError)):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0]]
    assert context.capture_positions == []
    scan = next(v for v in task.evidence if v['stage'] == 'floor_view_scan')
    assert scan['restoration'] == 'not_attempted'


def test_first_preview_refusal_has_no_action_or_capture():
    context, task = configured()
    context.fault = ('preview', 1)
    with pytest.raises(PrimitiveError, match='preview refused'):
        task.capture_origin()
    assert context.head_actions == [] and context.capture_positions == []
    assert any(v.get('state') == 'preview_refused' for v in task.evidence)


def test_later_preview_refusal_restores_from_last_verified_stop():
    context, task = configured()
    context.unreadable.add(1)
    context.fault = ('preview', 2)
    with pytest.raises(PrimitiveError, match='preview refused'):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0], REST]
    assert context.floor_reads == 1


@pytest.mark.parametrize('fault', ['endpoint', 'stopped_failure'])
def test_measured_stopped_failure_can_use_separate_restore_preview(fault):
    context, task = configured()
    context.fault = (fault, 1)
    with pytest.raises((PrimitiveError, EvoActionFailed)):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0], REST]
    assert context.capture_positions == []
    assert context.preview_number == 2


def test_restore_failure_prevents_origin_acceptance():
    context, task = configured()
    context.fault = ('preview', 2)
    with pytest.raises(PrimitiveError, match='restoration was not verified'):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0]]
    assert task._baseline is None


@pytest.mark.parametrize('change', ['limit', 'unknown', 'many', 'partial', 'list', 'no_rest'])
def test_floor_presets_require_known_source_limits_and_bounded_complete_targets(change):
    context = FloorViewContext()
    _, existing = ride(context)
    views, rest = VIEWS, REST
    if change == 'limit':
        views = ({**VIEWS[0], 'neck_pitch_joint': -.32},)
    elif change == 'unknown':
        del context.capabilities['joint_limits']['neck_pitch_joint']
    elif change == 'many':
        views = VIEWS*3
    elif change == 'partial':
        views = ({'neck_yaw_joint': .2},)
    elif change == 'list':
        views = list(VIEWS)
    else:
        rest = {}
    with pytest.raises((ValueError, PrimitiveError)):
        CabinRide(existing.task, replace(existing.settings, origin_floor_views=views, floor_rest_joints=rest))
    assert not context.head_actions


def prepare_destination(context, task):
    task.capture_origin()
    context.pose = [2., 0., math.pi]
    context.floor = '2'
    task._refresh_lighting('test admission')
    context.head_actions.clear()
    context.capture_positions.clear()


def test_destination_requires_new_capture_and_restores_before_forward_exit_check():
    context, task = configured()
    prepare_destination(context, task)
    result = task.wait_for_destination('2')
    assert result.outcome == 'succeeded'
    assert context.head_actions == [task.settings.destination_floor_views[0], REST]
    first_capture = result.measured['observation']['capture_us']
    next_result = task.wait_for_destination('2')
    assert next_result.measured['observation']['capture_us'] > first_capture
    assert len(context.head_actions) == 4


@pytest.mark.parametrize('fault', ['wrong_floor', 'pressure'])
def test_readable_wrong_destination_or_pressure_does_not_repeat_head_scan(fault):
    context, task = configured()
    prepare_destination(context, task)
    if fault == 'wrong_floor':
        context.floor_edit = lambda v: v['ocr'].update(text='1')
    else:
        context.baro_height = 0.
    with pytest.raises(PrimitiveError, match='did not verify the destination'):
        task.wait_for_destination('2')
    assert context.head_actions == [task.settings.destination_floor_views[0], REST]
    assert len(context.capture_positions) == 1



def test_cancellation_during_capture_cannot_report_success_or_move_rest():
    context, task = configured()
    cancelled = [False]
    task.base.cancelled = lambda: cancelled[0]
    context.floor_edit = lambda value: cancelled.__setitem__(0, True)
    with pytest.raises(PrimitiveError, match='return-to-rest is unconfirmed'):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0]]
    assert task._baseline is None


def test_destination_scan_deadline_is_not_relaxed_by_a_valid_capture():
    context, task = configured()
    prepare_destination(context, task)
    context.floor_edit = lambda value: context.simulation_clock.sleep(task.settings.arrival_timeout_s+1)
    with pytest.raises(PrimitiveError, match='exceeded the arrival deadline'):
        task.wait_for_destination('2')
    assert context.head_actions == [task.settings.destination_floor_views[0], REST]


def test_floor_capture_from_before_verified_view_stop_is_rejected():
    context, task = configured()
    context.floor_edit = lambda value: value.update(capture_us=task._last_motion_us)
    with pytest.raises(PrimitiveError, match='predates the last motion'):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0], REST]


def test_missing_post_motion_joint_feedback_withholds_restoration():
    context, task = configured()
    original = context.value

    def value(tool, **args):
        result = original(tool, **args)
        if tool == 'evo_status' and context.head_motion_number:
            result['feedback_fresh'] = False
        return result

    context.value = value
    with pytest.raises(PrimitiveError, match='measured robot state is unavailable'):
        task.capture_origin()
    assert context.head_actions == [VIEWS[0]]
    assert context.capture_positions == []
    assert next(v for v in task.evidence if v['stage'] == 'floor_view_scan')['restoration'] == 'not_attempted'
