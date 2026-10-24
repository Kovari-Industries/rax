"""CPU-only authorization checks for a stopped, source-qualified neck view."""
from copy import deepcopy
from dataclasses import replace
import time

import pytest

from auto_research_agent.api.base import PrimitiveError
from auto_research_agent.api.elevator_cabin import CabinRide
from test_elevator_cabin_floor_views import FloorViewContext, REST, VIEWS, configured, prepare_destination


DESTINATION_VIEWS = tuple(dict(v, neck_yaw_joint=v['neck_yaw_joint']*.75) for v in VIEWS)


def retained(context=None):
    context = context or FloorViewContext()
    context.capabilities['observation_travel_postures'] = [dict(REST), *deepcopy(DESTINATION_VIEWS)]
    return configured(context, retain_destination_view=True)


def test_retained_view_uses_new_capture_without_restoration_or_retimestamp():
    context, task = retained()
    prepare_destination(context, task)
    first = task.wait_for_destination('2')
    stamp = first.measured['observation']['capture_us']
    assert first.outcome == 'succeeded'
    assert context.head_actions == [DESTINATION_VIEWS[0]]
    assert {name: context.joints[name] for name in REST} == DESTINATION_VIEWS[0]
    scan = [e for e in task.evidence if e['stage'] == 'floor_view_scan'][-1]
    assert scan['restoration'] == 'retained_travel_view'
    assert scan['destination_check']['observation']['capture_us'] == stamp
    assert task._last_motion_us < stamp
    second = task.wait_for_destination('2')
    assert second.measured['observation']['capture_us'] > stamp
    assert context.head_actions == [DESTINATION_VIEWS[0], DESTINATION_VIEWS[0]]


def test_retained_view_complete_ride_has_fresh_authorization_after_exit_preview():
    context, task = retained()
    events = []
    original = context.value
    def record(tool, **args):
        if tool in {'rax_base_preview', 'rax_move_base', 'rax_scene', 'evo_status'}:
            events.append(tool)
        return original(tool, **args)
    context.value = record
    result = task.run('2')
    assert result.outcome == 'succeeded'
    assert context.head_actions == [VIEWS[0], REST, DESTINATION_VIEWS[0], DESTINATION_VIEWS[0]]
    last_preview = max(i for i, name in enumerate(events) if name == 'rax_base_preview')
    last_dispatch = max(i for i, name in enumerate(events) if name == 'rax_move_base')
    assert 'rax_scene' in events[last_preview+1:last_dispatch]
    assert 'evo_status' in events[last_preview+1:last_dispatch]
    authorization = [e for e in task.evidence if e['stage'] == 'exit_floor_authorization'][-1]
    assert authorization['accepted'] is True
    assert authorization['decision_wall_us'] >= authorization['observation']['capture_us']


@pytest.mark.parametrize('change', ['missing', 'partial', 'different', 'outside_source', 'unknown_source'])
def test_retention_requires_explicit_exact_complete_source_qualified_postures(change):
    context, task = retained()
    postures = context.capabilities['observation_travel_postures']
    if change == 'missing':
        context.capabilities.pop('observation_travel_postures')
    elif change == 'partial':
        postures[1].pop('neck_pitch_joint')
    elif change == 'different':
        postures[1]['neck_yaw_joint'] += .000001
    elif change == 'outside_source':
        postures[0]['neck_yaw_joint'] = 1.7
    else:
        context.capabilities['model']['limits'].pop('neck_yaw_joint')
    with pytest.raises(PrimitiveError, match='admitted|both neck|source|exactly'):
        CabinRide(task.task, task.settings)
    assert context.head_actions == []


@pytest.mark.parametrize('change', ['not_bool', 'no_views'])
def test_retention_setting_is_explicit_and_has_a_view(change):
    _, task = retained()
    updates = {'retain_destination_view': 1} if change == 'not_bool' else {'destination_floor_views': ()}
    with pytest.raises(ValueError, match='boolean|requires configured'):
        CabinRide(task.task, replace(task.settings, **updates))


@pytest.mark.parametrize('fault', ['wrong_floor', 'wrong_pressure', 'stale', 'cancelled'])
def test_failed_retained_scan_does_not_keep_an_unverified_view(fault):
    context, task = retained()
    prepare_destination(context, task)
    if fault == 'wrong_floor':
        context.floor_edit = lambda value: value['ocr'].update(text='1')
    elif fault == 'wrong_pressure':
        context.baro_height = 0.
    elif fault == 'stale':
        context.floor_edit = lambda value: value.update(capture_us=value['capture_us']-4_000_000)
    else:
        cancelled = [False]
        task.base.cancelled = lambda: cancelled[0]
        context.floor_edit = lambda value: cancelled.__setitem__(0, True)
    with pytest.raises(PrimitiveError):
        task.wait_for_destination('2')
    expected = [DESTINATION_VIEWS[0]] if fault == 'cancelled' else [DESTINATION_VIEWS[0], REST]
    assert context.head_actions == expected
    assert task._retained_floor_view is None
    assert not context.base_calls


def test_default_slow_restoration_refuses_expired_floor_capture(monkeypatch):
    context, task = configured()
    prepare_destination(context, task)
    original_value, real_time, real_ns = context.value, time.time, time.time_ns
    offset = [0.]
    monkeypatch.setattr(time, 'time', lambda: real_time()+offset[0])
    monkeypatch.setattr(time, 'time_ns', lambda: real_ns()+int(offset[0]*1e9))
    def slow_rest(tool, **args):
        result = original_value(tool, **args)
        if tool == 'evo_move_joints' and args['targets'] == REST:
            offset[0] += 4.
        return result
    context.value = slow_rest
    with pytest.raises(PrimitiveError, match='stale'):
        task.wait_for_destination('2')
    assert context.head_actions == [DESTINATION_VIEWS[0], REST]
    assert not context.base_calls


@pytest.mark.parametrize('change', ['expired', 'closed_door', 'pose_drift', 'light_off',
                                   'posture_drift', 'admission_revoked', 'new_motion', 'cancelled'])
def test_exit_preview_changes_prevent_base_dispatch(monkeypatch, change):
    context, task = retained()
    prepare_destination(context, task)
    original_value, real_time, real_ns = context.value, time.time, time.time_ns
    offset, cancelled = [0.], [False]
    monkeypatch.setattr(time, 'time', lambda: real_time()+offset[0])
    monkeypatch.setattr(time, 'time_ns', lambda: real_ns()+int(offset[0]*1e9))
    task.base.cancelled = lambda: cancelled[0]
    def changed_after_preview(tool, **args):
        result = original_value(tool, **args)
        if tool == 'rax_base_preview':
            if change == 'expired':
                offset[0] += 4.
            elif change == 'closed_door':
                context.door_state = 'closed'
            elif change == 'pose_drift':
                context.pose[0] += .011
            elif change == 'light_off':
                context.light_enabled = False
            elif change == 'posture_drift':
                context.joints['neck_yaw_joint'] += .011
            elif change == 'admission_revoked':
                context.capabilities['observation_travel_postures'] = [dict(REST)]
            elif change == 'new_motion':
                task._last_motion_us = time.time_ns()//1000
            else:
                cancelled[0] = True
        return result
    context.value = changed_after_preview
    with pytest.raises(PrimitiveError):
        task.exit_forward('2')
    assert not context.base_calls
    assert any(name == 'rax_base_preview' for name, _ in context.calls)
    assert not any(name == 'rax_move_base' for name, _ in context.calls)


def test_refused_preview_never_calls_before_dispatch():
    context, task = retained()
    original = context.value
    def refused(tool, **args):
        if tool == 'rax_base_preview':
            return {'valid': False}
        return original(tool, **args)
    context.value = refused
    calls = []
    with pytest.raises(PrimitiveError, match='preview refused'):
        task.base.translate_base(.1, 0., before_dispatch=lambda: calls.append(True))
    assert not calls and not context.base_calls


def test_cancel_after_before_dispatch_still_prevents_motion():
    context, task = retained()
    cancelled = [False]
    task.base.cancelled = lambda: cancelled[0]
    with pytest.raises(PrimitiveError, match='cancelled'):
        task.base.translate_base(.1, 0., before_dispatch=lambda: cancelled.__setitem__(0, True))
    assert not context.base_calls


def test_slow_final_scene_query_cannot_expire_the_authorized_floor_capture(monkeypatch):
    context, task = retained()
    prepare_destination(context, task)
    original, real_time, real_ns = context.value, time.time, time.time_ns
    offset, preview_done = [0.], [False]
    monkeypatch.setattr(time, 'time', lambda: real_time()+offset[0])
    monkeypatch.setattr(time, 'time_ns', lambda: real_ns()+int(offset[0]*1e9))
    def delayed(tool, **args):
        if tool == 'rax_scene' and preview_done[0]:
            offset[0] += 4.
        result = original(tool, **args)
        if tool == 'rax_base_preview':
            preview_done[0] = True
        return result
    context.value = delayed
    with pytest.raises(PrimitiveError, match='stale'):
        task.exit_forward('2')
    assert not context.base_calls
