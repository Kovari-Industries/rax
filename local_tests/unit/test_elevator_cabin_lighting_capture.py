"""CPU checks for immutable RGB capture receipts after stationary inference."""
from copy import deepcopy
import math

import pytest

from auto_research_agent.api import elevator_cabin
from auto_research_agent.api.base import PrimitiveError
from test_elevator_cabin import ride


@pytest.fixture
def delayed_capture(monkeypatch):
    # Exposure at 2000.0 s; inference and the current scene at 2000.2 s.
    # No sleep or simulator is required to test the two distinct timestamps.
    monkeypatch.setattr(elevator_cabin.time, 'time', lambda: 2000.2)
    monkeypatch.setattr(elevator_cabin.time, 'time_ns', lambda: 2_000_200_000_000)
    context, task = ride()
    task._doorway_id = context.door_id
    capture = context.scene(2_000_000_000)
    receipt = dict(capture['cabin']['lighting'], cabin_id='cabin-1', scene_id='scene-v1',
                   capture_scene=deepcopy(capture))
    def return_capture(value):
        value.clear()
        value.update(receipt)
    context.image_edit = return_capture
    return context, task, receipt


def test_stationary_inference_accepts_original_capture_without_retimestamp(delayed_capture):
    context, task, receipt = delayed_capture
    before = deepcopy(receipt)
    task._refresh_lighting('panel_capture', minimum_capture_us=receipt['capture_us'])
    evidence = task.evidence[-1]
    assert evidence['receipt'] == before == receipt
    assert evidence['current_capture_us']-receipt['capture_us'] == 200_000
    assert evidence['position_error_m'] == evidence['yaw_error_rad'] == 0
    assert task.settings.floor_max_age_s == 3.0
    assert task.settings.floor_max_skew_s == .1
    assert context.base_calls == []
    # Later adapter mutation cannot change the accepted evidence or cache.
    receipt['capture_scene']['pose'][0] = 99.
    assert evidence['receipt'] == before
    assert task._light_receipt == before


@pytest.mark.parametrize('field,delta', [(0, .011), (1, .011), (2, .011)])
def test_changed_capture_pose_is_refused(delayed_capture, field, delta):
    _, task, receipt = delayed_capture
    receipt['capture_scene']['pose'][field] += delta
    with pytest.raises(PrimitiveError, match='base moved since'):
        task._refresh_lighting('panel_capture')
    assert task._light_receipt is None
    assert not any(e.get('stage') == 'lighting_verified' for e in task.evidence)


def test_pose_comparison_uses_existing_tolerance_and_wrapped_yaw(delayed_capture):
    context, task, receipt = delayed_capture
    receipt['capture_scene']['pose'] = [0., 0., math.pi-.001]
    context.pose = [.009, 0., -math.pi+.001]
    task._refresh_lighting('panel_capture')
    assert task.evidence[-1]['position_error_m'] == pytest.approx(.009)
    assert task.evidence[-1]['yaw_error_rad'] == pytest.approx(.002)


@pytest.mark.parametrize('field,value,reason', [
    ('frame', 'world', 'frame or heading'),
    ('heading_source', 'unqualified_heading', 'frame or heading'),
    ('observation_id', '', 'frame or heading'),
    ('capture_us', 1_996_000_000, 'stale'),
    ('capture_us', 2_000_101_000, 'synchronized'),
])
def test_capture_scene_provenance_age_and_skew_remain_required(delayed_capture, field, value, reason):
    _, task, receipt = delayed_capture
    receipt['capture_scene'][field] = value
    with pytest.raises(PrimitiveError, match=reason):
        task._refresh_lighting('panel_capture')


@pytest.mark.parametrize('field,value', [('enabled', False), ('configuration_id', 'light-v2'),
                                       ('source', 'camera_observation')])
def test_capture_lighting_change_is_refused(delayed_capture, field, value):
    _, task, receipt = delayed_capture
    receipt['capture_scene']['cabin']['lighting'][field] = value
    with pytest.raises(PrimitiveError, match='lighting|does not match'):
        task._refresh_lighting('panel_capture')


@pytest.mark.parametrize('field,value', [('light_enabled', False),
                                       ('light_configuration_id', 'light-v2'),
                                       ('cabin_scene_id', 'scene-v2')])
def test_live_lighting_change_after_exposure_is_refused(delayed_capture, field, value):
    context, task, _ = delayed_capture
    original = context.image_edit
    def change(value):
        original(value)
        setattr(context, field, value)
    context.image_edit = change
    with pytest.raises(PrimitiveError, match='lighting|scene'):
        task._refresh_lighting('panel_capture')


def test_current_lighting_source_must_equal_capture_source(delayed_capture):
    context, task, _ = delayed_capture
    original_image, original_scene = context.image_edit, context.scene
    def change(value):
        original_image(value)
        def new_scene(stamp=None):
            scene = original_scene(stamp)
            scene['cabin']['lighting']['source'] = 'camera_observation'
            return scene
        context.scene = new_scene
    context.image_edit = change
    with pytest.raises(PrimitiveError, match='lighting source changed'):
        task._refresh_lighting('panel_capture')


@pytest.mark.parametrize('mode,reason', [('stale_image', 'stale'), ('old_capture', 'predates the decision'),
                                       ('last_motion', 'predates the last motion'),
                                       ('last_stop', 'predates the last motion')])
def test_delayed_receipt_does_not_relax_other_time_gates(delayed_capture, mode, reason):
    _, task, receipt = delayed_capture
    minimum = 0
    if mode == 'stale_image':
        receipt['capture_us'] -= 4_000_000
    elif mode == 'old_capture':
        minimum = receipt['capture_us']+1
    elif mode == 'last_motion':
        task._last_motion_us = receipt['capture_us']
    else:
        task.task._last_stop_us = receipt['capture_us']
    with pytest.raises(PrimitiveError, match=reason):
        task._refresh_lighting('panel_capture', minimum_capture_us=minimum)


def test_legacy_adapter_still_requires_exact_scene_sync(delayed_capture):
    _, task, receipt = delayed_capture
    receipt.pop('capture_scene')
    with pytest.raises(PrimitiveError, match='not synchronized'):
        task._refresh_lighting('panel_capture')
    receipt['capture_us'] += 200_000
    task._refresh_lighting('panel_capture')


def test_explicit_invalid_capture_scene_cannot_fall_back_to_current(delayed_capture):
    _, task, receipt = delayed_capture
    receipt['capture_scene'] = None
    with pytest.raises(PrimitiveError, match='frame or heading'):
        task._refresh_lighting('panel_capture')


@pytest.mark.parametrize('field,value', [('frame', 'world'), ('heading_source', 'unknown')])
def test_current_scene_frame_and_heading_remain_qualified(delayed_capture, field, value):
    context, task, _ = delayed_capture
    original = context.value
    def changed_scene(tool, **args):
        result = original(tool, **args)
        if tool == 'rax_scene' and context.image_calls:
            result[field] = value
        return result
    context.value = changed_scene
    with pytest.raises(PrimitiveError, match='scene frame or heading'):
        task._refresh_lighting('panel_capture')
