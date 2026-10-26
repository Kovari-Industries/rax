"""CPU contract checks. Sensor observations are synthetic; no physics runs here."""
from dataclasses import replace
import math
import time

import pytest

from test_elevator import settings
from test_elevator_sim import SimulationContext
from auto_research_agent.api.base import PrimitiveError
from auto_research_agent.api.elevator_cabin import CabinRide, CabinRideSettings
from auto_research_agent.api.elevator_sim import SimulationElevator


class Clock:
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class CabinContext(SimulationContext):
    def __init__(self, panel_y=-0.35):
        super().__init__()
        self.simulation_clock = Clock()
        self.panel_y = panel_y
        self.floor = '1'
        self.floor_reads = 0
        self.width = 2.4
        self.luma = .4
        self.baro_height = 3.6
        self.floor_edit = None
        self.turn_refused = False
        self.turn_endpoint_error = 0.
        self.turn_stop_verified = True
        self.panel_location = 'inside_cabin'
        self.confirm_activation = True
        self.light_enabled = True
        self.light_configuration_id = "light-v1"
        self.cabin_scene_id = "scene-v1"
        self.light_image_age_us = 0
        self.image_calls = 0
        self.image_edit = None

    def scene(self, stamp=None):
        if stamp is None:
            # Synthetic moves complete in one call. Let the wall clock advance
            # before exposure so its microsecond stamp is after the stop.
            time.sleep(0)
            stamp = time.time_ns()//1000
        value = super().scene(stamp)
        value['heading_source'] = 'simulator_ground_truth'
        value['panel'] = dict(id='test-panel', location=self.panel_location, cabin_id='cabin-1',
                              point=self.local((1.2, self.panel_y, 1.0)),
                              normal=[-math.cos(self.pose[2]), math.sin(self.pose[2]), 0.], uncertainty_m=.001)
        value['cabin'] = dict(id='cabin-1', scene_id=self.cabin_scene_id, doorway_id=self.door_id, panel_id='test-panel',
                              polygon_m=[self.local(p)[:2] for p in [(1., -self.width/2, 0.),
                                  (3.4, -self.width/2, 0.), (3.4, self.width/2, 0.), (1., self.width/2, 0.)]],
                              ceiling_height_m=2.5, geometry_uncertainty_m=.001,
                              lighting=dict(enabled=self.light_enabled, configuration_id=self.light_configuration_id,
                                            source='authored_simulation', capture_us=stamp-self.light_image_age_us,
                                            image_sha256='a'*64, mean_luma=self.luma))
        return value

    def value(self, tool, **args):
        if tool == 'rax_cabin_image':
            self.calls.append((tool, args))
            self.image_calls += 1
            scene = self.scene()
            value = dict(scene['cabin']['lighting'], cabin_id='cabin-1', scene_id=self.cabin_scene_id)
            if self.image_edit:
                self.image_edit(value)
            return value
        if tool == 'rax_floor_observation':
            self.calls.append((tool, args))
            self.floor_reads += 1
            stamp = time.time_ns()//1000
            height = 0 if self.floor == '1' else self.baro_height
            pressure = 101325*math.exp(-height*9.80665/(287.05*293.15))
            value = dict(observation_id=f'floor-{self.floor_reads}', capture_us=stamp,
                         ocr=dict(source='image_ocr', text=self.floor, text_readable=True, confidence=.98,
                                  capture_us=stamp, image_sha256='b'*64, doorway_id=self.door_id,
                                  location='landing_door_jamb'),
                         barometer=dict(source='simulated_barometer', pressure_pa=pressure,
                                        uncertainty_pa=.25, capture_us=stamp, stable=True,
                                        sample_count=5, span_pa=.2), scene=self.scene(stamp))
            if self.floor_edit:
                self.floor_edit(value)
            return value
        if tool == 'sim_button_observation':
            self.calls.append((tool, args))
            return dict(source='simulator_ground_truth', label=args['label'],
                        position=self.local((1.2, self.panel_y, 1.0)),
                        normal=[-math.cos(self.pose[2]), math.sin(self.pose[2]), 0.],
                        scene_pose=self.pose.copy(), capture_us=time.time_ns()//1000,
                        observation_id=f'button-{len(self.calls)}', uncertainty_m=.001)
        if tool == 'sim_button_activation':
            self.floor = '2'
            return dict(source='simulator_ground_truth', target='2', single_correct_press=self.confirm_activation,
                        released=True)
        if tool == 'rax_base_preview' and args['delta'][2] and self.turn_refused:
            self.calls.append((tool, args))
            return dict(valid=False)
        value = super().value(tool, **args)
        if tool == 'rax_move_base' and args['delta'][2]:
            self.pose[2] += self.turn_endpoint_error
            self.last_result['physical_stop_verified'] = self.turn_stop_verified
        return value


def ride(context=None, **changes):
    context = context or CabinContext()
    cfg = CabinRideSettings('1', {'1': 0., '2': 3.6}, entry_depth_m=1.0,
                            exit_depth_m=.5, arrival_timeout_s=.2, **changes)
    return context, CabinRide(SimulationElevator(context, settings(direct_doorway_staging=True)), cfg)


@pytest.mark.parametrize('panel_y,expected_arm', [(-.35, 'left'), (.35, 'right')])
def test_complete_ride_enters_then_turns_and_selects_mirrored_arm(panel_y, expected_arm):
    context, task = ride(CabinContext(panel_y))
    events = []
    result = task.run('2', on_stage=events.append)
    assert result.outcome == 'succeeded'
    assert result.measured['stages']['button']['measured']['arm'] == expected_arm
    assert context.pose == pytest.approx([.5, 0., math.pi])
    assert [e['stage'] for e in events if e['event'] == 'finished'] == [
        'origin', 'entry', 'turn', 'button', 'destination', 'exit']
    moves = [(i, a) for i, (t, a) in enumerate(context.calls) if t == 'rax_move_base']
    turn = [(i, a) for i, a in moves if a['delta'][2]]
    assert len(turn) == 1 and turn[0][1]['delta'][2] == math.pi
    assert turn[0][0] < next(i for i, (t, _) in enumerate(context.calls) if t == 'sim_button_observation')
    assert moves[-1][1]['delta'][0] > 0 and moves[-1][1]['doorway_id'] == context.door_id
    assert context.floor_reads >= 3  # Origin, destination, fresh pre-exit recheck.
    assert context.joints == context.scene_cfg['travel_joints']
    with pytest.raises(PrimitiveError, match='cannot repeat'):
        task.run('2')


def test_arm_refusal_selects_other_arm_without_relaxing_preview():
    context, task = ride()
    context.preview_refused.add('left')
    result = task.run('2')
    selection = result.measured['stages']['button']['measured']
    assert selection['arm'] == 'right'
    assert any(c['side'] == 'left' and not c['feasible'] for c in selection['arm_candidates'])


def test_no_reachable_arm_stops_before_contact_or_exit():
    context, task = ride()
    context.preview_refused.update({'left', 'right'})
    with pytest.raises(PrimitiveError, match='neither arm'):
        task.run('2')
    assert context.pose[0] == pytest.approx(2.)
    assert not any(t == 'evo_follow_path' for t, _ in context.calls)


@pytest.mark.parametrize('change,reason', [
    ('small_cabin', 'rotation envelope'), ('dark', 'too dark'), ('outside_panel', 'interior panel'),
    ('refused_turn', 'preview refused'), ('bad_turn_endpoint', 'turnaround endpoint'),
    ('turn_not_stopped', 'did not succeed'), ('unconfirmed_activation', 'activation')])
def test_unsafe_turn_or_interior_press_cannot_complete(change, reason):
    context, task = ride()
    if change == 'small_cabin':
        context.width = .7
    elif change == 'dark':
        context.luma = .01
    elif change == 'outside_panel':
        context.panel_location = 'outside_cabin'
    elif change == 'refused_turn':
        context.turn_refused = True
    elif change == 'bad_turn_endpoint':
        context.turn_endpoint_error = .02
    elif change == 'turn_not_stopped':
        context.turn_stop_verified = False
    elif change == 'unconfirmed_activation':
        context.confirm_activation = False
    with pytest.raises(RuntimeError, match=reason):
        task.run('2')
    assert context.floor_reads == 1
    if change in {'small_cabin', 'dark', 'refused_turn'}:
        assert not any(t == 'rax_move_base' and a['delta'][2] for t, a in context.calls)


@pytest.mark.parametrize('mode', ['wrong_ocr', 'pressure_wrong_floor', 'pressure_drift'])
def test_destination_needs_both_correct_ocr_and_pressure_delta(mode):
    context, task = ride()
    def edit(value):
        if context.floor == '2' and mode == 'wrong_ocr':
            value['ocr']['text'] = '1'
    context.floor_edit = edit
    if mode == 'pressure_wrong_floor':
        context.baro_height = 0.
    elif mode == 'pressure_drift':
        context.baro_height = 5.
    with pytest.raises(PrimitiveError, match='did not agree'):
        task.run('2')
    assert context.pose[0] == pytest.approx(2.)
    assert task.evidence[-1]['accepted'] is False


@pytest.mark.parametrize('mode,reason', [
    ('missing_ocr', 'OCR'), ('ground_truth', 'OCR'), ('stale', 'stale'),
    ('uncertain', 'uncertainty'), ('unstable', 'stable'), ('low_confidence', 'confidence'),
    ('wrong_door', 'OCR'), ('bad_image_hash', 'OCR'), ('bad_sync', 'synchronized'),
    ('few_samples', 'samples')])
def test_invalid_floor_observation_fails_before_entry(mode, reason):
    context, task = ride()
    def edit(value):
        if mode == 'missing_ocr':
            value.pop('ocr')
        elif mode == 'ground_truth':
            value['ocr']['source'] = 'simulator_ground_truth'
        elif mode == 'stale':
            value['capture_us'] -= 4_000_000
        elif mode == 'uncertain':
            value['barometer']['uncertainty_pa'] = 10.
        elif mode == 'unstable':
            value['barometer']['stable'] = False
        elif mode == 'low_confidence':
            value['ocr']['confidence'] = .2
        elif mode == 'wrong_door':
            value['ocr']['doorway_id'] = 'another-door'
        elif mode == 'bad_image_hash':
            value['ocr']['image_sha256'] = ''
        elif mode == 'bad_sync':
            value['barometer']['capture_us'] -= 200000
        elif mode == 'few_samples':
            value['barometer']['sample_count'] = 1
    context.floor_edit = edit
    with pytest.raises(PrimitiveError, match=reason):
        task.run('2')
    assert not context.base_calls


def test_late_floor_change_or_door_closure_cannot_reuse_verified_destination():
    for change in ('wrong_floor', 'close_door'):
        context, task = ride()
        def callback(event):
            if event['stage'] == 'destination' and event['event'] == 'finished':
                if change == 'close_door':
                    context.door_state = 'closing'
                else:
                    context.floor = '1'
        with pytest.raises(PrimitiveError, match='did not agree'):
            task.run('2', on_stage=callback)
        assert context.pose[0] == pytest.approx(2.)


def test_checked_turn_respects_existing_per_command_rotation_limit():
    context, task = ride()
    context.scene_cfg['max_turn_rad'] = math.pi/2
    result = task.run('2')
    turns = result.measured['stages']['turn']['measured']['turns']
    assert len(turns) == 2
    assert all(t['measured']['angle_travelled_rad'] == math.pi/2 for t in turns)


def test_turn_bound_does_not_use_only_axis_aligned_endpoint_footprint():
    context, task = ride()
    context.width = .7
    # Current half-width .2 fits, but its .320m rotation radius plus reserve does not.
    assert context.width/2 > context.scene_cfg['footprint'][1]+.061
    with pytest.raises(PrimitiveError, match='rotation envelope'):
        task.run('2')


def test_settings_reject_overlap_partial_turn_and_single_arm():
    context, task = ride()
    with pytest.raises(ValueError, match='180'):
        replace(task.settings, turn_rad=math.pi/2).validate()
    with pytest.raises(ValueError, match='overlap'):
        replace(task.settings, floor_heights_m={'1': 0., '2': 1.}).validate()
    with pytest.raises(ValueError, match='both'):
        CabinRide(SimulationElevator(context, settings(arms=settings().arms[:1])), task.settings)
    with pytest.raises(ValueError, match='must not turn'):
        CabinRide(SimulationElevator(context, settings(search_turns_rad=(.1,))), task.settings)


class HallTask:
    """A CPU action stub. Physics validation must supply a real Elevator task."""
    def __init__(self, context, *, confirmed=True):
        self.settings = settings(panel_id='hall-panel', press_offsets={'CALL': (0, 0)})
        self.context, self.confirmed, self.calls = context, confirmed, []

    def press_elevator_button(self, label):
        from auto_research_agent.api.base import PrimitiveResult
        self.calls.append(label)
        if self.confirmed:
            self.context.door_state = 'open'
        return PrimitiveResult('press_elevator_button', 'succeeded' if self.confirmed else 'stroke_complete_unconfirmed',
                               measured={'activation': 'succeeded' if self.confirmed else 'unknown'})


def test_closed_origin_uses_single_separate_hall_button_before_entry():
    context, task = ride()
    context.door_state = 'closed'
    hall = HallTask(context)
    task = CabinRide(task.task, task.settings, hall_task=hall)
    events = []
    result = task.run('2', on_stage=events.append)
    assert result.outcome == 'succeeded'
    assert hall.calls == ['CALL']
    assert [e['stage'] for e in events if e['event'] == 'finished'][:3] == ['origin', 'hall_call', 'entry']


@pytest.mark.parametrize('supply_hall', [False, True])
def test_closed_origin_without_confirmed_hall_call_cannot_enter(supply_hall):
    context, task = ride()
    context.door_state = 'closed'
    hall = HallTask(context, confirmed=False) if supply_hall else None
    task = CabinRide(task.task, task.settings, hall_task=hall)
    with pytest.raises(PrimitiveError, match='hall call'):
        task.run('2')
    assert not context.base_calls
    assert context.floor_reads == 1


def test_hall_task_cannot_reuse_inside_panel_or_have_destination_controls():
    context, task = ride()
    hall = HallTask(context)
    hall.settings = replace(hall.settings, panel_id='test-panel')
    with pytest.raises(ValueError, match='distinct'):
        CabinRide(task.task, task.settings, hall_task=hall)
    hall.settings = replace(hall.settings, panel_id='hall-panel', press_offsets={'CALL': (0, 0), '2': (0, 0)})
    with pytest.raises(ValueError, match='exactly one'):
        CabinRide(task.task, task.settings, hall_task=hall)


def test_full_robot_height_must_fit_cabin_before_entry():
    context, task = ride()
    context.scene_cfg['footprint'][2] = 2.49
    with pytest.raises(PrimitiveError, match='ceiling'):
        task.run('2')
    assert not context.base_calls


def test_stale_destination_observation_after_press_cannot_authorize_exit():
    context, task = ride()
    def edit(value):
        if context.floor == '2':
            value['ocr']['capture_us'] = 1
    context.floor_edit = edit
    with pytest.raises(PrimitiveError, match='stale'):
        task.run('2')
    assert context.pose[0] == pytest.approx(2.)


def test_intermediate_turn_checks_do_not_request_images_or_reject_old_rgb_receipt():
    context, task = ride()
    counts = {}
    def callback(event):
        if event['stage'] == 'entry' and event['event'] == 'finished':
            counts['before'] = context.image_calls
            context.light_image_age_us = 10_000_000
        if event['stage'] == 'turn' and event['event'] == 'finished':
            counts['after'] = context.image_calls
            context.light_image_age_us = 0
    assert task.run('2', on_stage=callback).outcome == 'succeeded'
    assert counts == {'before': 1, 'after': 1}
    captures = sum(t == 'sim_button_observation' for t, _ in context.calls)
    assert context.image_calls == captures+1  # Admission plus cached panel images; floor reuses its RGB.


@pytest.mark.parametrize('change', ['light_off', 'scene_changed', 'light_changed', 'geometry_stale'])
def test_continuous_cheap_state_and_geometry_gates_still_refuse(change):
    context, task = ride()
    def callback(event):
        if event['stage'] == 'entry' and event['event'] == 'finished':
            if change == 'light_off':
                context.light_enabled = False
            elif change == 'scene_changed':
                context.cabin_scene_id = 'scene-v2'
            elif change == 'light_changed':
                context.light_configuration_id = 'light-v2'
            else:
                context.stale = True
    with pytest.raises(PrimitiveError):
        task.run('2', on_stage=callback)
    assert context.image_calls == 1
    assert not any(t == 'rax_move_base' and a['delta'][2] for t, a in context.calls)


@pytest.mark.parametrize('change,reason', [('dark', 'too dark'), ('stale', 'stale'),
                                         ('other_scene', 'does not match'), ('old_capture', 'predates')])
def test_each_panel_capture_requires_current_matching_rgb_and_restores_observer(change, reason):
    context, task = ride()
    original = task.task.observe_button
    def edit(value):
        if context.image_calls > 1:
            if change == 'dark':
                value['mean_luma'] = .01
            elif change == 'stale':
                value['capture_us'] -= 4_000_000
            elif change == 'other_scene':
                value['scene_id'] = 'another-scene'
            else:
                value['capture_us'] -= 1000
    context.image_edit = edit
    with pytest.raises(PrimitiveError, match=reason):
        task.run('2')
    assert task.task.observe_button == original
    assert 'observe_button' not in task.task.__dict__
    assert context.pose[0] == pytest.approx(2.)


def test_floor_capture_requires_its_fresh_brightness_receipt_without_extra_image_call():
    context, task = ride()
    def edit(value):
        if context.floor == '2':
            value['scene']['cabin']['lighting']['capture_us'] -= 4_000_000
    context.floor_edit = edit
    with pytest.raises(PrimitiveError, match='stale'):
        task.run('2')
    captures = sum(t == 'sim_button_observation' for t, _ in context.calls)
    assert context.image_calls == captures+1
    assert context.pose[0] == pytest.approx(2.)


@pytest.mark.parametrize('entry_depth,can_turn', [(.65, False), (.84, True), (.87, True), (.90, True)])
def test_new_cabin_entry_depth_uses_front_wall_not_door_plane(entry_depth, can_turn):
    context, task = ride()
    original = context.scene
    def rectangle(stamp=None):
        scene = original(stamp)
        scene['cabin']['polygon_m'] = [context.local(p)[:2] for p in
            [(2., -1.05, 0.), (4., -1.05, 0.), (4., 1.05, 0.), (2., 1.05, 0.)]]
        return scene
    context.scene = rectangle
    context.pose = [1.89+entry_depth, 0., 0.]
    context.scene_cfg.update(footprint=[.387847231, .379347231, 1.8], margin_m=.03,
                             position_tolerance_m=.015)
    task._doorway_id = context.door_id
    task._refresh_lighting('admission')
    if can_turn:
        _, _, clearance = task._cabin(task._scene(), turn_clearance=True)
        assert clearance > 0
    else:
        with pytest.raises(PrimitiveError, match='rotation envelope'):
            task._cabin(task._scene(), turn_clearance=True)
    assert not context.base_calls


@pytest.mark.parametrize('lateral', [-.35, .35])
def test_optional_lateral_staging_is_inside_after_entry_before_turn(lateral):
    context, task = ride(cabin_staging_lateral_m=lateral)
    events = []
    assert task.run('2', on_stage=events.append).outcome == 'succeeded'
    names = [e['stage'] for e in events if e['event'] == 'finished']
    assert names == ['origin', 'entry', 'cabin_staging', 'turn', 'button', 'destination', 'exit']
    staging = next(e['result']['measured'] for e in events if e['stage'] == 'cabin_staging' and e['event'] == 'finished')
    assert staging['start_pose'] == pytest.approx([2., 0., 0.])
    assert staging['final_pose'] == pytest.approx([2., lateral, 0.])
    assert staging['minimum_target_turn_clearance_m'] > 0
    moves = [(i, args) for i, (tool, args) in enumerate(context.calls) if tool == 'rax_move_base']
    shift = next((i, args) for i, args in moves if args['delta'] == [0., lateral, 0.])
    turn = next((i, args) for i, args in moves if args['delta'][2])
    assert shift[0] < turn[0]
    assert shift[1].get('doorway_id') is None  # The robot is already wholly inside.


def test_lateral_target_turn_envelope_refusal_precedes_translation():
    context, task = ride(cabin_staging_lateral_m=.9)
    with pytest.raises(PrimitiveError, match='rotation envelope'):
        task.run('2')
    assert context.pose == pytest.approx([2., 0., 0.])
    assert not any(t == 'rax_move_base' and a['delta'][1] for t, a in context.calls)


@pytest.mark.parametrize('failure', ['preview', 'stop', 'endpoint'])
def test_lateral_staging_requires_preview_stop_and_measured_endpoint(failure):
    context, task = ride(cabin_staging_lateral_m=.35)
    original = context.value
    def value(tool, **args):
        is_staging = args.get('delta') == [0., .35, 0.]
        if is_staging and tool == 'rax_base_preview' and failure == 'preview':
            return {'valid': False}
        result = original(tool, **args)
        if is_staging and tool == 'rax_move_base':
            if failure == 'stop':
                context.last_result['physical_stop_verified'] = False
            elif failure == 'endpoint':
                context.pose[1] += .02
        return result
    context.value = value
    with pytest.raises(RuntimeError):
        task.run('2')
    assert not any(t == 'rax_move_base' and a['delta'][2] for t, a in context.calls)
    assert context.floor_reads == 1


def test_ordinary_scene_requires_only_cheap_light_state_fields():
    context, task = ride()
    original = context.value
    def value(tool, **args):
        result = original(tool, **args)
        if tool == 'rax_scene':
            light = result['cabin']['lighting']
            for key in ('capture_us', 'mean_luma', 'image_sha256'):
                light.pop(key)
        return result
    context.value = value
    assert task.run('2').outcome == 'succeeded'


def test_preexisting_button_observer_override_is_restored():
    context, task = ride()
    original = task.task.observe_button
    override = lambda *args, **kwargs: original(*args, **kwargs)
    task.task.observe_button = override
    assert task.run('2').outcome == 'succeeded'
    assert task.task.observe_button is override


@pytest.mark.parametrize("reported_normal_sign", [-1., 1.])
def test_forward_exit_orients_stable_door_normal_after_turn(reported_normal_sign):
    context, task = ride()
    context.pose = [2., 0., math.pi]
    original_scene = context.scene

    def scene(stamp=None):
        value = original_scene(stamp)
        # Stable world +X is body -X after the turn. Either plane-normal sign
        # must yield the same forward exit direction without mutating the scene.
        value["doorway"]["normal"] = [reported_normal_sign, 0., 0.]
        return value

    context.scene = scene
    _, door, center, normal = task.base.target("doorway", context.door_id)
    assert center[0] > 0
    assert normal == pytest.approx([1., 0., 0.])
    assert door["normal"] == [reported_normal_sign, 0., 0.]
    goal = [center[i]+normal[i]*task.settings.exit_depth_m for i in range(2)]
    assert goal[0] > center[0]
    assert goal[1] == pytest.approx(0., abs=1e-12)


@pytest.mark.parametrize("elapsed", [20., 20.01, -1., float("nan")])
@pytest.mark.parametrize("button_stage", ["hall_call", "button"])
def test_full_button_deadline_refuses_next_stage_without_runner(elapsed, button_stage):
    from auto_research_agent.api.base import PrimitiveResult
    context, task = ride(full_button_timeout_s=20.)
    if button_stage == "hall_call":
        context.door_state = "closed"
    def delayed_action(*args):
        context.simulation_clock.t += elapsed
        return PrimitiveResult("press_elevator_button", "succeeded")
    if button_stage == "hall_call":
        task.call_elevator = delayed_action
    else:
        task.press_cabin_button = delayed_action
    events = []
    with pytest.raises(PrimitiveError, match="time limit|clock"):
        task.run("2", on_stage=events.append)
    assert events[-1]["stage"] == button_stage and events[-1]["event"] == "failed"
    assert not any(e["stage"] == ("entry" if button_stage == "hall_call" else "destination") for e in events)
    assert not task.evidence[-1]["passed"]


@pytest.mark.parametrize("limit", [0., -1., 20.000001, 30., float("nan"), float("inf"), True])
def test_invalid_full_button_timeout_is_refused(limit):
    with pytest.raises(ValueError):
        ride(full_button_timeout_s=limit)


def test_full_button_deadline_accepts_completed_action_below_limit():
    context, task = ride(full_button_timeout_s=20.)
    original = task.press_cabin_button
    def measured_action(label):
        result = original(label)
        context.simulation_clock.t += 19.9
        return result
    task.press_cabin_button = measured_action
    assert task.run("2").outcome == "succeeded"
    receipt = next(e for e in task.evidence if e.get("check") == "full_button_deadline")
    assert receipt["passed"] and 19.9 <= receipt["duration_s"] < 20.


@pytest.mark.parametrize('arm',['left','right'])
def test_explicit_cabin_arm_uses_only_its_view_and_contact(arm):
    context,task=ride(cabin_press_arm=arm)
    result=task.run('2')
    assert result.measured['stages']['button']['measured']['arm']==arm
    captures=[a for t,a in context.calls if t=='sim_button_observation']
    assert captures and {a['side'] for a in captures}=={arm}
    other='left' if arm=='right' else 'right'
    for tool,args in context.calls:
        if tool=='evo_move_joints':
            assert all(value==context.scene_cfg['travel_joints'][name]
                       for name,value in args['targets'].items() if name.startswith(other+'_joint'))
        if tool=='evo_follow_path':
            assert all(other not in waypoint for waypoint in args['waypoints'])
    assert context.joints==context.scene_cfg['travel_joints']
    assert any(e.get('configured_arm')==arm for e in task.evidence)


@pytest.mark.parametrize('arm',['left','right'])
def test_explicit_cabin_arm_refusal_does_not_fall_back(arm):
    context,task=ride(cabin_press_arm=arm)
    context.preview_refused.add(arm)
    with pytest.raises(PrimitiveError,match='checked cabin approach'):
        task.run('2')
    assert not any(t=='evo_follow_path' for t,_ in context.calls)
    assert {a['side'] for t,a in context.calls if t=='sim_button_observation'}=={arm}
    assert context.floor_reads==1


@pytest.mark.parametrize('arm',[None,True,'either','RIGHT',[],{}])
def test_invalid_cabin_arm_refuses_before_motion(arm):
    context=CabinContext()
    with pytest.raises(ValueError,match='cabin_press_arm'):
        ride(context,cabin_press_arm=arm)
    assert not any(t in ('evo_move_joints','evo_follow_path','rax_move_base') for t,_ in context.calls)


@pytest.mark.parametrize('arm',['left','right'])
def test_explicit_single_profile_runs_same_ride_and_stows_full_robot(arm):
    context, original = ride(cabin_press_arm=arm)
    selected = original.task.profiles[arm]
    task = CabinRide(SimulationElevator(context, replace(original.task.settings, arms=(selected,))),
                     original.settings)
    result = task.run('2')
    assert result.outcome == 'succeeded'
    assert result.measured['stages']['button']['measured']['arm'] == arm
    assert {a['side'] for t,a in context.calls if t=='sim_button_observation'} == {arm}
    stows = [a['targets'] for t,a in context.calls if t=='evo_move_joints'
             and a['targets']==context.scene_cfg['travel_joints']]
    assert stows and context.joints == context.scene_cfg['travel_joints']
    assert any(n.startswith('left_joint') for n in stows[-1])
    assert any(n.startswith('right_joint') for n in stows[-1])
    other = 'left' if arm=='right' else 'right'
    assert all(other not in waypoint for tool,args in context.calls if tool=='evo_follow_path'
               for waypoint in args['waypoints'])


@pytest.mark.parametrize('arm',['left','right'])
def test_explicit_missing_profile_refuses_before_motion(arm):
    context, original = ride(cabin_press_arm=arm)
    other = 'left' if arm=='right' else 'right'
    task = SimulationElevator(context, replace(original.task.settings, arms=(original.task.profiles[other],)))
    with pytest.raises(ValueError,match='no calibrated profile'):
        CabinRide(task, original.settings)
    assert not any(t in ('evo_move_joints','evo_follow_path','rax_move_base') for t,_ in context.calls)


@pytest.mark.parametrize("button_stage", ["hall_call", "button"])
@pytest.mark.parametrize("clock_failure", [False, True])
def test_failed_full_button_keeps_timing_and_original_refusal(button_stage, clock_failure):
    context, task = ride(full_button_timeout_s=20.)
    if button_stage == "hall_call":
        context.door_state = "closed"
    refusal = RuntimeError("contact geometry refused")
    def failed_action(*args):
        context.simulation_clock.t += 8.25
        if clock_failure:
            def failed_clock():
                raise RuntimeError("clock unavailable")
            context.simulation_clock.monotonic = failed_clock
        raise refusal
    if button_stage == "hall_call":
        task.call_elevator = failed_action
    else:
        task.press_cabin_button = failed_action
    events = []
    with pytest.raises(RuntimeError) as error:
        task.run("2", on_stage=events.append)
    assert error.value is refusal
    receipts = [e for e in task.evidence if e.get("check") == "full_button_deadline"]
    assert len(receipts) == 1
    receipt = receipts[0]
    assert receipt["stage"] == button_stage and receipt["passed"] is False
    assert receipt["error"] == repr(refusal)
    if clock_failure:
        assert "clock unavailable" in receipt["clock_error"]
    else:
        assert receipt["duration_s"] == pytest.approx(8.25)
        assert receipt["end_s"] - receipt["start_s"] == pytest.approx(8.25)
    assert events[-1]["event"] == "failed"
    assert not any(e["stage"] == ("entry" if button_stage == "hall_call" else "destination") for e in events)
