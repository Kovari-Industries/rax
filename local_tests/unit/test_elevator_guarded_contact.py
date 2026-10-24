"""CPU-only guarded contact contracts; no optical or physical qualification."""
from copy import deepcopy
from dataclasses import replace
import time

import pytest

from auto_research_agent.api.base import PrimitiveError
from auto_research_agent.api.elevator import Elevator, FinalContactPolicy
from auto_research_agent.api.evo import Evo, EvoActionFailed
from test_elevator import RobotContext, settings


POLICY = FinalContactPolicy(.27, .32, 'synthetic-occlusion-proof')


class GuardContext(RobotContext):
    def __init__(self):
        super().__init__()
        self.capabilities['scene']['guarded_contact'] = True
        self.capabilities['scene']['guarded_contact_profiles'] = [
            dict(panel_id='test-panel', camera=side+'_arm', occlusion_qualification_id=POLICY.occlusion_qualification_id)
            for side in ('left', 'right')]
        self.capabilities['tools'].append({'name': 'evo_guarded_contact'})
        self.guards = []
        self.guard_change = None
        self.preview_change = None
        self.after_guard_change = None
        self.identity = True
        self.face_width = 1.
        self.panel_id = 'test-panel'
        self.guard_refused = False
        for palm in self.palms.values():
            palm['position'][0] = .3

    def scene(self, stamp=None):
        if stamp is None:
            time.sleep(0)
        result = super().scene(stamp)
        result['panel'].update(id=self.panel_id, uncertainty_m=.001,
            face=dict(center=self.local((1.,0.,1.)), horizontal_axis=[0.,1.,0.],
                      vertical_axis=[0.,0.,1.], width_m=self.face_width, height_m=1.,
                      source='synthetic finite panel fixture'))
        return result

    def value(self, tool, **args):
        if tool == 'evo_preview' and args['action']['kind'] == 'guarded_contact':
            self.calls.append((tool, deepcopy(args)))
            if self.preview_change:
                self.preview_change(args['action'])
            return {'valid': not self.guard_refused}
        if tool == 'evo_guarded_contact':
            self.calls.append((tool, deepcopy(args)))
            self.guards.append(deepcopy(args))
            observation = args['observation']
            side = args['side']
            self.palms[side] = dict(position=[p-n*args['retract_gap_m'] for p,n in zip(
                observation['position'],observation['normal'])],
                quaternion_xyzw=list(args['press_quaternion_xyzw']))
            receipt = dict(observation_id=observation['observation_id'],capture_us=observation['capture_us'],
                side=side,target=observation['label'],prepress_stop_verified=True,press_stop_verified=True,
                withdrawal_stop_verified=True,source_limits_verified=True,fixed_geometry_verified=True,
                single_correct_press=True,released=True)
            self.last_result = dict(action_id='guarded-1',state='succeeded',physical_stop_verified=True,
                                    guarded_contact=receipt)
            if self.guard_change:
                self.guard_change(self.last_result)
            if self.after_guard_change:
                self.after_guard_change()
            return {'action_id':'guarded-1'}
        result = super().value(tool, **args)
        if tool == 'rax_detect_buttons' and self.identity:
            result['model_receipt'] = '/synthetic/'+result['observation_id']+'.json'
            result['cameras'][args['camera']]['image_sha256'] = 'a'*64
        if tool == 'rax_camera_ray':
            result['surface']['face'] = deepcopy(self.capture['scene']['panel']['face'])
        return result


def task_and_selection(**changes):
    context = GuardContext()
    task = Elevator(context, settings(final_contact=POLICY, **changes))
    selection = task.select_press_arm(task.search_button('2'))
    assert selection.side is not None
    return context, task, selection


@pytest.mark.parametrize("side", ["left", "right"])
def test_guarded_opt_in_uses_one_transaction_and_fresh_postwithdraw_ocr(side):
    context = GuardContext()
    task = Elevator(context, settings(final_contact=POLICY))
    selection = task.select_press_arm(task.search_button("2", side), side)
    assert selection.side == side
    result = task.guarded_press_button(selection)
    assert result.outcome == 'succeeded'
    assert len(context.guards) == 1
    sent = context.guards[0]
    assert sent['position_tolerance_m'] == .004 and sent['angle_tolerance_rad'] == .01
    assert sent['stroke_m'] == task.settings.stroke_m and sent['dwell_s'] == task.settings.dwell_s
    assert sent['approach_step_m'] == task.settings.approach_step_m
    assert sent['retract_gap_m'] == POLICY.visible_gap_m
    assert sent['observation']['image_sha256'] == 'a'*64
    record = next(v for v in task.evidence if v.get('stage') == 'guarded_contact')
    assert record['state'] == 'verified'
    assert record['postwithdraw_capture_us'] > sent['observation']['capture_us']
    assert context.guards[0] == record['request']
    assert task._contact is None and task._guarded_attempted
    with pytest.raises(PrimitiveError, match='cannot repeat'):
        task.guarded_press_button(selection)
    assert len(context.guards) == 1


@pytest.mark.parametrize('change', ['missing_capability', 'missing_tool', 'invalid_policy', 'too_far', 'close_gap'])
def test_guarded_mode_requires_explicit_admission_and_bounded_policy(change):
    context = GuardContext()
    policy = POLICY
    if change == 'missing_capability':
        context.capabilities['scene']['guarded_contact'] = False
    elif change == 'missing_tool':
        context.capabilities['tools'] = [{'name':'evo_follow_path'}]
    elif change == 'invalid_policy':
        policy = {'visible_gap_m':.27}
    elif change == 'too_far':
        policy = replace(POLICY,max_inward_travel_m=1.)
    else:
        policy = replace(POLICY,visible_gap_m=.05)
    with pytest.raises((ValueError,PrimitiveError)):
        Elevator(context,settings(final_contact=policy))
    assert not context.guards


@pytest.mark.parametrize('fault', ['identity', 'finite_region', 'too_close'])
def test_unqualified_visible_target_refuses_before_guarded_motion(fault):
    context, task, selection = task_and_selection()
    if fault == 'identity':
        context.identity = False
    elif fault == 'finite_region':
        context.face_width = .001
    else:
        context.palms[selection.side]['position'][0] = .75
    with pytest.raises(PrimitiveError):
        task.guarded_press_button(selection)
    assert not context.guards


@pytest.mark.parametrize('fault', ['refused', 'expired', 'base', 'carriage', 'panel', 'geometry', 'cancelled'])
def test_final_preview_failure_or_changed_state_cannot_dispatch(monkeypatch,fault):
    context, task, selection = task_and_selection()
    real_time, real_ns = time.time, time.time_ns
    offset = [0.]
    monkeypatch.setattr(time,'time',lambda:real_time()+offset[0])
    monkeypatch.setattr(time,'time_ns',lambda:real_ns()+round(offset[0]*1e9))
    cancelled = [False]
    task.base.cancelled = lambda:cancelled[0]
    def change(action):
        if fault == 'refused':
            context.guard_refused = True
        elif fault == 'expired':
            offset[0] += 2.
        elif fault == 'base':
            context.pose[0] += .005
        elif fault == 'carriage':
            context.joints['carriage_joint'] += .005
        elif fault == 'panel':
            context.panel_id = 'another-panel'
        elif fault == 'geometry':
            context.face_width = .8
        else:
            cancelled[0] = True
    context.preview_change = change
    with pytest.raises((PrimitiveError,ValueError)):
        task.guarded_press_button(selection)
    assert not context.guards
    assert task._contact is not None
    with pytest.raises(PrimitiveError):
        task.guarded_press_button(selection)


@pytest.mark.parametrize('field', ['prepress_stop_verified','press_stop_verified','withdrawal_stop_verified',
                                  'source_limits_verified','fixed_geometry_verified','single_correct_press','released',
                                  'observation_id','capture_us','side','target'])
def test_incomplete_or_wrong_guarded_result_never_clears_contact_or_retries(field):
    context, task, selection = task_and_selection()
    context.guard_change = lambda result:result['guarded_contact'].pop(field)
    with pytest.raises(PrimitiveError,match='matching measured completion'):
        task.guarded_press_button(selection)
    assert task._contact['stage'] == 'guarded_attempted'
    assert task._guarded_attempted
    for function in (lambda:task.guarded_press_button(selection), task.stow_arms,
                     lambda:task.retract_arm(selection.side), lambda:task.press_button('2',selection.side)):
        with pytest.raises(PrimitiveError):
            function()
    assert len(context.guards) == 1


@pytest.mark.parametrize('fault', ['action_failed','unknown_dispatch','endpoint','orientation','base','carriage',
                                  'postwithdraw_missing','postwithdraw_ambiguous'])
def test_uncertain_action_or_postwithdraw_failure_stays_latched(fault):
    context, task, selection = task_and_selection()
    if fault == 'action_failed':
        context.guard_change = lambda result:result.update(state='failed',physical_stop_verified=False)
    elif fault == 'unknown_dispatch':
        def raise_unknown(result):
            raise RuntimeError('unknown dispatch state')
        context.guard_change = raise_unknown
    else:
        def change():
            if fault == 'endpoint':
                context.palms[selection.side]['position'][0] += .005
            elif fault == 'orientation':
                context.palms[selection.side]['quaternion_xyzw'] = [0.,0.,.01,(1-.01**2)**.5]
            elif fault == 'base':
                context.pose[0] += .005
            elif fault == 'carriage':
                context.joints['carriage_joint'] += .005
            elif fault == 'postwithdraw_missing':
                context.present[selection.side] = False
            else:
                context.duplicate_label = True
        context.after_guard_change = change
    with pytest.raises((PrimitiveError,EvoActionFailed,RuntimeError)):
        task.guarded_press_button(selection)
    assert task._contact['stage'] == 'guarded_attempted'
    assert task._guarded_attempted and len(context.guards) == 1


def test_disabled_default_uses_existing_contact_path():
    context = GuardContext()
    task = Elevator(context, settings())
    task.press_elevator_button('2')
    assert not context.guards


def test_coarse_lateral_staging_is_once_before_first_image_and_uses_compact_arms():
    context = GuardContext()
    task = Elevator(context,settings(coarse_panel_lateral_offset_m=.2))
    task.press_elevator_button('2')
    first_image = next(i for i,(name,_) in enumerate(context.calls) if name=='rax_detect_buttons')
    lateral = [(i,args) for i,(name,args) in enumerate(context.calls)
               if name=='rax_move_base' and abs(args['delta'][1])>.001]
    assert len(lateral)==1 and lateral[0][0]<first_image
    assert lateral[0][1]['delta'][1] == pytest.approx(-.2)
    assert len([e for e in task.evidence if e.get('stage')=='coarse_panel_staging']) == 1


@pytest.mark.parametrize('fault',['nonfinite','overstep','not_compact','wrong_panel'])
def test_coarse_staging_rejects_invalid_or_unsafe_requests(fault):
    context = GuardContext()
    if fault=='nonfinite':
        with pytest.raises(ValueError):
            Elevator(context,settings(coarse_panel_lateral_offset_m=float('nan')))
        return
    task = Elevator(context,settings(coarse_panel_lateral_offset_m=3. if fault=='overstep' else .2))
    if fault=='not_compact':
        context.joints['left_joint4']=1.
    elif fault=='wrong_panel':
        context.panel_id='other-panel'
    with pytest.raises(PrimitiveError):
        task._coarse_panel_stage()
    assert not context.base_calls


def test_generic_evo_callback_runs_after_preview_and_final_cancel_still_blocks_dispatch():
    context = GuardContext()
    cancelled = [False]
    called = []
    def callback():
        assert context.calls[-1][0]=='evo_preview'
        called.append(True)
        cancelled[0]=True
    with pytest.raises(EvoActionFailed,match='cancelled'):
        Evo(context).execute('evo_move_joints',targets={},cancelled=lambda:cancelled[0],before_dispatch=callback)
    assert called and not any(t=='evo_move_joints' for t,_ in context.calls)


def test_full_guarded_press_routes_to_transaction_then_stows():
    context = GuardContext()
    task = Elevator(context,settings(final_contact=POLICY))
    task.prepare_press_arm = lambda *args:pytest.fail('Legacy inward preparation must not run')
    result = task.press_elevator_button('2')
    assert result.outcome == 'succeeded' and result.measured['activation'] == 'succeeded'
    assert len(context.guards) == 1 and task._contact is None
    assert context.joints == context.scene_cfg['travel_joints']
    guard_index = next(i for i,(name,_) in enumerate(context.calls) if name=='evo_guarded_contact')
    afterward = [name for name,_ in context.calls[guard_index+1:]]
    assert afterward.index('rax_detect_buttons') < afterward.index('evo_move_joints')


def test_cabin_contact_composition_selects_guarded_route_and_stows():
    # Composition only: actual guarded source/FK/freshness behavior is tested
    # above with the image/guard context. This fake isolates the cabin branch.
    from test_elevator_cabin import ride
    from auto_research_agent.api.base import PrimitiveResult
    context, cabin = ride()
    cabin.task.settings = replace(cabin.task.settings,final_contact=POLICY)
    context.capabilities['scene'].update(guarded_contact=True,guarded_contact_profiles=[
        dict(panel_id='test-panel',camera=side+'_arm',occlusion_qualification_id=POLICY.occlusion_qualification_id)
        for side in ('left','right')])
    context.capabilities['tools'].append({'name':'evo_guarded_contact'})
    called = []
    def guarded(selection):
        called.append(selection.side)
        context.floor = '2'
        return PrimitiveResult('guarded_press_button','succeeded',measured={
            'activation':'succeeded','guarded_contact':{'synthetic_composition_only':True}})
    cabin.task.guarded_press_button = guarded
    cabin.task.prepare_press_arm = lambda *args:pytest.fail('Legacy inward preparation must not run')
    result = cabin.run('2')
    assert result.outcome == 'succeeded' and len(called) == 1
    assert result.measured['stages']['button']['measured']['guarded_contact']['synthetic_composition_only']


def test_coarse_stage_does_not_infer_a_label_or_request_images():
    context = GuardContext()
    task = Elevator(context,settings(coarse_panel_lateral_offset_m=.2))
    task._coarse_panel_stage()
    assert not any(name in ('rax_detect_buttons','rax_camera_ray') for name,_ in context.calls)
    assert task._last_stop_us > 0


def test_failed_guard_preview_does_not_reserve_a_physical_attempt():
    context, task, selection = task_and_selection()
    context.guard_refused = True
    with pytest.raises(ValueError,match='preview'):
        task.guarded_press_button(selection)
    assert not task._guarded_attempted
    assert task._contact['stage'] == 'guarded_prepared'
    assert not context.guards


@pytest.mark.parametrize('change',['carriage','carriage_nonfinite','tip','geometry'])
def test_state_drift_during_postwithdraw_inference_cannot_clear_contact(change):
    context, task, selection = task_and_selection()
    original = context.value
    def drifting(tool, **args):
        result = original(tool, **args)
        if tool == 'rax_detect_buttons' and context.guards:
            if change == 'carriage':
                context.joints['carriage_joint'] += .005
            elif change == 'carriage_nonfinite':
                context.joints['carriage_joint'] = float('nan')
            elif change == 'tip':
                context.palms[selection.side]['position'][0] += .005
            else:
                context.face_width = .8
        return result
    context.value = drifting
    with pytest.raises((PrimitiveError,ValueError)):
        task.guarded_press_button(selection)
    assert task._contact['stage'] == 'guarded_attempted'
    assert len(context.guards) == 1


def test_uncertain_visible_preparation_also_blocks_retry_stow_and_generic_recovery():
    context, task, selection = task_and_selection()
    context.fail_action = 'evo_follow_path'
    with pytest.raises(EvoActionFailed):
        task.guarded_press_button(selection)
    assert task._contact['stage'] == 'guarded_preparation_failed'
    assert not task._guarded_attempted and not context.guards
    before = len(context.calls)
    for function in (lambda:task.guarded_press_button(selection), task.stow_arms,
                     lambda:task.retract_arm(selection.side)):
        with pytest.raises(PrimitiveError):
            function()
    assert len(context.calls) == before
    assert next(e for e in task.evidence if e.get('stage')=='guarded_contact')['state']=='failed'


@pytest.mark.parametrize('identity', [{}, {'other':'id'}, {'left':''}, {'right':None}, {'left':7}])
def test_guarded_policy_mapping_requires_explicit_nonempty_arm_ids(identity):
    with pytest.raises(ValueError,match='mapping'):
        Elevator(GuardContext(),settings(final_contact=replace(POLICY,occlusion_qualification_id=identity)))


@pytest.mark.parametrize('change',['absent','empty','malformed','wrong_panel','wrong_id','duplicate'])
def test_guarded_mode_requires_exact_advertised_records_before_motion(change):
    context=GuardContext()
    scene=context.capabilities['scene']
    records=scene['guarded_contact_profiles']
    if change=='absent': del scene['guarded_contact_profiles']
    elif change=='empty': scene['guarded_contact_profiles']=[]
    elif change=='malformed': records[0]['camera']='unrecognized_camera'
    elif change=='duplicate': records.append(deepcopy(records[0]))
    else:
        for row in records:
            row['panel_id' if change=='wrong_panel' else 'occlusion_qualification_id']='different'
    with pytest.raises(PrimitiveError):
        Elevator(context,settings(final_contact=POLICY))
    assert not any(name.startswith('evo_move') or name=='evo_follow_path' for name,_ in context.calls)


@pytest.mark.parametrize('eligible_side',['left','right'])
def test_auto_search_and_selection_exclude_unqualified_arm_before_any_view_motion(eligible_side):
    context=GuardContext()
    context.capabilities['scene']['guarded_contact_profiles']=[row for row in
        context.capabilities['scene']['guarded_contact_profiles'] if row['camera']==eligible_side+'_arm']
    task=Elevator(context,settings(final_contact=POLICY))
    observations=task.search_button('2')
    selection=task.select_press_arm(observations)
    assert selection.side==eligible_side
    other='left' if eligible_side=='right' else 'right'
    assert any(c['side']==other and not c['feasible'] and 'qualification' in c['reason'] for c in selection.candidates)
    assert all(not n.startswith(other+'_joint') for tool,args in context.calls
               if tool=='evo_move_joints' for n in args['targets'])
    assert all(args['camera']==eligible_side+'_arm' for tool,args in context.calls if tool=='rax_detect_buttons')


def test_explicit_missing_side_has_no_mapping_fallback_or_task_motion():
    context=GuardContext()
    policy=replace(POLICY,occlusion_qualification_id={'right':POLICY.occlusion_qualification_id})
    task=Elevator(context,settings(final_contact=policy))
    with pytest.raises(PrimitiveError,match='qualification'):
        task.press_elevator_button('2','left')
    assert not any(name.startswith('evo_move') or name in ('evo_follow_path','rax_move_base') for name,_ in context.calls)


@pytest.mark.parametrize('side',['left','right'])
def test_explicit_arm_id_mapping_selects_the_exact_advertised_transaction(side):
    context=GuardContext()
    identities={'left':'left-proof','right':'right-proof'}
    for row in context.capabilities['scene']['guarded_contact_profiles']:
        row['occlusion_qualification_id']=identities[row['camera'].removesuffix('_arm')]
    task=Elevator(context,settings(final_contact=replace(POLICY,occlusion_qualification_id=identities)))
    selection=task.select_press_arm(task.search_button('2',side),side)
    assert task.guarded_press_button(selection).outcome=='succeeded'
    assert context.guards[0]['occlusion_qualification_id']==identities[side]


def test_changed_advertisement_after_selection_refuses_before_visible_preparation():
    context,task,selection=task_and_selection()
    records=context.capabilities['scene']['guarded_contact_profiles']
    records[:]=[r for r in records if r['camera']!=selection.side+'_arm']
    before=len(context.calls)
    with pytest.raises(PrimitiveError,match='qualification'):
        task.guarded_press_button(selection)
    assert all(tool=='evo_capabilities' for tool,_ in context.calls[before:])
    assert not task._guarded_sequence_started and not context.guards


@pytest.mark.parametrize('preview_kind',['follow_path','guarded_contact'])
def test_changed_advertisement_after_preview_cannot_dispatch_that_motion(preview_kind):
    context,task,selection=task_and_selection()
    original=context.value
    invalidated=[False]
    def changing(tool,**args):
        result=original(tool,**args)
        if tool=='evo_preview' and args['action']['kind']==preview_kind and not invalidated[0]:
            invalidated[0]=True
            rows=context.capabilities['scene']['guarded_contact_profiles']
            rows[:]=[r for r in rows if r['camera']!=selection.side+'_arm']
        return result
    context.value=changing
    before=len(context.calls)
    with pytest.raises(PrimitiveError,match='qualification'):
        task.guarded_press_button(selection)
    assert invalidated[0] and not context.guards
    assert not any(tool=='evo_'+preview_kind for tool,_ in context.calls[before:])
    assert task._contact is not None and task._guarded_sequence_started


def test_shared_context_keeps_hall_and_interior_eligibility_separate():
    context=GuardContext()
    context.capabilities['scene']['guarded_contact_profiles']=[
        dict(panel_id='hall-call',camera='right_arm',occlusion_qualification_id='hall-right-proof'),
        dict(panel_id='cabin-panel',camera='left_arm',occlusion_qualification_id='cabin-left-proof')]
    hall=Elevator(context,settings(panel_id='hall-call',final_contact=replace(POLICY,
        occlusion_qualification_id={'right':'hall-right-proof'})))
    cabin=Elevator(context,settings(panel_id='cabin-panel',final_contact=replace(POLICY,
        occlusion_qualification_id={'left':'cabin-left-proof'})))
    assert hall._guarded_require()=={'right':'hall-right-proof'}
    assert cabin._guarded_require()=={'left':'cabin-left-proof'}
    with pytest.raises(PrimitiveError,match='qualification'):
        hall._guarded_require(('left',))
    with pytest.raises(PrimitiveError,match='qualification'):
        cabin._guarded_require(('right',))


@pytest.mark.parametrize('synchronized',[False,True])
def test_changed_view_advertisement_after_preview_refuses_before_initial_preparation(synchronized):
    context=GuardContext()
    task=Elevator(context,settings(final_contact=POLICY,synchronized_view=synchronized))
    original=context.value
    def changing(tool,**args):
        result=original(tool,**args)
        if tool=='evo_preview':
            rows=context.capabilities['scene']['guarded_contact_profiles']
            rows[:]=[r for r in rows if r['camera']=='right_arm']
        return result
    context.value=changing
    with pytest.raises(PrimitiveError,match='qualification'):
        task.search_button('2','left')
    assert not any(tool.startswith('evo_move') for tool,_ in context.calls)
    assert not any(tool=='rax_detect_buttons' for tool,_ in context.calls)
