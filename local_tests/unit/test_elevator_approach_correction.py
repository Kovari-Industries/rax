"""Bounded correction during approach; no carriage or contact retry permission."""
from types import SimpleNamespace as NS
import pytest
from auto_research_agent.api.elevator import Elevator
from auto_research_agent.api.base import PrimitiveError
from test_elevator import settings


def fixture(**changes):
    t=object.__new__(Elevator);t.settings=settings(tangent_approach_correction=True)
    t.evidence=[];t.base=NS(settings={'angle_tolerance_rad':.02})
    t._contact=dict(side='right',label='2',stage='approaching',normal=(1.,0.,0.),pose=(0.,0.,0.),carriage=.2)
    t._contact.update(changes);point=[0.,0.,0.];t.paths=[]
    t._contact_state=lambda side:t._contact
    t._fresh_button=lambda b:None
    t._tip=lambda side:tuple(point)
    t._preparation_tip=lambda b,s:tuple(point)
    def path(side,target,**kwargs):t.paths.append(target);point[:]=target
    t._path=path
    button=NS(label='2',position=(.15,.006,.003),normal=(1.,0.,0.),scene_pose=(0.,0.,0.))
    return t,button


def test_tangent_correction_holds_gap_and_carriage_and_keeps_recovery_state():
    t,b=fixture();distance=t._correct_approach_aim(b,'right',.1)
    assert t.paths==[(0.,.006,.003)]
    assert distance==pytest.approx((.006**2+.003**2)**.5)
    assert t._contact['stage']=='approaching' and t._contact['carriage']==.2
    assert t.evidence[-1]['gap_m']==.15


@pytest.mark.parametrize('stage',['prepress','press_attempted','unknown'])
def test_correction_cannot_retry_or_follow_contact(stage):
    t,b=fixture(stage=stage)
    with pytest.raises(PrimitiveError):t._correct_approach_aim(b,'right',.1)
    assert not t.paths


def test_correction_bounds_and_failures_keep_original_recovery_reference():
    for change in ('normal','label','step','budget','disabled','state','sweep','path'):
        t,b=fixture();original=t._contact;budget=.1
        if change=='normal':b.normal=(0.,1.,0.)
        if change=='label':b.label='3'
        if change=='step':b.position=(.15,.021,0.)
        if change=='budget':budget=.001
        if change=='disabled':t.settings=settings()
        def refused(*args,**kwargs):raise PrimitiveError('original bound refused')
        if change=='state':t._contact_state=refused
        if change=='sweep':t._preparation_tip=refused
        if change=='path':t._path=refused
        with pytest.raises(PrimitiveError):t._correct_approach_aim(b,'right',budget)
        assert t._contact is original and not t.paths


def test_approach_uses_arm_only_correction_then_original_inward_steps():
    t,b=fixture();t._selected='right';b.uncertainty_m=.0001
    t._look=lambda *args:b;t._state=lambda:{'positions':{'carriage_joint':.2}}
    t.approach_button('2','right')
    assert t.paths[0]==(0.,.006,.003)
    assert all(0<q[0]-p[0]<=.020000001 for p,q in zip(t.paths,t.paths[1:]))
    assert t._contact['stage']=='prepress'
    assert b.position[0]-t._tip('right')[0]>=t.settings.prepress_gap_m


@pytest.mark.parametrize('inward_error',[0.,.000213,.002])
def test_final_approach_aims_inside_existing_gap_band(inward_error):
    t,b=fixture();t._selected='right';b.uncertainty_m=.0001
    b.position=(.0601,0.,0.)
    t._look=lambda *args:b;t._state=lambda:{'positions':{'carriage_joint':.2}}
    original=t._path
    def measured_path(side,target,**kwargs):
        original(side,(target[0]+inward_error,target[1],target[2]))
    t._path=measured_path
    t.approach_button('2','right')
    gap=b.position[0]-t._tip('right')[0]
    assert gap==pytest.approx(t.settings.prepress_gap_m+b.uncertainty_m+
                              .5*t.settings.position_tolerance_m-inward_error)
    assert gap>=t.settings.prepress_gap_m-b.uncertainty_m
    assert t._contact['stage']=='prepress'


def test_original_too_close_gap_still_refuses_without_motion():
    t,b=fixture();t._selected='right';b.uncertainty_m=.0001
    b.position=(.039887161025153044,0.,0.)
    t._look=lambda *args:b
    with pytest.raises(PrimitiveError,match='closer to the panel'):
        t.approach_button('2','right')
    assert not t.paths


@pytest.mark.parametrize('duration',[None,.1])
def test_optional_approach_duration_only_changes_approach_requests(duration):
    t,b=fixture();t.settings=settings(tangent_approach_correction=True,approach_segment_s=duration)
    t._selected='right';b.uncertainty_m=.0001
    t._look=lambda *args:b;t._state=lambda:{'positions':{'carriage_joint':.2}}
    original=t._path;durations=[]
    def path(side,target,**kwargs):
        durations.append(kwargs.get('duration_s'));original(side,target)
    t._path=path;t.approach_button('2','right')
    assert len(durations)>1 and all(x==duration for x in durations)
    assert t.settings.segment_s==settings().segment_s
    assert t.settings.dwell_s==settings().dwell_s


@pytest.mark.parametrize('duration',[.04,.099,0.,-1.,float('nan'),float('inf'),11.])
def test_invalid_approach_duration_is_refused(duration):
    with pytest.raises(ValueError):settings(approach_segment_s=duration).validate()


@pytest.mark.parametrize('field',['segment_s','approach_segment_s','retract_segment_s'])
def test_cartesian_duration_configuration_matches_shared_evo_minimum(field):
    from services.evo.contracts import validate_request
    with pytest.raises(ValueError):settings(**{field:.04}).validate()
    settings(**{field:.1}).validate()
    args=dict(frame='base_link',waypoints=[dict(duration_s=.1,right=dict(position=[.4,0.,1.],quaternion_xyzw=[0.,0.,0.,1.]))])
    validate_request('evo_follow_path',args)
    args['waypoints'][0]['duration_s']=.04
    with pytest.raises(ValueError):validate_request('evo_follow_path',args)
