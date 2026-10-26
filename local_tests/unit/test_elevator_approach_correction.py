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
    def path(side,target):t.paths.append(target);point[:]=target
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
        def refused(*args):raise PrimitiveError('original bound refused')
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
