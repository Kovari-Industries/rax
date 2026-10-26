"""Hall prefix admission. No simulator or robot connection."""
from types import SimpleNamespace as NS
import math
import pytest
from auto_research_agent.api.elevator import Elevator
from auto_research_agent.api.base import PrimitiveError,PrimitiveResult
from test_elevator import settings,RobotContext


def task(duration=19.,activation='succeeded',failure=None):
    t=object.__new__(Elevator);clock=NS(now=0.);clock.monotonic=lambda:clock.now
    t.base=NS(clock=clock,evidence=[],settings=dict(footprint=[.38,.34,1.8],margin_m=.03));t.evidence=[];t.entries=[]
    def press(*args):
        clock.now=duration
        if failure:raise failure
        return PrimitiveResult('press_elevator_button','succeeded',(),{'activation':activation})
    def entry(*args,**kwargs):
        t.entries.append((args,kwargs));return PrimitiveResult('navigate_into_elevator','succeeded',(),{'location':'inside'})
    t.press_elevator_button=press;t.navigate_into_elevator=entry
    return t


def test_checked_button_precedes_normal_entry():
    t=task();r=t.press_and_enter('CALL',doorway_id='door')
    assert r.outcome=='succeeded' and r.measured['button_timing']['duration_s']==19.
    assert t.entries==[(('door',),{'goal_depth_m':.9})]


@pytest.mark.parametrize('duration',[20.,20.1,-1.,float('nan'),float('inf')])
def test_late_or_invalid_clock_cannot_enter(duration):
    t=task(duration)
    with pytest.raises(PrimitiveError,match='deadline|clock'):t.press_and_enter('CALL')
    assert not t.entries and t.evidence[-1]['passed'] is False


@pytest.mark.parametrize('activation,failure',[('unknown',None),('succeeded',RuntimeError('stop refused'))])
def test_unconfirmed_or_failed_button_never_enters(activation,failure):
    t=task(activation=activation,failure=failure)
    with pytest.raises((PrimitiveError,RuntimeError)):t.press_and_enter('CALL')
    assert not t.entries and 'error' in t.evidence[-1]


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),True,20.1])
def test_deadline_cannot_be_relaxed(value):
    t=task()
    with pytest.raises(ValueError):t.press_and_enter('CALL',button_timeout_s=value)
    assert not t.entries


def test_separate_retraction_duration_keeps_default_and_bounds():
    assert settings().retract_segment_s is None
    settings(retract_segment_s=1.2).validate()
    for value in (0.,-1.,float('nan'),11.):
        with pytest.raises(ValueError):settings(retract_segment_s=value).validate()


@pytest.mark.parametrize('duration',[None,1.2])
def test_retract_dispatch_uses_selected_duration_without_changing_clearance(duration):
    t=object.__new__(Elevator);t.settings=settings(retract_segment_s=duration)
    t.evidence=[];t._contact={};calls=[]
    t._contact_state=lambda side:dict(point=(.04,0,0),normal=(1,0,0))
    t._tip=lambda side:(0,0,0)
    t._path=lambda side,target,**kwargs:calls.append((side,target,kwargs))
    r=t.retract_arm('right')
    assert calls==[('right',(-.04,0.,0.),{'duration_s':duration})]
    assert r.measured['clearance_m']==t.settings.retract_gap_m
    assert t._contact is None


def test_invalid_entry_goal_refuses_before_press():
    t=task()
    with pytest.raises(ValueError,match='before pressing'):t.press_and_enter('CALL',goal_depth_m=.1)
    assert not t.evidence and t.base.clock.now==0.


def test_button_subtask_uses_same_full_deadline_without_entry():
    t=task();r=t.press_with_deadline('CALL')
    assert r.outcome=='succeeded' and r.measured['activation']=='succeeded'
    assert r.measured['button_timing']['duration_s']==19.
    assert not t.entries


@pytest.mark.parametrize('duration',[20.,float('nan'),-1.])
def test_button_subtask_keeps_failure_time_and_refuses(duration):
    t=task(duration)
    with pytest.raises(PrimitiveError):t.press_with_deadline('CALL')
    assert t.evidence[-1]['passed'] is False and 'duration_s' in t.evidence[-1]
    assert not t.entries


@pytest.mark.parametrize('fault', [None, 'stale_door', 'closing_door', 'failed_stow'])
def test_button_handoff_uses_real_entry_freshness_and_stop_gates(fault):
    """Only the completed button is stubbed; entry uses actual Rax primitives."""
    from auto_research_agent.api.evo import EvoActionFailed
    context = RobotContext()
    task = Elevator(context, settings(direct_doorway_staging=True))
    clock = NS(now=0.)
    clock.monotonic = lambda: clock.now
    def sleep(seconds):
        clock.now += seconds
    clock.sleep = sleep
    task.base.clock = clock
    def completed_button(*args):
        clock.now = 19.
        if fault == 'stale_door':
            context.stale = True
        elif fault == 'closing_door':
            context.door_state = 'closing'
        elif fault == 'failed_stow':
            context.fail_action = 'evo_move_joints'
        return PrimitiveResult('press_elevator_button', 'succeeded', (), {'activation': 'succeeded'})
    task.press_elevator_button = completed_button
    if fault is None:
        result = task.press_and_enter('2', doorway_id='door-1', goal_depth_m=.5)
        assert result.measured['entry']['location'] == 'inside'
        assert result.measured['button_timing']['duration_s'] == 19.
        assert context.pose[0] == pytest.approx(1.5)
        assert context.joints == context.scene_cfg['travel_joints']
        assert context.base_calls and all(x['doorway_id'] == 'door-1' for x in context.base_calls)
    else:
        expected = {'stale_door': 'stale', 'closing_door': 'timeout', 'failed_stow': 'injected failure'}[fault]
        with pytest.raises((PrimitiveError, EvoActionFailed), match=expected):
            task.press_and_enter('2', doorway_id='door-1', goal_depth_m=.5)
        assert not context.base_calls
        assert context.pose == [0., 0., 0.]
        assert next(x for x in task.evidence if x.get('stage') == 'hall_button_deadline')['passed']
