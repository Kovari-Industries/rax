"""Synthetic contract tests, not hardware or GPU handoff qualification."""
import time
from copy import deepcopy
import math
import pytest
from test_elevator_cabin import CabinContext, ride
from auto_research_agent.api.elevator_handoff import CabinEntryHandoff, check_handoff_status
from auto_research_agent.api.base import PrimitiveError


def status(positions=None):
    t=time.time_ns()//1000;positions=positions or {'joint':0.}
    return dict(owner='receiver',armed=True,active_action=None,feedback_fresh=True,motion_blocked=False,
        last_action=dict(action_id='entry-final',state='succeeded',physical_stop_verified=True),
        state=dict(source_us=t,receipt_us=t,odometry_source_us=t,odometry_receipt_us=t,
            fault=None,positions=positions,velocities={n:0. for n in positions},odometry=dict(velocity=[0.,0.,0.])))


def test_native_shaped_status_is_admitted():
    row=check_handoff_status(status(),CabinEntryHandoff('receiver','entry-final',(2.,0.,math.pi)),
        {'joint':0.},{'joint':{'tolerance':.01}})
    assert row['action_id']=='entry-final'


@pytest.mark.parametrize('path,value',[
    (('owner',),'teleop'),(('armed',),False),(('active_action',),{'action_id':'moving'}),
    (('feedback_fresh',),False),(('motion_blocked',),True),
    (('last_action','action_id'),'old'),(('last_action','physical_stop_verified'),False),
    (('last_action','state'),'failed'),(('state','fault'),'drive fault'),
    (('state','velocities','joint'),.005),(('state','velocities','joint'),float('nan')),
    (('state','positions','joint'),.011),(('state','source_us'),0),
    (('state','odometry_source_us'),0),(('state','odometry','velocity'),[0.,.005,0.]),
    (('state','odometry','velocity'),[]),
])
def test_bad_handoff_is_refused(path,value):
    row=status();item=row
    for key in path[:-1]:item=item[key]
    item[path[-1]]=value
    with pytest.raises((PrimitiveError,ValueError)):
        check_handoff_status(row,CabinEntryHandoff('receiver','entry-final',(2.,0.,math.pi)),
            {'joint':0.},{'joint':{'tolerance':.01}})


@pytest.mark.parametrize('key',['owner','armed','active_action','last_action','state'])
def test_missing_fields_do_not_get_safe_defaults(key):
    row=status();row.pop(key)
    with pytest.raises(PrimitiveError):
        check_handoff_status(row,CabinEntryHandoff('receiver','entry-final',(2.,0.,math.pi)),
            {'joint':0.},{'joint':{'tolerance':.01}})


class HandoffContext(CabinContext):
    def value(self,tool,**args):
        if tool=='evo_status':
            self.calls.append((tool,args))
            return status(self.joints.copy())
        return super().value(tool,**args)


def test_handoff_keeps_origin_button_destination_exit_without_entry_turn():
    context,task=ride(HandoffContext());events=[]
    def operator():
        assert task._baseline is not None
        context.pose[:]=[2.,0.,math.pi]
        return CabinEntryHandoff('receiver','entry-final',(2.,0.,math.pi))
    result=task.run('2',entry_handoff=operator,on_stage=events.append)
    assert result.outcome=='succeeded'
    assert [e['stage'] for e in events if e['event']=='finished']==['origin','entry_handoff','button','destination','exit']
    assert not any(a['delta'][2] for t,a in context.calls if t=='rax_move_base')
    admission=next(e for e in task.evidence if e.get('stage')=='teleoperated_entry_admission')
    assert admission['passed'] and len(admission['samples'])==10


def test_handoff_wrong_pose_never_starts_button():
    context,task=ride(HandoffContext());events=[]
    def operator():
        context.pose[:]=[2.,0.,math.pi]
        return CabinEntryHandoff('receiver','entry-final',(2.1,0.,math.pi))
    with pytest.raises(PrimitiveError,match='configured pose'):
        task.run('2',entry_handoff=operator,on_stage=events.append)
    assert not any(e['stage']=='button' for e in events)
    assert not any(t=='sim_button_observation' for t,a in context.calls)
