import json
from pathlib import Path
import socket
import struct
import threading
import time
from types import SimpleNamespace

import pytest
from services.evo_v3.backend import EvoV3Backend, read_frame


def backend(tmp_path, **options):
    return EvoV3Backend(SimpleNamespace(root=tmp_path, backend_options=options))


def write_frame(path, age=0, generation=2, payload=b'\xff\xd8test\xff\xd9'):
    header = bytearray(64); header[:4] = b'EVJ1'
    struct.pack_into('<Q', header, 8, generation)
    struct.pack_into('<III', header, 24, 1920, 1080, len(payload))
    struct.pack_into('<Q', header, 52, int((time.time()-age)*1e6))
    path.write_bytes(header+payload)


def test_constructor_idle_close_and_unknown_inputs(tmp_path):
    b = backend(tmp_path, control_socket=str(tmp_path/'absent'))
    b.close(); b.close()
    with pytest.raises(ValueError, match='unknown Evo options'):
        backend(tmp_path, gains=1)
    with pytest.raises(RuntimeError, match='not enabled'):
        backend(tmp_path).call('evo_towel_align', {})
    with pytest.raises(ValueError, match='unknown Evo tool'):
        backend(tmp_path).call('evo_status', {'speed':1})


@pytest.mark.parametrize('age,generation,payload,reason', [
    (2,2,b'\xff\xd8x\xff\xd9','stale'),
    (-2,2,b'\xff\xd8x\xff\xd9','future'),
    (0,3,b'\xff\xd8x\xff\xd9','stabilize'),
    (0,2,b'broken','payload')])
def test_invalid_frames(tmp_path, age, generation, payload, reason):
    p=tmp_path/'frame'; write_frame(p,age,generation,payload)
    with pytest.raises(ValueError, match=reason):read_frame(p,.3)


def test_partial_observation_is_explicit_and_saved(tmp_path):
    cameras=tmp_path/'cameras';cameras.mkdir()
    write_frame(cameras/'base.jpeg.shm')
    b=backend(tmp_path,camera_directory=str(cameras))
    value,images=b.call('evo_observe',{})
    assert not value['ok'] and set(value['errors'])=={'left_arm','right_arm'}
    assert value['cameras']['base']['camera_to_base'] is None
    assert len(images)==1
    meta=tmp_path/'agent-workspace/runtime/frames'/value['frame_id']/'meta.json'
    assert json.loads(meta.read_text())==value


def test_policy_socket_cancel_is_only_acknowledgement(tmp_path):
    path=tmp_path/'policy.sock';server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(str(path));server.listen();received=[]
    def serve():
        with server.accept()[0] as s:
            received.append(s.recv(1024));s.sendall(b'{"result":"cancel_requested","state":"Settling"}\n')
    thread=threading.Thread(target=serve);thread.start()
    value,_=backend(tmp_path,control_socket=str(path)).call('evo_hold',{})
    thread.join();server.close()
    assert received==[b'cancel\n'] and not value['physical_stop_verified']


def test_registry_and_rgb_only_setup(tmp_path):
    from services.backends import backend_class
    from services.workspace.setup import Setup
    assert backend_class('evo-v3') is EvoV3Backend
    setup=Setup(robot='evo-v3',observation_profile='rgb-only',gemini=False)
    assert 'evo_observe' in setup.tools and 'rby1_move_eef' not in setup.tools
    with pytest.raises(ValueError,match='RGB-only'):Setup(robot='evo-v3',observation_profile='rgbd')


def test_evo_workspace_has_no_invented_motion_tools(tmp_path):
    from services.workspace import Setup, build, recipe_for
    workspace=tmp_path/'ws'
    build(recipe_for(),Setup(robot='evo-v3',observation_profile='rgb-only',gemini=False),workspace)
    text=(workspace/'.agents/skills/autonomous-control/SKILL.md').read_text()
    assert '`None`' not in text
    assert 'no motion tool' in text
    assert (workspace/'.agents/skills/towel-perception-trials/SKILL.md').is_file()


def attempt_reply(requested=0, completed=0, running=False, stopped=False, state='Preparing', session='runtime-one'):
    return {'protocol':'evo.towel.control.v2','result':'status','state':state,'error':None,
            'attempt':{'session':session,'requested':requested,'completed':completed,
                       'running':running,'stop_verified':stopped,'cancel_requested':False}}


def align_backend(tmp_path, monkeypatch, replies):
    from services import arming
    monkeypatch.setattr(arming,'read',lambda _: {'armed':True})
    b=backend(tmp_path,enable_alignment=True,poll_s=.001)
    calls=[]
    def command(name):
        calls.append(name)
        if name=='cancel':return {'result':'cancel_requested'}
        result=replies.pop(0)
        if isinstance(result,Exception):raise result
        return result
    b._command=command
    return b,calls


def test_fast_failure_is_matching_completed_attempt(tmp_path,monkeypatch):
    b,calls=align_backend(tmp_path,monkeypatch,[attempt_reply(),attempt_reply(1,running=True),
        attempt_reply(1,1,False,True,'Failed')])
    result,_=b.call('evo_towel_align',{})
    assert not result['ok'] and result['measured']['attempt']['completed']==1
    assert calls==['status','start','status']


def test_old_terminal_is_not_new_success(tmp_path,monkeypatch):
    b,calls=align_backend(tmp_path,monkeypatch,[attempt_reply(1,1,False,True,'ReadyForFoldingSetup'),
        attempt_reply(2,1,True),attempt_reply(2,1,True,False,'ReadyForFoldingSetup'),
        attempt_reply(2,2,False,True,'ReadyForFoldingSetup')])
    result,_=b.call('evo_towel_align',{})
    assert result['ok'] and calls.count('status')==3


@pytest.mark.parametrize('failure',[
    attempt_reply(1,1,False,False,'Failed'),
    attempt_reply(1,0,True,False,'Preparing','restarted'),
    ConnectionError('lost policy connection')])
def test_unsafe_outcome_cancels_once_without_restart(tmp_path,monkeypatch,failure):
    b,calls=align_backend(tmp_path,monkeypatch,[attempt_reply(),attempt_reply(1,running=True),failure])
    with pytest.raises(RuntimeError):b.call('evo_towel_align',{})
    assert calls.count('start')==1 and calls.count('cancel')==1


def test_disarm_during_attempt_cancels(tmp_path,monkeypatch):
    b,calls=align_backend(tmp_path,monkeypatch,[attempt_reply(),attempt_reply(1,running=True)])
    from services import arming
    values=iter([True,False]);monkeypatch.setattr(arming,'read',lambda _: {'armed':next(values)})
    with pytest.raises(RuntimeError,match='disarmed'):b.call('evo_towel_align',{})
    assert calls[-1]=='cancel'


def test_busy_race_does_not_cancel_another_attempt(tmp_path,monkeypatch):
    from services.evo_v3.backend import PolicyBusy
    b,calls=align_backend(tmp_path,monkeypatch,[attempt_reply(),PolicyBusy('busy')])
    with pytest.raises(PolicyBusy):b.call('evo_towel_align',{})
    assert calls==['status','start']


def test_unversioned_runtime_refused_before_start(tmp_path,monkeypatch):
    b,calls=align_backend(tmp_path,monkeypatch,[{'state':'Preparing'}])
    with pytest.raises(ValueError,match='protocol v2'):b.call('evo_towel_align',{})
    assert calls==['status']
