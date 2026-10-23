import json
import socket
import threading
from types import SimpleNamespace
import pytest
from services import arming
from operator_trials.evo_motion import validate_step, request

@pytest.mark.parametrize('args', [ {'delta':[[float('nan'),0,0],[0,0,0]]}, {'delta':[[.03,.03,0],[0,0,0]]}, {'delta':[[.03,0,0],[0,0,0]],'duration_s':2}, {'delta':[[0,0,0],[0,0,0]],'grippers':[0,.1]}, {'delta':[[0,0,0],[0,0,0]],'base':1} ])
def test_bad_steps(args):
    with pytest.raises(ValueError): validate_step(args)

def test_valid_step():
    validate_step({'delta':[[.03,0,0],[0,-.02,0]],'duration_s':3,'grippers':[-.6,0]})

@pytest.mark.parametrize('wrong_id,stopped', [(False,True),(True,True),(False,False)])
def test_only_same_attempt_with_stop(tmp_path,wrong_id,stopped):
    config=SimpleNamespace(root=tmp_path)
    arming.write(tmp_path,True,minutes=1)
    path=str(tmp_path/'motion.sock')
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(path);server.listen()
    def serve():
        conn,_=server.accept()
        with conn:
            data=conn.makefile('rb').readline();value=json.loads(data)
            conn.sendall((json.dumps({'protocol':'evo.rax.motion.v1','step_id':'wrong' if wrong_id else value['id'],'stop_verified':stopped,'ok':True})+'\n').encode())
    thread=threading.Thread(target=serve);thread.start()
    try:
        if wrong_id or not stopped:
            with pytest.raises(RuntimeError):request(config,path,{'delta':[[0,0,0],[0,0,0]]})
        else:assert request(config,path,{'delta':[[0,0,0],[0,0,0]]})[0]['ok']
    finally:thread.join(timeout=3);server.close()

def test_unarmed_does_not_connect(tmp_path):
    with pytest.raises(RuntimeError,match='arming'): request(SimpleNamespace(root=tmp_path),str(tmp_path/'missing'),{'delta':[[0,0,0],[0,0,0]]})
