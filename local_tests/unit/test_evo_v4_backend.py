from types import SimpleNamespace
import json
from pathlib import Path
import pytest
from services.backends import backend_class
from services.evo.backend import EvoBackend, EvoRefused
from services.evo_v3.backend import EvoV3Backend
from services.evo_v4.backend import EvoV4Backend
from services.workspace import Setup, build, recipe_for


def capabilities():
    return {'model': {'joint_names': ['hip_pitch_joint','carriage_joint'],
                     'limits': {'hip_pitch_joint': {'lower': -.17,'upper': 1.57},
                                'carriage_joint': {'lower': 0.,'upper': .264583333333}}},
            'scene': {'configured': True, 'settings': {'travel_joints': {'hip_pitch_joint': 0.}}}}


def test_distinct_v4_registration_and_workspace(tmp_path):
    assert backend_class('evo-v4') is EvoV4Backend
    assert not issubclass(EvoV4Backend, EvoV3Backend)
    assert issubclass(EvoV4Backend, EvoBackend)
    build(recipe_for(), Setup(robot='evo-v4',gemini=False), tmp_path/'workspace')
    folder=tmp_path/'workspace'
    assert json.loads((folder/'MCP_CONTRACT.json').read_text())['robot']=='evo-v4'
    assert 'Kovari Evo v4' in (folder/'AGENTS.md').read_text()
    for name in ('robot-interface','elevator-navigation','object-localization','live-stream'):
        assert (folder/f'.agents/skills/{name}/SKILL.md').is_file()
    assert 'hip_pitch_joint' in (folder/'.agents/skills/robot-interface/SKILL.md').read_text()
    with pytest.raises(ValueError,match='RGB-only'):
        Setup(robot='evo-v4',observation_profile='rgbd')


@pytest.mark.parametrize('fault', ['missing_hip','missing_limits','nan_limit','missing_travel_hip'])
def test_wrong_model_is_rejected_before_opening_motion_session(tmp_path,fault):
    cap=capabilities()
    if fault=='missing_hip':cap['model']['joint_names'].remove('hip_pitch_joint')
    if fault=='missing_limits':del cap['model']['limits']['carriage_joint']
    if fault=='nan_limit':cap['model']['limits']['hip_pitch_joint']['upper']=float('nan')
    if fault=='missing_travel_hip':cap['scene']['settings']['travel_joints']={}
    backend=EvoV4Backend(SimpleNamespace(root=tmp_path,backend_options={}))
    calls=[]
    def rpc(method,params,**kw):
        calls.append(method)
        return cap
    backend._rpc=rpc
    with pytest.raises(EvoRefused):
        backend.call('evo_move_joints',{'targets':{'right_joint1':.1}})
    assert calls==['capabilities']
    assert backend.session_id is None
    backend.close()


def test_loaded_v4_limits_are_preserved(tmp_path):
    cap=capabilities()
    backend=EvoV4Backend(SimpleNamespace(root=tmp_path,backend_options={}))
    backend._rpc=lambda method,params,**kw:cap
    result,_=backend.call('evo_capabilities',{})
    assert result['rax_robot']=='evo-v4'
    assert result['model']['limits']['carriage_joint']['upper']==.264583333333
    backend.close()


def test_v4_keeps_private_runtime_protections(tmp_path):
    from harness.claude_cli.launcher import protected_paths,denied_hosts
    config=SimpleNamespace(root=tmp_path,robot='evo-v4',backend_options={'control_socket':'/tmp/operator-evo-v4/agent.sock'})
    assert Path('/tmp/operator-evo-v4') in protected_paths(config)
    assert {'localhost','127.0.0.1','::1'} <= set(denied_hosts(config))
