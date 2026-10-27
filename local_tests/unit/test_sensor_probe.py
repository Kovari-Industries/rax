from types import SimpleNamespace
import pytest
from auto_research_agent.api.sensor_probe import probe_sensors


def test_readonly_probe_preserves_capture_stamps_and_continues_after_missing_scene():
    calls=[];events=[];ticks=iter(range(100))
    def value(tool):
        calls.append(tool)
        if tool=='rax_scene':raise RuntimeError('depth unavailable')
        return {'capture_us':123,'geometry_validated':False}
    result=probe_sensors(SimpleNamespace(value=value),events.append,samples=2,
                         clock=SimpleNamespace(monotonic=lambda:next(ticks),time=lambda:42))
    assert calls==['evo_capabilities','evo_status','evo_observe','rax_scene','evo_observe','rax_scene']
    assert result['complete'] and not result['passed']
    assert len(events)==12
    assert all(x['phase']=='started' for x in events[::2])
    assert all(x['phase']=='finished' and x['call_duration_s']==1 for x in events[1::2])
    assert events[5]['response']['capture_us']==123
    assert events[7]['passed'] is False


def test_rgb_only_does_not_claim_depth():
    result=probe_sensors(SimpleNamespace(value=lambda tool:{}),lambda row:None,samples=1,include_scene=False)
    assert result['passed']
    assert [x['tool'] for x in result['calls']]==['evo_capabilities','evo_status','evo_observe']

@pytest.mark.parametrize('samples',[0,101,True,1.5])
def test_bad_count_never_calls_backend(samples):
    with pytest.raises(ValueError):probe_sensors(None,lambda row:None,samples=samples)
