from types import SimpleNamespace as S
import pytest
from operator_trials.evo_hold import check_sample

def sample():
 return [S(feedback_stamp=S(sec=1,nanosec=0),command_stamp=S(sec=1,nanosec=0),feedback_valid=True,command_valid=True,feedback_faulted=False,feedback_stale=False,feedback_missing=False,status=1,position=0.,velocity=0.,command_position=.05,command_velocity=0.,reported_effort=8.7,rotor_temperature_c=35.,mos_temperature_c=31.) for _ in range(7)]

def test_recorded_scale_error_is_admissible():
 assert check_sample(sample(),1_010_000_000)==[.05]*7

@pytest.mark.parametrize('field,value', [('reported_effort',9.),('command_position',.1),('command_velocity',.001),('velocity',float('nan')),('feedback_faulted',True),('rotor_temperature_c',50.)])
def test_guard_rejects(field,value):
 s=sample();setattr(s[3],field,value)
 with pytest.raises(RuntimeError):check_sample(s,1_010_000_000)

def test_target_change_stale_and_future():
 for now,ref in [(1_100_000_000,None),(999_999_999,None),(1_010_000_000,[.04]*7)]:
  with pytest.raises(RuntimeError):check_sample(sample(),now,ref)

def test_legacy_correction_is_not_an_agent_tool(tmp_path):
 from services.evo_v3.backend import EvoV3Backend
 b=EvoV3Backend(S(root=tmp_path,backend_options={}))
 with pytest.raises(ValueError,match='unknown Evo tool'):b.call('evo_hold_correct',{'arm':'left'})
 with pytest.raises(ValueError):b.call('evo_hold_correct',{'arm':'left','torque':1.5})

@pytest.mark.parametrize('before,after,classification', [
 (1.776e-16,1.688e-16,'insufficient_baseline_error'),
 (.03,.0299,'no_measurable_improvement'),
 (.03,.028,'observed_improvement'),
 (.03,.04,'no_measurable_improvement'),
])
def test_effect_exceeds_quantization_and_noise(before,after,classification):
 from operator_trials.evo_hold import summarize_errors
 result=summarize_errors([before]*25,[after]*25)
 assert result['classification']==classification
 assert result['ok']==(classification=='observed_improvement')

@pytest.mark.parametrize('profile,overload', [('default',False),('attended_peak',True)])
def test_driver_profile_requires_exact_match(profile,overload):
 from operator_trials.evo_hold import check_driver_profile
 check_driver_profile(False,overload,3.,profile)
 for enabled,actual,slew in [(True,overload,3.),(False,not overload,3.),(False,overload,6.)]:
  with pytest.raises(RuntimeError):check_driver_profile(enabled,actual,slew,profile)

def test_attended_profile_keeps_finite_peak_guard():
 s=sample();s[3].reported_effort=10.
 with pytest.raises(RuntimeError):check_sample(s,1_010_000_000)
 check_sample(s,1_010_000_000,profile='attended_peak')
 s[3].reported_effort=15.
 with pytest.raises(RuntimeError):check_sample(s,1_010_000_000,profile='attended_peak')
 with pytest.raises(ValueError):check_sample(s,1_010_000_000,profile='unlimited')
