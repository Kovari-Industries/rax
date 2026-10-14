"""Refused requests name their reason; only a concrete invalid reason removes an episode from the success rate."""
from types import SimpleNamespace

import numpy as np
import pytest

from services.simulator.io import RejectedBeforeAction
from services.robodojo.session import RoboDojoSession


def session(tmp_path):
    value = RoboDojoSession.__new__(RoboDojoSession)
    value.env = SimpleNamespace(reward_manager=SimpleNamespace(get_score=lambda: [0.0]), get_score=lambda: None,
                                unstable_envs=set(), enforce_step_limit=True)
    value.output, value.episode_id, value.step_id = tmp_path, 'episode', 7
    value.video_writer, value.video_frames, value.finish_reason = None, 8, None
    value.metadata = {'control_dt': .04}
    value.native_step_limit = value.episode_step_limit = 100
    value.success = value.terminated = value.truncated = value.finished = False
    value.invalid_reason = None
    value._write_operator_state = lambda **kw: None
    return value


@pytest.mark.parametrize('actions', [np.zeros((1, 13)), np.full((1, 14), np.nan), np.full((1, 14), 2.0)])
def test_invalid_actions_are_rejected_before_any_native_action(tmp_path, actions):
    value = session(tmp_path)
    with pytest.raises(RejectedBeforeAction):
        value.chunk_step(actions)
    value.truncated = True
    with pytest.raises(RejectedBeforeAction, match='Episode finished'):
        value.chunk_step(np.zeros((1, 14)))


def test_stale_request_names_both_identities_and_invalid_episode_still_closes(tmp_path):
    value = session(tmp_path)
    with pytest.raises(ValueError, match='request for episode episode step 5; the simulator is at episode episode step 7'):
        value.dispatch('teacher_observation', {'episode_id': 'episode', 'step_id': 5})
    value.invalid_reason = 'episode_wall_timeout'
    with pytest.raises(ValueError, match='Stale'):
        value.dispatch('finish_pilot', {'episode_id': 'other', 'step_id': 7, 'reason': 'x'})
    result = value.dispatch('finish_pilot', {'episode_id': 'episode', 'step_id': 3, 'reason': 'controller_error'})
    assert result['episode_id'] == 'episode' and value.finished
    import json
    outcome = json.loads((tmp_path/'evaluation_outcome.json').read_text())
    assert outcome['invalid_reason'] == 'episode_wall_timeout' and outcome['valid_for_success_rate'] is False
    assert 'control_uncertain' not in outcome
