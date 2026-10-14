"""Call-wide rejection semantics through native dispatch, RPC, batching and gateway."""
from types import SimpleNamespace

import numpy as np
import pytest

from local_tests.unit.test_episode_errors import session as native_session
from local_tests.unit.test_motion_gateway_errors import call, target
from local_tests.unit.test_rpc_rejections import Socket
from services.controller.backend import NativeBackend
from services.controller.gateway import Gateway
from services.mcp_contract import Contract
from services.robodojo.mcp_server import RoboDojoMCP, SimulatorProcess
from services.simulator import protocol
from services.simulator.io import PartialExecutionError, RejectedBeforeAction


def native_gateway(tmp_path, monkeypatch, failure, fail_at=2):
    native = native_session(tmp_path)
    native.episode_dir = tmp_path / "episode"
    native.episode_dir.mkdir()
    native.metadata["control_frequency_hz"] = 25
    native.env.take_action_cnt = [0]
    native.env.end_flag = native.env.success = [False]
    def take_action(command):
        native.env.take_action_cnt[0] += 1
    native.env.take_action = take_action
    native._observe = lambda **_: {"states": np.zeros(14)}
    proposals = []
    def propose(targets):
        proposals.append(targets)
        if failure == "rejection" and len(proposals) == fail_at:
            # The same ValueError-to-RejectedBeforeAction path as DualKinematics.check.
            raise ValueError("injected FK check failure")
        return {"action": np.zeros(14), "diagnostics": {}}
    native.kinematics = SimpleNamespace(target=propose)

    client = protocol.RPCClient.__new__(protocol.RPCClient)
    client.sock = Socket()
    requests = []
    monkeypatch.setattr(protocol, "send_packet", lambda _, request: requests.append(request))
    def receive(_):
        request = requests[-1]
        if failure == "transport":
            raise EOFError("connection closed")
        if failure == "timeout":
            raise TimeoutError("response timed out")
        reply = {"version": protocol.VERSION, "request_id": request["request_id"]}
        try:
            reply.update(ok=True, result=native.dispatch(request["op"], request["args"]))
        except RejectedBeforeAction as exc:
            reply.update(ok=False, error=str(exc), error_type="RejectedBeforeAction")
        if failure == "identity":
            # Even a claimed pre-action rejection is untrusted with the wrong identity.
            reply.update(request_id="other", ok=False, error="bad target", error_type="RejectedBeforeAction")
        return reply
    monkeypatch.setattr(protocol, "receive_packet", receive)

    sim = SimulatorProcess.__new__(SimulatorProcess)
    sim.rpc, sim.finished, sim.log_path = client, False, None
    sim.process = SimpleNamespace(wait=lambda timeout: None, poll=lambda: None)
    sim.episode_id, sim.step_id, sim.metadata = native.episode_id, native.step_id, native.metadata
    robot = RoboDojoMCP.__new__(RoboDojoMCP)
    robot.sim = sim
    robot._observation_sequence_packet = lambda frames, transition: (transition, [])
    backend = NativeBackend.__new__(NativeBackend)
    backend.contract, backend.robot, backend._evaluation = Contract(), robot, None
    gateway = Gateway(backend, None, contract=backend.contract, audit=lambda _: None,
                      charge=lambda *_: None, refund=lambda *_: None, budget=lambda: {})
    return gateway, gateway.acquire(), native, requests, client


def test_later_eef_proposal_rejection_after_native_action_is_partial_and_continues(tmp_path, monkeypatch):
    gateway, token, native, requests, client = native_gateway(tmp_path, monkeypatch, "rejection")
    failure = call(gateway, token, "robodojo_step_eef", targets=[target()] * 2)["error"]["data"]
    assert [r["op"] for r in requests] == ["eef_joint_target", "chunk_step", "eef_joint_target"]
    assert native.env.take_action_cnt == [1] and native.step_id == 8
    assert failure["type"] == "PartialExecutionError" and "1 native action rows executed" in failure["reason"]
    assert "injected FK check failure" in failure["reason"]
    assert not failure["episode_ended"] and not gateway.terminal and client.sock is not None
    # The episode continues from the simulator's actual step.
    assert "result" in call(gateway, token, "robodojo_step_eef", targets=[target()])
    assert native.env.take_action_cnt == [2] and native.step_id == 9


def test_first_eef_proposal_rejection_is_recoverable(tmp_path, monkeypatch):
    gateway, token, native, _, client = native_gateway(tmp_path, monkeypatch, "rejection", fail_at=1)
    failure = call(gateway, token, "robodojo_step_eef", targets=[target()])["error"]["data"]
    assert failure["type"] == "RejectedBeforeAction" and "injected FK check failure" in failure["reason"]
    assert not failure["episode_ended"] and not gateway.terminal
    assert native.env.take_action_cnt == [0] and client.sock is not None
    assert "result" in call(gateway, token, "robodojo_step_eef", targets=[target()])
    assert native.env.take_action_cnt == [1]


@pytest.mark.parametrize("failure,cause", [("transport", "EOFError: connection closed"),
                                           ("timeout", "TimeoutError: response timed out"),
                                           ("identity", "ValueError: RPC response identity mismatch")])
def test_lost_simulator_connection_ends_the_episode_with_its_cause(tmp_path, monkeypatch, failure, cause):
    gateway, token, _, _, client = native_gateway(tmp_path, monkeypatch, failure)
    result = call(gateway, token, "robodojo_step_eef", targets=[target()])["error"]["data"]
    assert result["type"] == "SimulatorExited" and cause in result["reason"]
    assert "simulator process still running" in result["reason"]
    assert result["episode_ended"] and gateway.terminal and client.sock is None


def test_behavior_reply_observation_rejection_after_motion_is_partial(tmp_path):
    from local_tests.unit.test_behavior_backend import backend_for, hold_row
    backend = backend_for(tmp_path)
    backend.start(formal=False, seed=0)
    request = backend.sim.request
    def without_frames(op, **args):
        if op == "observe":
            raise RejectedBeforeAction("injected observation rejection")
        result = request(op, **args)
        result["frames"] = []
        return result
    backend.sim.request = without_frames
    gateway = Gateway(backend, None, contract=backend.contract, audit=lambda _: None,
                      charge=lambda *_: None, refund=lambda *_: None, budget=lambda: {})
    failure = call(gateway, gateway.acquire(), "behavior_step", actions=[hold_row().tolist()])["error"]["data"]
    assert backend.sim.step_id == 1 and failure["type"] == "PartialExecutionError"
    assert "injected observation rejection" in failure["reason"] and not gateway.terminal


def test_cached_native_plan_later_chunk_rejection_is_partial(tmp_path):
    native = native_session(tmp_path)
    native._motion_state = lambda: np.zeros(14)
    native._motion_plans = {"plan": {"motion_plan_id": "plan", "step_id": 7,
                                     "start_state": np.zeros(14), "actions": np.zeros((2, 14))}}
    def chunk(actions, **kwargs):
        if native.step_id == 8:
            raise RejectedBeforeAction("injected second chunk rejection")
        native.step_id += 1
        return {"frames": [], "steps": [{"step_id": native.step_id}]}
    native.chunk_step = chunk
    with pytest.raises(PartialExecutionError, match="1 native action rows executed"):
        native.dispatch("execute_motion_plan", {"episode_id": native.episode_id, "step_id": 7,
                                                "motion_plan_id": "plan"})
