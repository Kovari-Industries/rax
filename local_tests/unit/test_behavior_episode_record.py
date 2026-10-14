"""The simulator's operator record, the host's step and evaluation agree after a call fails.

A real BehaviorSession (physics and rendering stubbed) is served by the real RPC server to
a real host BehaviorProcess and BehaviorBackend.evaluate.
"""
import json
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services.behavior import robot as R
from services.behavior.backend import BehaviorBackend
from services.behavior.process import BehaviorProcess
from services.behavior.session import BehaviorSession
from services.simulator import rpc
from services.simulator.io import SimulatorError, SimulatorExited
from services.simulator.protocol import RPCClient


def hold_rows(count):
    row = np.zeros(R.ACTION_DIM)
    row[R.ACTION_SLICES["gripper_left"]] = row[R.ACTION_SLICES["gripper_right"]] = 1.0
    return np.asarray([row] * count, dtype=np.float32)


def session_failing_at(output, step, error):
    """A BehaviorSession whose row ``step`` raises ``error`` after the row ran."""
    session = BehaviorSession.__new__(BehaviorSession)
    session.output, session.task, session.mode, session.episode_mode = output, "turning_on_radio", "train", "exploration"
    session.episode_id, session.instance_id, session.step_id, session.step_limit = "ep1", 7, 0, 100
    session.episode_dir = output / "ep1"
    session.episode_dir.mkdir(parents=True)
    session.terminated = session.truncated = session.success = False
    session.invalid_reason = session.finish_reason = session.truth_log = None
    session.metadata = {"task": session.task, "instruction": None}
    session.grippers = [1.0, 1.0]
    session.lower, session.upper = np.full(R.ACTION_DIM, -3.0), np.full(R.ACTION_DIM, 3.0)
    session._agent_observation = lambda: {"step_id": session.step_id, "cam_head": np.zeros((2, 2, 3), np.uint8)}
    session._save_observation = lambda: None
    session._q_score = lambda: (0.0, {"satisfied": [0], "total": [1]})
    session._write_goal_report = lambda reason: None

    def step_row(row):
        session.step_id += 1
        if session.step_id == step:
            raise error
        return False
    session._step_row = step_row
    session.obs = session._agent_observation()
    session._write_operator_state()  # As reset() leaves it.
    return session


def serve(session, tmp_path):
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    thread = threading.Thread(target=rpc.serve, args=(session, port), daemon=True)
    thread.start()
    sim = BehaviorProcess(output_root=tmp_path)
    for _ in range(50):
        try:
            sim.rpc = RPCClient("127.0.0.1", port, timeout=10)
            break
        except OSError:
            time.sleep(0.05)
    sim.output_dir, sim.log_path = tmp_path, tmp_path / "sim.log"
    sim.log_path.write_text("")
    sim.episode_id, sim.step_id = "ep1", 0
    backend = SimpleNamespace(sim=sim, _evaluation=None, config=SimpleNamespace(task="turning_on_radio"))
    return sim, backend, thread


def record(tmp_path):
    return json.loads((tmp_path / "sim" / "operator_state.json").read_text())


def test_a_wall_timeout_inside_a_call_is_evaluated_from_the_final_summary(tmp_path):
    session = session_failing_at(tmp_path / "sim", 5, rpc.EpisodeWallTimeout())
    sim, backend, thread = serve(session, tmp_path)
    with pytest.raises(SimulatorExited, match="chunk_step failed with EOFError"):
        sim.request("chunk_step", actions=hold_rows(10))
    thread.join(timeout=10)
    assert sim.step_id == 0 and sim.finished  # The reply carrying step 5 never arrived.
    assert record(tmp_path)["step_id"] == 5 and record(tmp_path)["reason"] == "episode_wall_timeout"
    evaluation = BehaviorBackend.evaluate(backend)
    assert evaluation["step_id"] == 5 and evaluation["task_complete"] is False
    assert sim.step_id == 5


def test_a_simulator_error_after_rows_ran_records_their_step(tmp_path):
    session = session_failing_at(tmp_path / "sim", 3, RuntimeError("PhysX lost the articulation"))
    sim, backend, _ = serve(session, tmp_path)
    with pytest.raises(SimulatorError, match="PhysX lost the articulation") as error:
        sim.request("chunk_step", actions=hold_rows(10))
    assert error.value.state == {"episode_id": "ep1", "step_id": 3} and sim.step_id == 3 and not sim.finished
    assert record(tmp_path)["step_id"] == 3 and record(tmp_path)["reason"] is None
    assert sim.request("observe")["step_id"] == 3  # Not the frame from before the call.
    evaluation = BehaviorBackend.evaluate(backend)
    assert evaluation["step_id"] == 3 and evaluation["task_complete"] is False
    sim.rpc.close()


def test_a_record_behind_the_host_is_still_refused(tmp_path):
    (tmp_path / "sim").mkdir()
    (tmp_path / "sim" / "operator_state.json").write_text(json.dumps(
        {"episode_id": "ep1", "step_id": 4, "reason": "episode_wall_timeout",
         "reward": {"native_success": False}}))
    sim = BehaviorProcess(output_root=tmp_path)
    sim.output_dir, sim.log_path, sim.episode_id, sim.step_id, sim.finished = tmp_path, None, "ep1", 6, True
    backend = SimpleNamespace(sim=sim, _evaluation=None, config=SimpleNamespace(task="turning_on_radio"))
    with pytest.raises(RuntimeError, match="recorded episode 'ep1' at step 4, but this episode is 'ep1' at step 6"):
        BehaviorBackend.evaluate(backend)


@pytest.mark.parametrize("limit, message", [("MAX_DECOMPRESSED_BYTES", "RPC message too large"),
                                            ("MAX_WIRE_BYTES", "Compressed RPC message too large")])
def test_a_reply_too_large_to_send_is_reported_and_the_episode_continues(tmp_path, monkeypatch, limit, message):
    from services.simulator import protocol
    session = session_failing_at(tmp_path / "sim", -1, None)
    noise = np.random.default_rng(0).integers(0, 255, (200, 200, 3), dtype=np.uint8)  # Incompressible frames.
    session._agent_observation = lambda: {"step_id": session.step_id, "cam_head": noise}
    sim, backend, _ = serve(session, tmp_path)
    monkeypatch.setattr(protocol, limit, 100_000)
    with pytest.raises(SimulatorError) as error:
        sim.request("chunk_step", actions=hold_rows(10))
    assert str(error.value).startswith(f"SimulatorError: chunk_step ran, but its reply could not be sent ({message}: ")
    assert str(error.value).endswith(" bytes, limit 100000); observe to see the current state")
    assert error.value.state == {"episode_id": "ep1", "step_id": 10} and sim.step_id == 10 and not sim.finished
    monkeypatch.undo()
    assert sim.request("observe")["step_id"] == 10
    assert BehaviorBackend.evaluate(backend)["step_id"] == 10
    sim.rpc.close()


def test_the_agent_visible_episode_id_names_neither_split_nor_instance(tmp_path, monkeypatch):
    import sys
    from types import ModuleType
    obs_utils = ModuleType("omnigibson.eval.utils.obs_utils")
    obs_utils.create_video_writer = lambda **_: None
    for name in ("omnigibson", "omnigibson.eval", "omnigibson.eval.utils"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    monkeypatch.setitem(sys.modules, "omnigibson.eval.utils.obs_utils", obs_utils)
    session = session_failing_at(tmp_path / "sim", -1, None)
    session.episode_id, session.mode, session.odometry = None, "public_test", R.Odometry()
    session.evaluator = SimpleNamespace(load_batch=lambda batch: None, _set_video_writer=lambda *_: None,
                                        _write_video=lambda state: None)
    session.state = None
    session._gripper_opening = lambda side: 1.0
    session._world_pose = lambda: [0.0, 0.0, 0.0]
    session._object_orientations = lambda: {}
    reset = session.reset(304)
    episode_id = reset["episode_id"]
    assert len(episode_id) == 32 and int(episode_id, 16) >= 0  # Random hex only.
    assert not any(word in episode_id for word in ("turning_on_radio", "public_test", "train", "_"))
    recorded = json.loads((tmp_path / "sim" / "reset.json").read_text())
    assert recorded["episode_id"] == episode_id and recorded["instance_id"] == 304 and recorded["mode"] == "public_test"
    assert record(tmp_path)["episode_id"] == episode_id and record(tmp_path)["instance_id"] == 304
    session.truth_log.close()
