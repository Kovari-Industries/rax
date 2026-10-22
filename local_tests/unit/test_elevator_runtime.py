"""Check the trusted pulse adapter without robot hardware or network access."""
import threading
from types import SimpleNamespace

from services.evo_v3.elevator import ElevatorRuntime


def runtime(step, *, stopped=True, drive=None):
    calls = []
    backend = SimpleNamespace(_stop=threading.Event(), _failure=None)

    def rpc(method, params, **kwargs):
        calls.append((method, params))
        if method == 'drive':
            if drive:
                drive()
            return {'state': 'accepted', 'action_id': 'pulse'}
        if method == 'hold':
            return {'state': 'accepted', 'action_id': 'stop'}
        if method == 'action_status':
            return {'state': 'succeeded', 'physical_stop_verified': stopped,
                    'final_state': {'source_us': 123}}
        raise AssertionError(method)

    backend._rpc = rpc
    worker = ElevatorRuntime.__new__(ElevatorRuntime)
    worker.backend = backend
    worker._lock = threading.Lock()
    worker._active = None
    worker._results = {}
    worker._thread = None

    def local(method, params):
        if method == 'begin':
            return {'goal_id': 'goal'}
        if method == 'abort':
            return {'aborted': True}
        assert method == 'step'
        return step(params)

    worker.rpc = local
    return worker, calls


def finish(worker, ident):
    worker._thread.join(timeout=2)
    assert not worker._thread.is_alive()
    return worker.result(ident)


def test_pulse_deadline_is_forwarded_and_next_scene_follows_measured_stop():
    steps = []
    pulse = {'velocity': [0.02, 0., 0.], 'duration_s': 0.06, 'valid_until_us': 456}

    def step(params):
        steps.append(params)
        return pulse if len(steps) == 1 else {'done': True, 'base_result': {'arrival': True}}

    worker, calls = runtime(step)
    ident = worker.start({})['action_id']
    result = finish(worker, ident)
    assert result['state'] == 'succeeded'
    assert result['physical_stop_verified'] is True
    assert steps == [{'goal_id': 'goal', 'after_us': 0}, {'goal_id': 'goal', 'after_us': 123}]
    assert [p for m, p in calls if m == 'drive'] == [pulse]


def test_cancel_during_dispatch_reconciles_late_acceptance():
    entered, release = threading.Event(), threading.Event()

    def drive():
        entered.set()
        assert release.wait(2)

    worker, calls = runtime(lambda _: {'velocity': [0.02, 0., 0.], 'duration_s': .06,
                                      'valid_until_us': 456}, drive=drive)
    ident = worker.start({})['action_id']
    assert entered.wait(2)
    worker.cancel(ident)
    release.set()
    result = finish(worker, ident)
    assert result['state'] == 'cancelled'
    assert result['physical_stop_verified'] is True
    assert sum(method == 'drive' for method, _ in calls) == 1
    assert sum(method == 'hold' for method, _ in calls) >= 2


def test_worker_loss_requires_measured_stop():
    def unavailable(_):
        raise ConnectionError('worker stopped')

    worker, calls = runtime(unavailable)
    ident = worker.start({})['action_id']
    result = finish(worker, ident)
    assert result['state'] == 'failed'
    assert result['physical_stop_verified'] is True
    assert calls[0][0] == 'hold'
    assert not worker.backend._stop.is_set()


def test_unverified_stop_withdraws_lease():
    def unavailable(_):
        raise ConnectionError('worker stopped')

    worker, _ = runtime(unavailable, stopped=False)
    ident = worker.start({})['action_id']
    result = finish(worker, ident)
    assert result['state'] == 'failed'
    assert result['physical_stop_verified'] is False
    assert worker.backend._stop.is_set()
    assert worker.backend._failure
