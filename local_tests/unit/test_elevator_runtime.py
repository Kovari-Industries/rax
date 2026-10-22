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


def test_wrapper_uses_existing_evo_socket_and_forwards_command_deadline(tmp_path):
    from local_tests.unit.test_evo_v3_backend import service
    steps = []
    pulse = {'velocity': [.01, 0., 0.], 'duration_s': .06, 'valid_until_us': 456}

    def handler(request):
        method, params = request['method'], request['params']
        if method == 'elevator':
            operation = params['operation']
            if operation == 'begin':
                return {'goal_id': 'evo-goal'}
            if operation == 'abort':
                return {'aborted': True}
            assert operation == 'step'
            steps.append(params['params'])
            return pulse if len(steps) == 1 else {'done': True, 'base_result': {'arrival': True}}
        if method == 'drive':
            assert params == pulse
            return {'state': 'accepted', 'action_id': 'pulse'}
        if method == 'action_status':
            return {'state': 'succeeded', 'physical_stop_verified': True,
                    'final_state': {'source_us': 123}}
        raise AssertionError(method)

    with service(tmp_path, handler) as (backend, requests):
        accepted, _ = backend.call('rax_move_base', {'delta': [.1, 0., 0.]})
        result = finish(backend.elevator, accepted['action_id'])
        assert result['state'] == 'succeeded'
        assert result['physical_stop_verified'] is True
        assert steps[-1]['after_us'] == 123
        assert all(r.get('session_id') == backend.session_id for r in requests if r['method'] == 'elevator')
        assert not hasattr(backend.elevator, 'process')


def test_slow_evo_detector_does_not_block_hold(tmp_path):
    from local_tests.unit.test_evo_v3_backend import service
    from services.evo_v3.backend import EvoRefused
    entered, release, held = threading.Event(), threading.Event(), threading.Event()
    errors = []

    def handler(request):
        if request['method'] == 'elevator':
            assert request['params']['operation'] == 'detect'
            entered.set()
            assert release.wait(2)
            return EvoRefused('test detector unavailable')
        if request['method'] == 'hold':
            held.set()
            return {'action_id': 'hold', 'state': 'stopping'}
        raise AssertionError(request)

    with service(tmp_path, handler) as (backend, _):
        def detect():
            try:
                backend.call('rax_detect_buttons', {'camera': 'left_arm', 'score_threshold': .5})
            except EvoRefused as error:
                errors.append(error)
        worker = threading.Thread(target=detect)
        worker.start()
        try:
            assert entered.wait(2)
            backend.call('evo_hold', {})
            assert held.is_set() and worker.is_alive()
        finally:
            release.set()
            worker.join(2)
        assert len(errors) == 1
