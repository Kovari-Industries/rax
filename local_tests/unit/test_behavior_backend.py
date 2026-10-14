"""BEHAVIOR-1K backend: contract, validation, odometry, geometry, privacy and host separation.

No simulator: the backend is exercised against a fake simulator process.
"""
import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services.behavior import robot as R
from services.behavior import tools as T
from services.behavior.tasks import (FORMAL_COLLECTION, PUBLIC_TEST_INSTANCE_IDS, instance_ids, layout_instance,
                                     native_step_limit, task_info, task_names, task_timeouts, validate_layouts)
from services.benchmarks import BEHAVIOR_TOOLS, BENCHMARKS, ROBODOJO_TOOLS, planning_only
from services.mcp_contract import Contract
from services.simulator.io import RejectedBeforeAction

ROOT = Path(__file__).resolve().parents[2]


def hold_row():
    row = np.zeros(R.ACTION_DIM)
    row[R.ACTION_SLICES["gripper_left"]] = row[R.ACTION_SLICES["gripper_right"]] = 1.0
    return row


# ------------------------------------------------------------------ contract
def test_contract_selects_benchmark_tools():
    behavior = Contract(benchmark='behavior')
    assert behavior.robot_names == BEHAVIOR_TOOLS
    assert Contract(observation_profile='rgb-only', benchmark='behavior').robot_names == \
        BEHAVIOR_TOOLS - {'behavior_pixel_to_position'}
    assert Contract().robot_names == ROBODOJO_TOOLS
    with pytest.raises(PermissionError):
        behavior.require_robot('robodojo_step', {})
    with pytest.raises(PermissionError):
        Contract().require_robot('behavior_step', {})
    assert behavior.scoped_tools >= {'behavior_step', 'start_episode'}


def test_unknown_benchmark_is_rejected():
    with pytest.raises(ValueError, match='Unknown benchmark'):
        Contract(benchmark='isaaclab')


def test_manifest_shape_is_unchanged_for_robodojo():
    assert 'benchmark' not in Contract().manifest()
    assert Contract(benchmark='behavior').manifest()['benchmark'] == 'behavior'


def test_multi_env_scoping_adds_env_id_to_behavior_tools():
    contract = Contract(environments=2, benchmark='behavior')
    tools = {t['name']: t for t in contract.robot_definitions()}
    assert 'env_id' in tools['behavior_step']['inputSchema']['required']
    assert set(tools) == BEHAVIOR_TOOLS


def test_planning_only_and_mutating_sets():
    assert planning_only('behavior_free_space_move', {'preview_only': True})
    assert not planning_only('behavior_free_space_move', {'preview_only': False})
    assert not planning_only('behavior_step', {'preview_only': True})
    assert BENCHMARKS['behavior'].mutating <= BEHAVIOR_TOOLS
    assert {t['name'] for t in T.tool_definitions()} == BEHAVIOR_TOOLS


# ------------------------------------------------------------------ actions
def test_validate_actions_shape_limits_and_normalized_ranges():
    lower, upper = np.full(R.ACTION_DIM, -1.0), np.full(R.ACTION_DIM, 1.0)
    for name in ("trunk", "arm_left", "arm_right"):
        lower[R.ACTION_SLICES[name]], upper[R.ACTION_SLICES[name]] = -2.0, 2.0
    assert R.validate_actions([hold_row()], lower, upper).shape == (1, 23)
    for bad in ([], [[0.0] * 22], [hold_row()] * (R.MAX_ACTION_ROWS + 1), [[float('nan')] * 23]):
        with pytest.raises(RejectedBeforeAction):
            R.validate_actions(bad, lower, upper)
    row = hold_row()
    row[0] = 1.5
    with pytest.raises(RejectedBeforeAction, match='base'):
        R.validate_actions([row], lower, upper)
    row = hold_row()
    row[R.ACTION_SLICES["arm_left"].start] = 2.5
    with pytest.raises(RejectedBeforeAction, match='arm_left'):
        R.validate_actions([row], lower, upper)


def test_hold_action_repeats_measured_joints_and_gripper_commands():
    proprio = np.arange(R.PROPRIO_DIM, dtype=float)
    row = R.hold_action(proprio, (0.25, -0.5))
    fields = R.decode_proprio(proprio)
    assert row[R.ACTION_SLICES["base"]].tolist() == [0, 0, 0]
    assert row[R.ACTION_SLICES["arm_right"]].tolist() == fields["arm_right_qpos"]
    assert row[R.ACTION_SLICES["trunk"]].tolist() == fields["trunk_qpos"]
    assert row[14] == 0.25 and row[22] == -0.5


# ----------------------------------------------------------------- odometry
def test_odometry_integrates_robot_frame_velocity():
    odometry = R.Odometry()
    for _ in range(R.CONTROL_HZ):  # 1 s forward at 0.3 m/s
        odometry.update([0.3, 0.0, 0.0])
    assert odometry.x == pytest.approx(0.3, abs=1e-9) and abs(odometry.y) < 1e-12
    for _ in range(R.CONTROL_HZ):  # 1 s turning at pi/2 rad/s
        odometry.update([0.0, 0.0, math.pi / 2])
    assert odometry.yaw == pytest.approx(math.pi / 2, abs=0.03)
    for _ in range(R.CONTROL_HZ):  # forward in the new heading is +y in the start frame
        odometry.update([0.3, 0.0, 0.0])
    assert odometry.y == pytest.approx(0.3, abs=0.02) and odometry.x == pytest.approx(0.3, abs=0.02)


def test_base_command_converges_under_ideal_kinematics():
    odometry = R.Odometry()
    goal = R.relative_goal(odometry, 1.0, -0.5, 1.0)
    for _ in range(40 * R.CONTROL_HZ):
        command, (dist, yaw) = R.base_command(odometry, goal)
        assert np.all(np.abs(command) <= 1.0 + 1e-9)
        odometry.update(command * np.asarray(R.BASE_VELOCITY_SCALE))
        if dist < 0.01 and yaw < 0.01:
            break
    assert dist < 0.01 and yaw < 0.01


# ---------------------------------------------------------------- geometry
def test_pixel_and_base_projection_round_trip():
    intrinsics = {"matrix": [[306.0, 0, 360.0], [0, 306.0, 360.0], [0, 0, 1]]}
    # Camera 1.5 m up, looking along +x of the base (USD camera looks down its -Z).
    # Columns are the camera X (right), Y (up), Z (backward) axes in the base frame.
    look = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
    from scipy.spatial.transform import Rotation
    pose = {"position_m": [0.0, 0.0, 1.5], "quaternion_xyzw": Rotation.from_matrix(look).as_quat().tolist()}
    point = [2.0, 0.3, 1.2]
    u, v, z = T.base_to_pixel(point, intrinsics, pose)
    assert z == pytest.approx(2.0)
    depth = np.full((720, 720), z)
    back = T.pixel_to_base(depth, intrinsics, pose, u, v, radius=0)
    assert np.allclose(back["position_m"], point, atol=1e-6)
    assert T.pixel_to_base(np.zeros((720, 720)), intrinsics, pose, u, v)["valid"] is False


def test_request_validation_rejects_malformed_calls_before_dispatch():
    T.validate_request('behavior_status', {})
    for name, arguments in (('behavior_status', {'x': 1}), ('behavior_base_move', {'dx': 1}),
                            ('behavior_base_move', {'dx': 1, 'dy': 0, 'dyaw': True}),
                            ('behavior_free_space_move', {'arm': 'left', 'position': [0, 0, 0],
                                                          'quaternion_xyzw': [0, 0, 0, 2]}),
                            ('behavior_pixel_to_position', {'observation_step_id': 0,
                                                            'selections': [{'camera': 'cam_high', 'u': 1, 'v': 1}]}),
                            ('robodojo_step', {})):
        with pytest.raises(RejectedBeforeAction):
            T.validate_request(name, arguments)


# ------------------------------------------------------------------- tasks
def test_task_catalog_and_held_out_collections():
    assert 'turning_on_radio' in task_names() and len(task_names()) == 100  # the full 2026 challenge set
    assert sorted(task_info(t)['task_id'] for t in task_names()) == list(range(100))
    assert task_info('turning_on_radio')['results_2025']['teams'] > 0 and task_info('installing_a_modem')['results_2025'] is None
    for task in task_names():
        assert len(instance_ids(task, 0)) == 300 and task_timeouts(task)['native_step_limit'] > 0
    assert native_step_limit('turning_on_radio') == int(task_info('turning_on_radio')['human_mean_length_steps'] * 1.5)
    assert layout_instance('turning_on_radio', FORMAL_COLLECTION, 0) == 301
    assert instance_ids('turning_on_radio', FORMAL_COLLECTION) == PUBLIC_TEST_INSTANCE_IDS
    for task in task_names():
        assert not set(instance_ids(task, 0)) & set(PUBLIC_TEST_INSTANCE_IDS)
    with pytest.raises(ValueError):
        validate_layouts('turning_on_radio', FORMAL_COLLECTION, range(15, 25))
    # Exploration cycles through the train instances.
    train = instance_ids('turning_on_radio', 0)
    assert layout_instance('turning_on_radio', 0, len(train) + 3) == train[3]
    validate_layouts('turning_on_radio', 0, range(295, 305))
    with pytest.raises(ValueError):
        task_info('make_kong')
    timeouts = task_timeouts('turning_on_radio')
    # An episode ends at its step limit: no wall-clock limit, only a startup deadline.
    assert timeouts['episode_seconds'] is None and timeouts['startup_seconds'] == 900


# ---------------------------------------------------------------- backend
class FakeProcess:
    """In-memory stand-in for services.behavior.process.BehaviorProcess."""

    def __init__(self, output_root, success=False):
        self.output_root = Path(output_root)
        self.process = self.rpc = None
        self.episode_id, self.step_id, self.run_id, self.output_dir = None, 0, None, None
        self.finished, self.startup_seconds, self.success = False, 1.0, success
        self.requests = []

    @property
    def active(self):
        return self.process is not None and not self.finished

    def start(self, *, task, collection, instance_id, run_id, **kwargs):
        self.task, self.collection, self.instance_id, self.run_id = task, collection, instance_id, run_id
        self.start_options = kwargs
        self.output_dir = self.output_root / run_id
        (self.output_dir / 'sim').mkdir(parents=True)
        self.process, self.episode_id, self.step_id, self.finished = object(), f'ep-{instance_id}', 0, False
        self._state()
        return {'episode_id': self.episode_id, 'step_id': 0, 'native_step_limit': 3224}

    def _state(self):
        (self.output_dir / 'sim' / 'operator_state.json').write_text(json.dumps({
            'episode_id': self.episode_id, 'step_id': self.step_id,
            'reward': {'native_success': self.success, 'q_score': 0.5, 'native_score_percent': 50.0}}))

    def observation(self):
        obs = {c: np.full((8, 8, 3), 100, np.uint8) for c in R.CAMERAS}
        obs.update({f'{c}_depth_m': np.ones((8, 8), np.float32) for c in R.CAMERAS})
        obs.update(proprio=list(range(61)), proprio_fields=R.decode_proprio(np.arange(61.0)),
                   odometry=R.Odometry().pose(), gripper_commands={'left': 1, 'right': 1},
                   gripper_openings={'left': 1, 'right': 1}, instruction='Turn on the radio',
                   camera_intrinsics={c: {'matrix': [[8, 0, 4], [0, 8, 4], [0, 0, 1]]} for c in R.CAMERAS},
                   camera_poses={c: {'position_m': [0, 0, 1], 'quaternion_xyzw': [0, 0, 0, 1]} for c in R.CAMERAS},
                   step_id=self.step_id)
        return obs

    def request(self, op, **kwargs):
        self.requests.append((op, kwargs))
        if op == 'observe':
            return self.observation()
        if op == 'chunk_step':
            frames = []
            for _ in range(len(kwargs['actions'])):
                self.step_id += 1
                if self.step_id % R.FRAME_STRIDE == 0:
                    frames.append(self.observation())
            if not frames or frames[-1]['step_id'] != self.step_id:
                frames.append(self.observation())
            self._state()
            ended = self.success
            return {'episode_id': self.episode_id, 'step_id': self.step_id, 'frames': frames,
                    'steps': [{'step_id': self.step_id}], 'terminated': ended, 'truncated': False,
                    **({'success': True} if ended else {})}
        raise AssertionError(op)

    def finish(self, reason):
        self.finished = True

    def stop(self):
        self.process = None
        self.finished = True


def backend_for(tmp_path, profile='rgbd', success=False):
    from services.behavior.backend import BehaviorBackend
    config = SimpleNamespace(root=tmp_path, task='turning_on_radio', benchmark='behavior',
                             observation_profile=profile, exploration_envs=1, subagents=0, eval_seed=0,
                             formal_collection=1, sim_gpu='GPU-0', formal=SimpleNamespace(wall_seconds=3000))
    backend = BehaviorBackend(config)
    backend.sim = FakeProcess(tmp_path / 'native' / 'results', success=success)
    return backend


PRIVATE_WORDS = ('q_score', 'native_score', 'native_success', 'goal_predicates', 'world_base', 'robot_pos',
                 'joint_qpos', 'base_qpos', 'instance_id', 'seg_')


def assert_public(value):
    text = json.dumps(value)
    for word in PRIVATE_WORDS:
        assert word not in text, word


@pytest.mark.parametrize('wall, simulator_timeout', [(None, 0), (3000, 3000)])
def test_simulator_takes_the_wall_limit_only_when_there_is_one(tmp_path, wall, simulator_timeout):
    backend = backend_for(tmp_path)
    backend.config.formal.wall_seconds = wall
    backend.start(formal=True, seed=3)
    # 0: the simulator's own episode timer is off, so the episode ends at its step limit.
    assert backend.sim.start_options['episode_timeout_seconds'] == simulator_timeout


def test_backend_start_observe_step_and_evaluate(tmp_path):
    backend = backend_for(tmp_path)
    value, images = backend.start(formal=True, seed=3)
    assert backend.sim.collection == 1 and backend.sim.instance_id == 304
    assert value['environment_spec']['episode_step_limit'] == 3224
    assert len(images) == 6  # three RGB + three depth attachments
    assert_public(value)
    value, images = backend.call('behavior_step', {'actions': [hold_row().tolist()] * 13})
    assert value['step_id'] == 13 and value['frame_sequence']['frame_count'] == 3
    assert_public(value)
    manifest = json.loads((backend.frame_workspace / value['frame_sequence']['manifest_path']).read_text())
    assert [f['step_id'] for f in manifest['frames']] == [6, 12, 13]
    from services.controller.recording import IMAGE_NAME
    assert all(IMAGE_NAME.fullmatch(Path(item['path']).name) for f in manifest['frames'] for item in f['files'])
    evaluation = backend.evaluate()
    assert evaluation['task_complete'] is False and set(evaluation) >= {'status', 'episode_id', 'step_id'}
    assert_public(evaluation)
    point = backend.call('behavior_pixel_to_position',
                         {'observation_step_id': 13, 'selections': [{'camera': 'cam_head', 'u': 4, 'v': 4}]})[0]
    assert point['selections'][0]['valid']
    with pytest.raises(ValueError, match='Stale'):
        backend.call('behavior_pixel_to_position',
                     {'observation_step_id': 12, 'selections': [{'camera': 'cam_head', 'u': 4, 'v': 4}]})


def test_backend_rgb_only_removes_depth_and_calibration(tmp_path):
    backend = backend_for(tmp_path, 'rgb-only')
    value, images = backend.start(formal=False, seed=0)
    assert len(images) == 3
    text = json.dumps(value)
    assert 'camera_intrinsics' not in text and 'camera_poses' not in text and '"depth"' not in text
    assert 'behavior_pixel_to_position' not in {t['name'] for t in backend.tools()}
    with pytest.raises(PermissionError):
        backend.call('behavior_pixel_to_position',
                     {'observation_step_id': 0, 'selections': [{'camera': 'cam_head', 'u': 4, 'v': 4}]})


def test_backend_reports_success_and_episode_end(tmp_path):
    backend = backend_for(tmp_path, success=True)
    backend.start(formal=False, seed=0)
    value, _ = backend.call('behavior_step', {'actions': [hold_row().tolist()]})
    assert value['transition']['episode_ended'] is True and value['transition']['task_complete'] is True
    assert backend.evaluate()['task_complete'] is True


def test_backend_rejects_other_benchmark_tools(tmp_path):
    backend = backend_for(tmp_path)
    backend.start(formal=False, seed=0)
    with pytest.raises(PermissionError):
        backend.call('robodojo_step', {'actions': []})


# -------------------------------------------------------------- separation
def test_host_modules_never_import_the_simulator_stack():
    """Every host-side module imports with OmniGibson, Isaac Sim and BDDL unavailable."""
    code = r'''
import importlib, sys
class Block:
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in {'omnigibson', 'isaacsim', 'bddl', 'omni', 'pxr', 'curobo'}:
            raise ImportError('blocked ' + name)
sys.meta_path.insert(0, Block())
for module in ('services.benchmarks', 'services.mcp_contract', 'services.behavior.robot', 'services.behavior.tools',
               'services.behavior.tasks', 'services.behavior.process', 'services.behavior.preview', 'services.behavior.backend',
               'services.controller.backend', 'services.controller.gateway', 'services.controller.frontend',
               'services.controller.supervisor', 'services.exploration.research', 'services.workspace.build'):
    importlib.import_module(module)
print('ok')
'''
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0 and result.stdout.strip() == 'ok', result.stderr


def test_dashboard_infers_the_benchmark_from_the_task():
    from services.dashboard.sessions import benchmark_of, timeouts
    assert benchmark_of('turning_on_radio') == 'behavior' and benchmark_of('make_kong') == 'robodojo'
    assert timeouts('turning_on_radio')['native_step_limit'] == native_step_limit('turning_on_radio')


def test_gateway_lists_and_routes_behavior_tools(tmp_path):
    """The production gateway advertises and dispatches the session benchmark's robot tools."""
    from services.controller.gateway import Gateway
    backend = backend_for(tmp_path)
    events = []
    gateway = Gateway(backend, None, contract=backend.contract, audit=events.append, charge=lambda *_: None,
                      refund=lambda *_: None, budget=lambda: {})
    token = gateway.acquire()
    listed = gateway.handle(token, {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/list'})['result']['tools']
    names = {t['name'] for t in listed}
    assert BEHAVIOR_TOOLS <= names and not any(n.startswith('robodojo_') for n in names)
    backend.start(formal=False, seed=0)
    reply = gateway.handle(token, {'jsonrpc': '2.0', 'id': 2, 'method': 'tools/call',
                                   'params': {'name': 'behavior_step', 'arguments': {'actions': [hold_row().tolist()] * 2}}})
    assert json.loads(reply['result']['content'][0]['text'])['step_id'] == 2
    refused = gateway.handle(token, {'jsonrpc': '2.0', 'id': 3, 'method': 'tools/call',
                                     'params': {'name': 'robodojo_step', 'arguments': {'actions': []}}})
    assert 'error' in refused


def test_configuration_rejects_fields_that_do_not_apply_to_behavior(tmp_path):
    from dataclasses import replace
    from services.controller.config import Configuration, Limits
    from services.storage_root import artifact_root
    limits = Limits(60, 512, 1, 32, 32, 1024)
    base = Configuration(artifact_root() / 'unit-behavior-config', 'turning_on_radio', (0, 1), 0, 0,
                         'GPU-aaaa', limits, limits, limits, formal_eval_seed=1, benchmark='behavior')
    for change, message in (({'task_rules': 'code-summary'}, 'task_rules'), ({'write_handoff': True}, 'Adaptation'),
                            ({'formal_eval_seed': 0}, 'held-out'), ({'observation_profile': 'rgb-only'}, 'RGB-D')):
        with pytest.raises(ValueError, match=message):
            replace(base, **change)
    assert replace(base, demonstration_context='completion_sequence').demonstration_context == 'completion_sequence'
    assert replace(base, agent_effort='max').agent_effort == 'max'
    with pytest.raises(ValueError, match='agent_effort'):
        replace(base, agent_effort='ultra')


def _demo_cache(root, task='turning_on_radio', frames=((0, 5, 9), (0, 7))):
    jpeg = b'\xff\xd8' + b'\0' * 16 + b'\xff\xd9'
    demos = []
    for number, indices in enumerate(frames, 1):
        (root/task/f'demo-{number}').mkdir(parents=True)
        for index in indices:
            (root/task/f'demo-{number}/frame_{index:06d}.jpg').write_bytes(jpeg)
        demos.append({'directory': f'demo-{number}', 'episode_index': 40 + number, 'frames': [
            {'image': f'demo-{number}/frame_{index:06d}.jpg', 'frame_index': index, 'skill_end': None} for index in indices]})
    (root/task/'demos.json').write_text(json.dumps({'task': task, 'demos': demos}))


def test_behavior_demonstrations_expose_only_selected_images(tmp_path):
    from services.behavior.demonstrations import provision_trial_demonstration
    from services.robodojo.demonstrations import DemonstrationError
    cache, target = tmp_path/'cache', tmp_path/'trial/runtime/demonstrations'
    with pytest.raises(DemonstrationError, match='setup_behavior_demos'):
        provision_trial_demonstration(cache_root=cache, target=target, task='turning_on_radio',
                                      context_kind='terminal_state')
    _demo_cache(cache)
    result = provision_trial_demonstration(cache_root=cache, target=target, task='turning_on_radio',
                                           context_kind='terminal_state')
    manifest = json.loads((target/'manifest.json').read_text())
    assert result['image_count'] == 2 and manifest['ordered_images'] == ['demo-1/terminal.jpg', 'demo-2/terminal.jpg']
    result = provision_trial_demonstration(cache_root=cache, target=target, task='turning_on_radio',
                                           context_kind='completion_sequence')
    manifest = json.loads((target/'manifest.json').read_text())
    assert result['image_count'] == 5 and [len(d['ordered_images']) for d in manifest['demonstrations']] == [3, 2]
    from services.controller.demonstrations import task_demonstration_context
    text = task_demonstration_context(result)
    assert 'Supplied: 2 completion sequences, each in its own layout (5 images in all).' in text
    assert "`demonstrations` lists each sequence's images in order." in text and 'a completion sequence' not in text
    files = sorted(str(p.relative_to(target)) for p in target.rglob('*') if p.is_file())
    assert files == ['demo-1/frame_000.jpg', 'demo-1/frame_001.jpg', 'demo-1/frame_002.jpg',
                     'demo-2/frame_000.jpg', 'demo-2/frame_001.jpg', 'manifest.json']
    assert 'episode_index' not in (target/'manifest.json').read_text()
    assert provision_trial_demonstration(cache_root=cache, target=target, task='turning_on_radio',
                                         context_kind='none')['image_count'] == 0
    assert not list(target.iterdir())


def test_behavior_sessions_provision_from_the_behavior_cache(tmp_path):
    from services.controller.demonstrations import provision_demonstration
    _demo_cache(tmp_path/'reference-demos/behavior')
    config = SimpleNamespace(benchmark='behavior', task='turning_on_radio', demonstration_context='terminal_state',
                             root=tmp_path/'session')
    config.root.mkdir()
    context = provision_demonstration(config, tmp_path/'ws', tmp_path/'reference-demos')
    assert context['image_count'] == 2 and (tmp_path/'ws/runtime/demonstrations/demo-2/terminal.jpg').is_file()


def test_appdata_slots_hold_four_env_sessions_beside_processes_on_the_old_cap(tmp_path):
    import fcntl
    from services.behavior.process import AppdataSlot
    root = tmp_path/'appdata'
    root.mkdir()
    old = []  # A process started under the 16-slot cap holds the lowest slots.
    for index in range(16):
        handle = (root/f'slot-{index:02d}.lock').open('a')
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        old.append(handle)
    leases = [AppdataSlot(root) for _ in range(16)]  # Two 4-env sessions: a simulator and a preview per env.
    paths = [lease.acquire() for lease in leases]
    assert [p.name for p in paths] == [f'slot-{i:02d}' for i in range(31, 15, -1)]
    with pytest.raises(RuntimeError, match=f'All 32 BEHAVIOR appdata slots in {root} are in use'):
        AppdataSlot(root).acquire()
    old.pop(0).close()  # An old-cap lease ends; its slot is free for anyone.
    leases[0].release()
    later = [AppdataSlot(root), AppdataSlot(root)]  # Held: a lease lasts while its handle is open.
    assert [lease.acquire().name for lease in later] == ['slot-31', 'slot-00']
