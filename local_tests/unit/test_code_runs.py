"""Reports on a code-mode session's runs of its own code, for its later live copies."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PIL import Image

from services.agent_formal import code_runs, runner as agent_runner
from services.behavior import reviews

GOAL = {"schema": "behavior_goal_report_v1", "episode_id": "sorting_public_test_302_x", "episode_mode": "formal",
        "success": False, "reason": "close", "step_id": 400, "step_limit": 900,
        "natural_language_goal": [], "objects": {"cup.n.01_1": {"exists": True, "scene_name": "cup_qx7"}},
        "options": [[{"condition": ["ontop", "cup.n.01_1", "sink.n.01_1"], "initial": False, "final": False},
                     {"condition": ["inside", "cup.n.01_1", "cabinet.n.01_1"], "initial": False, "final": True}]]}


def trace(path, steps):
    path.write_text(''.join(json.dumps({'event': 'mcp_result', 'returned': {'step_id': s}}) + '\n' for s in steps))


def session(tmp_path, layouts=(0, 1, 2), workers=2):
    """A finished code-mode session whose formal batch ran the submitted code on ``layouts``
    (in formal-workers/, or in the session itself with one formal worker)."""
    root = tmp_path/'session'
    results = []
    for layout in layouts:
        index = layout + 1
        sim = (root/f'formal-workers/episode-{index:03d}/native/results/r/sim' if workers > 1
               else root/f'native/results/20261001T00000{index}Z_formal/sim')
        sim.mkdir(parents=True)
        (sim/'summary.json').write_text(json.dumps({'episode_id': f'e{layout}', 'step_id': 400, 'success': False}))
        (sim/'private_goal_report.json').write_text(json.dumps({**GOAL, 'episode_id': f'e{layout}'}))
        (sim/'operator_state.json').write_text(json.dumps({'episode_id': f'e{layout}'}))
        (sim/'evaluation_outcome.json').write_text(json.dumps({'native_score': 0.5, 'valid_for_score': True}))
        run = root/f'published/runs/formal-{layout}'
        run.mkdir(parents=True)
        trace(run/'mcp.jsonl', range(10, 400, 10))
        (run/'stdout.log').write_text('door attempt 0 failed\n')
        results.append({'mode': 'formal', 'episode_id': f'e{layout}', 'layout_id': layout,
                        'formal_episode_index': index, 'artifacts': {
                            'directory': f'runtime/autonomous_controller/runs/formal-{layout}',
                            'mcp_trace': f'runtime/autonomous_controller/runs/formal-{layout}/mcp.jsonl',
                            'stdout': f'runtime/autonomous_controller/runs/formal-{layout}/stdout.log'}})
    results.append({'mode': 'formal', 'driver': 'agent', 'episode_id': 'copy', 'layout_id': 0})
    (root/'state.json').write_text(json.dumps({'configuration': {'task': 'sorting_household_items'},
                                               'results': results}))
    return root


def fake_replay(diverged=None, matched=True, compared=None):
    def replay(run, output, gpu, *, steps):
        (output/'frames').mkdir(parents=True)
        frames = []
        for label, step in [*((f'step-{s:06d}', s) for s in steps), ('final', 400)]:
            for cam in code_runs.CAMERAS:
                path = f'frames/{label}_{cam}.png'
                Image.new('RGB', (64, 48), (step % 255, 0, 0)).save(output/path)
                frames.append({'step': step, 'camera': cam, 'path': path})
        record = {'replayed_steps': 400, 'recorded_steps': 400, 'compared_steps': 400 if compared is None else compared,
                  'diverged_at_step': diverged, 'diverged_threshold_m': 0.05, 'worst_base_error': {},
                  'goal_conditions_match': matched, 'frames': frames}
        (output/'replay.json').write_text(json.dumps(record))
        replay.calls.append((run['layout_id'], gpu, steps))
        return record
    replay.calls = []
    return replay


class Router:
    def __init__(self, seen):
        self.seen = seen

    def prepare(self, payload):
        self.seen.append(payload)
        return payload, None

    def execute(self, payload, reservation):
        return {'message': {'content': 'The code left cup_qx7 off the sink.'}, 'cost_usd': 0.04}, None


@pytest.mark.parametrize('workers', [1, 2])
def test_runs_are_the_bundle_batch_by_layout(tmp_path, workers):
    rows = code_runs.runs(session(tmp_path, workers=workers))
    assert [(r['layout_id'], r['index'], r['episode_id'], r['score']) for r in rows] == [
        (0, 1, 'e0', 0.5), (1, 2, 'e1', 0.5), (2, 3, 'e2', 0.5)]  # Never the copies' own rows.
    assert all(r['sim'].is_dir() for r in rows)


def test_sessions_built_together_share_the_replay_slots_of_each_gpu(tmp_path, monkeypatch):
    import threading
    import time
    lock, active, peak = threading.Lock(), {}, {}

    def build(session, run, gpu, **_):
        with lock:
            active[gpu] = active.get(gpu, 0) + 1
            peak[gpu] = max(peak.get(gpu, 0), active[gpu])
        time.sleep(0.05)
        with lock:
            active[gpu] -= 1
        return {'status': 'ready', 'layout_id': run['layout_id']}

    monkeypatch.setattr(code_runs, 'build', build)
    monkeypatch.setattr(code_runs, 'runs', lambda session: [{'layout_id': i} for i in range(3)])
    sessions = [session(tmp_path/name) for name in 'abc']
    assert code_runs.main([*map(str, sessions), '--gpus', '0,1', '--per-gpu', '1']) == 0
    assert peak == {'0': 1, '1': 1}  # Three sessions, one replay at a time on each GPU.


def test_key_steps_are_evenly_spaced_call_ends(tmp_path):
    trace(tmp_path/'t.jsonl', [5, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 400])
    assert code_runs.key_steps(tmp_path/'t.jsonl', 400, count=4) == [5, 40, 70, 110]
    assert code_runs.key_steps(tmp_path/'t.jsonl', 400, count=20) == [5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110]
    assert code_runs.key_steps(tmp_path/'missing', 400) == []


def test_a_report_holds_the_review_frames_and_log(tmp_path, monkeypatch):
    root = session(tmp_path)
    replay = fake_replay()
    monkeypatch.setattr(code_runs, 'replay', replay)
    seen = []
    record = code_runs.build(root, code_runs.runs(root)[1], 'GPU-1', router_factory=lambda: Router(seen),
                             instruction='Sort.', total=3)
    assert record['status'] == 'ready', record.get('traceback')
    assert replay.calls == [(1, 'GPU-1', [10, 60, 120, 170, 230, 280, 340, 390])]
    report = root/'code-run-reports/layout-001/report'
    assert sorted(p.name for p in report.iterdir()) == [
        'README.md', 'code_log.txt', 'final_cam_head.png', 'final_cam_left_wrist.png', 'final_cam_right_wrist.png',
        'timeline.png']
    text = (report/'README.md').read_text()
    assert text.startswith('# Run 2 of 3: your code on its own')
    assert 'Partial score: 0.500 (1 newly satisfied of the 2 goal conditions' in text
    assert 'the replay matched the run' in text and 'inside cup 1 cabinet 1' in text
    assert 'cup 1 off the sink' in text and 'cup_qx7' not in text  # Scene names become task names.
    assert (report/'code_log.txt').read_text() == 'door attempt 0 failed\n'
    with Image.open(report/'timeline.png') as sheet:
        assert sheet.size == (4 * 64, 2 * 48)
    # The reviewer reads the code's log and the replayed frames, with the score.
    user = seen[0]['messages'][1]['content']
    assert "The code's own log:\ndoor attempt 0 failed" in user[0]['text'] and len(user) == 1 + 4
    assert 'Partial score: 0.500' in user[0]['text'] and 'its own, unattended' in seen[0]['messages'][0]['content']
    usage = [json.loads(line) for line in (root/'operator-analysis/usage.jsonl').read_text().splitlines()]
    assert usage[-1]['kind'] == 'code_run' and usage[-1]['cost_usd'] == 0.04
    # A ready report is kept as it is.
    monkeypatch.setattr(code_runs, 'replay', lambda *a, **k: pytest.fail('replayed again'))
    assert code_runs.build(root, code_runs.runs(root)[1], 'GPU-1', router_factory=lambda: Router(seen))['status'] == 'ready'


@pytest.mark.parametrize('diverged, matched, note', [
    (200, False, 'drifted from the run at step 200'),
    (None, False, 'final goal conditions differ'),
])
def test_frames_a_replay_cannot_vouch_for_are_left_out(tmp_path, monkeypatch, diverged, matched, note):
    root = session(tmp_path)
    monkeypatch.setattr(code_runs, 'replay', fake_replay(diverged, matched))
    record = code_runs.build(root, code_runs.runs(root)[0], 'GPU-0', router_factory=lambda: Router([]), total=3)
    report = root/'code-run-reports/layout-000/report'
    assert record['status'] == 'ready' and note in (report/'README.md').read_text()
    assert not list(report.glob('final_*'))
    record = json.loads((root/'code-run-reports/layout-000/replay/replay.json').read_text())
    kept, _ = code_runs.faithful_frames(record)
    assert kept and all(f['step'] < (diverged or 401) and not f['path'].startswith('frames/final_') for f in kept)


def test_a_failed_report_records_its_whole_error(tmp_path, monkeypatch):
    root = session(tmp_path)
    monkeypatch.setattr(code_runs, 'replay', fake_replay(compared=10))
    record = code_runs.build(root, code_runs.runs(root)[0], 'GPU-0', router_factory=lambda: Router([]))
    assert record['status'] == 'error' and 'only 10 of 400 replayed steps' in record['error']
    assert 'Traceback' in record['traceback'] and not (root/'code-run-reports/layout-000/report').exists()

    def crash(run, output, gpu, *, steps):
        raise RuntimeError('replay exited with code 1; log tail:\nPhysX failed')
    monkeypatch.setattr(code_runs, 'replay', crash)
    record = code_runs.build(root, code_runs.runs(root)[1], 'GPU-0', router_factory=lambda: Router([]))
    assert record['error'] == 'RuntimeError: replay exited with code 1; log tail:\nPhysX failed'


def test_a_copy_gets_every_other_layouts_report(tmp_path, monkeypatch):
    root = session(tmp_path)
    monkeypatch.setattr(code_runs, 'replay', fake_replay())
    for run in code_runs.runs(root):
        code_runs.build(root, run, 'GPU-0', router_factory=lambda: Router([]), total=3)
    workspace = tmp_path/'workspace'
    assert code_runs.place(root, workspace, layout_id=1) == [0, 2]
    folder = workspace/code_runs.FOLDER
    assert sorted(p.name for p in folder.iterdir()) == ['INDEX.md', 'run-01', 'run-03']  # Never its own layout.
    assert '`run-01/`: partial score 0.500' in (folder/'INDEX.md').read_text()
    assert (folder/'run-03/README.md').stat().st_mode & 0o777 == 0o644
    assert code_runs.place(tmp_path/'live-copy-session', tmp_path/'other', layout_id=0) == []
    assert not (tmp_path/'other').exists()


@pytest.mark.parametrize('live_copy_session', [True, False])
def test_the_copy_prompt_names_the_reports_only_when_it_has_some(live_copy_session):
    config = SimpleNamespace(agent_formal=live_copy_session)
    text = agent_runner.episode_prompt(config, ['memory'], 9)
    assert ('code_run_reports/ has reports on 9 runs of your code' in text) is not live_copy_session
    # A code-mode explorer learned its runs end at the sandbox's artifact limit; its copy is told that
    # limit is gone.
    assert ('This episode has no recording cap' in text) is not live_copy_session
    assert 'code_run_reports' not in agent_runner.episode_prompt(config, ['memory'], 0)
    assert not any(word in text for word in ('host', 'score', 'held-out', 'formal', 'submit'))


def test_post_hoc_agent_formal_builds_the_reports_first(tmp_path, monkeypatch):
    from services.agent_formal import checkpoint
    from services.controller.supervisor import Supervisor
    from test_controller_backend import Backend, Runner
    from test_formal_batch import config
    c = config(tmp_path, formal_episodes=2)
    object.__setattr__(c, 'benchmark', 'behavior')
    s = Supervisor(c, Backend, runner_factory=Runner)
    s.state.update(submission={'bundle': 'b'}, formal_batch={'status': 'completed'},
                   exploration_checkpoint={'bundle': 'frozen', 'session_id': 'thread'})
    s._save()
    s.close()
    (c.root/'exploration-complete.json').write_text('{}')
    order = []
    monkeypatch.setattr(code_runs, 'prepare', lambda root, gpus: order.append(('reports', root, gpus)) or [
        {'layout_id': 0, 'status': 'ready'}, {'layout_id': 1, 'status': 'error', 'error': 'RuntimeError: boom'}])
    monkeypatch.setattr(checkpoint, 'agent_formal',
                        lambda config, supervisor, deployed: order.append('copies') or {'status': 'completed'})
    summary = checkpoint.after_exploration(c, {}, live_copies=True)
    assert order == [('reports', c.root, [c.sim_gpu]), 'copies']
    assert summary['code_run_reports'] == {'ready': 1, 'errors': {1: 'RuntimeError: boom'}}
    assert json.loads((c.root/'formal.json').read_text())['code_run_reports']['errors'] == {'1': 'RuntimeError: boom'}


def test_a_replay_writes_where_it_was_asked_from_any_directory(tmp_path, monkeypatch):
    """The simulator runs inside the replay directory; a relative session path must still land there."""
    import subprocess
    from services.behavior import process
    seen = {}
    def run(command, *, cwd, env, stdout, stderr, timeout):
        output = Path(command[command.index('--output') + 1])
        seen.update(output=output, cwd=cwd)
        (cwd/output/'replay.json').write_text('{"frames": []}')  # Where the child resolves it.
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(subprocess, 'run', run)
    monkeypatch.setattr(process, 'simulator_environment', lambda gpu, appdata: {'XDG_RUNTIME_DIR': str(tmp_path/'xdg')})
    monkeypatch.setattr(process, 'runtime_root', lambda: tmp_path/'runtime')
    monkeypatch.chdir(tmp_path)
    assert code_runs.replay({'sim': 'sim'}, Path('session/code-run-reports/layout-000/replay'), 'GPU-0',
                            steps=[1]) == {'frames': []}
    assert seen['output'].is_absolute() and seen['cwd'] == seen['output']


def test_prepare_builds_the_chosen_layouts_numbered_among_all(tmp_path, monkeypatch):
    root = session(tmp_path)
    monkeypatch.setattr(code_runs, 'replay', fake_replay())
    monkeypatch.setattr(code_runs.reviews, '_router', lambda: Router([]))
    monkeypatch.setattr(code_runs, 'build', lambda session, run, gpu, *, router_factory=code_runs.reviews._router,
                        instruction='', total=10, _build=code_runs.build: _build(
                            session, run, gpu, router_factory=lambda: Router([]), instruction=instruction, total=total))
    records = code_runs.prepare(root, ['GPU-0'], layouts={1}, log=lambda line: None)
    assert [r['layout_id'] for r in records] == [1] and records[0]['status'] == 'ready'
    assert (root/'code-run-reports/layout-001/report/README.md').read_text().startswith('# Run 2 of 3:')


def test_a_rebuilt_report_reuses_a_finished_replay(tmp_path, monkeypatch):
    root = session(tmp_path)
    replay = fake_replay()
    monkeypatch.setattr(code_runs, 'replay', replay)
    class Down(Router):
        def execute(self, payload, reservation):
            raise RuntimeError('HTTP 503')
    run = code_runs.runs(root)[0]
    assert code_runs.build(root, run, 'GPU-0', router_factory=lambda: Down([]))['error'] == 'RuntimeError: HTTP 503'
    record = code_runs.build(root, run, 'GPU-0', router_factory=lambda: Router([]), total=3)
    assert record['status'] == 'ready' and record['attempts'] == 2 and len(replay.calls) == 1
    assert sorted(p.name for p in (root/'code-run-reports/layout-000/report').glob('final_*')) == [
        'final_cam_head.png', 'final_cam_left_wrist.png', 'final_cam_right_wrist.png']
