"""Reports on a code-mode session's runs of its own code, for its later live copies.

A code-mode session's formal batch ran the agent's submitted code alone on the held-out
layouts. When live copies play the session afterwards (``--resume --agent-formal``), each of
those runs has a report so the copies can see how their code behaved:

- camera frames from a replay of the run's recorded actions (``services/behavior/replay.py``;
  the run's own frames may have been pruned), checked against its recorded base poses and
  final goal state; frames a diverged replay cannot vouch for are left out and said so;
- a detailed operator-side review (Gemini, the session's review ledger) from the simulator's
  goal report, the code's tool calls and log, and those frames;
- the code's own log.

A copy gets the reports of every other layout in ``code_run_reports/``, never the one of the
layout it plays. Host records: ``<session>/code-run-reports/layout-NNN/`` holds ``replay/``,
``record.json`` and ``report/`` (exactly what a copy sees).
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from services.behavior import analysis as A
from services.behavior import reviews
from services.dashboard import review

DIRECTORY = 'code-run-reports'
FOLDER = 'code_run_reports'  # In a copy's workspace.
CAMERAS = ('cam_head', 'cam_left_wrist', 'cam_right_wrist')
TIMELINE_FRAMES = 8
LOG_BYTES = 200_000  # Of the code's log a copy gets; the reviewer reads its last REVIEW_LOG_BYTES.
REVIEW_LOG_BYTES = 60_000
REPLAY_SECONDS = 6 * 3600


def layout_dir(session, layout_id):
    return Path(session)/DIRECTORY/f'layout-{int(layout_id):03d}'


def runs(session):
    """The formal batch's finished runs of the submitted code, by layout."""
    session = Path(session)
    state = json.loads((session/'state.json').read_text())
    # Bundle formal episodes run in formal-workers/, or in the session itself with one formal worker.
    episodes = {row['episode_id']: row for row in review.episodes(session)
                if row.get('episode_id') and row['mode'] != 'agent_formal'}
    found = {}
    for row in state.get('results', []):
        if row.get('mode') != 'formal' or row.get('driver') == 'agent' or row.get('episode_id') not in episodes:
            continue
        episode = episodes[row['episode_id']]
        found[row['layout_id']] = {
            'layout_id': row['layout_id'], 'index': row['formal_episode_index'], 'episode_id': row['episode_id'],
            'sim': session/episode['episode'], 'score': episode['native_score'], 'artifacts': row.get('artifacts') or {}}
    return [found[key] for key in sorted(found)]


def _published(session, logical):
    return reviews._published(session, logical) if logical else None


def key_steps(trace_path, final, count=TIMELINE_FRAMES):
    """Steps at evenly spaced ends of the code's tool calls, for the timeline."""
    steps = set()
    try:
        lines = Path(trace_path).read_text().splitlines()
    except (OSError, TypeError):
        lines = []
    for line in lines:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        value = event.get('returned') if event.get('event') == 'mcp_result' else None
        step = value.get('step_id') if isinstance(value, dict) else None
        if isinstance(step, int) and 0 < step < final:
            steps.add(step)
    steps = sorted(steps)
    if len(steps) <= count:
        return steps
    return sorted({steps[round(i * (len(steps) - 1) / (count - 1))] for i in range(count)})


def replay(run, output, gpu, *, steps):
    """Replay one run in a simulator on ``gpu``; returns its replay.json record."""
    from services.behavior.process import AppdataSlot, behavior_python, runtime_root, simulator_environment
    output = Path(output).resolve()  # The simulator runs from inside it.
    if output.exists():
        shutil.rmtree(output)  # A partial earlier replay of this run; it is replayed whole.
    output.mkdir(parents=True)
    slot = AppdataSlot(runtime_root()/'behavior-cache'/'appdata')
    appdata = slot.acquire()
    try:
        env = simulator_environment(gpu, appdata)
        Path(env['XDG_RUNTIME_DIR']).mkdir(mode=0o700, parents=True, exist_ok=True)
        log = output/'replay.log'
        with log.open('w') as stream:
            process = subprocess.run(
                [behavior_python(), '-m', 'services.behavior.replay', '--sim', str(Path(run['sim']).resolve()),
                 '--output', str(output), '--steps', ','.join(map(str, steps))],
                cwd=output, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=REPLAY_SECONDS)
    finally:
        slot.release()
    if process.returncode or not (output/'replay.json').is_file():
        tail = log.read_text(errors='replace')[-4000:]
        raise RuntimeError(f'replay exited with code {process.returncode}; log tail:\n{tail}')
    return json.loads((output/'replay.json').read_text())


def faithful_frames(record):
    """(frames a copy may see, a note on them): only frames the replay can vouch for."""
    diverged, matched = record['diverged_at_step'], record['goal_conditions_match']
    if record['compared_steps'] < record['replayed_steps']:
        # A step without a recorded base pose cannot be checked; treat the replay as drifted there.
        raise RuntimeError(f"only {record['compared_steps']} of {record['replayed_steps']} replayed steps have a "
                           'recorded base pose to compare')
    frames = [f for f in record['frames'] if diverged is None or f['step'] < diverged]
    if diverged is None and matched:
        return frames, ('Frames are from a replay of this run\'s recorded actions; the replay matched the run '
                        f'(base within {100 * record["diverged_threshold_m"]:.0f} cm at every step, same final goal '
                        'conditions).')
    finals = [f for f in record['frames'] if f['path'].startswith('frames/final_')]
    frames = [f for f in frames if f not in finals]
    if diverged is not None:
        return frames, (f'Frames are from a replay of this run\'s recorded actions. The replay drifted from the run '
                        f'at step {diverged} (base more than {100 * record["diverged_threshold_m"]:.0f} cm away), so '
                        'later frames, including the final ones, are left out.')
    return frames, ('Frames are from a replay of this run\'s recorded actions. The replayed base followed the run, '
                    'but its final goal conditions differ from the run\'s, so the final frames are left out and later '
                    'frames may not show the run exactly.')


def timeline(frames, root, path):
    """One sheet of the head-camera frames, labelled by step."""
    from PIL import Image, ImageDraw
    heads = [f for f in frames if f['camera'] == 'cam_head' and not f['path'].startswith('frames/final_')]
    if not heads:
        return False
    tiles = []
    for frame in heads:
        with Image.open(Path(root)/frame['path']) as image:
            tile = image.convert('RGB')
        tile.thumbnail((360, 360))
        ImageDraw.Draw(tile).text((6, 6), f'step {frame["step"]}', fill=(255, 255, 0))
        tiles.append(tile)
    columns = min(4, len(tiles))
    rows = -(-len(tiles) // columns)
    width, height = tiles[0].size
    sheet = Image.new('RGB', (columns * width, rows * height))
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % columns) * width, (i // columns) * height))
    sheet.save(path)
    return True


def code_log(session, artifacts):
    parts = []
    for key in ('stdout', 'stderr'):
        path = _published(session, artifacts.get(key))
        text = path.read_text(errors='replace') if path and path.is_file() else ''
        if text:
            parts.append(text if key == 'stdout' else '[stderr]\n' + text)
    return '\n'.join(parts)


def readme(run, report, explanation, *, score, total, frames_note, files):
    lines = [f'# Run {run["index"]} of {total}: your code on its own', '',
             f'Episode: `{report["episode_id"]}`', '', A.score_text(report, score), '',
             frames_note, '', 'Files: ' + ', '.join(files) + '.', '',
             '## Goal conditions at the end (ground truth)', '', A.report_text(report), '',
             '## Analyst review', '', A.agent_safe_text(explanation, report, detailed=True).strip()]
    return '\n'.join(lines) + '\n'


def build(session, run, gpu, *, router_factory=reviews._router, instruction='', total=10):
    """Replay, review and write one run's report; returns its record (errors recorded, not raised)."""
    session = Path(session)
    root = layout_dir(session, run['layout_id'])
    root.mkdir(parents=True, exist_ok=True)
    path = root/'record.json'
    previous = reviews._read(path)
    if previous.get('status') == 'ready':
        return previous
    record = {'layout_id': run['layout_id'], 'index': run['index'], 'episode_id': run['episode_id'],
              'score': run['score'], 'started_at': time.time(), 'attempts': previous.get('attempts', 0) + 1}
    try:
        report = A.ground_truth_report(run['sim'])
        if report is None:
            raise RuntimeError(f'the simulator wrote no goal report in {run["sim"]}')
        trace_path = _published(session, run['artifacts'].get('mcp_trace'))
        if (root/'replay'/'replay.json').is_file():  # Written last, so the earlier replay finished.
            replayed = json.loads((root/'replay'/'replay.json').read_text())
        else:
            replayed = replay(run, root/'replay', gpu, steps=key_steps(trace_path, report['steps']))
        record['replay'] = {k: replayed[k] for k in ('replayed_steps', 'recorded_steps', 'compared_steps',
                                                     'diverged_at_step', 'worst_base_error', 'goal_conditions_match')}
        reviews._write(path, {**record, 'status': 'reviewing'})
        frames, frames_note = faithful_frames(replayed)
        out = root/'report.tmp'
        if out.exists():
            shutil.rmtree(out)
        out.mkdir()
        files = []
        for frame in frames:
            if frame['path'].startswith('frames/final_'):
                name = f'final_{frame["camera"]}.png'
                shutil.copyfile(root/'replay'/frame['path'], out/name)
                files.append(name)
        if timeline(frames, root/'replay', out/'timeline.png'):
            files.append('timeline.png (head camera at the labelled steps)')
        log = code_log(session, run['artifacts'])
        (out/'code_log.txt').write_text(log[-LOG_BYTES:] if log else '(the code printed nothing)\n')
        files.append('code_log.txt (your code\'s own output in this run'
                     + (f', its last {LOG_BYTES // 1000} kB' if len(log) > LOG_BYTES else '') + ')')
        if reviews.spent(session) >= A.SESSION_BUDGET_USD:
            raise RuntimeError(f'the session review budget of ${A.SESSION_BUDGET_USD:g} is spent')
        images = A.rgb_images([out/f'final_{cam}.png' for cam in CAMERAS] + [out/'timeline.png'], limit=4)
        note = (('The images are ' + ('the robot\'s final head and wrist camera frames and ' if any(
            f.startswith('final_') for f in files) else '') + 'a timeline sheet of head-camera frames labelled by '
            'step, replayed from the run\'s recorded actions.') if images else 'No camera frames are available.')
        explanation, cost = A.explain(router_factory(), report, A.condensed_trace(trace_path) if trace_path else [],
                                      images, task_instruction=instruction, detailed=True, score=run['score'],
                                      code_log=log[-REVIEW_LOG_BYTES:], images_note=note)
        with (reviews.analysis_dir(session)/'usage.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'time': time.time(), 'episode_id': run['episode_id'], 'kind': 'code_run',
                                     'cost_usd': cost}) + '\n')
        (out/'README.md').write_text(readme(run, report, explanation, score=run['score'], total=total,
                                            frames_note=frames_note, files=files))
        for item in out.rglob('*'):
            item.chmod(0o644)
        if (root/'report').exists():
            shutil.rmtree(root/'report')
        out.rename(root/'report')
        record.update(status='ready', explanation=explanation, cost_usd=cost, frames_note=frames_note,
                      files=sorted(p.name for p in (root/'report').iterdir()), finished_at=time.time())
    except Exception as exc:  # noqa: BLE001 - recorded whole; the copies get the reports that exist
        import traceback
        record.update(status='error', error=f'{type(exc).__name__}: {exc}',
                      traceback=traceback.format_exc()[-4000:], finished_at=time.time())
    reviews._write(path, record)
    return record


def gpu_slots(gpus, per_gpu=1):
    """Replay slots, ``per_gpu`` on each GPU; every session given the same slots shares those GPUs."""
    slots = queue.Queue()
    for gpu in gpus:
        for _ in range(per_gpu):
            slots.put(gpu)
    return slots


def prepare(session, gpus, *, per_gpu=1, layouts=None, log=print, slots=None):
    """Build every missing report of a session's runs (or of ``layouts``), ``per_gpu`` replays at a
    time on each GPU; ``slots`` (gpu_slots) are shared with other sessions when given."""
    from services.behavior.tasks import task_info
    session = Path(session)
    everything = runs(session)
    found = [run for run in everything if layouts is None or run['layout_id'] in layouts]
    task = json.loads((session/'state.json').read_text())['configuration']['task']
    instruction = (task_info(task) or {}).get('instruction', '')
    slots = gpu_slots(gpus, per_gpu) if slots is None else slots

    def one(run):
        gpu = slots.get()
        try:
            record = build(session, run, gpu, instruction=instruction, total=len(everything))
        finally:
            slots.put(gpu)
        log(f'{session.name} layout {run["layout_id"]}: {record["status"]}'
            + (f' | replay {record.get("replay")}' if record.get('replay') else '')
            + (f' | {record["error"]}' if record.get('error') else ''))
        return record

    with ThreadPoolExecutor(max(1, len(gpus) * per_gpu)) as pool:
        return list(pool.map(one, found))


def place(session, workspace, *, layout_id):
    """Copy every other layout's ready report into a copy's workspace; returns the layouts given."""
    if layout_id is None and (Path(session)/DIRECTORY).is_dir():
        raise ValueError('A copy needs its layout to be given the reports of the other layouts')
    given, rows = [], []
    for root in sorted((Path(session)/DIRECTORY).glob('layout-*')):
        record = reviews._read(root/'record.json')
        if record.get('status') != 'ready' or record.get('layout_id') == layout_id or not (root/'report').is_dir():
            continue
        target = Path(workspace)/FOLDER/f'run-{record["index"]:02d}'
        shutil.copytree(root/'report', target)
        for path in [target, *target.rglob('*')]:
            path.chmod(0o755 if path.is_dir() else 0o644)
        given.append(record['layout_id'])
        score = 'unavailable' if record.get('score') is None else f'{record["score"]:.3f}'
        rows.append(f'- `run-{record["index"]:02d}/`: partial score {score}')
    if given:
        index = Path(workspace)/FOLDER/'INDEX.md'
        index.write_text('# Runs of your code on its own, on other layouts of this task\n\n'
                         'Each folder has README.md (the run\'s goal conditions at the end and an analyst\'s review '
                         'of what your code did), key camera frames and your code\'s log.\n\n' + '\n'.join(rows) + '\n')
        index.chmod(0o644)
        (Path(workspace)/FOLDER).chmod(0o755)
    return given


def main(argv=None):
    parser = argparse.ArgumentParser(description='Build the code-run reports of finished code-mode sessions.')
    parser.add_argument('sessions', nargs='+')
    parser.add_argument('--gpus', required=True, help='comma-separated simulator GPUs')
    parser.add_argument('--per-gpu', type=int, default=1)
    parser.add_argument('--layouts', default='', help='comma-separated layout ids (default: all)')
    args = parser.parse_args(argv)
    gpus = [g for g in args.gpus.split(',') if g]
    layouts = {int(x) for x in args.layouts.split(',') if x} or None

    def log(line):
        print(time.strftime('%m-%d %H:%M:%S'), line, flush=True)

    slots = gpu_slots(gpus, args.per_gpu)  # One set for all sessions: --per-gpu bounds each GPU's replays.
    with ThreadPoolExecutor(len(args.sessions)) as pool:
        results = list(pool.map(lambda s: prepare(s, gpus, per_gpu=args.per_gpu, layouts=layouts, log=log,
                                                  slots=slots), args.sessions))
    failed = [r for rows in results for r in rows if r['status'] != 'ready']
    log(f'{sum(len(r) for r in results) - len(failed)} ready, {len(failed)} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
