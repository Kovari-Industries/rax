"""Real bundle sandbox (bubblewrap) checks: isolation, the MCP loop, outputs and limits."""
import json
import subprocess
import os
from pathlib import Path
import shutil
import tempfile

import pytest

from services.controller.config import Limits
from services.controller.gateway import Gateway
from services.controller.sandbox import BundleSandbox as _BundleSandbox
from services.controller.storage import snapshot


# The repository's BEHAVIOR-1K checkout where it really is (the link's target): never in a sandbox.
BEHAVIOR_CHECKOUT = (Path(__file__).resolve().parents[2] / 'BEHAVIOR-1K').resolve()


class BundleSandbox(_BundleSandbox):
    """Runs at a workspace path that does not exist on the host, like a formal worker's."""
    def run(self, *args, **kwargs):
        if kwargs.get('gateway') is None:
            class Backend:
                def tools(self):
                    return []
            gateway = Gateway(Backend(), None, audit=lambda _: None, charge=lambda *_: None,
                              refund=lambda *_: None, budget=lambda: {})
            kwargs.update(gateway=gateway, token=gateway.acquire())
        kwargs.setdefault('workspace', kwargs['output'].parent/'agent-workspace')
        return super().run(*args, **kwargs)


@pytest.fixture
def sandbox_runtime(host_artifact_root):
    """A real session directory under the host's artifact root, as production creates them."""
    if not shutil.which('bwrap'):
        pytest.skip('bubblewrap required')
    root = (Path(__file__).resolve().parents[2] / 'runtime').resolve()
    if not root.is_relative_to(host_artifact_root) or not os.access(root, os.W_OK):
        pytest.skip(f'Writable runtime/ under the artifact root {host_artifact_root} required')
    parent = root / 'integration-tests'
    parent.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='bundle-sandbox-', dir=parent))
    yield output
    shutil.rmtree(output, ignore_errors=True)


def test_actual_sandbox_quarantine_and_artifact_export(sandbox_runtime):
    output = sandbox_runtime
    src = output / "source"
    src.mkdir()
    (src / "controller.py").write_text('''
import json, os, socket, importlib.util, subprocess, sys
from pathlib import Path
def main(ctx):
    assert Path.cwd().name == "agent-workspace" and Path("code/controller/controller.py").is_file()
    assert not Path("/var/run/docker.sock").exists() and not Path("/run/docker.sock").exists()
    assert not Path("/home").exists() and not Path(BEHAVIOR_CHECKOUT).exists()
    assert not Path("/dev/nvidia0").exists() and not Path("/dev/nvidiactl").exists()  # No GPU.
    assert not os.environ.get("OPENROUTER_API_KEY") and set(os.environ) >= {"HOME", "PYTHONPATH", "OMP_NUM_THREADS"}
    assert not [p for p in Path("/proc").iterdir() if p.name.isdigit() and int(p.name) > 20]  # Own PID namespace.
    print("python-print-checkpoint")
    print("python-stderr-checkpoint", file=sys.stderr)
    subprocess.run([sys.executable, "-c", "print('child-print-checkpoint')"], check=True)
    try:
        Path("api/runtime.py").write_text("modified")
        raise AssertionError("SDK was writable")
    except OSError:
        pass
    for address in [("1.1.1.1", 443), ("127.0.0.1", 80)]:
        try:
            connection = socket.create_connection(address, timeout=0.2)
        except OSError:
            continue
        connection.close()
        raise AssertionError("Network was reachable")
    Path("output/report.json").write_text(json.dumps({"isolated": True}))
'''.replace('BEHAVIOR_CHECKOUT', repr(str(BEHAVIOR_CHECKOUT))))
    snapshot(src, output / "bundle", max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    runner.preflight()
    result = runner.run(output / "bundle", mode="rehearsal", entrypoint="controller.py", output=output / "run")
    assert result["returncode"] == 0, (output / "run/stderr.log").read_text()
    assert json.loads((output / "run/files/report.json").read_text()) == {"isolated": True}
    assert "python-print-checkpoint" in (output / "run/stdout.log").read_text()
    assert "python-stderr-checkpoint" in (output / "run/stderr.log").read_text()
    assert "child-print-checkpoint" in (output / "run/stderr.log").read_text()
    assert not result["logs"]["stdout"]["truncated"]


def test_actual_sandbox_mcp_closed_loop(sandbox_runtime):
    output = sandbox_runtime
    src = output / "source"
    src.mkdir()
    (src / "controller.py").write_text('''
import json
import subprocess
def main(ctx):
    def data(reply):
        return json.loads(reply["content"][0]["text"])
    for i in range(3):
        state = data(ctx.call("robodojo_observe"))
        assert state["step_id"] == i
        next_state = data(ctx.call("robodojo_step", actions=[[0.0]*14]))
        assert next_state["step_id"] == i+1
''')
    class Backend:
        step = 0
        def call(self, name, arguments):
            if name == "robodojo_step":
                self.step += 1
            return {"step_id": self.step}, []
    backend = Backend()
    gateway = Gateway(backend, None, audit=lambda e: None, charge=lambda *_: None,
                      refund=lambda *args: None, budget=lambda: {})
    token = gateway.acquire()
    snapshot(src, output / "bundle", max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output / "bundle", mode="rehearsal", entrypoint="controller.py",
                        gateway=gateway, token=token, output=output / "run")
    assert result["returncode"] == 0, (output / "run/stderr.log").read_text()
    assert backend.step == 3


def test_local_png_to_gemini_through_real_sandbox(sandbox_runtime):
    from PIL import Image
    from services.controller.gemini import GeminiRouter, MODEL
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    Image.frombytes('RGB', (768, 768), os.urandom(768*768*3)).save(src/'crop.png')
    raw = (src/'crop.png').read_bytes()
    assert len(raw) > 1024*1024
    (src/'controller.py').write_text('''
import json
import subprocess
from pathlib import Path
def main(ctx):
    import base64
    raw = Path('code/controller/crop.png').read_bytes()
    from_file = {'type': 'image', 'mimeType': 'image/png', 'data': base64.b64encode(raw).decode()}
    result = ctx.call('gemini_generate', messages=[{'role': 'user', 'content': [from_file]}])
    value = json.loads(result['content'][0]['text'])
    assert value['message']['content'] == 'received-local-image'
    assert value['usage']['total_tokens'] == 10
    print('local-image-roundtrip-ok')
''')
    captured, charged = [], []
    def send(payload):
        captured.append(payload)
        return {'model': MODEL, 'choices': [{'message': {'content': 'received-local-image'}}],
                'usage': {'total_tokens': 10}}
    gateway = Gateway(None, GeminiRouter('host-test-key', send=send),
        audit=lambda e: None, charge=lambda amount, cost: charged.append(amount), refund=lambda *args: None,
        budget=lambda: {'calls_used': len(charged)})
    token = gateway.acquire()
    snapshot(src, output/'bundle', max_bytes=4*1024*1024)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py',
                        gateway=gateway, token=token, output=output/'run')
    assert result['returncode'] == 0, (output/'run/stderr.log').read_text()
    import base64
    sent = captured[0]['messages'][0]['content'][0]['image_url']['url']
    assert base64.b64decode(sent.split(',', 1)[1]) == raw
    assert len(charged) == 1
    assert 'local-image-roundtrip-ok' in (output/'run/stdout.log').read_text()


def test_function_call_roundtrip_through_isolated_python(sandbox_runtime):
    from services.controller.gemini import GeminiRouter, MODEL
    from test_gemini_contract import ASSISTANT, TOOLS
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    (src/'tools.json').write_text(json.dumps(TOOLS))
    (src/'controller.py').write_text('''
import json
import subprocess
from pathlib import Path
def main(ctx):
    tools = json.loads(Path(__file__).with_name('tools.json').read_text())
    messages = [{'role':'user', 'content':'Use add to compute 2+3.'}]
    def generate(**options):
        reply = ctx.call('gemini_generate', messages=messages, tools=tools, **options)
        return json.loads(reply['content'][0]['text'])
    first = generate(tool_choice='required', reasoning={'effort':'low'})
    messages.append(first['message'])
    for call in first['message']['tool_calls']:
        assert call['function']['name'] == 'add'
        args = json.loads(call['function']['arguments'])
        messages.append({'role':'tool', 'tool_call_id':call['id'],
                         'content':json.dumps({'answer':args['a']+args['b']})})
    final = generate(tool_choice='none', response_format={'type':'json_object'})
    assert json.loads(final['message']['content']) == {'answer':5}
    assert final['budget']['calls_used'] == 2
    Path('output/result.json').write_text(json.dumps(final))
''')
    sent, charged = [], []
    def send(payload):
        sent.append(payload)
        if len(sent) == 2:
            assert payload['messages'][1] == ASSISTANT
            assert json.loads(payload['messages'][2]['content']) == {'answer': 5}
        return {'model': MODEL, 'choices': [{'message': ASSISTANT if len(sent) == 1 else
                {'role': 'assistant', 'content': '{"answer":5}'},
                'finish_reason': 'tool_calls' if len(sent) == 1 else 'stop'}], 'usage': {'total_tokens': 10}}
    gateway = Gateway(None, GeminiRouter('host-only-key', send=send), audit=lambda _: None,
        charge=lambda amount, cost: charged.append(amount), refund=lambda *_: None, budget=lambda: {'calls_used': len(charged)})
    token = gateway.acquire()
    snapshot(src, output/'bundle', max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py',
                        gateway=gateway, token=token, output=output/'run')
    assert result['returncode'] == 0, (output/'run/stderr.log').read_text()
    assert len(sent) == 2 and 'host-only-key' not in (output/'run/files/result.json').read_text()


def test_terminal_rehearsal_copies_files_but_denies_further_calls(sandbox_runtime):
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    (src/'controller.py').write_text('''
from pathlib import Path
def main(ctx):
    ctx.call('robodojo_step', actions=[[0.0]*14])
    for tool in ('robodojo_step', 'robodojo_observe', 'gemini_generate'):
        try:
            ctx.call(tool)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Environment/API still accessible after terminal')
    Path('output/final.txt').write_text('terminal saved')
''')
    class Backend:
        calls = []
        def call(self, name, arguments):
            self.calls.append(name)
            return {'episode_ended': True}, []
    backend = Backend()
    gateway = Gateway(backend, None, audit=lambda e: None, charge=lambda *_: None,
                      refund=lambda *args: None, budget=lambda: {})
    token = gateway.acquire()
    snapshot(src, output/'bundle', max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py',
                        gateway=gateway, token=token, output=output/'run')
    assert result['reason'] == 'episode_ended' and result['returncode'] == 0
    assert backend.calls == ['robodojo_step']
    assert (output/'run/files/final.txt').read_text() == 'terminal saved'


def test_actual_sandbox_timeout_kills_descendants(sandbox_runtime):
    output = sandbox_runtime
    src = output / "source"
    src.mkdir()
    (src / "controller.py").write_text("import subprocess, time\ndef main(ctx):\n    subprocess.Popen(['sleep','300'])\n    time.sleep(300)\n")
    snapshot(src, output / "bundle", max_bytes=100000)
    runner = BundleSandbox(Limits(3, 128, 1, 32, 16, 100000))
    result = runner.run(output / "bundle", mode="rehearsal", entrypoint="controller.py", output=output / "run")
    assert result["reason"] == "timeout"
    assert not (output/'run/writable').exists()


@pytest.mark.parametrize('unsafe', ['symlink', 'hardlink'])
def test_evaluation_outputs_reject_links_and_release_mount(sandbox_runtime, unsafe):
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    link = ('os.symlink("code/controller/controller.py", "output/link.py")' if unsafe == 'symlink'
            else 'Path("output/normal.txt").write_text("data"); os.link("output/normal.txt", "output/alias.txt")')
    (src/'controller.py').write_text('import os\nfrom pathlib import Path\ndef main(ctx):\n    '+link+'\n')
    snapshot(src, output/'bundle', max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py', output=output/'run')
    assert result['returncode'] == 0 and result['output_error']
    assert not (output/'run/files').exists()
    assert not (output/'run/writable').exists()


def test_evaluation_output_filesystem_enforces_byte_cap(sandbox_runtime):
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    (src/'controller.py').write_text('''
import errno
from pathlib import Path
def main(ctx):
    try:
        Path('output/oversized.bin').write_bytes(b'x'*(1024*1024))
    except OSError as error:
        assert error.errno == errno.EFBIG  # The sandbox's file size limit is the artifact cap.
        Path('output/oversized.bin').unlink()
        Path('output/recovered.txt').write_text('cap-enforced')
    else:
        raise AssertionError('Output cap not enforced')
''')
    snapshot(src, output/'bundle', max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 65536))
    result = runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py', output=output/'run')
    assert result['returncode'] == 0 and not result['output_error']
    assert (output/'run/files/recovered.txt').read_text() == 'cap-enforced'


def test_runaway_output_stops_the_controller(sandbox_runtime):
    output = sandbox_runtime
    src = output/'source'
    src.mkdir()
    (src/'controller.py').write_text("import time\nfrom pathlib import Path\ndef main(ctx):\n"
                                     "    for i in range(64):\n        Path(f'output/{i}.bin').write_bytes(b'x'*8192)\n"
                                     "    time.sleep(30)\n")
    snapshot(src, output/'bundle', max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 65536))
    with pytest.raises(ValueError, match='twice its 65536-byte limit'):
        runner.run(output/'bundle', mode='rehearsal', entrypoint='controller.py', output=output/'run')
    assert runner.execution_seconds < 10 and not (output/'run/writable').exists()


def test_actual_sandbox_crash_and_truncation_logs(sandbox_runtime):
    output = sandbox_runtime
    src = output / "source"
    src.mkdir()
    (src / "controller.py").write_text("def main(ctx):\n    print('checkpoint-before-crash')\n    print('x' * (1024 * 1024 + 10))\n    raise RuntimeError('intentional-test-error')\n")
    snapshot(src, output / "bundle", max_bytes=100000)
    runner = BundleSandbox(Limits(30, 128, 1, 32, 16, 100000))
    result = runner.run(output / "bundle", mode="rehearsal", entrypoint="controller.py", output=output / "run")
    assert result["returncode"] == 1
    assert (output / "run/stdout.log").read_text().startswith("checkpoint-before-crash")
    assert result["logs"]["stdout"]["truncated"] is True
    assert result["logs"]["stdout"]["bytes_saved"] == 1024 * 1024
    assert "Traceback" in (output / "run/stderr.log").read_text()
    assert "intentional-test-error" in (output / "run/stderr.log").read_text()
    assert "checkpoint-before-crash" not in json.dumps(result)


def test_actual_sandbox_rehearsal_and_formal(sandbox_runtime):
    """Real script lifecycle, deterministic fake task; no claimed robot success."""
    from services.controller.config import Configuration
    from services.controller.supervisor import Supervisor
    output = sandbox_runtime
    limits = Limits(30, 128, 1, 32, 16, 1000000)
    config = Configuration(output / 'trial', 'make_kong', (0, 1), 2, 0,
                           'GPU-aaaa', limits, limits, limits)
    class Backend:
        step = 0
        def start(self, **kwargs):
            self.step = 0
            return {}, []
        def call(self, name, arguments):
            if name == 'robodojo_step':
                self.step += 1
            return {'step_id': self.step, 'episode_id': 'fake-episode'}, []
        def evaluate(self):
            return {'task_complete': self.step == 3}
        def close(self):
            pass
    src = output / 'source'
    src.mkdir()
    (src / 'controller.py').write_text('''
import json
import subprocess
def main(ctx):
    for i in range(3):
        value = json.loads(ctx.call('robodojo_observe')['content'][0]['text'])
        assert value['step_id'] == i
        ctx.call('robodojo_step', actions=[[0.0]*14])
    print('full-script-completed')
''')
    supervisor = Supervisor(config, Backend)
    try:
        gateway, token = supervisor.start_interactive()
        for i in range(3):
            reply = gateway.handle(token, {'jsonrpc': '2.0', 'id': i, 'method': 'tools/call',
                                          'params': {'name': 'robodojo_step', 'arguments': {}}})
            assert 'error' not in reply
        assert supervisor.finish_interactive()['evaluation']['task_complete']
        bundle, _ = supervisor.register(src)
        for formal in (False, True):
            result = supervisor.run(bundle, formal=formal)
            assert result['returncode'] == 0 and result['task_complete']
            assert result['start_step_id'] == 0 and result['end_step_id'] == 3
            assert not result['recording_errors']
            timing = result['timing']
            assert timing['controller_processing_seconds'] > 0
            assert timing['environment_setup_seconds'] > 0
            assert timing['environment_step_seconds'] > 0
            assert timing['mcp_tools']['robodojo_step']['calls'] == 3
            assert timing['mcp_tools']['robodojo_observe']['calls'] == 3
            assert timing['container_overhead_seconds'] > 0
            components = [v for k, v in timing.items() if k.endswith('_seconds')
                          and k not in {'total_seconds', 'wall_limit_seconds'}]
            assert sum(components) == pytest.approx(timing['total_seconds'])
            assert supervisor.backend is None
        assert supervisor.state['exploration_started'] == 2
        with pytest.raises(RuntimeError, match='no retry'):
            supervisor.run(bundle, formal=True)
    finally:
        supervisor.close()


@pytest.mark.gpu
def test_actual_sandbox_native_closed_loop_rehearsal(sandbox_runtime, monkeypatch):
    """Bundle sandbox -> MCP -> Isaac observation/action, planner, preview and artifacts."""
    import sys
    from services.controller.backend import NativeBackend
    from services.controller.config import Configuration
    from services.controller.supervisor import Supervisor
    output = sandbox_runtime
    try:
        query = subprocess.run(["nvidia-smi", "--query-gpu=uuid,memory.free,utilization.gpu", "--format=csv,noheader,nounits"],
                               capture_output=True, text=True, timeout=10, check=True)
        candidates = []
        for row in query.stdout.splitlines():
            gpu, free, utilization = map(str.strip, row.split(","))
            if int(free) >= 24 * 1024 and int(utilization) < 20:
                candidates.append((int(free), gpu))
        if not candidates:
            pytest.skip("No idle GPU with 24 GiB free")
        gpu = max(candidates)[1]
        ready = subprocess.run([sys.executable, "-c", "import importlib.util; assert importlib.util.find_spec('isaaclab'); assert importlib.util.find_spec('isaacsim')"], capture_output=True)
        if ready.returncode:
            pytest.skip("Native Isaac dependencies unavailable")
    except (OSError, subprocess.SubprocessError):
        pytest.skip("GPU unavailable")
    # NativeBackend configures a dedicated-process environment. Restore the test
    # process environment afterward, as pytest hosts multiple independent tests.
    previous = dict(os.environ)
    limits = Limits(240, 256, 1, 32, 16, 256 * 1024 * 1024)
    config = Configuration(output / "trial", "make_kong", (0, 1, 2), 3, 0,
                           gpu, limits, limits, limits)
    supervisor = Supervisor(config, lambda: NativeBackend(config))
    try:
        monkeypatch.setenv("ROBODOJO_PYTHON", sys.executable)
        gateway, original_token = supervisor.start_interactive()
        # This test verifies native transport, not task-solving ability.
        assert not supervisor.state['interactive_success']  # Soft workflow guidance only.
        src = output / "source"
        src.mkdir()
        (src / "controller.py").write_text('''
import base64, json
from pathlib import Path
def main(ctx):
    for _ in range(2):
        response = ctx.call("robodojo_observe")
        state = json.loads(response["content"][0]["text"])
        images = [b for b in response["content"] if b["type"] == "image"]
        assert len(images) >= 3
        assert all(base64.b64decode(b["data"]).startswith(b"\\x89PNG") for b in images)
        response = ctx.call("robodojo_step", actions=[state["states"]] * 3)
        feedback = json.loads(response["content"][0]["text"])
        assert feedback["step_id"] == state["step_id"] + 3
    # Exercise the production 25 Hz EEF/planner/preview contracts from Python
    # inside the bundle sandbox, including publication through the auto-research gateway.
    def data(response):
        assert len([b for b in response['content'] if b['type'] == 'image']) == 6
        return json.loads(response['content'][0]['text'])
    targets = {}
    for i, arm in enumerate(('left', 'right')):
        position = list(feedback['eef_positions'][i])
        position[2] += 0.005
        targets[arm] = {'position': position, 'quaternion_wxyz': feedback['eef_quaternions_wxyz'][i],
                        'gripper_closed': False, 'gripper_opening': feedback['states'][6 + 7*i]}
    feedback = data(ctx.call('robodojo_step_eef', targets=[targets]))
    assert feedback['step_id'] == 7
    assert len(feedback['transition']['steps'][0]['executed_action']) == 14
    target = {'position': list(feedback['eef_positions'][0]),
              'quaternion_wxyz': feedback['eef_quaternions_wxyz'][0]}
    target['position'][2] += 0.02
    plan = data(ctx.call('robodojo_free_space_move', arm='left', target=target,
                         preview_only=True, include_trajectory=True))
    motion = plan['motion_plan']
    assert motion['status'] == 'Success', motion
    actions = motion['trajectory_preview']['actions']
    assert motion['trajectory_preview']['frequency_hz'] == 25
    preview = data(ctx.call('robot_preview_run', robot_model='dual_arx_x5',
                            camera_model='robodojo_gemini_wrist_v1',
                            initial_state=feedback['states'], actions=actions))
    assert preview['final_observation']['attachments']
    assert preview['execution']['actions'] == actions
    feedback = data(ctx.call('robodojo_execute_motion_plan', **motion['execute_with']))
    assert [s['executed_action'] for s in feedback['transition']['steps']] == actions
    assert feedback['transition']['goal_reached'], feedback['transition']
    assert feedback['step_id'] == 7 + len(actions)
    print('planner-preview-execution-verified', feedback['step_id'])
    Path("output/feedback.json").write_text(json.dumps(feedback))
''')
        supervisor.finish_interactive()
        ident, _ = supervisor.register(src)
        result = supervisor.run(ident, formal=False)
        assert result["returncode"] == 0, list(output.rglob("stderr.log"))[0].read_text()
        assert supervisor.backend is None
        assert supervisor.state["exploration_started"] == 2
        assert result["start_step_id"] == 0 and result["end_step_id"] > 7
        assert not result["recording_errors"]
        view = config.root / "agent-view"
        trace = [json.loads(line) for line in (view / result["artifacts"]["mcp_trace"]).read_text().splitlines()]
        motions = [e for e in trace if e.get("event") == "mcp_result" and e.get("tool") == "robodojo_step"]
        assert len(motions) == 2
        for motion in motions:
            sequence = motion["returned"]["frame_sequence"]
            manifest = json.loads((view / sequence["manifest_path"]).read_text())
            assert manifest["frequency_hz"] == 25 and manifest["frame_count"] == 3
            for frame in manifest["frames"]:
                assert len(frame["states"]) == 14
                for file in frame["files"]:
                    assert (view / file["path"]).read_bytes().startswith(b"\x89PNG")
        request = {"jsonrpc": "2.0", "id": 10, "method": "tools/call", "params": {"name": "robodojo_observe", "arguments": {}}}
        assert "error" in gateway.handle(original_token, request)
        assert type(result["evaluation"]["task_complete"]) is bool
        # A real clean but unsuccessful rehearsal qualifies (formal gives partial credit).
        full = output / "formal-source"
        full.mkdir()
        (full / "controller.py").write_text("def main(ctx):\n    ctx.call('robodojo_observe')\n")
        bundle_id, manifest = supervisor.register(full)
        with pytest.raises(RuntimeError, match="clean rehearsal"):
            supervisor.run(bundle_id, formal=True)
        rehearsal = supervisor.run(bundle_id, formal=False)
        assert rehearsal["returncode"] == 0
        assert rehearsal["task_complete"] is False
        assert not supervisor.state["formal_reserved"] and supervisor.backend is None
        assert supervisor._has_qualifying_rehearsal(bundle_id, manifest)
    finally:
        supervisor.close()
        os.environ.clear()
        os.environ.update(previous)
