"""Run the actual setup shell against local fake activation/build tools; no installs."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import pytest

PROJECT = Path(__file__).resolve().parents[2]
PINNED = {'isaacsim': '5.1.0.0', 'torch': '2.7.0+cu128', 'numpy': '1.26.0', 'omnigibson': '3.9.3',
          'bddl': '3.7.0', 'nvidia-curobo': '0.0.0', 'warp-lang': '1.12.0'}


def fake_site(root, packages):
    """Installed-distribution metadata only, read by the script's version check."""
    for name, version in packages.items():
        info = root / f'{name.replace("-", "_")}-{version}.dist-info'
        info.mkdir(parents=True)
        (info / 'METADATA').write_text(f'Metadata-Version: 2.1\nName: {name}\nVersion: {version}\n')


def executable(path, body):
    path.write_text('#!/usr/bin/env bash\n' + body + '\n')
    path.chmod(0o755)


def run_setup(tmp_path, case, packages=PINNED):
    project, runtime, upstream = tmp_path / 'project', tmp_path / 'runtime', tmp_path / 'upstream'
    scripts, prefix, conda = project / 'scripts', runtime / 'envs/behavior', runtime / 'miniforge3/bin'
    for directory in (scripts, prefix / 'bin', conda, upstream):
        directory.mkdir(parents=True)
    for name in ('setup_behavior_env.sh', 'behavior_env.sh'):
        shutil.copyfile(PROJECT / 'scripts' / name, scripts / name)
    shutil.copyfile(PROJECT / 'dependencies.lock', project / 'dependencies.lock')
    if case == 'no_miniforge':
        shutil.rmtree(runtime / 'miniforge3')
    else:
        executable(conda / 'conda', 'exit 99')  # An install would be a test failure.
    fake_site(tmp_path / 'site', packages)
    # pip calls succeed without installing; the version check runs for real on the fake metadata.
    executable(prefix / 'bin/python', 'if [[ "$1" == "-m" ]]; then echo "$*" >> "$TEST_PIP_LOG"; exit 0; fi\n'
               'touch "$TEST_PYTHON_MARKER"\n'
               f'PYTHONPATH="{tmp_path / "site"}" exec "{sys.executable}" -S "$@"')
    fakebin = tmp_path / 'fakebin'
    fakebin.mkdir()
    # A download that is not the pinned installer must be refused before it runs.
    executable(fakebin / 'curl', 'while [[ $# -gt 0 ]]; do [[ "$1" == "-o" ]] && { out="$2"; shift; }; shift; done\n'
               'printf \'touch "%s"\\n\' "$TEST_INSTALL_MARKER" > "$out"')
    for compiler in ('gcc', 'g++'):
        executable(prefix / 'bin' / compiler, 'exit 0')
    executable(prefix / 'bin/nvcc', 'echo "Cuda compilation tools, release 12.8"\n' +
               ('exit 1' if case == 'nvcc_fails' else 'exit 0'))
    if case == 'nvcc_version':
        executable(prefix / 'bin/nvcc', 'echo "Cuda compilation tools, release 12.7"')
    executable(prefix / 'bin/git', f'source "{project / "dependencies.lock"}"\necho "$BEHAVIOR_COMMIT"')
    executable(upstream / 'setup.sh', 'touch "$TEST_BUILD_MARKER"\n' +
               ('exit 42' if case == 'build_fails' else 'exit 0'))
    activation = ['export CONDA_PREFIX="$1"']
    if case != 'prefix_only':
        activation.extend(['export CC="$1/bin/gcc"', 'export CXX="$1/bin/g++"'])
    if case == 'wrong_prefix':
        activation.append('export CONDA_PREFIX=/wrong/prefix')
    if case == 'wrong_python':
        activation.append('export PATH=/usr/bin:/bin')
    if case == 'missing_cxx':
        activation.append('unset CXX')
    if case == 'broken_compiler':
        executable(prefix / 'bin/g++', 'exit 1')
    if case == 'outside_compiler':
        activation.append('export CC=/usr/bin/cc')
    if case == 'wrong_cuda_home':
        activation.append('export CUDA_HOME=/wrong/cuda')
    if case == 'missing_nvcc':
        (prefix / 'bin/nvcc').unlink()
    activation.append('return 1' if case in ('report_failure', 'prefix_only') else 'return 0')
    if case != 'no_miniforge':
        (conda / 'activate').write_text('\n'.join(activation) + '\n')
    env = dict(os.environ)
    for key in ('CONDA_PREFIX', 'CC', 'CXX', 'BEHAVIOR_PYTHON'):
        env.pop(key, None)
    build_marker, python_marker = tmp_path / 'built', tmp_path / 'python-ran'
    env.update(BEHAVIOR_RUNTIME_ROOT=str(runtime), BEHAVIOR_ENV_ROOT=str(prefix), BEHAVIOR_SOURCE=str(upstream),
               TEST_BUILD_MARKER=str(build_marker), TEST_PYTHON_MARKER=str(python_marker),
               TEST_PIP_LOG=str(tmp_path / 'pip.log'), TEST_INSTALL_MARKER=str(tmp_path / 'installer-ran'),
               PATH=f'{fakebin}:{env["PATH"]}', TMPDIR=str(tmp_path))
    result = subprocess.run(['bash', str(scripts / 'setup_behavior_env.sh'), '--accept-nvidia-eula'],
                            env=env, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    return result, build_marker.exists(), python_marker.exists()


@pytest.mark.parametrize('case', ['valid', 'report_failure'])
def test_complete_activation_can_build_even_if_report_hook_returns_one(tmp_path, case):
    result, built, python_ran = run_setup(tmp_path, case)
    assert result.returncode == 0, result.stderr
    assert built and python_ran


@pytest.mark.parametrize('case', ['prefix_only', 'wrong_prefix', 'wrong_python', 'missing_cxx',
                                  'broken_compiler', 'outside_compiler', 'wrong_cuda_home',
                                  'missing_nvcc', 'nvcc_version', 'nvcc_fails'])
def test_activation_checks_stop_incomplete_toolchains_before_build(tmp_path, case):
    result, built, python_ran = run_setup(tmp_path, case)
    assert result.returncode != 0 and ('Activation failed' in result.stderr or 'nvcc 12.8' in result.stderr)
    assert not built and not python_ran


def test_errexit_is_restored_before_upstream_build(tmp_path):
    result, built, python_ran = run_setup(tmp_path, 'build_fails')
    assert result.returncode == 42
    assert built and not python_ran


def test_numpy_is_pinned_after_the_upstream_build(tmp_path):
    result, built, python_ran = run_setup(tmp_path, 'valid')
    assert result.returncode == 0, result.stderr
    assert 'pip install numpy==1.26.0' in (tmp_path / 'pip.log').read_text()
    assert 'BEHAVIOR environment ready' in result.stdout


@pytest.mark.parametrize('name, version', [('numpy', '1.26.4'), ('torch', '2.8.0+cu128'), ('isaacsim', '5.0.0.0')])
def test_a_version_other_than_the_pins_fails_the_setup(tmp_path, name, version):
    result, built, python_ran = run_setup(tmp_path, 'valid', {**PINNED, name: version})
    assert result.returncode != 0 and built and python_ran
    assert f'{name}=={version}' in result.stdout and 'environment check failed' in result.stderr


def test_a_missing_package_fails_the_setup(tmp_path):
    result, built, python_ran = run_setup(tmp_path, 'valid', {k: v for k, v in PINNED.items() if k != 'omnigibson'})
    assert result.returncode != 0 and 'omnigibson==MISSING' in result.stdout and 'omnigibson' in result.stderr


def test_missing_miniforge_is_downloaded_and_refused_unless_it_matches_the_pin(tmp_path):
    result, built, python_ran = run_setup(tmp_path, 'no_miniforge')
    assert result.returncode != 0 and 'FAILED' in (result.stdout + result.stderr)
    assert not (tmp_path / 'installer-ran').exists() and not built and not python_ran
    assert not list(tmp_path.glob('tmp*.sh'))  # The refused installer is removed.


def lock_values(text):
    return dict(line.split('=', 1) for line in text.splitlines() if '=' in line and not line.startswith('#'))


def run_assets(tmp_path, pins=None):
    """Run setup_behavior_assets.sh with a fake Hub client that serves local zips."""
    project, runtime, zips, fakes = (tmp_path / name for name in ('project', 'runtime', 'zips', 'fakes'))
    for directory in (project / 'scripts', runtime / 'envs/behavior/bin', zips, fakes / 'huggingface_hub',
                      fakes / 'omnigibson/utils'):
        directory.mkdir(parents=True)
    for name in ('setup_behavior_assets.sh', 'behavior_env.sh'):
        shutil.copyfile(PROJECT / 'scripts' / name, project / 'scripts' / name)
    lock = (PROJECT / 'dependencies.lock').read_text()
    values = lock_values(lock)
    for versioned in (values['BEHAVIOR_ROBOT_ASSETS'], values['BEHAVIOR_TASK_INSTANCES'], values['BEHAVIOR_ASSETS']):
        with zipfile.ZipFile(zips / f'{versioned}.zip', 'w') as z:
            z.writestr(f'{versioned}/payload.txt', versioned)
    for key, value in (pins or {}).items():
        digest = hashlib.sha256((zips / f'{values[value]}.zip').read_bytes()).hexdigest()
        lock = lock.replace(f'{key}={values[key]}', f'{key}={digest}')
    (project / 'dependencies.lock').write_text(lock)
    (fakes / 'huggingface_hub/__init__.py').write_text(
        'import json, os, shutil\n'
        'def hf_hub_download(repo_id, filename, repo_type, revision, local_dir):\n'
        '    with open(os.environ["TEST_HUB_LOG"], "a") as log:\n'
        '        log.write(json.dumps([repo_id, filename, repo_type, revision]) + "\\n")\n'
        '    os.makedirs(local_dir, exist_ok=True)\n'
        '    return shutil.copy(os.path.join(os.environ["TEST_ZIPS"], filename), local_dir)\n')
    (fakes / 'omnigibson/__init__.py').write_text('')
    (fakes / 'omnigibson/utils/__init__.py').write_text('')
    (fakes / 'omnigibson/utils/asset_utils.py').write_text(
        'import os\n'
        'def get_key_path():\n    return os.path.join(os.environ["OMNIGIBSON_DATA_PATH"], "omnigibson.key")\n'
        'def download_key():\n    open(get_key_path(), "w").close()\n')
    executable(runtime / 'envs/behavior/bin/python', f'PYTHONPATH="{fakes}" exec "{sys.executable}" -S "$@"')
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('BEHAVIOR_', 'OMNIGIBSON_', 'HF_', 'XDG_'))}
    env.update(BEHAVIOR_RUNTIME_ROOT=str(runtime), TEST_ZIPS=str(zips), TEST_HUB_LOG=str(tmp_path / 'hub.log'))
    result = subprocess.run(['bash', str(project / 'scripts/setup_behavior_assets.sh'), '--accept-dataset-tos'],
                            env=env, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    calls = [json.loads(line) for line in (tmp_path / 'hub.log').read_text().splitlines()] \
        if (tmp_path / 'hub.log').exists() else []
    return result, values, runtime / 'behavior-data', calls


ALL_PINS = {'BEHAVIOR_ROBOT_ASSETS_SHA256': 'BEHAVIOR_ROBOT_ASSETS',
            'BEHAVIOR_TASK_INSTANCES_SHA256': 'BEHAVIOR_TASK_INSTANCES', 'BEHAVIOR_ASSETS_SHA256': 'BEHAVIOR_ASSETS'}


def test_datasets_install_at_the_pinned_revision_when_every_zip_matches(tmp_path):
    result, values, data, calls = run_assets(tmp_path, ALL_PINS)
    assert result.returncode == 0, result.stderr
    revision = values['BEHAVIOR_DATASETS_REVISION']
    assert len(revision) == 40 and calls == [
        ['behavior-1k/zipped-datasets', f'{values[name]}.zip', 'dataset', revision]
        for name in ('BEHAVIOR_ROBOT_ASSETS', 'BEHAVIOR_TASK_INSTANCES', 'BEHAVIOR_ASSETS')]
    for name, versioned in (('omnigibson-robot-assets', values['BEHAVIOR_ROBOT_ASSETS']),
                            ('2026-challenge-task-instances', values['BEHAVIOR_TASK_INSTANCES']),
                            ('behavior-1k-assets', values['BEHAVIOR_ASSETS'])):
        assert (data / name / versioned / 'payload.txt').read_text() == versioned
    assert (data / 'omnigibson.key').exists()
    assert not list((tmp_path / 'runtime/behavior-staging').glob('*.zip'))


def test_a_zip_that_differs_from_its_pinned_sha256_is_refused_before_extraction(tmp_path):
    pins = {key: value for key, value in ALL_PINS.items() if key != 'BEHAVIOR_TASK_INSTANCES_SHA256'}
    result, values, data, calls = run_assets(tmp_path, pins)
    zip_name = f'{values["BEHAVIOR_TASK_INSTANCES"]}.zip'
    actual = hashlib.sha256((tmp_path / 'zips' / zip_name).read_bytes()).hexdigest()
    assert result.returncode != 0
    assert (f'{zip_name} at revision {values["BEHAVIOR_DATASETS_REVISION"]} has sha256 {actual}, '
            f'expected {values["BEHAVIOR_TASK_INSTANCES_SHA256"]}') in result.stderr
    assert (data / 'omnigibson-robot-assets').is_dir()
    assert not (data / '2026-challenge-task-instances').exists()
    assert not (data / '.2026-challenge-task-instances.partial').exists()
    assert not (tmp_path / 'runtime/behavior-staging' / zip_name).exists()
    assert len(calls) == 2 and not (data / 'omnigibson.key').exists()
