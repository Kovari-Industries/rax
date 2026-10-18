# Tests

`local_tests/unit/` holds the regression suite and `local_tests/goldens/` the reviewed
workspace renderings it compares against. Run everything from the repository root with
the host environment's Python.

## Unit suite

```bash
runtime/envs/host/bin/python -m pytest -m "not gpu" -q local_tests/unit
```

It uses fake simulators, fake robot backends and a local mock model provider. It makes no
paid model call, downloads nothing and needs no credentials, and it never starts MuJoCo:
`test_dexjoco_backend.py` imports every host module with dexjoco, mujoco, gymnasium and openpi
blocked. Every test writes under pytest's temporary directory, which the suite sets as the
artifact root (`RAX_ARTIFACT_ROOT`) for its duration.

Some tests use more of the machine and skip, with the reason, when it is missing
(`-rs` lists the skips):

| Needs | Tests |
| --- | --- |
| The pinned DexJoCo checkout (`scripts/bootstrap_sources.sh`) | Tests that read the official task configurations: the catalog, official layouts and cameras (`test_dexjoco_backend.py`, part of `test_dexjoco_reviews.py`) |
| Docker, the local `python:3.12-slim` image, and `runtime/` under the host's artifact root | Real containers: isolation, offline Python, rehearsal and formal lifecycles against a fake task, capped workspaces (`test_controller_docker.py`, `test_workspace_storage.py`, parallel formal workers) |
| The agent image `rax-auto-research:dev` | Development Python inside the real image (`test_development_python.py`); git and ripgrep checks in the Codex test |
| A native `codex` binary on `PATH` | `test_auto_research_codex.py`: the real Codex CLI in the container against a local fake model endpoint, including provider failures, retries, pauses and demonstration images |
| Node.js | A syntax check of the dashboard script |

## GPU tests

Tests marked `gpu` start the real official DexJoCo simulators in `runtime/envs/dexjoco`
([Setup, step 3](../docs/SETUP.md#3-dexjoco-environment)). They run only when
`DEXJOCO_TEST_GPU` names a GPU, and call no model:

```bash
DEXJOCO_TEST_GPU=0 runtime/envs/host/bin/python -m pytest -m gpu -vs \
    local_tests/unit/test_gpu_dexjoco_preview.py local_tests/unit/test_gpu_dexjoco_reviews.py
```

- `test_gpu_dexjoco_preview.py` runs the same rows on the real episode simulator and the
  robot-only preview simulator, and checks that the preview's states follow the episode's
  (about 1 to 2 minutes per task).
- `test_gpu_dexjoco_reviews.py` runs exploration episodes of every task and checks the ground
  truth that [episode reviews](../docs/DEXJOCO_BACKEND.md#episode-reviews-exploration-only) read: every measurement recorded,
  the state tail and MuJoCo agree at reset, fixed objects stay put, and untouched objects
  settle within the stated bound.

`scripts/dexjoco_doctor.py` checks a real simulator end to end; see
[Setup, step 7](../docs/SETUP.md#7-check-the-installation).

## Workspace goldens

`test_workspace_goldens.py` renders every workspace setup and compares it with
`local_tests/goldens/workspaces.json`. After an intended change to agent instructions or
skills, review the rendered difference, then refresh the index:

```bash
runtime/envs/host/bin/python scripts/render_workspaces.py --out /tmp/before   # before the change
runtime/envs/host/bin/python scripts/render_workspaces.py --out /tmp/after    # after it
diff -ru /tmp/before /tmp/after
runtime/envs/host/bin/python scripts/render_workspaces.py --index local_tests/goldens/workspaces.json
```
