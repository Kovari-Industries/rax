# Tests

`local_tests/unit/` holds the regression suite and `local_tests/goldens/` the reviewed
workspace renderings it compares against. Run everything from the repository root with
the simulator environment's Python.

## Unit suite

```bash
runtime/envs/robodojo/bin/python -m pytest -m "not gpu" -q local_tests/unit
```

It takes about 4 minutes and uses fake simulators, fake robot backends and a local mock
model provider. It makes no paid model call, downloads nothing and needs no credentials.
Every test writes under pytest's temporary directory, which the suite sets as the
artifact root (`ROBODOJO_ARTIFACT_ROOT`) for its duration.

Some tests use more of the machine and skip, with the reason, when it is missing
(`-rs` lists the skips):

| Needs | Tests |
| --- | --- |
| The pinned RoboDojo checkout (`scripts/bootstrap_sources.sh`) | Tests of the pinned source, most marked `upstream`: native source contracts, task coverage, Isaac Lab pose math, the drive-reset patch |
| Docker, the local `python:3.12-slim` image, and `runtime/` under the host's artifact root | Real containers: isolation, offline Python, rehearsal and formal lifecycles against a fake task, capped workspaces (`test_controller_docker.py`, `test_workspace_storage.py`, parallel formal workers) |
| The agent image `robodojo-auto-research:dev` | Development Python inside the real image (`test_development_python.py`); git and ripgrep checks in the Codex test |
| A native `codex` binary on `PATH` | `test_auto_research_codex.py`: the real Codex CLI in the container against a local fake model endpoint, including provider failures, retries, pauses and demonstration images |
| Node.js | A syntax check of the dashboard script |

To leave out the tests marked `upstream` as well, use `-m "not gpu and not upstream"`.

## GPU tests

Tests marked `gpu` start real Isaac Sim simulators. They are slow, and they skip unless
the environment is complete and a GPU is free.

- **RoboDojo** (`test_gpu_preview_integration.py`, `test_gpu_mcp_contract.py`,
  `test_gpu_joint_replay.py`, the GPU case of `test_controller_docker.py`): need an idle
  visible GPU (under 20% utilization) with at least 24 GiB free. They check free-space
  motion against the robot-only preview, the observation contract of both profiles,
  large-motion joint replay, and a full container rehearsal with planning and preview.

  ```bash
  runtime/envs/robodojo/bin/python -m pytest -m gpu -vs local_tests/unit
  ```

  Restrict the eligible GPUs with `CUDA_VISIBLE_DEVICES`.
- **BEHAVIOR-1K** (`test_gpu_behavior_backend.py`, `test_gpu_behavior_preview.py`): need
  the BEHAVIOR setup and run only when `BEHAVIOR_TEST_GPU` names a GPU:

  ```bash
  BEHAVIOR_TEST_GPU=0 runtime/envs/robodojo/bin/python -m pytest -m gpu -vs \
      local_tests/unit/test_gpu_behavior_backend.py local_tests/unit/test_gpu_behavior_preview.py
  ```

GPU tests close their simulators on failure and keep their reports and logs under
`runtime/integration-tests/`. They call no model.

## Workspace goldens

`test_workspace_goldens.py` renders every workspace setup and compares it with
`local_tests/goldens/workspaces.json`. After an intended change to agent instructions or
skills, review the rendered difference, then refresh the index:

```bash
runtime/envs/robodojo/bin/python scripts/render_workspaces.py --out /tmp/before   # before the change
runtime/envs/robodojo/bin/python scripts/render_workspaces.py --out /tmp/after    # after it
diff -ru /tmp/before /tmp/after
runtime/envs/robodojo/bin/python scripts/render_workspaces.py --index local_tests/goldens/workspaces.json
```
