# DexJoCo backend

This branch runs auto-research sessions on [DexJoCo](https://dexjoco.github.io) (arXiv
2605.16257): a Franka Panda arm (single-arm tasks) or two (bimanual tasks), each with a 16-DoF
Allegro hand, in MuJoCo, on the 11 official dexterous tasks. The harness, workspace generator,
supervisor, gateway, formal batches, agent formal and dashboard are the shared ones described in
[Architecture](ARCHITECTURE.md); this document covers what is specific to DexJoCo.

## Principle: the official DexJoCo, unchanged

The pinned official checkout (`DEXJOCO_COMMIT` in `dependencies.lock`, linked at `DexJoCo/`) is
never modified. Its control, actions, observations, termination, success, randomization, seeding
and step limits are the official ones. The harness reaches them through its own simulator RPC:

| Layer | Interpreter | DexJoCo dependencies |
| --- | --- | --- |
| Agent container | the offline agent image `rax-auto-research:dev` | none |
| Host harness (frontend, supervisor, gateway, workspace generator, dashboard) | `runtime/envs/host` | none: `services/dexjoco/{benchmark,backend,process,tools,tasks,task_rules,demonstrations}.py` use the standard library, msgpack, NumPy and PyYAML |
| Simulator (`python -m services.dexjoco.server`) | `DEXJOCO_PYTHON` (default `runtime/envs/dexjoco/bin/python`, built from the official `environment-dexjoco.yaml`) | the official `dexjoco` and `openpi-client` packages |

`services/dexjoco/session.py` builds the official evaluation wrapper `DexJoCoOpenPIEnv` the way
`dexjoco-openpi-eval` does (`_set_seed(seed)`, `start()`, one `reset()` per episode, the
click_mouse pre-roll) and drives it with `step()` and `stay()`. It only adds the harness
contract: identity-checked operations, agent observations and the operator files.
`local_tests/unit/test_dexjoco_backend.py` imports every host module with dexjoco, mujoco,
gymnasium and openpi blocked.

MuJoCo renders headless through NVIDIA EGL on the session's simulator GPU
(`MUJOCO_EGL_DEVICE_ID`, set by `services/dexjoco/process.py`); the simulator needs no container.

## Interface

| Tool | Official counterpart |
| --- | --- |
| `dexjoco_step` | rows of the official policy action, `DexJoCoOpenPIEnv.step`: single arm 22-D `[flange xyz, rotation vector, 16 hand joint targets]`; dual arm 44-D `[r_xyz, r_rotvec, r_hand16, l_xyz, l_rotvec, l_hand16]`. Absolute, world frame. 1 to 150 rows per call; stops when the episode ends |
| `dexjoco_hold` | `DexJoCoOpenPIEnv.stay()` for 1 to 150 steps (consecutive holds keep the first target, as the official loop does) |
| `dexjoco_observe` | the official evaluation inputs: `state[:23]` or `state[:46]` (flange poses with `[w, x, y, z]` quaternions, hand joints), the config's `camera_mapping` cameras at the environment's 640x640 (the official client's 224x224 resize is the policy's own preprocessing), the prompt; in the `rgbd` profile also depth from the official `collect_depth_frames` and world-frame camera calibration |
| `dexjoco_pixel_to_position` | depth back-projection of 1 to 32 pixels into world-frame points (`rgbd` only) |
| `dexjoco_status` | layouts, cameras, instruction, limits |
| `dexjoco_rotation_math` | rotation and pose arithmetic for action rows, converted as the official row conversion does (scipy `Rotation`): rotation conversions, pose composition, flange targets from fingertip offsets, and the official hold row from a state vector. Pure computation, served without an episode |
| `dexjoco_preview_describe` / `dexjoco_preview_run` | the robot-only preview: the task's official robot model on a floor plane, driven by the official flange controller with the environment's own gains and the same hand servos; no table, objects or contact. It never advances the episode (`live_step_id`) |

There is no motion planner: the official control is operational-space flange-pose control plus
the hands' position actuators. Each step is 20 ms of physics, paced at 30 steps per second of
wall clock by the official environment. A step reply carries the final frame, and every 10th
frame of the motion is saved. The privileged tail of the official state (initial object poses,
table height) never reaches the agent.

**Timing difference (zero-latency evaluation).** The official evaluation is asynchronous: the
environment keeps stepping with `stay()` while the policy infers. The harness steps only when
the agent or its controller acts, so DexJoCo episodes here are the official evaluation with zero
inference latency. Everything else is the official protocol.

## Tasks, layouts and scoring

The 11 official tasks are `hammer_nail`, `click_mouse`, `pick_bucket`, `pinch_tongs`,
`fold_glasses`, `water_plant` (single arm) and `bimanual_unlock_ipad`, `bimanual_hanoi`,
`bimanual_assembly`, `bimanual_microwave_cook`, `bimanual_photograph` (two arms). Each has two
harness tasks: `<task>` uses the official `configs/rand_obj` config, `<task>_random` the
`configs/rand_full` one (camera viewpoint, lighting and table texture also randomized).

- **Formal (collection 1)** is the official evaluation: seeds 0, 1 and 2 with 50 episodes each.
  The official episode k of seed S is the (k+1)-th reset of the environment built with S, so
  formal layout `L` is seed `L // 50`, reset `L % 50 + 1`. `services/dexjoco/checks.py` verifies
  that stepping between resets does not change later layouts (it consumes no randomness), so a
  fresh process per episode reproduces the official layouts exactly. A session evaluates up to
  all 150 formal episodes per task, which is the default.
- **Exploration (collection 0)** layout `i` is the first reset of seed `1000 + i`: never an
  official evaluation layout.
- **Scoring** is the official binary success (`info["succeed"]`): `native_score` 1.0 or 0.0, with
  no partial credit. `formal-scores.json` reports the success rate overall and per official seed
  (layouts 0-49, 50-99, 100-149), with the mean and standard deviation over the seeds.
- **Step limits** are the official environments' own caps (1000 to 1500 steps; click_mouse
  spends 30 of them on its pre-roll). The wall-clock limit of an episode is 600 s plus 5 s per
  step, and the simulator has 300 s to start.

`TASK.md` gives the official prompt. With `task_rules: code-summary` (`--task-rules
code-summary`) it adds `services/dexjoco/task_rules/<task>.md`: how each official success check
is defined, summarized from the pinned source without its exact values.

**Adaptation.** A finished session on `<task>` can be the source of an adaptation session on
`<task>_random`, which continues its agent with a small episode budget (see
[Usage](USAGE.md#5-adaptation)).

## Demonstrations

`scripts/setup_dexjoco_demos.py --gpu <gpu> --task <task>` installs official raw teleoperation
demonstrations per task (Hugging Face `DexJoCo/DexJoCo-Datasets-Raw` at
`DEXJOCO_DEMOS_REVISION`; the rand_full set for `_random` tasks) under
`runtime/reference-demos/dexjoco/<task>/`. It downloads the first candidates (`--candidates`,
default 8), and in the official environment replays each from its recorded initial state
(`services/dexjoco/demo_tools.py`, as the official `replay_demos_zarr.py`) and compares its
layout with all 150 formal layouts. The first three that succeed and show no formal layout become
the task's experts (the official recordings started from seed 0's sequence, so some demos would
otherwise show formal layouts).

- `demonstration_context` (`terminal_state` by default, or `completion_sequence`) copies
  keyframes of the experts' base-camera videos into `runtime/demonstrations/`.
- `expert_demonstrations: 3` mounts the three expert episodes read-only at `runtime/experts/` for
  the exploring agent: the official `replay.zarr` and videos unchanged, plus `trajectory.npz`
  (`action_rotvec` in the `dexjoco_step` layout, `state`, `timestamp`) and `episode.json`.
  Isolated rehearsal and formal containers never see them.

## Episode reviews (exploration only)

An operator-side analyst reviews each finished exploration episode (and rehearsal) from the
simulator's ground truth and writes the agent a short review of what went wrong. Reviews exist
for benchmarks whose simulator writes ground truth (`episode_reviews` in
`services/benchmarks.py`); DexJoCo does.

### Setting

**Episode reviews** is a session setup option (Off, On or Detailed; Off by default), chosen when
the session is prepared: in the dashboard setup form, or with
`scripts/configure_auto_research.py --episode-reviews off|on|detailed`. It is stored in
`<session>/operator-analysis/settings.json` (`enabled`, `detailed`), outside the agent's
configuration. Formal episodes are never reviewed; episodes that closed while reviews were off
stay unreviewed. The dashboard's episode panel has a Review column that opens each review.

### Shared lifecycle (`services/episode_reviews/`)

- `runner.py`: reviews are part of closing an episode. The session's episode management (the
  supervisor, or the exploration pool for parallel environments) reports each closed episode to
  the session's `EpisodeReviews` (the adapter's `episode_reviewer(root)`), owned by its MCP
  service (`services/controller/frontend.py`). The review runs in the background, up to 4 at
  once, after the episode's simulator files appear; a failed Gemini call is retried once. When
  the service starts (also on resume) it reviews episodes that closed while none ran.
- One Gemini call (through the OpenRouter router; key `~/.codex/secrets/openrouter_api_key` or
  `OPENROUTER_API_KEY`) reads the benchmark's ground-truth report, the episode's condensed tool
  trace and its final RGB frames. Costs go to `operator-analysis/usage.jsonl`, are capped at
  $10 per session and are never charged to the agent's Gemini budget.
- The operator record is `operator-analysis/<episode_id>.json` (0600, never mounted into the
  agent's container): the full report (with its private part), the explanation and the cost.
- The agent receives only `review.md` in that episode's own published run directory (the
  `artifacts.directory` its `finish` or `rehearse` reply named), read-only under
  `/workspace/runtime/autonomous_controller/runs/`. A standard review is the analyst's
  explanation alone, of up to about 600 words; a **detailed** review opens with the
  ground-truth report and the analyst writes up to about 1000 words with exact numbers.
- `workspace.py`: while reviews are on, the exploring agent's `TASK.md` ends with an Episode
  reviews section in the benchmark's words (where each review appears, what it says), and
  `AGENTS.md` reminds the agent to read each review before its next attempt. Both are written at
  deploy and refreshed on every resume (the adapter's `refresh_workspace`); formal copies get none.

A benchmark supplies its ground truth and wording through its adapter hook `review_analysis()`:
a module with `ground_truth_report(sim_dir)`, `condensed_trace(mcp_trace)`, `explain(...)`,
`agent_review(...)` and `task_instruction(task)` (see `services/episode_reviews/__init__.py`).
The runner finds the analysis through the session's configured benchmark, never by name.

### Ground truth and report

**Ground truth.** When an episode ends, the simulator (`services/dexjoco/session.py`) writes a
0600 `sim/private_goal_report.json` (`dexjoco_goal_report_v1`), read-only from the official
environment, which is never modified:

- success, termination, the end reason, steps and the step limit;
- the environment's own `info` at reset and at the end (hammer `nail_depth`/`hammer_hit`, tongs
  `pinch_count`, iPad `pressed_digits`, Hanoi `tower_state`/`illegal_reason`, photograph
  `region_pass`/`angle_pass`/`shutter_pressed` and their distances), which boolean info keys were
  ever true since reset, and every iPad digit pressed;
- the official state vector at reset and at the end; its privileged tail holds each task
  object's reset pose (the official config's `proprio_keys` order) and the table height offset;
- the task objects' poses, and the points and angles the success checks use, read from MuJoCo
  by the official XML names at reset and at the end (`services/dexjoco/objects.py`: free-joint
  and body poses, the bucket's corner markers, the glasses' temple hinges, the watering can's
  nozzle, the microwave door hinge, ...).

**Report** (`services/dexjoco/analysis.py`):

- the outcome and why the episode ended: on success, on a failure condition (the environment
  ended it before its step cap), at the step limit, or early (the agent's `finish`, a wall-clock
  limit, the controller run ending);
- the task's success and failure conditions as `services/dexjoco/task_rules/<task>.md` states
  them, each with what the simulator measured at the end, for example "the boxed food's centre
  ended 12 cm outside the box spanned by the bucket's corner markers", "digits pressed, in order:
  1, 3", "not met at the end; met at some earlier step"; a condition the environment does not
  report (the mouse display's colour, the microwave start button) says so;
- how each task object moved from its reset pose (the state tail) to the end: centimetres
  horizontally and vertically, and the tilt and the turn about the vertical of its world-frame
  rotation. Untouched objects settle under gravity right after reset, by up to about 3 cm and 20
  degrees; the report says so.

The conditions reach the agent and the analyst only in `task_rules: code-summary` sessions, whose
`TASK.md` already states them. With `public` (the official prompt only) the agent's review and the
analyst's input give the outcome, the end reason and the objects' movement; the operator's summary
always lists the conditions with what was measured. Agent-facing text carries relative
measurements only, never seeds, layout ids, world coordinates, the table height or the model's
internal names; Gemini's text is scrubbed of these as well. DexJoCo scores success or failure, so
no review carries a partial score. Anomalies (a state tail whose length or reset pose disagrees
with MuJoCo, a measurement that could not be read) go to the operator record's `warnings`.

**Checks.** `local_tests/unit/test_dexjoco_reviews.py` (synthetic goal reports, the lifecycle
with a fake router, the workspace text) and, on a GPU,
`DEXJOCO_TEST_GPU=0 runtime/envs/host/bin/python -m pytest -m gpu local_tests/unit/test_gpu_dexjoco_reviews.py`
(every task's real simulator: every measurement recorded, tail and MuJoCo agree at reset, fixed
objects stay put, settling within the stated bound).

## Setup

```bash
bash scripts/bootstrap_sources.sh                 # DexJoCo/ at DEXJOCO_COMMIT
bash scripts/setup_dexjoco_env.sh                 # runtime/envs/dexjoco, with the Miniforge of setup_host_env.sh
runtime/envs/host/bin/python scripts/setup_dexjoco_demos.py --gpu 0 --task pick_bucket
runtime/envs/host/bin/python scripts/dexjoco_doctor.py --gpu 0 --task pick_bucket
```

`setup_dexjoco_env.sh` installs the conda dependencies of the official yaml from conda-forge only
(some sites block the `defaults` channel) and its pip section verbatim. The doctor checks, per
task, the observation contract, a hold and an official action chunk, depth back-projection
against the true object position, the operator files, and the simulator-side checks (layout
determinism, timing). The full guide is [Setup](SETUP.md).
