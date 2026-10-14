# BEHAVIOR-1K backend

Auto-research sessions can run on [BEHAVIOR-1K](https://behavior.stanford.edu) instead of
RoboDojo: the Galaxea R1Pro mobile bimanual robot doing household tasks in OmniGibson.
The benchmark is a session setting (`--benchmark behavior`, or a task marked `· BEHAVIOR`
in the dashboard); the harness, workspace generator, supervisor, gateway, formal batches,
agent formal and dashboard are shared.

The pinned source is BEHAVIOR-1K `v3.9.3-post1` (`bd049de`), the 2026 challenge
evaluation tag. It runs on the same stack as RoboDojo: Isaac Sim 5.1.0, Python 3.11,
torch 2.7.0+cu128, numpy 1.26.

## Separation

| Layer | Interpreter | BEHAVIOR dependencies |
|---|---|---|
| Agent CLI, its commands and submitted programs (native, sandboxed) | `runtime/envs/robodojo` | none |
| Host harness (frontend, supervisor, gateway, workspace generator, dashboard) | `runtime/envs/robodojo` | none: `services/behavior/{backend,process,tools,tasks,robot}.py` use the stdlib, msgpack and numpy |
| Simulator (`python -m services.behavior.server`) | `runtime/envs/behavior` | OmniGibson, BDDL, cuRobo fork, warp 1.12, licensed assets |
| Motion preview (`python -m services.behavior.preview_server`) | `runtime/envs/behavior` | OmniGibson (robot and floor plane only) |

The host talks to the simulator over the shared msgpack RPC (`services/simulator/protocol.py`,
`rpc.py`). `local_tests/unit/test_behavior_backend.py` imports every host module with
OmniGibson, Isaac Sim, BDDL and cuRobo blocked. A separate environment is required
because OmniGibson pins warp 1.12 and a StanfordVL cuRobo fork, while RoboDojo uses its
own cuRobo fork and warp 1.11.

## Setup

```bash
bash scripts/bootstrap_sources.sh                       # also clones BEHAVIOR-1K, linked at BEHAVIOR-1K/
bash scripts/setup_behavior_env.sh --accept-nvidia-eula # runtime/envs/behavior
bash scripts/setup_behavior_assets.sh --accept-dataset-tos
bash scripts/setup_behavior_demos.sh                    # 2 head-camera demos per task (keyframes)
runtime/envs/robodojo/bin/python scripts/behavior_doctor.py --gpu 0
```

- **Sources.** The bootstrap places the BEHAVIOR-1K checkout (about 1 GB) next to the
  runtime storage and links it at `BEHAVIOR-1K/`.
- **Environment.** `setup_behavior_env.sh` installs the pinned Miniforge if neither setup
  script has yet, creates `runtime/envs/behavior` (Python 3.11 with conda nvcc 12.8, which
  cuRobo compiles against), runs the upstream setup and never touches
  `runtime/envs/robodojo`. The flag accepts the NVIDIA Isaac Sim EULA.
- **Assets.** The asset script accepts the BEHAVIOR-1K dataset terms (non-commercial
  research use; the data may be used only within OmniGibson). It installs the robot
  assets, the BEHAVIOR-1K assets, the 2026 task instances and the decryption key into
  `runtime/behavior-data`: about 37 GB installed. Downloads are resumable, checked
  against the checksums in `dependencies.lock` and staged in `runtime/behavior-staging`;
  each zip is deleted after unpacking unless `--keep-zips` is passed, so the peak is
  about 70 GB.
- **Demonstrations.** About 490 MB for all 100 tasks (see Demonstrations below).
- **Doctor.** One simulator on a train instance: startup time, stepping rate, peak GPU
  memory, camera intrinsics, projection consistency, gripper sign, odometry drift and
  (unless `--no-planner`) a cuRobo plan. It writes a JSON report.

Each simulator or preview process leases one of 32 appdata slots under
`runtime/behavior-cache/appdata` (kit, shader and texture caches). Concurrent processes
never share a slot, and a process that finds all 32 in use fails to start; sequential
reuse keeps caches warm.

## Launching a session

**Dashboard.** Choose a task marked `· BEHAVIOR`. Its defaults load into the editable fields
(first exploration layout 0, 10 formal episodes, 2 formal workers), and switching back to a
RoboDojo task restores that benchmark's values. Fields that do not apply are fixed and
disabled, with the reason as a tooltip:

| Field | Fixed to | Why |
|---|---|---|
| Observation profile | `rgbd` | The challenge allows RGB, depth and proprioception |
| Task rules | `public` | `TASK.md` always gives the official BDDL task definition |
| Formal collection | `1` | Formal uses the 20 held-out public test instances |
| Write handoff, adaptation source | off / none | Adaptation needs another layout collection; BEHAVIOR has one train collection |
| Adaptation context | `compact` | Adaptation is unavailable |

The configuration rejects other values for these fields, from the dashboard or the terminal.
**Episode reviews** (below) is a BEHAVIOR-only field. Prepare, then Launch, as for RoboDojo.

**Terminal.**

```bash
runtime/envs/robodojo/bin/python scripts/configure_auto_research.py --benchmark behavior \
    --task turning_on_radio --sim-gpu 0 --episodes 10
bash scripts/start_auto_research_agent.sh --config <printed operator-input.json>
```

Budget the simulator GPU at about 10.5 GB per BEHAVIOR simulator and 4.5 GB for the
motion preview: one simulator per exploration environment while the agent explores, plus
the preview once the agent first uses it, and one simulator per formal worker during the
formal batch. The first launch on each appdata slot builds caches (about 6 minutes
instead of 1.5).

## Tasks and layouts

`services/behavior/tasks.json` holds all 100 tasks of the 2026 BEHAVIOR Challenge, with
their scenes, rooms, official step limits and the official 2025 per-task results.
[BEHAVIOR tasks](BEHAVIOR_TASKS.md) lists them with human demonstration times and 2025
scores as a difficulty reference: tasks 0-49 carried over from 2025 (the easiest, by top-4
mean 2025 q-score: `turning_on_radio`, `moving_boxes_to_storage`, `picking_up_trash`,
`make_microwave_popcorn`); tasks 50-99 are new and have no official results yet.
`runtime/envs/robodojo/bin/python scripts/generate_behavior_tasks.py` regenerates the
catalog, the task list and any missing task rules from the checkout and the installed
task instances.

The step limit is the official timeout: 1.5 x the mean human demo length at 30 Hz. It
ranges from 3,224 (`turning_on_radio`) to 39,090 steps (`assembling_gift_baskets`).
Episodes have no wall-clock limit: an episode runs until its step limit or until it is
ended. A simulator has 900 s to start.

Layout collection 0 is the 300 train instances per task (exploration; layout ids wrap
around, so exploration cycles through them); collection 1 is the 20 public test instances
(IDs 301-320, held out, formal). Formal defaults to 10 episodes, the challenge's reported
protocol, with 2 concurrent workers.

`TASK.md` comes from `services/behavior/task_rules/<task>.md`: the official instruction
(50 tasks), a written goal (51 tasks; the other 49 point to the BDDL goal), and the BDDL
goal and initial conditions verbatim (the challenge permits BDDL definitions during
evaluation). A Scoring section, in every mode, explains the partial credit (the q-score)
and asks for the most of it in every episode. An agent formal copy's instructions also ask
it to make its one episode score as high as it can.

**Demonstrations.** `demonstration_context` works as for RoboDojo (default
`terminal_state`), from two official 2026 teleoperation demos per task
(`behavior-1k/2026-challenge-demos`, pinned in `dependencies.lock`). Each demo is a
different train instance near the task's median demo length.
`scripts/setup_behavior_demos.sh` streams only those episodes' head-camera segments (about
15 minutes for all 100 tasks, about 5 MB per task) and keeps keyframes in
`runtime/reference-demos/behavior/<task>/`: the start, the end of every annotated skill,
fillers so no gap exceeds 10 s, at most 48 per demo. It is resumable and takes
`--episodes`, `--tasks` and `--workers`. `terminal_state` gives each demo's final frame and
`completion_sequence` all its keyframes, listed per demo in the manifest. The agent gets
images only; the demos' actions, states, annotations and episode ids stay in the operator
cache. The challenge permits training on the demos.

## What the agent sees and does

Only challenge-track inputs (`docs/challenge/evaluation.md` upstream): RGB, linear depth and
the official 61-D proprioception, plus odometry integrated from the proprioceptive base
velocity (start-relative; no world pose, map, object state or segmentation). Camera
intrinsics come from the live sensors and camera poses are relative to the robot base.

| Tool | Contract |
|---|---|
| `behavior_status` / `behavior_observe` | episode, step budget, action layout and joint limits / cached RGB-D, calibration, named proprio, odometry |
| `behavior_step` | 1..150 native 23-D rows at 30 Hz: base velocity [vx, vy, wz] in [-1, 1], trunk and arms absolute joint targets, grippers in [-1, 1] (-1 closed, +1 open) |
| `behavior_base_move` | closed-loop relative planar move from odometry only |
| `behavior_step_eef` | one arm to a robot-base-frame pose by closed-loop IK (OmniGibson's IK-controller update on the relative Jacobian); trunk, base and other arm hold still; not collision-aware |
| `behavior_free_space_move` / `behavior_execute_motion_plan` | cuRobo plan of one arm (`use_trunk`: also the trunk) to a robot-base-frame pose, self-collision only; the other hand holds |
| `behavior_pixel_to_position` | depth pixel -> robot-base-frame point |
| `behavior_preview_describe` / `behavior_preview_run` | robot-only dry run of `behavior_step` rows or a `preview_only` plan, watched by fixed third-person cameras; never changes the episode |

Frames are saved every 6th step (5 Hz) and at the final step; only the final frame is
returned inline.

**Success and score.** An episode succeeds when `task_complete` is true (every BDDL goal
predicate holds). Its score is the challenge q-score: 1.0 on success, otherwise the
fraction of goal conditions false at the start that hold at the end, in the best goal
option. Formal results report the success rate and the mean q-score. Episode replies
report `task_complete` but never the q-score or predicate counts, which stay in the
private operator state; only a detailed episode review shows the agent its partial score.

The simulator is built by the official `BatchedEvaluator` (`num_envs=1`): the same
environment configuration, instance loading, settling, base mass, head aperture and
transition rules as `omnigibson.eval.eval`. Every episode is a fresh process.

`behavior_free_space_move` keeps one cuRobo generator per process (arms-only or with the
trunk) and frees it before building the other. Two generators with captured CUDA graphs
in one process crashed the simulator on the next replay (CUDA invalid argument or
misaligned address, then a segfault).

Explicit joint targets in `behavior_step` and preview actions/initial state accept
at most 1e-4 rad beyond a joint limit: the service clamps and reports small
corrections, and rejects larger violations before execution. Base and EEF convenience
moves generate their own hold targets for non-commanded joints from measured
proprioception. These holds always clamp to the joint limits, regardless of measured
excursion size, and report `transition.joint_target_clamping` with
`policy="clamp_measured_hold"`. Robot drift therefore does not reject a convenience
motion. Observations retain the raw measured joints and `joint_limit_violations`.

### Motion preview

`behavior_preview_run` replays motion on a second, persistent OmniGibson process
(`services/behavior/preview_session.py`). It loads the evaluation R1Pro
(`omnigibson/eval/r1pro.yaml`, base mass 250, 30 Hz actions over 120 Hz physics) on an
empty `Scene` with a floor plane and a `DummyTask`, so no scene, object or task data is
loaded. Three fixed 480x480 RGB cameras (`preview_front`, `preview_side`, `preview_top`)
frame about 2.3 m around the start pose; base moves beyond about half a metre leave
the front view.

- **Input:** 1..150 native rows (`actions`), or `motion_plan_id` from
  `behavior_free_space_move preview_only`. The live session returns that plan's rows
  without consuming it (`motion_plan_rows`), so the plan stays executable.
- **Start state:** each run resets the base to the origin and sets the trunk, arm and
  finger joints. They come from `initial_state`, or by default from the live episode's
  measured proprio and last gripper commands. The run then settles for 10 holding rows.
- **Reply:** `end_state` (joints and end-effector poses in the base frame),
  `base_motion` (from the preview base pose, start-relative) and `frame_sequence` under
  `runtime/frames/behavior_preview/`; only the final frame of each camera is inline.
- **Episode records:** the reply carries `live_step_id`, not `step_id`, so recordings
  and step budgets never count it.

The process starts on the first run (about 25 s), shares the simulator GPU with its
own appdata slot, takes about 1 s per 30 rows, and stops with the backend.

### Episode reviews (exploration only)

**Episode reviews** is a session setup option (Off, On or Detailed; Off by default;
BEHAVIOR only), chosen when the session is prepared: in the dashboard setup form, or
with `scripts/configure_auto_research.py --episode-reviews off|on|detailed`. When it is
on, a separate analyst reviews each exploration or rehearsal episode that closes. Formal
episodes are never reviewed. The setting is stored in `operator-analysis/settings.json`
(`enabled`, `detailed`), outside the agent's configuration. The dashboard's Review column
opens each review.

1. **Ground truth:** when an episode ends, the simulator writes a 0600
   `sim/private_goal_report.json`. It lists every grounded goal condition with its truth
   at reset and at the end, plus the extents of the task objects (and, for fixed
   furniture, of its collision links).
2. **Report:** `services/behavior/analysis.py` sorts the conditions into missed, undone,
   achieved and already true, using the goal option closest to completion. Each missed
   relation gets a line of relative geometry, such as "10 cm outside sink 1's footprint;
   no part of sink 1 is directly above it".
3. **Explanation:** one Gemini call reads the report, the episode's condensed tool trace
   and its final RGB frames, and writes a review of up to about 600 words.
4. **Delivery:** the review is the only thing the agent receives. It is written as
   `review.md` in that episode's own published run directory (the `artifacts.directory`
   its `finish` or `rehearse` reply named), which the agent reads read-only under
   `runtime/autonomous_controller/runs/` in its workspace. Nowhere else carries it, so a review
   cannot be confused with another episode's. Scene object names and score vocabulary are
   removed first. A **detailed** review also opens with the episode's partial score (the
   q-score) and the ground-truth goal checklist with its geometry, and the analyst writes
   up to about 1000 words with exact numbers. Scene object names stay removed.
5. **Instructions:** while reviews are on, the exploring agent's `TASK.md` ends with an
   Episode reviews section saying where each review appears and to read it before the next
   attempt (detailed: that it carries the partial score), and its `AGENTS.md` carries a
   one-line reminder before its Workflow section. Both are written at deploy and
   refreshed on every resume, so a changed setting takes effect at the next launch. Formal
   copies get neither.

Reviews are part of closing an episode. When an exploration episode closes (`finish`, a
new `start_episode`, a parallel slot's close) or a rehearsal ends, the session's episode
management (the supervisor, or the exploration pool for parallel environments) reports
it to `services/behavior/reviews.py`, owned by the session's MCP service
(`services/controller/frontend.py`). The review runs in the background, up to 4 at once,
so closing never waits for it; a failed Gemini call is retried once. When the service
starts (also on resume) it reviews episodes that closed while none ran. No dashboard is
involved; the dashboard only shows reviews.

The operator record is `operator-analysis/<episode_id>.json` (0600, in the session
directory, which the agent's sandbox hides). It holds the full report, the explanation and the cost. Costs
go to `operator-analysis/usage.jsonl`, are capped at $10 per session and are never
charged to the agent's Gemini budget. The key is `~/.codex/secrets/openrouter_api_key`
(or `OPENROUTER_API_KEY`).

## Measurements (RTX PRO 6000 Blackwell, shared GPU)

| | |
|---|---|
| Simulator ready, first launch (cold caches) | ~340 s |
| Simulator ready, warm appdata slot | ~90 s |
| Task-instance reset inside a loaded process | 4.3 s |
| Official `env.step` (full-resolution RGB-D, 3 cameras) | 79 ms (12.7 steps/s) |
| Holding measured joints for 60 steps | < 1e-5 rad drift |
| `behavior_base_move(0.5, 0, 0.5)` | reached in 50 steps; odometry within 1 cm / 0.008 rad of the true pose |
| Head-camera floor back-projection | z = -0.003 m |
| cuRobo generator warmup / plan | 16 s once / 0.3-4.6 s |
| Arm plan with trunk, before and after base motion | end-effector error < 0.1 mm |
| Arms-only plan (finetune L-BFGS on the PyTorch path) and arms-only IK | within 1 cm; trunk still |
| GPU memory: simulator / motion preview | ~10.5 GB / ~4.5 GB |

## Known limitations

- cuRobo on Blackwell (sm_120): OmniGibson drops R1Pro's whole-body embodiment on these
  GPUs. The arms-only `arm_no_torso` configuration's finetune trajopt would launch cuRobo's
  fused L-BFGS kernel (history 27) with 392 threads x 168 registers, over the per-block
  register limit ("too many resources requested for launch"; NVlabs/curobo#716, open).
  `services/behavior/planner.py` configures that one solver with `use_cuda_kernel: false`,
  the PyTorch path the trunk-enabled configuration already takes. Narrowing joint limits
  at runtime is not a substitute: cuRobo copies the limits at construction. The planner
  works relative to the holonomic base's virtual root, so base-frame targets are converted
  to world coordinates inside the simulator.
- `behavior_step_eef` is local: it masks joints held at their limits and reports
  `reached: false` when the remaining joints cannot close the error (common right after a
  planned move that leaves the elbow or wrist at a limit).
- Isaac Sim bundles `omni.warp.core` 1.8.2; `services.behavior.server` imports the pinned
  warp 1.12 and cuRobo before the Kit app starts so cuRobo does not bind the bundled copy.
- Episodes start a fresh simulator (about 90 s warm). Reusing a loaded process for
  exploration resets (4 s) is a possible optimization.
