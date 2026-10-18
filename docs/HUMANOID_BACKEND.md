# HumanoidArena backend

This branch runs auto-research sessions on [HumanoidArena](https://github.com/William-wAng618/HumanoidArena):
a Unitree G1 (29 DoF) with Dex3 hands under the frozen [SONIC](https://huggingface.co/nvidia/GEAR-SONIC)
whole-body controller, on the arena tasks that have official test episodes. The harness,
workspace generator, supervisor, gateway, formal batches, agent formal and dashboard are the
shared ones described in [Architecture](ARCHITECTURE.md); this document covers what is
specific to HumanoidArena.

Pinned sources (`dependencies.lock`): HumanoidArena `68479287` (`release/open-source-prep`), its
released simulation assets, and the SONIC encoder and decoder from Hugging Face
`nvidia/GEAR-SONIC` `6733128a`. The simulator runs on Isaac Sim 6.1 with Isaac Lab patched for
6.1 and cuRobo, in the image `scripts/build_humanoid_image.sh` builds. Upstream HumanoidArena
installs Isaac Sim 5.0; see [Setup](SETUP.md#3-simulator-image) for running on it instead.

## Separation

| Layer | Interpreter | HumanoidArena dependencies |
| --- | --- | --- |
| Agent CLI and its commands | `runtime/envs/host`, in the CLI's sandbox | none |
| Host harness (frontend, supervisor, gateway, workspace generator, dashboard) | `runtime/envs/host` | none: `services/humanoid/{benchmark,backend,process,tools,tasks,robot,task_rules,demonstrations}.py` use the standard library, msgpack and NumPy |
| Simulator (`python -m services.humanoid.server`) | `HUMANOID_PYTHON` | Isaac Sim 6.1, Isaac Lab, HumanoidArena, onnxruntime-gpu, cuRobo |

`HUMANOID_PYTHON` defaults to `scripts/humanoid_python.sh`, which runs the interpreter inside
the simulator image (`CONTAINER_CLI=docker` by default, or `podman`). The host talks to the
simulator over the shared msgpack RPC (`services/simulator/`).
`local_tests/unit/test_humanoid_backend.py` imports every host module with Isaac Sim, Isaac Lab,
the upstream task code, cuRobo and onnxruntime blocked.

## Setup

```bash
bash scripts/bootstrap_sources.sh                # HumanoidArena checkout, linked at HumanoidArena/
bash scripts/build_humanoid_image.sh --accept-nvidia-eula
bash scripts/setup_humanoid_assets.sh            # scene and robot assets, SONIC encoder/decoder, sha256-pinned
runtime/envs/host/bin/python scripts/setup_humanoid_demos.py   # three expert episodes per task (optional)
runtime/envs/host/bin/python scripts/humanoid_doctor.py --gpu 0 --task opendoor
```

`setup_humanoid_assets.sh` downloads from the pinned upstream URLs, or takes `--arena-zip` and
`--sonic-dir` copies that match the pinned hashes. The doctor starts one real simulator and
checks the camera, the joint layout, a standing hold, a cuRobo plan and the step speed; its
report goes under `runtime/diagnostics/humanoid/`. The full guide is [Setup](SETUP.md).

## Launching a session

```bash
runtime/envs/host/bin/python scripts/configure_auto_research.py --task opendoor \
    --sim-gpu 0 --episodes 16
bash scripts/start_auto_research_agent.sh --config <printed operator-input.json>
```

Defaults: 60 formal episodes (the official test episodes), 4 formal workers, no expert
episodes (`--expert-demonstrations 3` shows them; `--expert-content images` keeps only their
camera frames), no image demonstration. Task rules, the formal collection, handoff and
adaptation are fixed (see `fixed` in the registry entry in `services/benchmarks.py`). See
[Usage](USAGE.md) for every option.

## Tasks and layouts

`services/humanoid/tasks.py` holds the seven tasks with official test episodes, with their step
limits at 50 Hz: `opendoor` (1800 steps), `boxing_bag` (900), `pickplace_box` (1450),
`doubledesk` (2000), `sit_sofa` (2000), `football_single` (2000) and `warehouse_navigation`
(1800). The step limit is upstream's SONIC evaluation `MAX_STEPS`.

- **Formal (collection 1):** the 60 official test episodes, group seeds 0 to 2 with 20
  repeats each. Each seed is the last 4 bytes of sha256("<gym id>|<group>|<repeat>") masked to
  a positive int32, exactly upstream's `_derive_episode_seed`. The layout id is the episode
  index.
- **Exploration (collection 0):** layout ids map to seeds of the same distribution by a fixed
  derivation that never yields an official seed.

The environment resets its objects from the seed (`object_reset_seed_source: env_seed`). Every
episode is a fresh simulator process. `TASK.md` comes from
`services/humanoid/task_rules/<task>.md`: the starting scene, a brief description of what is
randomized, and readable success conditions, followed by generated episode and scoring sections.
Scoring is binary per episode: the formal result is the success rate.

Each episode also has a wall limit of 600 seconds plus 1 second per native step, and the
simulator has 900 seconds to start.

## What the agent sees and does

HumanoidArena's policy inputs only: the front camera, the 29 joint states, the root
orientation and the Dex3 finger state. No world position, map or object state.

| Tool | Contract |
| --- | --- |
| `humanoid_status` / `humanoid_observe` | episode, step budget, semantic40 layout, joint names and limits / cached front RGB (with depth, intrinsics and the camera pose in the pelvis frame for RGB-D), root orientation, joints, Dex3 fingers |
| `humanoid_step` | 1 to 100 semantic40 frames at 50 Hz (at most 2 s): root XY delta [0:2] and height [2] (ignored by SONIC), root rotation relative to the initial heading [3:9] (rot6d, row-major), 29 joint angles [9:38], binary hands [38:40]; frames past the step limit are cut |
| `humanoid_free_space_move` / `humanoid_execute_motion_plan` | a cuRobo plan of one arm (optionally the waist) to a palm, hand-centre or wrist pose in the pelvis or torso frame, self-collision only, executed as semantic40 frames that hold every other joint, the hands and the root rotation; `preview_only: true` plans without moving |
| `humanoid_pixel_to_position` | RGB-D only: depth pixels to points in the pelvis frame and the level frame (pelvis origin, z up, x along the heading) |

A step reply carries the final frame, `transition.feedback` (joint tracking error, root
rotation projection, tilt, fall) and `transition.steps[*].executed_action`; every 10th frame of
the motion is saved. An episode ends at task success, after the step limit, or at a fall
(upstream's rule: tilt above 75°, or above 60° with at least 50 N on a critical body, for five
steps). Only the binary `task_complete` leaves the host; the operator files keep the rest.

## Expert episodes

HumanoidArena released teleoperation episodes recorded through the same frozen SONIC
controller, on layouts of the same tasks (ModelScope `Twang2026/HumanoidArena_raw`, SONIC
branch, CC BY-NC 4.0). `scripts/setup_humanoid_demos.py` downloads three of them per task (the
episodes and their sha256 are pinned in `services/humanoid/expert_sources.json`), checks them
and installs them under `runtime/reference-demos/humanoid/<task>/experts/` with derived
per-frame views and previews.

With `expert_demonstrations: 3`, `runtime/experts/` in the agent's workspace links to the
task's three installed complete episodes, which the agent's sandbox reads and never writes:
raw arrays, a per-frame JSON view with a byte-offset index, front-camera video and previews.
Isolated rehearsal and formal runs never see them; a controller that needs something from
them must carry it in its bundle.

`expert_content: images` (`--expert-content images`, dashboard **Expert content**) shows the
camera-only set instead, for experiments without the expert's actions: per episode the
front-camera RGB video, the previews and the depth of every frame (`depth.npz`, float16
metres), with a metadata file of frame count, rate, image size and camera. The commanded
references, measured robot motion, hand commands, object states and layout are left out, and
the workspace text describes only what the frames show. Install the camera-only sets after the
complete episodes:

```bash
runtime/envs/host/bin/python scripts/setup_humanoid_demos.py --images
```

`demonstration_context` (`terminal_state` or `completion_sequence`) is separate: it copies
preview images of the same installed episodes (the last one, or all of them) into
`runtime/demonstrations/`, so it also needs the episodes installed.

## Simulator compatibility (Isaac Sim 6.1)

Project-side fixes, with the upstream files unchanged (`services/humanoid/sim/`):

- `compat.py`: the Kit 110 PreviewSurface shader-call rename, and the hardware DDS side channel
  disabled.
- `prim_reset.py`: PhysX does not follow USD edits of static colliders, so a reset that wrote one
  restarts physics and replays the identical seeded reset.
- `door_reset.py`: the inference profile spawns the OpenDoor door unmanaged; its physical state is
  restored at episode boundaries.
- `goal_collider.py`: the football goal collides as its exact triangle mesh; the 6.1-cooked convex
  decomposition puts an invisible 5.4 cm step across the goal mouth.
- The upstream assets log PhysX "triangle mesh … dynamic body, falling back to convexHull" for the
  door and the robot's ankle links at load. This is expected.
