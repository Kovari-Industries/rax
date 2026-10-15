# Adapting RAX to a new real robot

This guide explains how to bring the RAX harness from simulation to a physical robot. It
states five design principles and shows how two working ports apply them. It ends with
a bring-up checklist. The real-robot branches are the worked examples, so read them
alongside this guide.

| Branch | Robot | Vendor stack wired in | Planner wired in |
|---|---|---|---|
| `real/franka` | Franka FR3: one 7-DoF arm, Franka Hand, three ZED cameras | Polymetis HTTP bridges inside the DROID container; ZED SDK | none (Cartesian servo with an IK pre-check) |
| `real/rainbow` | Rainbow RB-Y1: wheeled base, 6-joint torso, two 7-DoF arms, Dynamixel grippers, head ZED 2i | `rby1_sdk` over gRPC, including the controller's own command modes (Cartesian, whole-body optimal control, impedance) | the controller's own IK and whole-body solver, through those modes (`services/rby1/controller_modes.py`) |

`real/rainbow` is the more complete port: whole-body and base moves, the controller's own
solvers, a self-collision and gravity-load check on every path, and a full
characterization record. Unless a path below says otherwise, it refers to the
real-robot branches.

## Contents

1. [The five principles](#the-five-principles)
2. [Architecture](#architecture)
3. [Principle 1: wire the control SDK, never modify it](#principle-1-wire-the-control-sdk-never-modify-it)
4. [Principle 2: measure every control mode, then write selection guidance](#principle-2-measure-every-control-mode-then-write-selection-guidance)
5. [Principle 3: universal skills, a sandbox and a separate Python environment](#principle-3-universal-skills-a-sandbox-and-a-separate-python-environment)
6. [Principle 4: observation on request and a continuous recording](#principle-4-observation-on-request-and-a-continuous-recording)
7. [Principle 5: keep the harness separate from the robot](#principle-5-keep-the-harness-separate-from-the-robot)
8. [Documenting a control tool](#documenting-a-control-tool)
9. [Bring-up checklist](#bring-up-checklist)
10. [Keeping robot branches in sync](#keeping-robot-branches-in-sync)

## The five principles

1. **Wire the vendor stack; never modify it.** The vendor control SDK, its firmware
   contract and its services stay as shipped. The robot reaches the agent only as MCP
   tools, implemented by a backend that calls the SDK. The agent *uses* the SDK and never
   *modifies* the robot. No tool writes parameters, sets gains or switches controller
   behaviour. The backend wraps the SDK's own solvers instead of writing its own.
2. **Measure before you advise.** Test every control mode on the hardware for accuracy
   and efficiency. The skills then tell the agent which mode suits which kind of motion,
   quoting the measured numbers.
3. **Give the agent universal skills and a sandbox.** Manual control, autonomous control,
   experience memory and continual learning are written once. They are robot-neutral and
   contain no task-specific advice. The agent works in a sandboxed workspace with its own
   Python environment, and what it learns accumulates in a skill library that persists
   across sessions.
4. **Observe on request; record continuously.** One tool returns a fresh, saved frame
   with its geometry described. A recorder keeps a high-rate sliding window on disk, and
   the agent can only read it.
5. **Keep the harness separate from the robot.** The harness (workspace generator, skills,
   launcher, sandbox, MCP frontend) knows nothing about the robot. Perception, control and
   planning (vendor planners, cuRobo, ...) are wired into the backend as SDKs. They reach
   the agent only through MCP.

## Architecture

```
 operator ── arm / disarm, replies ──► <session>/operator/   (files the agent cannot write)
                                              │ read every control tick
 agent CLI (sandboxed, cwd = agent-workspace) ▼
   ├─ MCP stdio ── mcp_bridge.py ──┐   robot service  services/controller/frontend.py
   └─ Bash / Python (sandbox) ─────┴──► Unix socket ─► (one process, one dispatcher thread)
        python code/... via api.runtime.Context          ├─ harness tools: status, request_operator, gemini_generate
                                                         └─ backend  services/<robot>/backend.py
                                                              ├─ vendor control SDK ── robot controller
                                                              ├─ planner SDK (vendor planner, cuRobo, ...)
                                                              ├─ camera SDK, kinematics (URDF)
                                                              └─ recorder ──► <session>/stream (read-only to the agent)
```

| Layer | Owns | Location |
|---|---|---|
| Robot backend | Everything robot-specific: MCP tool definitions, validation, safety checks, kinematics, cameras, planner calls, the recorder, dashboard telemetry | `services/<robot>/`, registered in `services/backends.py` |
| Operator frontend | Sessions, arming, operator requests, frames, read-only robot state | `services/dashboard/`, the `scripts/<robot>` CLI |
| Harness | Workspace generator, agent instructions and skills, launcher, sandbox, MCP frontend, Gemini broker | `services/workspace/`, `auto_research_agent/`, `agent_skills/`, `harness/`, `services/controller/` |

The robot service is the **only** process that commands the robot. It serializes calls
from the agent CLI and from the agent's scripts. It also writes an audit log of every
call (`<session>/mcp.jsonl`).

## Principle 1: wire the control SDK, never modify it

### What "wire" means

The backend is a thin adapter that checks everything. It translates an MCP call into SDK
calls, enforces limits before anything moves, watches the motion while it runs, and
reports measured results. It never forks, patches or monkey-patches the vendor package,
changes the robot controller's configuration, or alters message contracts.

| Do | Do not |
|---|---|
| Call the SDK's public API (for example `rby1_sdk`'s robot and command builders, or the Polymetis bridge's HTTP endpoints) | Edit the installed package, vendor a patched copy, or change firmware or server parameters |
| Override an unsafe default at the boundary, in the backend, and document why | Rely on side effects of construction (connecting must move nothing) |
| Pin the SDK version to the firmware and record the pair | Upgrade the vendor's own environment |
| Treat the vendor controller's own limits as the innermost safety layer | Bypass or loosen vendor limits |
| Wrap the SDK's own IK, planners and controller modes as tools | Write your own solver, planner or controller in the backend where the SDK already has one |
| Fix each mode's gains and solver settings in `DEFAULTS`, taken from the vendor's examples | Accept gains, stiffness, torque limits, solver settings or mode switches as agent arguments |

### The agent uses the SDK; it never modifies the robot

The agent decides *what* the robot does: which mode, which targets, and a speed below a
cap. It never decides *how the controller behaves*, and nothing it calls changes the
robot. Enforce this in the backend, not only in the skills:

- **No setter reaches the agent.** No tool calls any of these:
  - parameter writes or resets, factory resets
  - PID or gain setters
  - power, servo or brake control
  - control-manager enable, disable or fault reset
  - time scale, network or system time
  - LEDs, odometry reset, home offsets or model import

  Operator procedures that need any of these stay operator scripts, outside MCP.
- **No behaviour switches or tuning as arguments.** Modes that change how the controller
  treats the robot, such as gravity compensation and free-drive, are not wired. For the
  modes that are wired, these settings are backend options taken from the vendor's
  examples:
  - impedance stiffness and damping
  - torque limits
  - solver settings (stop costs, patience, scaling, tracking tolerances)

  Only the operator changes them, through `backend_options`.
- **Wrap, do not reimplement.** When the SDK or the controller offers IK, a whole-body
  solver, a planner or a Cartesian mode, the tool calls it. The backend does not write its
  own in its place. Where the controller solves the path itself, the backend checks the
  targets before the command. It also checks the measured state while the motion runs:
  limits, height, self-collision, torque and force. The tool description says that the
  joint path is the controller's.
- **Reading is fine.** Read-only queries can be tools: robot info, controller state,
  parameters and gains read back, fault logs, the controller log.
- **Unknown argument keys are refused**, never ignored. Otherwise the agent could believe
  it tuned something that was silently dropped.

`real/rainbow` is the worked example. `services/rby1/controller_modes.py` wires the
`CartesianCommandBuilder`, `OptimalControlCommandBuilder`, and the impedance, joint-group,
jog and stop builders, with their gains fixed in `DEFAULTS`. It leaves
`GravityCompensationCommandBuilder` and every setter unwired. It lists the controller
modes only after the operator enables them, because they are characterized first
(Principle 2).

More examples from the reference branches:

- **Franka** talks HTTP to the Polymetis arm and gripper bridges that run inside the DROID
  container (`services/franka/bridges.py`). libfranka's real-time threads never live in
  the harness process. A bridge `409` becomes `TargetRejected`, which tells the agent it
  hit a wall, not a fault.
- **Franka** corrects the bridge's steady-state sag with a host-side integral outer loop
  that sends `goal + bias` (`services/franka/servo.py`). It does not retune the bridge's
  impedance controller.
- **Rainbow**: connecting only opens the gRPC channel. Power, servo and the control
  manager are checked before every *armed* motion and brought up if they are off, after
  which the drives hold where they are. The vendor's helper that persists motion
  parameters to the controller's database is never called, so no robot parameter is
  written.
- **Rainbow**: a controller fault disarms the session, and the backend resets it only
  after the operator arms again on purpose. An emergency stop seen by the backend also
  disarms.
- **Rainbow** pins `rby1-sdk` to the version the controller reports, and upgrades the SDK
  only together with the robot firmware. The vendor's own environments stay as they are.

### The backend contract

`services/backends.py` defines the contract. The frontend never imports a backend
directly. It constructs the registered class once per session.

```python
class MyRobotBackend:
    mutating = frozenset({...})        # tools that can move hardware
    def __init__(self, config): ...    # config.backend_options override DEFAULTS; unknown keys raise
    def tools(self) -> list[dict]: ... # MCP definitions: name, description, inputSchema (additionalProperties: false)
    def validate(self, name, args): ...# raise before any motion: limits, reach, collision, arming
    def call(self, name, args) -> (value, images): ...
    def status(self) -> dict: ...      # merged into the harness `status` tool
    def close(self): ...               # release hardware; safe to call twice
```

- Prefix every tool with the robot key (`franka_*`, `rby1_*`). Backend tools may not
  reuse the names `status`, `request_operator` or `gemini_generate`.
- Put every tunable in one `DEFAULTS` dict at the top of the backend and expose it as
  `backend_options`. Skills quote these numbers.
- Register three things: the backend, its read-only dashboard telemetry (`TELEMETRY`),
  and its `ROBOT_FACTS` entry (`services/workspace/setup.py`).
- Keep the `mock` backend working. All automated tests run against it and never touch
  hardware.

### Safety layering (outermost first)

1. A physical e-stop and a person beside the robot.
2. Operator arming: a file under `<session>/operator/` that only the operator writes
   (`services/arming.py`; `<robot> arm` / `disarm`).
   - Motion tools refuse unless the robot is armed.
   - The backend re-reads the file **every control tick**, so a disarm stops a running
     motion within one period.
   - The agent cannot arm itself, because its sandbox cannot write there.
3. Backend checks:
   - joint limits minus a margin
   - per-call step caps and speed caps
   - a workspace box or end-effector height along the whole path (sampled with FK)
   - reach and self-collision
   - fresh state, a progress watchdog and a deadline
4. The vendor controller's own limits and watchdogs.

### Failure semantics (shared by every backend)

The frontend turns exceptions into one public error shape (`services/controller/errors.py`):
`{type, operation, stage, reason, episode_ended}`. `reason` is the complete message, with
credentials, URLs and host directories removed. Never replace an error with a summary
label such as "control uncertain": the agent needs the actual cause.

| Situation | Backend raises | Agent sees |
|---|---|---|
| Refused before motion | any exception in `validate` | `stage: validation` and the full reason |
| Stopped during motion (disarm, e-stop, stall, deadline, short of target, driver error) | an exception naming what happened, raised after cancelling and holding, with the measured `robot_state` when available | `stage: robot_execution`, the full reason and `robot_state` |
| Robot service or connection gone | an error carrying the exit or disconnect cause | the cause and `episode_ended: true` |

A `*_hold` tool that is always allowed, armed or not, is mandatory.

## Principle 2: measure every control mode, then write selection guidance

### Inventory the control modes

List every way the robot can move and give each one its own tool, so the agent picks the
semantics explicitly. Typical levels:

| Level | Franka | Rainbow | Notes |
|---|---|---|---|
| Joint space, one group | `franka_move_joints` | `rby1_move_joints` | postures, presets, retracting |
| Joint space, several groups | – | `rby1_move_body` | groups timed to finish together |
| Task space, one end effector | `franka_move_pose` (straight line) | `rby1_move_eef` (IK, then a joint path: **not** a straight line); `rby1_cartesian` (the controller's Cartesian command) | state the path semantics |
| Whole-body solver | – | `rby1_optimal_control` (whole-body IK on the controller) | the controller picks the joints |
| Dry run | – | `rby1_check` (the same checks as the motion tool, without moving) | check before moving |
| Gaze | – | `rby1_look_at` | head IK to a point |
| Mobile base | – | `rby1_base_move` (closed loop on wheel odometry) | state what closes the loop, and that it slips |
| End-effector actuation | `franka_gripper` (`open` / `move` / `grasp`) | `rby1_gripper` (open, current-limited close, width) | position and force grasps hold differently |
| Stop | `franka_hold` | `rby1_hold` | always allowed |

### Characterization protocol

Run the protocol through the **real backend**, with the same guards the agent gets, and
with an operator at the e-stop. Take one small step per arming, and return each step to
its start pose. `scripts/rby1_motion_check.py` and `scripts/rby1_characterize.py` on
`real/rainbow` are the templates: they check that a disarmed motion is refused, then
exercise every joint group, the end-effector and gaze tools, the grippers, the base and a
disarm in mid-motion. Re-run them after any firmware or SDK upgrade.

Measure these quantities for every tool:

| Quantity | How | Why the agent needs it |
|---|---|---|
| Final error | the measured pose (FK of the measured joints) against the target, position in mm and orientation in degrees; add an external check with a camera where feasible | to choose tolerances and know when a correction step is needed |
| Repeatability | the same move N times from the same start | to know whether replaying a recorded pose is trustworthy |
| Settle time and duration | wall time from call to reply, by move size | to plan the budget: fewer large calls or many small ones |
| Call overhead | the duration of a zero-size or tiny move, and of `status` / `observe` | to know how fast a closed loop in a script can run |
| Stop latency and overrun | disarm mid-motion: time to cancel, distance past the stop, hold drift over 3 s | to confirm the safety layer; quoted in the skill |
| Path deviation | FK sampled along the path against the straight line (task-space tools) | to know whether a tool is safe near obstacles |
| Failure behaviour | targets just outside each limit, unreachable IK, an infeasible planner goal | to know the exact refusal text the agent will see |
| Open-loop drift | for a base: commanded against observed displacement | to know how much to re-localize after driving |

Record the results, with the date and the firmware and SDK versions, in the robot's
documentation. Examples:

- Franka `franka_move_pose` with the integral servo: 10 mm moves, 3–5 cm moves and ±15°
  rotations settle within 1 mm and 0.5° in 1–6 s. The bridge rate limits are 50 mm/s and
  0.5 rad/s.
- Rainbow: a disarm cancels 0.08 s after the call; the controller then decelerates
  0.02–0.04 rad further and holds, drifting 0.0001 rad over 3 s. `rby1_move_eef` ends
  within 0.2–1.8 mm for 2–20 cm moves, repeats to a 0.09 mm spread, and its path stays
  within 6.7 mm of the straight line for 5 cm moves. `rby1_status` takes 8 ms and
  `rby1_observe` 63 ms. Driving 0.3 m reads 0.295 m on the wheel odometry.

Findings change the backend, not only the documentation. Examples from Rainbow:

- The controller kept each command for 1.0 s after reaching its target, which added
  0.8 s to every move with no gain in accuracy. The backend now uses 0.2 s.
- A hold command sent after a cancel is cancelled with it and would pull the joints back,
  so a stop only cancels and lets the controller hold.
- Base arrival tolerances of 2 cm stopped 5 cm moves at 3.1 cm; they are now 5 mm.
- After a torso joint met a real end stop inside its URDF limit and its drive faulted,
  the backend gained a squat limit, wider limit margins, a joint-torque guard during and
  between motions, a gravity-load check along every path, and in-place re-targeting of a
  joint left short of its target.

### Selection guidance

Turn the measurements into a scenario table in the robot's `robot-interface` skill. For
mobile or multi-limb robots, also use `whole-body-control` and `base-navigation`. Scenarios are *kinds of motion*, never tasks:

| Scenario | Prefer | Because (quote measurements) |
|---|---|---|
| Large reach around the body, or two arms together | a dry run, then a whole-body or planned move | collision and reach are verified before moving; one smooth motion |
| Short correction near the target or a contact | a task-space step with a small size | straight or near-straight, small error, measured result returned |
| Posture change, retract, presets | joint space | predictable path; no IK branch flips |
| Several body parts | whole-body joint move | the parts finish together; the slowest part sets the pace |
| Pointing a camera | the gaze tool | the camera is centred where depth is best |
| Changing where the robot stands | a base move with the arms stowed, then re-observe | open loop: positions measured before are stale |
| Holding an object | a force grasp, not a position move | a position-only close lets the object push the fingers open |
| Anything unexpected | hold, then observe | handled stops leave a measured state to plan from |

A rule of thumb for the skill text: *choose the highest level that expresses the intent*.
State each tool's path semantics in its description.

## Principle 3: universal skills, a sandbox and a separate Python environment

### Skill layout

Skills live once under `agent_skills/`. `services/workspace/` layers them per session
(`SKILL_LAYERS` in `recipes.py`). Agents see a flat `.agents/skills/` with robot-neutral
names. Task skills can therefore link to `robot-interface` or `object-localization`
without branching on the robot.

| Layer | Skills | Robot-specific? |
|---|---|---|
| `task/` | `manual-robot-control`, `autonomous-control`, `experience-memory`, `continual-learning` | no |
| `perception/common/` | `live-stream`, `wrist-camera-inspection` | no |
| `perception/<depth\|rgb-only>-<robot>/` | `object-localization` | yes |
| `control/<robot>/` | `robot-interface`, plus `whole-body-control`, `base-navigation` and `grasping` where they apply | yes |
| `auto-research/` | `gemini` | no |

Robot facts reach shared skills only through `ROBOT_FACTS[<robot>]` in
`services/workspace/setup.py`, rendered as `robot.*`. The keys are `name`, `body`,
`status`, `observe`, `move`, `move_joints`, `gripper`, `pixel`, `stream`, `cameras`,
`overview_camera`, `neutral`, `motion_example`, `grasp_example` and `wrist_camera`.
Skills that need a missing capability are gated off with a `SkillRule`; for example,
there is no `live-stream` skill without a recorder. Add a key there rather than an
`if robot == ...` in a template.

### The universal high-level skills

Each skill is a workflow contract. Write them precisely and goal-oriented, without
examples from any particular task. Robot details belong in `control/<robot>/`.
[`skills/`](../skills/README.md) holds robot-agnostic versions of these skills, ready to
copy, with a `robot-interface` template and each branch's version linked.

| Skill | Contract |
|---|---|
| `manual-robot-control` | Solve the task by hand from observations. Read `TASK.md` into a completion checklist, then inspect status and images. Ground each target in fresh perception, and take small steps that can be inspected. Distinguish commanded from measured state, and treat refusals as walls. After any error, observe before the next motion. Confirm every condition from fresh images of more than one view. Record the strategy in memory. |
| `autonomous-control` | Turn what worked by hand into `code/<project>/controller.py:main(ctx)`. It is built from step functions with observable preconditions and completion checks that return `{ok, measured, why}`. It re-localizes on every run, keeps tested ranges as named constants, can resume, and leaves a safe stop state on failure. Test it end to end: start from the neutral pose, ask for a `request_operator` reset, run unattended once per committed version, and log every run in `memory/RUNS.md`. Runs finished by hand do not count. |
| `experience-memory` | Task-scoped memory in `memory/`. Record a tip in `TIPS.md` as soon as an action visibly succeeds: observable preconditions, phase, tool and arguments or pose relation, effect, and frame paths. Refine the same row later. Promote reusable procedures to `memory/skills/<slug>/` with `evidence.json`, and index them in `INDEX.md`. Retrieve only the tips that match. Live perception and tool schemas override memory. On real robots, record the conditions under which a tip held, since the scene drifts. |
| `continual-learning` | Read, run and grow the robot's skill library (below). At the start, load only the library skills that match `TASK.md`, check their preconditions and run them. Before the session ends, contribute what worked as skills with code and evidence, and merge them so the next session finds them. Library skills never override a limit, a safety rule or a tool. |
| `live-stream` | When and how to read the recording (Principle 4). |
| `robot-interface` | Read first. The robot's tool map by level, its frames, the enforced limits with their values, the error contract, arming, vendor safety rules and script access. |
| `object-localization` | Pixels to metric points for the observation profile. Surface points are not grasp centres. Re-observe instead of guessing. |

Every robot gets these harness tools:

- `status`: task, budgets, `episode_ended`, and the merged robot status.
- `request_operator`: `help` or `reset`. It blocks until the operator answers, and the
  robot holds in the meantime.
- `gemini_generate`, when a key is configured.

The agent's top-level instructions (`auto_research_agent/AGENTS.md`) stay a short
workflow:

1. Safety first.
2. Solve the task by hand.
3. Automate it.
4. Test end to end with operator resets.
5. Keep `memory/PROGRESS.md` current, so the agent can continue after context compaction.

### A skill library that persists across sessions

`memory/` lives as long as one task. What an agent learns about the robot should outlive
it, so every real robot gets a **skill library**: verified, executable sub-task skills
that later sessions read, run, compose and improve. Give a new robot one from its first
session, so nothing it learns has to be imported later.

- **Where it lives.** The shared libraries are in
  [skills-lib](https://github.com/robot-agent-exploration/skills-lib), one branch per
  embodiment. Its [CONTRIBUTING.md](https://github.com/robot-agent-exploration/skills-lib/blob/main/CONTRIBUTING.md)
  defines the skill format (`SKILL.md` with seven sections, `code/` with an entry
  `main(ctx, ...) -> {ok, measured, why}`, and `evidence/` with run records and frames),
  the status rules and the checker `scripts/check_skills.py`.
- **How the harness wires it.** Keep a local library per robot under the artifact root
  (`<artifact root>/skill-library/<robot>/`), cloned from the robot's skills-lib branch.
  Each session gets a git clone of it as `skill-library/` in its workspace and works on
  its own `session/<session>` branch. A harness tool (`<library_merge>`) runs the checker
  and merges that branch into the local library; the agent never pushes on its own.
  Publish the merged library to skills-lib as a pull request into the robot's branch,
  and merge that branch back in before sessions so the robot starts from what other labs
  learned ([CONTRIBUTING.md section 5](https://github.com/robot-agent-exploration/skills-lib/blob/main/CONTRIBUTING.md#5-commit-and-merge)).
- **The setup skills read.** A skill is reused on the same robot with other cameras,
  layouts and tasks, so it reads the setup from the harness instead of copying it. Give
  every frame its camera intrinsics and extrinsics (Principle 4), and report limits, the
  workspace and named postures (such as home) through the tools.
- **Skill layout and sandbox.** Render `continual-learning` into `.agents/skills/` and
  gate it with a `SkillRule` on the library being present. The sandbox lets the agent write
  its workspace clone, never the local library or the remote.
- **Adding the robot to skills-lib.** Create its embodiment branch as described in
  [NEW_ROBOT.md](https://github.com/robot-agent-exploration/skills-lib/blob/main/NEW_ROBOT.md).

`real/rainbow` is the reference: `skill_library_merge` and its `continual-learning`
skill.

### The sandbox

The agent CLI runs with its own sandbox and an explicit allow list. On the reference
branches this is Claude Code on the host (`harness/claude_cli/launcher.py`) with
`--permission-mode dontAsk`:

- Read and Edit only inside `<session>/agent-workspace/`, plus Read of `<session>/stream/**`.
- Bash runs in the CLI's bubblewrap sandbox (`failIfUnavailable`, no unsandboxed
  commands).
  - Writes go only to the workspace.
  - The internet is reached through the sandbox's HTTP(S) proxy.
  - Host loopback stays closed, so vendor bridges on `127.0.0.1` are unreachable.
- `denyRead` covers:
  - the repository and the robot-controller checkouts
  - credentials (`~/.ssh`, ...)
  - the session's private files (`configuration.json`, `state.json`, `mcp.jsonl`, ...)
  - anything that would let a script talk to the robot directly (for Rainbow: the
    robot-controller checkouts and the vendor SDK environments)
- Linux sandboxes allow either all Unix sockets or none. Host sockets that would bypass
  the robot service are therefore hidden instead: docker/containerd (the robot
  containers), dbus, X11, `/run/user/<uid>`, and the camera server's private socket.
- The sandbox's proxy refuses the robot controller's own address, even though other
  internet hosts are allowed. Rainbow's controller serves unauthenticated gRPC, so a
  script with the vendor SDK could otherwise command the robot past the backend.
- The robot service brokers Gemini, with a per-session spending cap.

When you add a robot, add every path or address that grants direct robot access (SDK
configurations, certificates, controller checkouts, sockets, controller hosts) to
`PROTECTED_PATHS`, `HIDDEN_SOCKETS` or `DENIED_HOSTS`.

### A separate Python environment for the agent

The agent's scripts reach the robot only through `api.runtime.Context`, a stdlib MCP
client on the robot service's socket. They need **no vendor SDK**, so give the agent its
own environment:

- Create a dedicated venv (for example `runtime/envs/agent`) with general tools (numpy,
  opencv, scipy, ...) and **without** the control, planner or camera SDKs.
- Put its `bin/` first on the sandbox `PATH`, and set `PYTHONPATH` to the workspace.
- Keep three environments distinct:
  - the vendor's, left untouched
  - the robot service's, with SDK versions matched to the firmware
  - the agent's

### Subagents and models

Give the agent the CLI's native subagents and let it choose the model per job: a fast
model for structured perception (yes or no, where, how many), and a stronger one for
open-ended understanding or review. Subagents get the agent's tools, including the
robot. The backend's arming and guards apply to every call, and the instructions keep one
agent commanding the robot at a time.

If you put a relay between the agent CLI and its model provider, keep it transparent:

- Pass every request and response through as the native client would send and receive
  it: all headers in both directions, the body, and the provider's status and error
  message.
- Replace only the credential, and set only the framing headers the relay must own.
- Never filter models or headers to an allowlist. Provider features depend on the
  client's headers. For example, Codex's `session-id` routes a conversation to its prompt
  cache. Claude Code resolves its model aliases for subagents to ids that a short
  allowlist misses.
- Answer every refusal with a body that names the reason.
- Check a relay change on real calls: the cached-input share and the cost per call must
  match the native client's.

## Principle 4: observation on request and a continuous recording

### On-request observation

| Tool | Contract |
|---|---|
| `<robot>_status` | No motion. Joints, end-effector poses (base frame, from FK), gripper, arming, e-stop, limits, and which sensors are delivering now. |
| `<robot>_observe` | Captures now and returns downscaled previews (`agent_image_width`, default 640) plus the measured state. It saves full resolution to `runtime/frames/<frame_id>/`: `<camera>.jpg`, `<camera>_depth.png` (uint16 mm, 0 = invalid, RGB-D profile only), and `meta.json` with the capture time, joints, and per-camera `K`, `T_base_cam`, width and height. It reports per-camera errors instead of failing silently. |
| `<robot>_pixel_to_position` | Base-frame 3-D points for full-resolution pixels of a saved frame, using the median valid depth in a `(2r+1)²` window. |

Requirements:

- An observation is the **current** image. Refuse a frame older than a set age (Rainbow:
  `camera_max_age_s`, 1 s) rather than serve a stale one.
- The camera pose comes from one of three sources:
  - FK of the joints captured with the frame, composed with a hand-eye calibration, for
    cameras on moving links (the Rainbow head, `services/rby1/kinematics.py`)
  - extrinsic calibration, for fixed cameras
  - a calibrated hand-eye transform composed with FK, for wrist cameras (Franka,
    `services/franka/calibration.py`)

  Use an optical frame (z forward, x right, y down). Verify it by projecting known FK
  points into the image.
- Heavy camera work can run in a child process that is pinned to CPUs and killed with its
  parent (Franka `camera_server.py`, `PR_SET_PDEATHSIG`). Put it behind a socket that is
  hidden from the sandbox.

### Continuous recording

A recorder in the robot service **only reads**. From session start, it writes a sliding
window (`services/stream.py` defines the layout and the readers):

```
<session>/stream/
  <camera>/<UTC YYYYmmddTHHMM>/<epoch_ms>.jpg          RGB
  <camera>/<UTC YYYYmmddTHHMM>/<epoch_ms>_depth.png    uint16 mm, about 1 Hz
  robot/<UTC YYYYmmddTHHMM>.jsonl                      joints and end effectors with t_ms
  index.json                                           cameras, newest frame per camera, measured rates
```

- Rates are backend options. Franka records 30 Hz per camera and Rainbow about 10 Hz from
  the head camera. Both record depth at 1 Hz and robot state at 10 Hz, over a 600 s window.
- Files are written under a dot-name and then renamed, so readers never see partial files.
  Whole minutes older than the window are pruned.
- The launcher symlinks `agent-workspace/runtime/stream` to `<session>/stream` and grants
  Read only.
- The agent reads the recording in two ways:
  - the `<robot>_stream` tool: frames from `seconds_ago` with the robot state at that
    time, or a `sheet` contact sheet per camera
  - `python -m api.stream` (`changes`, `robot`, ...) from code
- An optional whole-session archive (Franka `services/franka/archive.py`) writes one
  H.264 MP4 per camera per minute, with a sidecar of exact frame times, for replay
  alongside the transcript. It is for the operator, not the agent.

The `live-stream` skill tells the agent three things:

- check the stream before any motion that follows a pause
- review a motion or a script run from its frames
- take metric geometry from `observe` frames rather than stream frames

## Principle 5: keep the harness separate from the robot

### What goes where

| Concern | Lives in | Never in |
|---|---|---|
| Vendor SDK calls, kinematics, the collision model, planner calls, camera drivers, the recorder, limits | `services/<robot>/` | the harness, the skills, the agent workspace |
| Robot facts the shared text needs | `ROBOT_FACTS` | `if robot == ...` in shared templates or code |
| Robot tool docs and selection guidance | `agent_skills/control/<robot>/`, `perception/*-<robot>/` | `task/` skills |
| Operator commands (arm, reply, ship) | `scripts/<robot>`, the dashboard | MCP tools |
| Setup lessons, versions, measurements | `<ROBOT>.md`, `docs/<robot>/` | agent skills (quote only what the agent needs) |

The harness must run unchanged against `mock`. If a new robot needs a harness change,
make the change robot-neutral and test it with `mock`.

### Planners and perception modules as SDKs

Wire planning (vendor planners, cuRobo) and heavy perception into the backend the same
way as the control SDK:

- Call their APIs. Do not fork them or change their contracts.
- Do not write a substitute in the backend when the vendor already provides one,
  including solvers that run on the robot's controller.
- The agent never imports them. Its scripts call the MCP tools.

Neither reference branch wires a separate motion planner: Franka moves in short checked
steps, and Rainbow uses the controller's own Cartesian and whole-body solvers. If you add
a planner, follow this pattern:

1. **Plan without moving.** No arming is needed. The inputs are end-effector poses or
   joint goals. The joints not being planned are locked at their measured values.
2. **Re-verify independently in the backend.** Check the final pose by FK, the joint
   limits, and the self-collision of every waypoint with the vendor's collision model.
   List any `problems`; if there are any, the plan is not executable.
3. **Store the plan** under an ID. It is valid for a fixed time and only from its start
   state.
4. **Execute** the plan as one timed trajectory on the robot's own trajectory interface,
   with the same per-tick arming, e-stop, freshness and progress checks as every motion.
   Refuse if the plan has problems or has expired, or if the robot has left its start.
5. **Report** planned and actual duration, and the final joints and poses.

Offer the planner's strategies as an explicit argument, and say in the reply which one
produced the plan. Let the same collision check guard the motion tools that do not plan,
so a straight joint move into self-collision is refused with a pointer to the planner.
Rainbow's `rby1_check` applies the first two steps to every motion tool: it runs the
same checks as the real call and reports the planned time, without moving.

The simulation branches wire cuRobo the same way, and the same distinctions hold on
hardware:

- planner samples are neither executable actions nor measured states
- a separate FK check rejects inaccurate plans
- execution reports whether the goal was measurably reached, separately from whether the
  command completed

### Storage

Sessions, frames, the stream and logs go under the artifact root (`runtime/`, ideally on
an SSD), never in the source tree. On a robot with one small disk, ship finished runs off
the robot. `services/runtime_ship.py` on `real/rainbow` does this in steps:

1. zip the run without secrets
2. rsync it off the robot
3. verify the sha256
4. free the local disk

Interrupted runs stay in place for `--resume`.

## Documenting a control tool

Each motion tool needs two texts:

- **The tool description and schema** own the exact fields, units, frames, per-call
  limits and reply fields. Interpolate the limits from `DEFAULTS` so they cannot drift.
- **The skill** owns tool choice, workflow, interpretation and non-obvious behaviour.

For every tool, the robot's documentation and skills together must answer these items:

| Item | Example (Rainbow `rby1_move_eef`) |
|---|---|
| Purpose and level | moves one arm's end effector to a base-frame pose; task space |
| Inputs, frames, units | `arm`, `position` (m, base frame), `quaternion` `[x, y, z, w]` (omitted: keep current), `relative` |
| Preconditions | operator arming, emergency stop released, fresh joint state |
| Enforced limits | at most 0.25 m per call; target within reach of that shoulder; end effectors above z = 0.10 m along the path; joint limits minus 0.1 rad; self-collision and gravity load along the path |
| Execution | IK on that arm only (torso, head and the other arm hold), then a joint-space move with acceleration ramps; the path is **not** a straight line |
| Watched while running | at 50 Hz: arming, emergency stop, state freshness, joint torque, wrist force, progress (2 s stall) |
| Termination | settled within 0.02 rad, a joint left short re-targeted where it is, or a handled stop |
| Reply | measured pose and its error |
| Failure modes | refusal (`validation`), a stop with the full reason and `robot_state`, a service exit |
| Measured accuracy and time | 0.2–1.8 mm for 2–20 cm moves; 0.09 mm spread over 5 repeats; path within 6.7 mm of straight for 5 cm |
| Use / avoid | short reaches clear of obstacles; for straight paths near contact, prefer the controller's Cartesian command |

Write numbers that come from the code, configuration, kinematics or measurements, never
estimates. If a vendor manual or the lab imposes a rule, enforce it in the backend *and*
explain it in the skill. For example, Rainbow keeps the torso upright by a fixed joint
relation, and refuses to drive the base with the torso low, leaning, or a hand stretched
out.

## Bring-up checklist

1. **Map the system.** Which computer runs the controller and which serves each sensor,
   network addresses, and the firmware version.
2. **Pick SDK versions for the firmware.** Create the robot-service environment on top of
   the vendor's without changing it. Create the agent environment without any SDK.
3. **Branch from the closest reference port** (see below). Add
   `services/<robot>/backend.py` and register it. Add its `ROBOT_FACTS`, its telemetry,
   and `scripts/<robot>` (arm, disarm, status, requests, reply).
4. **Connect without moving.** Verify that constructing the backend commands nothing.
   Override unsafe defaults at the boundary. Require fresh state for every check.
5. **Write unit tests with fakes** (`local_tests/unit/test_<robot>_backend.py`). Cover
   limits, refusals, handled stops, the error shape, tool schemas and skill rendering. No
   hardware.
6. **Run supervised characterization** (Principle 2) through the real backend. Check that
   a disarmed motion is refused, disarm mid-motion, and exercise every joint group, every
   tool and the base. Record the results.
7. **Check observation.** Check the intrinsics, `T_base_cam` (pixel-to-position on known
   points), depth validity, and the stream rates in `index.json`.
8. **Check the planner.** Plan without moving. Inject collisions and unreachable goals to
   see the `problems` text. Execute small clean plans.
9. **Write the skills.** Write `robot-interface` and the robot's control and localization
   skills from the measured numbers. Render a workspace and read it as the agent will.
10. **Set up the skill library.** Create the robot's branch in skills-lib, clone it as the
    local library, render `continual-learning`, and check that a session can merge a
    `draft` skill through `<library_merge>` (Principle 3).
11. **Audit the sandbox.** From inside an agent session, confirm that the agent cannot
    read the repository, credentials or SDK configurations, cannot reach vendor sockets or
    loopback bridges, and cannot write the arming file or the local skill library.
12. **Run a dry agent session** with a person at the e-stop, then move on to real tasks.

## Keeping robot branches in sync

Each robot lives on its own branch, which removes the simulators. Content on a robot
branch falls into two groups:

- **Robot-only content**, which stays on that branch: `services/<robot>/`,
  `agent_skills/control/<robot>/`, `agent_skills/perception/*-<robot>/`,
  `scripts/<robot>*`, `<ROBOT>.md` and `docs/<robot>/`.
- **Harness improvements**, which are robot-neutral and can move between branches: the
  launcher and sandbox, operator requests and handled-stop semantics, the workspace
  generator, robot-neutral `ROBOT_FACTS` keys, the stream layout and readers, dashboard
  telemetry, the runtime shipper, the skill-library merge tool, and the generic skills.

Start a new robot from the reference port closest to it: `real/franka` for a single arm,
and `real/rainbow` for a mobile or multi-limb robot. Remove the other
robot's backend and skills. Keep the harness robot-neutral, so improvements can flow
between the robot branches and the simulation branches.
