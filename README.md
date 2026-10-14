# RAX on a real Franka FR3

**[Project website](https://robot-agent-exploration.com/)** · [Paper](https://robot-agent-exploration.com/rax.pdf) · [All branches](https://github.com/robot-agent-exploration/rax)

This branch runs *RAX: Scaling Online Exploration for Robot Agents* on a real Franka
FR3 arm with a parallel gripper and ZED cameras. A Claude Code agent learns a
manipulation task through MCP tools served by a host-owned robot service. It first
solves the task by hand, then writes and tests its own controller, keeping notes and
verified skills in its workspace.

The simulation branches and the [real-robot adaptation guide](https://github.com/robot-agent-exploration/rax/blob/main/docs/REAL_ROBOT_ADAPTATION.md)
(design principles for bringing the harness to another robot) are linked from the
[index](https://github.com/robot-agent-exploration/rax).

## How it works

```
 Claude Code (host process, cwd = <session>/agent-workspace)
   ├─ MCP stdio ── mcp_bridge.py ──┐
   └─ Bash (bubblewrap sandbox) ───┴─ Unix socket ── robot service (services/controller/frontend.py)
                                                       └─ Franka backend ── HTTP bridges ── Polymetis ── FCI
                                                       └─ camera server (ZED cameras, recording)
 Claude Code ── http://127.0.0.1:<port> ── messages_relay.py (holds the model credential)
```

- **One robot owner.** The robot service is the only process that commands the robot.
  It serializes every call from the agent CLI and from the agent's own scripts, and
  writes an audit log (`mcp.jsonl`).
- **The workspace is the agent's only writable place.** Claude Code runs with
  `--permission-mode dontAsk`, so its file tools reach only the workspace. Bash runs in
  Claude Code's sandbox:
  - internet goes through the sandbox's proxy, but there is no host loopback, so the
    robot bridges and the dashboard are unreachable;
  - there is no GPU, and writes land only in the workspace;
  - it cannot read this repository, credentials, your robot stack (see
    [step 5](#5-protect-the-robot-stack)), docker/dbus/X11 sockets or the session's
    private files.
- **No credentials in the agent.** The relay injects the model credential. Gemini is
  brokered by the robot service, with a $10 cap per session.
- **Layered safety**, outermost first:
  1. the physical emergency stop and a person beside the robot;
  2. operator arming: motion tools refuse unless the session is armed, and a running
     motion holds within one servo period once the operator disarms;
  3. the backend: a TCP workspace box, checked on the goal and on every servo tick;
     joint moves checked by forward kinematics along the whole path; step limits;
  4. the bridge: rate limits, IK and joint-limit checks, and a command watchdog that holds
     the arm.

## Requirements

- **Robot:** a Franka FR3 with FCI, driven by [Polymetis](https://github.com/facebookresearch/fairo/tree/main/polymetis)
  (for example inside the [DROID](https://github.com/droid-dataset/droid) software
  stack). In front of Polymetis run two small HTTP bridges, one for the arm and one for
  the gripper, that implement the [bridge contract](#bridge-contract). The bridges own
  the Polymetis connections, so no realtime thread runs in this repository's processes.
  The bridges are not part of this repository.
- **Cameras:** ZED cameras with the ZED SDK and its Python API (`pyzed`), each with an
  extrinsic calibration. The default skills expect two fixed cameras and a wrist ZED Mini.
- **Host:** Linux (a PREEMPT_RT kernel if your Franka setup needs one) with:
  - `ffmpeg` (NVENC when available, else libx264) for the session archive;
  - `bubblewrap` and `socat` for Claude Code's sandbox;
  - [Claude Code](https://docs.claude.com/en/docs/claude-code/setup), installed natively.
- **Robot Python environment:** Python 3.10 or newer with `numpy`, `scipy`,
  `opencv-python`, `pyyaml`, `jinja2`, `pillow`, `pytest` and `pyzed`. The robot service,
  dashboard and tests run in it. The unit tests were run with Python 3.11.
- **A model credential:** a Claude account token, an Anthropic API key or an OpenRouter
  key (see [step 7](#7-agent-cli-and-credentials)). Optionally, an OpenRouter key for
  Gemini.

## Setup

### 1. Clone and choose where sessions live

```bash
git clone -b real/franka https://github.com/robot-agent-exploration/rax.git rax-franka
cd rax-franka
ln -s /large-disk/rax-franka-runtime runtime   # or: export RAX_ARTIFACT_ROOT=/large-disk/...
```

Sessions, logs and camera recordings go under the *artifact root*: `$RAX_ARTIFACT_ROOT`,
else the repository's `runtime/` directory. A session records every camera, so use a
disk with room (about 0.6 GB per hour for three cameras, plus a 10-minute raw window).

### 2. Robot Python environment

Create the environment with the packages listed above (install `pyzed` with the ZED
SDK's installer), then point the scripts at it:

```bash
export RAX_PYTHON=/path/to/robot-env/bin/python
```

Every script in `scripts/` uses `$RAX_PYTHON`, else the first `python3` on `PATH`.

### 3. Robot side

Start Polymetis and the two bridges, then check that both answer:

```bash
curl -s http://127.0.0.1:50054/state    # arm
curl -s http://127.0.0.1:50053/state    # gripper
```

The backend options `arm_url` and `gripper_url` change these addresses for sessions. The
dashboard and the `franka` operator command always use the defaults above.

Never run a second arm controller (for example franky) while Polymetis owns FCI.

#### Bridge contract

Both bridges speak JSON over HTTP. A target the arm bridge refuses (joint limit, IK,
another controller active) answers HTTP 409; the agent sees it as a refusal, not a
fault. Any other error status, or a JSON body with an `error` field, is a bridge error.

| Bridge | Request | Body | Answer |
| --- | --- | --- | --- |
| arm | `GET /state` | | `arm_pos` (flange/link8 position in the base frame, m), `arm_quat` ([x, y, z, w]), `arm_joint_position` (7 joints, rad), `policy_running` (bool) |
| arm | `POST /target` | `position`, `quaternion` (flange target) | any JSON; the bridge rate-limits (50 mm/s, 0.5 rad/s), IK-checks and joint-limit-checks the target |
| arm | `POST /hold` | `{}` | any JSON; the arm holds its pose |
| arm | `POST /move_joints` | `positions` (7), `time_to_go` (s) | any JSON, after the move |
| gripper | `GET /state` | | `width`, `max_width` (m), `is_grasped`, `is_moving`, `stale` (true when the gripper client died) |
| gripper | `POST /move` | `width`, `speed`, `force`, `blocking` | the gripper state |
| gripper | `POST /grasp` | `width`, `speed`, `force`, `blocking` | the gripper state |

The arm bridge must also hold the arm when no command arrives for 0.5 s (a command
watchdog). The backend's servo loop sends targets at 20 Hz while a move runs.

### 4. Cameras and calibration

Each camera's extrinsics is a YAML file with one 4×4 homogeneous `matrix`:

- a fixed camera stores `T_cam_base` (P_cam = T · P_base, base frame `fr3_link0`);
- the wrist camera stores `T_cam_gripper` against `fr3_hand_tcp`.

Put the files in one directory, then copy `examples/franka.json` and fill it in:

- each camera's `name`, ZED `serial`, `mount` (`fixed` or `tcp`) and `extrinsics` file,
  relative to `FRANKA_CALIBRATION_DIR` or absolute;
- `camera_cpus`: the CPUs the camera server is pinned to (`taskset -c`). Choose cores
  that no libfranka or realtime thread uses. In NEURAL depth mode each ZED spins a thread
  pool over every CPU it may use, and unpinned cameras can starve a realtime kernel;
- `workspace_min` / `workspace_max`: the TCP box in the base frame, in metres, above your
  table.

```bash
export FRANKA_CALIBRATION_DIR=/path/to/calibration
export RAX_BACKEND_OPTIONS=/path/to/franka.json
```

`RAX_BACKEND_OPTIONS` is the default `--backend-options` for every new session,
including sessions created in the dashboard. A Franka session without cameras is
refused when it is configured. `DEFAULTS` in `services/franka/backend.py` lists every
other option (speeds, step limits, recording rates) with its default, and an unknown
option is refused.

### 5. Protect the robot stack

The agent's Bash sandbox cannot read this repository, `~/.config/rax`, Claude/Codex
homes, SSH/AWS/gcloud credentials or `FRANKA_CALIBRATION_DIR`. List every other path
that would let a script reach the robot directly: controller checkouts, robot SDK
environments and their configurations.

```bash
export RAX_PROTECTED_PATHS=$HOME/droid:/opt/polymetis-env     # colon-separated, absolute
```

A session's backend option `protected_paths` (a list) adds more. The sandbox settings
are written when a session is first launched, so set these before that. A relative path
is refused.

### 6. The agent's Python (optional)

The agent's scripts reach the robot only through `api.runtime.Context`, a stdlib MCP
client on the robot service's socket, so they need no vendor SDK. By default the
agent's Bash finds the robot environment's Python first on `PATH`. To give the agent its
own environment (for example `numpy`, `scipy`, `opencv-python`), set the backend option
`agent_python` to that interpreter.

### 7. Agent CLI and credentials

The relay holds the credential; the agent never sees it. Choose the provider with
`--agent-provider` when you configure a session:

| Provider | Credential | Default model |
| --- | --- | --- |
| `claude-login` (default) | a long-lived Claude Code token: `bash scripts/setup_claude_experiment_login.sh` signs in to a separate experiment account in its own config directory and stores its `claude setup-token` token in `~/.config/rax/`. `--personal` uses your own signed-in account instead. | `claude-opus-5-5` |
| `anthropic` | an Anthropic API key file, passed to the launcher with `--key-file` | `claude-opus-5-5` |
| `openrouter` | an OpenRouter key file, passed with `--key-file` | `anthropic/claude-opus-5.5` |

Key files must be readable only by you (mode 0600). `--agent-model` overrides the model.

**Gemini (optional).** With an OpenRouter key at `~/.config/rax/openrouter.key` (0600),
the agent gets `gemini_generate` (Gemini 3.8 Flash), capped at $10 per session. The key
stays with the robot service. `--gemini-key-file` uses another key and `--no-gemini`
turns the tool off; without a key, the Gemini skill is left out of the workspace.

## Usage

### Prepare, launch, resume

```bash
$RAX_PYTHON scripts/configure_session.py --task stack_cups --task-file my_task.md
#   prints <session>/operator-input.json; add --robot mock for a hardware-free dry run
bash scripts/start_agent.sh --config <session>/operator-input.json
bash scripts/start_agent.sh --config <session>/operator-input.json --resume
```

- `configure_session.py` writes the configuration and starts nothing. The task file
  becomes the agent's `TASK.md`. Other options: `--observation-profile rgbd|rgb-only`,
  `--agent-effort high|xhigh|max`, `--session-hours` (wall-clock budget, default 4),
  `--max-model-calls`, `--subagents` (0 to 8, default 0).
- `start_agent.sh` builds the workspace, starts the relay and the robot service, and runs
  Claude Code in its terminal UI. `--non-interactive` runs the start prompt without a UI;
  `--exec-prompt TEXT` runs a different prompt; `--prepare-only` builds the session and
  starts nothing.
- `--resume` continues the session's own Claude Code conversation in the same workspace,
  with the wall-clock budget that is left. A non-interactive run that exhausts its Claude
  usage window waits for the reset by itself, off the session clock, and then continues.
- Stop a session with Ctrl-C or SIGTERM to the launcher; it stops the agent, then the
  robot service.

Sessions live in `<artifact root>/sessions/`. Each holds the workspace, the Claude Code
home with its transcripts (`claude-home/projects/`), the robot-call log `mcp.jsonl`,
`model-usage.jsonl`, `frontend.log`, the recording and the archive.

### Arm the robot

Motion tools refuse until the operator arms the session. Arm only with a person at the
emergency stop and the workspace clear. The agent cannot arm itself: its sandbox cannot
write the session root.

The `franka` command is the short form (`ln -s $PWD/scripts/franka ~/.local/bin/franka`
once):

| Command | What it does |
| --- | --- |
| `franka arm [minutes]` | arms the newest session after you type `ARM`; optionally expires |
| `franka disarm` | disarms every session, then asks the arm bridge to hold |
| `franka status` | arming state and open operator requests of recent sessions |
| `franka requests` / `franka reply [text]` | shows what the agent asks of the operator (for example a scene reset) / answers the oldest request |
| `franka home [--open]` | moves the arm to the ready pose after a table-clearance check and your typed `HOME`; refused while an agent session runs. `--open` opens the gripper |
| `franka clean-streams` | deletes the raw recording window of sessions that are not running (the archive stays) |

`scripts/arm_robot.py <session> arm|disarm|status [--minutes N]` does the same for one
named session.

### Dashboard

```bash
bash scripts/run_dashboard.sh            # prints http://127.0.0.1:8770/?token=...
# remote: ssh -L 8770:127.0.0.1:8770 <robot-host>, then open the printed URL locally
```

A loopback-only operator page. Opening the tokenized URL sets an HttpOnly cookie; every
other request needs it. It lets you:

- create a session and copy its interactive, headless and resume commands;
- launch, resume or stop a session; one session runs at a time (one robot);
- **ARM** (tick "a person is at the emergency stop", then confirm) and **DISARM**;
  **DISARM ALL** covers every session;
- watch the robot (TCP pose, joints, gripper, bridge health), the agent's latest camera
  frame, its transcript, the robot-call log and the service logs.

Never give the URL or token to an agent.

### Recording and archive

While the robot service runs, the camera server records every camera at 30 Hz (depth at
1 Hz) and the arm at 10 Hz into `<session>/stream/`. This is a 10-minute sliding window
the agent can read (`runtime/stream`, the `franka_stream` tool) but not write. Each
completed minute is also encoded into a whole-session archive:

```
<session>/archive/<camera>/<UTC YYYYmmddTHHMM>.mp4    H.264
<session>/archive/<camera>/<UTC YYYYmmddTHHMM>.json   capture time of every frame (epoch ms)
<session>/archive/robot/<UTC YYYYmmddTHHMM>.jsonl     arm and gripper at 10 Hz
```

Frame k of a minute's MP4 was captured at `frames_ms[k]` of its sidecar. The archive
therefore lines up with the Claude transcript, `mcp.jsonl` and `model-usage.jsonl`.

## Repository layout

| Path | Contents |
| --- | --- |
| `auto_research_agent/` | The agent's instructions (`AGENTS.md`, `START_PROMPT.md`) and its MCP client (`api/`) |
| `agent_skills/` | The agent's skills by axis: task, perception, control, Gemini ([README](agent_skills/README.md)) |
| `agent_blocks/` | Text blocks shared by the instruction and skill templates |
| `harness/claude_cli/` | The Claude Code launcher and sandbox settings, the credential relay, the MCP bridge, the experiment login |
| `services/controller/` | The robot service (MCP server), session ledger and audit, operator requests, brokered Gemini |
| `services/franka/` | The Franka backend: bridge clients, servo loop, kinematics and IK checks, arming, cameras, recording and archive |
| `services/dashboard/` | The operator dashboard |
| `services/workspace/` | The workspace generator |
| `services/mock_backend.py` | A hardware-free robot for tests and dry runs |
| `examples/franka.json` | Backend options to copy and fill in |
| `scripts/` | Configure, launch, dashboard, arming and login scripts |
| `local_tests/` | Unit tests ([README](local_tests/README.md)) |

## Tests

```bash
$RAX_PYTHON -m pytest -q
```

The suite uses temporary artifact roots, the `mock` robot backend, simulated bridges and
fake model providers. It never moves hardware, opens a camera, calls a model or needs
credentials. The archive tests skip without `ffmpeg`.
