# RAX on the Rainbow RB-Y1

**[Project website](https://robot-agent-exploration.com/)** · [Paper](https://robot-agent-exploration.com/rax.pdf) · [Main repository](https://github.com/robot-agent-exploration/rax)

This branch runs RAX (*RAX: Scaling Online Exploration for Robot Agents*) on a real
**Rainbow Robotics RB-Y1** (model A) mobile manipulator. A Claude Code agent learns a
task by exploring the robot through MCP tools: first by hand, then by writing and testing
its own code, with its perception checking the scene at the decisive points. The agent
runs on the host in Claude Code's sandbox; one trusted robot service owns the robot, and
the operator arms every motion.

The design principles and the porting steps for another robot are in the main
repository's [real-robot adaptation guide](https://github.com/robot-agent-exploration/rax/blob/main/docs/REAL_ROBOT_ADAPTATION.md).
Robot details, safety limits and what was measured on the robot: [RBY1.md](RBY1.md).
System map and setup lessons: [docs/rby1/SETUP_NOTES.md](docs/rby1/SETUP_NOTES.md).

> **This software moves a real robot.** Run it only with a person at the emergency stop,
> with the robot's workspace clear, and after the motion check below. Every motion tool
> refuses until the operator arms the session; the EMO buttons stay the outermost layer.

## How it fits together

```
 Claude Code (host process, cwd = <session>/agent-workspace)
   ├─ MCP stdio ── mcp_bridge.py ──┐
   └─ Bash (bubblewrap sandbox) ───┴─ Unix socket ── robot service (services/controller/frontend.py)
                                                       └─ Rby1Backend ── rby1_sdk ── gRPC ── controller (192.168.30.1)
 Claude Code ── http://127.0.0.1:<port> ── messages_relay.py (holds the model credential)
```

| Layer | What | Where |
|---|---|---|
| Backend (the robot) | MCP tools, safety checks, kinematics, cameras, live recording, dashboard telemetry | `services/rby1/`, registered in `services/backends.py` |
| Frontend (operator) | Dashboard: sessions, arming, operator requests, frames, live robot state | `services/dashboard/` |
| Harness | Workspace generator, agent instructions and skills, launcher, sandbox, relay, MCP frontend | `services/workspace/`, `auto_research_agent/`, `agent_skills/`, `harness/`, `services/controller/` |

- **One robot owner.** The robot service is the only process that commands the robot. It
  serializes every call from the agent CLI and the agent's scripts and writes an audit log
  (`mcp.jsonl`).
- **The workspace is the agent's only writable place.** Claude Code runs with
  `--permission-mode dontAsk`; Read and Edit reach only the workspace. Bash runs in Claude
  Code's sandbox: internet only through its proxy, which refuses the robot controller; no
  host loopback; writes only in the workspace; no access to this repository, credentials,
  the session's private files, docker/dbus/X11 sockets, or the paths you list in
  `RAX_PROTECTED_PATHS`.
- **No credentials in the agent.** A host relay injects the model credential. Gemini
  (`google/gemini-3.8-flash`, for checking images) is brokered by the robot service with a
  $10 cap per session.
- **Motion needs the operator.** Every motion tool refuses until the operator arms the
  session (`rby1 arm`, typing ARM). `rby1 disarm` stops a running motion within one
  control period, and the robot holds.

## Requirements

- **Robot:** RB-Y1 model A, controller firmware 0.8.0 (SDK commit `f3bb123`, v0.8.3),
  with its two Dynamixel grippers and the head ZED 2i.
- **UPC:** the robot's Jetson (aarch64, on the robot network 192.168.30.0/24), where the
  grippers and the head camera are connected over USB. Everything below runs there.
- **Software on that host:**
  - [Claude Code](https://docs.claude.com/en/docs/claude-code/setup) (`claude` on `PATH`);
  - `bubblewrap` and `socat` for Claude Code's sandbox (on Ubuntu 24.04, also an AppArmor
    profile that allows bwrap's user namespaces);
  - the [ZED SDK](https://www.stereolabs.com/developers) with its Python wheel (`pyzed`);
  - `rsync` and `ssh`, only if you ship finished runs off the robot.
- **A model provider:** a Claude account (subscription or API key), or an OpenRouter key.
  An OpenRouter key also enables the Gemini tool.

## Setup

### 1. Robot network and devices

Connect the host to the robot network and check that it reaches the controller
(`ping 192.168.30.1`). Give the gripper's serial adapter the stable name
`/dev/rby1_gripper` with a udev rule. Stop every other process that commands the robot or
opens its devices (another robot server, a gamepad teleop, a ZED publisher): one process
owns the grippers' bus and the camera.

### 2. Two Python environments

The robot service and the agent use separate environments, so the agent's scripts can
never import the robot SDK:

```bash
# Robot service (Python 3.10 tested): the SDK version must match the controller's firmware.
python3.10 -m venv ~/envs/rax-rby1
~/envs/rax-rby1/bin/pip install rby1-sdk==0.8.3 numpy scipy opencv-python-headless jinja2 tomli \
    pillow pytest imageio-ffmpeg
~/envs/rax-rby1/bin/pip install <the pyzed wheel from your ZED SDK installation>

# Agent: no robot SDK. Add the libraries you want the agent to have (RBY1.md lists ours).
python3 -m venv ~/envs/rax-agent
~/envs/rax-agent/bin/pip install numpy scipy opencv-python-headless pillow matplotlib

export RAX_PYTHON=~/envs/rax-rby1/bin/python
export RAX_AGENT_PYTHON=~/envs/rax-agent/bin/python
```

The agent's environment is read-only in the sandbox; the agent installs anything else with
`pip install --target .pylib` in its workspace.

### 3. Storage and the robot model

Sessions, live recordings, archives and logs go under the artifact root: `runtime/` in
this repository (make it a link to a disk with room), or `RAX_ARTIFACT_ROOT`.

Optionally give the agent the vendor robot model: copy `models/rby1a` from an `rby1-sdk`
checkout at the controller's commit to `runtime/robot-models/rby1a/`, and add the
standard-URDF copy that generic tools (pinocchio, yourdfpy, trimesh) need:

```bash
"$RAX_PYTHON" scripts/standardize_urdf.py runtime/robot-models/rby1a/urdf/model_v1.0.urdf \
    runtime/robot-models/rby1a/urdf/model_v1.0.standard.urdf
```

Sessions link it read-only as `agent-workspace/robot-model/`.

### 4. Protect the robot software on this host

List every path on the host that could reach the robot past the backend: vendor SDK
checkouts and environments, other robot servers, teleoperation stacks, and your robot
service environment if it lives outside the repository. Use colon-separated absolute
paths:

```bash
export RAX_PROTECTED_PATHS=~/envs/rax-rby1:~/rby1-sdk:/opt/robot-server
```

The agent's sandbox can neither read nor run them. Credentials (`~/.ssh`, `~/.claude`,
`~/.config/rax`, ...) and this repository are always protected. **An empty list is unsafe
on a robot host**, and the launcher warns about it. The sandbox proxy also refuses the
controller's address (192.168.30.1, plus the host of a configured `address` backend
option); a firewall rule for the agent's traffic is the stronger fix.

### 5. Model credentials

- **`claude-login` (default):** a long-lived token for a Claude account. Prefer a separate
  account dedicated to experiments. Run this once, and again to rotate the token:

  ```bash
  bash scripts/setup_claude_experiment_login.sh [--email experiment-account@example.com]
  bash scripts/setup_claude_experiment_login.sh --personal   # or bill your own signed-in account
  ```

  The token is stored in `~/.config/rax/` (`RAX_CLAUDE_EXPERIMENT_DIR` overrides). Your
  personal `~/.claude` login is refused unless you chose `--personal`.
- **`anthropic` or `openrouter`:** pass the key file to the launcher with `--key-file`.
- **Gemini (optional):** put an OpenRouter key in `~/.config/rax/openrouter.key` (mode
  0600), or pass `--gemini-key-file`. Without it sessions run without the Gemini tool.

### 6. Install the operator command

```bash
ln -s "$PWD/scripts/rby1" ~/.local/bin/rby1
```

## Check the robot first

Before the first session, and after any firmware or `rby1-sdk` upgrade, run the
supervised motion check through the real backend (the same guards the agent gets), with a
person at the emergency stop:

```bash
"$RAX_PYTHON" scripts/configure_session.py --task motion_check --task-text "motion check"
"$RAX_PYTHON" scripts/rby1_motion_check.py <session_root> status     # connects; nothing moves or powers on
"$RAX_PYTHON" scripts/rby1_motion_check.py <session_root> disarmed   # motion is refused while disarmed
rby1 arm 5                                                          # then run each motion step
"$RAX_PYTHON" scripts/rby1_motion_check.py <session_root> head
```

The steps are listed in the script's docstring. `scripts/rby1_characterize.py`,
`scripts/rby1_controller_check.py`, `scripts/rby1_large_motion.py` and
`scripts/rby1_health_check.py` measure the tools further; [RBY1.md](RBY1.md) records what
they measured on our robot.

Calibrate your robot's head camera and pass the 4x4 `link_head_2` → optical-frame
transform as the backend option `head_camera_extrinsic`; the default is our unit's
calibration. Set `camera_serial` if more than one ZED is connected.

## Run a session

```bash
"$RAX_PYTHON" scripts/configure_session.py --task fetch_cup --task-file task.md   # prints <session>/operator-input.json
bash scripts/start_agent.sh --config <session>/operator-input.json               # Claude Code's terminal UI
rby1 arm                 # person at the e-stop, workspace clear; type ARM (rby1 arm 30: 30 minutes)
rby1 requests            # what the agent asks you for;  rby1 reply "done" answers the oldest
rby1 disarm              # any time: running motions stop and the robot holds
```

- `configure_session.py` writes the configuration and starts nothing. Useful options:
  `--backend-options options.json` (any key of `DEFAULTS` in `services/rby1/backend.py`:
  speed caps, guards, camera, controller address), `--agent-python`,
  `--observation-profile rgbd|rgb-only`, `--agent-provider`, `--session-hours` (default
  4), `--subagents` (default 2; subagents never command the robot).
- `start_agent.sh` takes `--non-interactive` (run without the terminal UI), `--resume`
  (continue a stopped session's own conversation in the same workspace, with the harness
  files refreshed), `--key-file`, `--gemini-key-file`, `--no-gemini` and `--prepare-only`.
  `Ctrl-C` or `SIGTERM` stops the agent and the robot service.
- The agent reads `TASK.md` and its skills, explores the task by hand, then writes code in
  `code/` and runs it under its own observation. What it learns that works goes into a
  shared skill library (`runtime/skill-library/rby1/`, a git repository that every new
  session clones and merges back into).

**Safety while a session runs.** Arming is re-read every control tick (50 Hz), with the
EMO state polled each tick. A motion stops on disarm, EMO, stale state, a drive problem,
a torque or wrist-force guard, or the joint envelope, and ends with a hold where the
joints are. A controller fault or an EMO press disarms the session, and the backend resets
a fault only after you arm again on purpose. Limits and guards are backend options; see
`DEFAULTS` in `services/rby1/backend.py` and [RBY1.md](RBY1.md).

## Dashboard

```bash
bash scripts/run_dashboard.sh            # prints http://127.0.0.1:8770/?token=...
# from another machine: ssh -L 8770:127.0.0.1:8770 <user>@<robot host>, then open the printed URL
```

Sessions, configuration and launch, arming (with an e-stop confirmation), operator
requests, the latest frames, the audit log and live robot state (EMO, power, control
manager, battery, joints, wrist forces, end effectors) from read-only telemetry.

## Recording and storage

The backend records the head camera (about 10 Hz, depth at 1 Hz) and the robot state (10
Hz) into `<session>/stream`, keeping the last 10 minutes; the agent reads it with
`rby1_stream` and `api/stream.py`. Every session is also archived whole for replay
(`<session>/archive`: one H.264 MP4 per minute with per-frame capture times, plus the
robot state), about 145 MB per hour.

To keep the robot's disk from filling, set `RAX_SHIP_DEST=host:/dir`: after each run the
launcher zips the session, copies it with rsync over ssh, verifies its checksum and only
then deletes the local copy (`runtime/ship.log`). `rby1 ship [--dry-run]` does it by hand.
Nothing ships unless `RAX_SHIP_DEST` is set.

## Environment variables

| Variable | Meaning |
|---|---|
| `RAX_PYTHON` | The robot service's Python (with `rby1-sdk`); used by every script |
| `RAX_AGENT_PYTHON` | The Python for the agent's scripts (no robot SDK); a session's `agent_python` overrides it |
| `RAX_PROTECTED_PATHS` | Colon-separated absolute paths the agent's sandbox may not read |
| `RAX_ARTIFACT_ROOT` | Where sessions live (default: `runtime/`) |
| `RAX_CLAUDE_EXPERIMENT_DIR` | Where the `claude-login` token lives (default: `~/.config/rax`) |
| `RAX_SHIP_DEST`, `RAX_SHIP_SSH`, `RAX_SHIP_MIN_FREE_GB` | Shipping finished runs (off by default) |
| `RAX_SHIP` | `0` turns shipping off even with a destination |
| `RAX_SKILL_LIBRARY` | Another location for the skill library |
| `RAX_ROBOT` | The dashboard's default robot (`rby1`) |

## Tests

```bash
env -u PYTHONPATH "$RAX_PYTHON" -m pytest -q
```

The suite uses a simulated RB-Y1 behind the SDK surface (with the vendor URDF), the `mock`
backend and fake model providers. It never moves hardware, opens a camera, calls a model
or needs credentials. See [local_tests/README.md](local_tests/README.md).

## Repository layout

```
auto_research_agent/   the agent's instructions (AGENTS.md, START_PROMPT.md) and in-workspace API (api/)
agent_skills/          agent skills by axis: task, perception, control/rby1, auto-research
agent_blocks/          reusable text blocks for the instruction and skill templates
harness/claude_cli/    launcher.py, messages_relay.py (credential relay), mcp_bridge.py, experiment_login.py
services/controller/   the robot service (MCP frontend), session ledger, operator requests, Gemini broker
services/rby1/         the RB-Y1 backend: tools and checks, SDK calls (robot.py), kinematics, grippers, ZED, recorder
services/dashboard/    the operator dashboard
services/workspace/    the workspace generator
scripts/               configure_session.py, start_agent.sh, rby1, run_dashboard.sh, motion check and characterization
local_tests/           unit tests
```

## Citation

BibTeX coming soon. Until then, please cite the paper as listed on the
[project website](https://robot-agent-exploration.com/).
