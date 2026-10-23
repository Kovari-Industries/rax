---
name: robot-interface
description: The <robot name>'s tools, frames, units, limits, measured motion, gripper, cameras, arming and failure handling. Read before composing any robot action or writing robot code.
---

<!--
Template for the one robot-specific skill every universal skill points to. Copy it to
agent_skills/control/<robot>/robot-interface/SKILL.md and replace every <...>
placeholder (written \<...> in the body, so GitHub does not hide it as an HTML tag).

Rules:
- Every number comes from the code, configuration, kinematics or a measurement on this
  robot (see docs/REAL_ROBOT_ADAPTATION.md, "Characterization protocol"); never an
  estimate. Say where a number was measured if it can drift.
- Exact fields, units and per-call limits belong in the tool schemas; this skill owns
  tool choice, interpretation and non-obvious behaviour. Do not let the two disagree.
- Keep task advice out: no task names, object layouts or episode stories.
- Delete sections that do not apply (no base, no gripper, no stream), and remove these
  comments.

Worked examples on the RAX branches:
- Franka FR3, one arm on a table:
  https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/control/franka/robot-interface/SKILL.md
- Rainbow RB-Y1, mobile bimanual with torso and head:
  https://github.com/robot-agent-exploration/rax/blob/real/rainbow/agent_skills/control/rby1/robot-interface/SKILL.md
- Dual ARX X5 in simulation (RoboDojo):
  https://github.com/robot-agent-exploration/rax/blob/robodojo-behavior/container/agent_skills/control/robodojo/robot-interface/SKILL.md
- Galaxea R1Pro in simulation (BEHAVIOR-1K):
  https://github.com/robot-agent-exploration/rax/blob/robodojo-behavior/container/agent_skills/control/behavior/robot-interface/SKILL.md
-->

# \<Robot name> robot interface

<One paragraph: the body (base, torso, arms and their DoF, grippers or hands, head,
cameras), where it works (table, floor, rooms), and who stands at the emergency stop.
State that every command is physical and reaches the robot only through these tools.>

## Frames and units

- Robot frame `<name>`: \<origin; x, y, z directions>. Units: \<metres, radians>;
  quaternion order `<[x, y, z, w] or [w, x, y, z]>`.
- End effector / tool point `<name>`: <where it is relative to the flange or fingers,
  in metres>. Every pose tool uses \<this point>.
- <Which frames move with the robot (a mobile base), and what becomes stale after a
  base move.>
- \<Odometry or world frame, if any: how it is reported and how it drifts.>
- Reference heights: \<floor, table top, z in the robot frame, measured>.

## Tools by level

| Level | Tool | Use it for | Needs arming |
|---|---|---|---|
| Sense | `<status tool>` | \<joints, poses, gripper, arming, limits, faults> | no |
| | `<observe tool>` | \<cameras returned and saved; frame id> | no |
| | `<pixel-to-position tool>` | \<pixels of a saved frame to robot-frame points> | no |
| | `<stream tool>` | \<recent recording, if any> | no |
| Plan / dry run | `<check, FK, preview or planner tool>` | \<what it predicts without moving> | no |
| Task space | `<pose tool>` | \<one end effector to a pose; per-call limit> | yes |
| Joint space | `<joint tool>` | \<joint groups, presets> | yes |
| Gripper | `<gripper tool>` | \<open, close with force, set width> | yes |
| Base | `<base tool>` | \<drive and turn; limits> | yes |
| Always allowed | `<hold or stop tool>` | \<stop and hold everything> | no |

Choose the highest level that expresses the intent: <for example, aim the camera
rather than set head joints; a pose tool for a hand position; joints for postures and
recovery>. <Is a collision-aware planner wired in? If not, say that long motions are
sequences of checked short moves through clear postures.>

## Which tool for which motion (measured)

| Motion | Tool and arguments | Measured accuracy | Measured time | Avoid when |
|---|---|---|---|---|
| \<short reach in free space> | `<...>` | \<mm> | \<s> | <...> |
| \<straight approach near contact> | `<...>` | \<mm> | \<s> | <...> |
| \<posture change, retract, recovery> | `<...>` | \<rad> | \<s> | <...> |
| \<base move> | `<...>` | \<cm, deg; slip> | \<s> | <...> |

<Path shape (straight line or joint-space), settle tolerance, speed and acceleration
limits, repeatability, how a short move behaves near limits.>

## Neutral pose and recovery

- Neutral: \<preset and tool; what to pass through to reach it safely>.
- From an unknown posture: \<safe sequence>.

## Gripper

- \<Type, maximum opening, finger geometry if custom (measured offsets), force range.>
- <How a grasp is detected (reported flag, opening, force), and what a position-only
  command does with a held object.>
- <Grasping pattern for this hand: approach height, orientation, descent steps, the
  lowest safe height above the support surface.>

## Cameras and calibration

- \<Each camera: name, mount, resolution, depth or not, what moves it.>
- <What a saved frame contains: images, depth (units, invalid value), intrinsics,
  camera pose and when it is computed.>
- <Measured accuracy: plane fits, cross-view agreement, where depth fails, robot-body
  pixels to ignore.>
- \<Preview scaling between returned images and saved full-resolution frames.>

## Limits the backend enforces

| Limit | Value | Refused before moving or stopped while moving |
|---|---|---|
| Workspace box | \<x, y, z ranges> | <...> |
| Per-call distance or angle | <...> | <...> |
| Joint limits and margin | <...> | <...> |
| Speed, torque, force, stall | <...> | <...> |
| Self-collision, load, tip-over | <...> | <...> |

A refusal is a wall, not a fault: choose another approach, never a way around it.

## Arming and safety

- Motion and gripper tools work only while \<the operator has armed motion>. If they are
  not armed, <ask the operator with the operator tool, saying what you will do, and
  wait>; never work around arming or the emergency stop.
- \<The operator may disarm at any time: what a running motion does then.>
- <Vendor and lab rules, with the reason for each: what never to do (for example push
  against a stop, drive with an arm extended, move the torso with someone between the
  arms and the body).>
- After a fault or an emergency stop: <no motion and no recovery plan of your own; ask
  the operator, wait, then start with status and a fresh observation>.
- Before a motion, look at a fresh image of the space it sweeps.

## Reading errors

A failed tool returns its whole error: <the shape, e.g. `{type, operation, stage,
reason}` plus the measured `robot_state` when it failed during a motion>.

| You see | Meaning | Do |
|---|---|---|
| \<refused before moving> | nothing moved | fix the cause named in the reason |
| \<handled stop during a motion> | the robot holds at the measured state | observe, find the cause, plan from the measured state; never resend the same target |
| \<driver or SDK failure> | the robot's software failed; the backend held the robot | read the message, check status and a fresh observation; tell the operator if it repeats |

## From code

Python reaches the same tools through `api.runtime.Context`:
`ctx.call("<pose tool>", <arguments>)`. Calls are serialized with your direct calls;
check every reply and stop on the first unexpected one.
