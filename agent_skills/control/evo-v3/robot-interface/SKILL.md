---
name: robot-interface
description: Use Evo v3 policy status, fresh camera observation, and policy cancellation through the RAX backend. Read before Evo robot exploration.
---
# Evo v3 interface

Evo Orchestrator owns the robot commands and hardware bridges. This first RAX adapter
exposes `evo_status`, `evo_observe`, and `evo_hold`, each with no arguments.
Construction sends no command. The backend contains no ROS publisher or hardware SDK.

- Observe after a pause. Inspect all camera errors and capture times. A partial capture
  is returned for diagnosis with `ok: false`; it is not a valid three-camera bundle.
- `evo_observe` saves full-resolution JPEGs and metadata. It does not yet provide
  validated extrinsics or capture-time joints. Do not use it for metric movement.
- `evo_status` reads the configured policy socket. Its state is a policy state, not
  measured physical pose. A missing socket is an unavailable runtime, not an idle robot.
- `evo_hold` requests cancellation of this policy only. It is not the robot e-stop or a
  global stop. Verify measured stop through Evo before another operation.
- No start, base, carriage, arm, grasp, startup, homing, tuning, or reset tool is exposed.
  Do not substitute direct SDK or shell motion calls inside a RAX session.

Use the RGB localization skill for perception trials. Record results as observed until
fresh final geometry and independent physical checks establish task success.
