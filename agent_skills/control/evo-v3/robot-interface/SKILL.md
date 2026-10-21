---
name: robot-interface
description: Use Evo v3 policy status, fresh camera observation, and policy cancellation through the RAX backend. Read before Evo robot exploration.
---
# Evo v3 interface

Evo Orchestrator owns the robot commands and hardware bridges. This first RAX adapter
exposes `evo_status`, `evo_observe`, `evo_hold`, and `evo_towel_align`, each with no arguments. `evo_measured_state` reads measured joints and palm FK. `evo_step` accepts paired bounded translations and optional gripper targets.
Construction sends no command. The backend contains no motor SDK or controller-output publisher. Hold correction uses only the existing diagnostic lease interface.

- Observe after a pause. Inspect all camera errors and capture times. A partial capture
  is returned for diagnosis with `ok: false`; it is not a valid three-camera bundle.
- `evo_observe` saves full-resolution JPEGs and metadata. It does not yet provide
  validated extrinsics or capture-time joints. Do not use it for metric movement.
- `evo_status` reads the configured policy socket. Its state is a policy state, not
  measured physical pose. A missing socket is an unavailable runtime, not an idle robot.
- `evo_hold` requests cancellation of this policy only. It is not the robot e-stop or a
  global stop. Verify measured stop through Evo before another operation.
- `evo_towel_align` runs one configured attempt only when enabled by the operator and
  armed. It requires protocol v2. Check matching session/attempt, completed status, and
  stop proof. A failed attempt is not readiness. Do not retry an uncertain start.
- No generic base, carriage, startup, homing, tuning, or reset tool is exposed.
- `evo_step` uses the native Evo adapter and existing autonomy lanes, IK and Ruckig.
  Start recording, check fresh state and enable session motion before use. It holds
  measured orientation and limits each hand to 3 cm per call. Positive Z points down
  in carriage_link. Check `ok`, matching step identity, and `stop_verified`. A timeout
  or lost connection is not a successful step. Closing the connection withdraws the
  source; it is not a global stop.
- The first 2 cm upward trial failed: modeled IK residual was below 0.4 mm, but
  measured palm error was about 24 mm and elbow tracking error about 3.2 degrees.
  No successful grasp or fold is established. Resolve tracking before contact.
  Do not substitute direct SDK or shell motion calls inside a RAX session.
- Polling and socket timeouts are not measured stop latency. RAX process death is not yet
  covered by an Evo heartbeat lease; the policy retains its own bounded lifetime.

Use the RGB localization skill for perception trials. Record results as observed until
fresh final geometry and independent physical checks establish task success.

`evo_hold_correct` is a separately enabled J4 trial, not a general torque tool.
It requires a loaded #1230 driver, MCAP recording, session arming, a fixed joint
target, default slew, and no overload mode. It observes a baseline, renews the
short hardware lease, withdraws correction, and measures retained error. Keep
its event JSONL with the MCAP. Stop on a failed guard; never automatically repeat
a pulse or claim grasp accuracy from reduced encoder error. A driver replacement
requires supported arms because deactivation disables motors.
