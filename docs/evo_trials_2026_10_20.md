# Evo trial evidence from the earlier transport

This record preserves the trials on the target branch before the exclusive agent
backend replaced that transport. Tool names, sockets, and enablement instructions
below describe the earlier runtime. They are not current agent instructions.
The diagnostic source is retained under `operator_trials/`; it is not registered
as an agent tool or used as a backend fallback. The physical tracking limitations
and unqualified towel outcomes remain relevant evidence.

### Attempt lifecycle verification

Focused RAX tests cover old terminal results, fast refusal, runtime replacement,
connection loss, disarm, busy admission, and unversioned runtime rejection. Evo tests
cover pending cancellation, busy starts, reset of stop proof, and failed stop. No motion
has been issued through this new tool. The candidate Evo runtime is built separately;
RAX source alone cannot enable the required robot geometry or motion interfaces.

## Bounded RAX arm adapter trial

`evo_measured_state` and `evo_step` use the separate Rust `evo_rax_motion`
adapter in Evo's `evo_towel_folding` package. It publishes only on the existing
autonomy pose/gripper lanes. Evo retains arbitration, IK, Ruckig and hardware
control. The installed recovery runtime was preserved.

`motion_socket` defaults to `/run/evo/rax-motion.sock`; `enable_motion` defaults
to false. Start recording before enabling a trial. The transport checks session
arming during each call and closes the connection on disarm. The native adapter
withdraws its source on disconnect, fault or completion and checks measured
settling. A missing response never authorizes a retry. This is not a global stop.
The event JSONL is a recording sidecar; recorder activity is checked by the
trial operator, not enforced by the socket protocol.

Steps hold measured palm orientation, limit each translation to 3 cm and peak
speed to 1.5 cm/s, and bound gripper targets to [-0.6, 0] rad. Coordinates are
`carriage_link`, positive Z down. Completion requires measured tracking and
settling. These bounds do not establish collision clearance or contact targets.

The first live hold passed. A subsequent 2 cm upward step failed measured
settling: modeled palm positions moved about 4 mm downward. Motion was withdrawn
and stop was verified. No grasp or fold was attempted. The motion server was
then stopped. MCAP: jetson-105 `/home/benson/evo_bags/2026-10-20T23-09Z_run-0000`.
Task definition: `tasks/evo-v3/fold_towel_in_half.md`. Do not promote this
primitive as task-confirmed until the tracking error is resolved.

## Fixed-target J4 correction candidate

`evo_hold_correct({"arm":"left"})` runs one stationary baseline, the bounded
hardware-loop pulse from Evo #1230, and a retained-error observation after
withdrawal. `enable_hold_correction` defaults to false; session arming is also
required. Start MCAP before enabling the tool. It neither sets an arm pose nor
implements torque control in Python. All torque selection and fast guards stay
in the C++ hardware write loop. It refuses an enabled calibration session, non-default slew, or a driver
envelope that differs from the operator-configured profile. It stops on changed targets or unhealthy
feedback and does not retry failed pulses. This is a pilot, not five-pair
statistical acceptance or verified tool accuracy.

The new driver must be loaded before this tool can operate. Its installation
requires a controlled shutdown because the existing driver disables motors on
deactivation. The compiled overlay is separate from the active robot install.
No physical correction trial has been performed with this integration yet.

### First deployed hold pilot

The overlay loaded on jetson-105. MCAP `2026-10-20T23-26Z_run-0000`
contains startup, images and the first left J4 pilot. No added correction torque
was recorded; the driver ended with `predicted_target`. Withdrawal and measured
stop passed, and correction was disabled afterward. The towel was not touched.

Startup commanded J4 to 1.7 rad and settled at 1.672198 rad. IK then seeded from
measurement and published 1.672198 rad through the arm bridge. Thus the pilot
compared measurement with a newly held measured target, not the configured pose.
The original result incorrectly treated floating-point noise as improvement.
The evaluator now rejects a baseline below 0.001 rad before enabling correction;
improvement must exceed both 0.001 rad and 5% of baseline error. The raw pilot
result remains preserved with a separate corrected review.

Before another pilot, retain an explicit target through the authorized arm
command path and verify its identity during baseline and correction. Do not
claim the configuration error is fixed because a measured-pose hold has zero
tracking error. Base controller activation also failed during this restart;
resolve that before base motion. No full folding acceptance is established.

### Fold approach with hold assist

The 20 mm rise in MCAP `2026-10-20T23-31Z_run-0000` did not track.
Its left J4 pulse stopped when reported effort crossed 9 Nm (9.005 Nm recorded).
The generic driver reason was `invalid_hold_feedback`; do not interpret that
label alone as a communication fault.

An authorized retry used a 5 mm rise, with unchanged guards, in MCAP
`2026-10-20T23-35Z_run-0000`. Both pulses reached 1.5 Nm added torque and ended
at `pulse_timeout`. Left J4 did not move; right J4 moved one encoder count
(0.0003815 rad). Both measured stops and correction withdrawals passed. This is
not an effective correction at this pose. The towel was not grasped. Do not
repeat pulses or increase limits on the assumption that the earlier failure
was only a software admission problem.

### Attended peak profile

The operator requested the existing PR #1230 attended envelope for continued
fold trials. Set backend option `hold_profile: attended_peak` only with a driver
started with `calibration_overload_trial: true`. The default profile remains
`default`. A mismatch fails before enabling correction. The task cannot set an
arbitrary torque limit through a tool argument.

This uses the driver's existing ceiling below 15 Nm and cumulative exposure
below 30 seconds above 9 Nm per joint. Temperature, acquisition freshness,
target binding, cancellation and pulse caps remain active. The J4 pulse cap
stays 1.5 Nm for at most 2 seconds. Exposure spans normal motion and idle holds,
not just pulses. A trip is terminal for this attempt; do not restart to clear
its counter and continue. The separate attended deployment disables automatic
controller-manager restart. Motor-off restart still requires physical support.

Prepared robot manifest: `/data/rax-hold-1230/attended-config/manifest.yaml`.
Original manifest remains `/data/rax-hold-1230/config/manifest.yaml`.
At preparation time the default runtime is still active; no higher-torque
physical acceptance is claimed.

### Attended approach result

The attended peak profile was activated on both drivers after an operator-supported
restart. In MCAP `2026-10-20T23-42Z_run-0000`, one J4 pulse per arm after a
20 mm upward approach reduced retained encoder error from 3.26 to 1.66 degrees
on the left and 3.20 to 0.93 degrees on the right. Both pulses ended normally
at their duration limit and verified withdrawal and stop. Net modeled palm rise
was 8.46 mm left and 14.47 mm right; independent tool accuracy is not established.

The existing `ArmMotion.set_carriage` primitive then raised the carriage from
0.29596 to 0.34980 m in small measured waypoints with a fresh-feedback cancel
predicate, and verified stationary feedback. Images show the towel surface more
clearly, but no corner grasp or fold was attempted. The recorded initial step
still failed its original settling criterion before the assist; preserve that
first divergence. Do not describe this as a complete or task-confirmed fold.
