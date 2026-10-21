# Fold a towel in half

A flat rectangular towel is placed consistently on the table. The robot starts
aligned and within reach. Grasp the two near corners, lift the near edge clear
of the table, bring it toward the far edge, lower it, release, and withdraw.
Inspect the result and report whether the towel was folded in half.

## Execution

Use the Evo backend as the single command owner. Use the existing Evo arbitration,
IK and Ruckig path. Do not start ACT. Do not reuse historical grasp coordinates.
Start synchronized MCAP recording before motion and retain it through inspection.
Save each tool request, response, phase change, camera capture, measured state,
requested target, and motion error with timestamps and the recording identity.

Observe -> grasp near corners -> lift -> transfer -> lower -> release -> withdraw -> inspect.

| Phase | Intended observation |
|---|---|
| Start | Flat towel, both near corners identified, current grasp targets reachable |
| Grasp | Each gripper closes around its assigned near corner |
| Lift | Both corners rise with the grippers; the near edge clears the surface |
| Transfer | Both held corners move toward the corresponding far corners |
| Lower | The near half rests on the far half |
| Release | Both grippers open without pulling the towel away |
| Withdraw | Hands clear the cloth; cloth remains on the table |
| Inspect | One fold across the middle; two overlapping halves; near and far edges approximately coincide |

Keep the complete sequence as the task. Record ordinary position errors and visual
mismatches rather than declaring success from command completion. Stop for lost
feedback, rejected commands, unavailable contact targets, or a hardware fault.
Do not continue a contact motion based only on guessed image depth.

## Bounded hold assist during execution

Use the existing `evo_step` and `evo_hold_correct` tools as one approach sequence:

1. Execute the bounded arm segment through IK and Ruckig.
2. After a measured stop, retain the outgoing target. If the segment completed
   or only reported `measured settling timeout`, request one left J4 assist and
   then one right J4 assist. Skip an assist with insufficient baseline error.
3. Stop the sequence on a driver guard, lost feedback, or unverified withdrawal.
   Do not retry the pulse or raise its torque limit.
4. Capture fresh measured state and all camera views. Record residual position
   errors against the intended segment endpoint; do not replace that endpoint
   with measurement to declare success.
5. Continue the fold only when the observed motion supports the next action and
   contact targets are available. This is an execution assist, not a calibration
   campaign or evidence that the towel was grasped.

Keep MCAP active through each segment, assist, and inspection. Correction is
withdrawn before the next moving target. Existing driver guards remain active.

## Result

Report success, failure, indeterminate, or blocked-before-motion. Include the
recording and image paths, the first phase where observation differs from intent,
the evidence timestamp, and distinguish observed behavior from a proposed cause.
A closed gripper does not prove a grasp. Completed commands do not prove a fold.
Missing arm/gripper backend tools are a capability failure, not a physical attempt.
Do not claim a folded towel unless the final released state supports it.
