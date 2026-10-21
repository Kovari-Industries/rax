# Fold a towel in half

A flat rectangular towel is placed consistently on the table. The robot starts
aligned and within reach. Grasp the two near corners, lift the near edge clear
of the table, bring it toward the far edge, lower it, release, and withdraw.
Inspect the result and report whether the towel was folded in half.

## Execution

Use the Evo backend as the single command owner. Use the existing Evo arbitration,
IK and checked trajectory path. Do not start ACT. Do not reuse historical grasp coordinates.
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

## Current primitives

Use `api.cloth.Cloth.fold(axes=("v",))` with fresh ordered rectangle corners,
measured palm grasp targets, and a task perception function. Choose the fold axis
for the towel placement and robot workspace. Read the `cloth-handling` skill.
Use `evo_follow_path` or `evo_move_pair` for coordinated motion and `evo_set_grip`
for a retained finger position command. Preview and execution use Evo's checks.
Verify that both near corners move with the hands. Verify support and alignment
before release, then inspect the released towel.

The former `evo_step` and `evo_hold_correct` interfaces are replaced. Do not invoke
hardware parameter or torque diagnostics from the agent. Earlier physical trials
reported tracking error; a successful mock test does not resolve that error.
If Evo refuses a path or measured tracking fails, stop and report the constraint.
Do not retry a correction pulse or raise a limit.

## Result

Report success, failure, indeterminate, or blocked-before-motion. Include the
recording and image paths, the first phase where observation differs from intent,
the evidence timestamp, and distinguish observed behavior from a proposed cause.
A closed gripper does not prove a grasp. Completed commands do not prove a fold.
Missing arm/gripper backend tools are a capability failure, not a physical attempt.
Do not claim a folded towel unless the final released state supports it.
