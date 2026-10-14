---
name: free-space-move
description: Plan and execute short or long single-arm EEF translations, rotations or combined pose changes with cuRobo; inspect accuracy and replayable 25 Hz joint targets.
---

# RoboDojo cuRobo free-space move

`robodojo_free_space_move` plans and executes a single-arm move to a target EEF pose;
[robot-interface](../robot-interface/SKILL.md) says when to use it and when
`robodojo_step_eef`. Neither certifies task-object clearance or task success; verify
the measured result after execution.

## Ground and format the target

1. Read current measured EEF poses and joints with `robodojo_observe`. Use current
   perception to ground the goal; a visible surface point is not an object pose
   or the robot's grasp center.
2. Build a complete environment-origin `link6` pose. Use
   [pose math](../pose-math/SKILL.md) for frame-aware changes and
   `format_target(target_kind="free_space_move")`.
3. Move only the selected arm. Omitting its gripper opening preserves the measured
   opening at plan start. Let the formatter and live MCP schema supply the exact
   target fields; EEF stepping and free-space targets are not interchangeable.

The planner models robot self-collision and a static table. It does not receive
{% if camera_geometry %}
task-object poses, depth geometry, the other arm's collision geometry, or live
gripper geometry; its gripper model is fixed.
{% else %}
task-object poses, the other arm's collision geometry, or live gripper geometry; its
gripper model is fixed.
{% endif %}
Choose approach and retreat poses with those limits in mind. Planner success does
not establish a safe grasp, object clearance or successful manipulation.

## Execute directly or review first

Execute directly when the goal and collision assumptions are clear. Normal calls
plan and execute; preview is optional, not a prerequisite for every move.

When the path is uncertain, call with `preview_only: true` and
`include_trajectory: true`. This plans and caches without moving the robot:

- Require `motion_plan.status == "Success"`; inspect target/FK diagnostics and
  the returned collision model. Do not execute a rejected plan.
- Check the returned timing and remaining-budget fit, especially near the
  episode limit. Use nearer intermediate goals if a move exceeds tool limits.
- For an optional physical robot-only preview, pass `motion_plan.start_state`
  as `initial_state` and `motion_plan.trajectory_preview.actions` as `actions`
  to [action-preview](../action-preview/SKILL.md). These are the
  complete 25 Hz targets, not display-downsampled or dense velocity commands.
- If acceptable, execute the same cached ID with
  `robodojo_execute_motion_plan(motion_plan_id=...)`; do not replan unnecessarily.
  Any intervening robot action invalidates it. Read-only observations do not,
  but measured-state drift can. On `Plan_Stale` or `Plan_Not_Found`, discard the
  ID, observe and replan. Do not replay an old path from a different start state.

Use [action search](../action-search/SKILL.md) if comparing a few target
orientations or approach candidates would resolve uncertainty.

## Recover from planning failure

Read the returned `failure_stage`, `reason` and diagnostics; a generic failure does
not prove a specific collision or an unreachable target. For a path failure, execute
an intermediate pose that lifts, retracts or reorients the arm, observe, then plan the
next leg from the new state. If the goal itself stays infeasible, choose a different
approach orientation or position instead of repeating the request. Keep collision
checks enabled; do not use EEF or joint stepping to bypass a rejected plan.

## Verify the physical result

For direct or cached execution, `transition.status == "Success"` means all
command rows ran—not that the EEF reached the goal. Check
`transition.goal_reached` and `transition.measured_goal_error` separately against
the reported tolerances. Planned FK error is not a substitute for measured
post-execution error.

Inspect the attached final image and measured pose. If tracking is insufficient,
inspect the scene and any terminal/error flags before a fresh corrective plan or
small EEF adjustment. Never continue motion after episode end or blindly retry an
ambiguous execution. Even precise EEF arrival does not prove object/task success.

When motion dynamics matter, inspect selected before/intermediate/final images
using returned frame-sequence paths. Every executed 25 Hz frame is saved; there
is no need to load the whole recording into context.

## Replay

Actual executed commands are `transition.steps[*].executed_action`. They use the
same joint-target contract as `robodojo_step`; replay within its batch limits
from a compatible initial state. On `Partial`, only the returned executed
prefix is valid execution data, not the full planned sequence.

Pair each command with its preceding `observation_step_id` and observation,
not the post-action measured state.
