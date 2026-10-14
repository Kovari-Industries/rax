---
name: action-preview
description: Preview 25Hz joint targets on the dual-X5 robot without task objects, rewards, or privileged scene state.
---

# Robot-only preview

Use `robot_preview_run` for candidate joint trajectories. Its live schema supplies
the supported robot/camera identifiers and input limits. Start from the measured
state before the sequence, not the requested endpoint of an earlier action.
`robot_preview_describe` returns the preview's robot and camera model IDs, its
input schema, its outputs and what it cannot show, without running a preview.

For cuRobo candidates, plan with `preview_only: true, include_trajectory: true`;
use `motion_plan.start_state` and `motion_plan.trajectory_preview.actions`.
For replay, use `transition.steps[*].executed_action` plus its pre-action state.
Pass only absolute 14-D joint targets.

For a quick numeric check without images, `robodojo_fk_preview` runs a 50x14
absolute joint proposal through robot-only forward kinematics; like the preview, it
does not advance the simulation or plan around objects.

Inspect the final image and measured EEF result; use the returned frame paths
if intermediate motion matters. Compare candidates from the same initial state.
Execute a selected fresh cached plan with `robodojo_execute_motion_plan`.

Use the observed state explicitly: the preview cannot see the task scene.
Compare measured joint/EEF outcomes with the intended target, not just the image.
If a candidate moves undesirably, discard or regenerate it; do not execute it
merely because planning succeeded. Viewing a preview does not change RoboDojo.

To choose among several candidates (for example a wrist-orientation search), follow
[action search](../action-search/SKILL.md); this skill covers running one preview.
Do not pass EEF targets directly to this preview tool.

Preview contains robots and cameras only—not objects, table, ground, reward,
or task state. It cannot establish contact, clearance, grasp success,
or task completion. Verify these in RoboDojo.
