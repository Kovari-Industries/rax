---
name: motion-preview
description: Watch proposed R1Pro rows or a planned arm motion from fixed third-person cameras on a robot-only simulator before running them in the episode. Use before large trunk, arm or base motions, or to compare candidates from the same start.
---

# Motion preview

`behavior_preview_run` replays motion on a separate simulator that holds only the
same R1Pro on a floor plane, with the same controllers and 30 Hz rows as the
episode. It does not change the episode or count toward its steps.

## Workflow

1. Build the candidate: `behavior_step` rows (`actions`), or a
   `behavior_free_space_move` plan with `preview_only` (`motion_plan_id`).
2. Run the preview. Omit `initial_state` to start from the episode's measured joints
   and gripper commands; pass it to try a different start.
3. Inspect the final `preview_front`, `preview_side` and `preview_top` frames and the
   reply's `end_state` (joints, end-effector poses in the base frame) and
   `base_motion`. Open earlier frames from `frame_sequence` when the path matters.
4. If the motion is what you intended, send the same rows with `behavior_step`, or
   execute the plan with `behavior_execute_motion_plan` (a previewed plan stays
   executable until the robot moves). Otherwise adjust and preview again.

## Limits

The preview has no scene, furniture, objects or contact. It shows the robot's own
motion and self-collision only; check clearance, grasps and task progress with the
episode cameras. `behavior_preview_describe` lists the cameras, inputs and outputs.
