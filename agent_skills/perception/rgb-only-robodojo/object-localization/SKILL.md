---
name: object-localization
description: Locate RoboDojo objects from RGB images and measured robot poses, without depth, camera intrinsics or camera poses. Use to judge where objects are relative to the grippers before reaching, grasping or placing.
---

# Object localization: RGB only

Only RGB images and measured robot state are supplied: no depth, camera intrinsics
or camera poses. One image gives a direction, not a metric position, so never turn
image pixels alone into coordinates. Do not seek simulator files or hidden camera
or object state.

`eef_positions` and `eef_quaternions_wxyz` are measured `link6` poses,
ordered [left, right], in the environment frame. They are not fingertip,
grasp-centre, camera or object poses. Use achieved poses, not commanded targets;
the visible gripper feature may be offset from link6 and move when the gripper
opens or its orientation changes.

## Find the target

- Use `cam_high` for the overview: where the target is relative to the grippers,
  the table edges and other objects.
- Learn how the image maps to the environment: move an empty gripper a small,
  known displacement at a safe height and see how its visible feature moves in a
  fresh image. Do not assume image axes equal environment axes.
- A close wrist view ([wrist-camera inspection](../wrist-camera-inspection/SKILL.md))
  resolves much finer detail than `cam_high`. Recheck after moving any camera.

## Align with measured motion

- Bring an empty gripper near the target at a safe height, then compare the same
  visible gripper feature with the target in fresh images and correct with small
  motions. Use [free-space motion](../free-space-move/SKILL.md) for clear
  approaches and [pose math](../pose-math/SKILL.md) to format targets.
- Judge height and alignment from more than one view: a pixel overlap in one image
  does not prove contact or alignment, especially above the table (parallax).
  Confirm in a second camera or after a small motion before contact.
- A learned image-to-motion mapping holds only near that height, orientation and
  camera; refit after large changes and check it against the current scene before
  reusing it. Alignment motions consume episode steps; keep them few and small.
- Allow clearance for the whole gripper and any held object, and verify grasp and
  placement visually.
{% if not formal_episode %}
Record in memory what you aligned against, the views and motions used, and the
outcome, not coordinates.
{% endif %}
