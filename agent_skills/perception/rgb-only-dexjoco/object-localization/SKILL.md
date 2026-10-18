---
name: object-localization
description: Locate DexJoCo objects from RGB images and measured flange poses, without depth, camera intrinsics or camera poses. Use to judge where objects are relative to the hand before reaching, grasping or pressing.
---

# Object localization: RGB only

Only RGB images and the measured state are supplied: no depth, camera intrinsics or
camera poses. One image gives a direction, not a metric position, so never turn image
pixels alone into coordinates. Do not seek simulator files or hidden camera or object
state.

The state's flange poses are the arm's tool plate, not fingertip, palm, camera or
object poses; the fingertips are about 0.18 m beyond the flange at the start posture
and move with the finger joints.

## Find the target

- Use `{{ robot.overview_camera }}` for the overview: where the target is relative to the hand,
  the table edges and other objects.
- Learn how the image maps to the world: move the empty hand a small, known flange
  displacement at a safe height and see how a visible hand feature moves in a fresh
  image. Do not assume image axes equal world axes.
- The wrist camera resolves much finer detail near the hand. Recheck after moving
  any camera.

## Align with measured motion

- Bring the empty hand near the target at a safe height, then compare the same
  visible hand feature with the target in fresh images and correct with small flange
  moves.
- Judge height and alignment from more than one view: a pixel overlap in one image
  does not prove contact or alignment (parallax). Confirm in the other camera or
  after a small motion before contact.
- A learned image-to-motion mapping holds only near that height, orientation and
  camera; refit after large changes. Alignment motions consume episode steps; keep
  them few and small.
{% if not formal_episode %}
Record in memory what you aligned against, the views and motions used, and the
outcome, not coordinates.
{% endif %}
