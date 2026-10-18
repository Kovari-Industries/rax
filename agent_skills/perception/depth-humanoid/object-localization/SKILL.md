---
name: object-localization
description: Perceive and localize with the G1 front camera and its depth - calibrated pixel-to-3-D points, depth geometry, gated detection, localization relative to the scene and perception around contact. Use to find objects, surfaces and targets for walking and reaching.
---

# Object localization: calibrated, gated, relative to the scene

The observation gives no world position and no object state. Your heading is measured
(`proprio.root_orientation_world_wxyz` gives the yaw); your position and every object pose come
only from what you see. Every number your perception produces rests on a camera and body model: if
that model is wrong, a whole pipeline can look right and still miss.

## Calibration and verification

Observations carry `camera_intrinsics` and `camera_poses.cam_front` (the camera pose in the pelvis
frame), and the robot model in [robot-interface](../robot-interface/SKILL.md) composes the camera
mount and forward kinematics for any joint angles, so there is nothing to estimate there. The
pipeline that uses them is still worth verifying.

`humanoid_pixel_to_position` back-projects up to 32 pixels of the **current** observation into
the pelvis and level frames, using the median depth in a small window. Pass the observation's
`step_id` as `observation_step_id`; any motion makes the selection stale. `depth_spread_m` reports
how much depth varies in the window: a large spread means an edge or a thin part, and edge pixels
often take the depth of the background. `valid: false` means no usable depth there.

{{ block("humanoid.perception-checks", redo="recalibrate") }}

## Depth

Depth is the distance along the optical axis (not along the pixel's ray) in millimetres, `0` where
invalid; saved frames keep the depth PNG of every saved step.

- **Geometry first.** Support planes (the floor, raised surfaces) fitted with RANSAC, then height
  above a fitted plane, find resting objects, obstacles, edges and steps; connected regions filtered
  by the sizes and shapes in TASK.md come next, and colour separates objects at similar depth.
- **Geometry to measure:** distances for approaching and stopping, and an object's size and
  orientation from the principal axes of its points, compared with TASK.md.
- **Outcomes to check:** after a grasp, whether the object's points rose with the hand; after a
  release, whether the object rests inside the target and the hand has left.
- **Noise and speed.** Depth is noisy at edges, so medians over patches and frames help. Cropping to
  a region of interest and subsampling keeps processing fast.

{{ block("humanoid.robust-detection") }}

Measured sizes of these effects with this controller are in
[controller-field-guide](../controller-field-guide/SKILL.md).
