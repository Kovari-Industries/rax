---
name: object-localization
description: Perceive and localize with the G1 front camera's RGB images alone - estimating the camera from what you can see, gated detection, localization relative to the scene and perception around contact. Use to find objects, surfaces and targets for walking and reaching.
---

# Object localization: estimated, gated, relative to the scene

The observation gives no world position, no object state, no depth and no camera calibration. Your
heading is measured (`proprio.root_orientation_world_wxyz` gives the yaw); your position and every
object pose come only from what you see. Every number your perception produces rests on a camera and
body model you estimate yourself: if that model is wrong, a whole pipeline can look right and still
miss.

## Estimating the camera

Use these geometric constraints to estimate it, keeping the residuals:

- the known sizes of objects in TASK.md, and straight edges, for the focal length;
- your own hand at a few known joint configurations, with forward kinematics you write from the
  joint angles;
- the measured root orientation, for the body's tilt, instead of assuming a level head.

Posture changes the camera: squatting, leaning and turning the waist move it, so an estimate holds
only for the postures you checked it in. A pixel's distance then comes from intersecting its ray with
a known plane, such as the floor or a surface of known height from TASK.md.

{{ block("humanoid.perception-checks", redo="re-estimate") }}

{{ block("humanoid.robust-detection") }}
