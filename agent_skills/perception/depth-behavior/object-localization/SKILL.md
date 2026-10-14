---
name: object-localization
description: Convert pixels in the current R1Pro head or wrist camera images into metric 3-D points in the robot base frame using depth. Use to localize objects, surfaces and handles for navigation and arm targets.
---

# Object localization: depth

Depth attachments give linear depth along the optical axis, and observations carry
`camera_intrinsics` and `camera_poses` (camera pose in the robot base frame, quaternion
`xyzw`).

`behavior_pixel_to_position` back-projects up to 32 pixel selections from the
**current** observation into the robot base frame, using the linear depth image,
the camera intrinsics and the camera pose. Pass the observation's `step_id` as
`observation_step_id`; any robot motion makes the selection stale.

- Choose pixels on the object's visible surface near its centre, not on its edge:
  edge pixels often take the depth of the background. `depth_spread_m` reports how
  much depth varies in the sampled window; a large spread means an edge or a thin part.
- `valid: false` means no usable depth at that pixel (too far, a hole or a reflective
  surface). Pick another pixel or view from another camera.
- The returned point is on the surface facing the camera. For a grasp target, offset
  it toward the object's centre by about half its thickness along the viewing ray.
- A wrist camera close to the object gives the most accurate points; the head camera
  covers the whole scene at lower precision.
- Points are in the base frame at this step. Convert them to odometry coordinates
  before moving the base if you need them later (see
  [robot-interface](../robot-interface/SKILL.md)).
