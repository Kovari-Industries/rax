---
name: object-localization
description: Convert pixels in the current DexJoCo camera images into metric 3-D points in the world frame using depth. Use to localize objects, handles and contact points for flange and fingertip targets.
---

# Object localization: depth

Depth attachments give linear depth along each camera's optical axis, and
observations carry `camera_intrinsics` and `camera_poses` (camera pose in the world
frame, quaternion `[w, x, y, z]`, OpenGL convention: the camera looks along its -Z
axis with +Y up; image row 0 is at the top).

`dexjoco_pixel_to_position` back-projects up to 32 pixel selections from the
**current** observation into the world frame. Pass the observation's `step_id` as
`observation_step_id`; any robot motion makes the selection stale.

- Choose pixels on the object's visible surface near its centre, not on its edge:
  edge pixels often take the depth of the background. `depth_spread_m` reports how
  much depth varies in the sampled window; a large spread means an edge or a thin part.
- `valid: false` means no usable depth at that pixel. Pick another pixel or use the
  other camera.
- The returned point is on the surface facing the camera. For a grasp target, offset
  it toward the object's centre by about half its thickness along the viewing ray.
- Cross-check: the same feature seen from {{ robot.overview_camera }} and from the wrist camera
  should give the same world point within a few millimetres; if not, one pixel is off.
- The wrist camera moves with the hand: a close wrist view gives the most accurate
  points, the overview covers the whole table.
- Points are world-frame and stay valid until something moves the object. Flange
  targets are not fingertip targets: subtract the hand's offset (see
  [robot-interface](../robot-interface/SKILL.md)).
