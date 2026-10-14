---
name: object-localization
description: Locate visible object points in the robot's frame from the current observation, choosing the method the observation profile allows (depth with camera geometry, RGB with camera geometry, or RGB with measured robot motion only). Use before choosing any approach, grasp, placement or drive target, and again after anything moved.
---

# Object localization

Ground every target in the current observation. Never reuse pixels or positions after
the camera, the robot base or the object moved, and never replace missing geometry with
guessed coordinates. Fill in the robot's facts in [This robot](#this-robot); the
methods below are the same everywhere.

## Pixels

- Select pixels in the original, full-resolution image of the current observation, not
  in a downscaled preview or a composed video; convert preview pixels with the scale the
  reply gives.
- Pick several pixels inside the object's visible surface, away from edges, shadows and
  occluders: edge pixels take the depth of the background.
- Prefer features that look distinct from every view (tips, corners, marks); symmetric
  features are easy to mismatch between views.

## Choose the method

**Depth and camera geometry.** Back-project pixels with `<pixel_to_position>` (or with
the depth image, intrinsics and camera pose in your own code,
[references/geometry.md](references/geometry.md)). Discard points with invalid or
sparse depth, or far from the others. Cross-check from a second view or camera before a
precise action; disagreement beyond the robot's tolerance means a wrong pixel, so
reselect rather than average.

**RGB with camera geometry.** One pixel gives a ray, not a point. Either intersect the
ray with a known plane (a support surface of known height; use the object's contact
point with it, not its top), or triangulate the same physical feature from two camera
positions far enough apart that the rays meet at a wide angle with a small gap. Check an
estimate by projecting it back into a new image where the robot offers that.

**RGB only, no camera geometry.** Images give directions, not metric positions. Learn
how the image maps to robot motion by moving an empty gripper a small, known
displacement and watching its visible feature. Align the same gripper feature with the
target in fresh images from more than one view, with small motions near a safe height.
One image's overlap does not prove alignment or contact (parallax); confirm in a second
view or after a small motion. A learned mapping holds only near that height, orientation
and camera.

## Use the result carefully

- A localized point is a visible surface point, not an object centre, pose, grasp or
  clearance. Average several top-surface points for a centre, offset toward the centre
  by about half the object's thickness for a grasp, and choose approach offsets from
  current evidence.
- Close views resolve far more detail than overview cameras; get one with
  [camera-inspection](../camera-inspection/SKILL.md) for contact-critical targets, or
  confirm by touch where the robot senses force.
- Record in memory what you localized against, the views and the method, not one
  layout's coordinates.

## This robot

| Fact | Value |
|---|---|
| Cameras, resolution, what moves each | `<from robot-interface>` |
| Where a frame's images, depth, intrinsics and camera pose are saved | `<path and file names>` |
| Depth units and invalid value; where depth is unreliable | `<measured>` |
| Robot frame and units of returned points | `<from robot-interface>` |
| Cross-view tolerance, minimum ray angle, maximum triangulation gap | `<measured on this robot>` |
| Known plane heights (floor, table) | `<measured>` |
| Robot-body pixels to ignore | `<measured>` |
