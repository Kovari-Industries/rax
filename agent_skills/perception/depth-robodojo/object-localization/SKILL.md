---
name: object-localization
description: Use when a RoboDojo agent needs the 3-D environment position of a visible object surface point selected in one depth-enabled camera view or matched across multiple RGB views.
---

# Object localization: depth

Use `robodojo_pixel_to_position` with pixels selected from the current
observation. It uses observed depth/calibration or multi-view geometry, not
privileged object poses, segmentation or scene-manager state.

1. Retain the observation `step_id`, camera names, and original images. Do not
   select pixels from a composed video or stale observation.
2. Select a point inside the visible surface, using normalized coordinates if
   display resizing makes original pixels unreliable.
   Preserve camera names and pass the current `observation_step_id`. The live
   tool schemas define coordinate conventions and depth-sampling options.
3. Use `method: "auto"`:
   - With valid depth, one selected view is sufficient. The result is the
     depth-unprojected visible surface point. With several depth views, check
     `quality.max_depth_point_spread_m`: views should agree within millimetres.
   - Without valid depth, select the same recognizable physical feature in
     at least two views of this observation; do not pair unrelated silhouette
     centers. The tool triangulates only views from one observation, so place
     the cameras first and select in their simultaneous images.
4. Require `Success`, then check the quality yourself. The defaults are lenient:
   `Success` allows rays only 2° apart that miss by 3 cm.
   - Pass `max_triangulation_residual_m: 0.005` for manipulation targets.
   - Require `quality.minimum_ray_angle_degrees` of at least 10–15° and
     `quality.all_rays_forward`.
   - A large residual usually means the pixels mark different points. Reselect;
     do not average it away.
   - A point with `Low_Confidence` is not reliable geometry.
5. Treat `position_environment_m` as a visible surface point, not an object
   center or pose.

## Choose the views

All three cameras are 640 x 480, but they resolve different detail:
- `cam_high` sees the whole scene from about 0.6–1 m, about 2–3.5 mm per pixel.
- A wrist camera 0.15–0.3 m from the target resolves about 0.4–0.8 mm per pixel.

Use `cam_high` for the overview and a close wrist view for precision. Position an
empty arm's camera with [wrist-camera inspection](../wrist-camera-inspection/SKILL.md)
so the target is sharp and unoccluded and its ray meets the other camera's ray at a
wide angle. Prefer features that look distinct from every view (tips, ends, unique
corners); symmetric features are easy to mismatch.

## After motion

After the scene or wrist cameras move, observe again and reselect the point.
Wrist-camera calibration changes with robot motion. Never reuse a point after
`step_id` changes without confirming it from a fresh observation. A surface point
does not establish object orientation, full pose, collision clearance or a grasp;
choose task-appropriate approach offsets using current evidence and pose math.
