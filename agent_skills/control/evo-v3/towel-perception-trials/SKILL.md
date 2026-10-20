---
name: towel-perception-trials
description: Compare SAM towel outlines, local image-edge refinement, and temporal optical flow on saved Evo camera trials. Use to diagnose shallow-view corner instability before metric alignment.
---
# Towel perception trials

## Status
Observed in a fixed raised-arm scene. Not task-confirmed; no physical alignment or fold
was completed with this procedure.

## Preconditions
Fresh head and wrist images with camera-specific intrinsics, source capture IDs, and
one fixed pose. Preserve missing/ambiguous detections. These tests require no movement.

## Inputs
Use the Evo `evaluate_trial_geometry.py` tool with `--results`, `--images`, `--requests`,
and `--output`. Results contain `camera`, `index`, `status`, `corners_xy`; images use
`NN-camera.jpg`; requests use `camera-request.json` with the camera calibration.

## Procedure
1. Inspect images to confirm the intended towel. SAM is one semantic detector, not depth.
2. Correct the lens once. Fit the lower visible image edge, and measure its height at a
   fixed image X separately from the side-corner intersections.
3. Test local gradient refinement as an optional tool. Keep rejected frames in coverage
   counts. Do not widen its search until a desired edge is found.
4. Check forward/backward optical-flow consistency on towel texture between stationary
   frames. These are temporal correspondences in one camera, not stereo matches.
5. Freeze settings, capture a new trial, and compare failure rate and accepted-fit spread.

## Expected effect
Separate unstable side intersections from a stable line that may be useful for later
alignment. At one pose the SAM line was stable even when a corner moved by 20 pixels.
Gradient refinement helped the right wrist in the first capture, but the benefit did
not repeat in a new capture. It failed on the left in both. Do not apply one refinement or crop to every view.

## Failure handling
If identity, edge support, or tracking is ambiguous, return the failure and frame IDs.
No movement follows from an image-only result. The lower image edge is not a verified
physical near edge; mounting transforms and capture-time joints remain necessary.

## Evidence
See `EVO_V3.md` for the dated result table and the Evo package README for MCAP references.
Raw frames and per-frame results remain in the local artifact directories. No workspace
images or credentials are published with this skill.
