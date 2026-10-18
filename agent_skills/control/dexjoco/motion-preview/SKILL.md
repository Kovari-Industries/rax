---
name: motion-preview
description: Dry-run Panda + Allegro action rows on a robot-only simulator under the official controller and watch them from fixed third-person cameras before spending episode steps. Use before large or unfamiliar arm motions, to check reach, wrist rotation, elbow posture and hand shapes, or to compare candidate rows from the same start.
---

# Motion preview

`dexjoco_preview_run` runs `dexjoco_step` rows on a separate simulator that holds
only this task's robot{% if robot.arms == 'dual' %}s{% endif %} on a floor plane. The rows go through the same official
conversion and controller as in the episode (the flange controller and its gains,
20 ms of physics per row, the hand's position servos), so the flange path, its
tracking lag and the hand shapes follow the episode's free-space motion. The
preview does not change the episode or count toward its steps.

## Workflow

1. Build the candidate rows as for `dexjoco_step`
   ([robot-interface](../robot-interface/SKILL.md)).
2. Run the preview. Omit `initial_state` to start from the episode's measured state
   vector; pass a state vector to try another start, such as the state an earlier
   motion should end in.
3. Read `start_settle`. Each run first drives the arm from its home posture to the
   start flange pose; `converged: false` or a large flange error means that pose is
   hard to reach, and the rows start from where the arm got.
4. Inspect the final `preview_front`, `preview_side` and `preview_top` frames and
   compare `end_state` with the targets you meant to reach. Open earlier frames from
   `frame_sequence` when the path matters; each saved frame carries its state vector.
5. If the motion is what you intended, send the same rows with `dexjoco_step`.
   Otherwise adjust them and preview again.

## Limits

The preview has no table, objects or contact with the scene: it shows the robot's
own motion and self-collision only. Fingers close freely where an object would stop
them, and nothing is grasped or moved; check clearance, contact and task progress
with the episode cameras. The state vector has no arm joint angles, so the preview
rebuilds the elbow posture with the controller from the home posture and starts the
arm at rest; after earlier motion the episode arm's elbow sits differently, and the
flange path can differ by a fraction of a millimetre. Check elbow clearance in the
episode cameras. `dexjoco_preview_describe` lists the cameras, inputs and outputs.
