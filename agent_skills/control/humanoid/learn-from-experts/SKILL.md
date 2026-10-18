---
name: learn-from-experts
{% if expert_recordings %}
description: Use the complete expert episodes of this task in runtime/experts/ - teleoperated through the same SONIC controller on other layouts - for phase structure, timing, contact pre-load and reusable references, without replaying their positions. Read before building a contact or locomotion primitive from them.
{% else %}
description: Use the expert episodes of this task in runtime/experts/ - front-camera video, previews and depth of teleoperation through the same SONIC controller on other layouts - for phase structure, timing, stance and contact geometry, without their commands. Read before building a contact or locomotion primitive from what they show.
{% endif %}
---
{% if expert_recordings %}

# Learn from the expert episodes

`runtime/experts/` holds {{ expert_demonstrations }} complete episodes of this task, teleoperated
through the same frozen SONIC controller, each on a layout of its own. Their positions do not
transfer; their structure, timing and command patterns do. Read `runtime/experts/README.md`,
then one episode's `demos/NN/README.md`:

- `trajectory.jsonl`: one JSON line per 50 Hz control frame: the commanded references
  (`vla_action_*`: the frame sent to the controller and its parts, `hand_action_left` /
  `hand_action_right`: the continuous hand targets), the measured robot (`vla_state_*`,
  `robot_root_*`) and the episode's object states (`env_obj_*`). `trajectory-index.json` gives the
  byte range of each line, so you can read single frames.
- `metadata.json`: the episode's layout, initial object poses and joint order; `frames/index.json`
  and `videos/`: front-camera previews and the full video; `episode.npz`: every original array
  (`raw-array-catalog.json` lists them).

Check a field in one line before relying on it, and read selected frames rather than whole files.

## References and measurements, read separately

Each episode holds the expert's commanded references and the measured robot motion. The
controller lags and misses ([controller-tracking](../controller-tracking/SKILL.md)), so the two
differ, and around contact the difference is often deliberate: a reference that presses a hand a
few centimetres into a surface pre-loads the contact. One earlier agent built its contacts from the
measured poses, got no pre-load and failed until it added the expert's reference offsets during the
approach.

- **Reproducing a contact means reproducing the commanded references around it,** offsets into the
  contact surface included, not only the measured pose.
- **The phase structure** is worth extracting: approach distance and stance, pre-contact pose, how
  long the hand takes to close, lift, transport posture, turning rate, release. Your own episodes
  then show how each phase behaves for you.
- **The hands differ.** Your hand command is binary, but recorded hand commands are continuous joint
  targets, and an episode may close the hand only part way. Comparing the recorded finger targets
  with your own closed pose, and measuring the difference, comes before copying a palm pose and
  closing fully.
- **Re-anchoring before reuse.** Recorded headings and positions refer to the episode's own frame
  and layout, not to your start; a reused segment works when expressed relative to your current
  measured pose and the object you perceive.

## Making contact

- **Precision as the contact needs it** ([match-precision-to-error](../match-precision-to-error/SKILL.md)):
  the tolerance of the expert's contact, measured in your own attempts, tells you when to stop
  re-aligning.
- **Approaching and closing the way the expert does:** pre-load during the approach, close, and let
  the fingers settle before lifting; confirm the hold before moving on.
- **Rotated or displaced objects.** A failed attempt often moves the object; re-perceiving and trying
  an alternative arm solution (another target orientation, a mirrored contact) has rescued attempts
  that looked lost.
- **A contact method deserves a fair probe** before rejection: a few attempts at each of two or more
  magnitudes (closure levels, offsets into the surface, closing times), including the magnitude the
  episode itself uses, recorded as k/n per magnitude.
  [references/primitives.md](../controller-field-guide/references/primitives.md) in the field guide
  describes how to turn recorded contacts into reusable primitives.

The episodes are also independent truth for your perception: their object states say where each
object really was in their frames ([object-localization](../object-localization/SKILL.md)).
{% else %}

# Learn from the expert episodes

`runtime/experts/` holds {{ expert_demonstrations }} episodes of this task, teleoperated through the
same frozen SONIC controller, each on a layout of its own, as front-camera observations only. They
show what the expert did, not how it was commanded: there are no commanded references, measured
joint or root motion, hand commands or object states. Read `runtime/experts/README.md`, then one
episode's `demos/NN/README.md`:

- `front_rgb.mp4`: the complete front-camera video, one frame per 50 Hz control step.
- `frames/index.json` and `frames/`: evenly spaced previews with their frame index and elapsed time,
  an overview before you decode the video.
- `depth.npz`: the front-camera depth of every frame (`depth`: float16 metres, frames x height x
  width; `frame_index`).
- `metadata.json`: frame count, frame rate, image size and camera.

Read selected frames rather than whole files: a few previews first, then the stretch of video around
the phase you are building.

## What the frames show

Their positions do not transfer; their structure and timing do.

- **The phase structure** is worth extracting: approach distance and stance, pre-contact pose, how
  long the hand takes to close, lift, transport posture, turning, release. The frame index of each
  phase boundary gives a time budget per phase to compare with your own episodes.
- **Working distance.** Depth at the object and at the robot's hands just before contact gives the
  distance the expert worked from, which your approach can target.
- **Contact geometry**: which part of the hand meets the object, from which side, and whether the
  object moves while the fingers close.
- **Re-anchoring before reuse.** Each scene is the expert's own layout; what you take from it works
  when expressed relative to your current measured pose and the object you perceive.

## Making contact

- **The commands are yours to find.** The frames show the result of the expert's commands, not the
  commands. The controller lags and misses ([controller-tracking](../controller-tracking/SKILL.md)),
  and a hand resting on a surface in the video may have been commanded a few centimetres into it,
  which pre-loads the contact. Measure in your own attempts which commanded offset gives the contact
  the video shows.
- **The hands differ.** Your hand command is binary; how far the expert's fingers close in the video,
  compared with your own closed pose, tells you whether a full closure matches it.
- **Precision as the contact needs it** ([match-precision-to-error](../match-precision-to-error/SKILL.md)):
  the tolerance of the expert's contact, measured in your own attempts, tells you when to stop
  re-aligning.
- **Rotated or displaced objects.** A failed attempt often moves the object; re-perceiving and trying
  an alternative arm solution (another target orientation, a mirrored contact) has rescued attempts
  that looked lost.
- **A contact method deserves a fair probe** before rejection: a few attempts at each of two or more
  magnitudes (closure levels, offsets into the surface, closing times), recorded as k/n per magnitude.

The episodes are also test frames for your perception: your detector run on their frames shows
whether it finds the object across other layouts and viewpoints
([object-localization](../object-localization/SKILL.md)), though without object states they give no
error.
{% endif %}
