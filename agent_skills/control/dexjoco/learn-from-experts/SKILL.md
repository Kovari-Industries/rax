---
name: learn-from-experts
description: Use the recorded human demonstrations of this task in runtime/experts/ - teleoperated episodes on other layouts, with their official action rows, states and camera videos - to learn approach poses, hand postures and timing. Not a replay script.
---

# Learn from expert episodes

`runtime/experts/` holds {{ expert_demonstrations }} human teleoperation episodes of
this task, recorded by the benchmark's authors on other layouts with a data glove and
trackers, in the same simulator and with the same controller you use. Each episode
directory has:

- `trajectory.npz`: `action_rotvec` (rows in exactly the `dexjoco_step` layout),
  `state` (the official state vector; values after its first
{% if robot.arms == 'dual' %}46{% else %}23{% endif %} entries describe the
  recording's initial layout and are not available to you during an episode) and
  `timestamp` (seconds, {{ robot.hz }} rows per second).
- `replay.zarr/`: the same data in the original recording format.
- `videos/*.mp4`: the recording's camera videos.
- `episode.json`: the step count and how the episode was verified.

Use them to learn what works: where the flange goes relative to an object before
contact, the hand posture and how far past contact the fingers close, how fast the
hand moves near contact, and the order of phases. Measure these from the actions and
states, look at the videos, and turn them into rules that adapt to what you see.

The layouts differ from yours: replaying an episode's rows will not reproduce its
outcome. Read the files with your own code (numpy); they are read-only.
