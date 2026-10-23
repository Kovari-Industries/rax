---
name: live-stream
description: The continuous, read-only recording of the robot's cameras and state over the recent past. Use before acting after any pause, to see what changed while you were not looking, and to review what happened during a motion or a script run.
---

# Live stream: the world keeps changing

On a real robot, people move or remove objects, things settle, slide or fall, and
seconds to minutes pass while you think or write code. A frame you looked at earlier
describes the past, not the present. Where the harness records continuously,
`runtime/stream/` (read-only) holds a rolling window of every camera and the robot's
state; older data is deleted, so copy what you need to keep into `output/`.

## Two ways in

- **As a tool:** `<stream>` returns frames from chosen moments ("seconds ago") with the
  robot state then, or a contact sheet per camera over a time span. It moves nothing and
  needs no arming.
- **As files,** for code and bulk analysis: the stream helper module
  (`python -m api.stream ...`, or `from api import stream`) lists recent frames, builds
  contact sheets, finds the moments an image changed, and reads the robot-state history.

## When to look

- **Before any motion that follows a pause:** check that the scene still matches what
  you planned against. If something moved, localize again before moving.
- **What changed while you were away:** look at a contact sheet over that span, or list
  the moments the image changed and fetch frames at those moments. Compare with the
  robot-state history: a change while the robot was still was caused by something else.
- **After a motion or a script run:** review the frames during it (a slip, a knocked
  object, an arm that stopped early) instead of guessing from the end state.
- **In controllers:** check that the scene is quiet before committing to a motion, and
  keep evidence frames of each step.

Metric geometry still comes from `<observe>` frames and
[object-localization](../object-localization/SKILL.md); the stream is for timing,
change and review. Look at a few selected frames or sheets, never whole minutes of
images.
