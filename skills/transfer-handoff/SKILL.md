---
name: transfer-handoff
description: Write handoff/README.md, describing how to plan the motion for each subtask and object manipulation of this task, for a later session (or a later copy of you) that continues the task on scenes it has not seen. Use before exploration ends, and update it whenever you learn something new.
---

# Transfer handoff

A later session continues this task on new scenes, starting from `handoff/` and the
folders AGENTS.md says are kept, not from frames or logs. `handoff/README.md` tells it
how to plan each motion of the task so it does not have to work them out again.

## What to write

Start with the subtasks in order, one line each: what marks it complete in a fresh
observation and the `TASK.md` condition it serves.

Then give each subtask a section with one entry per object manipulation (reach, grasp,
lift, carry, place, push, release, ...):

- **Object and goal:** which object, what must change, and how to see that it did.
- **Locating it:** how to find the object and the target in a fresh observation (which
  camera, which visible feature, which skill or tool), and what to locate again after
  each move.
- **Motion plan:** the ordered moves, each relative to something observed (for example
  "\<distance> above \<observed feature>, \<orientation>"), with the arm, the tool that
  plans or executes each move, the gripper opening, approach and retreat directions,
  and the step sizes that worked.
- **Parameters:** each value with its unit, what it is measured from, the range tried
  and the value that worked.
- **Checks and failures:** how to confirm the manipulation worked, the failures seen,
  how to recognize each and what fixed it.
- **Code:** the function under `code/` that performs it and how to call it, if any.

Write what you verified and mark anything untested. Give positions relative to observed
features, never absolute coordinates from one layout. State facts, not paths that will
not resolve later. Keep it as text, with at most a few decisive images copied into
`handoff/`, and keep the kept folders within the size limit `status` reports.
