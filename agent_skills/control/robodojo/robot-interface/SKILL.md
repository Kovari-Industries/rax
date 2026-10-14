---
name: robot-interface
description: The dual ARX X5 table-top robot in RoboDojo - which motion tool to use for which objective, how bounded EEF steps behave, and the related pose, preview and search skills. Read before composing robot actions.
---

# Robot interface: dual ARX X5

Two six-joint arms with parallel grippers over a table. End-effector poses are
absolute `link6` poses in environment-origin metres with unit `[qw, qx, qy, qz]`
orientation; the live tool schemas give every argument and limit.

## Choose the tool to match the objective

- **One arm, one target pose:** prefer `robodojo_free_space_move` for short or
  long translations, rotations, or both, when cuRobo may choose the path. It
  usually provides better endpoint position/orientation accuracy than bounded
  `robodojo_step_eef` updates, so prefer it when accurate goal reaching is the
  priority. Verify the measured result rather than relying on planner success. Follow
  [free-space-move](../free-space-move/SKILL.md) for target formatting, optional
  preview and physical goal-error checks. A nearby target does not require EEF
  stepping by default.
- **Explicit local path or coordinated arms:** use `robodojo_step_eef` for
  observation-driven approach, contact, insertion, sliding, lifting or rotation
  adjustments where the intermediate targets matter. It is bounded local IK,
  not a full collision-aware trajectory planner; inspect the resulting motion.
- **Established joint command or replay:** use `robodojo_step` for known absolute
  joint targets, including executed targets recorded by the other motion tools.
  Joint angles alone do not specify the desired Cartesian path.

## Use `robodojo_step_eef` deliberately

Supply absolute dual-arm waypoints. Format each arm separately with
[pose-math](../pose-math/SKILL.md), then assemble the pair using the live tool schema.
Hold an inactive arm at its measured pose and preserve its measured gripper opening.

Each waypoint produces one bounded IK update from the **measured state after the
previous waypoint**, not from the previous requested pose. Solver limits bound
the update; they do not guarantee an achieved displacement.
One waypoint does not necessarily reach a distant goal: observe the actual EEF,
check `transition.ik_diagnostics`, and choose the next target accordingly. You
supply poses; you do not need to predict IK joint deltas.

Batch consecutive waypoints while motion is predictable. Shorten chunks as
contact or occlusion uncertainty increases so you can correct from observations.
For relative intent such as a lift or wrist rotation, compose an absolute target
with `robodojo_pose_math`; do not add quaternion components or send delta actions.
For gripper-only changes, use current measured EEF poses with the chosen gripper
fields. Verify object contact or retention visually rather than inferring it
from the commanded opening.

{% if multi_env %}
## Environment selection

- Pass the selected `env_id` on every `robodojo_*` call except `robodojo_pose_math`,
  on `robot_preview_*`, and on `start_episode`, `evaluate` and `finish`; for example
  `robodojo_step(env_id=i, actions=...)` or
  `robodojo_free_space_move(env_id=i, arm=..., target=...)`. `robodojo_status`
{% if agent_formal %}
  reports one environment; `status`, `robodojo_pose_math` and `gemini_generate` take
  no `env_id`. Live schemas define the other arguments.
{% else %}
  reports one environment; `status`, `robodojo_pose_math`,
  `gemini_generate`, `register`, `rehearse` and `submit` take no `env_id`.
  Live schemas define the other arguments.
{% endif %}
- Keep multi-call sequences in one environment: execute a previewed plan with
  `robodojo_execute_motion_plan` in the environment that planned it, as
  [multi-env exploration](../multi-env-exploration/SKILL.md) describes.
{% endif %}

## Related skills

- [pose-math](../pose-math/SKILL.md) for pose composition, frame changes and target
  formatting. If the tool fails, do not improvise a replacement conversion; an
  already valid absolute target can still be used.
- [free-space-move](../free-space-move/SKILL.md) for planned moves to pose targets;
  inspect goal-reaching feedback, not just planning success.
- [action-preview](../action-preview/SKILL.md) to inspect candidate joint
  trajectories when useful. Preview is optional and cannot establish object
  contact, grasp or task success.
- [action-search](../action-search/SKILL.md) when comparing plausible candidate
  motions from the same measured starting state.
{% if not formal_episode %}
- [references/pose-records.md](references/pose-records.md) for how poses and executed
  actions are recorded for reuse.
{% endif %}
