---
name: pose-math
description: Build, inspect, compose, or convert RoboDojo end-effector poses and rotations. Use when an agent needs a position/rotation delta, quaternion/Euler/matrix/axis-angle conversion, or an exact step_eef or cuRobo target.
---

# RoboDojo pose math

Use `robodojo_pose_math`; do not add quaternion components or reimplement pose
math. It is read-only, requires no active episode, and advances no physics.

## Pose and frame conventions

- Positions are environment-origin metres; orientation describes `link6` relative
  to environment axes, not an inferred object frame or fingertip frame.
- Motion targets use unit `[qw,qx,qy,qz]`; observations list arms as `[left,right]`.
  Preserve the tool's normalized quaternion output. `q` and `-q` are the same
  orientation; component-wise differences are not rotation error.
- `compose_pose`/`relative_pose` use the explicit `base_pose` as a fixed reference
  for that call. Choose local or environment axes deliberately.
  Neither implicitly selects the previous action or creates a delta-control mode.

Operations:

- Convert rotations before changing representation.
- Use `compose_pose` and `relative_pose` for frame-aware pose changes.
- Use `format_target` and `extract_target` at MCP action boundaries.

For `robodojo_step_eef`, format each arm separately, then assemble the returned
targets under `left` and `right`; hold an inactive arm at its measured pose.
For cuRobo, format only the selected arm.
Use the formatter for the destination tool rather than reusing another tool's
target object; gripper fields differ between EEF stepping and free-space motion.

Read [references/api.md](references/api.md) when exact arguments, conversion
conventions, or composition formulas are needed.
