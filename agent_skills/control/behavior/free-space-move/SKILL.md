---
name: free-space-move
description: Move an R1Pro end effector to a pose in the robot base frame, either with the cuRobo planner (arm, optionally with the trunk) or with arm-only closed-loop IK. Use for reaching, pre-grasp approaches, arm reconfiguration and short local moves; use behavior_step rows for grasping and gripper changes.
---

# Arm motion to a pose

Both tools take a target `position` (metres) and `quaternion_xyzw` in the **robot base
frame**, the frame of `proprio.eef_*_pos` and `eef_*_quat`, and move one hand while the
other hand holds its pose. Neither knows where tables, walls, doors or task objects are.

| Tool | Moves | Use for |
|---|---|---|
| `behavior_free_space_move` | one arm planned by cuRobo around the robot itself; with `use_trunk: true` also the trunk | larger moves and reconfiguration; with the trunk, lower, higher or farther reaches |
| `behavior_step_eef` | one arm only, closed-loop inverse kinematics toward the pose | approaches of up to about 20 cm, contact, pushing, pressing, small corrections |

With `use_trunk: true` the planner may move the trunk a lot even for a modest reach,
which swings the head camera and the whole upper body; use it only when the arm alone
cannot reach. Prefer `behavior_step_eef` near objects and furniture.

## Choosing a target

- Start from measured poses: the current end-effector pose is in `proprio`.
{% if camera_geometry %}
- Get object positions from [object-localization](../object-localization/SKILL.md).
{% endif %}
- Approach in two stages: a pre-grasp pose 8-15 cm back from the object along the
  approach direction (planner or IK), then a short final approach with
  `behavior_step_eef`. Keep the gripper open before contact.
- Keep the current orientation when you only need to translate: reuse the measured
  quaternion.
- If the target is out of reach, move the base closer first
  ([base-navigation](../base-navigation/SKILL.md)) or plan with `use_trunk: true`.

## Preview, execute, verify

- `behavior_free_space_move` with `preview_only: true` returns a plan summary,
  including how far the trunk moves, and a `motion_plan_id` without moving;
  `behavior_execute_motion_plan` runs it only if the robot has not moved since.
- `status: Planning_Failed` executes nothing; the target is unreachable or would
  self-collide.
- `behavior_step_eef` reports `reached` and the final position and orientation
  errors. It stops after `max_steps`. If the target is not reached, a joint may be at
  its limit, the pose may be out of the arm's range, or the hand may be blocked: plan
  the move instead, or approach from a different arm posture.
- After either tool, compare the measured end-effector pose with the target and check
  the wrist camera: a large error usually means contact with something the planner
  cannot see.

## Grasping after the approach

From the pre-grasp pose, advance with `behavior_step_eef`, close the gripper over
several `behavior_step` rows while holding the arm (gripper command toward -1), then
lift a few centimetres and check the wrist view and `gripper_openings` for a secure grasp.
