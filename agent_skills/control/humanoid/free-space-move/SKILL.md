---
name: free-space-move
description: Move one G1 hand to a pose in the pelvis or torso frame with the cuRobo arm planner, optionally using the waist, while the legs and everything else hold your last frame. Use for reaches, pre-contact approaches and arm reconfiguration; use humanoid_step frames for walking, contact and hand closing.
---

# Arm motion to a pose

`humanoid_free_space_move` plans one arm (`arm: left | right`) to a target `position`
(metres) and, optionally, `quaternion_wxyz` in a body frame (`frame: pelvis | torso`), for an
`end_effector`: `palm`, `hand_center` or `wrist`. Without a quaternion it plans position only.
The plan starts at the measured arm joints and avoids the robot's own body only: walls,
furniture and task objects are unknown to it.

The executed plan is one semantic40 frame per 50 Hz tick: the planned arm joints (with
`use_waist: true` also the three waist joints, kept within modest limits); every other joint,
both hands and the root rotation stay at the last frame you sent (at the measured state before
your first frame). The legs keep their references, so the robot does not step. `time_scale`
(1 to 4) slows the same path down.

## Choosing a target

- Express it relative to something you measured: the current hand pose, or a point from
  [object-localization](../object-localization/SKILL.md).
- Approach in two stages: a pre-contact pose 8 to 15 cm back along the approach direction with
  the planner, then a short final approach with `humanoid_step` frames in small corrective
  steps, settling before you judge where the hand ended.
- Reach is comfortable about 0.35 to 0.45 m ahead of the pelvis; beyond it errors grow and
  contact pushes the robot back ([controller-field-guide](../controller-field-guide/SKILL.md)).
  Walk closer first rather than stretching.
- The waist moves the torso, the other arm and the camera with it: re-observe after a waist
  plan.

## Preview, execute, verify

- `preview_only: true` returns the plan (duration, end-effector error, solved joints) and a
  `motion_plan_id` without moving; `humanoid_execute_motion_plan` runs it only if the robot has
  not moved since. A plan that fails executes nothing.
- The reply carries the usual `transition.feedback` and executed frames. Check the measured
  result, not just planning success: the controller follows arm references with a lag of a few
  tenths of a second, and a hand that met something the planner cannot see stops short.
- A plan replaces your arm references only; walking, turning and closing a hand stay
  `humanoid_step` frames built from the last frame you sent.
