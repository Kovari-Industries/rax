---
name: action-search
description: Search among RoboDojo action targets, rotations, or joint trajectories by previewing candidates from one measured robot state. Use when the intended motion is uncertain enough that comparing alternatives is worth the extra planning cost.
---

# Action search

Search only when uncertainty justifies preview cost.

- Start from one current `robodojo_observe` result. Keep the same measured
  state for every candidate.
- Vary only dimensions relevant to the objective. Use `robodojo_pose_math` for
  pose and rotation changes.
  Candidates may differ in approach position, orientation, arm, gripper opening
  or an established joint path; change only what helps distinguish them. Never
  edit quaternion components directly. For rotation search (for example the wrist
  orientation), hold the target position fixed, compare a small set of meaningful
  orientations by their high-camera and active-wrist images and measured EEF
  quaternion, then execute the selected fresh cuRobo plan, or use bounded absolute
  `robodojo_step_eef` adjustments when explicit intermediate control is needed.
- Preview each candidate as [action-preview](../action-preview/SKILL.md)
  describes (plan with `preview_only`, then `robot_preview_run`); never execute a
  search candidate while comparing it. Reject planning failures, excessive motion,
  or unacceptable final FK error.
- Rank candidates by how directly they reach the perceptually grounded target,
  orientation, smoothness, and modeled clearance. Stop when another preview is
  unlikely to change the choice.
  Prefer the simplest satisfactory motion, not the candidate with the most
  elaborate path. Compare final FK/physical preview errors and selected temporal
  frames as well as endpoint appearance.
- Discard undesirable previews. Execute a selected cached plan only while it
  remains fresh, as the live tool schemas define; replan on `Plan_Stale` or
  `Plan_Not_Found`.

Search validates robot motion, not consequences involving objects absent from
the preview. Judge contact, grasp outcome, and completion after execution in
RoboDojo.
cuRobo sees self-collision and a static table, not task objects, the other arm or
live gripper geometry. Robot-only preview has no table or objects at all. Search
cannot turn these limited models into evidence of grasp, contact or task success.
