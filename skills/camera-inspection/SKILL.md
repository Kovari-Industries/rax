---
name: camera-inspection
description: Get a closer or different view of small, occluded or out-of-view task details by moving a camera the robot carries (a wrist camera on an empty arm, a pan-tilt head, the torso or the base). Use when the overview image cannot settle a task-relevant question, and before motions that sweep space you have not seen.
---

# Camera inspection

A camera the robot carries is a movable viewpoint. Moving it closer or around reveals
detail an overview image cannot; this is physical repositioning, not digital zoom.
[Robot interface](../robot-interface/SKILL.md) lists this robot's cameras, what moves
each one, and the tools to aim them (`<aim_camera>` where the robot has one).

## Close-up with a wrist camera

- Use an arm that holds nothing. Never release a held object to get a view, and keep
  the other arm's grasp and the task state intact.
- Choose a clear approach from the overview, move with the robot's planned or small
  observation-guided motions, and keep the arm and gripper clear of the support
  surface, task objects and the robot's own body. A successful plan does not prove
  clearance from objects the planner cannot see.
- Inspect the fresh wrist image after each move and adjust distance and angle until
  the detail is clear and unoccluded by the gripper. Stop approaching once the view is
  useful, or when clearance is uncertain.

## Looking around

When the target, a placement spot, an obstacle or a person may be beside or behind the
robot, move the viewpoint instead of guessing. Before an in-place turn, a reverse or
any motion that sweeps space you cannot see, look at that space yourself; ask the
operator only when the views cannot settle it. Keep the arms close to the body while
looking, observe at each view, and note the joint state and base pose of each view so
you can relate it to the robot's frame. Turn back to the working view before
manipulating.

## After moving a camera

Run [object-localization](../object-localization/SKILL.md) on the new observation.
Never reuse pixels, rays or camera poses from before the camera moved. Move only when
the new view resolves a task-relevant uncertainty (it costs time or episode steps), and
return the arm or body to a clear pose if it obstructs the next manipulation.
