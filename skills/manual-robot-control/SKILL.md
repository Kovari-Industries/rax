---
name: manual-robot-control
description: Solve the task by choosing robot actions directly from current observations. Use first on a new task, for recovery, and to set up scenes for testing code.
---

# Manual robot control

Learn to complete the task yourself through the robot tools before you automate it.
Every motion changes the scene and costs time or episode steps, and on a real robot it
cannot be undone: act in small, inspectable chunks and stop when unsure.

## First look

1. Read `TASK.md` and write a completion checklist in your notes: the required object
   relationships, orientation, placement and robot end state.
2. Call `status` and read `MCP_SESSION.md` for budgets and the live tool schemas.
   Inspect the robot with `<status>` and the scene with `<observe>`. If no episode is
   running or motion is not armed, do what AGENTS.md says (start an episode, or wait
   for the operator to arm); never work around it.
3. View the current images. Identify the relevant objects and a short sequence of
   manipulation stages. If `TASK.md` lists demonstrations, read
   [in-context-action-learning](../in-context-action-learning/SKILL.md) first.
4. Read [robot-interface](../robot-interface/SKILL.md) before the first motion: it
   says which tool fits which objective on this robot, its frames and its limits.

## Observe, act, check

1. Ground the next target in a fresh observation with
   [object-localization](../object-localization/SKILL.md). If the view is unclear,
   get a better one with [camera-inspection](../camera-inspection/SKILL.md). Never
   replace missing geometry with guessed coordinates.
2. Choose a small action and the tool robot-interface recommends for that objective:
   a planned move to a pose, bounded local steps near contact, joint targets for
   known postures, or the gripper.
3. Dry-run large or unfamiliar motions with `<preview>` where the robot offers it. A
   preview shows the robot's motion, not how task objects respond.
4. Act, then read the whole reply. A commanded target is not a measured state: compare
   the measured pose with the target before choosing the next step.
5. Observe the result and revise. Save useful parameters and decisive frame paths, and
   record what worked at once as [experience-memory](../experience-memory/SKILL.md)
   describes.

Batching actions reduces calls, not executed motion. Shorten chunks as contact or
occlusion makes the outcome less predictable.

## Errors, refusals and change

- A refusal (workspace, joint, speed or safety limit) is a wall, not a fault. Choose
  another approach; never search for a way around it.
- After an error or an ambiguous motion, read the full error, check `<status>` and a
  fresh observation, and continue only from a known state. Never resend a failed
  motion unchanged.
- Time passes while you think, and on a real robot people and objects move. Before
  acting after a pause, observe again or check [live-stream](../live-stream/SKILL.md)
  where it is available.
- Reach the robot only through its tools. Decide from observations and your
  instructions instead of pausing to ask for action advice; ask the operator only for
  what the tools cannot do.

## Confirm success

Partial visible progress, a successful plan or a clean tool return is not completion.
Check every checklist item against fresh images from more than one view, and use the
harness's trusted success check when AGENTS.md names one. An episode that ended
without confirmed success is not evidence of completion.

Record the successful strategy and its evidence in `memory/`, then build a controller
from it with [autonomous-control](../autonomous-control/SKILL.md).
