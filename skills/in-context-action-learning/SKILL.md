---
name: in-context-action-learning
description: Infer task goals and a useful strategy from visual demonstrations (an image sequence of a human or robot completing the task, or a terminal-state image), and from videos of similar tasks and descriptions of the objects found on the internet. Use before acting.
---

# In-context action learning

Demonstrations are images only. They contain no actions, robot state, poses,
calibration or timing: use them for visual intent and strategy, never as executable
motion data. They may show a different layout, a human hand or another robot.
Satisfy every condition in `TASK.md` in the current scene.

## Retrieve

- Retrieve matching tips and learned skills first
  ([experience-memory](../experience-memory/SKILL.md)), then view the demonstration
  images `TASK.md` lists.
- For a sequence, keep the supplied order and identify the visible phases, contacts,
  object relations and the final effect. Do not infer timing or interpolate missing
  phases.
- For a terminal-state image, infer only the desired end relationships; devise the
  intermediate behavior yourself.
- Search the internet for videos of similar tasks and descriptions of the objects (size,
  weight, material, how their parts move), and sample a video's frames with OpenCV.
  Record what you used, with its URL, in memory.

## Adapt

- Transfer semantic structure: approach side, contact order, grasp region, relative
  placement and what completion looks like. A demonstrator's body does not define this
  robot's arm, pose or trajectory.
- Do not imitate every demonstrated action; choose simpler actions that reach the same
  required outcome on this robot.
- Ground all geometry again from current observations with
  [object-localization](../object-localization/SKILL.md), and build actions with the
  tools [robot-interface](../robot-interface/SKILL.md) describes. Never estimate
  metric targets or action magnitudes from demonstration pixels.
- When several interpretations remain plausible, compare them from the same measured
  start state, with the robot's dry-run tools where it has them.

## Update from experience

- Associate each visual interpretation with its observed effect; do not repeat an
  ineffective interpretation under the same conditions.
- On a new episode or attempt, revisit the failed attempt's before, action and after
  evidence and revise the plan.
- Turn the strategy into controller logic that reacquires current geometry instead of
  assuming the demonstrated layout.
- Promote only reusable, evidence-backed results through experience-memory, noting the
  demonstration images you used as provenance.
