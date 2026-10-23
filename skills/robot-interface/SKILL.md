---
name: robot-interface
description: Placeholder for the robot's own interface skill, which every universal skill links to for tools, frames, limits and safety. Replace it with the robot's skill; never install this file.
---

# Robot interface (supplied by each robot)

The universal skills link here (`../robot-interface/SKILL.md`) for everything that
depends on the robot: its tools by level, frames and units, enforced limits, measured
motion, gripper, cameras and calibration, arming and safety rules, and error shapes. On
a RAX branch this name resolves to the robot's own skill in
`agent_skills/control/<robot>/robot-interface/`.

To write one for a new robot, start from the
[robot-interface template](../templates/robot-interface/SKILL.md); the
[skills README](../README.md#tool-roles) lists the four existing robots' tools.
