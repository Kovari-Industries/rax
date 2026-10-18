"""TASK.md text for HumanoidArena tasks: objective, task facts, episode limits and scoring.

``task_rules/<task>.md`` holds the task facts from the pinned scene and task configuration
(starting scene, brief randomization and readable success conditions); the episode and scoring
sections are generated here from the task catalog.
"""
from pathlib import Path

from services.humanoid.tasks import CONTROL_HZ, native_step_limit, task_info, task_timeouts

RULES = Path(__file__).with_name("task_rules")
TITLES = {
    "opendoor": "Open the door", "boxing_bag": "Hit the boxing bag", "pickplace_box": "Pick and place the box",
    "doubledesk": "Move the hammer to the basket", "sit_sofa": "Sit on the sofa",
    "football_single": "Score a goal", "warehouse_navigation": "Navigate the warehouse",
}


def task_context(task, *, live_formal=False, evaluation=True, expert_demonstrations=0, expert_content="full"):
    """live_formal: for an agent formal copy. evaluation: False in live-copy mode.
    expert_content: what the expert episodes hold ('full' or 'images', camera only)."""
    info = task_info(task)
    rules = (RULES / f"{task}.md").read_text().strip()
    limit = native_step_limit(task)
    minutes = task_timeouts(task)["episode_seconds"] // 60
    lines = [f"# Task: {TITLES[task]}", "", f"**Objective.** {info.objective}", "", rules, "",
             "## Episode", "",
             f"- Step limit: {limit} control steps at {CONTROL_HZ} Hz ({limit / CONTROL_HZ:g} s of robot time).",
             "- The episode also ends if the robot falls. A fall cannot be undone."]
    if not live_formal:  # A formal copy reads its wall limit from `status` (FORMAL_AGENTS.md).
        lines.append(f"- Wall-clock limit per episode: about {minutes} minutes, including simulator startup.")
    lines.append("- Each episode draws a new layout from the randomization above.")
    if live_formal:
        lines.append("- This layout is new to you; there is no reset.")
    elif evaluation:
        lines.append("- Formal episodes are HumanoidArena's official test episodes, which exploration never uses.")
    if expert_demonstrations and expert_content == "images":
        lines += ["", "## Expert demonstrations", "",
                  f"`runtime/experts/` holds {expert_demonstrations} expert episodes of this task on other layouts, "
                  "recorded by teleoperation through the same SONIC controller, as front-camera observations only: "
                  "the RGB video, preview images and depth of every frame, without the commanded references, the "
                  "measured robot motion or the object states. Read "
                  "[learn-from-experts](.agents/skills/learn-from-experts/SKILL.md) before using them."]
    elif expert_demonstrations:
        lines += ["", "## Expert demonstrations", "",
                  f"`runtime/experts/` holds {expert_demonstrations} complete expert episodes of this task on other "
                  "layouts, recorded by teleoperation through the same SONIC controller: commanded references, "
                  "measured robot motion, object states and front-camera frames. Read "
                  "[learn-from-experts](.agents/skills/learn-from-experts/SKILL.md) before reusing them."]
    lines += ["", "## Success", "",
              "- An episode succeeds when the success condition above holds before the step limit and the robot "
              "has not fallen.",
              "- There is no partial credit: an episode that stops short of the condition, falls, or errors "
              "does not succeed."]
    lines.append("")
    return "\n".join(lines)
