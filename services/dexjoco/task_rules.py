"""TASK.md text for DexJoCo tasks: the official prompt, optional reviewed rules, limits, success,
and the episode-reviews section while the operator has reviews on.

``public`` gives what the official evaluation gives a policy: the task's prompt. ``code-summary``
adds ``task_rules/<task>.md``: how the official environment's success check is defined,
summarized from the pinned source without its exact values.
Episode reviews quote these files' conditions with what the simulator measured.
"""

import re
from pathlib import Path

from services.dexjoco.tasks import (CONTROL_HZ, FORMAL_LAYOUTS, official_config, task_info, native_step_limit,
                                    task_timeouts)
from services.episode_reviews.workspace import where

RULES = Path(__file__).with_name("task_rules")
TASK_RULES = ("public", "code-summary")


def rules_summary(task):
    text = (RULES / f"{task}.md").read_text().strip()
    identical = re.match(r"\*\*Rules:\*\* identical to `([a-z_]+)`\.", text)
    if identical:
        base = (RULES / f"{identical.group(1)}.md").read_text().strip().split("\n\n")
        variant = text[identical.end():].strip()
        base = [variant if part.startswith("**Scene variation:**") else part for part in base]
        text = "\n\n".join(base)
    return text


def task_context(task, *, task_rules="public", live_formal=False, evaluation=True):
    """live_formal: for an agent formal copy. evaluation: False in live-copy mode."""
    info = task_info(task)
    config = official_config(task)
    limit = native_step_limit(task)
    minutes = task_timeouts(task)["episode_seconds"] // 60
    arms = "two Franka Panda arms, each with a 16-DoF Allegro hand" if info["dual_arm"] else \
        "one Franka Panda arm with a 16-DoF Allegro hand"
    lines = [f"# Task: {task}", "", f"**Instruction:** {config['prompt']}", ""]
    if task_rules == "code-summary":
        lines += ["## Task rules (summarized from the official task code)", "", rules_summary(task), ""]
    lines += ["## Episode", "",
              f"- Robot: {arms}, on a table.",
              f"- Cameras: {', '.join(f'`{c}`' for c in config['camera_mapping'].values())}.",
              f"- Step limit: {limit} steps of 20 ms of physics each (paced at {CONTROL_HZ} per second); the episode "
              "ends there, on success, or on a failure condition."]
    if not live_formal:  # A formal copy reads its wall limit from `status` (FORMAL_AGENTS.md).
        lines.append(f"- Wall-clock limit per episode: about {minutes} minutes, including simulator startup.")
    if info["regime"] == "rand_full":
        lines.append("- Object placement, the table height, the third-person camera viewpoint, lighting and table "
                     "texture change between episodes.")
    else:
        lines.append("- Object placement and the table height change between episodes.")
    if live_formal:
        lines.append("- This layout is new to you; there is no reset.")
    elif evaluation:
        lines.append(f"- Exploration episodes use their own layouts; formal episodes are the {FORMAL_LAYOUTS} official "
                     "DexJoCo evaluation layouts, which exploration never uses.")
    lines += ["", "## Success", "",
              "- An episode succeeds or fails. There is no partial credit: an episode that ends without the success "
              "condition, hits the step limit, triggers a failure condition or errors does not succeed.", ""]
    return "\n".join(lines)


def review_section(level, *, rehearsals=True):
    """TASK.md "Episode reviews" lines for ``level`` ('on' or 'detailed'); see services/episode_reviews.
    rehearsals: False in live-copy mode, which has no bundle tools."""
    *head, last = where(rehearsals)
    detail = ["It opens with that ground-truth report and quotes exact distances, angles and counts."
              ] if level == "detailed" else []
    return [*head, last + " From the simulator's",
            "ground truth, it says whether the episode succeeded and why it ended, what the simulator measured",
            "for each success condition, and how the task objects moved.",
            *detail,
            "Success, the end reason and the measurements are exact; stated causes are the reviewer's",
            "inference. Read the review of each episode before your next attempt."]
