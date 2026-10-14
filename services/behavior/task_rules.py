"""TASK.md text for BEHAVIOR tasks: instruction, reviewed goal summary, BDDL goal and limits.

The challenge permits BDDL task definitions during evaluation, so the goal predicates are
given verbatim; ``task_rules/<task>.md`` holds the reviewed summary and the BDDL copied from
the pinned checkout. Partial-credit values stay in operator files; only detailed episode reviews report them.
"""
import json
from pathlib import Path

from services.behavior.tasks import native_step_limit, task_info

RULES = Path(__file__).with_name("task_rules")


def task_context(task, *, live_formal=False, evaluation=True):
    """live_formal: for an agent formal copy. evaluation: False in live-copy mode."""
    info = task_info(task)
    rules = (RULES / f"{task}.md").read_text().strip()
    limit = native_step_limit(task)
    rooms = ", ".join(room.rsplit("_", 1)[0].replace("_", " ") for room in info["rooms"])
    lines = [f"# Task: {info['title']}", ""]
    if info.get("instruction"):
        lines += [f"**Instruction:** {info['instruction']}", ""]
    lines += [rules, "",
              "## Episode",
              "",
              f"- Scene: `{info['scene_model']}`; loaded rooms: {rooms}.",
              "  The robot starts somewhere in these rooms; find the objects with the cameras.",
              f"- Step limit: {limit} native steps at 30 Hz (about {limit / 30 / 60:.0f} minutes of robot time)."]
    lines.append("- An episode has no wall-clock limit: it runs until the step limit or until it is ended.")
    lines.append("- Each episode is a different task instance: object and robot placements change.")
    if live_formal:
        lines.append("- This layout is new to you; there is no reset.")
    elif evaluation:
        lines.append("- Exploration episodes use training instances; formal episodes use held-out instances.")
    lines += ["", "## Scoring", "",
              "- An episode that satisfies every goal condition scores 1.0. Otherwise it earns partial credit:",
              "  the fraction of goal conditions that were false at the start and hold at the end, in the goal",
              "  option that scores highest. Conditions already true at the start add nothing.",
              "- Aim for full success, and make every episode earn as much partial credit as it can: secure",
              "  the conditions you can achieve reliably first and leave them holding, so a later failure",
              "  or the time limit keeps what was earned. An episode that errors scores zero."]
    lines.append("")
    return "\n".join(lines)


REVIEWS_HEADING = "## Episode reviews"


def review_level(session_root):
    """'detailed', 'on' or None, from the operator's review setting for a session."""
    try:
        value = json.loads((Path(session_root) / "operator-analysis" / "settings.json").read_text())
    except (OSError, ValueError):
        return None
    if value.get("enabled") is not True:
        return None
    return "detailed" if value.get("detailed") is True else "on"


def with_reviews(text, level, *, rehearsals=True):
    """TASK.md text with its episode-reviews section replaced for ``level`` (None removes it).
    rehearsals: False in live-copy mode, which has no bundle tools."""
    start = text.find("\n" + REVIEWS_HEADING + "\n")
    if start >= 0:
        end = text.find("\n## ", start + 1)
        text = text[:start] + ("\n" + text[end + 1:] if end >= 0 else "\n")
        text = text.rstrip("\n") + "\n"
    if not level:
        return text
    detail = ["It opens with the episode's partial score and the ground-truth goal checklist, and quotes",
              "exact distances."] if level == "detailed" else []
    return text.rstrip("\n") + "\n\n" + "\n".join([
        REVIEWS_HEADING, "",
        *(["About a minute after an exploration or rehearsal episode closes, a review of it appears as",
           "`review.md` in that episode's own run directory: the `artifacts.directory` its `finish` or",
           "`rehearse` reply named, under `runtime/autonomous_controller/runs/`. It says which goal conditions"]
          if rehearsals else
          ["About a minute after an exploration episode closes, a review of it appears as `review.md` in",
           "that episode's own run directory: the `artifacts.directory` its `finish` reply named, under",
           "`runtime/autonomous_controller/runs/`. It says which goal conditions"]),
        "ended satisfied and what was wrong with each missed one, from the simulator's ground truth.",
        *detail,
        "Goal status and geometry are exact; stated causes are the reviewer's inference. Read the review",
        "of each episode before your next attempt.", ""])


REVIEW_POINTER = ("While episode reviews are on, read each episode's `review.md` (`TASK.md`, Episode reviews)\n"
                  "before your next attempt: it reports the ground-truth end state your own checks cannot see.\n")


def with_review_pointer(text, level):
    """AGENTS.md text with the review reminder before its Workflow section, only while reviews are on."""
    text = text.replace(REVIEW_POINTER + "\n", "")
    anchor = "## Workflow\n"
    if level and anchor in text:
        text = text.replace(anchor, REVIEW_POINTER + "\n" + anchor, 1)
    return text
