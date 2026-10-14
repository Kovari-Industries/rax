"""Public task criteria, separate from private runtime evaluation state."""
from functools import lru_cache
import json
import re
from pathlib import Path


@lru_cache(maxsize=1)
def _snapshot():
    return json.loads(Path(__file__).with_suffix(".json").read_text())


# Operator choice for TASK.md: the public wiki rubric alone, or with rules summarized
# from the official task code (task_rules/<task>.md, reviewed text; no code is exported).
# The summaries say how each rule is defined, never its exact values (thresholds, sizes,
# positions, limits), so agents learn the idea of the rubric rather than tune to numbers.
TASK_RULES = ("public", "code-summary")
RULES_DIRECTORY = Path(__file__).with_name("task_rules")
EVALUATION_MECHANICS = """How the official evaluator decides, for every task:

- Success conditions are ordered stages. Each stage counts once, on the first step
  all its conditions hold at the same time; later changes do not undo it, and a
  stage is only tested after the previous one is complete.
- A "fails if at any step" condition ends the episode as a failure the moment it
  holds, even briefly. A "counted exactly" event must occur exactly that many times.
- The episode ends as a success on the first step every stage is complete (and every
  count is right); otherwise it runs to the step limit.
- A failed episode may still earn partial credit; the top tier requires full success.
"""


def _rules_file(task: str) -> Path:
    matches = [p for p in RULES_DIRECTORY.glob("*.md") if p.stem.lower() == task.lower()]
    if len(matches) != 1:
        raise ValueError(f"No reviewed task rules summary for {task!r} in {RULES_DIRECTORY}")
    return matches[0]


def task_rules_summary(task: str) -> str:
    """Reviewed rules for ``task``, summarized from the pinned official task code."""
    text = _rules_file(task).read_text().strip()
    identical = re.match(r"\*\*Rules:\*\* identical to `?([A-Za-z_]+)`?\.", text)
    if identical:
        # A *_random variant with the base task's rules: base rules, then the variant's scene notes.
        base = _rules_file(identical.group(1)).read_text().strip().split("\n\n")
        variant = text[identical.end():].strip()
        # The variant's scene text takes the place of the base task's.
        if variant and any(part.startswith("**Scene variation:**") for part in base):
            base = [variant if part.startswith("**Scene variation:**") else part for part in base]
        elif variant:
            base.append(variant)
        text = "\n\n".join(base)
    return text + "\n"


def task_rules_context(task: str, *, evaluation: bool = True) -> str:
    mechanics = EVALUATION_MECHANICS if evaluation else (
        EVALUATION_MECHANICS.replace("How the official evaluator decides", "How success is decided")
        .replace("A failed episode may still earn partial credit; the top tier requires full success.",
                 "A failed episode may still reach a lower row; the top row requires full success."))
    return "\n".join(["## Task rules (summarized from the official task code)", "",
                      mechanics, f"Task: `{task}`", "", task_rules_summary(task)])


def task_rubric_context(task: str, *, include_source_url: bool = True, task_rules: str = "public",
                        live_formal: bool = False, demonstrations: bool = False,
                        demonstration_images: int = 0, evaluation: bool = True) -> str:
    """live_formal: for an agent formal copy, whose AGENTS.md says how to check the conditions.
    demonstrations: visual demonstrations are supplied, so clauses about them apply;
    demonstration_images: how many (one reads "image", not "images").
    evaluation: False in live-copy mode, whose agents read the rubric as task conditions
    without scores (an agent formal copy implies it)."""
    evaluation = evaluation and not live_formal
    if task_rules not in TASK_RULES:
        raise ValueError(f"task_rules must be one of {TASK_RULES}")
    canonical = task.lower()
    if canonical.endswith("_random"):
        canonical = canonical.removesuffix("_random")
    snapshot = _snapshot()
    rubric = snapshot["tasks"].get(canonical)
    if rubric is None:
        raise ValueError(f"No reviewed public scoring rubric for task {task!r}; refresh the wiki snapshot before launching")
    source = (f" (snapshot {snapshot['retrieved_at_utc'][:10]})"
              + (f": {rubric['source_url']}" if include_source_url else ""))
    if evaluation:
        lines = ["## Public task scoring rubric", "", f"Task: `{task}`", "Public scoring rubric" + source, "",
                 "These are public scoring criteria, not live feedback. Use them to",
                 "understand the goal and judge visible evidence. You will not see scores",
                 "or hidden simulator state.", ""]
    else:
        lines = ["## Task conditions", "", f"Task: `{task}`", "Public task rubric" + source, "",
                 "Use these conditions to understand the goal and judge visible evidence;",
                 "hidden simulator state is not reported.", ""]
    if canonical != task.lower():
        lines += ["The randomized variant uses the base task's published rubric;",
                  "the wiki does not provide a separate "
                  + ("scoring table" if evaluation else "table") + " for this variant.", ""]
    images = "the demonstration image" if demonstration_images == 1 else "the demonstration images"
    if canonical == "make_kong":
        # Reviewed visual-demo clarification; preserve the published table below.
        lines += ['Clarification: The "target tile" in the rubric is moved from the left pile to stand upright at the right end of the front row, next to its rightmost tile, with its symbol oriented the same way.'
                  + (f" Use {images} to identify it." if demonstrations else ""), ""]
    elif canonical == "make_toast":
        lines += [
            "Clarification: "
            + (f"Use {images} to guide insertion depth and orientation. " if demonstrations else "")
            + "Seat one bread slice in each toaster slot "
            "rather than on the rim, keep all four slices upright with exactly two "
            "slices on the shelf, and press the lever fully down and ensure it stays "
            "down after withdrawing the gripper and returning both arms home.", "",
        ]
    elif canonical == "pour_by_language":
        lines += [
            "Clarification: Complete the language-specified pours in order. "
            "Tilt each bottle far enough and hold it tilted long enough to drain it "
            "nearly completely into its assigned bowl before returning it upright. "
            "Aim to nearly fill the bowl without spilling, but also check that the "
            "bottle is nearly empty; some visible liquid in the bowl is not enough. "
            "After the liquid has successfully transferred into its assigned bowl, "
            "return that bottle upright, then return both robot arms to their initial "
            "poses before proceeding to the next pour. Repeat these checkpoints after "
            "each pour, including the final one; matching only the final filled-bowl "
            "image is not sufficient for full credit.", "",
            "Checkpoint detail: Upright and return-home checks depend on transitions. "
            "Both arms must leave their home regions between return-home checkpoints; "
            "they must be outside those regions at the same time before returning. "
            "While the working arm is away, raise the idle arm more than 15 cm above "
            "its initial end-effector height, with margin and safe clearance, to "
            "move it outside the home-position tolerance. A small lift or keeping "
            "one arm parked at home can prevent a new checkpoint.", "",
        ]
    elif canonical == "swap_blocks":
        lines += [
            "Clarification: Either block may move first. Move it onto the empty "
            "mat, move the other block onto the newly vacated mat, then move the first "
            "block onto the other block's original mat. Lift the moved block more than "
            "3 cm above its resting position on each move; sliding alone does not "
            "satisfy the lift checkpoints. Placement requires block and mat root "
            "positions to be less than 3 cm apart in 3D, not just aligned from above.", "",
            "Button detail: Press and release once after each placement, for exactly "
            "three presses total. A counted press moves from above 95% to below 50% "
            "of the button's joint travel. Although a release checkpoint accepts "
            "above 90%, the next press cannot be counted until the button rebounds "
            "above 95%. Clear the button and let it rebound fully between presses; "
            "a fourth counted press fails the task.", "",
            "Finish detail: After the third press, release the button and return "
            "both arms to their initial end-effector poses, within 15 cm on each "
            "position axis and 20 degrees in orientation. This is a final-state "
            "check, not a return-home transition; an unused arm may remain at home.", "",
        ]
    lines += ["| Published score | Condition |" if evaluation else "| Progress (of 100) | Condition |", "|---|---|"]
    for row in rubric["criteria"]:
        lines.append(f"| {row['points']} | {row['condition'].replace('|', '&#124;')} |")
    compare = (f", comparing {images} with a current image" if demonstrations else "")
    if not live_formal:
        lines += ["", "If `evaluate` reports failure and the cause is unclear, verify each condition and "
                  f"clarification point by point against your final state{compare}."]
    context = "\n".join(lines) + "\n"
    if task_rules == "code-summary":
        context += "\n" + task_rules_context(task, evaluation=evaluation)
    return context
