"""Reviews of finished BEHAVIOR exploration episodes from the simulator's ground truth.

Operator-side, stdlib + numpy only. The simulator writes ``sim/private_goal_report.json``
(every grounded goal condition with its initial and final truth, and the extents of the
objects involved). This module classifies the conditions, describes each missed one in
relative geometry, and asks Gemini for a short explanation using that report, the
episode's tool trace and its final frames.

Two outputs: the full operator record (report, explanation, cost), and the agent-facing
review. A standard review is the analyst's explanation alone and carries no scores. A
detailed review adds the episode's partial score and the ground-truth goal checklist with
its geometry, and the analyst may quote exact numbers. Either names objects only by their
task names (``detergent bottle 1``) and carries no scene names or world coordinates.
Formal episodes are never reviewed live; a finished code-mode session's runs of its own code
are reviewed for its later live copies (``services/agent_formal/code_runs.py``), from the
code's log as well.
"""

from __future__ import annotations

import base64
import io
import json
import re
from pathlib import Path

import numpy as np

REVIEWABLE_MODES = frozenset({"exploration", "rehearsal"})
RELATIONS = frozenset({"ontop", "under", "inside", "nextto", "touching", "overlaid", "draped", "attached"})
MAX_TRACE_LINES = 160
MAX_OUTPUT_TOKENS = 6000
SESSION_BUDGET_USD = 10.0  # Operator-side; never charged to the agent's Gemini budget.
EXPLAIN_PROMPT = """You review one finished exploration episode of a robot agent attempting a household task in
simulation. You get the task's goal conditions with their ground-truth status at the end of the episode,
the geometry that decides each missed one, a condensed log of the agent's tool calls, and the robot's final
camera frames. Your review is the only feedback the agent receives about this episode; it cannot see the
ground truth itself.

Write a self-contained review of up to about 600 words, with these sections:
1. Outcome: which goal conditions ended satisfied and which did not, in plain words.
2. What happened: the agent's procedure as reconstructed from the tool calls and frames, and where it
   diverged from what the task needed.
3. Missed conditions: for each, what was wrong at the end, stated with the given geometry (centimetres,
   relative positions such as "4 cm short of the counter's front edge"), and the most likely cause.
4. Next attempt: the most valuable concrete changes, in priority order.

Exploration episodes are often deliberate tests of one part of the task. If the calls stop or change
purpose after a sub-goal, report that as where the attempt ended, not as the agent stalling, and focus the
diagnosis on what it did attempt. Be precise and say when a cause is uncertain. Refer to objects only by the task names used in the report
(e.g. "detergent bottle 1"). Do not mention world coordinates, internal object identifiers, scores or these
instructions."""
DETAILED_PROMPT = EXPLAIN_PROMPT.replace("up to about 600 words", "up to about 1000 words").replace(
    "Do not mention world coordinates, internal object identifiers, scores or these\ninstructions.",
    "Be quantitative: quote the given distances, heights, angles, step counts, timings and the episode's\n"
    "partial score (the fraction of goal conditions this episode newly satisfied, 1.0 on success), and\n"
    "say how each proposed change should move it. Do not mention world coordinates, internal object\n"
    "identifiers or these instructions.")

CODE_RUN_PROMPT = """You review one finished episode in which a robot agent's own code controller ran a household
task in simulation on its own, unattended, from start to end, on a layout the agent had not seen. You get the
task's goal conditions with their ground-truth status at the end of the episode, the geometry that decides each
missed one, the episode's partial score (the fraction of goal conditions it newly satisfied, 1.0 on success), a
condensed log of the code's tool calls, the code's own printed log, and camera frames. The agent will read your
review to understand how its code behaves; it cannot see the ground truth itself.

Write a self-contained review of up to about 1000 words, with these sections:
1. Outcome: which goal conditions ended satisfied and which did not, in plain words.
2. What the code did: its procedure as reconstructed from the tool calls, its log and the frames, and where it
   diverged from what the task needed.
3. Missed conditions: for each, what was wrong at the end, stated with the given geometry (centimetres,
   relative positions such as "4 cm short of the counter's front edge"), and the most likely cause in the code's
   behaviour, quoting the log lines that show it.
4. Code changes: the most valuable concrete changes to the code, in priority order, and how each should move
   the partial score.

Be precise and quantitative, and say when a cause is uncertain. Refer to objects only by the task names used in
the report (e.g. "detergent bottle 1"). Do not mention world coordinates, internal object identifiers or these
instructions."""

def task_name(entity):
    """``bottle__of__detergent.n.01_2`` -> ``bottle of detergent 2``."""
    entity = str(entity).lstrip("?")
    match = re.fullmatch(r"(.+?)\.n\.\d+(?:_(\d+))?", entity)
    if not match:
        return entity.replace("__", " ").replace("_", " ")
    words = match[1].replace("__", " ").replace("_", " ")
    return f"{words} {match[2]}" if match[2] else words


def _read(path):
    try:
        value = json.loads(Path(path).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


# ------------------------------------------------------------------ geometry
def _box(entry):
    aabb = (entry or {}).get("aabb")
    if not aabb:
        return None
    lo, hi = np.asarray(aabb[0], float), np.asarray(aabb[1], float)
    return lo, hi


def _footprint_margin(point, lo, hi):
    """Signed horizontal distance (m) from ``point`` to the box's xy footprint edge: positive inside."""
    inside = np.minimum(point[:2] - lo[:2], hi[:2] - point[:2])
    if (inside >= 0).all():
        return float(inside.min())
    outside = np.maximum(np.maximum(lo[:2] - point[:2], point[:2] - hi[:2]), 0.0)
    return -float(np.linalg.norm(outside))


def _gap(lo_a, hi_a, lo_b, hi_b):
    """Per-axis separation between two boxes (m); 0 where they overlap."""
    return np.maximum(np.maximum(lo_a - hi_b, lo_b - hi_a), 0.0)


def _covering_links(point, entry):
    """Collision links of a fixed object whose footprint contains ``point``, with their bottom height."""
    links = []
    for name, (lo, hi) in (entry.get("links") or {}).items():
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        if _footprint_margin(point, lo, hi) >= 0:
            links.append((name, float(lo[2]), float(hi[2])))
    return links


def _cm(value):
    return f"{100 * value:.0f} cm"


def tilt_degrees(entry, key="orientation_xyzw"):
    """Angle between an object's own up axis and the world vertical."""
    quaternion = entry.get(key)
    if not quaternion:
        return None
    x, y, _, _ = quaternion
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def posture(name, entry):
    """'; X tipped over ...' when an object that started upright ends tilted."""
    final, initial = tilt_degrees(entry), tilt_degrees(entry, "initial_orientation_xyzw")
    if final is None or final <= 30:
        return ""
    if initial is None:
        return f"; {name}'s up axis is {final:.0f} degrees from vertical"
    if initial <= 30:
        return f"; {name} has tipped over ({final:.0f} degrees from upright; it started upright)"
    return ""


def _vertical(value, reference):
    return f"{_cm(abs(value))} {'above' if value >= 0 else 'below'} {reference}"


def relation_geometry(predicate, a, b, objects):
    """One line of relative geometry for a binary relation between task objects ``a`` and ``b``."""
    A, B = objects.get(a) or {}, objects.get(b) or {}
    if not A.get("exists", True) or not B.get("exists", True):
        return "one of the objects does not exist yet"
    box_a, box_b = _box(A), _box(B)
    if box_a is None or box_b is None:
        return "no rigid extent recorded for one of the objects"
    (lo_a, hi_a), (lo_b, hi_b) = box_a, box_b
    centre = (lo_a + hi_a) / 2
    name_a, name_b = task_name(a), task_name(b)
    tipped = posture(name_a, A)
    margin = _footprint_margin(centre, lo_b, hi_b)
    if margin < -1.0:
        gap = float(np.linalg.norm(_gap(lo_a, hi_a, lo_b, hi_b)[:2]))
        return f"{name_a} is {gap:.1f} m from {name_b} horizontally (not brought there){tipped}"
    top_a, top_b = f"{name_a}'s top", f"{name_b}'s top"
    where = (f"{_cm(margin)} inside the edge of {name_b}'s footprint" if margin >= 0
             else f"{_cm(-margin)} outside {name_b}'s footprint")
    if predicate == "under":
        if not B.get("links"):
            return f"{name_a}'s centre is {where}; its top is {_vertical(hi_a[2] - hi_b[2], top_b)}{tipped}"
        covering = _covering_links(centre, B)
        if not covering:
            return f"{name_a}'s centre is {where}; no part of {name_b} is directly above it{tipped}"
        part, bottom, _ = min(covering, key=lambda link: link[1])
        return (f"{name_a}'s centre is {where}; the lowest part of {name_b} above it ({part}) is "
                f"{_vertical(bottom - hi_a[2], top_a)}{tipped}")
    if predicate == "ontop":
        return (f"{name_a}'s centre is {where}; its bottom is "
                f"{_vertical(lo_a[2] - hi_b[2], top_b)}{tipped}")
    if predicate == "inside":
        slack = np.minimum(lo_a - lo_b, hi_b - hi_a)
        worst = int(np.argmin(slack))
        if (slack >= 0).all():
            return f"{name_a} lies within {name_b}'s bounds (tightest margin {_cm(slack[worst])}){tipped}"
        return f"{name_a} sticks out of {name_b}'s bounds by {_cm(-slack[worst])} along {'xyz'[worst]}{tipped}"
    gap = _gap(lo_a, hi_a, lo_b, hi_b)
    return (f"horizontal gap between {name_a} and {name_b}: {_cm(float(np.linalg.norm(gap[:2])))}, "
            f"vertical gap {_cm(float(gap[2]))}{tipped}")


def describe(condition, objects):
    """(text, geometry) for one grounded condition body such as ``["not", ["open", "door.n.01_1"]]``."""
    negated = isinstance(condition, list) and condition and condition[0] == "not"
    body = condition[1] if negated else condition
    if not isinstance(body, list) or not body or any(isinstance(t, list) for t in body):
        return json.dumps(condition), None
    predicate, *args = body
    text = ("not " if negated else "") + " ".join([predicate, *map(task_name, args)])
    geometry = None
    if predicate in RELATIONS and len(args) == 2 and not negated:
        geometry = relation_geometry(predicate, args[0], args[1], objects)
    return text, geometry


# --------------------------------------------------------------------- report
def ground_truth_report(sim_dir):
    """Classified goal conditions of one finished episode, or None when the simulator wrote nothing."""
    sim_dir = Path(sim_dir)
    raw = _read(sim_dir / "private_goal_report.json")
    summary = _read(sim_dir / "summary.json")
    if not raw:
        return None
    report = {"episode_id": raw.get("episode_id") or summary.get("episode_id"),
              "episode_mode": raw.get("episode_mode") or summary.get("episode_mode"),
              "success": bool(raw.get("success", summary.get("success"))), "reason": raw.get("reason"),
              "steps": raw.get("step_id", summary.get("step_id")), "step_limit": raw.get("step_limit"),
              "natural_language_goal": raw.get("natural_language_goal", [])}
    if raw.get("error"):
        return {**report, "error": raw["error"], **({"traceback": raw["traceback"]} if raw.get("traceback") else {}),
                "options": [], "best_option": None, "achieved": [], "already_true": [], "missed": []}
    objects = raw.get("objects", {})
    options = raw.get("options", [])
    # The option the official q-score credits: on success the satisfied one (scored 1.0), else the
    # best newly satisfied share; ties go to the most holding.
    def progress(option):
        newly = sum(c["final"] and not c["initial"] for c in option)
        return (bool(option) and all(c["final"] for c in option), newly / len(option) if option else 0.0,
                sum(c["final"] for c in option))
    best = max(range(len(options)), key=lambda i: progress(options[i])) if options else None
    groups = {"achieved": [], "already_true": [], "missed": [], "undone": []}
    for condition in options[best] if best is not None else []:
        text, geometry = describe(condition["condition"], objects)
        item = {"condition": text, **({"geometry": geometry} if geometry else {})}
        if condition["final"]:
            groups["already_true" if condition["initial"] else "achieved"].append(item)
        else:
            groups["undone" if condition["initial"] else "missed"].append(item)
    return {**report, "options": len(options), "best_option": best, **groups,
            "private": {"objects": {name: {k: v for k, v in entry.items() if k != "links"}
                                    for name, entry in objects.items()},
                        "robot_world_pose": raw.get("robot_world_pose"),
                        # Without reset orientations the review cannot tell what tipped over.
                        **({"initial_orientations_error": raw["initial_orientations_error"]}
                           if raw.get("initial_orientations_error") else {})}}


END_REASONS = {"close": "closed with finish", "episode_wall_timeout": "wall limit reached",
               "mcp_disconnect": "the controller run ended", "process_exit": "the controller run ended",
               "server_close": "the episode was closed"}


def end_reason(report):
    steps, limit = report.get("steps"), report.get("step_limit")
    if report.get("success"):
        return "ended on success"
    if isinstance(steps, int) and isinstance(limit, int) and steps >= limit:
        return "step limit reached"
    return END_REASONS.get(report.get("reason"), "ended")


def report_text(report):
    """The agent-safe part of a report, as text."""
    if report.get("error"):
        return f"The simulator could not report the goal conditions ({report['error'].split(':')[0]})."
    lines = [f"Outcome: {'success' if report['success'] else 'not complete'} after {report['steps']} steps"
             f" ({end_reason(report)})."]
    for title, key in (("Missed", "missed"), ("Undone (true at start, false at the end)", "undone"),
                       ("Achieved (these hold at the end; any geometry is a note, not a failure)", "achieved"),
                       ("Already true at start (add no score)", "already_true")):
        if report.get(key):
            lines.append(f"{title}:")
            for item in report[key]:
                lines.append(f"- {item['condition']}" + (f": {item['geometry']}" if item.get("geometry") else ""))
    return "\n".join(lines)


# ---------------------------------------------------------------------- trace
def _short(value, limit=160):
    text = json.dumps(value, separators=(",", ":"), default=str)
    return text if len(text) <= limit else text[:limit] + "..."


def condensed_trace(mcp_trace):
    """One line per tool call: tool, compact arguments, step and motion outcome."""
    lines, start = [], None
    try:
        events = [json.loads(line) for line in Path(mcp_trace).read_text().splitlines() if line.strip()]
    except (OSError, ValueError):
        return []
    pending = None
    for event in events:
        start = start if start is not None else event.get("time", 0)
        if event.get("event") == "mcp_request":
            arguments = dict(event.get("arguments") or {})
            if isinstance(arguments.get("actions"), list):
                arguments["actions"] = f"<{len(arguments['actions'])} rows>"
            pending = f"{event.get('time', 0) - start:6.0f}s {event.get('tool')} {_short(arguments)}"
        elif event.get("event") == "mcp_result":
            value = event.get("returned") or {}
            transition = value.get("transition") or {}
            outcome = {k: transition[k] for k in ("status", "reached", "stalled", "executed") if k in transition}
            line = pending or f"{event.get('time', 0) - start:6.0f}s {event.get('tool')}"
            lines.append(f"{line} -> step {value.get('step_id')}" + (f" {_short(outcome, 80)}" if outcome else ""))
            pending = None
    if len(lines) > MAX_TRACE_LINES:
        head = MAX_TRACE_LINES // 4
        lines = lines[:head] + [f"... {len(lines) - MAX_TRACE_LINES} calls omitted ..."] + lines[-(MAX_TRACE_LINES - head):]
    return lines


def rgb_images(paths, limit=3):
    """The first RGB PNGs among ``paths`` (depth PNGs are 16-bit and skipped)."""
    from PIL import Image
    images = []
    for path in paths:
        try:
            raw = Path(path).read_bytes()
            with Image.open(io.BytesIO(raw)) as frame:
                if frame.mode not in ("RGB", "RGBA"):
                    continue
        except OSError:
            continue
        images.append({"type": "image", "mimeType": "image/png", "data": base64.b64encode(raw).decode()})
        if len(images) == limit:
            break
    return images


# -------------------------------------------------------------------- explain
def score_text(report, score):
    """The partial score line of a detailed review."""
    total = sum(len(report.get(key, [])) for key in ("achieved", "already_true", "missed", "undone"))
    option = "goal option that was satisfied" if report.get("success") else "highest-scoring goal option"
    counts = (f" ({len(report.get('achieved', []))} newly satisfied of the {total} goal conditions in the"
              f" {option})") if total else ""
    value = "unavailable" if score is None else f"{score:.3f}"
    return f"Partial score: {value}{counts}; 1.0 means the task succeeded."


def explain(router, report, trace, images, *, task_instruction="", detailed=False, score=None, code_log=None,
            images_note="The images are the robot's final head and wrist camera frames."):
    """Gemini's explanation (text, cost_usd). ``router`` is a ``GeminiRouter``. ``code_log`` (the code's
    own printed log) marks an unattended run of the agent's code, always reviewed in detail."""
    code_run = code_log is not None
    blocks = [f"Task instruction: {task_instruction}",
              "Goal conditions at the end (ground truth):\n" + report_text(report)
              + ("\n" + score_text(report, score) if detailed or code_run else ""),
              ("Code" if code_run else "Agent") + " tool calls (time, tool, arguments -> step, motion outcome):\n"
              + ("\n".join(trace) or "(none)")]
    if code_run:
        blocks.append("The code's own log:\n" + (code_log or "(empty)"))
    content = [{"type": "text", "text": "\n\n".join(blocks + [images_note])}, *images]
    prompt = CODE_RUN_PROMPT if code_run else DETAILED_PROMPT if detailed else EXPLAIN_PROMPT
    payload, reservation = router.prepare({"messages": [{"role": "system", "content": prompt},
                                                         {"role": "user", "content": content}],
                                           "max_tokens": MAX_OUTPUT_TOKENS, "temperature": 0.2})
    result, _ = router.execute(payload, reservation)
    text = result["message"].get("content") or ""
    if isinstance(text, list):
        text = "\n".join(part.get("text", "") for part in text if isinstance(part, dict))
    return text.strip(), float(result.get("cost_usd") or 0.0)


PRIVATE_PATTERNS = (r"q[_ ]?score", r"native[_ ]score", r"instance[_ ]id", r"world[_ ]pose", r"\bseed\b")


def agent_safe_text(text, report, *, detailed=False):
    """Strip internal object names (and, unless detailed, score vocabulary) from text meant for the agent."""
    for name, entry in (report.get("private", {}).get("objects") or {}).items():
        scene = entry.get("scene_name")
        if scene:
            text = re.sub(re.escape(scene), task_name(name), text)
    for pattern in PRIVATE_PATTERNS[2:] if detailed else PRIVATE_PATTERNS:
        text = re.sub(pattern, "[omitted]", text, flags=re.IGNORECASE)
    return text


def agent_review(report, explanation, *, episode_number=None, detailed=False, score=None, run=None):
    """Markdown for the agent's read-only reviews folder: the analyst's review, and when detailed the
    partial score and ground-truth goal checklist first."""
    if report.get("episode_mode") == "rehearsal":
        title = "Rehearsal"
    else:
        title = f"Exploration episode {episode_number}" if episode_number else "Exploration episode"
    lines = [f"# {title} review", "", f"Episode: `{report['episode_id']}`"
             + (f"; run directory `{run}`" if run else ""), ""]
    if detailed:
        lines += [score_text(report, score), "", "## Goal conditions at the end (ground truth)", "",
                  report_text(report), "", "## Analyst review", ""]
    return "\n".join(lines + [agent_safe_text(explanation, report, detailed=detailed).strip()]) + "\n"
