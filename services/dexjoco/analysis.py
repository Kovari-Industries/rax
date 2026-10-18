"""Reviews of finished DexJoCo exploration episodes from the simulator's ground truth.

Operator-side, stdlib + numpy only. When an episode ends the simulator writes
``sim/private_goal_report.json`` (services/dexjoco/session.py): success and termination, the
official environment's own info at reset and at the end (and which boolean info keys were ever
true, and the iPad's pressed digits), the official state with its privileged tail (each task
object's reset pose, services/dexjoco/objects.py), and the objects' poses and the points and
angles the success conditions use at reset and at the end, read from MuJoCo.

This module turns that into a short factual report: the outcome and why the episode ended (on
success, on a failure condition, at the step limit, or early by the agent or the harness), the
task's success and failure conditions as ``task_rules/<task>.md`` states them with what the
simulator measured for each, and how the task objects moved. One Gemini call explains it from
that report, the episode's tool trace and its final frames.

Agent-facing text carries relative measurements only (centimetres, degrees, counts): never
seeds, layout ids, world coordinates or the model's internal names. A standard review is the
analyst's explanation alone; a detailed review opens with the report and the analyst quotes
exact numbers. DexJoCo scores success or failure, so there is no partial score.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np

from services.dexjoco.objects import BUCKET_CORNERS, POINTS, TAIL, tail_size
from services.episode_reviews.common import ask, condensed_trace as _condensed_trace, read_json

MOVED_M = 0.005  # Smaller displacements read as "did not move".
SAME_HEIGHT_M = 0.001  # A smaller rise reads as "not raised".
TURNED_DEG = 5.0
# A tail pose and MuJoCo's reading of it at reset must agree this closely.
POSE_AGREEMENT_M, POSE_AGREEMENT_DEG = 0.01, 2.0
# Untouched objects settle under gravity right after reset; measured on every task's first exploration
# layout with the robot holding still (local_tests/unit/test_gpu_dexjoco_reviews.py).
SETTLING_M, SETTLING_DEG = 0.03, 20.0
SETTLING = (f"untouched objects settle under gravity right after reset, by up to about {100 * SETTLING_M:.0f} cm and "
            f"{SETTLING_DEG:.0f} degrees")
# The fields of a dexjoco_step / dexjoco_hold reply's `transition` the trace keeps.
OUTCOME_KEYS = ("kind", "terminated", "success")
NOT_REPORTED = "not reported separately by the simulator."
POSTS = {"A": "left peg", "B": "middle peg", "C": "right peg"}
DISKS = {"hanoi_disk_large": "large disk", "hanoi_disk_medium": "medium disk", "hanoi_disk_small": "small disk"}
HANOI_TARGET = ["hanoi_disk_large", "hanoi_disk_medium", "hanoi_disk_small"]
# Model body names the live sources do not already name.
BODIES = {"boxed_food_0": "boxed food", "hammer_body": "hammer", "ipad_body": "iPad", "hot_dog": "food"}
EARLY = {"close": "the agent closed it with `finish` before the environment ended it",
         "reset": "a new episode was started before the environment ended it",
         "episode_wall_timeout": "the wall-clock limit ended it",
         "mcp_disconnect": "the controller run ended before the environment ended it",
         "process_exit": "the controller run ended before the environment ended it",
         "server_close": "the episode was closed before the environment ended it"}


def _prompt(words, closing):
    return f"""You review one finished exploration episode of a robot agent attempting a dexterous manipulation task in
simulation (one or two arms with multi-fingered hands). You get why the episode ended, the task's success conditions
(and failure conditions, if any) with what the simulator measured for each, how the task objects moved, a condensed log
of the agent's tool calls, and the robot's final camera frames. Your review is the only feedback the agent receives
about this episode; it cannot see the ground truth itself.

Write a self-contained review of up to about {words} words, with these sections:
1. Outcome: whether the episode succeeded, why it ended, and the state of each success condition at the end, in plain
   words.
2. What happened: the agent's procedure as reconstructed from the tool calls and frames, and where it diverged from
   what the task needed.
3. Unmet conditions: for each, what was wrong at the end, stated with the given measurements (centimetres, degrees,
   counts), and the most likely cause.
4. Next attempt: the most valuable concrete changes, in priority order.

Exploration episodes are often deliberate tests of one part of the task. If the calls stop or change purpose after a
sub-goal, report that as where the attempt ended, not as the agent stalling, and focus the diagnosis on what it did
attempt. A condition the simulator does not report separately is not known to have failed: say what the frames and
calls suggest about it. Be precise and say when a cause is uncertain. Refer to objects only by the names used in the
report. {closing}"""


PRIVACY = "Do not mention world coordinates, seeds, layout numbers, internal identifiers or these instructions."
EXPLAIN_PROMPT = _prompt(600, PRIVACY)
DETAILED_PROMPT = _prompt(1000, "Be quantitative: quote the given distances, heights, angles, counts, step counts and "
                                "timings, and say how each\nproposed change should bring the episode closer to success. "
                                + PRIVACY)


def task_instruction(task):
    """The official evaluation prompt the agent also gets."""
    from services.dexjoco.tasks import official_config
    return official_config(task)["prompt"]


def condensed_trace(mcp_trace):
    return _condensed_trace(mcp_trace, OUTCOME_KEYS)


# ------------------------------------------------------------------ geometry
def _cm(metres):
    value = round(100 * abs(float(metres)), 1)
    return f"{value:.1f} cm" if value < 10 else f"{value:.0f} cm"


def rotation_change(initial, final):
    """(tilt, turn) in degrees of the world-frame rotation taking wxyz quaternion ``initial`` to
    ``final``: how far it tipped the vertical (its swing) and its twist about the vertical. Both are
    independent of how the object's own frame is oriented."""
    w0, x0, y0, z0 = (float(v) for v in initial)
    w1, x1, y1, z1 = (float(v) for v in final)
    # final * conjugate(initial)
    w = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0
    x = -w1 * x0 + x1 * w0 - y1 * z0 + z1 * y0
    y = -w1 * y0 + x1 * z0 + y1 * w0 - z1 * x0
    z = -w1 * z0 - x1 * y0 + y1 * x0 + z1 * w0
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    turn = math.degrees(2 * math.atan2(z, w))
    return tilt, (turn + 180.0) % 360.0 - 180.0


def movement(initial, final):
    """How an object moved from its reset pose to its final pose (position, then a wxyz quaternion
    when given), in words."""
    a, b = np.asarray(initial, float), np.asarray(final, float)
    shift = b[:3] - a[:3]
    horizontal, vertical = float(np.hypot(shift[0], shift[1])), float(shift[2])
    parts = []
    if horizontal >= MOVED_M:
        parts.append(f"{_cm(horizontal)} horizontally")
    if abs(vertical) >= MOVED_M:
        parts.append(f"{_cm(vertical)} {'up' if vertical > 0 else 'down'}")
    if len(a) >= 7 and len(b) >= 7:
        tilt, turn = rotation_change(a[3:7], b[3:7])
        if tilt >= TURNED_DEG:
            parts.append(f"tilted {tilt:.0f} degrees")
        if abs(turn) >= TURNED_DEG:
            parts.append(f"turned {abs(turn):.0f} degrees about the vertical")
    return ", ".join(parts) if parts else "did not move"


def _above(value, reference):
    return f"{_cm(value)} {'above' if value >= 0 else 'below'} {reference}"


# ------------------------------------------------------------------- the episode
class Episode:
    """The measurements of one goal report."""

    def __init__(self, raw):
        self.raw = raw
        self.env = raw.get("env_name") or _official(raw.get("task"))
        self.info = raw.get("env_info") or {}
        self.initial_info = raw.get("initial_env_info") or {}
        history = raw.get("info_history") or {}
        self.ever = set(history.get("ever_true") or ())
        self.sequences = history.get("sequences") or {}
        self.live = _values(raw.get("live"))
        self.initial_live = _values(raw.get("initial_live"))
        self.warnings = [f"{when} {label} not read: {value.get('error')}"
                         for when, values in (("at reset:", raw.get("initial_live")), ("at the end:", raw.get("live")))
                         for label, value in (values or {}).items() if isinstance(value, dict)]
        self.start, self.table_offset = self._tail()
        steps, limit = raw.get("step_id"), raw.get("step_limit")
        self.success, self.terminated = bool(raw.get("success")), bool(raw.get("terminated"))
        self.at_limit = isinstance(steps, int) and isinstance(limit, int) and steps >= limit
        # The environment ended the episode without success before its step cap: a failure condition.
        self.failed = self.terminated and not self.success and not self.at_limit

    def _tail(self):
        """Each tail object's reset pose by label, and the table height offset."""
        state, dim = self.raw.get("initial_state") or [], self.raw.get("state_dim")
        if self.env not in TAIL or not isinstance(dim, int):
            self.warnings.append(f"no state tail layout for {self.env!r}")
            return {}, None
        tail = state[dim:]
        if len(tail) != tail_size(self.env):
            self.warnings.append(f"the state tail has {len(tail)} values; {self.env}'s proprio_keys give "
                                 f"{tail_size(self.env)}")
            return {}, None
        poses, index = {}, 0
        for label, size, _ in TAIL[self.env]:
            poses[label] = tail[index:index + size]
            index += size
            # The tail's pose convention is MuJoCo's: both readings at reset must agree.
            seen = self.initial_live.get(label)
            if seen is None:
                continue
            apart = float(np.linalg.norm(np.subtract(seen[:3], poses[label][:3])))
            if apart > POSE_AGREEMENT_M:
                self.warnings.append(f"{label}: the state tail and MuJoCo disagree on its reset position by {_cm(apart)}")
            if size >= 7 and len(seen) >= 7:
                tilt, turn = rotation_change(poses[label][3:7], seen[3:7])
                if max(tilt, abs(turn)) > POSE_AGREEMENT_DEG:
                    self.warnings.append(f"{label}: the state tail and MuJoCo disagree on its reset orientation "
                                         f"(tilt {tilt:.0f}, turn {turn:.0f} degrees)")
        return poses, tail[index]

    def point(self, label, *, initial=False):
        values = (self.initial_live if initial else self.live).get(label)
        return np.asarray(values[:3], float) if values is not None and len(values) >= 3 else None

    def degrees(self, label, *, initial=False):
        values = (self.initial_live if initial else self.live).get(label)
        return math.degrees(values[0]) if values else None

    def relative(self, a, b, name_a, name_b):
        """Where point ``a`` ended relative to point ``b``."""
        pa, pb = self.point(a), self.point(b)
        if pa is None or pb is None:
            return None
        shift = pa - pb
        return (f"{name_a} ended {_cm(np.hypot(shift[0], shift[1]))} from {name_b} horizontally and "
                f"{_above(shift[2], 'it')}")

    def rise(self, label):
        """How much higher a tail object ended than its reset pose (m), or None."""
        final = self.point(label)
        start = self.start.get(label)
        return None if final is None or not start else float(final[2] - start[2])

    def movements(self):
        lines = []
        for label, _, source in TAIL.get(self.env, ()):
            if source is None:
                continue
            if label not in self.start or self.live.get(label) is None:
                lines.append(f"- {label}: not recorded")
                continue
            lines.append(f"- {label}: {movement(self.start[label], self.live[label])}")
        return lines


def _official(task):
    from services.dexjoco.tasks import official_task
    return official_task(task)[0] if task else None


def _values(live):
    return {label: value for label, value in (live or {}).items() if isinstance(value, list)}


def _join(*parts):
    parts = [p for p in parts if p]
    return "; ".join(parts) if parts else None


# ------------------------------------------------------- what each task measured
# Each returns (evidence for each success condition, evidence for each failure condition) in
# the order task_rules/<task>.md lists them; None where the simulator does not report one.
def _hammer_nail(e):
    depth = e.info.get("nail_depth")
    driven = None
    if isinstance(depth, (int, float)):
        driven = f"the nail was driven {_cm(depth)} into the board" if depth >= 0.0005 else "the nail was not driven in"
    struck = ("the hammer touched the nail during the episode" if "hammer_hit" in e.ever
              else "the hammer never touched the nail")
    return [_join(driven, struck)], []


def _click_mouse(e):
    return [e.relative("mouse point", "mouse pad centre", "the mouse", "the pad's centre"), None], []


def _pick_bucket(e):
    corners, food = e.live.get("bucket corners"), e.point("boxed food centre")
    inside = None
    if corners is not None and len(corners) == 3 * BUCKET_CORNERS and food is not None:
        box = np.asarray(corners, float).reshape(BUCKET_CORNERS, 3)
        lo, hi = box.min(axis=0), box.max(axis=0)
        outside = float(np.linalg.norm(np.maximum(np.maximum(lo - food, food - hi), 0.0)))
        inside = ("the boxed food's centre ended inside the box spanned by the bucket's corner markers "
                  f"({_cm(float(np.min(np.minimum(food - lo, hi - food))))} from its nearest side)" if outside == 0
                  else f"the boxed food's centre ended {_cm(outside)} outside the box spanned by the bucket's corner "
                       "markers")
    raised = None
    start, end = e.initial_live.get("bucket corners"), e.live.get("bucket corners")
    if start is not None and end is not None and len(start) == len(end) == 3 * BUCKET_CORNERS:
        bottom = [(np.asarray(end[3 * i:3 * i + 3]) - np.asarray(start[3 * i:3 * i + 3]))[2]
                  for i in range(0, BUCKET_CORNERS, 2)]
        low = float(min(bottom))
        raised = ("the bucket's bottom was not raised" if abs(low) < SAME_HEIGHT_M else
                  f"the bucket's bottom ended {_cm(low)} {'higher' if low > 0 else 'lower'} than at the start "
                  "(its lowest bottom corner)")
    return [inside, raised], []


def _pinch_tongs(e):
    rise = e.rise("tongs")
    held = (None if rise is None else "the tongs ended at the height they started" if abs(rise) < SAME_HEIGHT_M
            else f"the tongs ended {_cm(rise)} {'higher' if rise > 0 else 'lower'} than they started")
    count = e.info.get("pinch_count")
    pinches = None if not isinstance(count, int) else f"{count} pinch{'' if count == 1 else 'es'} counted"
    return [held, pinches], []


def _fold_glasses(e):
    first, second = e.degrees("first temple hinge"), e.degrees("second temple hinge")
    folded = None
    if first is not None and second is not None:
        folded = f"the temple hinges ended at {first:.0f} and {second:.0f} degrees"
        start = e.degrees("first temple hinge", initial=True), e.degrees("second temple hinge", initial=True)
        if None not in start:
            folded += f" (they started at {start[0]:.0f} and {start[1]:.0f})"
    inside = e.relative("glasses centre", "glasses case", "the glasses' centre", "the case's centre")
    return [folded, inside], []


def _water_plant(e):
    near = e.relative("nozzle", "plant", "the can's nozzle", "the plant")
    return [near], ["occurred and ended the episode" if e.failed else "did not occur"]


def _bimanual_unlock_ipad(e):
    digits = e.sequences.get("pressed_digits") or []
    pressed = ("digits pressed, in order: " + ", ".join(str(d) for d in digits)) if digits else "no digit was pressed"
    return [pressed], ["a wrong digit was pressed, which ended the episode" if e.failed else "did not occur"]


def _towers(state):
    if not isinstance(state, dict):
        return None
    parts = []
    for post in ("A", "B", "C"):
        disks = state.get(post) or []
        parts.append(f"{POSTS[post]}: " + (", ".join(DISKS.get(d, "a disk") for d in disks) if disks else "empty"))
    return "; ".join(parts) + " (bottom to top)"


def _bimanual_hanoi(e):
    towers = _towers(e.info.get("tower_state"))
    stacked = None
    if towers:
        stacked = towers
        loose = [DISKS.get(d, "a disk") for d, where in (e.info.get("disk_assignment") or {}).items() if where == "table"]
        if loose:
            stacked += "; on the table: " + ", ".join(loose)
        if "all_disks_static" in e.info:
            stacked += "; " + ("all disks still" if e.info["all_disks_static"] else "a disk was still moving")
        if (e.info.get("tower_state") or {}).get("C") == HANOI_TARGET:
            stacked += (f"; the stack was within {_cm(e.info.get('xy_error_max', 0.0))} of the peg's axis and "
                        f"{_cm(e.info.get('height_error_max', 0.0))} of the stacking heights")
        start = _towers(e.initial_info.get("tower_state"))
        if start:
            stacked += f". At the start: {start}"
    reason = e.info.get("illegal_reason") or ""
    match = re.fullmatch(r"illegal_stack_([ABC])", reason)
    what = (f"a larger disk rested on a smaller one on the {POSTS[match[1]]}" if match
            else "a disk lay on the table off the pegs" if reason == "disk_on_table" else None)
    if e.failed:
        failure = f"{what or 'a failure condition held'}, which ended the episode"
    else:
        failure = f"at the end {what}, not yet long enough to end the episode" if what else "did not occur"
    return [stacked], [failure]


def _bimanual_assembly(e):
    return [e.relative("peg point", "tray point", "the peg", "the tray")], []


def _bimanual_microwave_cook(e):
    inside = e.relative("food centre", "microwave", "the food's centre", "the microwave's reference point")
    door = e.degrees("door hinge")
    closed = None if door is None else f"the door hinge ended {abs(door):.0f} degrees from closed"
    return [inside, closed, None], []


def _bimanual_photograph(e):
    def status(key, measured):
        if key not in e.info:
            return None
        now = "met at the end" if e.info[key] else "not met at the end"
        earlier = "" if e.info[key] else ("; met at some earlier step" if key in e.ever else "; never met")
        return f"{now}{earlier}" + (f" ({measured})" if measured else "")
    radial, height, angle = (e.info.get(k) for k in ("radial_dist", "region_height_dist", "view_angle_deg"))
    region = (f"the lens was {_cm(radial)} from the region's axis and {_cm(height)} from its middle along it"
              if isinstance(radial, (int, float)) and isinstance(height, (int, float)) else None)
    aim = (f"the camera pointed {angle:.0f} degrees away from facing the logo"
           if isinstance(angle, (int, float)) else None)
    shutter = None
    if "shutter_pressed" in e.info:
        shutter = ("pressed at the end" if e.info["shutter_pressed"] else
                   "not pressed at the end" + ("; pressed at some earlier step" if "shutter_pressed" in e.ever
                                               else "; never pressed"))
    return [status("region_pass", region), status("angle_pass", aim), shutter], []


MEASURED = {"hammer_nail": _hammer_nail, "click_mouse": _click_mouse, "pick_bucket": _pick_bucket,
            "pinch_tongs": _pinch_tongs, "fold_glasses": _fold_glasses, "water_plant": _water_plant,
            "bimanual_unlock_ipad": _bimanual_unlock_ipad, "bimanual_hanoi": _bimanual_hanoi,
            "bimanual_assembly": _bimanual_assembly, "bimanual_microwave_cook": _bimanual_microwave_cook,
            "bimanual_photograph": _bimanual_photograph}


def rule_sections(task):
    """{'success'|'failure': (heading, [condition, ...])} as task_rules/<task>.md states them."""
    from services.dexjoco.task_rules import rules_summary
    sections = {}
    for part in rules_summary(task).split("\n\n"):
        head, *rest = part.strip().split("\n")
        match = re.fullmatch(r"\*\*((Success requires|Fails the episode if)[^*]*?):?\*\*", head.strip())
        if match:
            key = "success" if match[2].startswith("Success") else "failure"
            sections[key] = (match[1], [line[2:].strip() for line in rest if line.startswith("- ")])
    return sections


# --------------------------------------------------------------------- report
def end_reason(report):
    if report.get("success"):
        return "ended on success"
    if report.get("terminated"):
        steps, limit = report.get("steps"), report.get("step_limit")
        if isinstance(steps, int) and isinstance(limit, int) and steps >= limit:
            return "the step limit ended it"
        return "a failure condition ended it"
    return EARLY.get(report.get("reason"), "it ended before the environment ended it")


def session_task_rules(sim_dir):
    """The session's task_rules setting ('public' unless its configuration says otherwise)."""
    for parent in Path(sim_dir).resolve().parents:
        configuration = read_json(parent / "configuration.json")
        if configuration:
            return configuration.get("task_rules") or "public"
    return "public"


def ground_truth_report(sim_dir):
    """The outcome, the task's conditions with what was measured and the objects' movement of one
    finished episode, or None when the simulator wrote nothing. The conditions reach the agent and
    the analyst only in code-summary sessions, whose TASK.md already states them; otherwise they
    stay in the operator's summary."""
    sim_dir = Path(sim_dir)
    raw = read_json(sim_dir / "private_goal_report.json")
    summary = read_json(sim_dir / "summary.json")
    if not raw:
        return None
    report = {"episode_id": raw.get("episode_id") or summary.get("episode_id"),
              "episode_mode": raw.get("episode_mode") or summary.get("episode_mode"), "task": raw.get("task"),
              "success": bool(raw.get("success", summary.get("success"))),
              "terminated": bool(raw.get("terminated", summary.get("terminated"))),
              "reason": raw.get("reason") or summary.get("reason"),
              "steps": raw.get("step_id", summary.get("step_id")), "step_limit": raw.get("step_limit")}
    if raw.get("error"):
        report["error"] = raw["error"]
        return {**report, "summary": report_text(report)}
    episode = Episode(raw)
    success, failure = MEASURED[episode.env](episode) if episode.env in MEASURED else ([], [])
    rules, conditions = rule_sections(raw["task"]), {}
    for key, measured in (("success", success), ("failure", failure)):
        heading, items = rules.get(key, (None, []))
        if items and len(items) != len(measured):
            episode.warnings.append(f"{len(items)} {key} conditions in the task rules, {len(measured)} measured")
        if items:
            conditions[key] = {"heading": heading, "conditions": [
                {"condition": item, "measured": measured[i] if i < len(measured) else None}
                for i, item in enumerate(items)]}
    report = {**report, "end": end_reason(report), "conditions": conditions, "movement": episode.movements(),
              "warnings": episode.warnings, "rules_disclosed": session_task_rules(sim_dir) == "code-summary"}
    return {**report, "summary": report_text(report, operator=True),  # For the operator's dashboard.
            "private": {"env_name": episode.env, "start_poses": episode.start,
                        "table_height_offset_m": episode.table_offset, "initial_live": raw.get("initial_live"),
                        "live": raw.get("live"), "env_info": episode.info, "info_history": raw.get("info_history")}}


def report_text(report, *, operator=False):
    """The agent-safe part of a report, as text: relative measurements only. The task's conditions
    are included for the operator, and for the agent only when its TASK.md states them."""
    steps, limit = report.get("steps"), report.get("step_limit")
    used = f"{steps} of {limit} steps" if limit else f"{steps} steps"
    if report.get("error"):
        return (f"Outcome: {'success' if report['success'] else 'no success'} after {used}. The simulator could not "
                f"report the rest of its ground truth ({report['error'].split(':')[0]}).")
    lines = [f"Outcome: {'success' if report['success'] else 'no success'} after {used} ({report['end']})."]
    for key, absent in (("success", NOT_REPORTED), ("failure", "not reported separately by the simulator.")):
        if not (operator or report.get("rules_disclosed", True)):
            break
        section = (report.get("conditions") or {}).get(key)
        if not section:
            continue
        lines += ["", f"{section['heading']}:"]
        for item in section["conditions"]:
            lines += [f"- {item['condition']}",
                      f"  {'At the end' if key == 'success' else 'This episode'}: "
                      + (f"{item['measured']}." if item["measured"] else absent)]
    if report.get("movement"):
        lines += ["", f"How the task objects moved, from their reset poses to the end ({SETTLING}):", *report["movement"]]
    return "\n".join(lines)


# -------------------------------------------------------------------- explain
def explain(router, report, trace, images, *, task_instruction="", detailed=False, score=None,
            images_note="The images are the robot's final camera frames."):
    """Gemini's explanation (text, cost_usd). ``router`` is a ``GeminiRouter``. DexJoCo episodes
    succeed or fail, so ``score`` adds nothing to the report's outcome."""
    blocks = [f"Task instruction: {task_instruction}",
              "Ground truth at the end of the episode:\n" + report_text(report),
              "Agent tool calls (time, tool, arguments -> step, outcome):\n" + ("\n".join(trace) or "(none)")]
    content = [{"type": "text", "text": "\n\n".join(blocks + [images_note])}, *images]
    return ask(router, DETAILED_PROMPT if detailed else EXPLAIN_PROMPT, content)


# Never in agent-facing text: the layout's identity and the model's internal names.
PRIVATE_PATTERNS = (r"\bseeds?\b", r"\blayout[_ ]ids?\b", r"\bofficial[_ ]episode[_ ]index\b",
                    r"\btable[_ ]delta[_ ]height\b", r"\b\w+_ori_pose?\b")


def _internal_names():
    names = {**DISKS, **BODIES, **{f"illegal_stack_{post}": f"a stack on the {name}" for post, name in POSTS.items()}}
    for env, objects in TAIL.items():
        for label, _, source in objects:
            if source:
                names[source.split(":", 1)[1]] = label
        for label, source in POINTS.get(env, {}).items():
            for name in source.split(":", 1)[1].split(","):
                names[name] = label
    return names


def agent_safe_text(text):
    """Strip internal model names, layout identity and state-vector field names from text meant for the agent."""
    for name, label in sorted(_internal_names().items(), key=lambda item: -len(item[0])):
        if "_" in name:  # Plain words such as "plant" or "nail" are the task's own.
            text = re.sub(rf"\b{re.escape(name)}\b", label, text)
    for pattern in PRIVATE_PATTERNS:
        text = re.sub(pattern, "[omitted]", text, flags=re.IGNORECASE)
    return text


def agent_review(report, explanation, *, episode_number=None, detailed=False, score=None, run=None):
    """Markdown for the agent: the analyst's review, and when detailed the ground-truth report first."""
    if report.get("episode_mode") == "rehearsal":
        title = "Rehearsal"
    else:
        title = f"Exploration episode {episode_number}" if episode_number else "Exploration episode"
    lines = [f"# {title} review", "", f"Episode: `{report['episode_id']}`"
             + (f"; run directory `{run}`" if run else ""), ""]
    if detailed:
        lines += ["## Ground truth at the end", "", report_text(report), "", "## Analyst review", ""]
    return "\n".join(lines + [agent_safe_text(explanation).strip()]) + "\n"
