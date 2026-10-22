"""Optional geometric baseline. No robot, transport, camera, or model dependencies.

Rectangular, horizontal half-folds use the gravity-fold relation z=d-|b|.
This is a quasi-static, zero-thickness model, not a universal cloth constraint.
Learned policies can use the public motion primitives without using this module.
"""
from dataclasses import dataclass
import math


def point(value):
    value = tuple(value)
    if len(value) != 3 or any(
        type(x) not in (int, float) or not math.isfinite(x) for x in value
    ):
        raise ValueError("expected a finite metric point")
    return value


def rectangle(corners):
    corners = tuple(point(p) for p in corners)
    if len(corners) != 4:
        raise ValueError("four boundary-ordered corners are required")
    edges = [
        tuple(b - a for a, b in zip(corners[i], corners[(i + 1) % 4])) for i in range(4)
    ]
    lengths = [math.dist(e, (0, 0, 0)) for e in edges]
    if (
        min(lengths) < 0.01
        or max(p[2] for p in corners) - min(p[2] for p in corners) > 0.01
    ):
        raise ValueError("footprint must be a horizontal rectangle")
    for i in range(4):
        if (
            abs(sum(a * b for a, b in zip(edges[i], edges[(i + 1) % 4])))
            > 0.1 * lengths[i] * lengths[(i + 1) % 4]
        ):
            raise ValueError("corners must follow a rectangular boundary")
    if any(
        math.dist(edges[i], tuple(-x for x in edges[i + 2])) > 0.01 for i in range(2)
    ):
        raise ValueError("opposite edges do not match")
    return corners


def area(corners):
    return (
        abs(
            sum(
                a[0] * b[1] - b[0] * a[1]
                for a, b in zip(corners, corners[1:] + corners[:1])
            )
        )
        / 2
    )


def footprint_matches(expected, observed, tolerance):
    """Independent of corner start index/winding; labels cannot override geometry."""
    expected, observed = rectangle(expected), rectangle(observed)
    error = min(
        max(math.dist(a[:2], b[:2]) for a, b in zip(expected, order[i:] + order[:i]))
        for order in (observed, observed[::-1])
        for i in range(4)
    )
    return error <= tolerance and abs(area(observed) / area(expected) - 1) <= 0.2


@dataclass(frozen=True)
class FoldPlan:
    expected_corners: tuple
    lift_contacts: dict
    contacts: tuple
    line: tuple

    def target(self):
        return {
            "kind": "rectangular_half_fold",
            "corners": self.expected_corners,
            "line": self.line,
        }


def plan_half_fold(corners, line, contacts, *, test_lift_m, release_gap_m):
    """Plan material contact paths; palm/tool offsets belong to the caller."""
    corners = rectangle(corners)
    a, b = (point(p) for p in line)
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if (
        length < 1e-6
        or abs(a[2] - b[2]) > 0.01
        or any(abs(p[2] - a[2]) > 0.01 for p in corners)
    ):
        raise ValueError("fold line must be horizontal and nonzero")
    n = (-(b[1] - a[1]) / length, (b[0] - a[0]) / length)

    def signed(p):
        return sum((p[i] - a[i]) * n[i] for i in range(2))

    distances = [signed(p) for p in corners]
    half = max(abs(d) for d in distances)
    if (
        half < 0.005
        or any(abs(abs(d) - half) > min(0.005, half * 0.1) for d in distances)
        or sum(d > 0 for d in distances) != 2
    ):
        raise ValueError("baseline supports edge-parallel half-folds only")
    if len(contacts) != 2:
        raise ValueError("a rectangular fold needs two edge contacts")
    contacts = {name: point(p) for name, p in contacts.items()}
    signs = [signed(p) for p in contacts.values()]
    if any(abs(d) < half * 0.8 for d in signs) or signs[0] * signs[1] <= 0:
        raise ValueError("contacts must be on the same moving edge")
    side = 1 if signs[0] > 0 else -1
    moving = [p for p in corners if signed(p) * side > 0]
    if any(
        min(math.dist(p, q) for q in contacts.values()) > min(0.04, half * 0.2)
        for p in moving
    ):
        raise ValueError("contacts must capture both moving corners")
    for value in (test_lift_m, release_gap_m):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("lift and release gap must be finite and positive")
    if release_gap_m >= min(abs(d) for d in signs) / 4:
        raise ValueError("release gap is too large for the available material")

    def at(p, fraction):
        d = signed(p)
        return (
            p[0] - 2 * d * fraction * n[0],
            p[1] - 2 * d * fraction * n[1],
            p[2] + 2 * abs(d) * min(fraction, 1 - fraction),
        )

    lift = {
        k: at(p, min(test_lift_m, abs(signed(p)) * 0.25) / (2 * abs(signed(p))))
        for k, p in contacts.items()
    }
    steps = tuple(
        {
            k: at(p, f if f is not None else 1 - release_gap_m / (2 * abs(signed(p))))
            for k, p in contacts.items()
        }
        for f in (0.25, 0.5, 0.75, None)
    )
    # Clip the original polygon to the stationary half. Preserve boundary order.
    clipped = []
    for p, q in zip(corners, corners[1:] + corners[:1]):
        dp, dq = signed(p) * side, signed(q) * side
        if dp <= 0:
            clipped.append(p)
        if (dp > 0) != (dq > 0):
            t = dp / (dp - dq)
            clipped.append(tuple(x + t * (y - x) for x, y in zip(p, q)))
    return FoldPlan(rectangle(clipped), lift, steps, (a, b))


def placement_yaw(corners, pile_size, pile_yaw, margin):
    """Choose a fitting rectangle-equivalent orientation with the smallest turn."""
    corners = rectangle(corners)
    if len(pile_size) != 2 or any(
        type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in pile_size
    ):
        raise ValueError("invalid pile size")
    if not math.isfinite(pile_yaw) or not math.isfinite(margin) or margin < 0:
        raise ValueError("invalid placement bounds")
    edge = tuple(b - a for a, b in zip(corners[0], corners[1]))
    source = math.atan2(edge[1], edge[0])
    center = tuple(sum(p[i] for p in corners) / 4 for i in range(2))
    choices = []
    for k in range(4):
        angle = math.atan2(
            math.sin(pile_yaw - source + k * math.pi / 2),
            math.cos(pile_yaw - source + k * math.pi / 2),
        )
        c, s = math.cos(angle - pile_yaw), math.sin(angle - pile_yaw)
        points = [
            (
                c * (p[0] - center[0]) - s * (p[1] - center[1]),
                s * (p[0] - center[0]) + c * (p[1] - center[1]),
            )
            for p in corners
        ]
        if all(
            max(abs(p[i]) for p in points) + margin <= pile_size[i] / 2 + 1e-9
            for i in range(2)
        ):
            choices.append(angle)
    if not choices:
        raise ValueError("packet does not fit pile with margin")
    return min(choices, key=lambda a: (abs(a), a))
