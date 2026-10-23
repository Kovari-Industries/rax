"""Metric cloth geometry from calibrated rays and explicit measurements.

These helpers do not detect cloth or infer depth from RGB appearance.
"""
import math
from .cloth import _add, _sub, _scale, _norm, _unit, _vector


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _ray(value):
    if value.get("frame") != "base_link":
        raise ValueError("ray must use base_link")
    origin = _vector(value["origin"])
    direction = _unit(_vector(value["direction"]))
    stamp = value.get("capture_us")
    if (not isinstance(stamp, int) or isinstance(stamp, bool)) or stamp <= 0:
        raise ValueError("ray must have a capture timestamp")
    return origin, direction, stamp


def triangulate(
    first, second, *, max_skew_s=0.02, max_residual_m=0.005, min_angle_rad=0.1
):
    """Intersect matching pixels from two calibrated cameras in one capture.

    The residual is a consistency check, not a calibrated uncertainty estimate.
    The caller must verify correspondence and estimate grasp uncertainty.
    """
    limits = (max_skew_s, max_residual_m, min_angle_rad)
    if (
        any(not math.isfinite(v) or v <= 0 for v in limits)
        or min_angle_rad >= math.pi / 2
    ):
        raise ValueError("invalid triangulation bounds")
    a, u, ta = _ray(first)
    b, v, tb = _ray(second)
    if abs(ta - tb) / 1e6 > max_skew_s:
        raise ValueError("camera captures are not synchronized")
    cosine = _dot(u, v)
    denominator = 1 - cosine * cosine
    if denominator < math.sin(min_angle_rad) ** 2:
        raise ValueError("rays have insufficient triangulation angle")
    w = _sub(a, b)
    d, e = _dot(u, w), _dot(v, w)
    along_a = (cosine * e - d) / denominator
    along_b = (e - cosine * d) / denominator
    if min(along_a, along_b) <= 0:
        raise ValueError("intersection is behind a camera")
    p, q = _add(a, _scale(u, along_a)), _add(b, _scale(v, along_b))
    residual = _norm(_sub(p, q))
    if residual > max_residual_m:
        raise ValueError("matching rays do not intersect within tolerance")
    return {
        "position": _scale(_add(p, q), 0.5),
        "residual_m": residual,
        "capture_us": min(ta, tb),
        "geometry_source": "calibrated_triangulation",
    }


def on_measured_plane(ray, point, normal, *, verified_on_surface):
    """For a point known to lie on a measured plane, such as a flat towel corner.

    Never use this for a crumpled towel, a suspended corner, or an unknown pile top.
    """
    if verified_on_surface is not True:
        raise ValueError("the feature must be verified on the measured surface")
    origin, direction, stamp = _ray(ray)
    point, normal = _vector(point), _unit(_vector(normal))
    denominator = _dot(direction, normal)
    if abs(denominator) < 0.1:
        raise ValueError("ray is too close to parallel with the surface")
    distance = _dot(_sub(point, origin), normal) / denominator
    if distance <= 0:
        raise ValueError("surface is behind the camera")
    return {
        "position": _add(origin, _scale(direction, distance)),
        "capture_us": stamp,
        "geometry_source": "measured_surface",
    }
