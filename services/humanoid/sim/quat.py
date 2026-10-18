"""wxyz quaternion helpers (numpy only)."""
import math

import numpy as np


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v - x*i - y*j - z*k, w*i + x*v + y*k - z*j,
                     w*j - x*k + y*v + z*i, w*k + x*j - y*i + z*v])


def inverse(q):
    return np.asarray(q) * [1, -1, -1, -1]


def rotate(q, p):
    return multiply(multiply(q, np.r_[0, p]), inverse(q))[1:]


def yaw(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def from_yaw(angle):
    return np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])
