# Camera geometry in code

For a frame with intrinsics `K` (3x3) and camera pose `T_base_cam` (4x4, camera to the
robot frame), both taken at capture time. The robot's object-localization facts say
where a frame stores them and in which units its depth is saved.

## Depth back-projection

```python
import numpy as np

def back_project(u, v, depth_m, K, T_base_cam):
    p_cam = depth_m * np.linalg.solve(K, [u, v, 1.0])
    return (T_base_cam @ np.append(p_cam, 1.0))[:3]
```

Use the median of valid depth in a small window around the pixel, and skip pixels whose
window is mostly invalid.

## A pixel ray

```python
def ray(u, v, K, T_base_cam):
    d = T_base_cam[:3, :3] @ np.linalg.solve(K, [u, v, 1.0])
    return T_base_cam[:3, 3], d / np.linalg.norm(d)
```

## Ray and a horizontal plane

```python
def on_plane(origin, direction, height):
    if direction[2] >= 0:
        return None                     # the ray does not go down to the plane
    s = (height - origin[2]) / direction[2]
    return origin + s * direction
```

## Two-view triangulation

```python
def triangulate(o1, d1, o2, d2):
    w = o1 - o2
    a, b, c = d1 @ d1, d1 @ d2, d2 @ d2
    e, f = d1 @ w, d2 @ w
    den = a * c - b * b                 # ~0 when the rays are parallel
    s, t = (b * f - c * e) / den, (a * f - b * e) / den
    p1, p2 = o1 + s * d1, o2 + t * d2
    angle = np.degrees(np.arccos(np.clip(d1 @ d2, -1, 1)))
    return (p1 + p2) / 2, np.linalg.norm(p1 - p2), angle
```

Accept the midpoint only when the ray angle and the gap between the rays meet the
robot's measured thresholds; otherwise move the camera further or pick a sharper
feature.
