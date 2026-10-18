#!/usr/bin/env python3
"""Model of the robot you control: Unitree G1 (29 body joints) with two Dex3 hands.

Everything here comes from ``robot.json`` next to this file, which was exported from the
simulated robot itself (its USD articulation) and checked against the simulator's measured
link poses. Pure Python 3.6+; NumPy and Pillow are used when present.

An observation is the JSON of an observe or motion reply (its first text block), saved to a
file; a saved frame-sequence manifest works too (its last frame is used). Quick start::

    S=.agents/skills/robot-interface/scripts
    python3 $S/g1_model.py pose --observation obs.json             # key points, level frame
    python3 $S/g1_model.py pose --observation obs.json --png /tmp/pose.png   # stick figure
    python3 $S/g1_model.py deproject 320 300 --observation obs.json --depth-png frame_depth.png
    python3 $S/g1_model.py project 0.45 -0.2 -0.1 --observation obs.json   # level frame -> pixel
    python3 $S/g1_model.py semantic40      # what every action dimension controls
    python3 $S/g1_model.py hand            # Dex3 joints, open/closed poses, fingertips

From Python::

    import sys; sys.path.insert(0, ".agents/skills/robot-interface/scripts")
    import g1_model as g1
    pose = g1.pose_from_observation(g1.load_observation("obs.json"))   # or g1.MODEL.fk(joints29, ...)
    pose.point("right_hand_center", frame="level")            # metres, wxyz
    pose.deproject(u, v, depth_m, frame="level")               # depth = distance to image plane

Frames (all metres, quaternions [w, x, y, z]):

- ``pelvis``: the robot root body (the observation's root orientation is this body's
  orientation). x forward, y left, z up.
- ``level``: same origin as ``pelvis``, but gravity-aligned: z points straight up and x is the
  pelvis forward direction projected onto the horizontal plane. Needs the root orientation;
  without it ``level`` equals ``pelvis``. Heights in ``level`` are relative to the pelvis origin.
- any link name, ``camera`` (``d435_link``: x along the optical axis, y left, z up) and
  ``camera_optical`` (x right, y down, z forward: the image/depth frame).

There is no world frame: nothing here knows where the robot or any object is in the room.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "robot.json")


# ----------------------------------------------------------------------------- small linear algebra
def _quat_matrix(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _matrix_quat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    if q[0] < 0:
        q = [-v for v in q]
    return q


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
            a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
            a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2]]


def _t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _axis_angle(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def _compose(a, b):
    """(R, p) * (R2, p2): frame b expressed in a's parent."""
    return _mm(a[0], b[0]), [x + y for x, y in zip(_mv(a[0], b[1]), a[1])]


def _inverse(a):
    rt = _t(a[0])
    return rt, [-v for v in _mv(rt, a[1])]


def _yaw(rotation):
    return math.atan2(rotation[1][0], rotation[0][0])


def _round(values, digits=4):
    return [round(v, digits) for v in values]


IDENTITY = ([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.0, 0.0, 0.0])


# ----------------------------------------------------------------------------- model
class Model:
    """Kinematic tree, named points, hand poses, camera and control layout of the robot."""

    def __init__(self, data):
        self.data = data
        self.root = data["root_link"]
        self.joints = data["joints"]                      # parents before children
        self.canonical29 = list(data["canonical29"])
        self.hand_joints = data["hand_joints"]            # {"left": [7 names], "right": [...]}
        self.frames = data["frames"]
        self.camera = data["camera"]
        self.links = data["links"]
        self._joint_index = {j["name"]: j for j in self.joints}
        self._static = {}
        for joint in self.joints:
            joint["_origin"] = (_quat_matrix(joint["origin_quaternion_wxyz"]), list(joint["origin_m"]))

    # -- joint values
    def joint_values(self, joints29=None, hand=None, extra=None):
        """Merge body joints (29 values in canonical29 order, or a dict), hand joints (dict, or 14 values
        in hand_state order: left then right) and any extra dict into {joint name: radians}."""
        values = {}
        if joints29 is not None:
            if isinstance(joints29, dict):
                values.update(joints29)
            else:
                joints29 = list(joints29)
                if len(joints29) != 29:
                    raise ValueError("joints29 needs 29 values in canonical29 order")
                values.update(zip(self.canonical29, joints29))
        if hand is not None:
            if isinstance(hand, dict):
                values.update(hand)
            else:
                hand = list(hand)
                names = self.hand_joints["left"] + self.hand_joints["right"]
                if len(hand) != len(names):
                    raise ValueError("hand needs 14 values: left then right, in hand_joints order")
                values.update(zip(names, hand))
        if extra:
            values.update(extra)
        unknown = sorted(set(values) - set(self._joint_index))
        if unknown:
            raise ValueError("unknown joints: %s" % unknown)
        return values

    def fk(self, joints29=None, hand=None, root_quaternion_wxyz=None, extra=None):
        """Forward kinematics. Missing joints are taken as 0 and listed in Pose.defaulted_joints."""
        values = self.joint_values(joints29, hand, extra)
        poses = {self.root: IDENTITY}
        defaulted = []
        for joint in self.joints:
            parent = poses[joint["parent"]]
            local = joint["_origin"]
            if joint["type"] == "revolute":
                if joint["name"] in values:
                    angle = float(values[joint["name"]])
                else:
                    angle = 0.0
                    defaulted.append(joint["name"])
                local = (_mm(local[0], _axis_angle(joint["axis"], angle)), local[1])
            poses[joint["child"]] = _compose(parent, local)
        return Pose(self, poses, values, defaulted, root_quaternion_wxyz)

    # -- hands
    def hand_pose(self, side, closed):
        """Dex3 joint targets of the binary hand command: closed=False -> open, True -> closed, or a
        fraction 0..1 for an intermediate configuration (for geometry only; the command is binary)."""
        fraction = float(closed)
        names = self.hand_joints[side]
        closed_values = self.data["hand_poses"]["closed"][side]
        return {name: fraction * value for name, value in zip(names, closed_values)}


class Pose:
    """Link poses of one configuration, expressed in the pelvis frame (or ``level`` on request)."""

    def __init__(self, model, links, values, defaulted, root_quaternion_wxyz):
        self.model, self.links, self.values, self.defaulted_joints = model, links, values, defaulted
        self.root_quaternion_wxyz = list(root_quaternion_wxyz) if root_quaternion_wxyz is not None else None
        if root_quaternion_wxyz is not None:
            world = _quat_matrix(root_quaternion_wxyz)
            level = _mm(_axis_angle([0, 0, 1], -_yaw(world)), world)   # remove heading only
            self._level = (level, [0.0, 0.0, 0.0])
        else:
            self._level = IDENTITY

    # -- frames
    def frame_pose(self, name):
        """Pose of a named frame (link, named point or level) in the pelvis frame."""
        if name == "pelvis":
            return IDENTITY
        if name == "level":
            return _inverse(self._level)
        if name in self.links:
            return self.links[name]
        if name in self.model.frames:
            frame = self.model.frames[name]
            local = (_quat_matrix(frame.get("quaternion_wxyz", [1, 0, 0, 0])), frame["position_m"])
            return _compose(self.links[frame["link"]], local)
        raise KeyError("unknown frame %r; see names()" % name)

    def names(self):
        return ["pelvis", "level"] + sorted(self.links) + sorted(self.model.frames)

    def transform(self, position, from_frame="camera_optical", to_frame="level"):
        """Re-express a point (metres) from one frame into another."""
        source = self.frame_pose(from_frame)
        point_pelvis = [x + y for x, y in zip(_mv(source[0], position), source[1])]
        return self._to(point_pelvis, to_frame)

    def _to(self, point_pelvis, frame):
        if frame == "pelvis":
            return point_pelvis
        if frame == "level":
            return _mv(self._level[0], point_pelvis)
        target = self.frame_pose(frame)
        return _mv(_t(target[0]), [a - b for a, b in zip(point_pelvis, target[1])])

    def point(self, name, frame="level"):
        """Position and orientation of a link or named frame, in ``frame`` (default level)."""
        rotation, position = self.frame_pose(name)
        if frame == "pelvis":
            out_r, out_p = rotation, position
        elif frame == "level":
            out_r, out_p = _mm(self._level[0], rotation), _mv(self._level[0], position)
        else:
            target = _inverse(self.frame_pose(frame))
            out_r, out_p = _compose(target, (rotation, position))
        return {"position_m": _round(out_p, 5), "quaternion_wxyz": _round(_matrix_quat(out_r), 5),
                "frame": frame}

    def points(self, frame="level", names=None):
        names = names or list(self.model.data.get("summary_points", sorted(self.model.frames)))
        return {name: self.point(name, frame)["position_m"] for name in names}

    # -- camera
    def _camera(self):
        return self.frame_pose("camera_optical")

    def project(self, position, frame="level"):
        """Pixel (u = column, v = row) and depth of a point, or None if it is behind the camera or outside
        the image. ``frame`` is the frame of ``position``."""
        point = self.transform(position, from_frame=frame, to_frame="camera_optical")
        if point[2] <= 1e-6:
            return None
        k = self.model.camera["intrinsics"]
        u = k["fx"] * point[0] / point[2] + k["cx"]
        v = k["fy"] * point[1] / point[2] + k["cy"]
        width, height = k["width"], k["height"]
        return {"u": round(u, 2), "v": round(v, 2), "depth_m": round(point[2], 4),
                "inside_image": 0 <= u < width and 0 <= v < height}

    def deproject(self, u, v, depth_m, frame="level"):
        """3D point seen at pixel (u, v) with metric depth (distance to the image plane), in ``frame``."""
        k = self.model.camera["intrinsics"]
        optical = [(u - k["cx"]) * depth_m / k["fx"], (v - k["cy"]) * depth_m / k["fy"], depth_m]
        return _round(self.transform(optical, from_frame="camera_optical", to_frame=frame), 5)

    def ray(self, u, v, frame="level"):
        """Origin and unit direction of the camera ray through pixel (u, v), in ``frame``."""
        k = self.model.camera["intrinsics"]
        direction = [(u - k["cx"]) / k["fx"], (v - k["cy"]) / k["fy"], 1.0]
        norm = math.sqrt(sum(d * d for d in direction))
        origin = self.transform([0, 0, 0], "camera_optical", frame)
        tip = self.transform([d / norm for d in direction], "camera_optical", frame)
        return {"origin_m": _round(origin, 5), "direction": _round([a - b for a, b in zip(tip, origin)], 6)}

    def deproject_image(self, depth, frame="level", stride=1):
        """All pixels of a depth image (2D list or NumPy array, metres) as an HxWx3 array of points in
        ``frame`` (NaN where depth is invalid). Needs NumPy."""
        import numpy as np
        depth = np.asarray(depth, dtype=np.float64)[::stride, ::stride]
        k = self.model.camera["intrinsics"]
        rows, cols = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        u, v = cols * stride, rows * stride
        optical = np.stack([(u - k["cx"]) * depth / k["fx"], (v - k["cy"]) * depth / k["fy"], depth], -1)
        rotation, position = self._camera()
        pelvis = optical @ np.asarray(rotation).T + np.asarray(position)
        if frame == "pelvis":
            return pelvis
        if frame == "level":
            return pelvis @ np.asarray(self._level[0]).T
        target = self.frame_pose(frame)
        return (pelvis - np.asarray(target[1])) @ np.asarray(target[0])

    # -- summaries and pictures
    def summary(self, frame="level"):
        """Key points, the lowest sole height and what was assumed."""
        out = {"frame": frame, "points_m": self.points(frame)}
        soles = [self.point(n, frame)["position_m"][2] for n in ("left_sole_center", "right_sole_center")
                 if n in self.model.frames]
        if soles and frame == "level":
            floor = min(soles)
            out["lowest_sole_z_m"] = round(floor, 4)
            out["heights_above_lowest_sole_m"] = {n: round(p[2] - floor, 4) for n, p in out["points_m"].items()}
        closure = {}
        for side in ("left", "right"):
            names = self.model.hand_joints[side]
            targets = self.model.data["hand_poses"]["closed"][side]
            ratios = [self.values[n] / t for n, t in zip(names, targets) if abs(t) > 0.5 and n in self.values]
            if ratios:
                closure[side] = round(sum(ratios) / len(ratios), 3)
        if closure:
            out["hand_closure_fraction"] = closure   # 0 open, 1 at the closed targets (measured fingers)
        out["defaulted_joints"] = self.defaulted_joints
        if self.root_quaternion_wxyz is None and frame == "level":
            out["note"] = "No root orientation given: level equals pelvis (not gravity-aligned)."
        return out

    def segments(self, frame="level"):
        """Line segments (parent point, child point, side) of the stick figure."""
        segs = []
        for joint in self.model.joints:
            if joint["child"] in self.model.data.get("stick_skip", []):
                continue
            a = self.point(joint["parent"], frame)["position_m"]
            b = self.point(joint["child"], frame)["position_m"]
            segs.append((a, b, _side(joint["child"])))
        for name, frame_def in self.model.frames.items():
            if frame_def.get("stick"):
                a = self.point(frame_def["link"], frame)["position_m"]
                b = self.point(name, frame)["position_m"]
                segs.append((a, b, _side(name)))
        return segs

    def stick_figure(self, path, frame="level", title=None):
        """Draw the pose in three orthographic views. PNG with Pillow, otherwise SVG. Returns the path."""
        return _draw(self, path, frame, title)


def _side(name):
    return "left" if name.startswith("left") else "right" if name.startswith("right") else "center"


# ----------------------------------------------------------------------------- semantic40 helpers
def semantic40_layout():
    """One entry per action dimension: index, name, meaning and unit."""
    return MODEL.data["control"]["semantic40"]["dimensions"]


def semantic40_frame(joints29, hands=(False, False), xy_delta_m=(0.0, 0.0), root_height_m=None,
                     root_rot6d=(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)):
    """Build one 40-value semantic40 reference frame (one 50 Hz tick).

    joints29: 29 joint angles in canonical29 order, or a dict (missing joints are an error).
    hands: (left_closed, right_closed) booleans. xy_delta_m: reference root displacement for this tick in
    the reference root's own frame. root_height_m: reference pelvis height (default: the standing value of
    robot.json). root_rot6d: reference root rotation relative to the episode's initial heading.
    """
    control = MODEL.data["control"]["semantic40"]
    if isinstance(joints29, dict):
        missing = [n for n in MODEL.canonical29 if n not in joints29]
        if missing:
            raise ValueError("missing joints: %s" % missing)
        joints29 = [joints29[n] for n in MODEL.canonical29]
    joints29 = [float(v) for v in joints29]
    if len(joints29) != 29:
        raise ValueError("29 joint angles required")
    height = control["standing_root_height_m"] if root_height_m is None else float(root_height_m)
    frame = [float(xy_delta_m[0]), float(xy_delta_m[1]), height] + [float(v) for v in root_rot6d] + joints29 \
        + [1.0 if hands[0] else 0.0, 1.0 if hands[1] else 0.0]
    if len(frame) != 40:
        raise ValueError("internal layout error")
    return frame


def describe_semantic40(frame):
    """Name every value of a 40-value frame."""
    return {entry["name"]: frame[entry["index"]] for entry in semantic40_layout()}


def rot6d_from_yaw(yaw_rad):
    """root_rot6d for an upright reference turned yaw_rad (left positive) from the initial heading."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [c, -s, s, c, 0.0, 0.0]


# ----------------------------------------------------------------------------- observations
def _proprio(observation):
    prop = observation.get("proprio") or observation.get("proprioception")
    if not prop:
        raise ValueError("no proprio in this observation; save the JSON of an observe or motion reply")
    return prop


def pose_from_observation(observation):
    """FK of an observation: measured body joints, measured Dex3 joints from hand_state and the root
    orientation for the level frame."""
    prop = _proprio(observation)
    positions = prop.get("joint_positions_rad", prop.get("joint_positions_sonic_rad"))
    joints = dict(zip(prop["joint_names_canonical29"], positions))
    hand = None
    if "hand_state" in prop:
        hand = dict(zip(prop["hand_state"]["joint_names"], prop["hand_state"]["positions_rad"]))
    return MODEL.fk(joints, hand, prop.get("root_orientation_world_wxyz"))


def load_observation(path):
    """An observation from a saved file: the JSON of an observe or motion reply (or the reply's MCP
    content list), or a frame-sequence manifest, whose last frame is used."""
    with open(path) as handle:
        value = json.load(handle)
    if isinstance(value, dict) and "content" in value:
        value = value["content"]
    if isinstance(value, list):
        value = json.loads(next(block["text"] for block in value if block.get("type") == "text"))
    if "frames" in value and "proprio" not in value:
        value = value["frames"][-1]
    _proprio(value)
    return value


def depth_from_png(path):
    """A saved depth frame (uint16 PNG in millimetres, 0 invalid) as rows of metres, NaN invalid."""
    from PIL import Image
    image = Image.open(path)
    try:
        import numpy as np
        millimetres = np.asarray(image, dtype=np.float64)
        return np.where(millimetres > 0, millimetres * 0.001, np.nan)
    except ImportError:
        width, height = image.size
        values = list(image.getdata())
        return [[values[r * width + c] * 0.001 if values[r * width + c] else float("nan") for c in range(width)]
                for r in range(height)]


# ----------------------------------------------------------------------------- drawing
COLORS = {"left": (40, 90, 220), "right": (215, 50, 50), "center": (60, 60, 60)}
VIEWS = (("side x-z (forward right)", 0, 2, 1), ("rear y-z (robot left on the left)", 1, 2, -1),
         ("top x-y (forward up, left left)", 0, 1, None))


def _draw(pose, path, frame, title):
    segs = pose.segments(frame)
    markers = {name: pose.point(name, frame)["position_m"] for name in pose.model.data.get("markers", [])
               if name in pose.model.frames or name in pose.links}
    camera = [pose.point("camera_optical", frame)["position_m"],
              pose.transform([0, 0, 0.35], "camera_optical", frame)]
    points = [p for s in segs for p in s[:2]] + list(markers.values()) + camera
    panels = []
    for label, a, b, sign in VIEWS:
        if sign is None:   # top view: x up, y left -> screen x = -y, screen y = x
            project = lambda p: (-p[1], p[0])
        elif a == 1:       # rear view: robot left (+y) drawn on the left
            project = lambda p: (-p[1], p[2])
        else:
            project = lambda p, a=a, b=b: (p[a], p[b])
        panels.append((label, project))
    size, margin, top = 360, 28, 40
    span = 0.0
    for _, project in panels:
        xs = [project(p)[0] for p in points]
        ys = [project(p)[1] for p in points]
        span = max(span, max(xs) - min(xs), max(ys) - min(ys))
    scale = (size - 2 * margin) / max(span, 0.3)
    width, height = len(panels) * size, size + top
    heading = title or ("G1 pose, %s frame, 1 grid = 10 cm" % frame)

    def to_screen(index, project, p, centre):
        x, y = project(p)
        return (index * size + size / 2 + (x - centre[0]) * scale, top + size / 2 - (y - centre[1]) * scale)

    items = []
    for index, (label, project) in enumerate(panels):
        xs = [project(p)[0] for p in points]
        ys = [project(p)[1] for p in points]
        centre = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2)
        items.append(("text", (index * size + 8, top - 18), label, (0, 0, 0)))
        grid = 0.1
        gx0 = math.floor((centre[0] - size / 2 / scale) / grid)
        gx1 = math.ceil((centre[0] + size / 2 / scale) / grid)
        for g in range(gx0, gx1 + 1):
            x = index * size + size / 2 + (g * grid - centre[0]) * scale
            if index * size <= x <= (index + 1) * size:
                items.append(("line", (x, top, x, top + size), (228, 228, 228), 1))
        gy0 = math.floor((centre[1] - size / 2 / scale) / grid)
        gy1 = math.ceil((centre[1] + size / 2 / scale) / grid)
        for g in range(gy0, gy1 + 1):
            y = top + size / 2 - (g * grid - centre[1]) * scale
            if top <= y <= top + size:
                items.append(("line", (index * size, y, (index + 1) * size, y), (228, 228, 228), 1))
        for a, b, side in segs:
            p, q = to_screen(index, project, a, centre), to_screen(index, project, b, centre)
            items.append(("line", (p[0], p[1], q[0], q[1]), COLORS[side], 3))
        c0, c1 = to_screen(index, project, camera[0], centre), to_screen(index, project, camera[1], centre)
        items.append(("line", (c0[0], c0[1], c1[0], c1[1]), (240, 150, 0), 2))
        for name, p in markers.items():
            s = to_screen(index, project, p, centre)
            items.append(("dot", s, COLORS[_side(name)], 4))
        items.append(("line", (index * size, top, index * size, top + size), (150, 150, 150), 1))
    items.append(("text", (8, 6), heading, (0, 0, 0)))
    items.append(("text", (8, height - 16), "blue = left, red = right, orange = camera axis, dots = hand/foot points",
                  (90, 90, 90)))
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        if path.lower().endswith(".png"):
            path = path[:-4] + ".svg"
        _svg(path, width, height, items)
        return path
    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for item in items:
        if item[0] == "line":
            draw.line(item[1], fill=item[2], width=item[3])
        elif item[0] == "dot":
            x, y = item[1]
            r = item[3]
            draw.ellipse((x - r, y - r, x + r, y + r), fill=item[2])
        else:
            draw.text(item[1], item[2], fill=item[3])
    if not path.lower().endswith(".png"):
        path += ".png"
    image.save(path)
    return path


def _svg(path, width, height, items):
    rows = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d">' % (width, height),
            '<rect width="100%" height="100%" fill="white"/>']
    for item in items:
        if item[0] == "line":
            x0, y0, x1, y1 = item[1]
            rows.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="rgb%s" stroke-width="%d"/>'
                        % (x0, y0, x1, y1, str(tuple(item[2])), item[3]))
        elif item[0] == "dot":
            rows.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="rgb%s"/>' % (item[1][0], item[1][1], item[3],
                                                                             str(tuple(item[2]))))
        else:
            rows.append('<text x="%.1f" y="%.1f" font-size="12" fill="rgb%s">%s</text>'
                        % (item[1][0], item[1][1] + 12, str(tuple(item[3])), item[2]))
    rows.append("</svg>")
    with open(path, "w") as handle:
        handle.write("\n".join(rows))


# ----------------------------------------------------------------------------- loading and CLI
def load(path=DATA):
    with open(path) as handle:
        return Model(json.load(handle))


MODEL = load() if os.path.exists(DATA) else None   # the shipped bundle always has robot.json


def _pose_from_args(args):
    observation = load_observation(args.observation)
    return observation, pose_from_observation(observation)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="G1/Dex3 model tools (see the module docstring).")
    sub = parser.add_subparsers(dest="command")
    p = sub.add_parser("pose", help="key points of the robot in a saved observation")
    p.add_argument("--observation", required=True, help="JSON of an observe or motion reply")
    p.add_argument("--frame", default="level")
    p.add_argument("--png", default=None, help="also draw the stick figure here")
    p.add_argument("--all", action="store_true", help="every link and named frame, with orientations")
    d = sub.add_parser("deproject", help="pixel (u v [depth]) -> 3D point; depth from --depth-png if omitted")
    d.add_argument("u", type=float)
    d.add_argument("v", type=float)
    d.add_argument("depth", type=float, nargs="?")
    d.add_argument("--observation", required=True)
    d.add_argument("--depth-png", default=None, help="the saved depth frame of the same step")
    d.add_argument("--frame", default="level")
    j = sub.add_parser("project", help="3D point -> pixel")
    j.add_argument("x", type=float)
    j.add_argument("y", type=float)
    j.add_argument("z", type=float)
    j.add_argument("--frame", default="level")
    j.add_argument("--observation", required=True)
    sub.add_parser("semantic40", help="what each semantic40 action dimension controls")
    sub.add_parser("hand", help="Dex3 joints, open/closed poses and finger geometry")
    sub.add_parser("camera", help="front camera intrinsics and mount")
    args = parser.parse_args(argv)
    if args.command == "pose":
        observation, pose = _pose_from_args(args)
        if args.all:
            out = {name: pose.point(name, args.frame) for name in pose.names() if name not in ("level",)}
        else:
            out = pose.summary(args.frame)
        out["hand_state_used"] = "hand_state" in _proprio(observation)
        if args.png:
            out["stick_figure"] = pose.stick_figure(args.png, args.frame)
        print(json.dumps(out, indent=1))
    elif args.command == "deproject":
        _, pose = _pose_from_args(args)
        depth = args.depth
        if depth is None:
            if not args.depth_png:
                raise SystemExit("give the pixel's depth or --depth-png")
            depth = float(depth_from_png(args.depth_png)[int(round(args.v))][int(round(args.u))])
            if not math.isfinite(depth):
                raise SystemExit("no valid depth at that pixel")
        print(json.dumps({"u": args.u, "v": args.v, "depth_m": depth, "frame": args.frame,
                          "point_m": pose.deproject(args.u, args.v, depth, args.frame)}, indent=1))
    elif args.command == "project":
        _, pose = _pose_from_args(args)
        print(json.dumps(pose.project([args.x, args.y, args.z], args.frame), indent=1))
    elif args.command == "semantic40":
        print(json.dumps(MODEL.data["control"]["semantic40"], indent=1))
    elif args.command == "hand":
        print(json.dumps(MODEL.data["hand"], indent=1))
    elif args.command == "camera":
        print(json.dumps(MODEL.camera, indent=1))
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
