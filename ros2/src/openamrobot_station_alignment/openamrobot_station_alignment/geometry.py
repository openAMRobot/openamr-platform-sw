# SPDX-License-Identifier: MIT
"""Planar geometry of station-relative alignment. No ROS import, so the tests run anywhere.

A pose is (x, y, yaw) in metres and radians. A station layout describes a marker plate on a shelf front:
tags at known offsets along the shelf axis u, the outward face normal n = u rotated by +90 deg, and the station
(the base_footprint pose where the robot must stop) at `standoff` along n and `lateral` along u from the plate
origin, facing the shelf (heading -n).
"""
import math


def wrap(angle):
    """angle in (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def compose(a, b):
    """Pose b expressed in frame a, returned in the parent frame of a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def between(a, b):
    """Pose b in frame a (both given in the same parent frame)."""
    return compose(inverse(a), b)


def yaw_of_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_of_yaw(yaw):
    """(x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotation_matrix(x, y, z, w):
    """3 x 3 rotation matrix (rows) of a unit quaternion."""
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _unit(vx, vy):
    norm = math.hypot(vx, vy)
    if norm < 1e-9:
        raise ValueError("degenerate direction")
    return vx / norm, vy / norm


class Layout:
    """Marker plate layout of one station (config/station_alignment.yaml)."""

    def __init__(self, tag_offsets, standoff, lateral=0.0):
        if not tag_offsets:
            raise ValueError("a layout needs at least one tag")
        self.tag_offsets = {int(k): float(v) for k, v in tag_offsets.items()}
        self.standoff = float(standoff)
        self.lateral = float(lateral)

    def station_from_frame(self, origin_xy, u):
        """Station pose from the plate origin and the shelf axis u, all in the observer's frame."""
        nx, ny = -u[1], u[0]
        x = origin_xy[0] + nx * self.standoff + u[0] * self.lateral
        y = origin_xy[1] + ny * self.standoff + u[1] * self.lateral
        return (x, y, math.atan2(-ny, -nx))


def station_from_tags(layout, tags):
    """Station pose in the frame the tags are given in (normally base_footprint).

    tags: {id: (position (x, y, z), rotation matrix or None)}. With two or more known tags the shelf axis is the
    least-squares direction through the tag centres (the wide baseline): the single-tag face normal is not used.
    With one tag its face normal (rotation column z, either sign) is used, oriented towards the observer.
    Returns (pose, mode) with mode "baseline" or "single".
    """
    known = {i: t for i, t in tags.items() if i in layout.tag_offsets}
    if not known:
        raise ValueError("no tag of this layout")
    if len(known) >= 2:
        ids = sorted(known, key=lambda i: layout.tag_offsets[i])
        offsets = [layout.tag_offsets[i] for i in ids]
        points = [known[i][0] for i in ids]
        mo = sum(offsets) / len(offsets)
        mx = sum(p[0] for p in points) / len(points)
        my = sum(p[1] for p in points) / len(points)
        sxx = sum((o - mo) ** 2 for o in offsets)
        if sxx < 1e-9:
            raise ValueError("tags of the layout share one offset")
        ux = sum((o - mo) * (p[0] - mx) for o, p in zip(offsets, points)) / sxx
        uy = sum((o - mo) * (p[1] - my) for o, p in zip(offsets, points)) / sxx
        u = _unit(ux, uy)
        origin = (mx - u[0] * mo, my - u[1] * mo)
        return layout.station_from_frame(origin, u), "baseline"
    tag_id, (position, rotation) = next(iter(known.items()))
    if rotation is None:
        raise ValueError("one tag needs its orientation")
    nx, ny = _unit(rotation[0][2], rotation[1][2])
    if nx * -position[0] + ny * -position[1] < 0.0:
        nx, ny = -nx, -ny
    u = (ny, -nx)
    offset = layout.tag_offsets[tag_id]
    origin = (position[0] - u[0] * offset, position[1] - u[1] * offset)
    return layout.station_from_frame(origin, u), "single"


def mean_pose(poses):
    n = len(poses)
    if n == 0:
        raise ValueError("no pose")
    s = sum(math.sin(p[2]) for p in poses)
    c = sum(math.cos(p[2]) for p in poses)
    return (sum(p[0] for p in poses) / n, sum(p[1] for p in poses) / n, math.atan2(s, c))


def covariance(poses, floor_xy=0.003, floor_yaw=0.005):
    """6 x 6 row-major ROS pose covariance (x, y, z, roll, pitch, yaw) of planar samples.

    The sample covariance of x, y and yaw (n - 1 denominator), plus floor_xy^2 and floor_yaw^2 on the diagonal so
    that a few identical simulated frames never claim zero uncertainty. z, roll and pitch are not estimated: 1e6.
    """
    m = mean_pose(poses)
    d = [(p[0] - m[0], p[1] - m[1], wrap(p[2] - m[2])) for p in poses]
    n = len(poses)
    c3 = [[0.0] * 3 for _ in range(3)]
    if n > 1:
        for i in range(3):
            for j in range(3):
                c3[i][j] = sum(v[i] * v[j] for v in d) / (n - 1)
    c3[0][0] += floor_xy ** 2
    c3[1][1] += floor_xy ** 2
    c3[2][2] += floor_yaw ** 2
    cov = [0.0] * 36
    index = (0, 1, 5)
    for i in range(3):
        for j in range(3):
            cov[index[i] * 6 + index[j]] = c3[i][j]
    for k in (2, 3, 4):
        cov[k * 6 + k] = 1e6
    return cov


def corrected_goal(map_from_base, base_from_station):
    """The station in the map frame: the robot's current map pose composed with the measured station."""
    return compose(map_from_base, base_from_station)


def within(residual, tolerance_xy, tolerance_yaw):
    return math.hypot(residual[0], residual[1]) <= tolerance_xy and abs(wrap(residual[2])) <= tolerance_yaw


def pose_error(actual, target):
    """(position error, yaw error, along the target heading, to the left of it) of actual against target."""
    e = between(target, actual)
    return math.hypot(e[0], e[1]), abs(e[2]), e[0], e[1]
