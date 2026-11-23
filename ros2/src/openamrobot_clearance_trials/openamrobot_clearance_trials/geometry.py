# SPDX-License-Identifier: MIT
"""Plane geometry of the clearance trials: convex polygons, circles, distances. No ROS."""
import math

# Chassis collision box of openamrobot_description robo_urdf.urdf.xacro (base_link collision, 0.780 x 0.580 m,
# centred on base_link). This is what Gazebo collides. test_geometry.py checks it against the xacro.
CHASSIS = ((0.390, 0.290), (-0.390, 0.290), (-0.390, -0.290), (0.390, -0.290))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_of_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def place(polygon, pose):
    """The polygon (points in the body frame) at pose (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0] + c * x - s * y, pose[1] + s * x + c * y) for x, y in polygon]


def box(size_x, size_y, pose):
    hx, hy = size_x / 2.0, size_y / 2.0
    return place(((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)), pose)


def _point_segment(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _contains(polygon, p):
    sign = 0
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        cross = (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax)
        if cross != 0.0:
            s = 1 if cross > 0 else -1
            if sign and s != sign:
                return False
            sign = s
    return True


def _overlap(a, b):
    """Separating-axis test of two convex polygons."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            nx, ny = by - ay, ax - bx
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def polygon_distance(a, b):
    """Distance between two convex polygons; 0.0 when they touch or overlap."""
    if _overlap(a, b):
        return 0.0
    best = math.inf
    for p, q in ((a, b), (b, a)):
        n = len(q)
        for point in p:
            for i in range(n):
                best = min(best, _point_segment(point, q[i], q[(i + 1) % n]))
    return best


def circle_distance(polygon, centre, radius):
    """Distance between a convex polygon and a disc; 0.0 when they touch or overlap."""
    if _contains(polygon, centre):
        return 0.0
    n = len(polygon)
    d = min(_point_segment(centre, polygon[i], polygon[(i + 1) % n]) for i in range(n))
    return max(0.0, d - radius)


def shape_distance(polygon, shape):
    """shape: ("box", corners) or ("cyl", (x, y, radius))."""
    kind, data = shape
    if kind == "box":
        return polygon_distance(polygon, data)
    return circle_distance(polygon, (data[0], data[1]), data[2])
