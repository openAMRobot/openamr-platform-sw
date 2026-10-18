#!/usr/bin/env python3
"""Print the returns within 1 m of one LaserScan message per topic.

Used to measure the robot self-view sector in the scan. usage: scanstat.py TOPIC...
"""
import math
import sys

import rclpy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('scanstat')
    got = {}

    def mk(topic):
        def cb(m):
            if topic in got:
                return
            pts = []
            for i, r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min <= r <= 1.0:
                    a = math.degrees(m.angle_min + i * m.angle_increment)
                    pts.append((round(a), round(r, 3)))
            got[topic] = (m.header.frame_id, m.range_min, len(m.ranges), pts)
        return cb

    subs = [node.create_subscription(LaserScan, t, mk(t), 10) for t in sys.argv[1:]]
    while len(got) < len(subs):
        rclpy.spin_once(node, timeout_sec=0.5)
    for t, (frame, rmin, n, pts) in got.items():
        print(t, 'frame', frame, 'range_min', rmin, 'n', n, 'returns<=1m:', len(pts))
        if pts:
            print('    angles:', min(a for a, _ in pts), max(a for a, _ in pts))
        print('    all:', pts)


if __name__ == '__main__':
    main()
