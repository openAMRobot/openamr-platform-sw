#!/usr/bin/env python3
"""
LiDAR body filter for the real OpenAMRobot: ``/scan`` -> ``/scan_filtered``.

The robot's own chassis reflects the LiDAR. This node blanks those self-returns
so they are not treated as obstacles. Two kinds of masked sectors, configured by
ROS parameters (angles in degrees, in the LiDAR frame):

* ``full_mask_sectors_deg`` -- sectors where the body blocks everything (e.g. the
  rear panel): ranges are removed at ALL distances.
* ``close_mask_sectors_deg`` + ``close_max`` -- thin side posts that sit in front
  of real walls: only returns closer than ``close_max`` are removed, real walls
  further away in the same direction are kept.

The output QoS must be endpoint-compatible with the costmap observation source. Nav2 does NOT
universally require RELIABLE (SensorData sources default to BEST_EFFORT); here our costmap source
is configured RELIABLE, so we publish RELIABLE to match -- a BEST_EFFORT publisher with a RELIABLE
subscriber is silently dropped (verify with `ros2 topic info --verbose`).

The default angles are calibrated for this unit's LiDAR mount (rotated 180 deg);
re-measure them if the mount, chassis, or URDF changes.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan


def _pairs_rad(flat, name='sectors'):
    """Validate a flat ``[lo1,hi1,lo2,hi2,...]`` degree list and return ``[(lo,hi),...]`` radians.

    Rejects malformed input (Raj review PR1): odd length, non-finite values, and reversed/wrapped
    pairs (``lo > hi``). Wrap-around sectors are explicitly NOT supported — split them in two.
    """
    flat = list(flat or [])
    if len(flat) % 2 != 0:
        raise ValueError(f'{name}: needs an even number of values (lo,hi pairs), got {len(flat)}')
    if any(not math.isfinite(v) for v in flat):
        raise ValueError(f'{name}: non-finite value in {flat}')
    pairs = []
    for i in range(0, len(flat), 2):
        lo, hi = flat[i], flat[i + 1]
        if lo > hi:
            raise ValueError(f'{name}: reversed/wrapped pair ({lo},{hi}); give lo<=hi in degrees '
                             '(split a wrap-around sector into two pairs)')
        pairs.append((math.radians(lo), math.radians(hi)))
    return pairs


class ScanBodyFilter(Node):
    """Blank the robot's own body reflections from a LaserScan."""

    def __init__(self):
        super().__init__('scan_body_filter')
        self.declare_parameter('scan_in', '/scan')
        self.declare_parameter('scan_out', '/scan_filtered')
        self.declare_parameter('reliable_qos', True)
        self.declare_parameter('close_max', 0.40)
        self.declare_parameter('full_mask_sectors_deg', [-45.0, 49.0])
        self.declare_parameter('close_mask_sectors_deg', [-96.0, -73.0, 73.0, 96.0])

        scan_in = self.get_parameter('scan_in').value
        scan_out = self.get_parameter('scan_out').value
        self.close_max = self.get_parameter('close_max').value
        full_deg = self.get_parameter('full_mask_sectors_deg').value
        close_deg = self.get_parameter('close_mask_sectors_deg').value
        self.full = _pairs_rad(full_deg, 'full_mask_sectors_deg')
        self.close = _pairs_rad(close_deg, 'close_mask_sectors_deg')
        if not math.isfinite(self.close_max) or self.close_max <= 0.0:
            raise ValueError(f'close_max must be a positive finite distance, got {self.close_max}')

        if self.get_parameter('reliable_qos').value:
            pub_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE)
        else:
            pub_qos = qos_profile_sensor_data

        self.pub = self.create_publisher(LaserScan, scan_out, pub_qos)
        self.sub = self.create_subscription(
            LaserScan, scan_in, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'scan_body_filter: {scan_in} -> {scan_out} | full={full_deg} '
            f'| close<{self.close_max}m={close_deg}')

    @staticmethod
    def _in_any(sectors, ang):
        return any(lo <= ang <= hi for lo, hi in sectors)

    def cb(self, m):
        out = LaserScan()
        out.header = m.header
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        inf = float('inf')
        ranges = list(m.ranges)
        for i, v in enumerate(ranges):
            if not math.isfinite(v):
                continue
            ang = m.angle_min + i * m.angle_increment
            if self._in_any(self.full, ang):
                ranges[i] = inf
            elif v < self.close_max and self._in_any(self.close, ang):
                ranges[i] = inf
        out.ranges = ranges
        out.intensities = m.intensities
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanBodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
