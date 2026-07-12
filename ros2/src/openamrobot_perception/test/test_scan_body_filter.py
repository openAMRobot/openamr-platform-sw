"""Behavioral tests for the LiDAR body filter."""

import math

from openamrobot_perception.scan_body_filter import _pairs_rad, ScanBodyFilter
import pytest
import rclpy
from sensor_msgs.msg import LaserScan


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Provide a ROS context for node construction."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pairs_rad_validates_and_converts_degrees():
    pairs = _pairs_rad([-180.0, -90.0, 10.0, 20.0], 'mask')
    assert pairs[0] == pytest.approx(
        (math.radians(-180.0), math.radians(-90.0)))
    assert pairs[1] == pytest.approx(
        (math.radians(10.0), math.radians(20.0)))
    assert _pairs_rad(None) == []


@pytest.mark.parametrize('values', [
    [0.0],
    [20.0, 10.0],
    [0.0, float('inf')],
    [float('nan'), 10.0],
])
def test_pairs_rad_rejects_invalid_sectors(values):
    with pytest.raises(ValueError):
        _pairs_rad(values, 'mask')


def _scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = math.radians(-100.0)
    msg.angle_max = math.radians(100.0)
    msg.angle_increment = math.radians(50.0)
    msg.time_increment = 0.01
    msg.scan_time = 0.1
    msg.range_min = 0.05
    msg.range_max = 10.0
    msg.ranges = ranges
    msg.intensities = [1.0] * len(ranges)
    return msg


def test_callback_masks_body_and_preserves_scan_metadata():
    node = ScanBodyFilter()
    published = []
    node.pub = type('PublisherSpy', (), {'publish': staticmethod(published.append)})()
    try:
        msg = _scan([0.2, 0.2, 1.0, float('nan'), 0.2])
        node.cb(msg)
        out = published.pop()

        # With this 50-degree scan, only 0 degrees is inside the full mask.
        assert out.ranges[0] == pytest.approx(0.2)
        assert out.ranges[1] == pytest.approx(0.2)
        assert math.isinf(out.ranges[2])
        assert math.isnan(out.ranges[3])
        assert out.ranges[4] == pytest.approx(0.2)
        assert out.header.frame_id == msg.header.frame_id
        assert out.angle_increment == msg.angle_increment
        assert list(out.intensities) == list(msg.intensities)
    finally:
        node.destroy_node()


def test_close_sector_keeps_distant_obstacles():
    node = ScanBodyFilter()
    node.full = []
    node.close = _pairs_rad([-100.0, -70.0])
    published = []
    node.pub = type('PublisherSpy', (), {'publish': staticmethod(published.append)})()
    try:
        node.cb(_scan([0.2, 1.0, 1.0, 1.0, 1.0]))
        assert math.isinf(published[0].ranges[0])

        published.clear()
        node.cb(_scan([0.8, 1.0, 1.0, 1.0, 1.0]))
        assert published[0].ranges[0] == pytest.approx(0.8)
    finally:
        node.destroy_node()


def test_sector_boundaries_are_inclusive():
    assert ScanBodyFilter._in_any([(1.0, 2.0)], 1.0)
    assert ScanBodyFilter._in_any([(1.0, 2.0)], 2.0)
    assert not ScanBodyFilter._in_any([(1.0, 2.0)], 2.01)
