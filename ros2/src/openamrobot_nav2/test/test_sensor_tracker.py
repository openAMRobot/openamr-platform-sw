"""
Tests for SensorTracker: fresh, stale, absent and restored.

Uses made-up timestamps instead of a running sim. With use_sim_time on,
stopping the sim freezes /clock too, so a live test can never show a sensor
going stale.

Skipped if openamr_nav_msgs (openamrobot-interfaces) isn't installed.
"""

import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')

from openamr_nav_msgs.msg import SensorStatus  # noqa: E402,I100
from openamrobot_nav2.navigation_status_node import SensorTracker  # noqa: E402
from rclpy.time import Time  # noqa: E402


def make_time(seconds: float) -> Time:
    return Time(seconds=int(seconds), nanoseconds=int((seconds % 1) * 1e9))


REQUIRED_CFG = {
    'id': 'lidar_nav', 'kind': 'LIDAR', 'topic': '/scan_filtered',
    'frame_id': 'base_scan', 'required': True,
    'nominal_period_s': 0.1, 'stale_multiple': 5.0,  # stale after 0.5 s
}

OPTIONAL_CFG = {
    'id': 'imu', 'kind': 'IMU', 'topic': '/imu',
    'frame_id': 'base_link', 'required': False,
    'nominal_period_s': 0.02, 'stale_multiple': 5.0,
}


class TestSensorTracker(unittest.TestCase):

    def test_never_seen_required_is_unknown(self):
        tracker = SensorTracker(REQUIRED_CFG)
        self.assertEqual(tracker.state(make_time(0.0)), SensorStatus.STATE_UNKNOWN)

    def test_never_seen_optional_is_absent(self):
        tracker = SensorTracker(OPTIONAL_CFG)
        self.assertEqual(tracker.state(make_time(0.0)), SensorStatus.STATE_ABSENT)

    def test_fresh_message_is_ok(self):
        tracker = SensorTracker(REQUIRED_CFG)
        tracker.on_message(make_time(10.0))
        # 0.05 s later, well under the 0.5 s stale threshold
        self.assertEqual(tracker.state(make_time(10.05)), SensorStatus.STATE_OK)

    def test_becomes_stale_after_threshold(self):
        tracker = SensorTracker(REQUIRED_CFG)
        tracker.on_message(make_time(10.0))
        # 0.6 s later, past the 0.5 s threshold with nothing new
        self.assertEqual(tracker.state(make_time(10.6)), SensorStatus.STATE_STALE)

    def test_recovers_after_restored_message(self):
        tracker = SensorTracker(REQUIRED_CFG)
        tracker.on_message(make_time(10.0))
        self.assertEqual(tracker.state(make_time(10.6)), SensorStatus.STATE_STALE)
        # a new message arrives: back to OK, not latched STALE
        tracker.on_message(make_time(10.6))
        self.assertEqual(tracker.state(make_time(10.65)), SensorStatus.STATE_OK)

    def test_rate_hz_reflects_actual_interval(self):
        tracker = SensorTracker(REQUIRED_CFG)
        # 10 messages exactly 0.1 s apart should read back as ~10 Hz
        for i in range(10):
            tracker.on_message(make_time(i * 0.1))
        self.assertAlmostEqual(tracker.rate_hz(), 10.0, delta=0.1)

    def test_age_duration_matches_elapsed_time(self):
        tracker = SensorTracker(REQUIRED_CFG)
        tracker.on_message(make_time(10.0))
        age = tracker.age_duration(make_time(10.3))
        self.assertEqual(age.sec, 0)
        self.assertAlmostEqual(age.nanosec / 1e9, 0.3, places=2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
