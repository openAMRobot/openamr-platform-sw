"""
Node-level regression tests for ui_cmd_vel_relay_node.py (issue #35).

Same convention as test_navigation_status_node.py: skipped if rclpy isn't
installed here, run for real in a ROS 2 workspace.
"""
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time  # noqa: E402,I100
from geometry_msgs.msg import Twist, TwistStamped, Vector3  # noqa: E402
from openamrobot_nav2.ui_cmd_vel_relay_node import UiCmdVelRelayNode  # noqa: E402


def _stamped(x, age_s=0.0, now=None):
    now = now if now is not None else time.time()
    stamp_s = now - age_s
    msg = TwistStamped()
    msg.header.stamp = Time(sec=int(stamp_s), nanosec=int((stamp_s % 1) * 1e9))
    msg.twist = Twist(linear=Vector3(x=x))
    return msg


class NodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = UiCmdVelRelayNode()
        self.probe = rclpy.create_node('ui_cmd_vel_relay_probe')

    def tearDown(self):
        self.node.destroy_node()
        self.probe.destroy_node()

    def _spin_both(self, timeout_s=0.3):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)


class TestStaleCommandsAreDropped(NodeTestCase):

    def setUp(self):
        super().setUp()
        self.ui_pub = self.probe.create_publisher(TwistStamped, '/ui/cmd_vel', 1)
        self.received = []
        self.probe.create_subscription(Twist, 'cmd_vel_joy', self.received.append, 10)
        self._spin_both(0.1)  # let discovery settle before publishing

    def test_a_fresh_command_is_forwarded_as_plain_twist(self):
        self.ui_pub.publish(_stamped(x=0.3, age_s=0.0))
        self._spin_both()
        self.assertEqual(len(self.received), 1)
        self.assertAlmostEqual(self.received[0].linear.x, 0.3, places=3)

    def test_a_command_right_at_the_window_edge_is_forwarded(self):
        self.ui_pub.publish(_stamped(x=0.4, age_s=0.24))
        self._spin_both()
        self.assertEqual(len(self.received), 1)

    def test_a_command_older_than_the_window_is_dropped(self):
        self.ui_pub.publish(_stamped(x=0.9, age_s=0.5))
        self._spin_both()
        self.assertEqual(self.received, [])

    def test_a_late_delivered_stale_command_is_not_applied_once_fresh_data_resumes(self):
        # Simulates a connectivity gap: an old message and a fresh one both
        # sitting in the publisher's queue, delivered back-to-back once the
        # link recovers. The relay must judge each one independently against
        # the clock at receipt time, not forward the stale one just because
        # a fresh one follows right behind it.
        self.ui_pub.publish(_stamped(x=0.9, age_s=0.5))   # stale - must drop
        self._spin_both(0.1)  # let it be received and judged before the next one
        self.ui_pub.publish(_stamped(x=0.1, age_s=0.0))   # fresh - must forward
        self._spin_both()
        self.assertEqual(len(self.received), 1)
        self.assertAlmostEqual(self.received[0].linear.x, 0.1, places=3)
