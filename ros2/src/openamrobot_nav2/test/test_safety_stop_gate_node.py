"""
Node-level regression tests for safety_stop_gate_node.py (issue #35).

These exercise the real node against a real rclpy, the same convention as
test_navigation_status_node.py - skipped here if rclpy isn't installed, run
for real in a ROS 2 workspace.

Each test targets one specific property Mohamed Sayed's review of the
twist_mux proposal (platform-sw#35) asked to see demonstrated, not just
asserted - see safety_stop_gate_node.py's own docstring for the full
reasoning behind each one.
"""
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402,I100
from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


class NodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyStopGateNode()
        self.probe = rclpy.create_node('safety_stop_gate_probe')

    def tearDown(self):
        self.node.destroy_node()
        self.probe.destroy_node()

    def _spin_both(self, timeout_s=0.5):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _call_trigger(self, service_name):
        client = self.probe.create_client(Trigger, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=2.0),
            f'{service_name} never came up')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)
        self.assertTrue(future.done(), f'{service_name} call timed out')
        return future.result()


class TestStartupLockedByDefault(NodeTestCase):

    def test_constructed_node_is_already_locked(self):
        # [Startup]: self._locked must already be True the instant the node
        # exists, not "soon after" - there is no code path between
        # construction and this read that could have left it False.
        self.assertTrue(self.node._locked)

    def test_lock_topic_carries_true_before_any_timer_tick(self):
        # The retained TRANSIENT_LOCAL sample must exist from construction,
        # not from the first timer callback - a late subscriber created
        # immediately after this node (the "mux starts after the gate"
        # order) must see True without waiting a tick period.
        received = []
        self.probe.create_subscription(
            Bool, '/safety/stop_lock', received.append, 1)
        self._spin_both(timeout_s=0.2)
        self.assertTrue(received, 'no retained lock sample seen')
        self.assertTrue(received[0].data)


class TestReleaseAndEngage(NodeTestCase):

    def test_release_service_clears_the_lock(self):
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success)
        self.assertFalse(self.node._locked)

    def test_releasing_an_already_released_lock_is_reported_not_an_error(self):
        self._call_trigger('/safety/stop_lock_release')
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success)
        self.assertIn('already', result.message)

    def test_engage_service_re_locks_it(self):
        self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(self.node._locked)
        result = self._call_trigger('/safety/stop_lock_engage')
        self.assertTrue(result.success)
        self.assertTrue(self.node._locked)

    def test_a_direct_false_publish_on_the_lock_topic_does_not_clear_it(self):
        # [Release]: this node does not subscribe to its own lock topic at
        # all, so nothing here could even see, let alone act on, a false
        # published by some other node - release only ever happens through
        # release_lock(), called from the service callback. This test
        # documents that self._locked is unaffected by *this node's own*
        # publish activity; the actual bypass-prevention is the
        # single-publisher graph invariant (stop_lock_invariants.py), which
        # cannot be checked from inside a unit test of one node in
        # isolation - see that tool and this node's own docstring.
        self.node._lock_pub.publish(Bool(data=False))
        self._spin_both(timeout_s=0.1)
        self.assertTrue(self.node._locked)


class TestStopChannelIsAlwaysZero(NodeTestCase):

    def test_cmd_vel_stop_publishes_only_while_locked(self):
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_stop', received.append, 10)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(received, 'expected cmd_vel_stop while locked')
        for msg in received:
            self.assertEqual(msg, Twist())  # every field zero, nothing else possible

    def test_cmd_vel_stop_stops_publishing_once_released(self):
        self._call_trigger('/safety/stop_lock_release')
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_stop', received.append, 10)
        self._spin_both(timeout_s=0.3)
        self.assertEqual(received, [])
