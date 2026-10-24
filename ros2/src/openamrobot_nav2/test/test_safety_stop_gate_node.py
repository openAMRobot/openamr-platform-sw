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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
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
        # Mohamed's review (platform-sw#90): this used to spin BOTH nodes
        # for 0.2s with a plain probe subscriber, so it could pass purely
        # from _tick()'s periodic republish (10 Hz) and proved nothing
        # about a sample being retained "before any timer tick."
        #
        # Pin the claim down structurally instead of racing a clock: only
        # self.probe is ever spun in this test - self.node (the gate) is
        # NEVER spun here, so its create_timer(_tick) callback cannot
        # execute even once, no matter how long this waits for DDS
        # discovery. Whatever the probe receives, it received with the
        # gate's own timer callback never having run - that is the actual
        # guarantee in the test's name, independent of wall-clock timing.
        #
        # The probe's own subscription QoS matters here too, and this is
        # the exact mistake Mohamed's review is about (platform-sw#90):
        # create_subscription(..., 1) with a plain int depth defaults to
        # VOLATILE durability in rclpy, same as twist_mux's real
        # LockTopicHandle. A volatile subscriber only gets LIVE samples
        # published after it matches - it does NOT receive a retained
        # TRANSIENT_LOCAL sample from before it existed. The first version
        # of this test "worked" only because _tick()'s periodic republish
        # kept generating fresh live samples once both nodes were spun -
        # exactly the thing Mohamed's review said this test could not tell
        # apart from the real guarantee. To actually exercise durability,
        # the probe has to request it: a TRANSIENT_LOCAL subscriber
        # matched against a TRANSIENT_LOCAL publisher receives the
        # retained sample on match, which is the one property this test is
        # named for.
        lock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = []
        self.probe.create_subscription(
            Bool, '/safety/stop_lock', received.append, lock_qos)
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.05)
        self.assertTrue(
            received,
            'no lock sample ever reached a probe subscribed after '
            'construction - the gate node was never spun, so this is not '
            'a timing issue, the retained sample itself is missing')
        self.assertTrue(received[0].data)
        # This only proves THIS node's own publisher/QoS behaves as
        # intended. It is NOT evidence about twist_mux: its own lock
        # subscriber uses SystemDefaultsQoS() (volatile), so it does NOT
        # receive this retained sample at all (platform-sw#90) - only a
        # live message published after it subscribes. That is the real
        # mux's behavior, covered by test_twist_mux_startup_and_restart.py,
        # not by anything that can be proven from this node in isolation.


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
