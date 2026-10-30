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


class TestPostMuxGate(NodeTestCase):
    """
    cmd_vel_gated (platform-sw#90, Alex's review).

    These reproduce Mohamed's/Nikolai's equal-priority case at node level: a
    nonzero Twist arriving on cmd_vel_mux - exactly what the mux would
    forward if some OTHER publisher won arbitration on cmd_vel_stop, which
    this node cannot prevent (see the docstring's [Equal-priority nonzero
    stop] section) - must never reach cmd_vel_gated while locked.
    """

    def _publish_mux(self, msg):
        pub = self.probe.create_publisher(Twist, 'cmd_vel_mux', 10)
        deadline = time.monotonic() + 1.0
        # Wait for the gate's subscription to actually match before
        # publishing, else the message can be sent into a graph with no
        # matched subscriber yet and silently dropped (no retained sample -
        # cmd_vel_mux has no special QoS here).
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(
            pub.get_subscription_count(), 0,
            'gate node never subscribed to cmd_vel_mux')
        pub.publish(msg)
        pub.destroy()

    def test_nonzero_on_cmd_vel_mux_comes_out_zero_on_cmd_vel_gated_while_locked(self):
        nonzero = Twist()
        nonzero.linear.x = 0.3
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self.assertTrue(self.node._locked)  # sanity: still locked (default)
        self._publish_mux(nonzero)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(
            received, 'expected at least one cmd_vel_gated message '
            '(the tick-driven zero, if nothing else)')
        for msg in received:
            self.assertEqual(
                msg, Twist(),
                'a nonzero Twist on cmd_vel_mux reached cmd_vel_gated while '
                'locked - the post-mux gate did not hold (platform-sw#90)')

    def test_cmd_vel_mux_is_forwarded_unchanged_while_unlocked(self):
        self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(self.node._locked)
        nonzero = Twist()
        nonzero.linear.x = 0.3
        nonzero.angular.z = -0.2
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self._publish_mux(nonzero)
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(received, 'cmd_vel_mux message never reached cmd_vel_gated')
        self.assertEqual(
            received[-1], nonzero,
            'unlocked pass-through must forward the mux output unchanged, '
            'not substitute or drop fields')

    def test_cmd_vel_gated_still_zero_if_mux_goes_silent_while_locked(self):
        # No publish to cmd_vel_mux at all here - only _tick()'s own
        # independent force-zero should be responsible for this.
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self.assertTrue(self.node._locked)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(received, 'expected cmd_vel_gated while locked, even with a silent mux')
        for msg in received:
            self.assertEqual(msg, Twist())


class SimClockPostMuxGate(unittest.TestCase):
    """
    cmd_vel_gated under a simulated clock that has never ticked once
    (platform-sw#90, Mohamed's follow-up review at 710c98b).

    The remaining ask was to demonstrate the FINAL cmd_vel_gated path -
    not just cmd_vel_mux - through a zero/paused simulated clock, a gate
    start or restart, and positive forwarding after an explicit release.

    safety_stop_gate_node.py's own docstring claims cmd_vel_gated's value
    depends only on self._locked, decided synchronously in __init__
    before any publisher, subscription or clock involvement exists - never
    on now(), /clock, or anything twist_mux itself does with simulated
    time. These tests exist to hold the real node to that claim, under
    the strongest form of "paused simulated clock" available: one that
    has never received a single /clock message, in this process, ever.
    Every node in this class runs with use_sim_time:=true and there is no
    /clock publisher anywhere - if cmd_vel_gated's correctness secretly
    depended on now() ticking even once, this is the condition most
    likely to expose it.

    Result: the event-driven side of the node (self._locked itself, and
    the _on_mux_output subscription callback that republishes onto
    cmd_vel_gated) is confirmed clock-independent - it fires correctly on
    message/service receipt with zero /clock messages ever seen. The
    _tick() belt-and-suspenders republish is a separate story: it runs on
    a timer created with create_timer(), which under use_sim_time:=true
    uses ROS time, and ROS time does not advance - at all - without an
    actual /clock publisher. So with no /clock ever received, _tick()
    simply never fires, in this node or in any other node in this
    situation; that is standard rclpy sim-time timer behavior, not a bug
    in this file. test_tick_does_not_fire_without_a_clock_tick below
    pins that down explicitly, because it is a real, narrower limit on
    the periodic backstop that's worth being honest about: if /clock is
    late to start or stalls while the mux goes silent, the *periodic*
    zero-republish will not run until a /clock message arrives - only the
    message-driven path covers that window.

    This class tests the gate node directly against a probe publisher on
    cmd_vel_mux, the same pattern as TestPostMuxGate above - it does not
    start a real twist_mux subprocess. Whether the real twist_mux binary
    itself forwards cmd_vel_nav onto cmd_vel_mux under use_sim_time:=true
    is a separate, still-open question (see the PR's Not verified list);
    it is not what these tests claim to answer.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyStopGateNode()
        self.probe = rclpy.create_node('safety_stop_gate_sim_probe')

    def tearDown(self):
        self.node.destroy_node()
        self.probe.destroy_node()

    def _spin_both(self, timeout_s=0.5):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _publish_mux(self, msg):
        pub = self.probe.create_publisher(Twist, 'cmd_vel_mux', 10)
        deadline = time.monotonic() + 1.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(
            pub.get_subscription_count(), 0,
            'gate node never subscribed to cmd_vel_mux under use_sim_time:=true')
        pub.publish(msg)
        pub.destroy()

    def test_locked_from_construction_with_no_clock_message_ever_received(self):
        # self._locked is a plain bool, set in __init__ before any
        # publisher, subscriber or timer exists - this is decided before
        # a /clock message could possibly matter, and it is.
        self.assertTrue(self.node._locked)

    def test_tick_does_not_fire_without_a_clock_tick(self):
        # Documents a real, narrow limit (not a bug in this file): the
        # periodic _tick() republish runs on a create_timer() timer,
        # which under use_sim_time:=true is driven by ROS time, and ROS
        # time does not move without an actual /clock publisher. With
        # none ever received, _tick() does not fire in this window, so
        # cmd_vel_gated sees nothing from the timer path - only the
        # message-driven _on_mux_output path covers a silent mux during
        # a stalled/late /clock. See the class docstring.
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self._spin_both(timeout_s=0.3)
        self.assertEqual(
            received, [],
            '_tick() fired without any /clock message ever being '
            'published - if this now fails, the sim-time timer behavior '
            'this test documents has changed and the comment above (and '
            'the PR write-up) needs updating, not this assertion')

    def test_nonzero_on_mux_still_comes_out_zero_with_no_clock_ever_received(self):
        nonzero = Twist()
        nonzero.linear.x = 0.3
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self.assertTrue(self.node._locked)
        self._publish_mux(nonzero)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(received)
        for msg in received:
            self.assertEqual(
                msg, Twist(),
                'a nonzero Twist on cmd_vel_mux reached cmd_vel_gated while '
                'locked under a simulated clock that never ticked')

    def test_release_and_positive_forward_with_no_clock_ever_received(self):
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(
            client.wait_for_service(timeout_sec=2.0),
            '/safety/stop_lock_release never came up')
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)
        self.assertTrue(future.done(), 'release call timed out')
        self.assertFalse(self.node._locked)

        nonzero = Twist()
        nonzero.linear.x = 0.4
        nonzero.angular.z = 0.1
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self._publish_mux(nonzero)
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(
            received,
            'cmd_vel_mux message never reached cmd_vel_gated after a '
            'confirmed Resume, under a simulated clock that never ticked')
        self.assertEqual(received[-1], nonzero)

    def test_restart_comes_back_locked_with_no_clock_ever_received(self):
        # A gate-node restart under the same never-ticked simulated clock:
        # destroy and recreate, confirm it comes back locked before
        # anything else happens, exactly as at first construction. This
        # is the _locked flag itself (see test_locked_from_construction
        # above for why cmd_vel_gated's *periodic* republish is a
        # separate claim, covered by test_tick_does_not_fire_...).
        self.node.destroy_node()
        self.node = SafetyStopGateNode()
        self.assertTrue(self.node._locked)

    def test_restart_then_mux_message_still_comes_out_zero(self):
        # The property that actually matters after a restart: once the
        # node is back up, a message on cmd_vel_mux still comes out zero
        # on cmd_vel_gated while locked, with zero /clock messages ever
        # received across either instance's lifetime.
        self.node.destroy_node()
        self.node = SafetyStopGateNode()
        self.assertTrue(self.node._locked)
        nonzero = Twist()
        nonzero.linear.x = 0.3
        received = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', received.append, 10)
        self._publish_mux(nonzero)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(received, 'cmd_vel_mux message never reached cmd_vel_gated after restart')
        for msg in received:
            self.assertEqual(msg, Twist())
