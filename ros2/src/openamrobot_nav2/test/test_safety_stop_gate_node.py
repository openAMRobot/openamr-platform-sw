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

rclpy = pytest.importorskip('rclpy')  # tracking: #35

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray  # noqa: E402,I100
from action_msgs.srv import CancelGoal  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from openamr_nav_msgs.msg import StopState  # noqa: E402
from openamrobot_nav2.goal_authority import ProducerState  # noqa: E402
from openamrobot_nav2.safety_stop_gate_node import (  # noqa: E402
    _CANCEL_ACTIONS, _CMD_VEL_INPUT_TIMEOUTS, _LEDGER_PRODUCERS, _LEDGER_STATUS_DISCOVERY_GRACE_S,
    _real_action_server_name, SafetyStopGateNode, T_READY)
from rclpy.duration import Duration  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_action_status_default)
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


_NAV_ONLY_PRODUCERS = [a for a in _CANCEL_ACTIONS if a not in ('dock_robot', 'undock_robot')]


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
        self._cancel_services = {}
        self._dock_abort_service = None

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

    def _publish_to_gate(self, msg_type, topic, msg, qos=10):
        """Publish once to a topic the gate subscribes to, waiting for it to match first."""
        pub = self.probe.create_publisher(msg_type, topic, qos)
        deadline = time.monotonic() + 1.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(
            pub.get_subscription_count(), 0, f'gate node never subscribed to {topic}')
        pub.publish(msg)
        pub.destroy()

    def _ensure_cancel_service_reachable(self, action):
        """
        Stand up a fake cancel_goal server for action, if not already up.

        Release condition (a) now checks this client's service_is_ready()
        as well as content (platform-sw#90, Mohamed's review) - a test that
        publishes a goal status at all is implicitly claiming that action's
        server is reachable, unless it specifically wants to test otherwise
        (see _take_down_cancel_service).
        """
        if action in self._cancel_services:
            return

        def _cb(request, response):
            del request
            response.return_code = CancelGoal.Response.ERROR_NONE
            return response
        self._cancel_services[action] = self.probe.create_service(
            CancelGoal, f'{_real_action_server_name(action)}/_action/cancel_goal', _cb)
        self._spin_both(timeout_s=0.3)  # let the gate's client discover it

    def _take_down_cancel_service(self, action):
        """Simulate that action's server having crashed/gone away entirely."""
        self._ensure_cancel_service_reachable(action)
        self.probe.destroy_service(self._cancel_services.pop(action))
        self._spin_both(timeout_s=0.3)  # let discovery catch up with the removal

    def _ensure_dock_abort_service_reachable(self):
        """Do the same as _ensure_cancel_service_reachable, for /dock_trigger/abort."""
        if self._dock_abort_service is not None:
            return

        def _cb(request, response):
            del request
            response.success = True
            response.message = ''
            return response
        self._dock_abort_service = self.probe.create_service(
            Trigger, '/dock_trigger/abort', _cb)
        self._spin_both(timeout_s=0.3)

    def _take_down_dock_abort_service(self):
        self._ensure_dock_abort_service_reachable()
        self.probe.destroy_service(self._dock_abort_service)
        self._dock_abort_service = None
        self._spin_both(timeout_s=0.3)

    def _publish_goal_status(self, action, statuses):
        """statuses: an iterable of action_msgs/GoalStatus status ints (empty = no active goal)."""
        self._ensure_cancel_service_reachable(action)
        msg = GoalStatusArray()
        for s in statuses:
            gs = GoalStatus()
            gs.status = s
            msg.status_list.append(gs)
        # Matches the gate's own subscription QoS (platform-sw#90, panthera-
        # momagdii's P2 review): real action status is RELIABLE +
        # TRANSIENT_LOCAL depth 1 (rcl_action_qos_profile_status_default),
        # not the default volatile this test used before - a mismatch here
        # means the publisher/subscriber never even match.
        self._publish_to_gate(
            GoalStatusArray, f'{_real_action_server_name(action)}/_action/status', msg,
            qos=qos_profile_action_status_default)

    def _publish_goal_authority_heartbeat(
            self, producer='undock_robot', instance_id='test-instance', outstanding=0):
        """D-13 (platform-sw#124): establishes ledger IDLE for a producer off the legacy topic.

        4-field form (panthera-momagdii's #90 review, P2): the gate now
        requires a heartbeat to carry its outstanding count, fail-closed
        when missing.
        """
        self._publish_to_gate(
            String, 'goal_authority_events',
            String(data=f'{producer}|heartbeat|{instance_id}|{outstanding}'))

    def _publish_all_clear_goal_status(self):
        for action in _CANCEL_ACTIONS:
            self._publish_goal_status(action, [])
        # Every ledger producer (platform-sw#124, D-13) needs its own
        # heartbeat - publishing an empty legacy status for one is harmless
        # but establishes nothing on its own, now that navigate_to_pose is
        # also ledger-backed (navigate_to_pose_goal_proxy_node.py).
        for producer in _LEDGER_PRODUCERS:
            self._publish_goal_authority_heartbeat(producer=producer)

    def _publish_dock_status(self, status: str):
        self._ensure_dock_abort_service_reachable()
        self._publish_to_gate(String, 'dock_trigger_status', String(data=status))

    def _make_all_release_conditions_clear(self):
        """
        platform-sw#119: every one of (a)/(b)/(c) in the release-refusal gate
        needs a believable 'nothing is in the way' baseline before a release
        this file isn't specifically testing the refusal of can succeed - a
        test only has to publish something DIFFERENT for the one condition
        it actually targets.
        """
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)

    def _release_now(self):
        """
        Release the lock in tests that are not about startup readiness or
        the release-refusal gate itself.

        Backdates the node's own recorded start time so T_READY (platform-
        sw#90 review, Alex, BLOCKER 2) has already elapsed, publishes the
        'nothing is in the way' baseline every release condition (a)/(b)/(c)
        needs (platform-sw#119), then calls /safety/stop_lock_release. Tests
        whose actual subject is something else (cmd_vel_stop, the post-mux
        gate while unlocked) use this instead of calling the release service
        directly - see TestReleaseReadinessIsRelativeToStartup for T_READY
        itself and TestReleaseBlocked* for the refusal conditions themselves.
        """
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._make_all_release_conditions_clear()
        return self._call_trigger('/safety/stop_lock_release')


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
        result = self._release_now()
        self.assertTrue(result.success)
        self.assertFalse(self.node._locked)

    def test_releasing_an_already_released_lock_is_reported_not_an_error(self):
        self._release_now()
        result = self._release_now()
        self.assertTrue(result.success)
        self.assertIn('already', result.message)

    def test_engage_service_re_locks_it(self):
        self._release_now()
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
        self._release_now()
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
        self._release_now()
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


class TestReleaseBlockedByActiveGoal(NodeTestCase):
    """
    platform-sw#119 release condition (a): every tracked action must report
    no active goal, with fresh status.
    """

    def test_release_refused_while_a_nav_goal_is_active(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._publish_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('navigate_to_pose', result.message)
        self.assertTrue(self.node._locked)

    def test_release_refused_while_a_goal_is_cancelling(self):
        # CANCELING, not yet terminal - the cancel was sent but not confirmed
        # done, so this must block the same as EXECUTING does.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._publish_goal_status('dock_robot', [GoalStatus.STATUS_CANCELING])
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_robot', result.message)

    def test_release_refused_while_an_actions_status_was_never_received(self):
        # platform-sw#119: "stale or missing status means refused" - a status
        # array we have never seen at all must not be treated as "probably
        # fine." navigate_to_pose is ledger-backed now (D-13,
        # navigate_to_pose_goal_proxy_node.py), not status-topic-content, so
        # this targets navigate_through_poses instead - still on the legacy
        # path. Every action but it gets a clear status/heartbeat here.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for action in _CANCEL_ACTIONS:
            if action == 'navigate_through_poses':
                continue
            if action in _LEDGER_PRODUCERS:
                self._publish_goal_authority_heartbeat(producer=action)
            else:
                self._publish_goal_status(action, [])
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('navigate_through_poses', result.message)
        self.assertIn('never received', result.message)

    def test_release_refused_while_the_ledger_producer_was_never_heard_from(self):
        # The ledger-side equivalent of the test above, for navigate_to_pose
        # specifically: no heartbeat, no sent/done, ever - D-13 rejects
        # inferring "clear" from silence here too, same as the legacy path,
        # just with the ledger's own wording ("unknown", not "never
        # received") since it is a different mechanism underneath.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for action in _CANCEL_ACTIONS:
            if action == 'navigate_to_pose':
                continue
            if action in _LEDGER_PRODUCERS:
                self._publish_goal_authority_heartbeat(producer=action)
            else:
                self._publish_goal_status(action, [])
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('navigate_to_pose', result.message)
        self.assertIn('unknown', result.message)

    def test_release_accepted_with_an_old_all_clear_sample_if_the_server_is_reachable(self):
        # platform-sw#90, Mohamed's review (P1): ROS 2 action status only
        # publishes on a transition, not as a heartbeat - a healthy, idle
        # server's last sample can age indefinitely with nothing wrong.
        # Content plus a live service_is_ready() check now proves no old
        # goal has authority; message age alone no longer refuses this.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        time.sleep(1.3)  # real elapsed time, no further status for undock_robot
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success, result.message)

    def test_release_refused_while_an_actions_cancel_service_is_unreachable(self):
        # Same old, stale "all clear" sample as above, but this time the
        # server behind it is actually gone (not just quiet) - release must
        # still refuse, with the same undifferentiated reason as "never
        # received," since this gate's job is to stay refused correctly,
        # not to tell those two cases apart (see _release_blockers).
        # dock_robot, not undock_robot: this is the legacy status-topic
        # reachability check (D-13, platform-sw#124) - undock_robot moved to
        # the goal-authority ledger, which does not consult this client at
        # all, so this scenario no longer applies to it.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        self._take_down_cancel_service('dock_robot')
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_robot', result.message)
        self.assertIn('stale or never received', result.message)

    def test_release_accepted_once_every_action_reports_clear(self):
        result = self._release_now()
        self.assertTrue(result.success)
        self.assertFalse(self.node._locked)


class TestReleaseBlockedByDockTrigger(NodeTestCase):
    """
    platform-sw#119 release condition (b): dock_trigger must not be
    mid-sequence, with fresh status.
    """

    def test_release_refused_while_docking(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('docking')
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_trigger', result.message)

    def test_release_refused_while_undocking(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('undocking')
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_trigger', result.message)

    def test_release_accepted_while_resting_docked(self):
        # 'docked' (successfully resting, not mid-sequence) must not block
        # release - only the two actively-moving states above do. This is a
        # judgment call on Alex's #119 wording ("dock_trigger reports idle"
        # read literally would never release while docked at all) - flagged
        # for Alex/Mohamed to confirm, not assumed silently.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('docked')
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success)

    def test_release_refused_while_dock_status_was_never_received(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_trigger', result.message)
        self.assertIn('never received', result.message)

    def test_release_accepted_with_an_old_idle_sample_if_the_abort_service_is_reachable(self):
        # platform-sw#90, Mohamed's review: dock_trigger's own idle
        # heartbeat is every 2.0s (dock_trigger.py's _heartbeat_status) -
        # any fixed freshness window shorter than that could refuse release
        # on a perfectly idle dock. Content plus a live service_is_ready()
        # check, not message age, is what this gate judges now.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        time.sleep(1.3)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success, result.message)

    def test_release_refused_while_dock_abort_service_is_unreachable(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)
        self._take_down_dock_abort_service()
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('dock_trigger', result.message)
        self.assertIn('stale or never received', result.message)


class TestReleaseIgnoresDockTriggerWhenDockingDisabled(NodeTestCase):
    """
    platform-sw#90, panthera-momagdii's review: _release_blockers() used to
    require dock_trigger_status/dock_trigger/abort unconditionally, even on
    the nav-only profile where enabled_producers excludes both dock_robot
    and undock_robot and no dock_trigger process ever starts at all - that
    blocked Resume forever on that profile.
    """

    @classmethod
    def setUpClass(cls):
        # Global CLI parameter override, same pattern
        # test_d13_real_dock_trigger_clean_boot_and_exclusion.py's
        # TestEnabledProducersExcludesUndockRobot already uses -
        # SafetyStopGateNode's own __init__ takes no kwargs to forward one
        # through directly.
        rclpy.init(args=[
            '--ros-args', '-p',
            f"enabled_producers:=[{','.join(_NAV_ONLY_PRODUCERS)}]",
        ])

    def test_release_succeeds_with_no_dock_trigger_contact_ever(self):
        self.assertEqual(self.node._enabled_producers, set(_NAV_ONLY_PRODUCERS))
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for action in _NAV_ONLY_PRODUCERS:
            if action in _LEDGER_PRODUCERS:
                self._publish_goal_authority_heartbeat(producer=action)
            else:
                self._publish_goal_status(action, [])
        self._spin_both(timeout_s=0.2)
        # Deliberately never publishes dock_trigger_status and never stands
        # up /dock_trigger/abort - the nav-only profile starts no
        # dock_trigger process at all.
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(
            result.success,
            f'nav-only profile (no dock_robot/undock_robot enabled) still '
            f'blocked on dock_trigger despite it never starting: {result.message}')


class TestReleaseBlockedByCmdVelInputs(NodeTestCase):
    """
    platform-sw#119 release condition (c): cmd_vel_nav/dock/joy must each be
    zero or silent past their own timeout.
    """

    def test_release_refused_while_cmd_vel_nav_is_fresh_and_nonzero(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        nonzero = Twist()
        nonzero.linear.x = 0.2
        self._publish_to_gate(Twist, 'cmd_vel_nav', nonzero)
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('cmd_vel_nav', result.message)

    def test_manual_zero_does_not_substitute_for_an_active_nav_goal(self):
        # platform-sw#119's exact finding: a manual zero on cmd_vel_joy must
        # not rearm/release while an old nav goal is still active - the two
        # are independent conditions, (a) and (c), both required.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        self._publish_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._publish_to_gate(Twist, 'cmd_vel_joy', Twist())
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)
        self.assertIn('navigate_to_pose', result.message)

    def test_release_accepted_once_a_nonzero_input_goes_silent_past_its_own_timeout(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        nonzero = Twist()
        nonzero.linear.x = 0.2
        self._publish_to_gate(Twist, 'cmd_vel_dock', nonzero)
        self._spin_both(timeout_s=0.1)
        # cmd_vel_dock's own twist_mux.yaml timeout is 0.3s - backdate the
        # gate's recorded receive time rather than a real sleep.
        self.node._cmd_vel_last_at['cmd_vel_dock'] -= Duration(
            seconds=_CMD_VEL_INPUT_TIMEOUTS['cmd_vel_dock'] + 0.1)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success)

    def test_release_accepted_when_an_input_was_never_received_at_all(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        # no cmd_vel_* publish at all - never-received counts as silent by
        # definition, not as unknown (unlike the action/dock statuses above).
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success)


class TestEngageCancelsGoalsAndAbortsDocking(NodeTestCase):
    """platform-sw#119: engage must actively cancel, not just mask cmd_vel_gated downstream."""

    def _serve_cancel_goal(self, action):
        calls = []

        def _cb(request, response):
            calls.append(request)
            response.return_code = 0
            return response

        self.probe.create_service(
            CancelGoal, f'{_real_action_server_name(action)}/_action/cancel_goal', _cb)
        return calls

    def test_engage_calls_cancel_goal_on_every_tracked_action(self):
        calls = {action: self._serve_cancel_goal(action) for action in _CANCEL_ACTIONS}
        self._spin_both(timeout_s=0.5)  # let the gate's clients discover the fake servers
        self._call_trigger('/safety/stop_lock_engage')
        self._spin_both(timeout_s=0.3)
        for action, received in calls.items():
            self.assertTrue(received, f'{action} cancel_goal was never called')
            self.assertEqual(
                received[0].goal_info, GoalInfo(),
                f'{action} cancel_goal was not a zero-id, zero-stamp cancel-all request')

    def test_cancellation_repeats_on_a_later_tick_while_still_locked(self):
        calls = {action: self._serve_cancel_goal(action) for action in _CANCEL_ACTIONS}
        self._spin_both(timeout_s=0.5)
        before = {a: len(c) for a, c in calls.items()}
        self._spin_both(timeout_s=0.5)
        for action, received in calls.items():
            self.assertGreater(
                len(received), before[action],
                f'{action} was not re-cancelled on a later tick while still locked '
                '(platform-sw#119 test 6: a goal sent during the stop must be cancelled too)')

    def test_dock_trigger_abort_is_called_on_engage(self):
        calls = []

        def _cb(request, response):
            calls.append(request)
            response.success = True
            response.message = ''
            return response

        self.probe.create_service(Trigger, '/dock_trigger/abort', _cb)
        self._spin_both(timeout_s=0.5)
        self._call_trigger('/safety/stop_lock_engage')
        self._spin_both(timeout_s=0.3)
        self.assertTrue(calls, 'dock_trigger ~/abort was never called')

    def test_a_goal_that_bypassed_the_proxy_via_goal_pose_is_still_cancelled_and_blocks_resume(
            self):
        # panthera-momagdii's #90 second review, the safety-critical finding:
        # a goal submitted via RViz/goal_pose_nav never reaches
        # navigate_to_pose_goal_proxy_node.py at all - bt_navigator's own
        # internal self_client_ sends it directly on navigate_to_pose_real
        # (same node-level remap, navigate_to_pose.cpp:47,269-273). Before
        # the gate's cancel client/status subscription were redirected to
        # navigate_to_pose_real, Stop never reached this goal and Resume
        # could release while bt_navigator was still driving it. Standing in
        # for that goal here the same way this file's other tests stand in
        # for a real action server - a fake cancel_goal service plus a real
        # action-status sample - directly on navigate_to_pose_real. The
        # proxy's own heartbeat (published below via
        # _publish_all_clear_goal_status) truthfully reports zero
        # outstanding goals of ITS OWN the whole time, since this goal
        # never went through it - exactly like production, the ledger
        # alone would read IDLE here; only the independent action-status
        # reconciliation against navigate_to_pose_real catches this.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._publish_all_clear_goal_status()
        self._publish_dock_status('idle')
        calls = self._serve_cancel_goal('navigate_to_pose')
        self._publish_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_both(timeout_s=0.3)

        self._call_trigger('/safety/stop_lock_engage')
        self._spin_both(timeout_s=0.3)
        self.assertTrue(
            calls, 'engage never called cancel_goal on navigate_to_pose_real - a goal that '
            'bypassed the proxy via goal_pose_nav would have kept driving through Stop')

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'Resume succeeded while the bypass-path goal on navigate_to_pose_real was still '
            'EXECUTING - cancelling it is not evidence it has actually stopped')
        self.assertIn('navigate_to_pose', result.message)

        self._publish_goal_status('navigate_to_pose', [])
        self._spin_both(timeout_s=0.3)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(
            result.success,
            f'Resume stayed refused once navigate_to_pose_real genuinely went clear: '
            f'{result.message}')


class TestEngageDoesNotClearALiveLedgerEntry(NodeTestCase):
    """
    platform-sw#90, panthera-momagdii's review (reproducer A): engage_lock()
    discarded undock_robot's ledger entry on every call, including while a
    goal was genuinely outstanding under the SAME, still-live instance -
    fire-and-forget cancellation is not proof the goal is gone. Only a
    genuinely orphaned entry (left by a GONE instance) may ever be
    reset_producer()'d - see goal_authority.py's is_orphaned().
    """

    def test_engage_does_not_clear_an_active_undock_goal_from_the_ledger(self):
        self.node._goal_authority.notify_sent('undock_robot', instance_id='live', now_s=0.0)
        self._call_trigger('/safety/stop_lock_engage')
        self.assertEqual(
            self.node._goal_authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE,
            "engage_lock() discarded a live instance's own outstanding undock goal")

    def test_repeated_engage_while_locked_does_not_clear_it_either(self):
        self.node._goal_authority.notify_sent('undock_robot', instance_id='live', now_s=0.0)
        self._call_trigger('/safety/stop_lock_engage')
        self._call_trigger('/safety/stop_lock_engage')  # repeated, already locked
        self.assertEqual(
            self.node._goal_authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)

    def test_release_stays_refused_through_engage_until_the_real_done_arrives(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        # Real wall-clock now_s throughout, matching what the gate's own
        # _release_blockers() uses internally for the staleness check - a
        # tiny fake now_s here would read as "stale" against the gate's
        # real clock regardless of this test's actual subject.
        now_s = self.node.get_clock().now().nanoseconds / 1e9
        self.node._goal_authority.notify_sent('undock_robot', instance_id='live', now_s=now_s)
        self._call_trigger('/safety/stop_lock_engage')
        # The two LEGACY actions only, by hand - not
        # _publish_all_clear_goal_status(), which also heartbeats
        # undock_robot's ledger under its own fixed 'test-instance' id and
        # would spuriously orphan the 'live' instance this test is tracking.
        # navigate_to_pose is ledger-backed too now, but under a DIFFERENT
        # producer name/entry - a plain heartbeat for it is harmless here.
        for legacy_action in ('navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_goal_authority_heartbeat(producer='navigate_to_pose')
        self._publish_dock_status('idle')
        self._spin_both(timeout_s=0.2)

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'release succeeded although undock_robot was never confirmed done - '
            'fire-and-forget cancellation plus a heartbeat is not proof (#90 review)')
        self.assertIn('undock_robot', result.message)

        # the real result finally arrives (dock_trigger's actual cancel/
        # result callback, not engage_lock() guessing it is gone)
        now_s = self.node.get_clock().now().nanoseconds / 1e9
        self.node._goal_authority.notify_done('undock_robot', instance_id='live', now_s=now_s)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success, result.message)

    def test_engage_still_clears_a_genuine_orphan(self):
        # the positive case: a PRIOR instance left a goal outstanding and is
        # gone (orphaned) - engage is still the correct, deliberate recovery
        # trigger for THAT case, same as before this fix.
        self.node._goal_authority.notify_sent('undock_robot', instance_id='gone', now_s=0.0)
        self.node._goal_authority.heartbeat('undock_robot', instance_id='live', now_s=1.0)
        self.assertTrue(self.node._goal_authority.is_orphaned('undock_robot'))
        self._call_trigger('/safety/stop_lock_engage')
        self.assertFalse(
            self.node._goal_authority.is_orphaned('undock_robot'),
            'engage_lock() must still clear a genuine orphan (D-13/D-14: the '
            'deliberate recovery action)')


class TestHeartbeatOutstandingCountIsTakenAsTruth(NodeTestCase):
    """
    platform-sw#90, panthera-momagdii's P1 review: a gate that restarts
    between a producer's own 'sent' and 'done' never sees 'sent' on its
    fresh ledger object - only the producer's next heartbeat carries the
    truth after that. Simulated by publishing a real goal_authority_events
    heartbeat message with a nonzero outstanding count, WITHOUT ever
    publishing a 'sent' event first on this gate's own ledger - exactly
    what a restarted gate sees.
    """

    def test_heartbeat_reporting_outstanding_blocks_release(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for legacy_action in ('navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_goal_authority_heartbeat(producer='navigate_to_pose')
        self._publish_dock_status('idle')
        self._publish_to_gate(
            String, 'goal_authority_events',
            String(data='undock_robot|heartbeat|gate-restart-sim|1'))
        self._spin_both(timeout_s=0.2)

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'release succeeded despite a heartbeat reporting outstanding=1 - '
            'this ledger never saw a sent event (simulating a gate restart '
            "mid-undock), so the heartbeat itself must carry the truth")
        self.assertIn('undock_robot', result.message)

    def test_a_later_heartbeat_reporting_zero_clears_it(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for legacy_action in ('navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_goal_authority_heartbeat(producer='navigate_to_pose')
        self._publish_dock_status('idle')
        self._publish_to_gate(
            String, 'goal_authority_events',
            String(data='undock_robot|heartbeat|gate-restart-sim|1'))
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(result.success)

        self._publish_to_gate(
            String, 'goal_authority_events',
            String(data='undock_robot|heartbeat|gate-restart-sim|0'))
        self._spin_both(timeout_s=0.2)
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(result.success, result.message)


class TestHeartbeatWireFormatIsStrict(NodeTestCase):
    """
    platform-sw#90, panthera-momagdii's P2 review: a heartbeat must carry
    its outstanding count explicitly, and that count must never be
    negative - both cases are rejected fail-closed (the message is simply
    ignored, leaving any prior ledger state untouched), not silently
    applied as outstanding=0.
    """

    def test_a_legacy_three_field_heartbeat_is_rejected_fail_closed(self):
        # A producer still on the legacy (pre-fix) 3-field heartbeat form
        # during a rolling update, with a real goal outstanding, must not
        # silently clear the ledger to IDLE.
        self.node._goal_authority.notify_sent('undock_robot', instance_id='live', now_s=0.0)
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|live'))
        self._spin_both(timeout_s=0.2)
        self.assertEqual(
            self.node._goal_authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE,
            'a 3-field (legacy) heartbeat must be rejected fail-closed, not '
            'applied with outstanding=0')

    def test_a_negative_outstanding_count_is_rejected_fail_closed(self):
        self.node._goal_authority.notify_sent('undock_robot', instance_id='live', now_s=0.0)
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|live|-1'))
        self._spin_both(timeout_s=0.2)
        self.assertEqual(
            self.node._goal_authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE,
            'a negative outstanding count must be rejected fail-closed, not applied')


class TestLedgerReconciliationAcrossProducerAndGateRestart(NodeTestCase):
    """
    platform-sw#90, panthera-momagdii's P1 review: the ledger alone cannot
    recover an orphan record lost across a gate restart that follows a
    producer restart. A fresh producer instance legitimately reports zero
    of its OWN outstanding goals, reading IDLE on a fresh ledger too, even
    if the ORIGINAL goal (sent by the now-gone instance) is still executing
    on the real action server - this exercises exactly that double-restart
    sequence against a second, freshly-created gate node, with the action
    server's own retained TRANSIENT_LOCAL status sample as the one thing
    that survives both restarts.
    """

    def test_action_status_still_blocks_after_the_orphan_record_is_lost(self):
        # The real action server's own retained status - same QoS the gate
        # itself now uses - survives both the producer restart and the
        # gate restart below, since it is the PUBLISHER's own retained
        # sample, not anything either restarted process held in memory.
        status_pub = self.probe.create_publisher(
            GoalStatusArray, 'undock_robot/_action/status',
            qos_profile_action_status_default)
        active = GoalStatusArray()
        gs = GoalStatus()
        gs.status = GoalStatus.STATUS_EXECUTING
        active.status_list.append(gs)
        deadline = time.monotonic() + 2.0
        while status_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self._spin_both(0.02)
        status_pub.publish(active)
        self._spin_both(timeout_s=0.2)

        # Instance 'a' sends a goal, then the PRODUCER restarts (instance
        # 'b', outstanding=0 of its own) - gate 1 correctly orphans it.
        self.node._goal_authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        self._spin_both(timeout_s=0.2)
        self.assertTrue(self.node._goal_authority.is_orphaned('undock_robot'))

        # Now the GATE itself restarts - a fresh node, fresh ledger, no
        # memory of the orphan above. Only the retained action-status
        # sample (published above, still alive) and the producer's own
        # next heartbeat (still instance 'b', still outstanding=0 of its
        # own) are available to it.
        self.node.destroy_node()
        self.node = SafetyStopGateNode()
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._spin_both(timeout_s=0.3)  # let the new node's subscriptions discover/match
        for legacy_action in ('navigate_to_pose', 'navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_dock_status('idle')
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        self._spin_both(timeout_s=0.3)

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'release succeeded after a gate restart lost the orphan record, '
            'despite the real action server still reporting an active goal')
        self.assertIn('undock_robot', result.message)
        status_pub.destroy()

    def test_release_refused_before_the_retained_status_has_a_chance_to_arrive(self):
        # panthera-momagdii's P1 re-review: the original version of this
        # fix only checked statuses truthy-vs-not, so "never arrived yet"
        # and "will never arrive" were indistinguishable and the former
        # fell through to allow release. Reproduced here by calling
        # release IMMEDIATELY after constructing the new gate - no spin at
        # all - so even a genuinely active retained sample has had zero
        # chance to be delivered yet. The explicit discovery-grace barrier
        # must refuse regardless of real DDS timing, not rely on luck.
        self.node._goal_authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        self._spin_both(timeout_s=0.2)
        self.assertTrue(self.node._goal_authority.is_orphaned('undock_robot'))

        self.node.destroy_node()
        self.node = SafetyStopGateNode()
        self.node._start -= Duration(seconds=T_READY + 1.0)
        for legacy_action in ('navigate_to_pose', 'navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_dock_status('idle')
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        # Reset the grace-period clock to "now" right before release,
        # deterministically - the setup above (3 legacy publishes, each
        # waiting on its own subscription match) takes real, variable wall
        # time that is not bounded under full-suite CPU load, so relying on
        # total elapsed-since-construction staying under the grace period
        # is inherently flaky. This proves the barrier itself blocks when
        # the window has NOT elapsed, independent of how long setup took.
        self.node._constructed_at_s = self.node.get_clock().now().nanoseconds / 1e9

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'release succeeded immediately after a gate restart, before the '
            'discovery-grace period elapsed - an active retained sample '
            'would have had no chance to arrive yet')
        self.assertIn('undock_robot', result.message)

    def test_a_late_publisher_match_gets_its_own_fresh_grace_window(self):
        # panthera-momagdii's #90 second review: the original fix anchored
        # the whole grace period to node construction, so slow discovery
        # (busy executor, loaded CI runner) could let the window elapse
        # BEFORE a genuinely active retained sample ever got a chance to be
        # delivered - a release that only ever checked elapsed-since-
        # construction would have wrongly gone through here. Reproduced by
        # backdating _constructed_at_s far past the grace window FIRST,
        # then only standing up undock_robot's status publisher afterward -
        # discovery of it (and this test's grace window) can only start
        # from that later moment, never from the (already long expired)
        # construction time.
        #
        # Deliberately no notify_sent/instance-mismatch dance here (unlike
        # TestLedgerReconciliationAcrossProducerAndGateRestart's other
        # tests): that marks the ledger entry `orphaned`, and state()
        # returns UNKNOWN unconditionally for an orphaned entry regardless
        # of now_s - the ledger branch would then block on its own, above,
        # and this test would never actually reach the discovery-grace
        # code it means to exercise. A single fresh heartbeat establishes
        # a clean IDLE with no orphan, so release falls through to the
        # status-reconciliation branch below, which is what this test is
        # about.
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        self._spin_both(timeout_s=0.2)
        self.assertFalse(self.node._goal_authority.is_orphaned('undock_robot'))
        # navigate_to_pose is ledger-backed too (not legacy) - heartbeat it
        # cleanly so its own discovery-grace/reconciliation never surfaces
        # as a blocker here and masks whether undock_robot specifically
        # blocked, which is all this test cares about.
        self._publish_goal_authority_heartbeat(producer='navigate_to_pose')
        for legacy_action in ('navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_dock_status('idle')

        # Construction-anchored window is already long over by now.
        self.node._constructed_at_s -= (_LEDGER_STATUS_DISCOVERY_GRACE_S + 5.0)

        # The real server's status publisher only shows up now - standing
        # in for the slow-discovery case this fix exists for.
        status_pub = self.probe.create_publisher(
            GoalStatusArray, 'undock_robot/_action/status',
            qos_profile_action_status_default)
        deadline = time.monotonic() + 2.0
        while status_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self._spin_both(0.02)
        # Deliberately no publish() here: proving the match itself (not
        # yet having delivered any sample) still starts a fresh window,
        # not that a construction-anchored deadline had already passed.

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            'release succeeded right as undock_robot\'s status publisher was '
            'first discovered - a fresh grace window anchored to that match '
            'must still apply, independent of how long ago construction was')
        self.assertIn('undock_robot', result.message)
        status_pub.destroy()

    def test_release_succeeds_once_the_grace_window_elapses_even_if_no_sample_ever_arrives(self):
        # panthera-momagdii's #90 sixth review, P1: an earlier round of this
        # fix (914a2dd) made this branch block INDEFINITELY once a publisher
        # was matched but had sent no sample, reasoning that a known-to-
        # exist publisher owes evidence before being trusted. That broke
        # D-13 test 1: a real rclpy action server (bt_navigator) publishes
        # NO status at all until its first goal state change (rclcpp_action
        # Jazzy server.cpp:609,:687; Nikolai measured 0 status messages in
        # 12s on an idle stack) - "matched but no sample" is the PERMANENT
        # state on a genuine clean boot, not a transient race, so blocking
        # forever there made clean-boot Resume impossible. Reverted: once
        # the (publisher-match-anchored) grace window elapses, trust the
        # ledger's own IDLE read, same as the bounded-wait shape this
        # branch had at `f482cfe`. The actual remaining race (the ledger
        # read itself being trustworthy) is closed at the proxy instead -
        # see NavigateToPoseGoalProxy's own heartbeat-suppression fix and
        # D13TestOneCleanBootResumeTestCase in
        # test_navigate_to_pose_goal_proxy_node.py, which is D-13's own
        # "test 1" with real action servers end to end.
        self._publish_to_gate(
            String, 'goal_authority_events', String(data='undock_robot|heartbeat|b|0'))
        self._spin_both(timeout_s=0.2)
        self.assertFalse(self.node._goal_authority.is_orphaned('undock_robot'))
        self._publish_goal_authority_heartbeat(producer='navigate_to_pose')
        for legacy_action in ('navigate_through_poses', 'dock_robot'):
            self._publish_goal_status(legacy_action, [])
        self._publish_dock_status('idle')

        # Real server exists (its status publisher is up) but - standing in
        # for a genuinely idle real action server, which never publishes
        # anything at all until its first goal - never sends a sample.
        status_pub = self.probe.create_publisher(
            GoalStatusArray, 'undock_robot/_action/status',
            qos_profile_action_status_default)
        deadline = time.monotonic() + 2.0
        while status_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self._spin_both(0.02)

        # Force the match to be recorded now, then wait past the grace
        # window - deterministic, not real elapsed wall time.
        self._spin_both(timeout_s=0.1)
        now_s = self.node.get_clock().now().nanoseconds / 1e9
        self.node._status_publisher_matched_at['undock_robot'] = (
            now_s - (_LEDGER_STATUS_DISCOVERY_GRACE_S + 1.0))

        result = self._call_trigger('/safety/stop_lock_release')
        self.assertTrue(
            result.success,
            'release stayed refused after the grace window elapsed even though no active '
            f'goal was ever reported - D-13 test 1 requires Resume here: {result.message}')
        status_pub.destroy()


class TestStopStatePublishing(NodeTestCase):
    """/safety/stop_state (platform-sw#119, interfaces#35)."""

    def _latest_state(self, timeout_s=0.5):
        received = []
        self.probe.create_subscription(StopState, '/safety/stop_state', received.append, 10)
        self._spin_both(timeout_s=timeout_s)
        self.assertTrue(received, 'no /safety/stop_state message received')
        return received[-1]

    def test_locked_by_default(self):
        state = self._latest_state()
        self.assertEqual(state.state, StopState.STATE_LOCKED)

    def test_running_once_released(self):
        self._release_now()
        state = self._latest_state()
        self.assertEqual(state.state, StopState.STATE_RUNNING)

    def test_release_refused_reports_a_reason(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        # deliberately leave every condition unclear - nothing published.
        self._call_trigger('/safety/stop_lock_release')
        state = self._latest_state()
        self.assertEqual(state.state, StopState.STATE_RELEASE_REFUSED)
        self.assertTrue(state.reason)

    def test_refusal_reason_persists_across_ticks_until_the_next_accepted_release(self):
        # A UI that reloads mid-refusal must still see why, not just whoever
        # happened to be listening at the moment of the refused call.
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._call_trigger('/safety/stop_lock_release')  # refused
        state1 = self._latest_state()
        self.assertEqual(state1.state, StopState.STATE_RELEASE_REFUSED)
        state2 = self._latest_state()
        self.assertEqual(state2.state, StopState.STATE_RELEASE_REFUSED)
        self.assertEqual(state2.reason, state1.reason)

    def test_refusal_clears_on_a_fresh_engage(self):
        self.node._start -= Duration(seconds=T_READY + 1.0)
        self._call_trigger('/safety/stop_lock_release')  # refused
        self.assertEqual(self._latest_state().state, StopState.STATE_RELEASE_REFUSED)
        self._call_trigger('/safety/stop_lock_engage')
        state = self._latest_state()
        self.assertEqual(state.state, StopState.STATE_LOCKED)
        self.assertEqual(state.reason, '')


class TestReleaseReadinessIsRelativeToStartup(NodeTestCase):
    """
    T_READY under wall clock (platform-sw#90 review, Alex, BLOCKER 2).

    _on_release compares self.get_clock().now() directly against T_READY,
    an ABSOLUTE clock reading. Under wall-clock (the real robot's default),
    now() is seconds since the Unix epoch - always far past T_READY=1.0s -
    so the check is satisfied unconditionally from the first instant the
    node exists and protects nothing there. It only ever does anything
    under sim time pinned near zero.

    Expected fix: record the start time in __init__ and compare ELAPSED
    time since then, not an absolute reading - so a release called within
    the first T_READY seconds of the node's own life is refused regardless
    of which clock is running.
    """

    def test_release_refused_immediately_after_construction_under_wall_clock(self):
        # No sim time involved - this is wall-clock, which is what real
        # hardware runs by default. The call happens within milliseconds of
        # __init__ in setUp(), well inside T_READY=1.0s of the node's own
        # uptime.
        result = self._call_trigger('/safety/stop_lock_release')
        self.assertFalse(
            result.success,
            "release succeeded within T_READY of the gate node's own "
            'startup under wall-clock time - the readiness check is '
            'comparing against an absolute clock reading instead of '
            'elapsed time since construction, so it protects nothing on '
            'real hardware (platform-sw#90 review, Alex, BLOCKER 2)')
        self.assertTrue(self.node._locked)


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

    # Release with no /clock ever received is refused since #90 (release needs ROS
    # time >= T_ready), so the old release-and-forward test for that case no longer
    # describes the gate. The refusal is covered by TestReleaseRefusedWithNoClock
    # and the positive forwarding by TestReadyReleaseForwards in
    # test_sim_clock_readiness_full_chain.py.

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
