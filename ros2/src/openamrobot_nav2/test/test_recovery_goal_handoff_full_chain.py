"""
Live, integrated regression for platform-sw#127 (Mohamed's 2026-11-08 comment).

The two unit-level tests in test_status_trackers.py
(test_a_cancelled_goal_s_trailing_primary_failure_can_leak_into_the_next_goal,
test_a_late_new_goal_reset_can_erase_the_new_goal_s_own_first_failure) replay
the race as direct Python calls on a bare RecoveryTracker. Mohamed asked for
those replayed through the real messaging layer instead: real goal UUIDs on
/navigate_to_pose/_action/status, real nav2_msgs/BehaviorTreeLog events on
/behavior_tree_log, delivered to the real NavigationStatusNode over actual
rclpy pub/sub, in both delivery orders.

Mohamed's 2026-11-08 15:33 review on this exact file (first version): a
fixed-duration spin after publish proves nothing fired. In particular
test_live_new_goal_s_own_first_failure_can_be_erased_by_a_late_reset's only
final assertion was assertNotEqual(reason, RECOVERY_LIMIT_REACHED) - an
untouched tracker starts at NONE, so that assertion also passes if no
message was ever delivered. _deliver_until below replaces the fixed sleep
with a bounded wait that FAILS the test if the expected intermediate state
is not observed, and every stage asserts concrete progress
(self.node._task.goal_id for the status topic, self.node._recovery's own
counters for the BT log) before the next stage runs, including an explicit
positive-delivery control in the second test.

This is still a single-process test (no Gazebo, no real bt_navigator) - it
does not reproduce Nav2's own internal scheduling or real network jitter.
What it does add over the unit-level replay: the two topics are independent
rclpy publishers with their own QoS and discovery, the node consumes them
through its real subscription callbacks (not a direct method call), and
both goals carry real 16-byte unique_identifier_msgs/UUID values matched
through actual action_msgs/GoalInfo fields - confirming the race is a
property of the message contract (nav2_msgs/BehaviorTreeStatusChange has no
goal-correlating field at all: timestamp, node_name, uid [a BT node id, not
a goal id], previous_status, current_status) and not an artifact of the
simpler unit-test harness.

Needs a real rclpy and openamr_nav_msgs (openamrobot-interfaces); skipped
without either.
"""
import time
import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')
rclpy = pytest.importorskip('rclpy')

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray  # noqa: E402,I100
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange  # noqa: E402
from openamr_nav_msgs.msg import NavigationStatus  # noqa: E402
from openamrobot_nav2.navigation_status_node import NavigationStatusNode  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402


def _uuid(n):
    return UUID(uuid=list(bytes([n]) * 16))


def _goal_status_array(n, status=GoalStatus.STATUS_EXECUTING, sec=10):
    return GoalStatusArray(status_list=[
        GoalStatus(
            goal_info=GoalInfo(
                goal_id=_uuid(n), stamp=TimeMsg(sec=sec, nanosec=0)),
            status=status)])


def _bt_event(node_name, current_status, previous_status='IDLE', uid=1):
    return BehaviorTreeStatusChange(
        timestamp=TimeMsg(sec=0, nanosec=0), node_name=node_name, uid=uid,
        previous_status=previous_status, current_status=current_status)


def _bt_log(*events):
    return BehaviorTreeLog(event_log=list(events))


class RecoveryGoalHandoffFullChain(unittest.TestCase):
    """Real pub/sub between a probe node and the real NavigationStatusNode."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = NavigationStatusNode()
        self.probe = rclpy.create_node('recovery_handoff_probe')
        self._nav_status_pub = self.probe.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        self._bt_log_pub = self.probe.create_publisher(
            BehaviorTreeLog, '/behavior_tree_log', 10)

    def tearDown(self):
        self.node.destroy_node()
        self.probe.destroy_node()

    def _deliver_until(self, pub, msg, predicate, description, timeout_s=3.0):
        """
        Publish over real rclpy and wait for observable proof it arrived.

        Mohamed's review: a fixed-duration spin never confirms the
        subscriber's callback actually ran, so a silently-undelivered
        message and a correctly-processed one look identical to the test.
        This fails loudly instead of continuing on a guess.
        """
        pub.publish(msg)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail(
            f'{description}: not observed within {timeout_s}s - the message '
            'was published but its callback-side effect never appeared')

    def _deliver_status(self, n, **kwargs):
        self._deliver_until(
            self._nav_status_pub, _goal_status_array(n, **kwargs),
            lambda: self.node._task.goal_id == bytes([n]) * 16,
            f'goal {n} status was processed (self.node._task.goal_id updated)')

    def _deliver_bt_failure(self, expected_primary_failures, goal_started=False):
        events = []
        if goal_started:
            events.append(_bt_event('NavigateRecovery', 'RUNNING', previous_status='IDLE'))
        events.append(_bt_event('NavigateWithReplanning', 'FAILURE'))
        self._deliver_until(
            self._bt_log_pub, _bt_log(*events),
            lambda: self.node._recovery._primary_failures == expected_primary_failures,
            f'BT failure was processed (_primary_failures reached {expected_primary_failures})')

    def _deliver_bt_recovery_failure(self):
        # NavigateRecovery FAILURE always reassigns _exhausted to True or
        # False, whichever the outcome - force it to an impossible sentinel
        # first so a changed value is unambiguous proof the callback ran,
        # independent of which of the two outcomes this test expects
        self.node._recovery._exhausted = None
        self._deliver_until(
            self._bt_log_pub, _bt_log(_bt_event('NavigateRecovery', 'FAILURE')),
            lambda: self.node._recovery._exhausted is not None,
            'NavigateRecovery FAILURE was processed')

    def test_live_cancel_then_requeue_lets_a_stray_failure_inflate_the_next_goal(self):
        # goal A (uuid 1): deep into recovery over real BT log messages, not
        # yet exhausted - each step's intermediate count is itself the proof
        # of delivery, not a side observation
        self._deliver_status(1)
        for i in range(1, 6):
            self._deliver_bt_failure(i)
        self.assertEqual(self.node._recovery._primary_failures, 5)

        # goal B (uuid 2) is sent and its status reaches the node first -
        # delivered as a real, separate GoalStatusArray message. Its own
        # status-topic reset is what zeroes A's count - prove that reset
        # itself fired, not just that the publish call returned.
        self._deliver_until(
            self._nav_status_pub, _goal_status_array(2),
            lambda: (self.node._task.goal_id == bytes([2]) * 16
                     and self.node._recovery._primary_failures == 0),
            "goal 2's status was processed AND reset A's primary-failure count")

        # A's last in-flight primary tick, delivered late relative to B's
        # own status - still just "NavigateWithReplanning FAILURE" on
        # /behavior_tree_log, nothing in the real message ties it to A.
        # Its delivery is proven by the count moving from 0 to 1.
        self._deliver_bt_failure(1)

        # B then fails 6 more times entirely on its own (counts 2 through 7)
        for i in range(2, 8):
            self._deliver_bt_failure(i)

        self._deliver_bt_recovery_failure()
        self.assertEqual(
            self.node._recovery.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            "exhausted one failure early over real pub/sub: A's stray tick "
            "counted toward B's budget")

    def test_live_new_goal_s_own_first_failure_can_be_erased_by_a_late_reset(self):
        # goal B's BT run starts and already records one genuine failure of
        # its own, over real /behavior_tree_log messages - delivery proven by
        # the count actually reaching 1, not assumed from a fixed sleep
        self._deliver_bt_failure(1, goal_started=True)

        # positive-delivery control (Mohamed's review): confirm the status
        # message is actually processed, and that it actually erases the
        # count just proven above, not merely that nothing contradicts a
        # default NONE reason
        self._deliver_until(
            self._nav_status_pub, _goal_status_array(2),
            lambda: (self.node._task.goal_id == bytes([2]) * 16
                     and self.node._recovery._primary_failures == 0),
            "goal 2's own late status was processed AND erased its own first failure")

        # B fails 6 more times on its own (counts 1 through 6); it should
        # exhaust at 7 but the late reset erased the first one, so it never
        # reaches 7
        for i in range(1, 7):
            self._deliver_bt_failure(i)
        self.assertEqual(
            self.node._recovery._primary_failures, 6,
            "delivery control: exactly 6 of B's own failures were counted, "
            'confirming the erased first one is the only missing count')

        self._deliver_bt_recovery_failure()
        self.assertNotEqual(
            self.node._recovery.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            "one failure short of exhaustion over real pub/sub: B's own "
            'first failure was erased by its own late-delivered reset')


if __name__ == '__main__':
    unittest.main(verbosity=2)
