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

    def _publish_and_deliver(self, pub, msg, settle_s=0.3):
        pub.publish(msg)
        # real delivery through rclpy, not a direct method call - spin both
        # ends until the subscription's callback has actually run
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _fail_primary_via_bt_log(self, times, goal_started=False):
        for i in range(times):
            events = []
            if goal_started and i == 0:
                events.append(_bt_event('NavigateRecovery', 'RUNNING', previous_status='IDLE'))
            events.append(_bt_event('NavigateWithReplanning', 'FAILURE'))
            self._publish_and_deliver(self._bt_log_pub, _bt_log(*events))

    def test_live_cancel_then_requeue_lets_a_stray_failure_inflate_the_next_goal(self):
        # goal A (uuid 1): deep into recovery over real BT log messages, not
        # yet exhausted
        self._publish_and_deliver(self._nav_status_pub, _goal_status_array(1))
        self._fail_primary_via_bt_log(5)

        # goal B (uuid 2) is sent and its status reaches the node first -
        # delivered as a real, separate GoalStatusArray message
        self._publish_and_deliver(self._nav_status_pub, _goal_status_array(2))

        # A's last in-flight primary tick, delivered late relative to B's
        # own status - still just "NavigateWithReplanning FAILURE" on
        # /behavior_tree_log, nothing in the real message ties it to A
        self._fail_primary_via_bt_log(1)

        # B then fails 6 more times entirely on its own, and NavigateRecovery
        # gives up
        self._fail_primary_via_bt_log(6)
        self._publish_and_deliver(
            self._bt_log_pub, _bt_log(_bt_event('NavigateRecovery', 'FAILURE')))

        self.assertEqual(
            self.node._recovery.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            "exhausted one failure early over real pub/sub: A's stray tick "
            "counted toward B's budget")

    def test_live_new_goal_s_own_first_failure_can_be_erased_by_a_late_reset(self):
        # goal B's BT run starts and already records one genuine failure of
        # its own, over real /behavior_tree_log messages
        self._fail_primary_via_bt_log(1, goal_started=True)

        # B's own status-topic "new goal" notification, delivered late
        # relative to the BT log - a real, separate GoalStatusArray message
        # for the SAME goal id, not a different one
        self._publish_and_deliver(self._nav_status_pub, _goal_status_array(2))

        # B fails 6 more times on its own; it should exhaust (7 total) but
        # the late reset erased the first one
        self._fail_primary_via_bt_log(6)
        self._publish_and_deliver(
            self._bt_log_pub, _bt_log(_bt_event('NavigateRecovery', 'FAILURE')))

        self.assertNotEqual(
            self.node._recovery.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            "one failure short of exhaustion over real pub/sub: B's own "
            'first failure was erased by its own late-delivered reset')


if __name__ == '__main__':
    unittest.main(verbosity=2)
