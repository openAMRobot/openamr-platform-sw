#!/usr/bin/env python3
"""
Unit-level tests for goal_pose_forwarder_node.py (platform-sw#90, Mohamed's review on 31dcd09).

Against a fake navigate_to_pose action server, same technique
test_navigate_to_pose_goal_proxy_node.py uses for its own proxy - not the full
navigation_launch.py/tf2_ros stack test_goal_pose_forwarder_full_chain.py needs, since neither
property tested here depends on a real bt_navigator, TF or the gate's own stop_state topic.
"""
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90

from openamr_nav_msgs.msg import StopState  # noqa: E402,I100

from openamrobot_nav2.goal_pose_forwarder_node import GoalPoseForwarder  # noqa: E402

from rclpy.action import ActionServer, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402


class _FakeNavigateToPose:
    """Stands in for navigate_to_pose_goal_proxy_node.py's own public action server."""

    def __init__(self, node, accept=True):
        self.accepted_goals = []
        self.cancel_requested = threading.Event()
        self._accept = accept
        self._finish = threading.Event()
        self._server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())

    def _on_goal(self, _request):
        return GoalResponse.ACCEPT if self._accept else GoalResponse.REJECT

    def _on_cancel(self, _goal_handle):
        from rclpy.action import CancelResponse
        self.cancel_requested.set()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        # Per-goal cancel check, not just the shared _finish flag: two goals can be
        # executing concurrently here (the whole point of the two-poses test below), and
        # _on_cancel's own self._finish.set() must end only the goal it was actually
        # requested against, not every goal this fake server happens to be running.
        self.accepted_goals.append(goal_handle)
        while not self._finish.is_set() and not goal_handle.is_cancel_requested:
            time.sleep(0.02)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif goal_handle.is_active:
            goal_handle.succeed()
        return NavigateToPose.Result()

    def finish_now(self):
        self._finish.set()

    def destroy(self):
        self._server.destroy()


class GoalPoseForwarderNodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.server_node = rclpy.create_node('fake_navigate_to_pose_server')
        self.forwarder = GoalPoseForwarder()
        self.probe = rclpy.create_node('goal_pose_forwarder_node_probe')
        self.executor = MultiThreadedExecutor()
        for n in (self.server_node, self.forwarder, self.probe):
            self.executor.add_node(n)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()
        self.goal_pose_pub = self.probe.create_publisher(PoseStamped, 'goal_pose', 10)

    def tearDown(self):
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.goal_pose_pub.destroy()
        self.forwarder.destroy_node()
        self.server_node.destroy_node()
        self.probe.destroy_node()

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.02)

    def _release(self):
        state_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.probe.create_publisher(StopState, '/safety/stop_state', state_qos)
        deadline = time.monotonic() + 2.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        pub.publish(StopState(state=StopState.STATE_RUNNING))
        self._spin_for(0.2)
        pub.destroy()

    def _publish_goal_pose(self, x):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        self.goal_pose_pub.publish(pose)

    def test_goal_pose_before_the_server_is_ready_is_rejected_not_lost(self):
        # Mohamed's #90 review on 31dcd09, P2, item 2: send_goal_async() against a server
        # that is not up never answers at all - before the fix, a goal_pose sent before
        # navigate_to_pose exists was silently lost with no log and no record anywhere.
        # Here, no _FakeNavigateToPose is ever started, so the public action never comes
        # up at all. _current_goal_handle staying None is not by itself proof of anything -
        # it also stays None on the OLD code, which hangs the send forever instead of
        # rejecting it, for the same reason. The real claim is that send_goal_async is
        # never even attempted once the forwarder has checked readiness - asserted directly
        # against the ActionClient's own pending-goal bookkeeping, not inferred.
        self._release()
        self.assertEqual(
            len(self.forwarder._client._pending_goal_requests), 0,
            'test precondition: no goal should be outstanding before the publish below')
        self._publish_goal_pose(1.0)
        self._spin_for(1.0)
        self.assertEqual(
            len(self.forwarder._client._pending_goal_requests), 0,
            'a goal_pose sent before the server exists must never reach send_goal_async at '
            'all - a goal left outstanding here means the send was attempted and is now '
            'hung forever waiting on a server that will never answer, exactly the silent '
            'loss this check exists to prevent')

    def test_two_poses_inside_the_accept_delay_leave_exactly_one_goal_active(self):
        # Mohamed's #90 review on 31dcd09, P2, item 3: _current_goal_handle is still None
        # for the first goal_pose until its own send_goal_async resolves - a second
        # goal_pose arriving in that exact window saw no handle to cancel and both goals
        # could end up active. Delays BOTH the server's goal acceptance and this test's own
        # publish of the second pose so the window is deterministic, not raced.
        fake = _FakeNavigateToPose(self.server_node)
        try:
            deadline = time.monotonic() + 5.0
            while not self.forwarder._client.server_is_ready() and time.monotonic() < deadline:
                self._spin_for(0.05)
            self.assertTrue(self.forwarder._client.server_is_ready())
            self._release()

            self._publish_goal_pose(1.0)
            self._publish_goal_pose(2.0)  # arrives well inside the first's own accept delay

            def _two_goals_reached_the_server():
                return len(fake.accepted_goals) >= 2
            deadline = time.monotonic() + 10.0
            while not _two_goals_reached_the_server() and time.monotonic() < deadline:
                self._spin_for(0.05)
            self.assertTrue(
                _two_goals_reached_the_server(),
                'both goal_pose messages must still reach the server (nothing dropped)')

            def _exactly_one_goal_remains_active():
                active = [g for g in fake.accepted_goals if g.is_active]
                return len(active) == 1
            deadline = time.monotonic() + 10.0
            while not _exactly_one_goal_remains_active() and time.monotonic() < deadline:
                self._spin_for(0.05)
            active = [g for g in fake.accepted_goals if g.is_active]
            self.assertEqual(
                len(active), 1,
                f'expected exactly one active goal after two poses inside the accept delay, '
                f'found {len(active)} - the first must be cancelled once its own acceptance '
                'arrives, not left running alongside the second')
        finally:
            fake.finish_now()
            self._spin_for(0.3)
            fake.destroy()


if __name__ == '__main__':
    unittest.main()
