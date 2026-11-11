#!/usr/bin/env python3
"""
Live proof of navigate_to_pose_goal_proxy_node.py (platform-sw#124, D-13).

Against a fake navigate_to_pose_real action server (standing in for the real
bt_navigator, which needs costmaps/TF/a map to do anything useful - this
node's own relay/ledger behavior does not depend on any of that), not a
mock of send_goal_async: a real rclpy ActionServer/ActionClient pair, same
convention as the rest of this package's live node tests.

The single most safety-critical property tested here is cancellation
forwarding (test_cancelling_the_proxy_goal_cancels_the_real_goal): a cancel
accepted on the proxy's own goal handle must reach the real action server,
not just mark the wrapper cancelled while the real goal keeps executing.
"""
import threading
import time
import unittest
import unittest.mock

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90/#124

from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from openamrobot_nav2.navigate_to_pose_goal_proxy_node import (  # noqa: E402
    NavigateToPoseGoalProxy)


class _FakeRealServer:
    """
    Stands in for bt_navigator on navigate_to_pose_real.

    Holds every accepted goal open (never completes it on its own) until the
    test explicitly finishes or cancels it, so cancellation-forwarding has
    something real to observe: if the proxy's own cancel never reaches here,
    this goal simply never ends.

    start_delayed=True defers standing up the ActionServer itself until
    start() is called - used to reproduce the exact window between the
    proxy accepting a goal and the real server existing at all (panthera-
    momagdii's #90 review, P1/P2 races), deterministically rather than by
    racing real timing.
    """

    def __init__(self, node, start_delayed=False):
        self.node = node
        self.cancel_requested = threading.Event()
        self._goal_handle = None
        self._finish = threading.Event()
        self._cb_group = ReentrantCallbackGroup()
        self._server = None
        if not start_delayed:
            self.start()

    def start(self):
        self._server = ActionServer(
            self.node, NavigateToPose, 'navigate_to_pose_real',
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group)

    def _on_cancel(self, goal_handle):
        del goal_handle
        self.cancel_requested.set()
        self._finish.set()
        return CancelResponse.ACCEPT

    def finish_now(self):
        self._finish.set()

    def _execute(self, goal_handle):
        self._goal_handle = goal_handle
        while not self._finish.wait(timeout=0.05):
            pass
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif goal_handle.is_active:
            goal_handle.succeed()
        return NavigateToPose.Result()

    def destroy(self):
        if self._server is not None:
            self._server.destroy()


class NavigateToPoseGoalProxyTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.real_server_node = rclpy.create_node('fake_navigate_to_pose_real_server')
        # Delayed - most tests call self.fake_real.start() as their own
        # first line; the two race tests below deliberately delay it
        # further to reproduce the exact windows panthera-momagdii's #90
        # review found.
        self.fake_real = _FakeRealServer(self.real_server_node, start_delayed=True)
        self.proxy = NavigateToPoseGoalProxy()
        self.probe = rclpy.create_node('navigate_to_pose_goal_proxy_probe')
        self._received_events = []
        self.probe.create_subscription(
            String, 'goal_authority_events',
            lambda msg: self._received_events.append(msg.data), 10)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.real_server_node)
        self.executor.add_node(self.proxy)
        self.executor.add_node(self.probe)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        self.client = ActionClient(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(
            self.client.wait_for_server(timeout_sec=5.0),
            'navigate_to_pose proxy action server never came up')

    def tearDown(self):
        self.fake_real.finish_now()
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.client.destroy()
        self.fake_real.destroy()
        self.proxy.destroy_node()
        self.real_server_node.destroy_node()
        self.probe.destroy_node()

    def _send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        send_future = self.client.send_goal_async(goal)
        end = time.monotonic() + 5.0
        while not send_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(send_future.done(), 'send_goal_async never completed')
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted, 'proxy did not accept the goal')
        return goal_handle

    def test_a_goal_is_relayed_to_the_real_server_and_completes(self):
        self.fake_real.start()
        goal_handle = self._send_goal()
        end = time.monotonic() + 2.0
        while self.fake_real._goal_handle is None and time.monotonic() < end:
            time.sleep(0.02)
        self.assertIsNotNone(
            self.fake_real._goal_handle,
            'the real (fake) server never received the relayed goal')

        self.fake_real.finish_now()
        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done(), 'get_result_async never completed')
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_SUCCEEDED)

    def test_cancelling_the_proxy_goal_cancels_the_real_goal(self):
        # The safety-critical property: a cancel on the WRAPPER must reach
        # the REAL goal, not just mark the wrapper cancelled while the real
        # one keeps executing underneath.
        self.fake_real.start()
        goal_handle = self._send_goal()
        end = time.monotonic() + 2.0
        while self.fake_real._goal_handle is None and time.monotonic() < end:
            time.sleep(0.02)
        self.assertIsNotNone(self.fake_real._goal_handle)

        cancel_future = goal_handle.cancel_goal_async()
        end = time.monotonic() + 5.0
        while not cancel_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(cancel_future.done(), 'cancel_goal_async never completed')

        self.assertTrue(
            self.fake_real.cancel_requested.wait(timeout=5.0),
            'the real (fake) server never received a cancel request - the '
            'proxy did not forward it, leaving the real goal running')

        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done())
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)

    def test_goal_authority_events_bracket_the_real_goals_lifetime(self):
        self.fake_real.start()
        goal_handle = self._send_goal()
        end = time.monotonic() + 2.0
        while 'navigate_to_pose|sent|' not in str(self._received_events) \
                and time.monotonic() < end:
            time.sleep(0.02)
        sent_events = [e for e in self._received_events if e.startswith('navigate_to_pose|sent|')]
        self.assertTrue(sent_events, f'no sent event seen - got {self._received_events}')

        self.fake_real.finish_now()
        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done())

        end = time.monotonic() + 2.0
        while 'navigate_to_pose|done|' not in str(self._received_events) \
                and time.monotonic() < end:
            time.sleep(0.02)
        done_events = [e for e in self._received_events if e.startswith('navigate_to_pose|done|')]
        self.assertTrue(done_events, f'no done event seen - got {self._received_events}')

        sent_instance = sent_events[0].split('|')[2]
        done_instance = done_events[0].split('|')[2]
        self.assertEqual(
            sent_instance, done_instance,
            'sent/done instance_id mismatch - not the same producer instance')

    def test_heartbeat_reports_zero_outstanding_when_idle(self):
        # Clean-boot story (D-13/D-14): no goal ever sent, and this
        # producer's own heartbeat alone must still establish IDLE within
        # the same ~2s bound dock_trigger.py's heartbeat does.
        self.fake_real.start()
        deadline = time.monotonic() + 3.0
        heartbeats = []
        while time.monotonic() < deadline:
            heartbeats = [
                e for e in self._received_events if e.startswith('navigate_to_pose|heartbeat|')]
            if heartbeats:
                break
            time.sleep(0.05)
        self.assertTrue(heartbeats, 'no heartbeat seen within 3s of startup')
        _, _, _, outstanding = heartbeats[0].split('|')
        self.assertEqual(outstanding, '0')

    def test_heartbeat_never_reports_zero_between_acceptance_and_execute_starting(self):
        # panthera-momagdii's #90 review, P1: the public ActionServer
        # accepts a goal before _execute (an independently-scheduled
        # coroutine on a separate MultiThreadedExecutor task) ever runs.
        # A live, timing-based version of this test (send a goal, fire one
        # heartbeat, assert outstanding != 0) is NOT reliable either way:
        # verified directly against the pre-fix code (old _execute() doing
        # the increment/publish itself, no handle_accepted_callback) that
        # such a test still passes, because in practice the executor
        # schedules _execute's first two lines fast enough to win a single
        # manually-triggered heartbeat almost every time - the real gap
        # this finding describes is real but too small for a poll-based
        # test to depend on.
        #
        # White-box instead: call _handle_accepted directly with a stub
        # goal handle and assert the ordering invariant the fix
        # establishes structurally - outstanding is incremented and 'sent'
        # is published strictly before goal_handle.execute() is invoked,
        # with no async gap in between at all (same call stack, same
        # thread). The pre-fix code has no _handle_accepted method wired
        # into handle_accepted_callback, so execute() runs with no
        # ordering guarantee relative to the bookkeeping at all.
        self._received_events.clear()
        events = []
        goal_handle = unittest.mock.Mock()
        goal_handle.execute.side_effect = lambda: events.append('execute')

        self.proxy._handle_accepted(goal_handle)

        self.assertEqual(
            self.proxy._outstanding, 1,
            '_handle_accepted must increment outstanding synchronously, '
            'before goal_handle.execute() schedules _execute at all')
        deadline = time.monotonic() + 2.0
        sent_events = []
        while time.monotonic() < deadline:
            sent_events = [
                e for e in self._received_events if e.startswith('navigate_to_pose|sent|')]
            if sent_events:
                break
            time.sleep(0.02)
        self.assertTrue(sent_events, 'no sent event published by _handle_accepted')
        self.assertEqual(
            events, ['execute'],
            'goal_handle.execute() must be called, and only after the '
            'outstanding/sent bookkeeping above has already happened')

    def test_a_cancel_before_the_real_handle_exists_is_replayed_once_it_appears(self):
        # panthera-momagdii's #90 review, P2: a cancel can arrive after the
        # proxy accepted the goal but before send_goal_async completes and
        # _real_goal_handles[key] is populated. The real server is started
        # here only AFTER the cancel request below, guaranteeing that
        # window is hit deterministically rather than raced.
        goal_handle = self._send_goal()  # accepted - real server still not started

        cancel_future = goal_handle.cancel_goal_async()
        end = time.monotonic() + 2.0
        while not cancel_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(cancel_future.done(), 'cancel_goal_async never completed')

        # Only now does the real server (and therefore a real goal handle
        # for the proxy to attach the pending cancel to) come into being.
        self.fake_real.start()

        self.assertTrue(
            self.fake_real.cancel_requested.wait(timeout=5.0),
            'the real (fake) server never received a cancel request - a cancel '
            'that arrived before the real handle existed was dropped instead '
            'of being replayed once the handle appeared')

        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done(), 'get_result_async never completed')
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
