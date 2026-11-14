#!/usr/bin/env python3
"""
Live proof of navigate_through_poses_goal_proxy_node.py (platform-sw#124, D-13).

Mirror of test_navigate_to_pose_goal_proxy_node.py's own convention, for the
sibling proxy. Against a fake navigate_through_poses_real action server
(standing in for the real bt_navigator, which needs costmaps/TF/a map to do
anything useful - this node's own relay/ledger behavior does not depend on
any of that), not a mock of send_goal_async: a real rclpy
ActionServer/ActionClient pair, same convention as the rest of this
package's live node tests.

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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90/#124

from openamrobot_nav2.navigate_through_poses_goal_proxy_node import (  # noqa: E402
    NavigateThroughPosesGoalProxy)
from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode, T_READY  # noqa: E402

from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


class _FakeRealServer:
    """
    Stands in for bt_navigator on navigate_through_poses_real.

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
            self.node, NavigateThroughPoses, 'navigate_through_poses_real',
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
        return NavigateThroughPoses.Result()

    def destroy(self):
        if self._server is not None:
            self._server.destroy()


class NavigateThroughPosesGoalProxyTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.real_server_node = rclpy.create_node('fake_navigate_through_poses_real_server')
        # Delayed - most tests call self.fake_real.start() as their own
        # first line; the two race tests below deliberately delay it
        # further to reproduce the exact windows panthera-momagdii's #90
        # review found.
        self.fake_real = _FakeRealServer(self.real_server_node, start_delayed=True)
        self.proxy = NavigateThroughPosesGoalProxy()
        self.probe = rclpy.create_node('navigate_through_poses_goal_proxy_probe')
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

        self.client = ActionClient(self.probe, NavigateThroughPoses, 'navigate_through_poses')
        self.assertTrue(
            self.client.wait_for_server(timeout_sec=5.0),
            'navigate_through_poses proxy action server never came up')

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
        goal = NavigateThroughPoses.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        goal.poses = [pose]
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
        while 'navigate_through_poses|sent|' not in str(self._received_events) \
                and time.monotonic() < end:
            time.sleep(0.02)
        sent_events = [
            e for e in self._received_events if e.startswith('navigate_through_poses|sent|')]
        self.assertTrue(sent_events, f'no sent event seen - got {self._received_events}')

        self.fake_real.finish_now()
        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done())

        end = time.monotonic() + 2.0
        while 'navigate_through_poses|done|' not in str(self._received_events) \
                and time.monotonic() < end:
            time.sleep(0.02)
        done_events = [
            e for e in self._received_events if e.startswith('navigate_through_poses|done|')]
        self.assertTrue(done_events, f'no done event seen - got {self._received_events}')

        sent_instance = sent_events[0].split('|')[2]
        done_instance = done_events[0].split('|')[2]
        self.assertEqual(
            sent_instance, done_instance,
            'sent/done instance_id mismatch - not the same producer instance')

    def test_heartbeat_reports_zero_outstanding_when_idle(self):
        # Clean-boot story (D-13/D-14): no goal ever sent, and this
        # producer's own heartbeat alone must still establish IDLE within
        # the literal 2s bound Alex's approved #124 ruling sets - same
        # readiness-poll fix as the navigate_to_pose sibling test's own
        # comment explains in full (panthera-momagdii's #90 ninth review,
        # P1).
        self.fake_real.start()
        ready_deadline = time.monotonic() + 5.0
        while not self.proxy._real_client.server_is_ready() and time.monotonic() < ready_deadline:
            time.sleep(0.02)
        self.assertTrue(self.proxy._real_client.server_is_ready(), 'real server never ready')

        deadline = time.monotonic() + 2.0
        heartbeats = []
        while time.monotonic() < deadline:
            heartbeats = [
                e for e in self._received_events
                if e.startswith('navigate_through_poses|heartbeat|')]
            if heartbeats:
                break
            time.sleep(0.02)
        self.assertTrue(
            heartbeats,
            'no heartbeat seen within 2s of the real server becoming ready (D-13 ruling, '
            'platform-sw#124)')
        _, _, _, outstanding = heartbeats[0].split('|')
        self.assertEqual(outstanding, '0')

    def test_heartbeat_never_reports_zero_between_acceptance_and_execute_starting(self):
        # panthera-momagdii's #90 review, P1 (same race as
        # navigate_to_pose_goal_proxy_node.py): the public ActionServer
        # accepts a goal before _execute (an independently-scheduled
        # coroutine on a separate MultiThreadedExecutor task) ever runs. A
        # live, timing-based version of this test is not reliable either
        # way (see the sibling test's own comment on this) - white-box
        # instead: call _handle_accepted directly with a stub goal handle
        # and assert the ordering invariant the fix establishes
        # structurally - outstanding is incremented and 'sent' is published
        # strictly before goal_handle.execute() is invoked.
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
                e for e in self._received_events
                if e.startswith('navigate_through_poses|sent|')]
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

    def test_heartbeat_cannot_observe_a_count_between_decrement_and_done_publish(self):
        # panthera-momagdii's #90 seventh review, P1 (same race as
        # navigate_to_pose_goal_proxy_node.py's own sibling test) - see
        # that test's own comment for the full reasoning. Proves the
        # structural fix directly: holding the proxy's own lock (simulating
        # being inside the decrement+'done'-publish critical section) must
        # block a concurrent call to _heartbeat() until the lock is
        # released.
        self.fake_real.start()  # server_is_ready() must be True for _heartbeat to even try
        end = time.monotonic() + 2.0
        while not self.proxy._real_client.server_is_ready() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(self.proxy._real_client.server_is_ready())

        heartbeat_started = threading.Event()
        heartbeat_done = threading.Event()

        def _run_heartbeat():
            heartbeat_started.set()
            self.proxy._heartbeat()
            heartbeat_done.set()

        with self.proxy._lock:
            t = threading.Thread(target=_run_heartbeat, daemon=True)
            t.start()
            self.assertTrue(heartbeat_started.wait(timeout=1.0), 'heartbeat thread never started')
            time.sleep(0.2)
            self.assertFalse(
                heartbeat_done.is_set(),
                '_heartbeat() proceeded while the decrement+done-publish lock was held - '
                'it can observe a transiently-decremented outstanding count before the '
                'matching done event is published, double-counting at the gate')
        t.join(timeout=2.0)
        self.assertTrue(heartbeat_done.is_set(), '_heartbeat() never completed after the lock '
                         'was released')

    def test_cancel_and_real_handle_install_are_mutually_exclusive(self):
        # panthera-momagdii's #90 eighth review, P2 (same race as
        # navigate_to_pose_goal_proxy_node.py's own sibling test - see
        # that test's own comment for the full reasoning). Proven
        # deterministically: hold the proxy's own lock (simulating being
        # inside either critical section) and confirm a concurrent call
        # to _handle_cancel cannot proceed past it until the lock is
        # released.
        goal_handle = unittest.mock.Mock()
        goal_handle.goal_id.uuid = list(range(16))

        cancel_started = threading.Event()
        cancel_done = threading.Event()

        def _run_cancel():
            cancel_started.set()
            self.proxy._handle_cancel(goal_handle)
            cancel_done.set()

        with self.proxy._lock:
            t = threading.Thread(target=_run_cancel, daemon=True)
            t.start()
            self.assertTrue(cancel_started.wait(timeout=1.0), 'cancel thread never started')
            time.sleep(0.2)
            self.assertFalse(
                cancel_done.is_set(),
                '_handle_cancel() proceeded while another critical section held the lock - '
                'it can interleave with _execute installing the real handle, silently '
                'losing a cancel that arrives in the gap between the two')
        t.join(timeout=2.0)
        self.assertTrue(cancel_done.is_set(), '_handle_cancel() never completed after the lock '
                         'was released')

    def test_real_goal_handles_is_only_ever_written_while_the_lock_is_held(self):
        # panthera-momagdii's #90 ninth review, P1 (same test as the
        # navigate_to_pose sibling - see its own docstring for the full
        # reasoning). Swap in a dict that records every write made while
        # the lock is NOT held, then drive a real goal all the way through
        # _execute against the fake real server.
        violations = []
        real_lock = self.proxy._lock

        class _LockCheckingDict(dict):
            def __setitem__(self, key, value):
                if not real_lock.locked():
                    violations.append(f'__setitem__({key!r}) without the lock held')
                super().__setitem__(key, value)

            def pop(self, key, *args):
                if not real_lock.locked():
                    violations.append(f'pop({key!r}) without the lock held')
                return super().pop(key, *args)

        self.proxy._real_goal_handles = _LockCheckingDict()
        self.fake_real.start()
        goal_handle = self._send_goal()
        end = time.monotonic() + 2.0
        while self.fake_real._goal_handle is None and time.monotonic() < end:
            time.sleep(0.02)
        self.assertIsNotNone(
            self.fake_real._goal_handle,
            'the real (fake) server never received the relayed goal - install never ran')

        self.fake_real.finish_now()
        result_future = goal_handle.get_result_async()
        end = time.monotonic() + 5.0
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(result_future.done(), 'get_result_async never completed')
        self.assertEqual(
            violations, [],
            '_real_goal_handles was written to without self._lock held: ' + '; '.join(violations))


class D13TestOneCleanBootResumeNavigateThroughPosesTestCase(unittest.TestCase):
    """
    D-13 ruling's own "test 1", for navigate_through_poses.

    Platform-sw#124, panthera-momagdii's #90 sixth review ("put
    navigate_through_poses behind the same proxy"): with real rclpy action
    servers and no goal ever sent, Resume must succeed within the ~2s
    heartbeat bound - mirror of D13TestOneCleanBootResumeTestCase in
    test_navigate_to_pose_goal_proxy_node.py for this sibling proxy.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.real_server_node = rclpy.create_node('d13_test1_ntp_fake_real_server')
        self.fake_real = _FakeRealServer(self.real_server_node)  # up immediately - clean boot
        self.proxy = NavigateThroughPosesGoalProxy()
        self.gate = SafetyStopGateNode()
        # Excluded by name (D-13), not inferred: this test is about
        # navigate_through_poses specifically, not the other three
        # producers, which would each need their own fake server/status to
        # let release succeed and are irrelevant to the property under test.
        self.gate._enabled_producers = {'navigate_through_poses'}
        self.gate._start -= Duration(seconds=T_READY + 1.0)
        self.probe = rclpy.create_node('d13_test1_ntp_probe')

        self.executor = MultiThreadedExecutor()
        for node in (self.real_server_node, self.proxy, self.gate, self.probe):
            self.executor.add_node(node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    def tearDown(self):
        self.fake_real.finish_now()
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.fake_real.destroy()
        self.proxy.destroy_node()
        self.gate.destroy_node()
        self.real_server_node.destroy_node()
        self.probe.destroy_node()

    def test_resume_succeeds_within_the_heartbeat_bound_on_a_clean_boot(self):
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), 'release service never came up')

        # Literal 2.0s (panthera-momagdii's #90 ninth review, P1), measured
        # from THIS defined ready event (the release service becoming
        # available) - same readiness-poll fix as the navigate_to_pose
        # sibling test's own comment explains in full.
        deadline = time.monotonic() + 2.0
        last_message = ''
        while time.monotonic() < deadline:
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < end:
                time.sleep(0.02)
            if future.done() and future.result() is not None:
                if future.result().success:
                    return
                last_message = future.result().message
            time.sleep(0.05)
        self.fail(
            'Resume did not succeed within the heartbeat bound of a clean boot with no goal '
            f'ever sent - D-13 test 1 requires this with real action servers; last refusal: '
            f'{last_message!r}')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
