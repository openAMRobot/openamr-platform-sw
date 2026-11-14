#!/usr/bin/env python3
"""
Goal-owning proxy for navigate_to_pose (platform-sw#124, D-13).

The approved STOP-RELEASE decision (platform-sw#124, P-03 rev20.6 item 30,
platform-lead ruling A1) requires "positive idle evidence from every enabled
motion producer, through ONE platform node that owns the navigation and dock
goal handles" - undock_robot already has this (dock_trigger.py's own
heartbeat/sent/done wiring to goal_authority_events). navigate_to_pose did
not: it was left on the legacy action-status-content path, which cannot
distinguish "clean boot, zero goals ever sent" from "never heard from" -
the exact ambiguity D-13 exists to close - and (BotshareAI's platform-sw#90
review, 11 Nov; panthera-momagdii's confirmation) nothing stopped a client
other than dock_trigger.py from submitting an untracked goal directly to
bt_navigator's own action server, since nothing OWNED that goal handle.

This node closes both gaps for navigate_to_pose specifically, without any
change needed in dock_trigger.py or openamrobot-ui: navigation_launch.py
remaps bt_navigator's real action server to navigate_to_pose_real and this
node claims the public navigate_to_pose name instead - every existing
sender (dock_trigger.py's own ActionClient, openamrobot-ui's direct
ActionClient) keeps calling the same name it already does and transparently
routes through here. Every accepted goal is relayed to the real server
end to end (goal, feedback, result, cancellation), and reported to the
goal-authority ledger via the same wire protocol (goal_authority_events)
dock_trigger.py already uses - heartbeat every 2.0s carrying this node's own
current outstanding count (D-13: "within 2s" clean-boot Resume bound comes
from this same heartbeat period), sent right before relaying, done once the
real goal's own result arrives.

Cancellation forwarding is the one safety-critical property here: a cancel
accepted on this node's own (proxy) goal handle must reach the REAL goal on
bt_navigator, not just mark the wrapper cancelled while the real goal keeps
executing - see _handle_cancel and its own regression test.

navigate_through_poses is now covered by the same pattern -
navigate_through_poses_goal_proxy_node.py. dock_robot is NOT yet covered -
same architecture, tracked as a further fast-follow (needs #125's
docking_server work first).
"""
import threading
import uuid

from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

_REAL_ACTION_NAME = 'navigate_to_pose_real'
_PUBLIC_ACTION_NAME = 'navigate_to_pose'
_PRODUCER_NAME = 'navigate_to_pose'
_HEARTBEAT_PERIOD_S = 2.0  # matches dock_trigger.py's own heartbeat cadence


class NavigateToPoseGoalProxy(Node):
    """Owns every navigate_to_pose goal handle; see module docstring."""

    def __init__(self):
        """Stand up the public action server and the real-server client."""
        super().__init__('navigate_to_pose_goal_proxy')
        self._instance_id = uuid.uuid4().hex
        self._outstanding = 0
        # panthera-momagdii's #90 seventh review, P1: the heartbeat timer
        # and the goal-execution callbacks all run under this node's own
        # MultiThreadedExecutor/ReentrantCallbackGroup, genuinely
        # concurrently (not just interleaved coroutines) - _heartbeat()
        # reads/publishes self._outstanding as an authoritative SNAPSHOT
        # (GoalAuthority.heartbeat() overwrites the ledger's count with it),
        # while 'sent'/'done' are DELTA events (+1/-1). Without this lock, a
        # heartbeat can read _outstanding strictly between _execute's own
        # decrement and its 'done' publish, publishing an
        # already-decremented snapshot; the gate then applies that snapshot
        # AND the delta from the 'done' event that follows, decrementing
        # twice for one real completion. Every mutation of _outstanding and
        # every publish derived from it happens inside this lock, as one
        # atomic unit, so a heartbeat can never observe a count from
        # between a decrement and its own matching event's publish.
        self._lock = threading.Lock()
        self._real_goal_handles = {}  # proxy goal uuid bytes -> real ClientGoalHandle
        # panthera-momagdii's #90 review, P2: a cancel can arrive after
        # acceptance but before send_goal_async resolves and
        # _real_goal_handles is populated - tracked here and replayed the
        # instant the real handle appears, instead of being silently
        # dropped. panthera-momagdii's #90 eighth review, P2: reading
        # _real_goal_handles.get(key) in _handle_cancel and installing it in
        # _execute are two separate steps - without the SAME lock around
        # both the read-or-add in _handle_cancel and the install-and-check
        # in _execute, a cancel could observe no real handle yet, _execute
        # could then install the handle and observe no pending cancel yet
        # (since the cancel had not added it), and both sides would finish
        # believing the other one will handle it - the cancel is silently
        # lost and the real goal keeps running. self._lock (already used
        # for the outstanding/heartbeat race below) also guards every
        # access to _real_goal_handles/_pending_cancels for this reason.
        self._pending_cancels = set()  # proxy goal uuid bytes
        cb_group = ReentrantCallbackGroup()

        self._goal_authority_pub = self.create_publisher(String, 'goal_authority_events', 10)
        self._real_client = ActionClient(
            self, NavigateToPose, _REAL_ACTION_NAME, callback_group=cb_group)
        self._server = ActionServer(
            self, NavigateToPose, _PUBLIC_ACTION_NAME,
            execute_callback=self._execute,
            handle_accepted_callback=self._handle_accepted,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group)
        self.create_timer(_HEARTBEAT_PERIOD_S, self._heartbeat, callback_group=cb_group)
        # panthera-momagdii's #90 sixth review, P1: D-13 makes this proxy
        # the clean-boot evidence for navigate_to_pose ("that node knows
        # zero goals active, including at boot") - waiting for the first
        # 2.0s timer tick before ever heartbeating left a real window where
        # the gate had no evidence at all yet, and the gate's own bounded
        # wait (safety_stop_gate_node.py's _release_blockers) depends on
        # this heartbeat being immediate. Fire one right away.
        self._heartbeat()
        # panthera-momagdii's #90 ninth review, P1: the timer above has its
        # own phase set here, at construction, and the attempt just above
        # is a no-op whenever navigate_to_pose_real is not ready yet (the
        # usual case) - without this poll, the first SUCCESSFUL heartbeat
        # then has to wait for that timer's own next 2.0s tick, which can
        # land up to a full period after the real server actually becomes
        # ready, missing D-13's literal "within 2s" clean-boot bound. Poll
        # readiness on a short period instead and fire the moment it turns
        # true; self-cancels once it has, since the regular periodic timer
        # above is sufficient from then on.
        self._server_ready_poll_timer = self.create_timer(
            0.1, self._poll_for_first_heartbeat, callback_group=cb_group)

    def _poll_for_first_heartbeat(self) -> None:
        # panthera-momagdii's #90 review on d29dfc0: this used to check
        # server_is_ready() itself, then call _heartbeat(), which checks
        # server_is_ready() AGAIN - if the real server flickered not-ready
        # strictly between those two separate checks, _heartbeat() would
        # return having published nothing, but the poll cancelled itself
        # regardless, leaving this producer with no fast-path heartbeat at
        # all until the regular 2.0s periodic timer's next tick - exactly
        # the D-13 "within 2s" bound this poll exists to protect. Fixed by
        # removing the redundant duplicate check entirely: _heartbeat() is
        # now the single source of truth on whether it actually published,
        # and the poll only cancels on that same answer - there is no
        # longer a second, separate check for the real server to flicker
        # between.
        if not self._heartbeat():
            return
        self._server_ready_poll_timer.cancel()

    def _heartbeat(self) -> bool:
        # panthera-momagdii's #90 sixth review, P1: a heartbeat claiming
        # "zero outstanding" is only boot evidence if the real server this
        # proxy relays to actually exists - if navigate_to_pose_real is not
        # ready (bt_navigator absent/not yet up), silence here is correct:
        # D-13 requires an absent server to read UNKNOWN and block, not a
        # false "idle" from a proxy that has nothing real to report on.
        # Skipping lets the gate's own stale_after timeout age this
        # producer back to UNKNOWN on its own. Returns whether a heartbeat
        # was actually published, so _poll_for_first_heartbeat can tell a
        # real send apart from this early return.
        if not self._real_client.server_is_ready():
            return False
        with self._lock:
            self._goal_authority_pub.publish(String(
                data=f'{_PRODUCER_NAME}|heartbeat|{self._instance_id}|{self._outstanding}'))
        return True

    def _handle_accepted(self, goal_handle) -> None:
        # panthera-momagdii's #90 review, P1: the public ActionServer
        # accepts a goal (and could be queried via GoalAuthority/the
        # heartbeat timer) before _execute, an independently-scheduled
        # coroutine under this node's MultiThreadedExecutor/
        # ReentrantCallbackGroup, ever gets to run - a heartbeat firing in
        # that gap would report outstanding=0 with an accepted goal already
        # sitting there. handle_accepted_callback runs synchronously as
        # part of the server's own acceptance sequence, before
        # goal_handle.execute() schedules _execute at all, so there is no
        # gap here for the heartbeat to interleave into.
        with self._lock:
            self._outstanding += 1
            self._goal_authority_pub.publish(String(
                data=f'{_PRODUCER_NAME}|sent|{self._instance_id}'))
        goal_handle.execute()

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        # Forward immediately, fire-and-forget (same convention
        # safety_stop_gate_node.py's own _cancel_all_goals uses): the real
        # goal's own result, not this call's response, is what the ledger's
        # 'done' event and this proxy's own goal_handle.canceled()/succeed()
        # ultimately depend on - never assumed from the cancel reply alone.
        key = bytes(goal_handle.goal_id.uuid)
        # panthera-momagdii's #90 eighth review, P2: the lookup here and
        # the install in _execute must be mutually exclusive - see
        # self._lock's own comment above. Whichever side gets the lock
        # first decides correctly: if this runs first, the handle is
        # genuinely not installed yet, so pending_cancels is the right
        # place; if _execute's install runs first, the handle is already
        # there and this cancels it directly instead of ever touching
        # pending_cancels - either order, exactly one of the two paths
        # below fires, never neither.
        with self._lock:
            real_handle = self._real_goal_handles.get(key)
            if real_handle is None:
                # Either the real goal was never sent (this goal is still
                # between acceptance and send_goal_async resolving), or it
                # has already finished - replay it in _execute if the
                # former turns out to be true; harmless if the goal is
                # already gone, since _execute's own finally clause always
                # discards this.
                self._pending_cancels.add(key)
        if real_handle is not None:
            real_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        # Outstanding count and the 'sent' event are handled in
        # _handle_accepted (panthera-momagdii's #90 review, P1) - not here,
        # to avoid double-counting.
        try:
            if not self._real_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    f'{_REAL_ACTION_NAME} action server not available - aborting')
                if goal_handle.is_active:
                    goal_handle.abort()
                return NavigateToPose.Result()

            def _feedback_cb(feedback_msg):
                if goal_handle.is_active:
                    goal_handle.publish_feedback(feedback_msg.feedback)

            send_future = self._real_client.send_goal_async(
                goal_handle.request, feedback_callback=_feedback_cb,
                goal_uuid=goal_handle.goal_id)
            real_goal_handle = await send_future
            if not real_goal_handle.accepted:
                if goal_handle.is_active:
                    goal_handle.abort()
                return NavigateToPose.Result()

            key = bytes(goal_handle.goal_id.uuid)
            # panthera-momagdii's #90 eighth review, P2: install and the
            # pending-cancel check must be one atomic step against
            # _handle_cancel's own lookup-or-add - see self._lock's own
            # comment and _handle_cancel's.
            with self._lock:
                self._real_goal_handles[key] = real_goal_handle
                replay_cancel = key in self._pending_cancels
                if replay_cancel:
                    self._pending_cancels.discard(key)
            if replay_cancel:
                # A cancel arrived before this real handle existed - replay
                # it now instead of having silently dropped it in
                # _handle_cancel.
                real_goal_handle.cancel_goal_async()
            try:
                result_response = await real_goal_handle.get_result_async()
            finally:
                with self._lock:
                    self._real_goal_handles.pop(key, None)
                    self._pending_cancels.discard(key)

            if goal_handle.is_active:
                if result_response.status == GoalStatus.STATUS_CANCELED:
                    # The real goal can end up CANCELED through a path that
                    # never went through THIS wrapper's own cancel_callback
                    # (e.g. safety_stop_gate_node.py cancelling
                    # navigate_to_pose_real directly) - canceled() is only a
                    # valid transition if a cancel was actually requested
                    # and accepted on this goal handle; calling it
                    # otherwise raises. abort() reports the same practical
                    # outcome (goal did not succeed) without that
                    # precondition.
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                elif result_response.status == GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            return result_response.result
        finally:
            with self._lock:
                self._outstanding = max(0, self._outstanding - 1)
                self._goal_authority_pub.publish(String(
                    data=f'{_PRODUCER_NAME}|done|{self._instance_id}'))


def main():
    """Entry point for the navigate_to_pose_goal_proxy_node console script."""
    rclpy.init()
    node = NavigateToPoseGoalProxy()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
