#!/usr/bin/env python3
"""
Goal-owning proxy for navigate_through_poses (platform-sw#124, D-13).

Same architecture as navigate_to_pose_goal_proxy_node.py, built as its own
fast-follow (that file's own module docstring named this explicitly:
"navigate_through_poses and dock_robot are NOT yet covered by this pattern -
same architecture, tracked as a fast-follow"). The approved STOP-RELEASE
decision (platform-sw#124, P-03 rev20.6 item 30, platform-lead ruling A1)
requires "positive idle evidence from every enabled motion producer, through
ONE platform node that owns the navigation and dock goal handles" -
navigate_through_poses was left on the legacy action-status-content path,
which cannot distinguish "clean boot, zero goals ever sent" from "never
heard from" - the exact ambiguity D-13 exists to close.

This node closes that gap for navigate_through_poses specifically, without
any change needed in any existing sender: navigation_launch.py remaps
bt_navigator's real action server to navigate_through_poses_real and this
node claims the public navigate_through_poses name instead - every existing
sender keeps calling the same name it already does and transparently routes
through here. Every accepted goal is relayed to the real server end to end
(goal, feedback, result, cancellation), and reported to the goal-authority
ledger via the same wire protocol (goal_authority_events) dock_trigger.py
and navigate_to_pose_goal_proxy_node.py already use - heartbeat every 2.0s
carrying this node's own current outstanding count, sent right before
relaying, done once the real goal's own result arrives.

Cancellation forwarding is the one safety-critical property here: a cancel
accepted on this node's own (proxy) goal handle must reach the REAL goal on
bt_navigator, not just mark the wrapper cancelled while the real goal keeps
executing - see _handle_cancel and its own regression test.

Unlike navigate_to_pose, stock Nav2's NavigateThroughPosesNavigator has no
documented topic-triggered internal self_client_ analogous to the
goal_pose/self_client_ bypass navigate_to_pose has (RViz's "2D Goal Pose"
tool is single-pose only) - so there is no known equivalent bypass path for
this action. safety_stop_gate_node.py still redirects its cancel client and
status subscription to navigate_through_poses_real regardless, for the same
defensive symmetry as navigate_to_pose and because the node-level remap in
navigation_launch.py would catch any such client if one existed.

dock_robot is NOT yet covered by this pattern - same architecture, tracked
as a further fast-follow (needs #125's docking_server work first).
"""
import threading
import uuid

from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateThroughPoses

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

_REAL_ACTION_NAME = 'navigate_through_poses_real'
_PUBLIC_ACTION_NAME = 'navigate_through_poses'
_PRODUCER_NAME = 'navigate_through_poses'
_HEARTBEAT_PERIOD_S = 2.0  # matches dock_trigger.py's own heartbeat cadence


class NavigateThroughPosesGoalProxy(Node):
    """Owns every navigate_through_poses goal handle; see module docstring."""

    def __init__(self):
        """Stand up the public action server and the real-server client."""
        super().__init__('navigate_through_poses_goal_proxy')
        self._instance_id = uuid.uuid4().hex
        self._outstanding = 0
        # panthera-momagdii's #90 seventh review, P1 (same race as
        # navigate_to_pose_goal_proxy_node.py): the heartbeat timer and the
        # goal-execution callbacks all run under this node's own
        # MultiThreadedExecutor/ReentrantCallbackGroup, genuinely
        # concurrently - _heartbeat() reads/publishes self._outstanding as
        # an authoritative SNAPSHOT (GoalAuthority.heartbeat() overwrites
        # the ledger's count with it), while 'sent'/'done' are DELTA events
        # (+1/-1). Without this lock, a heartbeat can read _outstanding
        # strictly between _execute's own decrement and its 'done' publish,
        # publishing an already-decremented snapshot; the gate then applies
        # that snapshot AND the delta from the 'done' event that follows,
        # decrementing twice for one real completion. Every mutation of
        # _outstanding and every publish derived from it happens inside
        # this lock, as one atomic unit.
        self._lock = threading.Lock()
        self._real_goal_handles = {}  # proxy goal uuid bytes -> real ClientGoalHandle
        # Same race as navigate_to_pose_goal_proxy_node.py's P2: a cancel can
        # arrive after acceptance but before send_goal_async resolves and
        # _real_goal_handles is populated - tracked here and replayed the
        # instant the real handle appears, instead of being silently
        # dropped. Same eighth-review P2 as that sibling file too: reading
        # _real_goal_handles.get(key) in _handle_cancel and installing it in
        # _execute are two separate steps - without the SAME lock around
        # both, a cancel could observe no real handle yet, _execute could
        # then install the handle and observe no pending cancel yet, and
        # both sides would finish believing the other handles it - the
        # cancel is silently lost. self._lock also guards every access to
        # _real_goal_handles/_pending_cancels for this reason.
        self._pending_cancels = set()  # proxy goal uuid bytes
        cb_group = ReentrantCallbackGroup()

        self._goal_authority_pub = self.create_publisher(String, 'goal_authority_events', 10)
        self._real_client = ActionClient(
            self, NavigateThroughPoses, _REAL_ACTION_NAME, callback_group=cb_group)
        self._server = ActionServer(
            self, NavigateThroughPoses, _PUBLIC_ACTION_NAME,
            execute_callback=self._execute,
            handle_accepted_callback=self._handle_accepted,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group)
        self.create_timer(_HEARTBEAT_PERIOD_S, self._heartbeat, callback_group=cb_group)
        # Same as navigate_to_pose_goal_proxy_node.py (panthera-momagdii's
        # #90 sixth review, P1): D-13 makes this proxy the clean-boot
        # evidence for navigate_through_poses - fire one heartbeat right
        # away instead of waiting for the first 2.0s timer tick.
        self._heartbeat()

    def _heartbeat(self) -> None:
        # Same reasoning as navigate_to_pose_goal_proxy_node.py: a heartbeat
        # claiming "zero outstanding" is only boot evidence if the real
        # server this proxy relays to actually exists - if
        # navigate_through_poses_real is not ready, silence here is
        # correct and lets the gate's own stale_after timeout age this
        # producer back to UNKNOWN on its own.
        if not self._real_client.server_is_ready():
            return
        with self._lock:
            self._goal_authority_pub.publish(String(
                data=f'{_PRODUCER_NAME}|heartbeat|{self._instance_id}|{self._outstanding}'))

    def _handle_accepted(self, goal_handle) -> None:
        # Same P1 race as navigate_to_pose_goal_proxy_node.py: the public
        # ActionServer accepts a goal before _execute (an independently-
        # scheduled coroutine) ever runs - handle_accepted_callback runs
        # synchronously as part of the server's own acceptance sequence,
        # before goal_handle.execute() schedules _execute at all, so there
        # is no gap here for the heartbeat to interleave into.
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
        # self._lock's own comment above.
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
        # _handle_accepted - not here, to avoid double-counting.
        try:
            if not self._real_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    f'{_REAL_ACTION_NAME} action server not available - aborting')
                if goal_handle.is_active:
                    goal_handle.abort()
                return NavigateThroughPoses.Result()

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
                return NavigateThroughPoses.Result()

            key = bytes(goal_handle.goal_id.uuid)
            # panthera-momagdii's #90 eighth review, P2: install and the
            # pending-cancel check must be one atomic step against
            # _handle_cancel's own lookup-or-add.
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
                    # navigate_through_poses_real directly) - canceled() is
                    # only a valid transition if a cancel was actually
                    # requested and accepted on this goal handle; calling it
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
    """Entry point for the navigate_through_poses_goal_proxy_node console script."""
    rclpy.init()
    node = NavigateThroughPosesGoalProxy()
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
