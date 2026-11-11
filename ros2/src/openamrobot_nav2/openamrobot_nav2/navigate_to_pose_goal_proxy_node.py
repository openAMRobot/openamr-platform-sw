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

navigate_through_poses and dock_robot are NOT yet covered by this pattern -
same architecture, tracked as a fast-follow, not attempted here to keep
this node reviewable on its own.
"""
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
        self._real_goal_handles = {}  # proxy goal uuid bytes -> real ClientGoalHandle
        # panthera-momagdii's #90 review, P2: a cancel can arrive after
        # acceptance but before send_goal_async resolves and
        # _real_goal_handles is populated - tracked here and replayed the
        # instant the real handle appears, instead of being silently
        # dropped.
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

    def _heartbeat(self) -> None:
        self._goal_authority_pub.publish(String(
            data=f'{_PRODUCER_NAME}|heartbeat|{self._instance_id}|{self._outstanding}'))

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
        real_handle = self._real_goal_handles.get(key)
        if real_handle is not None:
            real_handle.cancel_goal_async()
        else:
            # Either the real goal was never sent (this goal is still
            # between acceptance and send_goal_async resolving), or it has
            # already finished - replay it in _execute if the former turns
            # out to be true; harmless if the goal is already gone, since
            # _execute's own finally clause always discards this.
            self._pending_cancels.add(key)
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
            self._real_goal_handles[key] = real_goal_handle
            if key in self._pending_cancels:
                # panthera-momagdii's #90 review, P2: a cancel arrived
                # before this real handle existed - replay it now instead
                # of having silently dropped it in _handle_cancel.
                self._pending_cancels.discard(key)
                real_goal_handle.cancel_goal_async()
            try:
                result_response = await real_goal_handle.get_result_async()
            finally:
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
