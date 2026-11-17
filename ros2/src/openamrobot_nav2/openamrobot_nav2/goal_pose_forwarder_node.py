"""Forwarder for /goal_pose (RViz "2D Goal Pose", the UI's map-click goal).

Platform-sw#90/#124, Alex's 14 Nov owner-decision review on head 162a19a.

Nikolai's 14 Nov Gazebo reproduction on this PR's own head confirmed the defect this replaces:
navigation_launch.py remaps bt_navigator's internal goal_pose topic to goal_pose_nav, and the
nav-only profile's old forwarder (openamrobot_bringup/goal_relay.launch.py, a plain
`topic_tools relay /goal_pose -> /goal_pose_nav`) fed that straight into bt_navigator's own
internal self_client_ - a goal sent this way never reaches the PUBLIC navigate_to_pose action
navigate_to_pose_goal_proxy_node.py owns, so the UI's own status/cancel on the public name saw
nothing, and a cancel reached a server holding no such goal (cancelled_goals=0) while the robot
kept driving. This node is the real fix: every /goal_pose message becomes one NavigateToPose
goal on the PUBLIC action, through the proxy, the same path any other navigate_to_pose client
uses - the proxy's own cancel/release handling then applies to UI goals exactly as it already
does to everything else.

Contract (Alex's review, verbatim points, narrow adapter, NAV-ONLY PROFILE ONLY - the docking
helper's own /goal_pose path is explicitly out of scope here, moved to #124 with the dock
proxy):
  - One active goal: a new pose cancels the previous one first, then sends the new one. The
    old goal ends in a terminal state (through the proxy's own cancel handling, unchanged).
  - While Stop is active (StopState != RUNNING), the goal is rejected before ever reaching the
    public action - not queued, not retried, not replayed once Resume happens. Only a fresh
    /goal_pose message after Resume sends anything.
  - Rejection, cancel and failure are all logged here; whether the goal itself was ever
    accepted/cancelled/succeeded is visible on the public action's own status topic, same as
    any other navigate_to_pose client - this node adds no status topic of its own.
  - Restart-safe by construction: _current_goal_handle starts at None and nothing here
    persists state across a restart, so a fresh process tracks and sends nothing until its
    own first /goal_pose message.
  - A goal already running on the real server stays cancellable through the gate exactly as
    today (safety_stop_gate_node.py's own engage-cancels-all-four-actions logic is unchanged
    and unaware this node exists - it already cancels navigate_to_pose_real directly).

navigation_launch.py's own remap of bt_navigator's internal goal_pose topic moves from
goal_pose_nav (nobody publishes there in the nav-only profile once this node replaces the old
relay) to goal_pose_bt_navigator_unused, a name nothing publishes to at all, in EITHER profile -
see that file's own comment for why this does not touch the docking path's current behavior.
"""
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

from openamr_nav_msgs.msg import StopState

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_PUBLIC_ACTION_NAME = 'navigate_to_pose'
_GOAL_POSE_TOPIC = 'goal_pose'


class GoalPoseForwarder(Node):
    """Forward /goal_pose onto the public navigate_to_pose action; see module docstring."""

    def __init__(self):
        """Stand up the stop_state/goal_pose subscriptions and the public action client."""
        super().__init__('goal_pose_forwarder')
        # Fail-closed default (platform-sw#90 convention, same reasoning as the goal-owning
        # proxies' own ledger-stale-means-UNKNOWN logic): until this node's own stop_state
        # subscription delivers a retained sample, treat Stop as active and send nothing,
        # rather than forwarding a goal before this node actually knows the gate's state.
        self._stop_state = StopState.STATE_LOCKED
        self._current_goal_handle = None
        # Mohamed's #90 review on 31dcd09, P2 (item 3): a second goal_pose can arrive while
        # the first's send_goal_async is still pending (_current_goal_handle is still None,
        # since _on_goal_response has not run yet), so cancelling only _current_goal_handle
        # misses it and both end up active. _goal_generation increments on every accepted
        # goal_pose; _on_goal_response compares against it once the send resolves and cancels
        # immediately if a newer goal_pose has since superseded it, instead of relying on
        # Nav2's own preemption for a case this node's own "one active goal" contract already
        # promises to guarantee.
        self._goal_generation = 0

        cb_group = ReentrantCallbackGroup()
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            StopState, '/safety/stop_state', self._on_stop_state, state_qos,
            callback_group=cb_group)
        self._client = ActionClient(
            self, NavigateToPose, _PUBLIC_ACTION_NAME, callback_group=cb_group)
        self.create_subscription(
            PoseStamped, _GOAL_POSE_TOPIC, self._on_goal_pose, 10, callback_group=cb_group)

    def _on_stop_state(self, msg: StopState) -> None:
        self._stop_state = msg.state

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self._stop_state != StopState.STATE_RUNNING:
            self.get_logger().warning(
                'goal_pose received while Stop is active (state=%d) - rejected, not queued '
                'or retried; send a fresh goal_pose after Resume' % self._stop_state)
            return

        # Mohamed's #90 review, P2 (item 2): send_goal_async() against a server that is not
        # up yet never answers at all - a goal sent in that window was silently lost, with
        # none of the rejection/failure logging the module docstring promises. Checked and
        # logged here, same as every other rejection path in this node.
        if not self._client.server_is_ready():
            self.get_logger().warning(
                f'{_PUBLIC_ACTION_NAME} action server not ready - goal_pose rejected, not '
                'queued or retried; send a fresh goal_pose once it is up')
            return

        # One active goal: cancel whatever this node itself last sent before sending the new
        # one, fire-and-forget (same convention the goal-owning proxies' own cancel paths
        # use) - the old goal's own terminal transition happens through the proxy's existing
        # cancel handling, unchanged by this node.
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        # See _goal_generation's own comment: this goal_pose's generation is recorded now,
        # before the send even resolves, so _on_goal_response can tell whether it is still
        # the newest one by the time acceptance comes back.
        self._goal_generation += 1
        generation = self._goal_generation

        goal = NavigateToPose.Goal()
        goal.pose = msg
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, generation))

    def _on_goal_response(self, future, generation) -> None:
        goal_handle = future.result()
        if generation != self._goal_generation:
            # A further goal_pose arrived and was accepted before THIS goal even finished
            # being accepted - cancel it immediately instead of leaving two goals active
            # (Mohamed's #90 review, P2, item 3).
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warning(
                f'{_PUBLIC_ACTION_NAME} rejected the forwarded goal_pose goal - not retrying')
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(goal_handle, f))

    def _on_result(self, goal_handle, future) -> None:
        # Only clear the tracked handle if a NEWER goal has not already replaced it (a new
        # goal_pose can arrive and send before this goal's own result comes back).
        if self._current_goal_handle is goal_handle:
            self._current_goal_handle = None
        result_response = future.result()
        self.get_logger().info(
            f'{_PUBLIC_ACTION_NAME} goal reached terminal status {result_response.status}')


def main():
    """Entry point for the goal_pose_forwarder_node console script."""
    rclpy.init()
    node = GoalPoseForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
