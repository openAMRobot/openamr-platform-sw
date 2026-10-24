#!/usr/bin/env python3
"""
Relay from the UI's stamped manual-drive topic to twist_mux's manual input.

Nik's Gate A read fixes the interface: "UI publishes TwistStamped on
/ui/cmd_vel; the relay drops commands older than 0.25 s and feeds the mux
manual input" - cmd_vel_joy, per twist_mux.yaml's manual topic (this
supersedes Nik's earlier /cmd_vel_manual placeholder name). This node is
that relay. It is also where the mux's use_stamped: false decision actually
holds: every input the mux sees is unstamped geometry_msgs/Twist, so the one
stamped source in the whole graph (the UI) is converted here, once, rather
than making the mux negotiate stamped and unstamped inputs at the same time
(not supported in twist_mux 4.5.0 - see twist_mux.yaml's header comment).

Staleness (the "stale UI commands" case in both Mohamed's and Nik's
reviews): a message is forwarded only if `now - header.stamp <= STALE_AFTER`
at the moment it is RECEIVED, not against whenever it happens to be
processed - there is no buffering of rejected messages and no queue replay:
the subscription QoS depth is 1, so a connectivity gap does not build up a
backlog that then gets flushed as a burst of stale-but-queued commands once
the link recovers; each arriving message is judged independently against
the clock at receipt time, so a late-delivered old message is dropped on its
own terms the same way a merely-slow one would be, and never "applied late"
once fresh data resumes - there is nothing here to replay it from.

use_sim_time is honoured (self.get_clock().now()), so the staleness check
reads the UI message's stamp against this node's own clock, which also
suggests a limit of this design worth being explicit about on the cross-repo
contract Nik asked for: the UI and this relay need to share the SAME clock
source (both on use_sim_time false in the real world, or both on the same
/clock in sim) for the 0.25 s window to mean the same thing on both ends.
"""
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

STALE_AFTER_S = 0.25


class UiCmdVelRelayNode(Node):

    def __init__(self):
        super().__init__('ui_cmd_vel_relay_node')
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.create_subscription(TwistStamped, '/ui/cmd_vel', self._on_ui_cmd_vel, sub_qos)

    def _on_ui_cmd_vel(self, msg: TwistStamped):
        if self._is_stale(msg):
            return
        self._pub.publish(msg.twist)

    def _is_stale(self, msg: TwistStamped) -> bool:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        return (now_s - stamp_s) > STALE_AFTER_S


def main(args=None):
    rclpy.init(args=args)
    node = UiCmdVelRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
