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

Future-dated stamps (platform-sw#35, Mohamed's review relayed via Parth):
`_is_stale()` used to check only age_s > STALE_AFTER_S, so a stamp claiming
to be AHEAD of this node's clock produced a negative age_s, which is never
greater than STALE_AFTER_S - any future-dated command passed as "fresh" no
matter how far ahead it claimed to be. A command this far out of the
expected domain (clock skew gone wrong, a corrupted stamp, a replayed or
forged message) is exactly as untrustworthy as a stale one and is now
rejected the same way. A small FUTURE_TOLERANCE_S absorbs ordinary clock
skew between co-located processes (NOT network latency between the browser
and this node - that only ever makes a message look OLDER, never younger)
so genuinely fresh traffic does not flap on sub-millisecond jitter.

This still does not resolve the broader cross-repo contract Mohamed asked
for: whether the browser's own wall-clock stamp can be trusted at all
(his stated preference is stamping at the robot-side ROS publisher/bridge
instead), explicit stale/session/replay rejection on the browser-to-bridge
path, or behavior across a clock change, a delayed-delivery/reconnect
queue, or a real-vs-sim clock mismatch. Those need the contract record
linked and matching UI-side tests before this interface is frozen - not
something this file alone can settle.
"""
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

STALE_AFTER_S = 0.25

# Absorbs ordinary clock skew between co-located processes only - never
# network latency (that only makes a stamp look older, not younger). A
# stamp this far ahead of this node's own clock is out-of-domain and is
# rejected exactly like a too-old one. See the future-dated-stamps note
# in the module docstring.
FUTURE_TOLERANCE_S = 0.01


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
        age_s = now_s - stamp_s
        # age_s > STALE_AFTER_S: too old. age_s < -FUTURE_TOLERANCE_S: a
        # future-dated stamp beyond ordinary clock-skew slack - just as
        # out-of-domain as a stale one (platform-sw#35).
        return age_s > STALE_AFTER_S or age_s < -FUTURE_TOLERANCE_S


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
