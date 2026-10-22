#!/usr/bin/env python3
"""
Safety stop gate for twist_mux's software_stop lock/input pair (issue #35).

Mohamed Sayed's review of the posted twist_mux config (platform-sw#35) raised
three properties of that config this node has to actually satisfy, not just
assert:

[Startup] "timeout: 0.0 disables lock expiry; it does not start the mux
locked. The stored Bool starts false." True of twist_mux's own internal
state - a Lock with no message ever received defaults to not-engaged, and
timeout: 0.0 only means it never expires once engaged, not that it begins
engaged. So the lock has to already read true the moment anything else in
the graph could possibly start commanding motion, before this node has
received or sent a single message:

  - self._locked is set True in __init__, before any publisher or
    subscription is created - there is no window where this node exists but
    has not yet decided it is locked.
  - the /safety/stop_lock publisher uses TRANSIENT_LOCAL durability, so a
    twist_mux that starts (or restarts) AFTER this node already has an
    engaged lock gets that retained True immediately on subscribing, before
    its first spin - durability does the "startup order" half of this for
    free, in both start orders (gate first or mux first) and across a mux
    restart specifically, since the retained sample lives with this node,
    not the mux.
  - belt-and-suspenders beyond durability alone: _tick() republishes the
    current lock value at TICK_HZ regardless of whether it changed, so even
    a QoS/transport hiccup that lost the retained sample is corrected within
    one tick period, not left to rely on durability being perfect.

[Release] "The mux directly accepts Bool messages on its configured lock
topic, and false clears that lock. A ... helper ignoring false on the same
topic[s] does not prevent the mux's direct callback from accepting it."
Correct, and this node does not try to intercept or filter that topic - a
second publisher racing this one would still reach the mux. The actual
enforcement is architectural, not a code-level interception: this node is
the ONLY thing in the graph with a publisher on /safety/stop_lock, period -
release happens by calling release_lock() from the /safety/stop_lock_release
service callback, which is the only code path that ever sets self._locked to
False. That "only publisher" property is a graph invariant, not a runtime
guarantee this node could enforce on its own (ROS topics have no ACL) - it
is checked the same way Nik's own /cmd_vel single-publisher check works:
tools/issue35_arbitration/stop_lock_invariants.py asserts
`ros2 topic info /safety/stop_lock -v` and `... cmd_vel_stop -v` each show
exactly one publisher, this node. That check belongs in the same bench run
as the A1/A2 evidence Nik asked for, not just in CI.

[Equal-priority nonzero stop] "the mux also permits a nonzero Twist from
cmd_vel_stop at [255] and forwards it unchanged." True of the mux; the fix
is making it structurally impossible for this topic to ever carry anything
but zero: this node has no subscription at all feeding into the Twist it
publishes on cmd_vel_stop - _tick() always publishes a freshly-constructed,
default-valued Twist() (every field is 0.0 by construction), never a value
derived from any input. There is no code path in this file that could put a
nonzero value there. Combined with the same sole-publisher invariant as
above, cmd_vel_stop can only ever carry zero.

[Loss of the zero publisher while locked] If this node's process dies while
locked, cmd_vel_stop and the lock's periodic republish both stop - the
retained TRANSIENT_LOCAL sample on /safety/stop_lock still holds the lock
true indefinitely (timeout: 0.0 never expires it), so the mux keeps masking
every lower-priority input; but cmd_vel_stop itself will time out at its
configured 0.5s, so the mux is left with no active input at all - which
is the same empty-mux-input the "loss of zero publisher" bench case
(platform-sw#35, Mohamed's review) exists to measure against real twist_mux
4.5.0 (that behavior - silence vs. an explicit zero - is a property of the
mux, not this node, and needs its own bench measurement, not an assumption).
The node-level mitigation for the crash itself is launch-level respawn
(navigation_launch.py, respawn=True) as the interim measure pending #75's
dedicated lifecycle manager with supervised respawn for the whole reactive
motion layer - this node is exactly the kind of thing #75 is meant to cover.

[Re-engaging the lock] Not addressed anywhere in the reviewed proposal - the
/safety/stop_lock_release service that clears it has no documented
counterpart for re-engaging it (e.g. from the UI's "Stop (software)" button,
openamrobot-ui#34). /safety/stop_lock_engage is added here as the symmetric
service, same reasoning as release being a deliberate service call rather
than a topic publish (Kunal's own proposal note) - but the UI-side wiring to
it is unconfirmed and needs the same cross-repo contract record nik asked
for on the release/relay interface, not assumed from this file alone.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

TICK_HZ = 10.0


class SafetyStopGateNode(Node):

    def __init__(self):
        super().__init__('safety_stop_gate_node')
        # Locked before anything else in this constructor runs - see the
        # [Startup] note above. Nothing downstream of this line can observe
        # this node in an unlocked state before it has decided it is locked.
        self._locked = True

        lock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lock_pub = self.create_publisher(Bool, '/safety/stop_lock', lock_qos)
        self._stop_pub = self.create_publisher(Twist, 'cmd_vel_stop', 10)

        self.create_service(Trigger, '/safety/stop_lock_release', self._on_release)
        self.create_service(Trigger, '/safety/stop_lock_engage', self._on_engage)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        # Publish the initial state immediately, not just on the first timer
        # tick - the retained TRANSIENT_LOCAL sample exists from the first
        # possible instant this node could be observed by anything else.
        self._publish_lock()

    def release_lock(self) -> bool:
        """Clear the lock. Returns False if it was already clear (idempotent, informative)."""
        if not self._locked:
            return False
        self._locked = False
        self._publish_lock()
        return True

    def engage_lock(self) -> bool:
        """Set the lock. Returns False if it was already set (idempotent, informative)."""
        if self._locked:
            return False
        self._locked = True
        self._publish_lock()
        return True

    def _on_release(self, request, response):
        del request  # Trigger has no request fields
        changed = self.release_lock()
        response.success = True
        response.message = 'released' if changed else 'already released'
        return response

    def _on_engage(self, request, response):
        del request
        changed = self.engage_lock()
        response.success = True
        response.message = 'engaged' if changed else 'already engaged'
        return response

    def _publish_lock(self):
        self._lock_pub.publish(Bool(data=self._locked))

    def _tick(self):
        # Republish every tick regardless of whether the value changed -
        # the TRANSIENT_LOCAL retained sample is the primary mechanism, this
        # is the belt-and-suspenders correction for a lost/late subscriber.
        self._publish_lock()
        if self._locked:
            # Always a fresh, default-constructed Twist - see the
            # [Equal-priority nonzero stop] note above. Never published at
            # all while unlocked, so cmd_vel_stop drops out of the mux's
            # active topics via its own 0.5s timeout instead of this node
            # having to decide what a meaningful "unlocked" Twist would be.
            self._stop_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
