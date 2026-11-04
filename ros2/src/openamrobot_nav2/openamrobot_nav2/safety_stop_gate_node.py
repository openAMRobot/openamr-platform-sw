#!/usr/bin/env python3
"""
Functional stop path gate for twist_mux's software_stop lock/input pair (issue #35).

This is the functional "Stop (software)" path (register STOP-SCOPE), not the
hardwired E-stop. It owns /safety/stop_lock (the only publisher in the
graph), force-zeroes the mux's output while locked, and gates the final
cmd_vel_gated path directly off its own lock state rather than off mux
arbitration.

[Startup] This node starts locked before any publisher or subscription
exists (self._locked = True is the first statement in __init__), so there
is no window where it could be observed unlocked. That alone is not what
makes a fresh or restarted twist_mux also start locked, though: twist_mux's
lock subscriber uses volatile QoS, so it never receives this node's
TRANSIENT_LOCAL retained sample - only a live message published after it
subscribes. The actual mechanism is twist_mux.yaml's
locks.software_stop.timeout (0.3s, nonzero): hasExpired() is (now() -
stamp_) > timeout, and stamp_ starts at epoch 0, so an untouched handle
reads as already-expired (i.e. locked) from the instant it exists -
fail-closed by default. _tick()'s periodic republish at TICK_HZ then lands
a real value well inside that 0.3s window during normal operation, and the
TRANSIENT_LOCAL durability still helps any other subscriber using a
compatible QoS. See twist_mux.yaml's own comment for the use_sim_time
cold-start case and the launch-level stagger (navigation_launch.py) that
covers it.

[Release] This node is the only publisher on /safety/stop_lock, and release
only happens through release_lock(), called from the
/safety/stop_lock_release service - no code path other than that one ever
sets self._locked to False. That "only publisher" property is a graph
invariant this node cannot enforce on its own (ROS topics have no ACL); it
is checked externally by tools/issue35_arbitration/stop_lock_invariants.py.

[Equal-priority nonzero stop / post-mux gate] twist_mux's own arbitration
treats cmd_vel_stop as one candidate among others at its priority - nothing
stops some other publisher
putting a nonzero Twist on the same topic at the same priority, which the
mux would then select and forward like any valid input (equal priority is
not masked). This node does not rely on cmd_vel_stop winning arbitration at
all: it subscribes to the mux's raw output (cmd_vel_mux) and republishes
onto cmd_vel_gated, which navigation_launch.py feeds to velocity_smoother
instead of cmd_vel_mux directly. _on_mux_output() ignores the received
Twist and republishes a fresh zero whenever locked; while unlocked it
forwards unchanged. _tick() also force-publishes a zero on cmd_vel_gated at
TICK_HZ while locked, independent of whether the mux produced anything, so
a mux that itself goes silent while locked still gets an
actively-republished zero. cmd_vel_gated's value therefore depends only on
self._locked, never on what the mux decided to select.

[Rearm latch] release_lock() alone must not let stale motion resume: a
still-active, uncancelled nav/dock stream would otherwise forward on the
very first mux message after release. cmd_vel_gated keeps forcing zero
after release until the mux output has itself been zero at least once (see
_on_mux_output's self._rearmed tracking); only then does pass-through
resume. Cancelling the stale goal itself is a separate UI/BT-contract
concern (openamrobot-ui#50); this gate holds on its own regardless.

[Loss of the zero publisher while locked] If this node dies, it stops
republishing. The lock expires after 0.3s without a refresh; an expired
lock reads as locked (fail-closed), and the volatile subscriber ignores any
retained sample, so nothing can hold a stale unlocked state. cmd_vel_gated
goes silent, velocity_smoother waits 0.3s then decelerates to zero, and the
firmware watchdog is the independent backstop after about 200ms
(docs/safety/02_limits_and_watchdog.md Section 2).

CI-tested: the shipped lock timeout is above 0, startup and restart
ordering with the real mux, and the smoother's lost-input distance bound.
Bench-unverified: the gate dying mid-run end to end, what the mux emits
after the lock expires, the real stopping distance, and the firmware
watchdog.

navigation_launch.py passes respawn=use_respawn for every node including
this one; use_respawn's own default is False, so this node is not
respawned on crash unless a launch explicitly passes use_respawn:=True.
Until #75's dedicated lifecycle manager with supervised respawn exists for
the whole reactive motion layer, the interim mitigation is operational:
either launch with use_respawn:=True, or rely on the firmware /cmd_vel
watchdog (~200ms, independent of this whole ROS graph) as the actual
backstop for this crash case.

[Re-engaging the lock] /safety/stop_lock_engage is the symmetric service to
release, for re-engaging the lock (e.g. from the UI's "Stop (software)"
button, openamrobot-ui#34) - a deliberate service call, same as release,
rather than a topic publish. UI-side wiring to it needs its own cross-repo
contract record, not assumed from this file alone.

[T_READY and the frozen-clock limitation] Release is refused while elapsed
time since this node's own construction is below T_READY (1.0s) - see
_on_release. A frozen simulation clock is a documented simulation-only
limitation: with a stalled /clock, _tick() does not fire, and only the
engage-time zero and _on_mux_output produce output.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

TICK_HZ = 20.0
T_READY = 1.0  # twice the longest mux input timeout (0.5 s)


class SafetyStopGateNode(Node):

    def __init__(self):
        super().__init__('safety_stop_gate_node')
        # Locked before anything else in this constructor runs - see the
        # [Startup] note above. Nothing downstream of this line can observe
        # this node in an unlocked state before it has decided it is locked.
        self._locked = True
        self._start = self.get_clock().now()

        self._rearmed = True

        lock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lock_pub = self.create_publisher(Bool, '/safety/stop_lock', lock_qos)
        self._stop_pub = self.create_publisher(Twist, 'cmd_vel_stop', 10)

        # Post-mux gate (platform-sw#90, Alex's review) - see the
        # [Equal-priority nonzero stop] docstring section above. navigation_
        # launch.py feeds cmd_vel_gated (not cmd_vel_mux directly) to
        # velocity_smoother.
        self._gated_pub = self.create_publisher(Twist, 'cmd_vel_gated', 10)
        self._mux_sub = self.create_subscription(
            Twist, 'cmd_vel_mux', self._on_mux_output, 10)

        self.create_service(Trigger, '/safety/stop_lock_release', self._on_release)
        self.create_service(Trigger, '/safety/stop_lock_engage', self._on_engage)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        # Publish the initial state immediately, not just on the first timer
        # tick - the retained TRANSIENT_LOCAL sample exists from the first
        # possible instant this node could be observed by anything else.
        self._publish_lock()
        self._stop_pub.publish(Twist())
        self._gated_pub.publish(Twist())
        # early subscribers may miss it, and the 10 Hz tick covers that.

    def release_lock(self) -> bool:
        """Clear the lock. Returns False if it was already clear (idempotent, informative)."""
        if not self._locked:
            return False
        self._locked = False
        self._rearmed = False
        self._publish_lock()
        return True

    def engage_lock(self) -> bool:
        """Set the lock. Returns False if it was already set (idempotent, informative)."""
        if self._locked:
            # The already engaged path does not send them.
            return False
        self._locked = True
        self._publish_lock()
        self._stop_pub.publish(Twist())
        self._gated_pub.publish(Twist())
        return True

    def _on_release(self, request, response):
        del request  # Trigger has no request fields

        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed < T_READY:
            response.success = False
            response.message = (
                f'clock not ready (current time: {elapsed:.2f}, '
                f'required: {T_READY})')
            return response

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

    def _on_mux_output(self, msg):
        """
        Gate the mux's own arbitrated output (platform-sw#90, Alex's review).

        See the [Equal-priority nonzero stop] docstring section above for the
        full reasoning. While locked, whatever the mux selected is discarded
        and a fresh zero goes out instead - this does not depend on cmd_vel_
        stop having won arbitration, so a nonzero Twist from any OTHER
        publisher on cmd_vel_stop (or any other input) still comes out zero
        here. While unlocked, the message is forwarded unchanged: no behavior
        change for normal navigation/docking/manual operation.

        Rearm latch (platform-sw#90 review, Alex, BLOCKER 1): release_lock()
        alone must not let stale motion resume. After release, cmd_vel_gated
        keeps forcing zero until the mux output has itself been zero at least
        once - only then does normal pass-through resume. Cancelling the
        active nav/dock goal on engage is a separate UI/BT-contract concern
        (openamrobot-ui#50); this gate holds on its own regardless.
        """
        if self._locked:
            self._gated_pub.publish(Twist())
            return
        if not self._rearmed:
            if msg == Twist():
                self._rearmed = True
            else:
                self._gated_pub.publish(Twist())
                return
        self._gated_pub.publish(msg)

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
            # Same reasoning, one hop downstream: force cmd_vel_gated to zero
            # on every tick too, not only in reaction to a cmd_vel_mux
            # message - if the mux itself goes silent while locked (its own
            # inputs all timed out), cmd_vel_gated must still carry an
            # actively-republished zero, not silence.
            self._gated_pub.publish(Twist())


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
