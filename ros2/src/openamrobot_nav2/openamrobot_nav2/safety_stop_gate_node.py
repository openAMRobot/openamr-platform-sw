#!/usr/bin/env python3
"""
Functional stop path gate for twist_mux's software_stop lock/input pair (issue #35).

Mohamed Sayed's review of the posted twist_mux config (platform-sw#35) raised
three properties of that config this node has to actually satisfy, not just
assert:

[Startup] "timeout: 0.0 disables lock expiry; it does not start the mux
locked. The stored Bool starts false." True of twist_mux's own internal
state - a Lock with no message ever received defaults to not-engaged. The
reviewed configuration used a 0.0 timeout (meaning an engaged lock would
stay held without updates), but the shipped lock timeout is actually 0.3 s.
Either way, the lock has to already read true the moment anything else in
the graph could possibly start commanding motion, before this node has
received or sent a single message:

  - self._locked is set True in __init__, before any publisher or
    subscription is created - there is no window where this node exists but
    has not yet decided it is locked.
  - the /safety/stop_lock publisher here uses TRANSIENT_LOCAL durability,
    but that is NOT what makes a fresh or restarted twist_mux start locked.
    Mohamed's review (platform-sw#90) caught this: twist_mux's own
    LockTopicHandle subscribes with rclcpp::SystemDefaultsQoS() (volatile -
    see twist_mux/topic_handle.hpp), so it never receives this publisher's
    retained sample at all, only a live message published after it
    subscribes. Durability was this node's own earlier (wrong) assumption,
    written before checking the mux's actual subscriber QoS.
  - the real fix is in twist_mux.yaml: locks.software_stop.timeout is
    nonzero (0.3), so twist_mux's own hasExpired() - (now() - stamp_) >
    timeout, with stamp_ starting at epoch 0 - reads expired, i.e. locked,
    by default the instant the mux exists, before it has received anything
    from this node. _tick()'s periodic republish (below) then lands a real
    True well inside that 0.3s window, same as always. See twist_mux.yaml's
    own comment for the full reasoning, including the use_sim_time
    cold-start edge case and the launch-level stagger
    (navigation_launch.py) that covers it.
  - belt-and-suspenders beyond the timeout fix alone: _tick() republishes
    the current lock value at TICK_HZ regardless of whether it changed, so
    even a QoS/transport hiccup that lost a sample is corrected within one
    tick period, and the TRANSIENT_LOCAL durability on this publisher still
    helps any OTHER subscriber in the graph that (unlike twist_mux) actually
    uses a durability-compatible QoS.

The lock timeout is 0.3 s. The mux's lock subscriber is volatile and depends
on the 10 Hz republish.

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
cmd_vel_stop at [255] and forwards it unchanged." True of the mux. cmd_vel_stop
itself still can only ever carry zero from this node (unchanged - see below),
but that alone does not close the gap: it only constrains what THIS node
publishes there, not what the mux actually selects and forwards. twist_mux's
own arbitration treats cmd_vel_stop as one candidate among others at its
priority; nothing stops some OTHER publisher from putting a nonzero Twist on
the same topic at the same priority (ROS topics have no ACL, same point as
[Release] above), and if that happened the mux would select and forward it
exactly like any other valid input - equal priority is not masked. Mohamed's
review (platform-sw#90) named the concrete case: Nikolai's A2 case 3.

The actual fix (platform-sw#90, Alex's review) does not try to win that
argument on the mux's own arbitration ground at all - it adds a second,
independent enforcement stage AFTER the mux, where the lock state is checked
directly rather than inferred from which input won arbitration:

  - this node subscribes to cmd_vel_mux (the mux's raw arbitrated output,
    same topic test_twist_mux_startup_and_restart.py already probes) and
    republishes onto cmd_vel_gated, which navigation_launch.py now feeds to
    velocity_smoother INSTEAD of cmd_vel_mux directly;
  - _on_mux_output() ignores the received Twist entirely and republishes a
    fresh Twist() whenever self._locked is True - what the mux selected,
    including a nonzero cmd_vel_stop forwarded by some other publisher, never
    reaches cmd_vel_gated while locked;
  - while unlocked it republishes the received Twist unchanged (pass-through)
    - no behavior change for the normal navigation/docking/manual case;
  - _tick() ALSO force-publishes a fresh zero on cmd_vel_gated at TICK_HZ
    while locked, independent of whether cmd_vel_mux produced anything -
    same belt-and-suspenders reasoning as the existing cmd_vel_stop tick
    below, and it means a mux that itself goes silent while locked (its own
    inputs having timed out) still gets an actively-republished zero on
    cmd_vel_gated, not silence.

This closes the gap unconditionally: cmd_vel_gated's value depends only on
self._locked, read directly in this node, never on what the mux decided to
select or forward. The original cmd_vel_stop mechanism (below) is kept as-is,
not replaced - it is still a valid defense-in-depth input (mux-level masking
of lower-priority sources via the lock, independent of this node's own
downstream pass), and the twist_mux.yaml comment on
topics.software_stop.timeout documents the ordering invariant between that
timeout and the lock's own timeout that still matters for it.

[Loss of the zero publisher while locked] The gate node dies, and it stops
republishing. The lock expires after 0.3 s without a refresh. An expired
lock reads as locked, so it fails closed. The volatile subscriber ignores
the retained sample, so it cannot hold the lock. The cmd_vel_gated output
goes silent. The velocity_smoother waits 0.3 s, then decelerates to zero.
The firmware watchdog acts as the independent backstop per
docs/safety/02_limits_and_watchdog.md §2, after about 200 ms.

CI-tested: the shipped lock timeout is above 0, startup and restart ordering
with the real mux, and the smoother lost-input distance bound.
Bench-unverified: the gate dying mid-run end to end, what the mux emits
after the lock expires (silence or an explicit zero), the real stopping
distance, and the firmware watchdog.

Alex's review (platform-sw#90) caught this node's own earlier claim here as
false: navigation_launch.py passes respawn=use_respawn for every node in the
file, this one included - it is not a literal respawn=True, and use_respawn's
own DeclareLaunchArgument default is 'False'. So as actually shipped (no
use_respawn:=True override at launch time), this node is NOT respawned on
crash by default; the sentence above describing launch-level respawn as the
interim mitigation was describing a configuration nobody runs unless they
pass that argument explicitly. Until #75's dedicated lifecycle manager with
supervised respawn for the whole reactive motion layer exists, the real
interim mitigation is operational: either launch with use_respawn:=True, or
rely on the firmware /cmd_vel watchdog (docs/safety/02_limits_and_watchdog.md
§2, ~200 ms, independent of this whole ROS graph) as the actual backstop for
this crash case.

[Re-engaging the lock] Not addressed anywhere in the reviewed proposal - the
/safety/stop_lock_release service that clears it has no documented
counterpart for re-engaging it (e.g. from the UI's "Stop (software)" button,
openamrobot-ui#34). /safety/stop_lock_engage is added here as the symmetric
service, same reasoning as release being a deliberate service call rather
than a topic publish (Kunal's own proposal note) - but the UI-side wiring to
it is unconfirmed and needs the same cross-repo contract record nik asked
for on the release/relay interface, not assumed from this file alone.

Frozen-clock note: simulation only, with a stalled /clock the tick does not
fire and only the engage-time zero and _on_mux_output produce output.
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
