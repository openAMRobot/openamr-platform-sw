#!/usr/bin/env python3
"""
Functional stop path gate for twist_mux's software_stop lock/input pair (issue #35).

This is the functional "Stop (software)" path (register STOP-SCOPE), not the
hardwired E-stop. It owns /safety/stop_lock (the only publisher in the
graph), force-zeroes the mux's output while locked, and gates the final
cmd_vel_gated path directly off its own lock state rather than off mux
arbitration.

[platform-sw#119, Alex's Option B decision, 5 Nov 2026] The Twist-based rearm
latch (platform-sw#90) is REMOVED. It inferred authority from the shape of
cmd_vel_mux traffic - a zero proved neutral for one instant, not that the old
goal actually lost authority, and a source that went silent without a final
zero deadlocked it forever (platform-sw#119). Neither failure mode is fixable
by tuning the traffic-shape rule itself. Replaced with: engage actively
cancels every goal and aborts docking (see engage_lock/_tick), and release is
refused outright, with a reason, until that cancellation is independently
confirmed and every direct cmd_vel input has gone quiet. Once release is
accepted, cmd_vel_gated passes through immediately - there is no latch left
to satisfy.

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

[Engage: active cancellation] engage_lock() cancels every goal on
navigate_to_pose, navigate_through_poses, dock_robot and undock_robot (a
zero-id, zero-stamp action_msgs/CancelGoal request cancels all goals on that
server, same convention openamrobot-ui#50 uses) and calls dock_trigger's
~/abort service, which sets its own cancel flag (platform item 4) without
waiting for the sequence to actually unwind. _tick() repeats both calls
every tick while locked, not just once on the engage transition, so a goal
sent DURING the stop is cancelled too (platform-sw#119 test 6) - this node
does not try to prevent a goal from being accepted while locked, only to
make sure none survives it. A client call that finds its server not ready
(not up, or gone) is skipped for that tick and retried the next one; nothing
here blocks waiting on a response.

[Release: three-part refusal gate] /safety/stop_lock_release is refused,
with a reason in both the service response and /safety/stop_state, until
ALL of:
  (a) none of the four actions above reports an accepted, executing or
      cancelling goal, AND each one has reported SOME status at least once
      with its cancel_goal service currently reachable (see _release_blockers);
  (b) dock_trigger's own last-known status is not docking/undocking, AND it
      has reported some status at least once with its ~/abort service
      currently reachable (same reasoning as (a));
  (c) cmd_vel_nav, cmd_vel_dock and cmd_vel_joy - read directly, not via
      cmd_vel_mux - are each either literally zero or have gone silent past
      their own twist_mux.yaml timeout (a topic with no message at all ever
      received counts as silent by definition, not as unknown).
Once accepted, cmd_vel_gated passes through immediately; there is nothing
left to rearm.

[(a)/(b) are judged by content and live reachability, not message age -
platform-sw#90, Mohamed's review] An earlier version of (a)/(b) also
required each status sample to be no more than 1.0s old. ROS 2 action
status only publishes on a goal-state transition, not as a periodic
heartbeat (design.ros2.org/articles/actions.html#goal-status-topic), and
dock_trigger's own idle heartbeat is every 2.0s (dock_trigger.py's
_heartbeat_status) - a healthy, idle system's last sample can simply age
past any fixed window with nothing wrong, refusing Resume forever. Content
plus service_is_ready() on the same client already used to cancel/abort
(see _cancel_clients/_dock_abort_client) proves the same thing without a
clock: "never received any status at all" still refuses (cannot confirm
clear, same as before), an active goal/docking status always refuses
regardless of age (fail-closed in the safe direction), and a clear status
whose server has since gone fully unreachable (crashed, never answers)
refuses with the same undifferentiated reason - the one case this change
removes is "clear, reachable, just quiet for a while," which was never
actually unsafe.

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

[Loss of the zero publisher while locked] If this node dies, it stops
republishing. The lock expires after 0.3s without a refresh; an expired
lock reads as locked (fail-closed), and the volatile subscriber ignores any
retained sample, so nothing can hold a stale unlocked state. cmd_vel_gated
goes silent, velocity_smoother waits 0.3s then decelerates to zero, and the
firmware watchdog is the independent backstop after about 200ms
(docs/safety/02_limits_and_watchdog.md Section 2).

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

[/safety/stop_state] Published every tick (and once immediately at
startup): STATE_LOCKED while locked with no release yet attempted,
STATE_RELEASE_REFUSED with the same reason the last refused release got
(held until the next accepted release - a UI that reloads mid-refusal must
still see it, not just whoever happened to be listening at the moment of
the refused call), or STATE_RUNNING once unlocked. See openamr_nav_msgs/
StopState.msg (interfaces#35).
"""
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from openamr_nav_msgs.msg import StopState
from openamrobot_nav2.goal_authority import GoalAuthority, ProducerState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

TICK_HZ = 20.0
T_READY = 1.0  # twice the longest mux input timeout (0.5 s)

# Actions the gate cancels on engage and re-cancels every tick while locked
# (platform-sw#119). Cancellation itself is fire-and-forget per tick; whether
# it actually took is judged independently, from each action's own status
# array, at release time - see _release_blockers.
_CANCEL_ACTIONS = (
    'navigate_to_pose',
    'navigate_through_poses',
    'dock_robot',
    'undock_robot',
)
# Producers backed by the goal-authority ledger (D-13) - a subset of
# _CANCEL_ACTIONS. The other three stay on the OLD status-topic-content
# check until openamrobot-ui also reports through this channel (cross-repo,
# not in this PR) - see _release_blockers' "ledger" vs "legacy" branches.
_LEDGER_PRODUCERS = ('undock_robot',)

# How long without a heartbeat/notify before a ledger producer reads as
# UNKNOWN again (D-13: "missing, stale... blocks"). Matches dock_trigger's
# own 2.0s heartbeat period with a margin for one missed beat, same
# reasoning as _CMD_VEL_INPUT_TIMEOUTS' per-topic values above.
_LEDGER_STALE_AFTER_S = 4.5

# cmd_vel inputs the gate reads directly for release condition (c), each with
# its own twist_mux.yaml timeout - NOT re-derived here; keep these in sync
# with that file by hand, same as TICK_HZ/T_READY's own cross-file comments.
_CMD_VEL_INPUT_TIMEOUTS = {
    'cmd_vel_nav': 0.5,
    'cmd_vel_dock': 0.3,
    'cmd_vel_joy': 0.25,
}

_ACTIVE_GOAL_STATUSES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)


class SafetyStopGateNode(Node):

    def __init__(self):
        super().__init__('safety_stop_gate_node')
        # Locked before anything else in this constructor runs - see the
        # [Startup] note above. Nothing downstream of this line can observe
        # this node in an unlocked state before it has decided it is locked.
        self._locked = True
        self._start = self.get_clock().now()
        self._release_refused = False
        self._refusal_reason = ''

        lock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lock_pub = self.create_publisher(Bool, '/safety/stop_lock', lock_qos)
        self._state_pub = self.create_publisher(StopState, '/safety/stop_state', lock_qos)
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

        # --- goal cancellation (platform-sw#119, release condition a) ---
        self._cancel_clients = {
            action: self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
            for action in _CANCEL_ACTIONS
        }
        self._goal_statuses = {action: None for action in _CANCEL_ACTIONS}  # list[int] or None
        # True once a CancelGoal response for this action came back rejected
        # (Mohamed's 7 Nov 05:23 review, panthera-momagdii's #90 re-review):
        # content-plus-service-readiness alone cannot see a rejected cancel
        # against an already-"clear" status - this is independent evidence,
        # the actual CancelGoal.Response, not inferred from anything else.
        # Only ever updated from a real response (see
        # _make_cancel_response_cb); a reject here is never auto-cleared by
        # engage, same reasoning as the ledger's own orphaned flag.
        self._cancel_rejected = {action: False for action in _CANCEL_ACTIONS}
        for action in _CANCEL_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'{action}/_action/status',
                self._make_goal_status_cb(action), 10)

        # --- dock_trigger abort (release condition b) ---
        self._dock_abort_client = self.create_client(Trigger, '/dock_trigger/abort')
        self._dock_status = None
        self.create_subscription(
            String, 'dock_trigger_status', self._on_dock_status, 10)

        # --- direct cmd_vel inputs (release condition c) ---
        self._cmd_vel_last = {name: None for name in _CMD_VEL_INPUT_TIMEOUTS}
        self._cmd_vel_last_at = {name: None for name in _CMD_VEL_INPUT_TIMEOUTS}
        for name in _CMD_VEL_INPUT_TIMEOUTS:
            self.create_subscription(
                Twist, name, self._make_cmd_vel_cb(name), 10)

        # --- goal authority ledger (platform-sw#124, D-13) ---
        self._goal_authority = GoalAuthority(_LEDGER_PRODUCERS, _LEDGER_STALE_AFTER_S)
        self.create_subscription(
            String, 'goal_authority_events', self._on_goal_authority_event, 10)

        # --- enabled producers (D-13: "the launch profile names the
        # enabled producers... disabled producers excluded by name, never
        # inferred from absence") ---
        self.declare_parameter('enabled_producers', list(_CANCEL_ACTIONS))
        self._enabled_producers = set(self.get_parameter('enabled_producers').value)
        unknown = self._enabled_producers - set(_CANCEL_ACTIONS)
        if unknown:
            raise ValueError(f'enabled_producers names unknown action(s): {unknown}')

        self.create_timer(1.0 / TICK_HZ, self._tick)
        # Publish the initial state immediately, not just on the first timer
        # tick - the retained TRANSIENT_LOCAL sample exists from the first
        # possible instant this node could be observed by anything else.
        self._publish_lock()
        self._publish_state()
        self._stop_pub.publish(Twist())
        self._gated_pub.publish(Twist())
        # early subscribers may miss it, and the 20 Hz tick covers that.

    def release_lock(self) -> bool:
        """Clear the lock. Returns False if it was already clear (idempotent, informative)."""
        if not self._locked:
            return False
        self._locked = False
        self._publish_lock()
        return True

    def engage_lock(self) -> bool:
        """Set the lock. Returns False if it was already set (idempotent, informative)."""
        self._release_refused = False
        self._refusal_reason = ''
        self._cancel_all_goals()
        self._abort_docking()
        for producer in _LEDGER_PRODUCERS:
            if self._goal_authority.is_orphaned(producer):
                self._goal_authority.reset_producer(producer)  # D-13/D-14: engage is
                # the deliberate recovery action that may discard a genuinely
                # orphaned ledger entry (left by a GONE instance) - never a live
                # instance's own, currently-outstanding goal (platform-sw#90,
                # panthera-momagdii's review, reproducer A: fire-and-forget
                # cancellation plus a heartbeat is not proof the goal is gone).
                # See goal_authority.is_orphaned()'s own docstring.
        if self._locked:
            return False
        self._locked = True
        self._publish_lock()
        self._stop_pub.publish(Twist())
        self._gated_pub.publish(Twist())
        return True

    def _cancel_all_goals(self) -> None:
        """
        Cancel every goal on every tracked action, fire-and-forget.

        A zero goal_id with a zero stamp cancels all goals on that server
        (action_msgs/CancelGoal; same convention openamrobot-ui#50 uses for
        its own Stop button). Skips a client whose server is not ready
        rather than blocking - the next tick (while still locked) retries.
        Whether a cancel actually took is judged independently at release
        time from each action's own status array, not from this call's
        result.
        """
        zero_goal = GoalInfo()
        for action, client in self._cancel_clients.items():
            if client.service_is_ready():
                future = client.call_async(CancelGoal.Request(goal_info=zero_goal))
                future.add_done_callback(self._make_cancel_response_cb(action))

    def _abort_docking(self) -> None:
        if self._dock_abort_client.service_is_ready():
            self._dock_abort_client.call_async(Trigger.Request())

    def _make_goal_status_cb(self, action: str):
        def _cb(msg: GoalStatusArray) -> None:
            self._goal_statuses[action] = [s.status for s in msg.status_list]
        return _cb

    def _make_cancel_response_cb(self, action: str):
        def _cb(future) -> None:
            try:
                response = future.result()
            except Exception:
                return
            if response is None:
                return
            self._cancel_rejected[action] = response.return_code != CancelGoal.Response.ERROR_NONE
        return _cb

    def _on_dock_status(self, msg: String) -> None:
        self._dock_status = msg.data

    def _make_cmd_vel_cb(self, name: str):
        def _cb(msg: Twist) -> None:
            self._cmd_vel_last[name] = msg
            self._cmd_vel_last_at[name] = self.get_clock().now()
        return _cb

    def _on_goal_authority_event(self, msg: String) -> None:
        try:
            producer, event, instance_id = msg.data.split('|', 2)
        except ValueError:
            self.get_logger().warn(f'malformed goal_authority_events message: {msg.data!r}')
            return
        if producer not in _LEDGER_PRODUCERS:
            self.get_logger().warn(
                f'goal_authority_events for an unmanaged producer: {producer!r}')
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        if event == 'heartbeat':
            self._goal_authority.heartbeat(producer, instance_id, now_s)
        elif event == 'sent':
            self._goal_authority.notify_sent(producer, instance_id, now_s)
        elif event == 'done':
            self._goal_authority.notify_done(producer, instance_id, now_s)
        else:
            self.get_logger().warn(f'unknown goal_authority_events event: {event!r}')

    def _release_blockers(self) -> list:
        """
        Return a list of human-readable reasons release is still refused.

        Empty means every condition (a)/(b)/(c) in the module docstring's
        [Release: three-part refusal gate] section holds. Each entry names
        WHICH condition is blocking, not why (never received, a rejected
        cancel, a timed-out one and a server that is simply gone now all
        surface the same way here) - that is deliberate, not a missing
        diagnostic: this node's job is to stay refused correctly, not to
        explain a remote server's own failure mode. See the module
        docstring's [(a)/(b) are judged by content and live reachability,
        not message age] note for why this is not time-based.
        """
        blockers = []
        now_s = self.get_clock().now().nanoseconds / 1e9
        for action in _CANCEL_ACTIONS:
            if action not in self._enabled_producers:
                continue  # excluded by name (D-13) - never inferred, never checked
            if action in _LEDGER_PRODUCERS:
                state = self._goal_authority.state(action, now_s)
                if state != ProducerState.IDLE:
                    blockers.append(f'{action}: {state.value} (goal-authority ledger)')
                continue
            # LEGACY path - status-topic content, not the ledger. Known weaker
            # than D-13's requirement (inherits the exact gap it was written
            # to close) until this producer's sender(s) - today, openamrobot-ui
            # for navigate_to_pose/navigate_through_poses, nothing yet sends
            # dock_robot pending #125 - also report through goal_authority_events.
            if self._cancel_rejected[action]:
                # Independent of status content/service-readiness: a
                # rejected cancel is never terminal evidence, and must keep
                # blocking even if the last-seen status already looked
                # clear (Mohamed's 7 Nov 05:23 review).
                blockers.append(f'{action}: cancel rejected')
                continue
            statuses = self._goal_statuses[action]
            if statuses is None:
                blockers.append(f'{action}: status stale or never received')
                continue
            if any(s in _ACTIVE_GOAL_STATUSES for s in statuses):
                blockers.append(f'{action}: goal still active')
                continue
            if not self._cancel_clients[action].service_is_ready():
                blockers.append(f'{action}: status stale or never received')

        if self._enabled_producers & {'dock_robot', 'undock_robot'}:
            # Only check dock_trigger when docking is actually enabled (D-13,
            # panthera-momagdii's #90 review) - the nav-only profile starts no
            # dock_trigger process at all, so requiring its status/abort
            # service unconditionally would block Resume forever on that
            # profile. Excluded by name, same as _CANCEL_ACTIONS above -
            # never inferred, never checked.
            if self._dock_status is None:
                blockers.append('dock_trigger: status stale or never received')
            elif self._dock_status in ('docking', 'undocking'):
                blockers.append(f'dock_trigger: {self._dock_status}')
            elif not self._dock_abort_client.service_is_ready():
                blockers.append('dock_trigger: status stale or never received')

        for name, timeout in _CMD_VEL_INPUT_TIMEOUTS.items():
            last = self._cmd_vel_last[name]
            if last is None or last == Twist():
                continue
            age_s = (self.get_clock().now() - self._cmd_vel_last_at[name]).nanoseconds / 1e9
            if age_s <= timeout:
                blockers.append(f'{name}: nonzero, {age_s:.2f}s old (timeout {timeout}s)')

        return blockers

    def _on_release(self, request, response):
        del request  # Trigger has no request fields

        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed < T_READY:
            response.success = False
            response.message = (
                f'clock not ready (current time: {elapsed:.2f}, '
                f'required: {T_READY})')
            self._release_refused = True
            self._refusal_reason = response.message
            return response

        blockers = self._release_blockers()
        if blockers:
            response.success = False
            response.message = '; '.join(blockers)
            self._release_refused = True
            self._refusal_reason = response.message
            return response

        self._release_refused = False
        self._refusal_reason = ''
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

    def _publish_state(self):
        msg = StopState()
        if not self._locked:
            msg.state = StopState.STATE_RUNNING
        elif self._release_refused:
            msg.state = StopState.STATE_RELEASE_REFUSED
            msg.reason = self._refusal_reason
        else:
            msg.state = StopState.STATE_LOCKED
        self._state_pub.publish(msg)

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

        No rearm latch here anymore (platform-sw#119) - the release itself
        is what proves it is safe to forward again (see _release_blockers),
        not a post-hoc zero observed on this topic.
        """
        if self._locked:
            self._gated_pub.publish(Twist())
            return
        self._gated_pub.publish(msg)

    def _tick(self):
        # Republish every tick regardless of whether the value changed -
        # the TRANSIENT_LOCAL retained sample is the primary mechanism, this
        # is the belt-and-suspenders correction for a lost/late subscriber.
        self._publish_lock()
        self._publish_state()
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
            # platform-sw#119: repeat the cancellation every tick, not just
            # once on the engage transition, so a goal sent DURING the stop
            # is cancelled too (test 6).
            self._cancel_all_goals()
            self._abort_docking()


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
