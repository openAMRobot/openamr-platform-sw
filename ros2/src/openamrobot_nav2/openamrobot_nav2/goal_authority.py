"""
Goal-authority ledger for the stop-path gate (platform-sw#124, D-13/D-14).

Implements the "supervised goal proxy" platform-sw#124's
D-13 ruling requires: a component that KNOWS "zero goals active", including
at boot, without relying on the action status topic (Nikolai's 8 Nov
measurement: the four */_action/status topics are silent on a genuinely idle
stack - 0 messages over 12s, twice - because a server that has never
transitioned has nothing to publish; TRANSIENT_LOCAL QoS only helps a
RESTARTED subscriber catch a server's already-published state, not a server
that has said nothing yet).

Design choice, stated plainly for review: true interception (this node
relaying every send_goal call) would need every goal-sender rewritten,
including openamrobot-ui (a separate repo, out of this PR's reach). Instead
this is a NOTIFICATION ledger: a producer this repo controls calls
heartbeat()/notify_sent()/notify_done() around its own, unchanged
send_goal_async() call and its own existing idle heartbeat (dock_trigger.py
already heartbeats its status every 2.0s - this reuses that cadence, which
is also where D-13's "within 2 s" bound for clean-boot Resume comes from:
Resume cannot honestly call a producer IDLE before its first heartbeat ever
arrives, and that first arrival is bounded by the producer's own heartbeat
period).

"Never heard from at all" and "heard from, reporting idle" are DIFFERENT
claims - D-13 rejects inferring the first from silence ("no status counts
as clear"), but a producer that has actively reported idle at least once,
recently, is not silence, it is evidence. heartbeat()/notify_sent() both
establish liveness; state() additionally checks that the MOST RECENT contact
(of either kind) is not older than stale_after, so a producer that stops
heartbeating degrades back to UNKNOWN rather than being clear forever on one
old sample - the same fail-closed intent as the old code's content check,
now with an actual liveness signal behind it instead of inferring content
age from a topic ROS2 actions do not publish periodically.

navigate_to_pose IS wired to this ledger (platform-sw#90, panthera-momagdii's
review): navigate_to_pose_goal_proxy_node.py claims the public navigate_to_pose
action name and heartbeats/notifies through here on every goal it relays - no
openamrobot-ui change needed, since navigation_launch.py's own node-level
remap transparently redirects every existing sender into the proxy. The one
path still NOT covered by this ledger is bt_navigator's own internal
self_client_ (goal_pose/goal_pose_nav-triggered goals), which the SAME
node-level remap also renames but which never passes through the proxy at
all - SafetyStopGateNode's own _release_blockers reconciles against that
server's real action-status topic (navigate_to_pose_real) for this specific
gap, independent of this ledger.

navigate_through_poses and dock_robot are NOT yet wired to this ledger (the
former also sendable directly by openamrobot-ui) and stay on the OLD
status-topic-content check (see SafetyStopGateNode's own _release_blockers
for both paths side by side) - this is explicitly WEAKER than the ledger (it
inherits the exact status-topic gap D-13 was written to close) and is called
out as such in the release-blocker message, so this isn't silently assumed
fixed. Closing that gap for real needs the same goal-owning-proxy pattern
applied to each, or (for navigate_through_poses) openamrobot-ui to
heartbeat/notify through this node too - a cross-repo contract change for
the UI case, not something this file can do alone.

Restart detection: each producer passes its own process-lifetime instance_id
(e.g. a uuid4 generated once in its __init__, stable until that process
restarts) on every heartbeat/notify call. If a NEW instance_id arrives while
the ledger still shows an outstanding (sent, not yet done) goal under the
OLD instance_id, the old entry cannot be trusted to ever receive its
notify_done (that process is gone) and is NOT silently cleared - stays
UNKNOWN until the new instance explicitly reports a goal completing, same
fail-closed rule as "never heard from at all".
"""
import dataclasses
import enum


class ProducerState(enum.Enum):
    UNKNOWN = 'unknown'  # never heard from, gone stale, or a restart orphaned a goal
    IDLE = 'idle'        # live contact from the current instance; zero outstanding
    ACTIVE = 'active'    # one or more outstanding goals from the current instance


@dataclasses.dataclass
class _ProducerLedgerEntry:
    instance_id: str = None
    outstanding: int = 0
    orphaned: bool = False  # a previous instance left something outstanding when it changed
    last_contact_s: float = None  # seconds, caller's own clock - see state()


class GoalAuthority:
    """
    In-process ledger for producers that heartbeat/notify around their own
    send_goal.

    Owned by (not a separate node from) SafetyStopGateNode - see that file's
    integration. No ROS publishers/subscribers of its own; producers call
    heartbeat()/notify_sent()/notify_done() indirectly, by publishing a
    plain std_msgs/String on the goal_authority_events topic
    (SafetyStopGateNode._on_goal_authority_event parses it and calls these
    methods), and the gate reads state() directly (same process, no IPC)
    for its own release decision.
    All methods take now_s explicitly (the caller's own clock reading) -
    this class is deliberately clock-free, same testing convention as
    status_trackers.py's trackers.
    """

    def __init__(self, managed_producers, stale_after_s):
        self._entries = {name: _ProducerLedgerEntry() for name in managed_producers}
        self._stale_after_s = stale_after_s

    def _touch(self, producer, instance_id, now_s):
        entry = self._entries[producer]
        if entry.instance_id is not None and entry.instance_id != instance_id:
            # a different process instance than the one we last heard from -
            # whatever it had outstanding is now unaccountable, not cleared.
            # Sticky (platform-sw#90, panthera-momagdii's review, reproducer
            # B): a LATER restart whose own outstanding count happens to be
            # zero must not un-orphan an EARLIER restart's still-unresolved
            # goal - only reset_producer() may clear `orphaned`.
            entry.orphaned = entry.orphaned or entry.outstanding > 0
            entry.outstanding = 0
        entry.instance_id = instance_id
        entry.last_contact_s = now_s
        return entry

    def heartbeat(
            self, producer: str, instance_id: str, now_s: float, outstanding: int = 0) -> None:
        """
        Report liveness and the producer's own current outstanding count.

        panthera-momagdii's #90 review, P1: a gate that restarts between a
        producer's own notify_sent() and notify_done() never saw the sent
        event - without this, the next heartbeat alone would read IDLE with
        a goal genuinely still in flight. The producer's self-reported count
        is taken as truth here, the same trust boundary notify_sent/
        notify_done already extend to the producer. default=0 keeps every
        existing caller that never sent a count (a genuinely idle heartbeat)
        behaving exactly as before.

        Raises ValueError for a negative count (panthera-momagdii's #90
        review, P2): state() treats any value <= 0 as IDLE, so a negative
        count would silently read the same as a genuinely idle report -
        this must never be stored, not just "normalized" to 0.
        """
        if outstanding < 0:
            raise ValueError(f'outstanding must be >= 0, got {outstanding}')
        entry = self._touch(producer, instance_id, now_s)
        entry.outstanding = outstanding

    def notify_sent(self, producer: str, instance_id: str, now_s: float) -> None:
        entry = self._touch(producer, instance_id, now_s)
        entry.outstanding += 1

    def notify_done(self, producer: str, instance_id: str, now_s: float) -> None:
        entry = self._entries[producer]
        if entry.instance_id != instance_id:
            # a done notification from an instance we are not currently
            # tracking as the live one - ignore rather than decrementing the
            # wrong instance's count
            return
        entry.last_contact_s = now_s
        entry.outstanding = max(0, entry.outstanding - 1)
        # orphaned is deliberately NOT auto-cleared here: it describes a
        # goal an OLDER, now-gone instance left unaccounted for, which this
        # (the current) instance reaching its own zero says nothing about -
        # see reset_producer().

    def reset_producer(self, producer: str) -> None:
        """
        Explicitly clear an orphaned flag, discarding the current instance.

        There is no automatic way to resolve an orphan - the instance that
        owned it is gone and will never call notify_done. Call this only
        from a deliberate operator/system recovery action (this repo's
        intended caller: SafetyStopGateNode.engage_lock(), which already
        re-cancels every goal on every tick while locked - the same event
        that would make a stuck orphaned goal's real-world effect, if any,
        actually stop). Discarding instance_id too means the NEXT
        heartbeat/notify, from whichever instance, is treated as a fresh
        first contact, not implicitly trusted as a continuation.
        """
        self._entries[producer] = _ProducerLedgerEntry()

    def is_orphaned(self, producer: str) -> bool:
        """
        Report whether a now-gone instance left something outstanding.

        This is deliberately narrower than `state(...) == UNKNOWN` (which
        also covers "never heard from" and "stale") - it is the ONE
        condition reset_producer() may be called for (platform-sw#90,
        panthera-momagdii's review, reproducer A): a live instance's own,
        currently-outstanding goal is not orphaned, and must not be reset
        just because engage/Stop was pressed - fire-and-forget cancellation
        is not proof the goal is gone, so the outstanding count must survive
        until the real notify_done() arrives (or the instance itself
        becomes a genuine orphan via a later restart).
        """
        return self._entries[producer].orphaned

    def state(self, producer: str, now_s: float) -> ProducerState:
        entry = self._entries[producer]
        if entry.instance_id is None or entry.orphaned:
            return ProducerState.UNKNOWN
        if now_s - entry.last_contact_s > self._stale_after_s:
            return ProducerState.UNKNOWN
        if entry.outstanding > 0:
            return ProducerState.ACTIVE
        return ProducerState.IDLE

    def managed(self):
        return tuple(self._entries)
