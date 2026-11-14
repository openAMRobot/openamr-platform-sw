"""
Pure logic of the base status monitor (no ROS imports, injected clock).

The 2.0 base controller publishes /base/status (10 Hz, stale after 0.5 s) and
/base/aux_status (5 Hz, stale after 0.6 s), I8 frozen fields kept in proposal rev 4
(openamrobot-interfaces #38). This module decides one thing: may the existing host-side
command mux lock be released? It can only keep it engaged or refuse a release; it never
commands motion.

Rules (P-03 rev20.6 item 30 A1, I8 proposal rev 4 freshness and UNKNOWN rules):

* Freshness is measured from the LOCAL receipt time on a MONOTONIC clock supplied by the
  caller. The header stamp is never used. A negative age (the clock went backwards, or a
  caller mixed clocks) is STALE, never fresh.
* boot_id (uint32, 0 = UNKNOWN) and sequence (uint32) are frozen identity fields. A
  missing, non-integer, boolean, negative or out-of-range value is Malformed. A Malformed
  sample never counts towards liveness.
* A first message after start-up (or after a gap longer than the stale time) may be a
  retained snapshot. It does not prove the publisher is alive. The stream is live only
  after a later message arrives within the stale time of the previous one.
* A boot identity change or a sequence number that goes backwards is a publisher
  restart. The stream is not live again until the next message. A restart never clears a
  fault the controller held as latched before it: the gate carries the fault, shows it in
  the operator text and keeps the lock engaged until the explicit operator Resume, whose
  reply names it as acknowledged. Nothing the restarted controller reports can clear it.
* Malformed messages and unsupported contract versions are failures, never UNKNOWN
  and never nominal.
* The lock starts engaged. Once engaged it is released only by an explicit resume
  request, and only while every blocker is gone. Nothing resumes by itself, and a
  resume never starts motion; it only clears the inhibit.
* This is a functional host inhibit. The hardwired E-stop chain is the safety
  function and this module never replaces it.
"""
from dataclasses import dataclass, field
import enum
import math

# Defaults from I8 rev 3 (frozen 6 Nov 2026, decision 9).
STATUS_STALE_S = 0.5
AUX_STALE_S = 0.6

CONTROLLER_STATES = ('UNKNOWN', 'INITIALIZING', 'READY', 'DEGRADED', 'FAULTED')
# Safety state machine rev 3 (platform-fw #17). Motion is allowed only in RUN.
# I8 proposal rev 4 adds UNKNOWN and INHIBITED (E-01 rev 4). Proposed, not approved.
SAFETY_STATES = ('UNKNOWN', 'LOCKED', 'RUN', 'INHIBITED', 'CHARGING', 'CHARGER_FAULT')

SEQUENCE_MODULUS = 2 ** 32
UINT16_MODULUS = 2 ** 16


class Blocker(str, enum.Enum):
    """Reasons that keep the command mux locked."""

    NO_STATUS = 'NO_STATUS'
    STATUS_STALE = 'STATUS_STALE'
    STATUS_NOT_LIVE = 'STATUS_NOT_LIVE'
    STATUS_MALFORMED = 'STATUS_MALFORMED'
    CONTRACT_VERSION = 'CONTRACT_VERSION'
    CONTROLLER_NOT_READY = 'CONTROLLER_NOT_READY'
    SAFETY_STATE_NOT_RUN = 'SAFETY_STATE_NOT_RUN'
    RECOVERY_REQUIRED = 'RECOVERY_REQUIRED'
    FAULT_INHIBITS_MOTION = 'FAULT_INHIBITS_MOTION'
    NO_AUX_STATUS = 'NO_AUX_STATUS'
    AUX_STALE = 'AUX_STALE'
    AUX_NOT_LIVE = 'AUX_NOT_LIVE'
    AUX_MALFORMED = 'AUX_MALFORMED'
    MESSAGE_TYPE_UNAVAILABLE = 'MESSAGE_TYPE_UNAVAILABLE'
    CONFIG_INVALID = 'CONFIG_INVALID'


# Short operator text for each blocker. Fault text comes from the registry, not from here.
BLOCKER_TEXT = {
    Blocker.NO_STATUS: 'No status from the base controller yet.',
    Blocker.STATUS_STALE: 'Base controller status is stale (no message for 0.5 s). '
                          'Check the link to the base controller.',
    Blocker.STATUS_NOT_LIVE: 'Base controller status not confirmed live yet.',
    Blocker.STATUS_MALFORMED: 'Base controller sent a malformed status. Motion stays blocked.',
    Blocker.CONTRACT_VERSION: 'Base controller status uses an unsupported contract version.',
    Blocker.CONTROLLER_NOT_READY: 'Base controller is not READY.',
    Blocker.SAFETY_STATE_NOT_RUN: 'Base controller is LOCKED (not in RUN). Release the E-stop, '
                                  'press RESET, then Resume.',
    Blocker.RECOVERY_REQUIRED: 'Base controller requires recovery before motion.',
    Blocker.FAULT_INHIBITS_MOTION: 'A base controller fault inhibits motion.',
    Blocker.NO_AUX_STATUS: 'No auxiliary status (lift, head light) from the base yet.',
    Blocker.AUX_STALE: 'Auxiliary status is stale (no message for 0.6 s).',
    Blocker.AUX_NOT_LIVE: 'Auxiliary status not confirmed live yet.',
    Blocker.AUX_MALFORMED: 'Base controller sent a malformed auxiliary status.',
    Blocker.MESSAGE_TYPE_UNAVAILABLE: 'The base status message package is not installed.',
    Blocker.CONFIG_INVALID: 'The base status monitor configuration is invalid. '
                            'Motion stays blocked.',
}


@dataclass(frozen=True)
class Fault:
    """One active fault entry (group 18001 for base codes)."""

    code: int
    severity: str = 'UNKNOWN'
    motion_inhibit: bool = True
    active: bool = True
    latched: bool = False


@dataclass(frozen=True)
class Malformed:
    """A message that broke schema or semantic rules. Never UNKNOWN, never nominal."""

    reason: str


@dataclass(frozen=True)
class BaseSnapshot:
    """The fields of /base/status the gate needs."""

    contract_version: int
    boot_id: object
    sequence: int
    controller_state: str
    safety_state: str
    recovery_required: bool = False
    faults: tuple = ()


@dataclass(frozen=True)
class AuxSnapshot:
    """The fields of /base/aux_status the gate needs (freshness and identity only)."""

    contract_version: int
    boot_id: object
    sequence: int


def is_uint(value, modulus):
    """Return True only for a real integer (never bool) in 0..modulus-1."""
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value < modulus


def _validate_identity(snapshot):
    if not is_uint(snapshot.contract_version, UINT16_MODULUS):
        return Malformed('contract_version is not a uint16: %r' % (snapshot.contract_version,))
    if not is_uint(snapshot.boot_id, SEQUENCE_MODULUS) or snapshot.boot_id == 0:
        return Malformed('boot_id is not a non-zero uint32: %r' % (snapshot.boot_id,))
    if not is_uint(snapshot.sequence, SEQUENCE_MODULUS):
        return Malformed('sequence is not a uint32: %r' % (snapshot.sequence,))
    return None


def validate_aux(snapshot):
    """Return Malformed(reason) if the aux snapshot breaks the frozen identity rules."""
    return _validate_identity(snapshot)


def validate_base(snapshot):
    """Return Malformed(reason) if the snapshot breaks the rules, else None."""
    bad = _validate_identity(snapshot)
    if bad is not None:
        return bad
    if snapshot.controller_state not in CONTROLLER_STATES:
        return Malformed('controller_state out of range: %r' % (snapshot.controller_state,))
    if snapshot.safety_state not in SAFETY_STATES:
        return Malformed('safety_state out of range: %r' % (snapshot.safety_state,))
    # Contradiction: READY with an active motion-inhibiting fault.
    if snapshot.controller_state == 'READY' and any(
            f.active and f.motion_inhibit for f in snapshot.faults):
        return Malformed('READY with an active motion-inhibiting fault')
    return None


class FaultRegistry:
    """
    Operator text by fault code, loaded from the registry in openamrobot-interfaces.

    The consumer keeps no text of its own. The file format is not final (I8 decision 5,
    TODO(Evans)); this reader takes ``{code: {name: ..., text: ...}}`` or
    ``{code: text}``. An unknown code stays reportable with a generic line.
    """

    def __init__(self, entries=None):
        self._entries = {}
        for code, value in (entries or {}).items():
            if isinstance(value, dict):
                self._entries[int(code)] = (str(value.get('name', '')), str(value.get('text', '')))
            else:
                self._entries[int(code)] = ('', str(value))

    def __len__(self):
        return len(self._entries)

    def text_for(self, code):
        """Return the operator line for a fault code."""
        name, text = self._entries.get(int(code), ('', ''))
        if text:
            return 'Fault %d%s: %s' % (code, ' (%s)' % name if name else '', text)
        return 'Fault %d: not in the fault registry; see the base controller documentation.' % code


class Freshness:
    """Local-receipt freshness with the retained-only rule."""

    def __init__(self, stale_s):
        self.stale_s = float(stale_s)
        self.last = None
        self.live = False
        self._broken = False
        self.count = 0

    def _fresh(self, now):
        # A negative age means the clock went backwards: never fresh.
        return self.last is not None and 0.0 <= now - self.last < self.stale_s

    def received(self, now, restart=False):
        """
        Record a valid message.

        The stream is live only if the previous VALID message was fresh and no Malformed
        message came in between: a Malformed sample never counts towards liveness.
        """
        self.live = self._fresh(now) and not restart and not self._broken
        self._broken = False
        self.last = now
        self.count += 1

    def invalid(self, now):
        """Record a Malformed message: it breaks the liveness chain and is not evidence."""
        del now
        self.live = False
        self._broken = True
        self.count += 1

    def state(self, now):
        """Return NEVER, STALE, NOT_LIVE or LIVE."""
        if self.last is None:
            return 'NEVER'
        if not self._fresh(now):
            return 'STALE'
        return 'LIVE' if self.live else 'NOT_LIVE'

    def age(self, now):
        """Seconds since the last message, or inf if none."""
        return math.inf if self.last is None else max(0.0, now - self.last)


@dataclass
class GateState:
    """Result of an evaluation."""

    locked: bool
    blockers: tuple
    status_state: str
    aux_state: str
    base: object = None
    summary: str = ''
    operator_lines: tuple = field(default_factory=tuple)


class BaseGate:
    """The command-lock state machine. Time is passed in by the caller."""

    def __init__(self, registry=None, require_aux=True, supported_versions=(1,),
                 status_stale_s=STATUS_STALE_S, aux_stale_s=AUX_STALE_S):
        self.registry = registry or FaultRegistry()
        self.require_aux = require_aux
        self.supported_versions = tuple(supported_versions)
        self.status = Freshness(status_stale_s)
        self.aux = Freshness(aux_stale_s)
        self.base = None            # last valid BaseSnapshot
        self.base_bad = None        # last Malformed, cleared by the next valid message
        self.aux_snapshot = None
        self.aux_bad = None
        self.type_unavailable = False
        self.config_error = None
        self.sequence_gaps = 0
        self.restarts = 0
        self.carried_faults = {}    # code -> Fault latched before a publisher restart
        self._latched = True        # engaged at boot
        self._last_state = None

    # -- inputs -------------------------------------------------------------
    def on_status(self, snapshot, now):
        """Feed one /base/status message (a BaseSnapshot or Malformed)."""
        if isinstance(snapshot, BaseSnapshot):
            snapshot_bad = validate_base(snapshot)
        else:
            snapshot_bad = snapshot
        if snapshot_bad is not None:
            self.base_bad = snapshot_bad
            # keep the last valid snapshot only to compare identity; it no longer counts
            self._base_ref = self.base or getattr(self, '_base_ref', None)
            self.base = None
            self.status.invalid(now)
            return self.evaluate(now)
        reference = self.base or getattr(self, '_base_ref', None)
        restart = self._is_restart(reference, snapshot)
        if restart:
            self.restarts += 1
            for fault in reference.faults:
                if fault.latched or (fault.active and fault.motion_inhibit):
                    self.carried_faults[fault.code] = fault
        elif self.base is not None and snapshot.sequence != (
                self.base.sequence + 1) % SEQUENCE_MODULUS:
            self.sequence_gaps += 1
        self.base_bad = None
        self._base_ref = None
        self.base = snapshot
        self.status.received(now, restart=restart)
        return self.evaluate(now)

    def on_aux(self, snapshot, now):
        """Feed one /base/aux_status message (an AuxSnapshot or Malformed)."""
        if isinstance(snapshot, AuxSnapshot):
            snapshot_bad = validate_aux(snapshot)
        else:
            snapshot_bad = snapshot
        if snapshot_bad is not None:
            self.aux_bad = snapshot_bad
            self._aux_ref = self.aux_snapshot or getattr(self, '_aux_ref', None)
            self.aux_snapshot = None
            self.aux.invalid(now)
            return self.evaluate(now)
        restart = self._is_restart(
            self.aux_snapshot or getattr(self, '_aux_ref', None), snapshot)
        if restart:
            self.restarts += 1
        self._aux_ref = None
        self.aux_bad = None
        self.aux_snapshot = snapshot
        self.aux.received(now, restart=restart)
        return self.evaluate(now)

    @staticmethod
    def _is_restart(previous, current):
        if previous is None:
            return False
        if previous.boot_id != current.boot_id:
            return True
        # Same boot id but the sequence went back by more than half the range is a wrap,
        # anything smaller going backwards is a restart.
        delta = (current.sequence - previous.sequence) % SEQUENCE_MODULUS
        return delta == 0 or delta > SEQUENCE_MODULUS // 2

    def set_config_error(self, reason):
        """Record a configuration error (fail closed). There is no way to clear it."""
        self.config_error = str(reason)

    def set_type_unavailable(self, value=True):
        """Record that the message package could not be imported (fail closed)."""
        self.type_unavailable = value

    # -- evaluation ---------------------------------------------------------
    def blockers(self, now):
        """List every reason motion must stay blocked right now."""
        found = []
        if self.type_unavailable:
            found.append(Blocker.MESSAGE_TYPE_UNAVAILABLE)
        if self.config_error is not None:
            found.append(Blocker.CONFIG_INVALID)
        state = self.status.state(now)
        if self.base_bad is not None:
            found.append(Blocker.STATUS_MALFORMED)
        elif state == 'NEVER':
            found.append(Blocker.NO_STATUS)
        elif state == 'STALE':
            found.append(Blocker.STATUS_STALE)
        elif state == 'NOT_LIVE':
            found.append(Blocker.STATUS_NOT_LIVE)
        if self.base is not None and state != 'NEVER':
            base = self.base
            if base.contract_version not in self.supported_versions:
                found.append(Blocker.CONTRACT_VERSION)
            if base.controller_state != 'READY':
                found.append(Blocker.CONTROLLER_NOT_READY)
            if base.safety_state != 'RUN':
                found.append(Blocker.SAFETY_STATE_NOT_RUN)
            if base.recovery_required:
                found.append(Blocker.RECOVERY_REQUIRED)
            if any(f.active and f.motion_inhibit for f in base.faults):
                found.append(Blocker.FAULT_INHIBITS_MOTION)
        if self.require_aux:
            aux_state = self.aux.state(now)
            if self.aux_bad is not None:
                found.append(Blocker.AUX_MALFORMED)
            elif aux_state == 'NEVER':
                found.append(Blocker.NO_AUX_STATUS)
            elif aux_state == 'STALE':
                found.append(Blocker.AUX_STALE)
            elif aux_state == 'NOT_LIVE':
                found.append(Blocker.AUX_NOT_LIVE)
            if (self.aux_snapshot is not None and
                    self.aux_snapshot.contract_version not in self.supported_versions):
                found.append(Blocker.CONTRACT_VERSION)
        return tuple(dict.fromkeys(found))

    def evaluate(self, now):
        """Re-evaluate. Any blocker engages the latch; only resume() releases it."""
        blockers = self.blockers(now)
        if blockers or self.carried_faults:
            self._latched = True
        lines = [BLOCKER_TEXT[b] for b in blockers if b != Blocker.FAULT_INHIBITS_MOTION]
        if self.base is not None:
            for fault in self.base.faults:
                if fault.active:
                    lines.append(self.registry.text_for(fault.code))
        for code in sorted(self.carried_faults):
            lines.append('Before restart: ' + self.registry.text_for(code))
        if self._latched and not blockers and self.carried_faults:
            summary = ('Stopped. The base controller restarted with fault(s) %s latched. '
                       'Check them; Resume acknowledges them.'
                       % ', '.join(str(c) for c in sorted(self.carried_faults)))
        elif self._latched and not blockers:
            summary = 'Stopped. Base is ready; press Resume to allow commands.'
        elif blockers:
            summary = lines[0] if lines else BLOCKER_TEXT[blockers[0]]
        else:
            summary = 'Base RUN. Commands allowed.'
        self._last_state = GateState(
            locked=self._latched, blockers=blockers,
            status_state=self.status.state(now), aux_state=self.aux.state(now),
            base=self.base, summary=summary, operator_lines=tuple(lines))
        return self._last_state

    def resume(self, now):
        """
        Explicit resume request. Returns (accepted, message).

        Clears the inhibit only when every blocker is gone. It never restarts motion.
        """
        state = self.evaluate(now)
        if state.blockers:
            return False, 'Resume refused: ' + '; '.join(
                BLOCKER_TEXT[b] for b in state.blockers)
        if not self._latched:
            return True, 'Already released.'
        acknowledged = sorted(self.carried_faults)
        self.carried_faults = {}
        self._latched = False
        self._last_state = self.evaluate(now)
        message = 'Inhibit cleared. Send a new command to move.'
        if acknowledged:
            message = 'Acknowledged fault(s) from before the controller restart: %s. %s' % (
                ', '.join(str(c) for c in acknowledged), message)
        return True, message
