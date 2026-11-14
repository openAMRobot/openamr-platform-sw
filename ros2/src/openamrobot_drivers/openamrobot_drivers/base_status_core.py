"""
Pure logic of the base status monitor (no ROS imports, injected clock).

The 2.0 base controller publishes /base/status (10 Hz, stale after 0.5 s) and
/base/aux_status (5 Hz, stale after 0.6 s), contract I8 rev 3. This module decides
one thing: may the command mux let velocity commands through?

Rules (P-03 rev20.6 item 30 A1, I8 rev 3 section 3.2 and 6):

* Freshness is measured from the LOCAL receipt time. The header stamp is never used.
* A first message after start-up (or after a gap longer than the stale time) may be a
  retained snapshot. It does not prove the publisher is alive. The stream is live only
  after a later message arrives within the stale time of the previous one.
* A boot identity change or a sequence number that goes backwards is a publisher
  restart. The stream is not live again until the next message.
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
SAFETY_STATES = ('LOCKED', 'RUN', 'CHARGING', 'CHARGER_FAULT')

SEQUENCE_MODULUS = 2 ** 32


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
}


@dataclass(frozen=True)
class Fault:
    """One active fault entry (group 18001 for base codes)."""

    code: int
    severity: str = 'UNKNOWN'
    motion_inhibit: bool = True
    active: bool = True


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


def validate_base(snapshot):
    """Return Malformed(reason) if the snapshot breaks the rules, else None."""
    if snapshot.controller_state not in CONTROLLER_STATES:
        return Malformed('controller_state out of range: %r' % (snapshot.controller_state,))
    if snapshot.safety_state not in SAFETY_STATES:
        return Malformed('safety_state out of range: %r' % (snapshot.safety_state,))
    if not isinstance(snapshot.sequence, int) or not 0 <= snapshot.sequence < SEQUENCE_MODULUS:
        return Malformed('sequence is not a uint32: %r' % (snapshot.sequence,))
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
        self.count = 0

    def received(self, now, restart=False):
        """Record a message. The stream is live if the previous one was fresh."""
        previous_fresh = self.last is not None and (now - self.last) < self.stale_s
        self.live = previous_fresh and not restart
        self.last = now
        self.count += 1

    def state(self, now):
        """Return NEVER, STALE, NOT_LIVE or LIVE."""
        if self.last is None:
            return 'NEVER'
        if now - self.last >= self.stale_s:
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
        self.sequence_gaps = 0
        self.restarts = 0
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
            self.base = None
            self.status.received(now, restart=True)
            return self.evaluate(now)
        restart = self._is_restart(self.base, snapshot)
        if restart:
            self.restarts += 1
        elif self.base is not None and snapshot.sequence != (
                self.base.sequence + 1) % SEQUENCE_MODULUS:
            self.sequence_gaps += 1
        self.base_bad = None
        self.base = snapshot
        self.status.received(now, restart=restart)
        return self.evaluate(now)

    def on_aux(self, snapshot, now):
        """Feed one /base/aux_status message (an AuxSnapshot or Malformed)."""
        if isinstance(snapshot, Malformed):
            self.aux_bad = snapshot
            self.aux_snapshot = None
            self.aux.received(now, restart=True)
            return self.evaluate(now)
        restart = self._is_restart(self.aux_snapshot, snapshot)
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

    def set_type_unavailable(self, value=True):
        """Record that the message package could not be imported (fail closed)."""
        self.type_unavailable = value

    # -- evaluation ---------------------------------------------------------
    def blockers(self, now):
        """List every reason motion must stay blocked right now."""
        found = []
        if self.type_unavailable:
            found.append(Blocker.MESSAGE_TYPE_UNAVAILABLE)
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
        if blockers:
            self._latched = True
        lines = [BLOCKER_TEXT[b] for b in blockers if b != Blocker.FAULT_INHIBITS_MOTION]
        if self.base is not None:
            for fault in self.base.faults:
                if fault.active:
                    lines.append(self.registry.text_for(fault.code))
        if self._latched and not blockers:
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
        self._latched = False
        self._last_state = self.evaluate(now)
        return True, 'Inhibit cleared. Send a new command to move.'
