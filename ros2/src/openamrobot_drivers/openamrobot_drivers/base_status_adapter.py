"""
Map /base/status and /base/aux_status messages onto the gate's snapshots.

This is the ONLY module that knows I8 field names and enum numbers. A later change to the
message set touches this file and nothing else.

Two kinds of field are read here:

* FROZEN in I8 (6 Nov 2026, kept in proposal rev 4): ``contract_version`` (uint16),
  ``boot_id`` (uint32) and ``sequence`` (uint32). They are validated strictly: a missing,
  non-integer, boolean, negative or out-of-range value makes the whole message Malformed.
  ``boot_id`` 0 is UNKNOWN and cannot identify a boot, so it is Malformed as well.
* PROPOSED, not approved: every other field name and every enum number. The table
  ``I8_REV4_PROPOSED`` follows openamrobot-interfaces #38 (I8 proposal rev 4, 10 Nov 2026).
  It is a working representation for tests and the bench, not a contract. TODO(Evans):
  confirm or replace when I8 decisions 1, 2 and 4 close.

A message that lacks a mapped field, or carries a value outside its table, is Malformed (a
failure), never UNKNOWN and never nominal.
"""
from openamrobot_drivers.base_status_core import (
    AuxSnapshot, BaseSnapshot, Fault, Malformed, validate_aux, validate_base)

UINT16_MAX = 2 ** 16 - 1
UINT32_MAX = 2 ** 32 - 1

# Field names of the first draft (strings for states, booleans for flags). Kept for the
# existing tests and for fixtures that carry symbolic names.
DEFAULT_FIELDS = {
    'contract_version': 'contract_version',
    'boot_id': 'boot_id',
    'sequence': 'sequence',
    'controller_state': 'controller_state',
    'safety_state': 'safety_state',
    'recovery_required': 'recovery_required',
    'faults': 'faults',
    'fault_code': 'code',
    'fault_severity': 'severity',
    'fault_motion_inhibit': 'motion_inhibit',
    'fault_active': 'active',
    'fault_lifecycle': None,
    'fault_condition_active': None,
    'fault_recovery_required': None,
}

DEFAULT_ENUMS = {
    'controller_state': {},
    'safety_state': {},
    'recovery': {},
    'fault_lifecycle': {},
    'observation': {},
}

# PROPOSED, not approved: I8 proposal rev 4 as drafted in openamrobot-interfaces #38.
# BaseStatus.state, BaseStatus.operating_state, BaseStatus.recovery, BaseFault.lifecycle,
# BaseFault.condition_active / recovery_required (OBS_*) and BaseFault.inhibits_motion.
# TODO(Evans): I8 decisions 1 (message set), 2 (state semantics) and 4 (fault lifecycle).
I8_REV4_PROPOSED = {
    'name': 'i8_rev4_proposed',
    'fields': {
        'controller_state': 'state',
        'safety_state': 'operating_state',
        'recovery_required': 'recovery',
        'fault_severity': 'severity',
        'fault_motion_inhibit': 'inhibits_motion',
        'fault_active': None,
        'fault_lifecycle': 'lifecycle',
        'fault_condition_active': 'condition_active',
        'fault_recovery_required': 'recovery_required',
    },
    'enums': {
        'controller_state': {0: 'UNKNOWN', 1: 'INITIALIZING', 2: 'READY', 3: 'DEGRADED',
                             4: 'FAULTED'},
        'safety_state': {0: 'UNKNOWN', 1: 'LOCKED', 2: 'RUN', 3: 'INHIBITED', 4: 'CHARGING',
                         5: 'CHARGER_FAULT'},
        'recovery': {0: 'UNKNOWN', 1: 'NOT_REQUIRED', 2: 'REQUIRED'},
        'fault_lifecycle': {0: 'UNKNOWN', 1: 'ACTIVE', 2: 'LATCHED', 3: 'ACKNOWLEDGED',
                            4: 'CLEARED'},
        'observation': {0: 'UNKNOWN', 1: 'NO', 2: 'YES'},
        'fault_severity': {0: 'UNKNOWN', 1: 'WARNING', 2: 'FAULT'},
    },
}

PROFILES = {'draft': None, I8_REV4_PROPOSED['name']: I8_REV4_PROPOSED}

# A fault in one of these lifecycle states is held by the controller: a restart must not be
# read as clearing it (I8 rev 4 rules). UNKNOWN counts as held: 0 never reads as benign.
HELD_LIFECYCLES = ('UNKNOWN', 'LATCHED', 'ACKNOWLEDGED')


class FieldMap:
    """Field names and integer enum tables, overridable by parameters."""

    def __init__(self, fields=None, enums=None, profile=None):
        self.fields = dict(DEFAULT_FIELDS)
        self.enums = {k: dict(v) for k, v in DEFAULT_ENUMS.items()}
        self.profile = 'draft'
        if profile is not None:
            self.profile = profile['name']
            self.fields.update(profile['fields'])
            for key, table in profile['enums'].items():
                self.enums[key] = dict(table)
        self.fields.update(fields or {})
        for key, table in (enums or {}).items():
            self.enums[key] = {int(n): str(v) for n, v in table.items()}

    def enum_name(self, key, value):
        """Return the symbolic name of an enum field, or raise ValueError."""
        if isinstance(value, str):
            return value.upper()
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError('%s has type %s' % (key, type(value).__name__))
        table = self.enums.get(key, {})
        if value not in table:
            raise ValueError('%s value %d has no entry in the enum table' % (key, value))
        return table[value]


def strict_uint(name, value, maximum):
    """Return value if it is a real integer in 0..maximum, else raise ValueError."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError('%s is not an integer: %r' % (name, value))
    if not 0 <= value <= maximum:
        raise ValueError('%s out of range: %r' % (name, value))
    return value


def _get(msg, name):
    if name is None or not hasattr(msg, name):
        raise ValueError('missing field %s' % name)
    return getattr(msg, name)


def _identity(msg, f):
    """Read the frozen identity fields. Raises ValueError on any contract break."""
    version = strict_uint('contract_version', _get(msg, f['contract_version']), UINT16_MAX)
    boot_id = strict_uint('boot_id', _get(msg, f['boot_id']), UINT32_MAX)
    if boot_id == 0:
        raise ValueError('boot_id is 0 (UNKNOWN): the boot cannot be identified')
    sequence = strict_uint('sequence', _get(msg, f['sequence']), UINT32_MAX)
    return version, boot_id, sequence


def _flag(fmap, msg, field_key, enum_key):
    """Read a flag that is either a bool (draft) or an OBS_* / enum value (rev 4)."""
    value = _get(msg, fmap.fields[field_key])
    if isinstance(value, bool):
        return value
    return fmap.enum_name(enum_key, value)


def _fault(entry, fmap):
    f = fmap.fields
    code = strict_uint('fault code', _get(entry, f['fault_code']), UINT16_MAX)
    inhibit = _get(entry, f['fault_motion_inhibit'])
    if not isinstance(inhibit, bool):
        raise ValueError('fault %d motion inhibit flag is not a bool: %r' % (code, inhibit))
    severity = getattr(entry, f['fault_severity'], 'UNKNOWN')
    if not isinstance(severity, str):
        severity = fmap.enum_name('fault_severity', severity) \
            if 'fault_severity' in fmap.enums else str(severity)
    if f['fault_lifecycle'] is None:
        # draft layout: one 'active' flag, no lifecycle
        active = getattr(entry, f['fault_active'], True)
        if not isinstance(active, bool):
            raise ValueError('fault %d active flag is not a bool: %r' % (code, active))
        return Fault(code=code, severity=severity, motion_inhibit=inhibit, active=active,
                     latched=False)
    lifecycle = fmap.enum_name('fault_lifecycle', _get(entry, f['fault_lifecycle']))
    condition = fmap.enum_name(
        'observation', _get(entry, f['fault_condition_active']))
    recovery = fmap.enum_name(
        'observation', _get(entry, f['fault_recovery_required']))
    # Conservative reading: anything not positively CLEARED with the condition gone is active.
    active = not (lifecycle == 'CLEARED' and condition == 'NO')
    latched = lifecycle in HELD_LIFECYCLES or recovery != 'NO'
    return Fault(code=code, severity=severity, motion_inhibit=inhibit, active=active,
                 latched=latched)


def _recovery_required(fmap, msg):
    value = _flag(fmap, msg, 'recovery_required', 'recovery')
    if isinstance(value, bool):
        return value
    # UNKNOWN (0) is never read as "not required".
    return value != 'NOT_REQUIRED'


def base_snapshot(msg, fmap=None):
    """Return a BaseSnapshot, or Malformed with the reason."""
    fmap = fmap or FieldMap()
    f = fmap.fields
    try:
        version, boot_id, sequence = _identity(msg, f)
        faults = tuple(_fault(entry, fmap) for entry in _get(msg, f['faults']))
        snapshot = BaseSnapshot(
            contract_version=version,
            boot_id=boot_id,
            sequence=sequence,
            controller_state=fmap.enum_name(
                'controller_state', _get(msg, f['controller_state'])),
            safety_state=fmap.enum_name('safety_state', _get(msg, f['safety_state'])),
            recovery_required=_recovery_required(fmap, msg),
            faults=faults)
    except (ValueError, TypeError) as err:
        return Malformed(str(err))
    return validate_base(snapshot) or snapshot


def aux_snapshot(msg, fmap=None):
    """Return an AuxSnapshot, or Malformed with the reason."""
    fmap = fmap or FieldMap()
    try:
        version, boot_id, sequence = _identity(msg, fmap.fields)
        snapshot = AuxSnapshot(contract_version=version, boot_id=boot_id, sequence=sequence)
    except (ValueError, TypeError) as err:
        return Malformed(str(err))
    return validate_aux(snapshot) or snapshot
