"""
Map /base/status and /base/aux_status messages onto the gate's snapshots.

The .msg files of openamr_base_msgs are not published yet (openamrobot-interfaces #30:
the draft is due 10 Nov). Field names and enum values below are therefore DEFAULTS that
follow the I8 rev 3 groups, not a contract. Every one can be overridden by a node
parameter, and each is marked TODO(Evans). A message that lacks a mapped field is
Malformed (a failure), never UNKNOWN and never nominal.
"""
from openamrobot_drivers.base_status_core import (
    AuxSnapshot, BaseSnapshot, Fault, Malformed)

# TODO(Evans): field names per the #30 draft PR.
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
}

# TODO(Evans): enum values per the #30 draft PR. The state NAMES come from I8 rev 3 section
# 3.1 and platform-fw #17; the numbers are NOT known. Integer enums are refused until the
# table is given, so an unmapped number is Malformed instead of a guess.
DEFAULT_ENUMS = {
    'controller_state': {},
    'safety_state': {},
}


class FieldMap:
    """Field names and integer enum tables, overridable by parameters."""

    def __init__(self, fields=None, enums=None):
        self.fields = dict(DEFAULT_FIELDS)
        self.fields.update(fields or {})
        self.enums = {k: dict(v) for k, v in DEFAULT_ENUMS.items()}
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


def _get(msg, name):
    if not hasattr(msg, name):
        raise ValueError('missing field %s' % name)
    return getattr(msg, name)


def _boot_id(value):
    """Return the boot identity as a hashable, comparable value."""
    try:
        return tuple(value)
    except TypeError:
        return value


def base_snapshot(msg, fmap=None):
    """Return a BaseSnapshot, or Malformed with the reason."""
    fmap = fmap or FieldMap()
    f = fmap.fields
    try:
        faults = []
        for entry in _get(msg, f['faults']):
            faults.append(Fault(
                code=int(_get(entry, f['fault_code'])),
                severity=str(getattr(entry, f['fault_severity'], 'UNKNOWN')),
                motion_inhibit=bool(_get(entry, f['fault_motion_inhibit'])),
                active=bool(getattr(entry, f['fault_active'], True))))
        return BaseSnapshot(
            contract_version=int(_get(msg, f['contract_version'])),
            boot_id=_boot_id(_get(msg, f['boot_id'])),
            sequence=int(_get(msg, f['sequence'])),
            controller_state=fmap.enum_name(
                'controller_state', _get(msg, f['controller_state'])),
            safety_state=fmap.enum_name('safety_state', _get(msg, f['safety_state'])),
            recovery_required=bool(_get(msg, f['recovery_required'])),
            faults=tuple(faults))
    except (ValueError, TypeError) as err:
        return Malformed(str(err))


def aux_snapshot(msg, fmap=None):
    """Return an AuxSnapshot, or Malformed with the reason."""
    fmap = fmap or FieldMap()
    f = fmap.fields
    try:
        return AuxSnapshot(
            contract_version=int(_get(msg, f['contract_version'])),
            boot_id=_boot_id(_get(msg, f['boot_id'])),
            sequence=int(_get(msg, f['sequence'])))
    except (ValueError, TypeError) as err:
        return Malformed(str(err))
