"""Tests of the message adapter: field mapping and Malformed handling."""
from types import SimpleNamespace as NS

from openamrobot_drivers.base_status_adapter import aux_snapshot, base_snapshot, FieldMap
from openamrobot_drivers.base_status_core import AuxSnapshot, BaseSnapshot, Malformed


def msg(**over):
    fields = {
        'contract_version': 1, 'boot_id': [1, 2, 3, 4], 'sequence': 7,
        'controller_state': 'READY', 'safety_state': 'RUN', 'recovery_required': False,
        'faults': []}
    fields.update(over)
    return NS(**fields)


def test_string_enums_map_without_a_table():
    snap = base_snapshot(msg())
    assert isinstance(snap, BaseSnapshot)
    assert snap.controller_state == 'READY' and snap.safety_state == 'RUN'
    assert snap.boot_id == (1, 2, 3, 4)


def test_integer_enum_without_table_is_malformed_not_a_guess():
    result = base_snapshot(msg(controller_state=2))
    assert isinstance(result, Malformed)
    assert 'enum table' in result.reason


def test_integer_enum_with_table():
    fmap = FieldMap(enums={'controller_state': {2: 'READY'}, 'safety_state': {1: 'RUN'}})
    snap = base_snapshot(msg(controller_state=2, safety_state=1), fmap)
    assert (snap.controller_state, snap.safety_state) == ('READY', 'RUN')


def test_unmapped_integer_value_is_malformed():
    fmap = FieldMap(enums={'controller_state': {2: 'READY'}, 'safety_state': {1: 'RUN'}})
    assert isinstance(base_snapshot(msg(controller_state=9, safety_state=1), fmap), Malformed)


def test_missing_field_is_malformed():
    bad = msg()
    del bad.sequence
    result = base_snapshot(bad)
    assert isinstance(result, Malformed) and 'sequence' in result.reason


def test_renamed_fields_through_the_map():
    fmap = FieldMap(fields={'sequence': 'seq'})
    renamed = msg()
    renamed.seq = renamed.sequence
    del renamed.sequence
    assert base_snapshot(renamed, fmap).sequence == 7


def test_faults_are_mapped():
    entry = NS(code=18001, severity='ERROR', motion_inhibit=True, active=True)
    snap = base_snapshot(msg(controller_state='FAULTED', faults=[entry]))
    assert snap.faults[0].code == 18001 and snap.faults[0].motion_inhibit


def test_fault_entry_missing_inhibit_flag_is_malformed():
    entry = NS(code=18001, severity='ERROR')
    assert isinstance(base_snapshot(msg(faults=[entry])), Malformed)


def test_aux_snapshot():
    snap = aux_snapshot(msg())
    assert isinstance(snap, AuxSnapshot) and snap.sequence == 7
    bad = msg()
    del bad.boot_id
    assert isinstance(aux_snapshot(bad), Malformed)


def test_bool_is_not_an_enum_value():
    assert isinstance(base_snapshot(msg(controller_state=True)), Malformed)
