"""
Regression tests for the review findings on platform-sw #132 and the fw2 host bridge rules.

Each test names the finding it covers. They use the pure gate and the adapter, no ROS.
"""
from types import SimpleNamespace as NS

from openamrobot_drivers.base_status_adapter import (
    aux_snapshot, base_snapshot, FieldMap, I8_REV4_PROPOSED)
from openamrobot_drivers.base_status_core import (
    AuxSnapshot, BaseGate, BaseSnapshot, Blocker, Fault, Freshness, Malformed,
    validate_aux, validate_base)
import pytest

BOOT = 0x5EED0001
REV4 = FieldMap(profile=I8_REV4_PROPOSED)


def base(seq, boot=BOOT, ctl='READY', safety='RUN', recovery=False, faults=(), version=1):
    return BaseSnapshot(version, boot, seq, ctl, safety, recovery, tuple(faults))


def aux(seq, boot=BOOT, version=1):
    return AuxSnapshot(version, boot, seq)


def raw_aux(**over):
    fields = {'contract_version': 1, 'boot_id': BOOT, 'sequence': 1}
    fields.update(over)
    return NS(**fields)


def raw_status(**over):
    fields = {'contract_version': 1, 'boot_id': BOOT, 'sequence': 1, 'state': 2,
              'operating_state': 2, 'recovery': 1, 'faults': []}
    fields.update(over)
    return NS(**fields)


def live_gate(t=100.0, seq=1):
    """Return a gate with live status and aux, released by one Resume."""
    gate = BaseGate()
    for i in range(4):
        gate.on_status(base(seq + i), t + 0.1 * i)
        gate.on_aux(aux(seq + i), t + 0.1 * i)
    t += 0.3
    assert gate.resume(t)[0]
    assert not gate.evaluate(t).locked
    return gate, t, seq + 3


BAD_IDENTITY = [
    ('missing', None),
    ('bool true', True),
    ('bool false', False),
    ('negative', -1),
    ('float', 1.0),
    ('string', '7'),
    ('too large', 2 ** 32),
]


# --- review finding 3 (P1): bool or out-of-range aux sequence -------------------------
@pytest.mark.parametrize('label,value', BAD_IDENTITY)
def test_adapter_rejects_bad_aux_sequence(label, value):
    msg = raw_aux(sequence=value)
    if value is None:
        del msg.sequence
    assert isinstance(aux_snapshot(msg), Malformed), label


@pytest.mark.parametrize('label,value', BAD_IDENTITY)
def test_adapter_rejects_bad_status_sequence(label, value):
    msg = raw_status(sequence=value)
    if value is None:
        del msg.sequence
    assert isinstance(base_snapshot(msg, REV4), Malformed), label


@pytest.mark.parametrize('label,value', BAD_IDENTITY)
def test_core_rejects_bad_sequence_even_without_the_adapter(label, value):
    assert isinstance(validate_aux(aux(value)), Malformed), label
    assert isinstance(validate_base(base(value)), Malformed), label


def test_malformed_first_aux_sample_does_not_count_toward_liveness():
    """Reviewer reproduction: sequence=True then valid sequence 2 within 0.1 s."""
    gate = BaseGate()
    for i in range(1, 5):
        gate.on_status(base(i), 10.0 + 0.1 * i)
    gate.on_aux(aux(True), 10.40)
    state = gate.on_aux(aux(2), 10.45)
    assert gate.aux.state(10.45) != 'LIVE'
    assert Blocker.AUX_NOT_LIVE in state.blockers
    assert not gate.resume(10.45)[0]


def test_malformed_first_status_sample_does_not_count_toward_liveness():
    gate = BaseGate(require_aux=False)
    gate.on_status(Malformed('bad'), 10.0)
    state = gate.on_status(base(2), 10.05)
    assert Blocker.STATUS_NOT_LIVE in state.blockers


def test_malformed_aux_from_the_raw_adapter_path():
    gate = BaseGate()
    for i in range(1, 5):
        gate.on_status(base(i), 10.0 + 0.1 * i)
    gate.on_aux(aux_snapshot(raw_aux(sequence=True)), 10.40)
    gate.on_aux(aux_snapshot(raw_aux(sequence=2)), 10.45)
    assert gate.aux.state(10.45) != 'LIVE'


# --- boot id (comment findings, both reviewers) --------------------------------------
BAD_BOOT_ID = BAD_IDENTITY + [('zero is UNKNOWN', 0), ('list', [1, 2, 3, 4])]


@pytest.mark.parametrize('label,value', BAD_BOOT_ID)
def test_bad_boot_id_is_malformed_for_both_streams(label, value):
    status, aux_msg = raw_status(boot_id=value), raw_aux(boot_id=value)
    if value is None:
        del status.boot_id
        del aux_msg.boot_id
    assert isinstance(base_snapshot(status, REV4), Malformed), label
    assert isinstance(aux_snapshot(aux_msg), Malformed), label
    assert isinstance(validate_base(base(1, boot=value)), Malformed), label
    assert isinstance(validate_aux(aux(1, boot=value)), Malformed), label


def test_missing_boot_id_blocks_readiness():
    gate = BaseGate(require_aux=False)
    for i in range(1, 5):
        msg = raw_status(sequence=i)
        del msg.boot_id
        state = gate.on_status(base_snapshot(msg, REV4), 10.0 + 0.1 * i)
    assert Blocker.STATUS_MALFORMED in state.blockers
    assert not gate.resume(10.5)[0]


def test_reused_boot_id_with_sequence_restart_is_a_restart():
    gate, t, seq = live_gate()
    gate.on_status(base(0), t + 0.05)
    assert gate.restarts == 1
    assert gate.evaluate(t + 0.05).locked


def test_boot_id_change_on_aux_alone_is_a_restart_and_locks():
    gate, t, seq = live_gate()
    gate.on_status(base(seq + 1), t + 0.05)
    state = gate.on_aux(aux(seq + 1, boot=BOOT + 1), t + 0.05)
    assert gate.restarts == 1
    assert state.locked and Blocker.AUX_NOT_LIVE in state.blockers


@pytest.mark.parametrize('label,value', [('bool', True), ('negative', -1), ('float', 1.0),
                                         ('too large', 2 ** 16), ('missing', None)])
def test_bad_contract_version_is_malformed(label, value):
    msg = raw_status(contract_version=value)
    if value is None:
        del msg.contract_version
    assert isinstance(base_snapshot(msg, REV4), Malformed), label


@pytest.mark.parametrize('version', [0, 2, 65535])
def test_unsupported_contract_version_degrades_safely(version):
    gate = BaseGate()
    for i in range(1, 5):
        gate.on_status(base(i, version=version), 10.0 + 0.1 * i)
        gate.on_aux(aux(i), 10.0 + 0.1 * i)
    state = gate.evaluate(10.4)
    assert state.locked and Blocker.CONTRACT_VERSION in state.blockers
    assert not gate.resume(10.4)[0]


def test_contract_version_change_while_released_locks():
    gate, t, seq = live_gate()
    state = gate.on_status(base(seq + 1, version=2), t + 0.05)
    assert state.locked and Blocker.CONTRACT_VERSION in state.blockers


# --- monotonic freshness (comment finding P2) ----------------------------------------
def test_backward_clock_step_makes_the_stream_stale_not_live():
    """Reviewer reproduction: receive at 100.0 and 100.1, evaluate at 90.2."""
    fresh = Freshness(0.5)
    fresh.received(100.0)
    fresh.received(100.1)
    assert fresh.state(100.2) == 'LIVE'
    assert fresh.state(90.2) == 'STALE'


def test_message_received_before_the_previous_one_is_not_live():
    fresh = Freshness(0.5)
    fresh.received(100.0)
    fresh.received(90.0)
    assert fresh.state(90.1) != 'LIVE'


def test_backward_clock_step_locks_a_released_gate():
    gate, t, seq = live_gate()
    state = gate.evaluate(t - 10.0)
    assert state.locked


# --- latched faults and restarts -----------------------------------------------------
def test_restart_never_clears_a_latched_fault():
    gate, t, seq = live_gate()
    fault = Fault(18004, severity='FAULT', motion_inhibit=True, active=True, latched=True)
    gate.on_status(base(seq + 1, ctl='FAULTED', safety='LOCKED', faults=[fault]), t + 0.1)
    # the controller reboots and reports a clean state under a new boot id
    t += 0.2
    for i in range(5):
        state = gate.on_status(base(i + 1, boot=BOOT + 1), t + 0.1 * i)
        gate.on_aux(aux(i + 1, boot=BOOT + 1), t + 0.1 * i)
    t += 0.4
    state = gate.evaluate(t)
    assert state.locked
    assert any('18004' in line for line in state.operator_lines)
    assert 18004 in gate.carried_faults
    # only an explicit operator Resume acknowledges it, never the restart itself
    ok, message = gate.resume(t)
    assert ok and '18004' in message
    assert not gate.carried_faults


def test_restart_without_latched_fault_still_needs_resume():
    gate, t, seq = live_gate()
    for i in range(5):
        gate.on_status(base(i + 1, boot=BOOT + 1), t + 0.1 * (i + 1))
        gate.on_aux(aux(i + 1, boot=BOOT + 1), t + 0.1 * (i + 1))
    assert gate.evaluate(t + 0.5).locked


# --- loss can only reduce readiness (property test, fixed seed) ----------------------
def test_loss_or_staleness_never_raises_readiness():
    import random
    rng = random.Random(132)
    for _ in range(200):
        gate = BaseGate()
        t, seq, aseq, boot = 0.0, 0, 0, BOOT
        prev_locked = True
        for _ in range(80):
            action = rng.choice(['status', 'aux', 'both', 'both', 'wait', 'drop', 'bad',
                                 'restart', 'resume', 'resume'])
            t += rng.choice([0.05, 0.1, 0.1, 0.2])
            resumed = False
            if action in ('status', 'both'):
                seq += 1
                gate.on_status(base(seq, boot=boot), t)
            if action in ('aux', 'both'):
                aseq += 1
                gate.on_aux(aux(aseq, boot=boot), t)
            if action == 'drop':
                t += rng.choice([0.5, 0.6, 1.0])
            if action == 'bad':
                gate.on_aux(Malformed('x'), t) if rng.random() < 0.5 else \
                    gate.on_status(Malformed('x'), t)
            if action == 'restart':
                boot += 1
                seq, aseq = 0, 0
            if action == 'resume':
                resumed = gate.resume(t)[0]
            state = gate.evaluate(t)
            if prev_locked and not state.locked:
                assert resumed, 'lock released without an accepted Resume'
            if action in ('wait', 'drop', 'bad', 'restart') and prev_locked:
                assert state.locked
            if gate.status.state(t) != 'LIVE' or gate.aux.state(t) != 'LIVE':
                assert state.locked
            prev_locked = state.locked


def test_retained_message_alone_is_never_live_for_aux():
    gate = BaseGate()
    gate.on_aux(aux(1), 10.0)
    assert gate.aux.state(10.1) == 'NOT_LIVE'
    assert gate.aux.state(11.0) == 'STALE'


def test_monitor_config_requires_aux_and_uses_the_rev4_profile():
    import os
    import yaml
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'base_status_monitor.yaml')
    with open(path) as handle:
        params = yaml.safe_load(handle)['base_status_monitor']['ros__parameters']
    assert params['require_aux'] is True
    assert params['allow_aux_optional'] is False
    assert params['field_profile'] == I8_REV4_PROPOSED['name']
    assert params['status_stale_s'] == 0.5 and params['aux_stale_s'] == 0.6
