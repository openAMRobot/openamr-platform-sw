"""Tests of the base gate: freshness, latch, fault text and the resume rule."""
from openamrobot_drivers.base_status_core import (
    AuxSnapshot, BaseGate, BaseSnapshot, Blocker, Fault, FaultRegistry, Freshness, Malformed,
    validate_base)
import pytest

BOOT = (1, 2, 3, 4)


def base(seq=0, boot=BOOT, ctl='READY', safety='RUN', recovery=False, faults=(), version=1):
    return BaseSnapshot(version, boot, seq, ctl, safety, recovery, tuple(faults))


def aux(seq=0, boot=BOOT, version=1):
    return AuxSnapshot(version, boot, seq)


class Rig:
    """Drives a gate with a fake clock: status at 10 Hz, aux at 5 Hz."""

    def __init__(self, **gate_kwargs):
        self.gate = BaseGate(**gate_kwargs)
        self.t = 100.0
        self.seq = 0
        self.aseq = 0

    def tick(self, dt=0.1, **status_kwargs):
        self.t += dt
        self.seq += 1
        return self.gate.on_status(base(seq=self.seq, **status_kwargs), self.t)

    def aux_tick(self):
        self.aseq += 1
        return self.gate.on_aux(aux(seq=self.aseq), self.t)

    def run_ok(self, seconds=1.0, **status_kwargs):
        for _ in range(int(seconds * 10)):
            self.tick(**status_kwargs)
            if self.seq % 2 == 0:
                self.aux_tick()
        return self.gate.evaluate(self.t)


def test_boot_is_locked_with_no_status():
    gate = BaseGate()
    state = gate.evaluate(0.0)
    assert state.locked
    assert Blocker.NO_STATUS in state.blockers
    assert Blocker.NO_AUX_STATUS in state.blockers


def test_fresh_run_still_locked_until_explicit_resume():
    rig = Rig()
    state = rig.run_ok()
    assert state.blockers == ()
    assert state.locked, 'nothing resumes by itself'
    ok, _ = rig.gate.resume(rig.t)
    assert ok
    assert not rig.gate.evaluate(rig.t).locked


def test_resume_refused_with_reason_while_blocked():
    rig = Rig()
    rig.run_ok(safety='LOCKED')
    ok, message = rig.gate.resume(rig.t)
    assert not ok
    assert 'LOCKED' in message
    assert rig.gate.evaluate(rig.t).locked


def test_retained_message_alone_is_not_live():
    gate = BaseGate(require_aux=False)
    state = gate.on_status(base(), 10.0)
    assert Blocker.STATUS_NOT_LIVE in state.blockers
    ok, _ = gate.resume(10.1)
    assert not ok
    # a retained message and then silence: stale, never live
    assert Blocker.STATUS_STALE in gate.evaluate(11.0).blockers


def test_retained_then_live_within_threshold():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 10.0)
    state = gate.on_status(base(seq=2), 10.1)
    assert state.blockers == ()


def test_second_message_after_a_long_gap_is_not_live():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 10.0)
    state = gate.on_status(base(seq=2), 10.6)
    assert Blocker.STATUS_NOT_LIVE in state.blockers


def test_stale_status_locks_and_stays_locked_after_return():
    rig = Rig()
    rig.run_ok()
    assert rig.gate.resume(rig.t)[0]
    rig.t += 0.49
    assert not rig.gate.evaluate(rig.t).locked
    rig.t += 0.02            # 0.51 s since the last message
    state = rig.gate.evaluate(rig.t)
    assert state.locked and Blocker.STATUS_STALE in state.blockers
    # the publisher comes back, fresh RUN: still locked until Resume
    rig.run_ok(0.5)
    assert rig.gate.evaluate(rig.t).locked
    assert rig.gate.resume(rig.t)[0]


def test_stale_boundary_uses_0_5_s():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 0.0)
    gate.on_status(base(seq=2), 0.1)
    assert gate.status.state(0.59) == 'LIVE'
    assert gate.status.state(0.6) == 'STALE'


def test_aux_stale_uses_0_6_s():
    rig = Rig()
    rig.run_ok()
    assert rig.gate.resume(rig.t)[0]
    # status keeps arriving, aux stops
    for _ in range(5):
        rig.tick()
    state = rig.gate.evaluate(rig.t)
    assert Blocker.AUX_STALE not in state.blockers      # 0.5 s, under 0.6 s
    assert not state.locked
    rig.tick()
    rig.tick()                # 0.7 s without aux, clear of float rounding at 0.6
    state = rig.gate.evaluate(rig.t)
    assert Blocker.AUX_STALE in state.blockers
    assert state.locked


@pytest.mark.parametrize('safety', ['LOCKED', 'CHARGING', 'CHARGER_FAULT'])
def test_safety_state_not_run_locks(safety):
    rig = Rig()
    rig.run_ok()
    assert rig.gate.resume(rig.t)[0]
    rig.tick(safety=safety)
    state = rig.gate.evaluate(rig.t)
    assert state.locked
    assert Blocker.SAFETY_STATE_NOT_RUN in state.blockers
    assert 'LOCKED' in state.summary or 'RUN' in state.summary


@pytest.mark.parametrize('ctl', ['UNKNOWN', 'INITIALIZING', 'DEGRADED', 'FAULTED'])
def test_controller_not_ready_locks(ctl):
    rig = Rig()
    state = rig.run_ok(ctl=ctl)
    assert state.locked
    assert Blocker.CONTROLLER_NOT_READY in state.blockers


def test_recovery_required_locks_even_if_run():
    rig = Rig()
    state = rig.run_ok(recovery=True)
    assert Blocker.RECOVERY_REQUIRED in state.blockers
    assert not rig.gate.resume(rig.t)[0]


def test_fault_locks_and_gives_registry_text():
    registry = FaultRegistry({18001: {'name': 'CMD_TIMEOUT', 'text': 'No /cmd_vel for 200 ms.'}})
    rig = Rig(registry=registry)
    state = rig.run_ok(ctl='FAULTED', faults=[Fault(18001, 'ERROR', True)])
    assert state.locked
    assert Blocker.FAULT_INHIBITS_MOTION in state.blockers
    assert any('18001' in line and 'No /cmd_vel for 200 ms.' in line
               for line in state.operator_lines)
    assert 'No /cmd_vel for 200 ms.' in state.summary or 'FAULTED' in state.summary or \
        'READY' in state.summary


def test_unknown_fault_code_stays_reportable():
    rig = Rig()
    state = rig.run_ok(ctl='DEGRADED', faults=[Fault(18999, 'WARN', True)])
    assert any('18999' in line and 'not in the fault registry' in line
               for line in state.operator_lines)


def test_non_inhibiting_fault_is_reported_but_does_not_block_by_itself():
    rig = Rig()
    state = rig.run_ok(ctl='READY', faults=[Fault(18002, 'INFO', False)])
    assert Blocker.FAULT_INHIBITS_MOTION not in state.blockers
    assert any('18002' in line for line in state.operator_lines)


def test_fault_cleared_does_not_release_the_latch():
    rig = Rig()
    rig.run_ok(ctl='FAULTED', faults=[Fault(18001)])
    state = rig.run_ok(1.0)
    assert state.blockers == () and state.locked
    assert rig.gate.resume(rig.t)[0]


def test_resume_never_published_state_is_motion():
    # resume only flips the lock flag; the gate has no velocity output at all
    assert not hasattr(BaseGate(), 'cmd_vel')


def test_publisher_restart_new_boot_id_needs_a_new_resume():
    rig = Rig()
    rig.run_ok()
    assert rig.gate.resume(rig.t)[0]
    rig.t += 0.1
    state = rig.gate.on_status(base(seq=5, boot=(9, 9, 9, 9)), rig.t)
    assert rig.gate.restarts == 1
    assert Blocker.STATUS_NOT_LIVE in state.blockers
    assert state.locked


def test_sequence_going_backwards_is_a_restart():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=100), 0.0)
    gate.on_status(base(seq=101), 0.1)
    state = gate.on_status(base(seq=3), 0.2)
    assert gate.restarts == 1
    assert state.locked


def test_sequence_gap_counted_not_blocking():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 0.0)
    gate.on_status(base(seq=2), 0.1)
    state = gate.on_status(base(seq=7), 0.2)
    assert gate.sequence_gaps == 1
    assert state.blockers == ()


def test_sequence_wraps_at_uint32():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=2 ** 32 - 1), 0.0)
    gate.on_status(base(seq=0), 0.1)
    assert gate.restarts == 0 and gate.sequence_gaps == 0


def test_malformed_is_a_failure_not_unknown_not_nominal():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 0.0)
    gate.on_status(base(seq=2), 0.1)
    state = gate.on_status(Malformed('bad enum'), 0.2)
    assert Blocker.STATUS_MALFORMED in state.blockers
    assert Blocker.CONTROLLER_NOT_READY not in state.blockers   # not mapped to UNKNOWN
    # the next valid message clears the malformed blocker, but the latch stays engaged
    state = gate.on_status(base(seq=4), 0.3)
    assert Blocker.STATUS_MALFORMED not in state.blockers
    assert state.locked


def test_validator_rejects_out_of_range_and_contradiction():
    assert isinstance(validate_base(base(ctl='BOGUS')), Malformed)
    assert isinstance(validate_base(base(safety='BOGUS')), Malformed)
    assert isinstance(validate_base(base(seq=-1)), Malformed)
    assert isinstance(validate_base(base(seq=2 ** 32)), Malformed)
    assert isinstance(validate_base(base(faults=[Fault(18001)])), Malformed)
    assert validate_base(base(ctl='FAULTED', faults=[Fault(18001)])) is None


def test_contract_version_mismatch_blocks_status_and_aux():
    gate = BaseGate(supported_versions=(1,))
    gate.on_status(base(seq=1, version=2), 0.0)
    state = gate.on_status(base(seq=2, version=2), 0.1)
    assert Blocker.CONTRACT_VERSION in state.blockers
    gate = BaseGate(supported_versions=(1,))
    gate.on_aux(aux(version=2), 0.0)
    assert Blocker.CONTRACT_VERSION in gate.evaluate(0.0).blockers


def test_type_unavailable_fails_closed():
    gate = BaseGate(require_aux=False)
    gate.on_status(base(seq=1), 0.0)
    gate.on_status(base(seq=2), 0.1)
    gate.set_type_unavailable()
    assert Blocker.MESSAGE_TYPE_UNAVAILABLE in gate.evaluate(0.1).blockers


def test_aux_not_required_profile():
    rig = Rig(require_aux=False)
    for _ in range(5):
        rig.tick()
    state = rig.gate.evaluate(rig.t)
    assert state.blockers == ()


def test_header_stamp_is_ignored():
    # the snapshot has no stamp field at all: freshness can only use local time
    assert 'stamp' not in BaseSnapshot.__dataclass_fields__


def test_freshness_age():
    fresh = Freshness(0.5)
    assert fresh.age(1.0) == float('inf')
    fresh.received(2.0)
    assert fresh.age(2.25) == pytest.approx(0.25)


def test_registry_accepts_plain_text_and_dict():
    registry = FaultRegistry({'18001': 'plain text', 18002: {'name': 'X', 'text': 'dict text'}})
    assert len(registry) == 2
    assert 'plain text' in registry.text_for(18001)
    assert '(X)' in registry.text_for(18002)
