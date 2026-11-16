"""CTL-017 emulator with the Gazebo lift hooks and the permit gate. Protocol evidence only.

The permit vectors P1 to P12 are those of the permit contract proposal in
openamr-upperbody-fw (docs/ctl017_permit_contract.md, section 6). P8 and P9 are open choices
there and are not modelled.
"""
import time

import can
import pytest

from conftest import FakeLift, wait_for
from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.ctl017_gazebo import GazeboCtl017, PermitConfig
from openamrobot_sil.stm32_host import LiftHost

PERMIT = PermitConfig(cob_id=0x410, flag_bits=8, boot_bits=16, seq_bits=16, reason_bits=8,
                      permit_value=1, timeout_ms=200, refused_result=0x80)


class Rig:
    def __init__(self, make_bus, permit, position_m=0.0):
        self.emu = GazeboCtl017(make_bus(), permit=permit)
        self.emu.attach_external_plant(position_m)
        self.lift = FakeLift(self.emu, position_m).start()
        self.emu.start()
        self.host = LiftHost(make_bus(), permit=permit).start()
        self.raw = make_bus()

    def ready(self):
        # IDLE and at standstill: the node answers BUSY until its quiet time has passed
        return wait_for(lambda: (self.host.status or {}).get('state') == c.IDLE
                        and not self.host.status['flags'] & c.F_MOVING)

    def result(self):
        return (self.host.status or {}).get('cmd_result')

    def moving(self):
        return self.emu.drive_on

    def stop(self):
        self.host.stop()
        self.emu.stop()
        self.lift.stop()


@pytest.fixture
def rig(make_bus):
    rigs = []

    def factory(permit=PERMIT, position_m=0.0):
        r = Rig(make_bus, permit, position_m)
        rigs.append(r)
        return r
    yield factory
    for r in rigs:
        r.stop()


def test_stop_sensors_and_hall_count_follow_the_gazebo_joint(make_bus):
    emu = GazeboCtl017(make_bus())
    emu.attach_external_plant(0.0)
    emu.start()
    try:
        assert wait_for(lambda: emu.state == c.IDLE)   # power-up on the L1000 sensor
        assert emu.sensor_bits() == c.IN_STOP0 and emu.hall_count == 0
        emu.set_joint_position(0.100)
        assert wait_for(lambda: emu.sensor_bits() == 0)
        assert emu.hall_count == pytest.approx(100.0 / c.MM_PER_HALL, abs=1)
        emu.set_joint_position(0.175)
        assert wait_for(lambda: emu.sensor_bits() == c.IN_STOP1)
        emu.set_joint_position(0.350)
        assert wait_for(lambda: emu.sensor_bits() == c.IN_STOP2)
        assert emu.hall_count == pytest.approx(350.0 / c.MM_PER_HALL, abs=1)
        assert emu.demand_velocity_m_s() == 0.0  # the joint moved, the motor was never on
    finally:
        emu.stop()


def test_without_gate_the_v1_lift_moves_to_the_commanded_stop(rig):
    r = rig(permit=None)
    assert r.ready()
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    assert wait_for(r.moving, 1.0)
    assert r.emu.demand_velocity_m_s() == pytest.approx(0.018)
    assert wait_for(lambda: r.host.status['active_stop'] == 1, timeout=14.0)
    assert r.lift.position_m == pytest.approx(0.175, abs=0.003)
    assert r.host.status['position_mm'] == pytest.approx(180.0, abs=2.0)
    assert r.host.status2['inputs'] & c.IN_STOP1
    assert r.emu.faults == 0 and r.result() == c.R_OK


def test_p1_p12_no_permit_frame_refuses_the_move(rig):
    r = rig()
    r.host.permit_silent = True   # heartbeats and commands only: a heartbeat is not a permit
    assert r.ready()
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    assert wait_for(lambda: r.result() == PERMIT.refused_result)
    time.sleep(0.5)
    assert not r.moving() and r.lift.position_m == 0.0
    assert r.emu.permit_refusals == 1 and r.emu.permit_rx_count == 0


def test_explicit_no_permit_frames_refuse_move_and_homing(rig):
    r = rig()
    assert r.ready()
    assert wait_for(lambda: r.emu.permit_rx_count > 3)  # frames with flag 0 arrive
    for op in (c.OP_MOVE_TO_STOP, c.OP_START_HOMING):
        r.host.command(op, 2)
        assert wait_for(lambda: r.host.status['last_seq'] == r.host.seq)
        assert r.result() == PERMIT.refused_result
    time.sleep(0.3)
    assert not r.moving() and r.lift.position_m == 0.0


def test_p2_fresh_permit_lets_the_lift_move(rig):
    r = rig()
    assert r.ready()
    r.host.set_permit(True)
    assert wait_for(lambda: r.emu.permit_fresh())
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    assert wait_for(r.moving, 1.0)
    assert wait_for(lambda: r.result() == c.R_OK)
    assert wait_for(lambda: r.lift.position_m > 0.010, 2.0)


def test_p3_stale_permit_refuses_the_move(rig):
    r = rig()
    assert r.ready()
    r.host.set_permit(True)
    assert wait_for(lambda: r.emu.permit_fresh())
    r.host.permit_silent = True
    time.sleep(PERMIT.timeout_ms / 1000.0 + 0.1)
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    assert wait_for(lambda: r.result() == PERMIT.refused_result)
    assert not r.moving()


def start_move(r, stop=2):
    assert r.ready()
    r.host.set_permit(True)
    assert wait_for(lambda: r.emu.permit_fresh())
    r.host.command(c.OP_MOVE_TO_STOP, stop)
    assert wait_for(lambda: r.lift.position_m > 0.010, 3.0)


def assert_halted_without_replay(r, restore):
    assert wait_for(lambda: not r.moving(), 0.5)
    assert r.emu.target_stop == 0xFF            # target dropped
    assert r.emu.permit_stops == 1
    assert wait_for(lambda: r.host.status['state'] == c.IDLE, 2.0)
    held = r.lift.position_m
    restore()
    assert wait_for(lambda: r.emu.permit_fresh())
    time.sleep(0.6)                             # the keep-alive of the old command still runs
    assert not r.moving() and r.lift.position_m == held and r.emu.faults == 0


def test_p4_p11_withdrawal_stops_the_lift_and_nothing_replays(rig):
    r = rig()
    start_move(r)
    t0 = time.monotonic()
    r.host.set_permit(False)
    assert wait_for(lambda: not r.moving(), 0.5, step=0.001)
    assert time.monotonic() - t0 < 0.15         # at once, not after the freshness time
    assert_halted_without_replay(r, lambda: r.host.set_permit(True))


def test_p5_expiry_stops_the_lift(rig):
    r = rig()
    start_move(r)
    t0 = time.monotonic()
    r.host.permit_silent = True
    assert wait_for(lambda: not r.moving(), 1.0, step=0.001)
    elapsed = time.monotonic() - t0
    assert PERMIT.timeout_ms / 1000.0 - 0.06 <= elapsed <= PERMIT.timeout_ms / 1000.0 + 0.15

    def restore():
        r.host.permit_silent = False
    assert_halted_without_replay(r, restore)


def test_p6_new_producer_boot_identity_stops_the_lift(rig):
    r = rig()
    start_move(r)
    r.host.boot_id = (r.host.boot_id + 1) & 0xFFFF or 1
    assert wait_for(lambda: not r.moving(), 0.5)
    assert r.emu.permit_stops == 1 and r.emu.target_stop == 0xFF


def test_p7_nmt_transition_drops_the_permit(rig):
    r = rig()
    assert r.ready()
    r.host.set_permit(True)
    assert wait_for(lambda: r.emu.permit_fresh())
    r.host.permit_silent = True
    r.raw.send(can.Message(arbitration_id=0, data=[0x80, c.NODE_ID], is_extended_id=False))
    r.raw.send(can.Message(arbitration_id=0, data=[0x01, c.NODE_ID], is_extended_id=False))
    assert wait_for(lambda: not r.emu.permit_valid, 0.15)


def test_p10_short_permit_frame_does_not_refresh(rig):
    r = rig()
    r.host.permit_silent = True
    assert r.ready()
    for _ in range(5):
        r.raw.send(can.Message(arbitration_id=PERMIT.cob_id, data=[1, 0, 0],
                               is_extended_id=False))
        time.sleep(0.02)
    assert wait_for(lambda: r.emu.rpdo_len_errors == 5)
    assert not r.emu.permit_fresh() and r.emu.permit_rx_count == 0


def test_sdo_cannot_write_a_permit(rig):
    r = rig()
    assert r.ready()
    assert not any(index >= 0x2000 and 'permit' in name.lower()
                   for index, _s, name, *_ in c.OD_V1)  # v1 has no permit object to write


def test_loss_of_lift_feedback_ends_in_the_stall_fault(rig):
    """A lift joint that does not move (Gazebo paused or blocked) gives no Hall pulses."""
    r = rig(permit=None)
    assert r.ready()
    r.lift.upper_m = 0.0   # the stand-in joint cannot leave 0
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    assert wait_for(lambda: r.emu.faults & (1 << (c.F03 - 1)), 2.0)
    assert not r.moving()
