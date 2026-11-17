# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""How the drive comes back after a clock reset or stall in stm32:=renode_readonly (#141) under
the clock contract (#151, #158). Proposed reconciliation, ws1/reconcile-141.

The command-only stand-in (no SDO frame, SIL receive PDOs) and the drive with 2000h stored must
follow the same reset and stall gates as the SDO stand-in: the drive power cycles, the stand-in
drops the twist and sends the enable sequence again, the target is zero until a new command,
2000h is the stored value again, and the stand-in sends no SDO frame at any time.
Protocol evidence only. Not safety evidence.
"""
import can
import pytest

from conftest import wait_for
from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.sim_time import FedClock, ManualClock
from openamrobot_sil.stm32_host import DriveHost
from openamrobot_sil.zlac_gazebo import RPDO1_BASE, RPDO2_BASE, GazeboZlac8015d
from test_clock_gates import frames

LINK_LOSS_MS = 250


@pytest.fixture
def readonly_drive(make_bus):
    clock = ManualClock(10.0)
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock, rpdo=True,
                          stored_link_loss_ms=LINK_LOSS_MS).start()
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400, clock=clock,
                     mode='command_only')
    raw = make_bus()
    yield clock, emu, host, raw
    emu.stop()


def run_host(clock, host, n=10):
    for _ in range(n):
        clock.advance(0.02)
        host.follow_clock()
        host.step()


def wait_target(emu, rpm):
    return wait_for(lambda: emu.state()['target_rpm'] == rpm, 1.0)


@pytest.mark.parametrize('event', ['stall', 'reset'])
def test_command_only_drive_comes_back_at_zero_after_a_clock_event(readonly_drive, event):
    clock, emu, host, raw = readonly_drive
    host.set_twist(0.2, 0.0)
    run_host(clock, host)
    assert wait_for(lambda: emu.enabled, 1.0) and wait_target(emu, [19, 19])
    frames(raw, 0x701)
    if event == 'stall':
        clock.declare_stall()
    else:
        clock.reset(0.0)
    assert wait_for(lambda: getattr(emu, 'clock_%ss' % event) == 1, 1.0)
    assert not emu.enabled and emu.state()['target_rpm'] == [0, 0]   # power cycle
    assert emu.state()['link_loss_ms'] == LINK_LOSS_MS                # stored 2000h restored
    assert b'\x00' in frames(raw, 0x701)                              # boot-up of the drive
    if event == 'stall':
        clock.declare_resume()
    run_host(clock, host)                                             # stand-in brings it up
    assert host.clock_events == 1 and host.state()['configured']
    assert wait_for(lambda: emu.enabled, 1.0)
    assert wait_target(emu, [0, 0])                                   # not the old twist
    host.set_twist(0.2, 0.0)                                          # a new command moves it
    run_host(clock, host, 2)
    assert wait_target(emu, [19, 19])
    assert host.state()['sdo_frames_refused'] == 0                    # never tried an SDO
    assert frames(raw, canopen_common.SDO_RX_BASE + 1) == []          # and none on the bus


def test_command_only_receive_pdos_before_the_first_clock_do_nothing(make_bus):
    """#160 with #141: before the first /clock the drive is not powered, so a receive PDO
    (enable or target) is dropped; after the first clock the drive starts disabled with the
    stored 2000h."""
    clock = FedClock()
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock, rpdo=True,
                          stored_link_loss_ms=LINK_LOSS_MS).start()
    tx = make_bus()
    try:
        for word in (0x06, 0x07, 0x0F):
            tx.send(can.Message(arbitration_id=RPDO2_BASE + 1, data=word.to_bytes(2, 'little'),
                                is_extended_id=False))
        tx.send(can.Message(arbitration_id=RPDO1_BASE + 1,
                            data=(19).to_bytes(4, 'little', signed=True) * 2,
                            is_extended_id=False))
        assert wait_for(lambda: emu.pre_clock_drops >= 4, 1.0)
        assert emu.rpdo_frames == 0 and not emu.enabled
        clock.update(0.0)                                  # first /clock
        assert wait_for(lambda: emu.powered, 1.0)
        state = emu.state()
        assert not state['enabled'] and state['target_rpm'] == [0, 0]
        assert state['link_loss_ms'] == LINK_LOSS_MS
    finally:
        emu.stop()
