import time

import can
import pytest

from canopen_common import SdoAbort, SdoClient
from conftest import collect, drain, wait_for
from zlac8015d_emulator import Zlac8015dEmulator


@pytest.fixture
def zlac(make_bus):
    node = Zlac8015dEmulator(make_bus(), node_id=1).start()
    yield node
    node.stop()


@pytest.fixture
def sdo(master, zlac):
    return SdoClient(master, 1)


def enable(sdo):
    sdo.write(0x6060, 0, 3, 1, signed=True)
    for cw in (0x06, 0x07, 0x0F):
        sdo.write(0x6040, 0, cw, 2)


def run_at(sdo, sub1, sub2):
    sdo.write(0x60FF, 1, sub1, 4, signed=True)
    sdo.write(0x60FF, 2, sub2, 4, signed=True)


def test_heartbeat_and_boot(master, make_bus):
    node = Zlac8015dEmulator(make_bus(), node_id=1)
    drain(master)
    node.start()
    try:
        frames = collect(master, 0.45)
    finally:
        node.stop()
    hb = [f for f in frames if f.arbitration_id == 0x701]
    assert hb[0].data[0] == 0x00  # boot-up message
    assert len(hb) >= 4 and all(f.data[0] == 0x05 for f in hb[1:])


def test_nmt_stop_and_start(master, zlac):
    master.send(can.Message(arbitration_id=0, data=[0x02, 1], is_extended_id=False))
    assert wait_for(lambda: zlac.nmt_state == 0x04)
    master.send(can.Message(arbitration_id=0, data=[0x01, 0], is_extended_id=False))
    assert wait_for(lambda: zlac.nmt_state == 0x05)


def test_sdo_read_write_and_aborts(sdo):
    assert sdo.read(0x2000, 0) == 0
    assert sdo.read(0x6083, 1) == 500
    sdo.write(0x6083, 1, 300, 4)
    assert sdo.read(0x6083, 1) == 300
    with pytest.raises(SdoAbort) as e:
        sdo.read(0x7777, 0)
    assert e.value.code == 0x06020000
    with pytest.raises(SdoAbort) as e:
        sdo.write(0x6041, 0, 1, 2)
    assert e.value.code == 0x06010002
    with pytest.raises(SdoAbort):
        sdo.write(0x60FF, 1, 5000, 4, signed=True)


def test_enable_and_speed_ramp(sdo):
    sdo.write(0x6083, 1, 50, 4)
    sdo.write(0x6083, 2, 50, 4)
    enable(sdo)
    assert sdo.read(0x6041, 0) & 0x0007 == 0x0007
    run_at(sdo, 100, -100)
    assert wait_for(lambda: sdo.read(0x606C, 1, signed=True) == 1000)
    assert sdo.read(0x606C, 2, signed=True) == -1000  # 0.1 r/min units


def test_no_motion_until_enabled(sdo):
    run_at(sdo, 100, 100)
    time.sleep(0.2)
    assert sdo.read(0x606C, 1, signed=True) == 0


def test_default_link_loss_keeps_last_speed(zlac, sdo):
    sdo.write(0x6083, 1, 20, 4)
    sdo.write(0x6083, 2, 20, 4)
    enable(sdo)
    run_at(sdo, 60, 60)
    time.sleep(0.2)
    time.sleep(1.0)  # no traffic at all for one second, 2000h = 0
    assert zlac.rpm[0] == pytest.approx(60, abs=0.5)
    assert zlac.link_loss_events == 0


def test_link_loss_stops_after_timeout(zlac, sdo):
    sdo.write(0x6083, 1, 20, 4)
    sdo.write(0x6083, 2, 20, 4)
    sdo.write(0x6084, 1, 20, 4)
    sdo.write(0x6084, 2, 20, 4)
    sdo.write(0x2000, 0, 250, 2)
    enable(sdo)
    run_at(sdo, 60, 60)
    assert wait_for(lambda: zlac.rpm[0] > 59)
    t0 = time.monotonic()
    assert wait_for(lambda: zlac.link_lost, timeout=1.0)
    elapsed = time.monotonic() - t0
    assert 0.15 < elapsed < 0.5
    assert wait_for(lambda: abs(zlac.rpm[0]) < 0.5, timeout=1.0)
    assert zlac.link_loss_events == 1


def test_link_loss_not_triggered_with_keepalive(zlac, sdo):
    sdo.write(0x2000, 0, 250, 2)
    enable(sdo)
    run_at(sdo, 60, 60)
    for _ in range(10):
        time.sleep(0.1)
        sdo.write(0x60FF, 1, 60, 4, signed=True)
    assert zlac.link_loss_events == 0


def test_save_2010_persists_across_power_cycle(zlac, sdo):
    sdo.write(0x2000, 0, 250, 2)
    zlac.power_cycle()
    assert sdo.read(0x2000, 0) == 0  # not saved yet
    sdo.write(0x2000, 0, 250, 2)
    sdo.write(0x2010, 0, 1, 2)
    sdo.write(0x2000, 0, 999, 2)
    zlac.power_cycle()
    assert sdo.read(0x2000, 0) == 250
    assert zlac.saves == 1


@pytest.mark.parametrize("inject,expected", [
    (lambda z: z.inject_over_current("high"), 0x00040000),
    (lambda z: z.inject_over_current("low"), 0x00000004),
    (lambda z: z.inject_under_voltage(), 0x0002),
    (lambda z: z.inject_encoder_error("low"), 0x0800),
    (lambda z: z.inject_encoder_error("high"), 0x08000000),
])
def test_faults_stop_wheels_and_report_603f(zlac, sdo, inject, expected):
    sdo.write(0x6083, 1, 20, 4)
    sdo.write(0x6083, 2, 20, 4)
    sdo.write(0x6084, 1, 20, 4)
    sdo.write(0x6084, 2, 20, 4)
    enable(sdo)
    run_at(sdo, 60, 60)
    assert wait_for(lambda: zlac.rpm[0] > 59)
    inject(zlac)
    assert sdo.read(0x603F, 0) == expected
    assert sdo.read(0x6041, 0) & 0x0008
    assert wait_for(lambda: abs(zlac.rpm[0]) < 0.5, timeout=1.0)
    # fault reset, then drive can be enabled again
    sdo.write(0x6040, 0, 0x80, 2)
    assert sdo.read(0x603F, 0) == 0
    enable(sdo)
    assert sdo.read(0x6041, 0) & 0x0004


def test_quick_stop_forces_zero_speed(zlac, sdo):
    sdo.write(0x6083, 1, 20, 4)
    sdo.write(0x6083, 2, 20, 4)
    enable(sdo)
    run_at(sdo, 80, 80)
    assert wait_for(lambda: zlac.rpm[0] > 79)
    sdo.write(0x6040, 0, 0x02, 2)
    assert wait_for(lambda: abs(zlac.rpm[0]) < 0.5, timeout=1.0)
    assert not sdo.read(0x6041, 0) & 0x0020


def test_bus_off_silences_node_and_recovers(master, zlac, sdo):
    drain(master)
    zlac.set_bus_off(True)
    time.sleep(0.05)
    drain(master)
    assert not [f for f in collect(master, 0.35) if f.arbitration_id == 0x701]
    with pytest.raises(TimeoutError):
        sdo.read(0x2000, 0)
    zlac.set_bus_off(False)
    frames = collect(master, 0.3)
    assert any(f.arbitration_id == 0x701 and f.data[0] == 0 for f in frames)  # boot-up again
    assert sdo.read(0x2000, 0) == 0


def test_bus_off_with_link_loss_protection_stops_wheels(zlac, sdo):
    sdo.write(0x2000, 0, 250, 2)
    sdo.write(0x6084, 1, 20, 4)
    sdo.write(0x6084, 2, 20, 4)
    sdo.write(0x6083, 1, 20, 4)
    sdo.write(0x6083, 2, 20, 4)
    enable(sdo)
    run_at(sdo, 60, 60)
    assert wait_for(lambda: zlac.rpm[0] > 59)
    zlac.set_bus_off(True)  # drive stays on, host frames no longer arrive
    assert wait_for(lambda: zlac.link_lost, timeout=1.0)
    assert wait_for(lambda: abs(zlac.rpm[0]) < 0.5, timeout=1.0)
