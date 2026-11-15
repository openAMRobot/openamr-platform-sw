"""ZLAC8015D emulator with the Gazebo plant hooks. Protocol evidence only."""
import time

import pytest

from conftest import FakeWheels, collect, wait_for
from openamrobot_sil.can_sil import canopen_common

SdoAbort, SdoClient = canopen_common.SdoAbort, canopen_common.SdoClient
from openamrobot_sil.zlac_gazebo import (RPM_TO_RAD_S, GazeboZlac8015d, twist_to_wheel_speeds,
                                         wheel_speeds_to_twist)

LINK_LOSS_MS = 250


@pytest.fixture
def drive(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1).start()
    sdo = SdoClient(make_bus(), 1)
    yield emu, sdo
    emu.stop()


def enable(sdo, rpm=(19, 19), link_loss_ms=None):
    sdo.write(0x6060, 0, 3, 1, signed=True)
    if link_loss_ms is not None:
        sdo.write(0x2000, 0, link_loss_ms, 2)
    for word in (0x06, 0x07, 0x0F):
        sdo.write(0x6040, 0, word, 2)
    sdo.write(0x60FF, 1, rpm[0], 4, signed=True)
    sdo.write(0x60FF, 2, rpm[1], 4, signed=True)


def test_heartbeat_on_0x701(make_bus, drive):
    frames = collect(make_bus(), 0.55)
    beats = [f for f in frames if f.arbitration_id == 0x701]
    assert 4 <= len(beats) <= 7
    assert all(bytes(f.data) == b'\x05' for f in beats)


def test_no_demand_before_the_enable_sequence(drive):
    emu, sdo = drive
    sdo.write(0x60FF, 1, 19, 4, signed=True)
    sdo.write(0x60FF, 2, 19, 4, signed=True)
    time.sleep(0.2)
    assert emu.demand_rad_s() == (0.0, 0.0)


def test_demand_follows_the_velocity_target(drive):
    emu, sdo = drive
    enable(sdo, (19, -19))
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01 and abs(emu.rpm[1] + 19) < 0.01)
    left, right = emu.demand_rad_s()
    assert left == pytest.approx(19 * RPM_TO_RAD_S) and right == pytest.approx(-19 * RPM_TO_RAD_S)


def test_velocity_mode_only(drive):
    _emu, sdo = drive
    with pytest.raises(SdoAbort):
        sdo.write(0x6060, 0, 1, 1, signed=True)


def test_encoder_object_reports_the_gazebo_wheel_speed(drive):
    emu, sdo = drive
    enable(sdo)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    assert sdo.read(0x606C, 1, signed=True) == 190  # no Gazebo: vendored internal model
    emu.set_wheel_feedback(1.0, -0.5)               # Gazebo wheel speeds, rad/s
    time.sleep(0.05)
    assert sdo.read(0x606C, 1, signed=True) == round(1.0 / RPM_TO_RAD_S * 10)
    assert sdo.read(0x606C, 2, signed=True) == round(-0.5 / RPM_TO_RAD_S * 10)
    assert emu.state()['feedback'] == 'gazebo'
    assert emu.rpm[0] == pytest.approx(19, abs=0.01)  # the demand is not the feedback


def test_stale_gazebo_feedback_falls_back_to_the_internal_model(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1, feedback_timeout_s=0.1).start()
    try:
        emu.set_wheel_feedback(1.0, 1.0)
        assert emu.state()['feedback'] == 'gazebo'
        time.sleep(0.25)
        assert emu.state()['feedback'] == 'internal'
    finally:
        emu.stop()


def test_link_loss_zeroes_the_demand_within_the_configured_time(drive):
    emu, sdo = drive
    enable(sdo, link_loss_ms=LINK_LOSS_MS)
    assert sdo.read(0x2000, 0) == LINK_LOSS_MS
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    last_frame = time.monotonic()  # from here on the master is silent
    assert wait_for(lambda: emu.link_lost, timeout=1.0, step=0.001)
    detected = time.monotonic() - last_frame
    assert wait_for(lambda: emu.demand_rad_s() == (0.0, 0.0), timeout=1.0, step=0.001)
    stopped = time.monotonic() - last_frame
    # not before the configured time; after it within thread-timing margin plus the ramp
    assert LINK_LOSS_MS / 1000.0 - 0.02 <= detected <= LINK_LOSS_MS / 1000.0 + 0.15
    assert stopped <= LINK_LOSS_MS / 1000.0 + 0.25
    assert emu.link_loss_events == 1


def test_factory_link_loss_value_zero_keeps_the_last_speed(drive):
    emu, sdo = drive
    enable(sdo)
    assert sdo.read(0x2000, 0) == 0
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    time.sleep(0.6)
    assert emu.rpm[0] == pytest.approx(19, abs=0.01) and not emu.link_lost


def test_fault_stops_the_demand_until_fault_reset(drive):
    emu, sdo = drive
    enable(sdo)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    emu.inject_over_current('low')
    assert wait_for(lambda: emu.demand_rad_s() == (0.0, 0.0))
    assert sdo.read(0x6041, 0) & 0x0008
    assert sdo.read(0x603F, 0) == 0x0004
    sdo.write(0x6040, 0, 0x0F, 2)  # enabling does not clear a fault
    time.sleep(0.1)
    assert emu.demand_rad_s() == (0.0, 0.0)
    sdo.write(0x6040, 0, 0x80, 2)
    for word in (0x06, 0x07, 0x0F):
        sdo.write(0x6040, 0, word, 2)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)


def test_fake_wheels_close_the_loop(drive):
    emu, sdo = drive
    wheels = FakeWheels(emu, scale=0.5)  # wheels that turn at half the demand
    try:
        enable(sdo, (20, 20))
        assert wait_for(lambda: sdo.read(0x606C, 1, signed=True) == 100)
    finally:
        wheels.stop()


def test_twist_conversion_round_trip():
    left, right = twist_to_wheel_speeds(0.2, 0.3, 0.1, 0.4)
    assert left == pytest.approx(1.4) and right == pytest.approx(2.6)
    assert wheel_speeds_to_twist(left, right, 0.1, 0.4) == pytest.approx((0.2, 0.3))
