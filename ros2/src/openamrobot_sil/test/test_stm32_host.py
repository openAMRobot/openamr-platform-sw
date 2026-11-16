"""Host test client against the drive emulator. The client is a test stand-in on vcan."""
import time

import pytest

from conftest import FakeWheels, wait_for
from openamrobot_sil.stm32_host import DriveHost
from openamrobot_sil.zlac_gazebo import GazeboZlac8015d

LINK_LOSS_MS = 250


@pytest.fixture
def rig(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1).start()
    wheels = FakeWheels(emu)
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400).start()
    yield emu, host
    host.stop()
    wheels.stop()
    emu.stop()


def test_bring_up_writes_mode_link_loss_time_and_enable(rig):
    emu, host = rig
    assert wait_for(lambda: host.state()['configured'])
    assert emu.od.get(0x2000, 0) == LINK_LOSS_MS
    assert emu.od.get(0x6060, 0) == 3 and emu.enabled
    assert emu.demand_rad_s() == (0.0, 0.0)


def test_0_2_m_per_s_command_turns_the_wheels_and_odometry_returns(rig):
    emu, host = rig
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: emu.state()['target_rpm'] == [19, 19])  # 0.2 m/s on a 0.1 m wheel
    assert wait_for(lambda: host.state()['v'] == pytest.approx(0.199, abs=0.002))
    x0, t0 = host.state()['x'], time.monotonic()
    time.sleep(1.0)
    state = host.state()
    assert (state['x'] - x0) / (time.monotonic() - t0) == pytest.approx(0.199, abs=0.02)
    assert abs(state['y']) < 0.01 and state['w'] == pytest.approx(0.0, abs=0.02)
    assert state['sdo_errors'] == 0


def test_turn_command_gives_opposite_wheel_speeds(rig):
    emu, host = rig
    host.set_twist(0.0, 1.0)
    assert wait_for(lambda: emu.state()['target_rpm'] == [-19, 19])
    assert wait_for(lambda: host.state()['w'] == pytest.approx(0.995, abs=0.01))


def test_silent_master_trips_the_drive_link_loss_and_recovers(rig):
    emu, host = rig
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    t0 = time.monotonic()
    host.set_silent(True)
    assert wait_for(lambda: emu.link_lost, 1.0, step=0.001)
    assert wait_for(lambda: emu.demand_rad_s() == (0.0, 0.0), 1.0, step=0.001)
    assert time.monotonic() - t0 <= LINK_LOSS_MS / 1000.0 + 0.3
    time.sleep(0.3)
    assert emu.demand_rad_s() == (0.0, 0.0)
    host.set_silent(False)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01, 2.0)
    assert not emu.link_lost
