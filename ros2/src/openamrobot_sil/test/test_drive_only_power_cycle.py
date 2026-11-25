"""A power cycle of the drive that the host does not see (upperbody-sw #137, start-up standstill).

In a SIL launch the drive emulator and the host stand-in are two processes, each with its own
clock watch. A stall of /clock that only the drive process declares is, by the clock contract,
a power cycle of the drive: disabled, zero targets, boot-up frame. These tests ask whether the
host stand-in brings the drive up again when it did not see the stall itself.
Diagnostic tests on a scratch branch. They change no device logic.
"""
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


def test_control_the_drive_turns_before_the_power_cycle(rig):
    emu, host = rig
    assert wait_for(lambda: host.state()['configured'] and emu.enabled)
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)


def test_host_enables_the_drive_again_after_a_drive_only_power_cycle(rig):
    emu, host = rig
    assert wait_for(lambda: host.state()['configured'] and emu.enabled)
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    emu.power_cycle()  # what _follow_clock() does in the drive process at a stall or reset
    assert not emu.enabled
    recovered = wait_for(lambda: emu.enabled and abs(emu.rpm[0] - 19) < 0.01, 5.0)
    state = host.state()
    assert recovered, (
        'drive stays disabled while the host keeps commanding it: host configured=%s, '
        'sdo_errors=%d, statusword=0x%02X, target_rpm=%s; drive enabled=%s, rpm=%s, 2000h=%d'
        % (state['configured'], state['sdo_errors'], state['statusword'], state['target_rpm'],
           emu.enabled, [round(r, 2) for r in emu.rpm], emu.od.get(0x2000, 0)))
