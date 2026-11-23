"""stm32:=renode_readonly: the host test client commands the drive without any SDO frame.

The drive emulator takes the SIL receive PDOs (zlac_gazebo RPDO1/RPDO2) and starts with 2000h
stored. A firmware-like SDO client reads 606Ch next to it, and the drive bus is audited.
Protocol evidence only, on vcan or python-can's virtual bus.
"""
import struct
import threading
import time

import can
import pytest

from conftest import FakeWheels, wait_for
from openamrobot_sil import renode_fw
from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.stm32_host import DriveHost, SdoForbidden, _CommandOnlyBus
from openamrobot_sil.zlac_gazebo import GazeboZlac8015d

LINK_LOSS_MS = 250


class Recorder:
    """Records every frame on the bus, as the drive bus audit does."""

    def __init__(self, bus):
        self.bus, self.frames, self._run = bus, [], True
        self.audit = renode_fw.SdoAudit(1)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            msg = self.bus.recv(timeout=0.05)
            if msg is not None:
                t = time.monotonic()
                self.frames.append((t, msg.arbitration_id, bytes(msg.data)))
                self.audit.feed(msg.arbitration_id, msg.data, t, msg.is_extended_id)

    def stop(self):
        self._run = False
        self._thread.join(timeout=1)


@pytest.fixture
def rig(make_bus):
    recorder = Recorder(make_bus())
    emu = GazeboZlac8015d(make_bus(), 1, rpdo=True, stored_link_loss_ms=LINK_LOSS_MS).start()
    wheels = FakeWheels(emu)
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400, mode='command_only').start()
    yield emu, host, recorder
    host.stop()
    wheels.stop()
    emu.stop()
    recorder.stop()


def test_command_only_client_enables_and_commands_the_drive_without_sdo(rig):
    emu, host, recorder = rig
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: emu.state()['target_rpm'] == [19, 19])
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01 and abs(emu.rpm[1] - 19) < 0.01)
    assert emu.enabled and emu.od.get(0x2000, 0) == LINK_LOSS_MS
    time.sleep(0.3)
    sdo = [f for f in recorder.frames if f[1] in (0x601, 0x581)]
    assert sdo == [], 'the command-only client sent SDO traffic: %r' % sdo[:3]
    assert any(f[1] == 0x201 for f in recorder.frames)
    assert host.state()['pdo_frames_sent'] > 10 and host.state()['sdo_frames_refused'] == 0
    assert emu.state()['rpdo_rejected'] == 0


def test_firmware_like_reader_and_command_only_client_share_the_bus(rig, make_bus):
    """One upload at a time, as the image does: the audit finds no overlap and no request
    other than uploads, and every request on the bus is one the reader sent."""
    emu, host, recorder = rig
    reader = canopen_common.SdoClient(make_bus(), 1, timeout=0.04)
    host.set_twist(0.2, 0.0)
    assert wait_for(lambda: abs(emu.rpm[0] - 19) < 0.01)
    sent, values = 0, []
    for _ in range(40):
        for index, sub in ((0x606C, 1), (0x606C, 2), (0x6041, 0), (0x603F, 0), (0x2026, 5)):
            sent += 1
            values.append(reader.read(index, sub, signed=index == 0x606C))
            time.sleep(0.02)
    time.sleep(0.1)
    a = recorder.audit.summary()
    assert a['non_upload'] == 0 and a['overlaps'] == 0, a
    assert a['uploads'] == a['requests'] == sent
    # 606Ch:01 after the first cycle (the fake wheels follow the ramp 10 ms late): 19 r/min
    assert all(v == pytest.approx(190, abs=2) for v in values[5::5]), values[::5]
    assert emu.state()['link_lost'] is False


def test_command_only_bus_refuses_sdo_frames(make_bus):
    guarded = _CommandOnlyBus(make_bus(), 1)
    with pytest.raises(SdoForbidden):
        guarded.send(can.Message(arbitration_id=0x601, data=bytes(8), is_extended_id=False))
    guarded.send(can.Message(arbitration_id=0x201, data=bytes(8), is_extended_id=False))
    assert guarded.refused == 1 and guarded.sent == 1


def test_sdo_mode_still_writes_and_reads_by_sdo(make_bus):
    """The default mode (stm32:=host) is unchanged: it does use SDO."""
    recorder = Recorder(make_bus())
    emu = GazeboZlac8015d(make_bus(), 1).start()
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400).start()
    try:
        assert wait_for(lambda: host.state()['configured'])
        time.sleep(0.1)
        assert any(f[1] == 0x601 for f in recorder.frames)
        assert host.state()['mode'] == 'sdo'
    finally:
        host.stop()
        emu.stop()
        recorder.stop()


def test_rpdo_is_ignored_unless_enabled(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1).start()
    tx = make_bus()
    try:
        for word in (0x06, 0x07, 0x0F):
            tx.send(can.Message(arbitration_id=0x301, data=struct.pack('<H', word),
                                is_extended_id=False))
        tx.send(can.Message(arbitration_id=0x201, data=struct.pack('<ii', 19, 19),
                            is_extended_id=False))
        time.sleep(0.2)
        assert not emu.enabled and emu.state()['target_rpm'] == [0, 0]
        assert emu.state()['rpdo_frames'] == 0
    finally:
        emu.stop()


def test_rpdo_out_of_range_target_is_rejected_and_not_applied(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1, rpdo=True).start()
    tx = make_bus()
    try:
        tx.send(can.Message(arbitration_id=0x201, data=struct.pack('<ii', 5000, 10),
                            is_extended_id=False))
        assert wait_for(lambda: emu.state()['rpdo_frames'] == 1)
        assert emu.state()['rpdo_rejected'] == 1
        assert emu.state()['target_rpm'] == [0, 10]
    finally:
        emu.stop()
