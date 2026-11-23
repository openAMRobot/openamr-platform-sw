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


# --- late SDO replies (Baseline 1 DDS benchmark run 44346730235, job 134951171122) -----------
#
# On a slow runner a reply came after the client's 0.1 s timeout. The client took the next frame
# from 581h as the answer to its next request: a write confirmation (command specifier 60h) as
# the answer of a 606Ch read, KeyError 96, and the drive thread ended for the rest of the run.

class _LateReplyDrive:
    """SDO server of node 1 on its own bus handle: answers in order, and holds back the reply to
    one chosen write of 60FFh:01 for `delay_s` (longer than the client timeout)."""

    def __init__(self, bus, late_write, delay_s, left_rpm_x10=100, right_rpm_x10=-200):
        import threading
        self.bus, self.late_write, self.delay_s = bus, late_write, delay_s
        self.values = {(0x606C, 1): left_rpm_x10, (0x606C, 2): right_rpm_x10}
        self.writes_60ff_1 = 0
        self.late_sent = threading.Event()
        self._held = []
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _reply(self, data):
        import can
        self.bus.send(can.Message(arbitration_id=0x581, data=data, is_extended_id=False))

    def _run(self):
        import struct
        while self._running:
            now = time.monotonic()
            for item in list(self._held):
                if now >= item[0]:
                    self._held.remove(item)
                    self._reply(item[1])
                    self.late_sent.set()
            msg = self.bus.recv(timeout=0.002)
            if msg is None or msg.arbitration_id != 0x601:
                continue
            cs = msg.data[0]
            index, sub = struct.unpack('<HB', bytes(msg.data[1:4]))
            if cs == 0x40:
                value = self.values.get((index, sub), 0)
                self._reply(bytes([0x43]) + struct.pack('<HBi', index, sub, value))
                continue
            reply = struct.pack('<BHB4x', 0x60, index, sub)
            if (index, sub) == (0x60FF, 1):
                self.writes_60ff_1 += 1
                if self.writes_60ff_1 == self.late_write:
                    self._held.append((time.monotonic() + self.delay_s, reply))
                    continue
            self._reply(reply)

    def stop(self):
        self._running = False
        self._thread.join(timeout=1)


def test_a_late_sdo_reply_does_not_end_the_drive_thread_or_swap_the_readings(make_bus):
    server = _LateReplyDrive(make_bus(), late_write=3, delay_s=0.15)
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.1).start()
    try:
        assert wait_for(server.late_sent.is_set, 3.0), 'the late reply was not sent'
        samples = host.state()['samples']
        assert wait_for(lambda: host.state()['samples'] >= samples + 20, 3.0), \
            'no odometry sample after the late reply'
        assert host._thread.is_alive(), 'the drive thread ended after a late SDO reply'
        state = host.state()
        assert state['sdo_errors'] >= 1                       # the timeout itself is counted
        assert state['sdo_stale_replies'] >= 1                # the late reply was dropped
        left, right = state['wheel_rad_s']                    # 10.0 and -20.0 rpm, never swapped
        assert left == pytest.approx(10.0 * 2 * 3.141592653589793 / 60, rel=1e-6)
        assert right == pytest.approx(-20.0 * 2 * 3.141592653589793 / 60, rel=1e-6)
    finally:
        host.stop()
        server.stop()


def test_without_a_new_drive_sample_no_odometry_is_offered_again(make_bus):
    """The odometry publisher of the host client offers only new 606Ch samples: when the drive
    stops answering, the last twist is not published again as if it were a new reading."""
    server = _LateReplyDrive(make_bus(), late_write=0, delay_s=0.0)
    host = DriveHost(make_bus(), 1, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.05).start()
    try:
        assert wait_for(lambda: host.new_sample(0) is not None, 3.0)
        server.stop()                                           # the drive stops answering
        time.sleep(0.3)
        last = host.state()['samples']
        assert host.state()['wheel_rad_s'][0] != 0.0            # the last twist is still held
        time.sleep(0.3)
        assert host.new_sample(last) is None
        assert host.state()['sdo_errors'] >= 1
    finally:
        host.stop()
        server.stop()
