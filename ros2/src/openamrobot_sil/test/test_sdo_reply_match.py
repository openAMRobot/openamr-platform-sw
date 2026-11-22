# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""MatchedSdoClient and the drive host against SDO servers that answer late, twice, out of order
or with frames that are not the answer (review of #169).

Each test marked "revert:" fails when DriveHost goes back to the vendored
canopen_common.SdoClient, which takes the first frame from 580h + node as the answer. The two
RTF 0.1 tests at the end describe the current clock domains (SDO deadline on steady time, loop
pacing and odometry integration on the worker clock); they are characterisation, not a revert
check, and change when openamr-platform-fw#57 moves the SDO timeout to the plant clock."""
import math
import struct
import threading
import time

import can
import pytest

from conftest import wait_for
from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.sim_time import ManualClock
from openamrobot_sil.stm32_host import DriveHost, MatchedSdoClient

NODE = 1
RX, TX = 0x600 + NODE, 0x580 + NODE
LINK_LOSS_MS = 250


def upload(index, sub, value):
    return bytes([0x43]) + struct.pack('<HBi', index, sub, value)


def confirm(index, sub):
    return struct.pack('<BHB4x', 0x60, index, sub)


def abort(index, sub, code=0x06020000):
    return struct.pack('<BHBI', 0x80, index, sub, code)


class ScriptedServer:
    """SDO server of node 1 on its own bus handle. For every request, `script(n, cs, index, sub)`
    returns the frames to send, as (delay_s, data) or (delay_s, data, arbitration_id, extended).
    `n` counts requests from 0."""

    def __init__(self, bus, script):
        self.bus, self.script = bus, script
        self.requests = []
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            msg = self.bus.recv(timeout=0.002)
            if msg is None or msg.arbitration_id != RX:
                continue
            cs = msg.data[0]
            index, sub = struct.unpack('<HB', bytes(msg.data[1:4]))
            n = len(self.requests)
            self.requests.append((cs, index, sub))
            for frame in self.script(n, cs, index, sub):
                delay, data = frame[0], frame[1]
                arb = frame[2] if len(frame) > 2 else TX
                ext = frame[3] if len(frame) > 3 else False
                if delay:
                    time.sleep(delay)
                self.bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=ext))

    def stop(self):
        self._running = False
        self._thread.join(timeout=1)


@pytest.fixture
def serve(make_bus):
    servers = []

    def factory(script):
        server = ScriptedServer(make_bus(), script)
        servers.append(server)
        return server
    yield factory
    for server in servers:
        server.stop()


def client(make_bus, timeout=0.1):
    return MatchedSdoClient(make_bus(), NODE, timeout=timeout)


# --- matching by index, subindex and command specifier ---------------------------------------

def test_read_takes_only_an_upload_answer_of_its_own_object(make_bus, serve):
    """revert: the vendored client takes the 60h confirmation and raises KeyError 96."""
    def script(n, cs, index, sub):
        return [(0, confirm(index, sub)),                   # write confirmation, not an upload
                (0, upload(index, sub + 1, 7)),             # upload of another subindex
                (0, upload(index, sub, 99)[:7]),            # short DLC
                (0, upload(index, sub, 98), TX, True),      # extended id 581h: not the server
                (0, upload(index, sub, 97), TX + 1),        # node 2
                (0, upload(index, sub, 42))]                # the answer
    serve(script)
    sdo = client(make_bus)
    assert sdo.read(0x606C, 1, signed=True) == 42
    assert sdo.stale_replies == 3  # 60h, other subindex, short DLC; other ids are not counted


def test_write_takes_only_a_confirmation_of_its_own_object(make_bus, serve):
    """revert: the vendored write returns on the upload frame; the leftover frames are then
    read as the answer of the next read (KeyError 96)."""
    def script(n, cs, index, sub):
        if cs == 0x40:
            return [(0, upload(index, sub, 5))]
        return [(0, upload(index, sub, 1)), (0, confirm(index + 1, sub)), (0, confirm(index, sub))]
    serve(script)
    sdo = client(make_bus)
    sdo.write(0x60FF, 1, 19, 4, signed=True)
    assert sdo.read(0x606C, 1, signed=True) == 5
    assert sdo.stale_replies == 2


def test_an_abort_of_the_request_raises_and_an_abort_of_another_object_is_dropped(
        make_bus, serve):
    """revert: the vendored client raises SdoAbort on the abort of 6041h while 606Ch is read."""
    def script(n, cs, index, sub):
        if index == 0x603F:
            return [(0, abort(index, sub, 0x06090011))]
        return [(0, abort(0x6041, 0)), (0, upload(index, sub, 11))]
    serve(script)
    sdo = client(make_bus)
    assert sdo.read(0x606C, 2, signed=True) == 11
    with pytest.raises(canopen_common.SdoAbort) as err:
        sdo.read(0x603F, 0)
    assert err.value.code == 0x06090011 and '603Fh:00' in str(err.value)


def test_a_short_abort_frame_is_dropped_and_the_request_times_out(make_bus, serve):
    """revert: the vendored client unpacks the abort code from a 4-byte frame (struct.error)."""
    serve(lambda n, cs, index, sub: [(0, abort(index, sub)[:4])])
    sdo = client(make_bus, timeout=0.05)
    with pytest.raises(TimeoutError):
        sdo.read(0x606C, 1)
    assert sdo.stale_replies == 1


def test_duplicated_answers_are_not_taken_by_the_next_request(make_bus, serve):
    """Every answer sent twice. revert: the second read returns the duplicate of the first."""
    values = {(0x606C, 1): 100, (0x606C, 2): -200}

    def script(n, cs, index, sub):
        frame = upload(index, sub, values[(index, sub)]) if cs == 0x40 else confirm(index, sub)
        return [(0, frame), (0, frame)]
    serve(script)
    sdo = client(make_bus)
    for _ in range(3):
        assert sdo.read(0x606C, 1, signed=True) == 100
        assert sdo.read(0x606C, 2, signed=True) == -200
        sdo.write(0x60FF, 1, 19, 4, signed=True)
    assert sdo.stale_replies >= 8


def test_a_late_answer_that_comes_after_the_next_request_is_not_its_answer(make_bus, serve):
    """Out of order: the answer of request 0 comes after the client's timeout, just before the
    answer of request 1. revert: the second read returns request 0's value."""
    def script(n, cs, index, sub):
        if n == 0:
            return []                                        # held back, sent with request 1
        if n == 1:
            return [(0, upload(0x606C, 1, 100)), (0.005, upload(index, sub, -200))]
        return [(0, upload(index, sub, 0))]
    serve(script)
    sdo = client(make_bus, timeout=0.05)
    with pytest.raises(TimeoutError):
        sdo.read(0x606C, 1, signed=True)
    assert sdo.read(0x606C, 2, signed=True) == -200
    assert sdo.stale_replies == 1


# --- the drive thread after rejected replies; odometry only from valid samples ----------------

def _drive_script(values, junk):
    def script(n, cs, index, sub):
        if cs == 0x40:
            frame = upload(index, sub, values.get((index, sub), 0))
        else:
            frame = confirm(index, sub)
        return junk(n, cs, index, sub) + [(0, frame)]
    return script


def test_the_drive_thread_survives_a_stream_of_rejected_frames(make_bus, serve):
    """Before every answer the server sends a frame the client must reject (60h to a read, an
    upload to a write, the abort of another object, a short frame). revert: KeyError 96 ends the
    drive thread and samples stop."""
    junk_frames = [lambda i, s: confirm(i, s), lambda i, s: upload(i, s, 1),
                   lambda i, s: abort(0x2000, 0), lambda i, s: upload(i, s, 1)[:5]]

    def junk(n, cs, index, sub):
        return [(0, junk_frames[n % len(junk_frames)](index, sub))]
    serve(_drive_script({(0x606C, 1): 100, (0x606C, 2): -200}, junk))
    host = DriveHost(make_bus(), NODE, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.1).start()
    try:
        assert wait_for(lambda: host.state()['samples'] >= 30, 5.0), host.state()
        assert host._thread.is_alive()
        state = host.state()
        assert state['sdo_errors'] == 0
        assert state['sdo_stale_replies'] >= 30
        assert state['wheel_rad_s'][0] == pytest.approx(10.0 * 2 * math.pi / 60, rel=1e-6)
        assert state['wheel_rad_s'][1] == pytest.approx(-20.0 * 2 * math.pi / 60, rel=1e-6)
    finally:
        host.stop()


def test_an_aborted_speed_read_gives_no_sample_and_no_integration_across_the_gap(
        make_bus, serve):
    """While the drive aborts the 606Ch:02 read, no new sample is offered and the pose does not
    move; afterwards integration starts again from the next sample, not across the gap."""
    aborting = threading.Event()
    values = {(0x606C, 1): 100, (0x606C, 2): 100}          # 10 rpm both wheels: straight ahead

    def script(n, cs, index, sub):
        if aborting.is_set() and (index, sub) == (0x606C, 2):
            return [(0, abort(index, sub))]
        value = values.get((index, sub), 0)
        frame = upload(index, sub, value) if cs == 0x40 else confirm(index, sub)
        return [(0, frame)]
    serve(script)
    host = DriveHost(make_bus(), NODE, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.1).start()
    try:
        assert wait_for(lambda: host.state()['samples'] >= 5, 3.0)
        aborting.set()
        time.sleep(0.1)
        frozen = host.state()
        time.sleep(0.5)
        during = host.state()
        assert host.new_sample(frozen['samples']) is None
        assert during['x'] == frozen['x'] and during['sdo_errors'] > frozen['sdo_errors']
        aborting.clear()
        assert wait_for(lambda: host.new_sample(during['samples']) is not None, 3.0)
        after = host.new_sample(during['samples'])
        # One sample after the gap restarts integration: at most one 20 ms cycle of travel
        # (v = 10 rpm * 2 pi / 60 * 0.1 m = 0.105 m/s), never the 0.5 s gap.
        assert after['x'] - during['x'] < 0.105 * 0.1
    finally:
        host.stop()


# --- clock domains at RTF 0.1 (characterisation; openamr-platform-fw#57) ----------------------

class _RtfFeeder:
    """Advances a ManualClock at `rtf` times wall time, like /clock from a slow Gazebo."""

    def __init__(self, clock, rtf, step_s=0.002):
        self.clock, self.rtf, self.step_s = clock, rtf, step_s
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            time.sleep(self.step_s)
            self.clock.advance(self.step_s * self.rtf)

    def stop(self):
        self._running = False
        self._thread.join(timeout=1)


def test_at_rtf_0_1_odometry_integrates_on_the_worker_clock(make_bus, serve):
    """Pacing and integration follow the fed clock: at RTF 0.1 the distance per simulated second
    is the wheel speed, and there are about 50 cycles per simulated second, not per wall second."""
    values = {(0x606C, 1): 100, (0x606C, 2): 100}
    serve(_drive_script(values, lambda *a: []))
    clock = ManualClock(0.0)
    feeder = _RtfFeeder(clock, 0.1)
    host = DriveHost(make_bus(), NODE, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.1,
                     clock=clock).start()
    try:
        assert wait_for(lambda: host.state()['samples'] >= 3, 5.0)
        s0, t0 = host.state(), clock.now()
        time.sleep(2.0)                                       # about 0.2 s simulated
        s1, t1 = host.state(), clock.now()
        sim_dt = t1 - t0
        v = 10.0 * 2 * math.pi / 60 * 0.100
        assert (s1['x'] - s0['x']) / sim_dt == pytest.approx(v, rel=0.15)
        assert (s1['samples'] - s0['samples']) / sim_dt == pytest.approx(50, rel=0.3)
        assert s1['sdo_errors'] == 0
    finally:
        host.stop()
        feeder.stop()


def test_at_rtf_0_1_the_sdo_deadline_is_steady_time_not_simulated_time(make_bus, serve):
    """The SDO timeout is 0.1 s of steady time (clock_inventory.yaml: current steady, target
    gazebo, to_change, openamr-platform-fw#57). A drive that answers 150 ms of wall time late,
    which is 15 ms of simulated time at RTF 0.1, is counted as a timeout: under low RTF the
    deadline does not stretch with the plant."""
    def script(n, cs, index, sub):
        if (index, sub) == (0x606C, 2):
            return [(0.15, upload(index, sub, 100))]
        frame = upload(index, sub, 100) if cs == 0x40 else confirm(index, sub)
        return [(0, frame)]
    serve(script)
    clock = ManualClock(0.0)
    feeder = _RtfFeeder(clock, 0.1)
    host = DriveHost(make_bus(), NODE, LINK_LOSS_MS, 0.100, 0.400, sdo_timeout_s=0.1,
                     clock=clock).start()
    try:
        assert wait_for(lambda: host.state()['sdo_errors'] >= 2, 5.0)
        assert host.state()['samples'] == 0
        assert host._thread.is_alive()
        assert wait_for(lambda: host.state()['sdo_stale_replies'] >= 1, 3.0)
    finally:
        host.stop()
        feeder.stop()
