"""A candump trace replayed into vcan_drive in place of the drive emulator, read by the
firmware image in Renode.

Real: the openamr-platform-fw telemetry image at the pinned commit, preset RenodeTest.
Emulated: the MCU (Renode) and the bus (vcan). The drive is a recording: SIL_REPLAY_TRACE,
default traces/zlac_synthetic_v1.candump, which is synthetic. A bench capture of the drive bus
(candump -L) can be dropped in through that variable; the checks take their expected values
from the trace itself. No drive emulator, no host test client and no Gazebo run here.
Needs vcan, SIL_RENODE, SIL_FW_ELF and SIL_FW_DIR; a missing one FAILS with "BLOCKED:".
"""
import os
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'test_renode'))
from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs  # noqa: E402

from openamrobot_sil import renode_fw  # noqa: E402
from openamrobot_sil.candump_replay import Replay  # noqa: E402
from openamrobot_sil.canbus import open_bus  # noqa: E402

TRACE = os.environ.get('SIL_REPLAY_TRACE',
                       os.path.join(PACKAGE_DIR, 'traces', 'zlac_synthetic_v1.candump'))
WORK = '/tmp/openamrobot_sil_replay'
LOG = os.path.join(WORK, 'node.log')
IFACE = 'vcan_drive'
HELPERS = os.path.join(PACKAGE_DIR, 'renode', 'sil_renode_helpers.py')


def recorded(index, sub):
    """Values the trace holds for one object, in trace order (signed 32 bit)."""
    key = bytes([index & 0xFF, index >> 8, sub])
    return [int.from_bytes(f['data'][4:8], 'little', signed=True)
            for f in renode_fw.read_candump(TRACE)
            if f['id'] == 0x581 and f['data'][1:4] == key and f['data'][0] != 0x80]


def in_order(seen, source):
    """True when `seen` is a subsequence of `source`."""
    it = iter(source)
    return all(any(x == y for y in it) for x in seen)


class Listener:
    def __init__(self):
        self.bus, self.frames, self._run = open_bus(IFACE), [], True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            msg = self.bus.recv(timeout=0.05)
            if msg is not None:
                self.frames.append((msg.arbitration_id, bytes(msg.data)))

    def stop(self):
        self._run = False
        self._thread.join(timeout=1)
        self.bus.shutdown()


@pytest.fixture(scope='module')
def rig():
    renode, elf, fw_dir = renode_inputs()
    # The node bridges all three FDCANs; vcan_bms and vcan_aux stay empty in this test.
    r = subprocess.run(['sh', os.path.join(PACKAGE_DIR, 'scripts', 'sil_vcan.sh'), 'up', IFACE,
                        'vcan_bms', 'vcan_aux'], capture_output=True, text=True)
    if r.returncode != 0:
        pytest.fail('BLOCKED: no vcan interfaces %s: %s' % (IFACE, r.stderr.strip()),
                    pytrace=False)
    print('%s; trace %s (%d frames)' % (pinned_line(), os.path.basename(TRACE),
                                        len(renode_fw.read_candump(TRACE))))
    os.makedirs(WORK, exist_ok=True)
    rclpy.init()
    probe = FwProbe('sil_replay_probe')
    spinner = Spinner(probe)
    listener = Listener()
    replay_bus = open_bus(IFACE)
    replay = Replay.from_file(replay_bus, TRACE)
    with open(LOG, 'w', encoding='utf-8') as f:
        proc = subprocess.Popen(
            [sys.executable, '-m', 'openamrobot_sil.nodes.renode_fw_node', '--ros-args',
             '-p', 'renode:=%s' % renode, '-p', 'elf:=%s' % elf, '-p', 'fw_dir:=%s' % fw_dir,
             '-p', 'helpers:=%s' % HELPERS, '-p', 'work_dir:=%s' % WORK,
             '-p', 'config_file:=%s' % os.path.join(PACKAGE_DIR, 'config', 'sil.yaml'),
             '-p', 'can_link:=socketcan'],
            stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        # The image runs first, against a silent bus; the replay starts once it reports.
        if not probe.wait(lambda: len(probe.lines_after(0)) >= 3, 180.0, 0.2):
            with open(LOG, errors='replace') as f:
                tail = f.read()[-3000:]
            pytest.fail('FAILED: no status lines from the image after 180 s\n%s' % tail,
                        pytrace=False)
        t_start = time.monotonic()
        replay.start()
        assert probe.wait(lambda: replay.ended, replay.duration_s + 30.0)
        t_end = time.monotonic()
        # After the trace: the drive is silent. Wait for the image to say so.
        probe.wait(lambda: probe.last_status_line()['drive']['hb']['st'] == 'lost',
                   renode_fw.liveness_wait_s(renode_fw.FW_HB_TIMEOUT_US))
        probe.wait(lambda: len(probe.lines_after(time.monotonic() - 0.01)) >= 1, 5.0)
        t_lost = time.monotonic()
        probe.wait(lambda: len(probe.lines_after(t_lost)) >= 6,
                   renode_fw.liveness_wait_s(600000))
        time.sleep(0.5)
        yield {'probe': probe, 'replay': replay, 'listener': listener, 't_start': t_start,
               't_end': t_end, 't_lost': t_lost}
    finally:
        replay.stop()
        replay_bus.shutdown()
        listener.stop()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        spinner.stop()
        probe.destroy_node()
        rclpy.shutdown()


def test_image_reports_the_recorded_wheel_values_in_trace_order(rig):
    probe, replay = rig['probe'], rig['replay']
    during = [s for t, s in probe.lines_after(rig['t_start']) if t <= rig['t_end']]
    seen = {ch: [s['drive']['wheel'][ch]['raw'] for s in during
                 if s['drive']['wheel'][ch]['v'] == 'valid'] for ch in ('ch1', 'ch2')}
    state = replay.state()
    print('replay %s; the image printed %d lines during the trace; ch1 values %s' % (
        state, len(during), seen['ch1'][:12]))
    assert state['answered'] >= 10 and state['sent_unasked'] == state['unasked_total'] > 0
    for ch, sub in (('ch1', 1), ('ch2', 2)):
        source = recorded(0x606C, sub)
        assert len(seen[ch]) >= 3, seen
        assert in_order(seen[ch], source), (ch, seen[ch], source[:30])
        assert len(set(seen[ch])) >= min(3, len(set(source)))       # values, not one constant
        if 0 not in source:
            assert 0 not in seen[ch]
    assert any(s['drive']['hb']['st'] == 'alive' for s in during)
    # What the replay served is what the trace held, object by object, in order.
    for index, sub in replay.answers:
        served = [d for i, s, d in replay.served if (i, s) == (index, sub)]
        source = [f['data'] for f in renode_fw.read_candump(TRACE) if f['id'] == 0x581
                  and f['data'][1:4] == bytes([index & 0xFF, index >> 8, sub])]
        assert served == source[:len(served)]


def test_after_the_trace_the_image_reports_lost_and_unknown_never_zero(rig):
    probe = rig['probe']
    after = [s for _, s in probe.lines_after(rig['t_lost'])]
    assert len(after) >= 6
    last = after[-1]
    assert last['drive']['hb']['st'] == 'lost'
    for ch in ('ch1', 'ch2'):
        o = last['drive']['wheel'][ch]
        assert o['v'] in ('timeout', 'stale') and o['raw'] == 'unknown', o
    zero_recorded = 0 in recorded(0x606C, 1) + recorded(0x606C, 2)
    for _, s in probe.lines_after(rig['t_end']):
        for ch in ('ch1', 'ch2'):
            assert zero_recorded or s['drive']['wheel'][ch]['raw'] != 0, s['drive']['wheel']
    with probe.lock:
        late = [w for t, w in probe.wheels if t > rig['t_lost']]
    assert late and all(is_nan(x) for w in late[-3:] for x in w)


def test_replay_sent_no_master_frame_and_the_image_only_uploads(rig):
    """The trace holds requests, an NMT start and a receive PDO from its master side. None
    of them may reach the bus: the only SDO requests on it are the image's own uploads."""
    replay, listener, probe = rig['replay'], rig['listener'], rig['probe']
    trace = renode_fw.read_candump(TRACE)
    master_ids = {0x000, 0x201, 0x301, 0x401, 0x501}
    assert replay.state()['skipped_master_frames'] == sum(
        f['id'] in master_ids | {0x601} for f in trace)
    assert not [f for f in listener.frames if f[0] in master_ids]
    audit = renode_fw.FirmwareTxAudit().feed_file(os.path.join(WORK, 'fwtx-1.log')).summary()
    on_bus = sum(ident == 0x601 for ident, _ in listener.frames)
    print('firmware tap %d uploads, findings %s; 601h frames on %s: %d; replay saw %d '
          'requests' % (audit['uploads'], audit['findings'], IFACE, on_bus,
                        replay.state()['requests']))
    assert audit['violations'] == 0 and audit['uploads'] > 20, audit['examples']
    assert set(audit['by_label']) == {'fdcan1'}
    assert abs(audit['uploads'] - on_bus) <= 6        # every 601h frame is the image's
    assert all(data[:1] == b'\x40' for ident, data in listener.frames if ident == 0x601)
    assert probe.tx_audit['violations'] == 0
