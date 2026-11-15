"""The real image in Renode against the platform-fw drive stub: transmit audit of the three
FDCANs, negative tests, pty relay and time. No vcan and no Gazebo.

Renode-tested: Renode 1.16.1 runs the RenodeTest image of openamr-platform-fw at the pinned
commit. The drive is the platform-fw fixture tools/ci/renode_drive_stub.py of the same
commit, loaded into Renode on the CAN1 hub (can_link:=stub). Its values are made up
(606Ch = 1234 and -1234). FDCAN2 and FDCAN3, which this image starts and only receives on,
each sit on a hub of their own. Every frame an FDCAN transmits goes to the firmware tap;
every frame on a hub goes to the hub tap.

Order matters: the tests share one Renode run, and the last test audits all of it.
Needs SIL_RENODE, SIL_FW_ELF, SIL_FW_DIR. Missing inputs FAIL with "BLOCKED:".
Development evidence only: no board, no drive, no bus.
"""
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy

from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs, run_after_banner
from openamrobot_sil import renode_fw

HERE = os.path.dirname(os.path.abspath(__file__))
HELPERS = os.path.join(os.path.dirname(HERE), 'renode', 'sil_renode_helpers.py')
WORK = os.environ.get('SIL_RENODE_STUB_WORK', '/tmp/openamrobot_sil_renode_stub')
LOG = os.path.join(WORK, 'node.log')
LINK = os.path.join(WORK, 'hostlink')
STUB_WHEELS = (1234, -1234)          # 606Ch:01 and :02 of the platform-fw stub


def wheel(line, ch):
    return line['drive']['wheel'][ch]


class StubProbe(FwProbe):
    def mon(self, command):
        self.command('monitor ' + command)
        time.sleep(0.03)     # one command per poll of the node, in order

    def stub_set(self, key, value):
        self.mon('tlm_stub_set "%s" %d' % (key, value))

    def time(self):
        return (self.status or {}).get('time')

    def tx_audit_now(self):
        """The firmware transmit audit, read from the tap files of this run."""
        audit = renode_fw.FirmwareTxAudit()
        for start in range(1, self.status['renode_starts'] + 1):
            audit.feed_file(os.path.join(WORK, 'fwtx-%d.log' % start))
        return audit.summary()

    def hub_frames(self):
        frames = []
        for start in range(1, self.status['renode_starts'] + 1):
            frames += renode_fw.read_candump(os.path.join(WORK, 'hub-%d.log' % start))
        return frames

    def lines_since(self, t0):
        return [r for _, r in self.lines_after(t0)]

    def wait_lines(self, t0, n, virtual_us):
        ok = self.wait(lambda: len(self.lines_since(t0)) >= n,
                       renode_fw.liveness_wait_s(virtual_us))
        if not ok:
            pytest.fail('BLOCKED: fewer than %d status lines in %.0f s of wall time; time %s' % (
                n, renode_fw.liveness_wait_s(virtual_us), self.time()), pytrace=False)


@pytest.fixture(scope='module')
def fw():
    renode, elf, fw_dir = renode_inputs()
    stub = os.path.join(fw_dir, 'tools', 'ci', 'renode_drive_stub.py')
    if not os.path.exists(stub):
        pytest.fail('BLOCKED: %s does not exist' % stub, pytrace=False)
    print(pinned_line())
    os.makedirs(WORK, exist_ok=True)
    rclpy.init()
    probe = StubProbe()
    spinner = Spinner(probe)
    with open(LOG, 'w', encoding='utf-8') as f:
        proc = subprocess.Popen(
            [sys.executable, '-m', 'openamrobot_sil.nodes.renode_fw_node', '--ros-args',
             '-p', 'renode:=%s' % renode, '-p', 'elf:=%s' % elf, '-p', 'fw_dir:=%s' % fw_dir,
             '-p', 'helpers:=%s' % HELPERS, '-p', 'work_dir:=%s' % WORK,
             '-p', 'can_link:=stub', '-p', 'stub:=%s' % stub, '-p', 'test_commands:=true',
             '-p', 'hostlink_uart:=usart3', '-p', 'hostlink_pty:=%s' % LINK],
            stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        if not probe.wait(lambda: len(probe.lines_after(0)) >= 8 and probe.time(), 180.0, 0.2):
            with open(LOG, errors='replace') as f:
                tail = f.read()[-3000:]
            pytest.fail('FAILED: no status lines from the image after 180 s\n%s' % tail,
                        pytrace=False)
        yield probe
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        spinner.stop()
        probe.destroy_node()
        rclpy.shutdown()


def test_the_image_starts_all_three_fdcans(fw):
    """CCCR.INIT is 1 after reset and 0 once HAL_FDCAN_Start ran. Read while the image runs.
    The zero-transmit result below is about peripherals that are running."""
    fw.command('can_state')
    assert fw.wait(lambda: 'SIL_CAN' in fw.status['reports'], 30.0)
    state = fw.status['reports']['SIL_CAN']
    print('CCCR %s' % state)
    for fdcan in ('fdcan1', 'fdcan2', 'fdcan3'):
        assert state[fdcan]['cccr'] & 1 == 0, state


def test_drive_stub_values_reach_the_image(fw):
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['v'] == 'valid', 60.0)
    line = fw.last_status_line()
    assert (wheel(line, 'ch1')['raw'], wheel(line, 'ch2')['raw']) == STUB_WHEELS
    assert line['drive']['hb']['st'] == 'alive'
    with fw.lock:
        ch1, ch2 = fw.wheels[-1][1]
    assert abs(ch1 - 1234 * renode_fw.RPM_PER_UNIT * renode_fw.RAD_S_PER_RPM) < 1e-9
    assert ch2 == -ch1


def test_the_image_receives_on_can2_and_can3_and_transmits_nothing_there(fw):
    """FDCAN2 and FDCAN3 are active: frames put on their hubs raise the image's own receive
    counters. The firmware tap holds no frame from either peripheral."""
    cnt = fw.last_status_line()['cnt']
    c2, c3 = cnt.get('c2_rx', 0), cnt.get('c3_rx', 0)
    for _ in range(5):
        fw.mon('sil_hub_send "can2" 0x18904001 "01 00 01 00 75 30 03 20"')   # Daly 0x90 answer
        fw.mon('sil_hub_send "can3" 0x710 "05"')                             # CTL-017 heartbeat
        fw.mon('sil_hub_send "can3" 0x190 "03 03 E8 03 00 00 FF 01"')        # TPDO1 LiftStatus
    assert fw.wait(lambda: fw.last_status_line()['cnt'].get('c2_rx', 0) == c2 + 5
                   and fw.last_status_line()['cnt'].get('c3_rx', 0) == c3 + 10, 60.0), (
        fw.last_status_line()['cnt'])
    time.sleep(1.0)
    audit = fw.tx_audit_now()
    print('image counters c2_rx %d c3_rx %d; firmware tap by peripheral %s' % (
        c2 + 5, c3 + 10, audit['by_label']))
    assert audit['by_label'].get('fdcan2', 0) == 0 and audit['by_label'].get('fdcan3', 0) == 0
    assert set(audit['by_label']) == {'fdcan1'} and audit['violations'] == 0
    # Nothing but the injected frames was ever on those two hubs.
    assert not [f for f in fw.hub_frames() if f['interface'] in ('can2', 'can3')]
    cnt = fw.last_status_line()['cnt']
    assert cnt.get('c2_tx_ref', 0) == 0 and cnt.get('c3_tx_ref', 0) == 0


# ---- read-only proof --------------------------------------------------------------------
def test_firmware_tap_counts_what_the_image_says_it_sent(fw):
    """The tap is trusted because it agrees with the image's own "tx" counter."""
    line = fw.last_status_line()
    audit = fw.tx_audit_now()
    assert audit['uploads'] >= line['cnt']['tx'] > 20
    assert audit['uploads'] - line['cnt']['tx'] <= 12      # requests since that line
    assert sorted(audit['upload_objects']) == ['2026:05', '603F:00', '6041:00', '606C:01',
                                               '606C:02']


def test_a_foreign_write_on_the_bus_is_not_attributed_to_the_firmware(fw):
    """Control for the attribution: an SDO download, an NMT frame and a receive PDO sent by
    another bus member are on the CAN1 hub, and the firmware tap does not hold them."""
    before = fw.tx_audit_now()
    fw.mon('sil_hub_send "can1" 0x601 "2B 40 60 00 0F 00 00 00"')    # 6040h := 000Fh
    fw.mon('sil_hub_send "can1" 0x000 "01 01"')                      # NMT start node 1
    fw.mon('sil_hub_send "can1" 0x201 "00 00 00 00 00 00 00 00"')    # SIL RPDO1 (60FFh)
    assert fw.wait(lambda: sum(f['interface'] == 'can1.inject' and f['id'] in (0x601, 0, 0x201)
                               for f in fw.hub_frames()) >= 3, 30.0)
    foreign = renode_fw.FirmwareTxAudit()
    for f in fw.hub_frames():
        if f['interface'] == 'can1.inject' and f['id'] in (0x601, 0, 0x201):
            foreign.feed('can1.inject', f['id'], f['data'])
    assert foreign.findings['sdo_download'] == 1 and foreign.findings['nmt'] == 1
    assert foreign.findings['restricted_write'] == 2
    time.sleep(1.0)
    after = fw.tx_audit_now()
    assert after['violations'] == before['violations'] == 0, after['examples']
    assert after['uploads'] > before['uploads']


# ---- negative tests ---------------------------------------------------------------------
def test_duplicate_sdo_owner_is_detected_and_does_not_corrupt_the_values(fw):
    """A second client reads the drive while the image does. The bus then carries more
    upload requests than the image sent, the image counts answers it did not ask for, and no
    foreign answer becomes one of its values."""
    t0 = time.monotonic()
    first = fw.last_status_line()
    sent = 0
    for _ in range(25):
        fw.mon('sil_hub_send "can1" 0x601 "40 41 60 00 00 00 00 00"')    # foreign 6041h upload
        sent += 1
        time.sleep(0.1)
    fw.wait_lines(t0, 15, 1500000)
    lines = fw.lines_since(t0)
    last = lines[-1]
    odd = lambda s: s['cnt'].get('sdo_un', 0) + s['cnt'].get('sdo_wo', 0)   # noqa: E731
    print('foreign uploads %d; image counted %d unexpected answers; cnt %s' % (
        sent, odd(last) - odd(first), last['cnt']))
    assert odd(last) - odd(first) >= sent - 2
    for s in lines:
        assert (wheel(s, 'ch1')['raw'], wheel(s, 'ch2')['raw']) == STUB_WHEELS
        assert s['drive']['sw']['raw'] == '0x0237' and s['drive']['fault']['raw'] == '0x00000000'
        assert s['cnt']['sdo_ok'] <= s['cnt']['tx']
    # Detection on the bus: uploads on the hub against uploads the firmware transmitted.
    hub_uploads = sum(f['id'] == 0x601 and f['data'][:1] == b'\x40' for f in fw.hub_frames())
    assert hub_uploads - fw.tx_audit_now()['uploads'] in range(sent - 1, sent + 13)
    assert fw.tx_audit_now()['violations'] == 0


def test_bus_loss_gives_lost_heartbeat_and_unknown_wheels_then_recovers(fw):
    assert fw.wait(lambda: fw.last_status_line()['drive']['hb']['st'] == 'alive', 30.0)
    tx_before = fw.last_status_line()['cnt']['tx']
    t0 = time.monotonic()
    fw.command('can1_off')
    try:
        assert fw.wait(lambda: fw.last_status_line()['drive']['hb']['st'] == 'lost',
                       renode_fw.liveness_wait_s(renode_fw.FW_HB_TIMEOUT_US))
        t_lost = time.monotonic()
        # Allowed under the time rule: not earlier than the timeout in wall time. The last
        # heartbeat may be up to one heartbeat period (100 ms) older than the command.
        assert t_lost - t0 >= renode_fw.not_before_wall_s(renode_fw.FW_HB_TIMEOUT_US - 100000)
        fw.wait_lines(t_lost, 8, 800000)
        lines = fw.lines_since(t0)
    finally:
        fw.command('can1_on')
    lost = [s for s in lines if s['drive']['hb']['st'] == 'lost']
    # By the image's own clock: lost only after its 300 ms timeout.
    assert all(s['drive']['hb']['age_us'] > renode_fw.FW_HB_TIMEOUT_US for s in lost)
    for s in lines:
        for ch in ('ch1', 'ch2'):
            assert wheel(s, ch)['raw'] != 0, ('a wheel value of 0 was reported', s['seq'])
    for s in lost[-4:]:
        for ch in ('ch1', 'ch2'):
            assert wheel(s, ch)['v'] in ('timeout', 'stale') and wheel(s, ch)['raw'] == 'unknown'
    with fw.lock:
        late = [w for t, w in fw.wheels if t > t_lost + 0.5]
    assert late and all(is_nan(x) for w in late[-4:] for x in w)
    # The image keeps asking (Renode models no acknowledge), and still only uploads.
    assert lost[-1]['cnt']['tx'] > tx_before and fw.tx_audit_now()['violations'] == 0
    assert fw.wait(lambda: fw.last_status_line()['drive']['hb']['st'] == 'alive', 60.0)
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['raw'] == 1234, 60.0)


def test_stale_telemetry_drive_answers_stop_while_the_heartbeat_goes_on(fw):
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['v'] == 'valid', 30.0)
    t0 = time.monotonic()
    fw.stub_set('response_us', 30000000)      # every answer 30 s late: later than this test
    try:
        assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['raw'] == 'unknown',
                       renode_fw.liveness_wait_s(600000))
        t1 = time.monotonic()
        fw.wait_lines(t1, 6, 600000)
        lines = fw.lines_since(t1)
    finally:
        fw.stub_set('response_us', 500)
    for s in lines:
        assert s['drive']['hb']['st'] == 'alive', s['drive']['hb']
        for ch in ('ch1', 'ch2'):
            o = wheel(s, ch)
            assert o['v'] in ('timeout', 'stale') and o['raw'] == 'unknown', o
    for s in fw.lines_since(t0):
        for ch in ('ch1', 'ch2'):
            assert wheel(s, ch)['raw'] in (STUB_WHEELS[0], STUB_WHEELS[1], 'unknown')
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['v'] == 'valid', 60.0)


def test_stale_console_is_flagged_and_wheels_become_nan(fw):
    """The image stops printing (emulation paused): the node must not repeat old values."""
    assert fw.wait(lambda: fw.status['stale'] is False, 10.0)
    fw.command('pause')
    try:
        assert fw.wait(lambda: fw.status['stale'] is True, fw.status['stale_after_s'] + 10.0)
        t_stale = time.monotonic()
        time.sleep(1.0)
        with fw.lock:
            wheels = [w for t, w in fw.wheels if t > t_stale + 0.3]
        assert wheels and all(is_nan(x) for w in wheels for x in w)
    finally:
        fw.command('resume')
    assert fw.wait(lambda: fw.status['stale'] is False, 30.0)
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['raw'] == 1234, 30.0)


def tf_luna_frame(distance_cm, good=True):
    body = [0x59, 0x59, distance_cm & 0xFF, distance_cm >> 8, 0xF4, 0x01, 0x00, 0x09]
    return ' '.join('%02X' % b for b in body + [(sum(body) + (0 if good else 1)) & 0xFF])


def test_uart_corruption_is_counted_and_never_becomes_a_range(fw):
    """TF-Luna port USART2: good frames, then frames with a wrong checksum, cut frames and
    noise. No corrupted frame becomes a distance, the value goes stale, good frames recover."""
    tf = lambda s: s['sens']['usart2']   # noqa: E731
    fw.mon('tlm_stub_tfluna "sysbus.usart2" 10 150 500')
    assert fw.wait(lambda: tf(fw.last_status_line())['mm'] == 1500, 60.0)
    fw.mon('tlm_stub_tfluna "sysbus.usart2" 0 150 500')     # stop the good frames
    t_stop = time.monotonic() + 0.3
    fw.wait_lines(t_stop, 2, 200000)
    t0 = time.monotonic()
    before = fw.last_status_line()
    good_n = tf(before)['n']
    assert good_n > 0
    for _ in range(12):
        fw.mon('sil_uart_hex "sysbus.usart2" "%s"' % tf_luna_frame(777, good=False))
        fw.mon('sil_uart_hex "sysbus.usart2" "59 59 09 03 F4"')              # cut frame
        fw.mon('sil_uart_hex "sysbus.usart2" "FF 00 A5 59 13 59 59 59"')     # noise
        time.sleep(0.1)
    fw.wait_lines(t0, 8, 800000)
    lines = fw.lines_since(t0)
    last = lines[-1]
    print('TF-Luna after corruption: %s; counters %s' % (tf(last), last['cnt']))
    for s in lines:
        assert tf(s)['mm'] in (1500, 'unknown'), tf(s)       # never 7770, never 0
    assert tf(last)['n'] == good_n                            # no corrupted frame was accepted
    assert tf(last)['st'] != 'ok' and tf(last)['mm'] == 'unknown' and tf(last)['v'] != True  # noqa: E712,E501
    bad = lambda s: sum(v for k, v in s['cnt'].items() if k.startswith('tf_'))   # noqa: E731
    assert bad(last) - bad(before) >= 10, last['cnt']
    fw.mon('tlm_stub_tfluna "sysbus.usart2" 10 150 500')
    assert fw.wait(lambda: tf(fw.last_status_line())['st'] == 'ok'
                   and tf(fw.last_status_line())['mm'] == 1500, 60.0)
    fw.mon('tlm_stub_tfluna "sysbus.usart2" 0 150 500')


# ---- host link --------------------------------------------------------------------------
def read_link(seconds):
    """Read the host link pty for some wall time; return the bytes."""
    fd = os.open(LINK, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    data = b''
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                data += os.read(fd, 65536)
            except BlockingIOError:
                time.sleep(0.02)
    finally:
        os.close(fd)
    return data


def test_host_link_pty_reader_can_leave_and_return_without_stopping_the_image(fw):
    """The pty relay on the console UART of this image: a reader of the link goes away for
    longer than Renode's pty buffer lasts and comes back. Without the relay Renode blocks in
    the write and the emulation stops. The image keeps its sequence and reports no missed
    tick deadline. The micro-ROS image gets the same relay between its USART3 and the agent."""
    first = read_link(2.0)
    assert b'"t":"st"' in first, first[:200]
    t0 = time.monotonic()
    tick_miss = fw.status['tick_miss_total']
    seq0 = fw.last_status_line()['seq']
    # Nobody reads: 45 status lines are about 27 kB, more than the 20 kB that stalled Renode.
    fw.wait_lines(t0, 45, 4500000)
    link = fw.status['relay']
    assert link['opened'] and link['dropped_bytes'] > 0, link
    assert link['alive'] and link['errors'] == 0, link
    again = read_link(2.0)
    assert b'"t":"st"' in again
    lines = fw.lines_since(t0)
    seqs = [s['seq'] for s in lines]
    assert seqs == list(range(seqs[0], seqs[0] + len(seqs))) and seqs[0] <= seq0 + 2
    assert fw.status['tick_miss_total'] == tick_miss == 0
    assert not any('tick_miss' in s['cnt'] or 'dropped' in s['cnt'] for s in lines)


# ---- reset and time ---------------------------------------------------------------------
def test_transmit_audit_goes_on_over_a_machine_reset(fw):
    """A second boot in the same Renode: the tap keeps recording, the audit stays clean."""
    before = fw.tx_audit_now()['uploads']
    old, new, t_cmd = run_after_banner(fw, 'reset')
    assert new['boot'] == old['boot'] + 1
    fw.wait_lines(t_cmd, 12, 1200000)
    assert fw.wait(lambda: wheel(fw.last_status_line(), 'ch1')['raw'] == 1234, 60.0)
    audit = fw.tx_audit_now()
    assert audit['uploads'] > before + 20 and audit['violations'] == 0


def test_virtual_time_never_runs_ahead_of_wall_time(fw, record_property):
    t = fw.time()
    print('Renode virtual/wall ratio %.2f over %.1f s of wall time (recent %.2f); shortest '
          '%d us slice took %d us of wall time' % (
              t['ratio'], t['wall_s'], t['ratio_recent'], t['slice_us'],
              t['min_slice_wall_us']))
    record_property('renode_virtual_wall_ratio', '%.3f' % t['ratio'])
    assert t['virtual_s'] > 5.0 and t['throttle']
    assert t['ratio'] <= 1.0 and t['ratio_recent'] <= 1.0
    # The rule itself: no slice of virtual time took less wall time than its length.
    assert t['min_slice_wall_us'] >= t['slice_us']


def test_whole_run_the_firmware_transmitted_only_sdo_uploads_on_any_bus(fw):
    """Read-only proof over the whole run: every boot, the bus loss, the foreign frames, the
    reset. Any NMT, SDO download, receive PDO or other frame from FDCAN1, 2 or 3 fails, and
    FDCAN2 and FDCAN3, which are running, must not have transmitted at all."""
    time.sleep(1.0)
    tx_total = fw.status['fw_tx_total']      # the image's count first, then the tap: tap >= count
    audit = fw.tx_audit_now()
    live = None

    def published():
        nonlocal live
        live = fw.tx_audit
        return live and live['frames'] >= audit['frames']
    assert fw.wait(published, 10.0)
    boots = len(fw.boots())
    print('firmware tap: %d frames %s, uploads %d, findings %s; image tx %d over %d boots' % (
        audit['frames'], audit['by_label'], audit['uploads'], audit['findings'], tx_total,
        boots))
    assert boots >= 2 and audit['frames'] > 300
    assert audit['violations'] == 0 and live['violations'] == 0, audit['examples']
    assert all(count == 0 for count in audit['findings'].values()), audit['findings']
    assert set(audit['by_label']) == {'fdcan1'}       # zero frames from FDCAN2 and FDCAN3
    assert 0 <= audit['uploads'] - tx_total <= 12 * boots
