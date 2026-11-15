"""renode_fw: Renode script, banner check, console parser, wheel speeds and the SDO audit.

Pure logic: no Renode, no vcan, no ROS. The Renode runs are in test_renode_node (Renode-tested)
and test_integration/test_sil_renode.py (Gazebo-integrated).
"""
import math
import os

import pytest

from conftest import PACKAGE_DIR
from openamrobot_sil import renode_fw

BANNER = {'t': 'boot', 'project': 'openamr-platform-fw telemetry_image',
          'commit': renode_fw.FW_COMMIT[:12], 'build': 'Debug', 'params': 'test', 'boot': 1,
          'boot_record': 'cold'}
UROS_STATUS = {'boot': 1, 'seq': 3, 'rec': 'cold', 'up_us': 400000,
               'commit': renode_fw.FW_COMMIT[:12], 'params': 'test', 'uros': 'run',
               'lift': {'hb': 'alive', 'st': 'IDLE', 'pos': 50, 'age_us': 1000}, 'bms': 'locked'}


def status(ch1, ch2):
    return {'t': 'st', 'drive': {'wheel': {'ch1': ch1, 'ch2': ch2}}}


def test_pinned_values():
    assert renode_fw.FW_COMMIT == '763bdd3ffe4a5a582445f72d4845d8a0eedeba3c'
    assert renode_fw.FW_PRESET == 'RenodeTest'
    assert renode_fw.FW_PRESET_UROS == 'TelemetryUrosSerialSimBench'
    assert renode_fw.FW_ELF_UROS.endswith('/build/TelemetryUrosSerialSimBench/telemetry_image.elf')
    assert sorted(renode_fw.UROS_TOPICS) == ['/base_fw/drive_heartbeat', '/base_fw/status',
                                             '/base_fw/tof/front', '/base_fw/tof/rear',
                                             '/base_fw/wheel_speeds']
    assert renode_fw.RENODE_VERSION == '1.16.1'
    assert renode_fw.RENODE_URL.endswith('/v1.16.1/renode-1.16.1.linux-portable-dotnet.tar.gz')
    assert renode_fw.RENODE_SHA256 == (
        '00e113cdbd0f5354cf2f64bbe3f5a070d8958409542fca66e45ac97d982938c0')


def test_resc_is_the_platform_fw_machine_with_the_bridge_on_vcan_drive():
    text = renode_fw.resc('/fw/t.elf', '/fw/add.repl', '/w/con.log', '/s/h.py', 'vcan_drive')
    lines = text.splitlines()
    assert lines[:4] == ['mach create "openamr"',
                         'machine LoadPlatformDescription @platforms/cpus/stm32h743.repl',
                         'machine LoadPlatformDescription @/fw/add.repl',
                         'sysbus Tag <0x58024804, 0x58024807> "CSR1" 0x2000']
    assert '    sysbus LoadELF @/fw/t.elf' in lines
    assert 'sysbus.usart3 CreateFileBackend @/w/con.log true' in lines
    hub = lines.index('emulation CreateCANHub "can1"')
    assert lines[hub + 1:hub + 4] == ['connector Connect sysbus.fdcan1 can1',
                                      'machine CreateSocketCANBridge "can1_bridge" "vcan_drive"',
                                      'connector Connect can1_bridge can1']
    assert 'cpu PerformanceInMips 5' in lines
    assert lines[-5:] == ['include @/s/h.py', 'sil_follow_systick', 'sil_throttle_start 5',
                          'logLevel 3', 'start']


def test_resc_bridges_fdcan2_and_fdcan3_to_vcan_bms_and_vcan_aux():
    text = renode_fw.resc('/fw/t.elf', '/fw/add.repl', '/w/con.log', '/s/h.py', 'vcan_drive',
                          aux_interfaces=('vcan_bms', 'vcan_aux'))
    lines = text.splitlines()
    for n, iface in ((2, 'vcan_bms'), (3, 'vcan_aux')):
        hub = lines.index('emulation CreateCANHub "can%d"' % n)
        assert lines[hub + 1:hub + 4] == [
            'connector Connect sysbus.fdcan%d can%d' % (n, n),
            'machine CreateSocketCANBridge "can%d_bridge" "%s"' % (n, iface),
            'connector Connect can%d_bridge can%d' % (n, n)]


def test_resc_of_the_uros_image_puts_usart3_on_a_pty_without_throttle():
    text = renode_fw.resc('/fw/u.elf', '/fw/add.repl', '/w/con.log', '/s/h.py', 'vcan_drive',
                          aux_interfaces=('vcan_bms', 'vcan_aux'), uros_pty='/w/usart3-1.pty')
    lines = text.splitlines()
    assert 'emulation CreateUartPtyTerminal "uros_pty" "/w/usart3-1.pty" true' in lines
    assert 'connector Connect sysbus.usart3 uros_pty' in lines
    assert 'CreateFileBackend' not in text
    # WFI image: Renode idles at host speed; neither the MIPS cap nor the throttle is used.
    assert 'PerformanceInMips' not in text and 'sil_throttle_start' not in text
    assert lines[-4:] == ['include @/s/h.py', 'sil_follow_systick', 'logLevel 3', 'start']


def test_reset_of_the_uros_image_sets_the_clock_hook_without_throttle():
    assert renode_fw.reset_commands(throttle=False) == ['pause', 'machine Reset',
                                                        'sil_follow_systick', 'start']


def test_status_of_the_pinned_uros_build_passes():
    assert renode_fw.check_uros_status(UROS_STATUS)


@pytest.mark.parametrize('change, message', [
    ({'params': 'default'}, 'params'),
    ({'commit': renode_fw.FW_COMMIT[:12] + '-dirty'}, 'commit'),
    ({'commit': '28303cb01b82'}, 'commit'),
    ({'commit': None}, 'commit'),
])
def test_status_that_is_not_the_pinned_uros_build_fails(change, message):
    with pytest.raises(renode_fw.BannerError, match=message):
        renode_fw.check_uros_status(dict(UROS_STATUS, **change))


def test_uros_wheel_speeds_become_rad_s_and_nan_stays_nan():
    ch1, ch2 = renode_fw.uros_wheel_rad_s([191.0, float('nan')])
    assert ch1 == pytest.approx(19.1 * 2 * math.pi / 60)
    assert math.isnan(ch2)


def test_lift_position_is_mm_or_none_when_unknown():
    assert renode_fw.lift_mm(UROS_STATUS) == pytest.approx(5.0)
    assert renode_fw.lift_mm(dict(UROS_STATUS, lift={'pos': 'unknown'})) is None
    assert renode_fw.lift_mm({}) is None


def test_time_ratio_follows_the_uptime_and_restarts_with_a_boot():
    t = renode_fw.TimeRatio(window_s=10.0)
    assert t.ratio() is None
    for i in range(11):
        t.add(100.0 + i, i * 900000, 1)      # 0.9 s of virtual time per wall second
    assert t.ratio() == pytest.approx(0.9)
    t.add(112.0, 100000, 2)                  # new boot: the window starts again
    assert t.ratio() is None


def test_resc_without_interface_has_no_bridge():
    text = renode_fw.resc('/fw/t.elf', '/fw/add.repl', '/w/con.log', '/s/h.py', None)
    assert 'SocketCANBridge' not in text and 'connector Connect sysbus.fdcan1 can1' in text


def test_reset_pauses_and_sets_the_hooks_again_before_start():
    assert renode_fw.reset_commands() == ['pause', 'machine Reset', 'sil_follow_systick',
                                          'sil_throttle_start 5', 'start']


def test_helpers_file_defines_the_commands_the_script_uses():
    with open(os.path.join(PACKAGE_DIR, 'renode', 'sil_renode_helpers.py')) as f:
        text = f.read()
    for name in ('mc_sil_follow_systick', 'mc_sil_throttle_start', 'mc_sil_time_report'):
        assert 'def %s(' % name in text


def test_banner_of_the_pinned_test_build_passes():
    assert renode_fw.check_banner(BANNER)


@pytest.mark.parametrize('change, message', [
    ({'params': 'default'}, 'params'),
    ({'params': None}, 'params'),
    ({'commit': 'fcf81f8e2fe2-dirty'}, 'commit'),
    ({'commit': '28303cb01b82'}, 'commit'),
    ({'commit': 'unknown'}, 'commit'),
    ({'project': 'something else'}, 'project'),
])
def test_banner_that_is_not_the_pinned_test_build_fails(change, message):
    banner = dict(BANNER, **change)
    with pytest.raises(renode_fw.BannerError, match=message):
        renode_fw.check_banner(banner)


def test_console_parser_joins_partial_lines_and_skips_blank_ones():
    p = renode_fw.ConsoleParser()
    assert p.feed('\n{"t":"boot","boot":1') == []
    out = p.feed('}\n\n{"t":"st","seq":0}\nnot json\n{"t":"st"')
    assert [r['t'] if r else None for _, r in out] == ['boot', 'st', None]
    assert p.bad_lines == ['not json']
    assert p.feed(',"seq":1}\n')[0][1]['seq'] == 1


def test_valid_wheel_samples_become_rad_s():
    ch1, ch2 = renode_fw.wheel_rad_s(status({'v': 'valid', 'raw': 191}, {'v': 'valid', 'raw': -60}))
    assert ch1 == pytest.approx(19.1 * 2 * math.pi / 60)
    assert ch2 == pytest.approx(-6.0 * 2 * math.pi / 60)


@pytest.mark.parametrize('sample', [
    {'v': 'timeout', 'raw': 'unknown'}, {'v': 'stale', 'raw': 'unknown'},
    {'v': 'not_read', 'raw': 'unknown'}, {'v': 'unconfirmed', 'raw': 0},
    {'v': 'aborted', 'raw': 'unknown'}, {'v': 'valid', 'raw': 'unknown'}])
def test_a_wheel_value_that_is_not_valid_is_nan_never_zero(sample):
    ch1, ch2 = renode_fw.wheel_rad_s(status(sample, {'v': 'valid', 'raw': 0}))
    assert math.isnan(ch1)
    assert ch2 == 0.0   # a valid zero stays a zero


def upload(index, sub):
    return bytes([0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])


def answer(index, sub, value=0):
    return bytes([0x43, index & 0xFF, index >> 8, sub]) + value.to_bytes(4, 'little')


def firmware_like(audit, t0=0.0, cycles=10, answered=True):
    t = t0
    for _ in range(cycles):
        for index, sub in ((0x606C, 1), (0x606C, 2), (0x6041, 0), (0x603F, 0), (0x2026, 5)):
            audit.feed(0x601, upload(index, sub), t)
            if answered:
                audit.feed(0x581, answer(index, sub), t + 0.002)
                t += 0.020
            else:
                t += 0.060   # 40 ms timeout, next 20 ms tick
    return t


def test_audit_of_firmware_like_traffic_is_clean():
    a = renode_fw.SdoAudit(1)
    t = firmware_like(a)
    firmware_like(a, t, cycles=5, answered=False)
    s = a.summary()
    assert s['requests'] == s['uploads'] == 75
    assert s['non_upload'] == 0 and s['overlaps'] == 0 and s['unmatched_responses'] == 0
    assert s['timeouts'] == 24   # the last unanswered one is still open


def test_audit_finds_an_sdo_download_from_another_client():
    a = renode_fw.SdoAudit(1)
    firmware_like(a, cycles=2)
    a.feed(0x601, bytes([0x23, 0xFF, 0x60, 0x01, 19, 0, 0, 0]), 1.0)   # 60FFh:01 write
    assert a.summary()['non_upload'] == 1


def test_audit_finds_two_outstanding_requests():
    a = renode_fw.SdoAudit(1)
    a.feed(0x601, upload(0x606C, 1), 0.000)
    a.feed(0x601, upload(0x606C, 1), 0.005)   # a second client's upload, first still open
    s = a.summary()
    assert s['overlaps'] == 1 and s['non_upload'] == 0
    assert '606C:01' in s['examples'][0]


def test_audit_answer_for_another_object_does_not_close_the_request():
    a = renode_fw.SdoAudit(1)
    a.feed(0x601, upload(0x606C, 1), 0.000)
    a.feed(0x581, answer(0x6041, 0), 0.002)
    a.feed(0x601, upload(0x606C, 2), 0.010)
    s = a.summary()
    assert s['unmatched_responses'] == 1 and s['overlaps'] == 1


def test_audit_ignores_other_nodes_and_extended_frames():
    a = renode_fw.SdoAudit(1)
    a.feed(0x602, bytes([0x23] + [0] * 7), 0.0)
    a.feed(0x601, bytes(8), 0.0, extended=True)
    a.feed(0x201, bytes(8), 0.0)
    assert a.summary()['requests'] == 0


def test_candump_line():
    assert renode_fw.candump_line(1.5, 'vcan_drive', 0x601, upload(0x606C, 1)) == \
        '(1.500000) vcan_drive 601#406C600100000000'
