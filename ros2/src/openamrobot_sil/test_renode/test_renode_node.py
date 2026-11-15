"""The renode_firmware node with the real image in Renode, without vcan and Gazebo.

Renode-tested: Renode 1.16.1 runs the RenodeTest image of openamr-platform-fw at the pinned
commit; FDCAN1 is on a Renode CAN hub with nothing else on it (can_link:=none), so the image
sees a silent drive. Needs SIL_RENODE, SIL_FW_ELF, SIL_FW_DIR and, for the negative banner
test, SIL_FW_ELF_DEFAULT (scripts/sil_renode_firmware.sh --default-too). Missing inputs FAIL
with "BLOCKED:".
"""
import os
import signal
import subprocess
import sys

import pytest
import rclpy

from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs, run_after_banner
from openamrobot_sil import renode_fw

HERE = os.path.dirname(os.path.abspath(__file__))
HELPERS = os.path.join(os.path.dirname(HERE), 'renode', 'sil_renode_helpers.py')
WORK = os.environ.get('SIL_RENODE_WORK', '/tmp/openamrobot_sil_renode_node')
LOG = os.path.join(WORK, 'node.log')


def start_node(elf, renode, fw_dir, work, log, banner_timeout=120.0):
    os.makedirs(work, exist_ok=True)
    with open(log, 'w', encoding='utf-8') as f:
        return subprocess.Popen(
            [sys.executable, '-m', 'openamrobot_sil.nodes.renode_fw_node', '--ros-args',
             '-p', 'renode:=%s' % renode, '-p', 'elf:=%s' % elf, '-p', 'fw_dir:=%s' % fw_dir,
             '-p', 'helpers:=%s' % HELPERS, '-p', 'work_dir:=%s' % work,
             '-p', 'can_link:=none', '-p', 'banner_timeout_s:=%f' % banner_timeout],
            stdout=f, stderr=subprocess.STDOUT, start_new_session=True)


def stop_node(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


@pytest.fixture(scope='module')
def fw():
    renode, elf, fw_dir = renode_inputs()
    print(pinned_line())
    rclpy.init()
    probe = FwProbe()
    spinner = Spinner(probe)
    proc = start_node(elf, renode, fw_dir, WORK, LOG)
    try:
        if not probe.wait(lambda: len(probe.lines_after(0)) >= 5, 180.0, 0.2):
            with open(LOG, errors='replace') as f:
                tail = f.read()[-3000:]
            pytest.fail('FAILED: no status lines from the image after 180 s\n%s' % tail,
                        pytrace=False)
        yield probe
    finally:
        stop_node(proc)
        spinner.stop()
        probe.destroy_node()
        rclpy.shutdown()


def test_banner_is_the_pinned_renode_test_build(fw, record_property):
    assert fw.wait(lambda: fw.banner() is not None, 30.0)
    banner = fw.banner()
    record_property('fw_commit', renode_fw.FW_COMMIT)
    record_property('fw_preset', renode_fw.FW_PRESET)
    record_property('banner_commit', banner['commit'])
    print('%s; banner commit %s params %s' % (pinned_line(), banner['commit'], banner['params']))
    assert banner['params'] == 'test'
    assert renode_fw.FW_COMMIT.startswith(banner['commit'])
    assert banner['boot'] == 1 and banner['boot_record'] == 'cold'
    s = fw.status
    assert s['banner']['commit'] == banner['commit'] and s['fatal'] is None
    assert s['fw_commit'] == renode_fw.FW_COMMIT and s['fw_preset'] == 'RenodeTest'
    assert s['bad_lines'] == 0


def test_virtual_time_does_not_run_ahead_of_wall_time(fw):
    t0, a = fw.lines_after(0)[1]
    assert fw.wait(lambda: fw.lines_after(0)[-1][0] - t0 >= 3.0, 30.0)
    t1, b = fw.lines_after(0)[-1]
    virtual, wall = (b['up_us'] - a['up_us']) / 1e6, t1 - t0
    print('virtual %.2f s in wall %.2f s (ratio %.2f)' % (virtual, wall, virtual / wall))
    assert virtual > 0.2   # the image runs
    # throttle: at most one 5 ms slice ahead, plus topic and file polling jitter
    assert virtual <= wall + 0.15, (virtual, wall)


def test_silent_drive_gives_unknown_wheels_never_zero(fw):
    assert fw.wait(lambda: fw.last_status_line()['drive']['hb']['st'] == 'lost', 30.0)
    for _, s in fw.lines_after(0):
        for ch in ('ch1', 'ch2'):
            o = s['drive']['wheel'][ch]
            assert o['raw'] == 'unknown' and o['v'] in ('not_read', 'timeout'), o
    with fw.lock:
        wheels = list(fw.wheels)
    assert wheels and all(is_nan(x) for _, pair in wheels for x in pair)


@pytest.fixture(scope='module')
def after_reset(fw):
    return run_after_banner(fw, 'reset')


def test_machine_reset_restarts_the_sequence_from_zero(fw, after_reset):
    _, new, t_cmd = after_reset
    assert fw.wait(lambda: len(fw.boot_lines(new['boot'], t_cmd)) >= 5, 60.0)
    seqs = [r['seq'] for r in fw.boot_lines(new['boot'], t_cmd)]
    assert seqs[0] == 0 and seqs == list(range(len(seqs))), seqs[:10]


def test_machine_reset_increases_the_boot_id(fw, after_reset):
    old, new, _ = after_reset
    assert new['boot'] == old['boot'] + 1 and new['boot_record'] == 'retained'
    assert fw.status['renode_starts'] == 1


@pytest.fixture(scope='module')
def after_restart(fw, after_reset):
    return run_after_banner(fw, 'restart')


def test_fresh_renode_start_restarts_the_sequence_from_zero(fw, after_restart):
    _, new, t_cmd = after_restart
    assert fw.wait(lambda: len(fw.boot_lines(new['boot'], t_cmd)) >= 5, 60.0)
    seqs = [r['seq'] for r in fw.boot_lines(new['boot'], t_cmd)]
    assert seqs[0] == 0 and seqs == list(range(len(seqs))), seqs[:10]


def test_fresh_renode_start_has_boot_id_1(fw, after_restart):
    old, new, _ = after_restart
    assert old['boot'] == 2
    assert new['boot'] == 1 and new['boot_record'] == 'cold'
    assert fw.status['renode_starts'] == 2


def test_image_without_test_parameters_is_refused():
    """The Debug build prints "params":"default": the node must fail, not run on."""
    renode, _, fw_dir = renode_inputs()
    elf = os.environ.get('SIL_FW_ELF_DEFAULT', '')
    if not elf or not os.path.exists(elf):
        pytest.fail('BLOCKED: SIL_FW_ELF_DEFAULT=%r does not exist' % elf, pytrace=False)
    work = WORK + '_default'
    log = os.path.join(work, 'node.log')
    proc = start_node(elf, renode, fw_dir, work, log)
    try:
        code = proc.wait(timeout=180)
    finally:
        stop_node(proc)
    with open(log, errors='replace') as f:
        text = f.read()
    assert code == 1, text[-2000:]
    assert "banner check failed: banner params 'default', expected 'test'" in text
