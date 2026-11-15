"""The renode_firmware node in micro-ROS mode with the real image in Renode, without vcan and Gazebo.

Renode-tested: Renode 1.16.1 runs the TelemetryUrosSerialSimBench image of openamr-platform-fw at
the pinned commit (micro-ROS over USART3, serial transport: simulation and bench only); the node
starts the micro-ROS agent on the Renode pseudo-terminal of USART3 and follows /base_fw/*. CAN1,
CAN2 and CAN3 are on Renode hubs with nothing else on them (can_link:=none), so the image sees a
silent drive. Needs SIL_RENODE, SIL_FW_DIR, SIL_FW_ELF_UROS and SIL_UROS_AGENT
(scripts/sil_renode_firmware.sh --uros) and ROS_DOMAIN_ID 0 (the image's domain). Missing inputs
FAIL with "BLOCKED:".
"""
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy

from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs, uros_inputs
from openamrobot_sil import renode_fw

HERE = os.path.dirname(os.path.abspath(__file__))
HELPERS = os.path.join(os.path.dirname(HERE), 'renode', 'sil_renode_helpers.py')
WORK = os.environ.get('SIL_RENODE_UROS_WORK', '/tmp/openamrobot_sil_renode_uros_node')
LOG = os.path.join(WORK, 'node.log')


def start_node(elf, agent, renode, fw_dir):
    os.makedirs(WORK, exist_ok=True)
    with open(LOG, 'w', encoding='utf-8') as f:
        return subprocess.Popen(
            [sys.executable, '-m', 'openamrobot_sil.nodes.renode_fw_node', '--ros-args',
             '-p', 'renode:=%s' % renode, '-p', 'elf:=%s' % elf, '-p', 'fw_dir:=%s' % fw_dir,
             '-p', 'helpers:=%s' % HELPERS, '-p', 'work_dir:=%s' % WORK,
             '-p', 'can_link:=none', '-p', 'transport:=uros', '-p', 'uros_agent:=%s' % agent,
             '-p', 'banner_timeout_s:=150.0'],
            stdout=f, stderr=subprocess.STDOUT, start_new_session=True)


@pytest.fixture(scope='module')
def fw():
    renode, _, fw_dir = renode_inputs()
    elf, agent = uros_inputs()
    print(pinned_line(renode_fw.FW_PRESET_UROS))
    rclpy.init()
    probe = FwProbe('sil_fw_uros_probe')
    spinner = Spinner(probe)
    proc = start_node(elf, agent, renode, fw_dir)
    try:
        if not probe.wait(lambda: len(probe.uros_after(0)) >= 10, 180.0, 0.2):
            with open(LOG, errors='replace') as f:
                tail = f.read()[-3000:]
            pytest.fail('FAILED: no /base_fw/status from the image after 180 s\n%s' % tail,
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


def test_status_topic_is_the_pinned_uros_build(fw, record_property):
    s = fw.last_uros()
    record_property('fw_commit', renode_fw.FW_COMMIT)
    record_property('fw_preset', renode_fw.FW_PRESET_UROS)
    print('%s; status commit %s params %s uros %s' % (
        pinned_line(renode_fw.FW_PRESET_UROS), s['commit'], s['params'], s['uros']))
    assert renode_fw.check_uros_status(s)
    assert s['uros'] == 'run' and s['bms'] == 'locked'
    assert fw.wait(lambda: fw.status['transport'] == 'uros' and fw.status['agent_running'], 10.0)
    assert fw.banner()['commit'] == s['commit']


def test_every_topic_of_the_image_is_there_and_nothing_else(fw):
    names = dict(fw.get_topic_names_and_types())
    for topic, type_name in renode_fw.UROS_TOPICS.items():
        assert type_name in names.get(topic, []), (topic, names.get(topic))
    extra = [t for t in names if t.startswith(renode_fw.UROS_NAMESPACE + '/')
             and t not in renode_fw.UROS_TOPICS]
    assert not extra, extra


def test_silent_drive_gives_nan_wheels_never_zero_and_heartbeat_lost(fw):
    assert fw.wait(lambda: fw.last_hb() == 'lost', 30.0)
    t0 = time.monotonic()
    assert fw.wait(lambda: len(fw.wheels_since_raw(t0)) >= 10, 10.0)
    for w in fw.wheels_since_raw(t0):
        assert all(is_nan(x) for x in w), w
    with fw.lock:
        converted = [w for t, w in fw.wheels if t >= t0]
    assert converted and all(is_nan(x) for w in converted for x in w)


def test_time_ratio_is_measured_and_reported(fw, record_property):
    assert fw.wait(lambda: fw.status['time_ratio'] is not None, 15.0)
    time.sleep(5.0)
    ratio = fw.status['time_ratio']
    record_property('renode_time_ratio', ratio)
    # Reported, not a pass condition: above 0.8 is the goal (see sil-topology.md).
    print('Renode virtual time / wall time: %.3f (goal above 0.8)' % ratio)
    assert 0.0 < ratio < 1.5


@pytest.fixture(scope='module')
def after_reset(fw):
    before = fw.last_uros()
    t_cmd = time.monotonic()
    fw.command('reset')
    assert fw.wait(lambda: any(s['boot'] == before['boot'] + 1 for _, s in fw.uros_after(t_cmd)),
                   90.0), 'no status of a new boot 90 s after the reset'
    return before, t_cmd


def test_machine_reset_increases_the_boot_id_and_restarts_the_sequence(fw, after_reset):
    before, t_cmd = after_reset
    assert fw.wait(lambda: len(fw.uros_boot(before['boot'] + 1, t_cmd)) >= 5, 30.0)
    new = fw.uros_boot(before['boot'] + 1, t_cmd)
    assert new[0]['rec'] == 'retained'
    # The client needs a few periods to reconnect after the reset; the sequence counts report
    # periods from the reset, so it starts low and rises.
    seqs = [s['seq'] for s in new]
    assert seqs[0] < 100 and seqs == sorted(seqs), seqs[:10]
    assert new[0]['up_us'] < 10_000_000


def test_fresh_renode_start_has_boot_id_1_and_the_agent_restarts(fw, after_reset):
    t_cmd = time.monotonic()
    starts = fw.status['renode_starts']
    fw.command('restart')
    assert fw.wait(lambda: fw.status['renode_starts'] == starts + 1, 60.0)
    assert fw.wait(lambda: any(s['boot'] == 1 and s['rec'] == 'cold'
                               for _, s in fw.uros_after(t_cmd)), 150.0)
    assert fw.status['agent_running']
