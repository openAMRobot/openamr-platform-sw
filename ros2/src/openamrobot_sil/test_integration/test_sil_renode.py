"""End-to-end: the firmware image in Renode on vcan_drive, the drive emulator, Gazebo.

sil.launch.py stm32:=renode_readonly. Real: the openamr-platform-fw telemetry image at the
pinned commit, preset RenodeTest. Emulated: the MCU (Renode), the drive (SIL ZLAC8015D
emulator, wheels from Gazebo), the bus (vcan) and the drive commands (host test client, SIL
receive PDOs only). Needs vcan, Gazebo Harmonic, SIL_RENODE, SIL_FW_ELF and SIL_FW_DIR; a
missing one FAILS with "BLOCKED:". Development evidence only: no board, no drive, no robot.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import can
from geometry_msgs.msg import Twist
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'test_renode'))
from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs, run_after_banner  # noqa: E402,E501

from openamrobot_sil import renode_fw  # noqa: E402

LOG = os.environ.get('SIL_RENODE_LAUNCH_LOG', '/tmp/sil_renode_launch.log')
CAPTURE = os.environ.get('SIL_DRIVE_CAPTURE', '/tmp/openamrobot_sil_vcan_drive.log')
WORK = '/tmp/openamrobot_sil_renode'
DRIVE_STATE = '/sil/drive/emulator_state'
HOST_STATE = '/sil/drive/status'


class Probe(FwProbe):
    def __init__(self):
        super().__init__('sil_renode_test_probe')
        self.last = {}
        self.cmd = None
        for topic in (DRIVE_STATE, HOST_STATE):
            self.create_subscription(String, topic, lambda m, t=topic: self.last.__setitem__(
                t, json.loads(m.data)), 10)
        self.create_subscription(JointState, '/sil/gz/joint_states',
                                 lambda m: self.last.__setitem__('joints', m), 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.inject_pub = self.create_publisher(String, '/sil/drive/inject', 10)
        self.create_timer(0.05, self.send_cmd)

    def send_cmd(self):
        if self.cmd is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.cmd
            self.cmd_pub.publish(msg)

    def gz_wheels(self):
        joints = self.last['joints']
        names = list(joints.name)
        return (joints.velocity[names.index('left_wheel_joint')],
                joints.velocity[names.index('right_wheel_joint')])

    def fw_wheels(self):
        with self.lock:
            return self.wheels[-1][1] if self.wheels else None


def tail_log(lines=80):
    try:
        with open(LOG, encoding='utf-8', errors='replace') as f:
            return ''.join(f.readlines()[-lines:])
    except OSError:
        return '(no launch log)'


@pytest.fixture(scope='module')
def sil():
    for tool in ('gz', 'ros2', 'ip'):
        if shutil.which(tool) is None:
            pytest.fail('BLOCKED: %s is not installed' % tool, pytrace=False)
    renode_inputs()
    print(pinned_line())
    with open(LOG, 'w', encoding='utf-8') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'openamrobot_sil', 'sil.launch.py', 'stm32:=renode_readonly',
             'capture_file:=%s' % CAPTURE, 'renode_work_dir:=%s' % WORK],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    spinner = Spinner(probe)
    try:
        ready = probe.wait(lambda: (
            len(probe.lines_after(0)) >= 5
            and probe.audit is not None
            and probe.last['joints'] is not None
            and probe.last[DRIVE_STATE]['feedback'] == 'gazebo'
            and probe.last[HOST_STATE]['configured']
            and probe.last[HOST_STATE]['mode'] == 'command_only'), timeout=240.0, step=0.2)
        if not ready:
            blocked = 'BLOCKED' in tail_log(400)
            pytest.fail('%s: SIL stack with Renode not ready after 240 s. Seen: %s, fw %s\n%s' % (
                'BLOCKED' if blocked else 'FAILED', sorted(probe.last), probe.status,
                tail_log()), pytrace=False)
        time.sleep(2.0)  # let the robot settle on the floor
        yield probe
    finally:
        probe.cmd = None
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-f', 'gz sim'], check=False)
        subprocess.run(['pkill', '-f', 'Renode'], check=False)
        spinner.stop()
        probe.destroy_node()
        rclpy.shutdown()


def test_banner_is_the_pinned_renode_test_build(sil, record_property):
    _, banner = sil.banners()[0]
    record_property('fw_commit', renode_fw.FW_COMMIT)
    record_property('fw_preset', renode_fw.FW_PRESET)
    print('%s; banner commit %s params %s; CAN1 on %s' % (
        pinned_line(), banner['commit'], banner['params'], sil.status['can_interface']))
    assert banner['params'] == 'test' and renode_fw.FW_COMMIT.startswith(banner['commit'])
    assert sil.status['can_link'] == 'socketcan' and sil.status['can_interface'] == 'vcan_drive'
    # The drive answers through the bridge: heartbeat and SDO answers reach the image.
    assert sil.wait(lambda: sil.last_status_line()['drive']['hb']['st'] == 'alive', 30.0)
    assert sil.wait(lambda: sil.last_status_line()['drive']['sw']['v'] == 'valid', 30.0)


def test_firmware_wheel_speeds_match_gazebo_at_0_2_m_per_s(sil):
    sil.cmd = (0.2, 0.0)
    try:
        time.sleep(3.0)
        fw, gz = sil.fw_wheels(), sil.gz_wheels()
        line = sil.last_status_line()
    finally:
        sil.cmd = (0.0, 0.0)
    print('after 3 s: firmware ch1 %.3f ch2 %.3f rad/s, Gazebo left %.3f right %.3f rad/s' % (
        fw[0], fw[1], gz[0], gz[1]))
    for name in ('ch1', 'ch2'):
        assert line['drive']['wheel'][name]['v'] == 'valid', line['drive']['wheel'][name]
    # 0.2 m/s on a 0.1 m wheel: 2 rad/s. ch1 is the left axis, ch2 the right (SIL convention).
    assert gz[0] > 1.5 and gz[1] > 1.5
    for f, g in zip(fw, gz):
        assert abs(f - g) <= 0.2 * abs(g), (fw, gz)
    assert not {'overrun', 'dropped'} & set(line['cnt']), line['cnt']
    assert sil.wait(lambda: abs(sil.gz_wheels()[0]) < 0.1, 5.0)


def test_stopped_drive_gives_heartbeat_lost_and_unknown_wheels_never_zero(sil):
    sil.cmd = (0.2, 0.0)
    assert sil.wait(lambda: sil.fw_wheels()[0] > 1.5, 10.0)
    t0 = time.monotonic()
    sil.inject_pub.publish(String(data='busoff'))   # the drive emulator falls silent
    try:
        assert sil.wait(lambda: sil.last_status_line()['drive']['hb']['st'] == 'lost', 60.0)
        t_lost = time.monotonic()
        time.sleep(3.0)
        lines = sil.lines_after(t0)
        with sil.lock:
            wheels = [(t, w) for t, w in sil.wheels if t >= t0]
    finally:
        sil.cmd = (0.0, 0.0)
        sil.inject_pub.publish(String(data='busoff-clear'))
    for _, s in lines:
        for ch in ('ch1', 'ch2'):
            assert s['drive']['wheel'][ch]['raw'] != 0, ('a wheel value of 0 was reported',
                                                         s['seq'], s['drive']['wheel'])
    late = [s for t, s in lines if t > t_lost + 1.5]
    assert late
    for s in late:
        assert s['drive']['hb']['st'] == 'lost'
        for ch in ('ch1', 'ch2'):
            o = s['drive']['wheel'][ch]
            assert o['v'] in ('timeout', 'stale') and o['raw'] == 'unknown', o
    late_wheels = [w for t, w in wheels if t > t_lost + 1.5]
    assert late_wheels and all(is_nan(x) for w in late_wheels for x in w)
    assert all(x != 0.0 for _, w in wheels for x in w)
    # the drive comes back
    assert sil.wait(lambda: sil.last_status_line()['drive']['hb']['st'] == 'alive', 60.0)
    assert sil.wait(lambda: sil.last_status_line()['drive']['wheel']['ch1']['v'] == 'valid',
                    60.0)


@pytest.fixture(scope='module')
def after_reset(sil):
    return run_after_banner(sil, 'reset')


def test_machine_reset_restarts_the_sequence_from_zero(sil, after_reset):
    _, _, t_new = after_reset
    assert sil.wait(lambda: len(sil.lines_after(t_new)) >= 5, 60.0)
    seqs = [r['seq'] for _, r in sil.lines_after(t_new)]
    assert seqs[0] == 0 and seqs == list(range(len(seqs))), seqs[:10]


def test_machine_reset_increases_the_boot_id(sil, after_reset):
    old, new, _ = after_reset
    assert new['boot'] == old['boot'] + 1 and new['boot_record'] == 'retained'
    assert sil.status['renode_starts'] == 1


@pytest.fixture(scope='module')
def after_restart(sil, after_reset):
    return run_after_banner(sil, 'restart', timeout=180.0)


def test_fresh_renode_start_restarts_the_sequence_from_zero(sil, after_restart):
    _, _, t_new = after_restart
    assert sil.wait(lambda: len(sil.lines_after(t_new)) >= 5, 60.0)
    seqs = [r['seq'] for _, r in sil.lines_after(t_new)]
    assert seqs[0] == 0 and seqs == list(range(len(seqs))), seqs[:10]
    # the new instance's bridge works: the drive answers again
    assert sil.wait(lambda: sil.last_status_line()['drive']['sw']['v'] == 'valid', 60.0)


def test_fresh_renode_start_has_boot_id_1(sil, after_restart):
    old, new, _ = after_restart
    assert old['boot'] == 2
    assert new['boot'] == 1 and new['boot_record'] == 'cold'
    assert sil.status['renode_starts'] == 2


def audit_capture_file(path):
    audit = renode_fw.SdoAudit(1)
    frames = 0
    with open(path, encoding='utf-8') as f:
        for line in f:
            stamp, _iface, frame = line.split()
            ident, data = frame.split('#')
            audit.feed(int(ident, 16), bytes.fromhex(data), float(stamp.strip('()')),
                       len(ident) > 3)
            frames += 1
    return frames, audit.summary()


def test_host_client_sent_no_sdo_and_never_two_requests_outstanding(sil):
    """Negative test over the whole capture of vcan_drive."""
    time.sleep(1.0)
    live, status, host = sil.audit, sil.status, sil.last[HOST_STATE]
    frames, s = audit_capture_file(live['capture_file'])
    print('capture %s: %d frames; audit %s; firmware tx %d over %d boots' % (
        live['capture_file'], frames, {k: v for k, v in s.items() if k != 'examples'},
        status['fw_tx_total'], len(status['boots'])))
    assert frames >= live['frames'] > 1000
    assert s['non_upload'] == 0, s['examples']
    assert s['overlaps'] == 0, s['examples']
    assert s['uploads'] > 200
    # Every upload request on the bus is one the image counted as sent ("tx"). The image prints
    # its counter every 100 ms, so up to one period of requests (5) per boot is not counted yet.
    extra = s['uploads'] - status['fw_tx_total']
    assert 0 <= extra <= 6 * len(status['boots']), (s['uploads'], status['fw_tx_total'])
    assert host['mode'] == 'command_only' and host['sdo_frames_refused'] == 0
    assert host['pdo_frames_sent'] > 100
    assert sil.last[DRIVE_STATE]['rpdo_frames'] > 100


def test_audit_sees_a_foreign_sdo_request(sil):
    """Control: the capture and audit do see an SDO request the image did not send."""
    before = sil.audit['non_upload']
    bus = can.Bus(interface='socketcan', channel='vcan_drive')
    try:
        bus.send(can.Message(arbitration_id=0x601, data=bytes([0x2F, 0x60, 0x60, 0, 3, 0, 0, 0]),
                             is_extended_id=False))   # 6060h := 3, the value it already has
    finally:
        bus.shutdown()
    assert sil.wait(lambda: sil.audit['non_upload'] == before + 1, 5.0)
