"""End-to-end: the micro-ROS firmware image in Renode on the three SIL buses, emulators, Gazebo.

sil.launch.py stm32:=renode_readonly. Real: the openamr-platform-fw telemetry image at the pinned
commit, preset TelemetryUrosSerialSimBench (micro-ROS over USART3, serial transport: simulation
and bench only), read through a micro-ROS agent the firmware node starts on the Renode
pseudo-terminal. Emulated: the MCU (Renode), the drive (SIL ZLAC8015D emulator, wheels from
Gazebo), the lift (CTL-017 emulator, lift joint in Gazebo), the BMS (Daly emulator), the buses
(vcan; FDCAN1, FDCAN2, FDCAN3 through Renode's SocketCAN bridges) and the commands (host test
client: drive receive PDOs only, NMT start and commands of the lift, Daly polls). Needs vcan,
Gazebo Harmonic, SIL_RENODE, SIL_FW_DIR, SIL_FW_ELF_UROS, SIL_UROS_AGENT and ROS_DOMAIN_ID 0; a
missing one FAILS with "BLOCKED:". Development evidence only: no board, no drive, no robot.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import can
from geometry_msgs.msg import Twist
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int8, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'test_renode'))
from fw_probe import FwProbe, Spinner, is_nan, pinned_line, renode_inputs, uros_inputs  # noqa: E402,E501

from openamrobot_sil import renode_fw  # noqa: E402
from openamrobot_sil.ctl017_gazebo import JOINT_ZERO_MM  # noqa: E402

LOG = os.environ.get('SIL_RENODE_LAUNCH_LOG', '/tmp/sil_renode_launch.log')
CAPTURE = os.environ.get('SIL_DRIVE_CAPTURE', '/tmp/openamrobot_sil_vcan_drive.log')
AUX_CAPTURE = os.environ.get('SIL_AUX_CAPTURE', '/tmp/openamrobot_sil_vcan_aux.log')
WORK = '/tmp/openamrobot_sil_renode'
DRIVE_STATE = '/sil/drive/emulator_state'
HOST_STATE = '/sil/drive/status'
LIFT_STATE = '/sil/lift/emulator_state'
LIFT_HOST = '/sil/lift/status'
BMS_HOST = '/sil/bms/status'
IDLE = 3                       # CTL-017 state IDLE
TIME_RATIO_GOAL = 0.8          # a goal, reported; not a pass condition


class Probe(FwProbe):
    def __init__(self):
        super().__init__('sil_renode_test_probe')
        self.last = {}
        self.cmd = None
        self.lift_joint = []   # (monotonic time, lift joint position in m)
        for topic in (DRIVE_STATE, HOST_STATE, LIFT_STATE, LIFT_HOST, BMS_HOST):
            self.create_subscription(String, topic, lambda m, t=topic: self.last.__setitem__(
                t, json.loads(m.data)), 10)
        self.create_subscription(JointState, '/sil/gz/joint_states', self.on_joints, 10)
        for topic in ('/odom', '/sil/gz/odom', '/sil/fw/odom'):
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.last.__setitem__(
                t, m), 10)
        self.create_subscription(String, '/sil/fw/odom_state', lambda m: self.last.__setitem__(
            'odom_state', json.loads(m.data)), 10)
        self.clock = []        # (wall monotonic s, Gazebo time s), from the joint state stamps
        # not cmd_pub: that is FwProbe's /sil/fw/command publisher
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.inject_pub = self.create_publisher(String, '/sil/drive/inject', 10)
        self.lift_pub = self.create_publisher(Int8, '/sil/lift/command', 10)
        self.permit_pub = self.create_publisher(Bool, '/sil/lift/permit', 10)
        self.create_timer(0.05, self.send_cmd)

    def on_joints(self, msg):
        self.last['joints'] = msg
        stamp = msg.header.stamp
        with self.lock:
            self.clock.append((time.monotonic(), stamp.sec + stamp.nanosec / 1e9))
            del self.clock[:-3000]
        names = list(msg.name)
        if 'lift_joint' in names:
            with self.lock:
                self.lift_joint.append((time.monotonic(), msg.position[names.index('lift_joint')]))

    def gazebo_rtf(self, seconds):
        """Gazebo real-time factor over the last `seconds` of wall time: Gazebo time of the
        joint state stamps (100 Hz) against the wall time they arrived at."""
        with self.lock:
            window = [c for c in self.clock if c[0] >= self.clock[-1][0] - seconds]
        (w0, g0), (w1, g1) = window[0], window[-1]
        return (g1 - g0) / (w1 - w0), w1 - w0

    def publishers(self, topic):
        return sorted(info.node_name for info in self.get_publishers_info_by_topic(topic))

    def send_cmd(self):
        if self.cmd is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.cmd
            self.twist_pub.publish(msg)

    def gz_wheels(self):
        joints = self.last['joints']
        names = list(joints.name)
        return (joints.velocity[names.index('left_wheel_joint')],
                joints.velocity[names.index('right_wheel_joint')])

    def gz_lift_mm(self, at=None):
        """Gazebo lift joint as mm above the lower end of travel, at time at (nearest sample)."""
        with self.lock:
            samples = list(self.lift_joint)
        if at is None:
            t, pos = samples[-1]
        else:
            t, pos = min(samples, key=lambda s: abs(s[0] - at))
        return JOINT_ZERO_MM + pos * 1000.0

    def wheels_since(self, t0):
        with self.lock:
            return [w for t, w in self.wheels if t >= t0]

    def fw_wheels(self):
        with self.lock:
            return self.wheels[-1][1] if self.wheels else None


class AuxCapture:
    """candump -L capture of vcan_aux (CAN3: CTL-017 and host test client) for the whole run.
    Receive only: it opens the bus and never sends."""

    def __init__(self, path, interface='vcan_aux'):
        self.path = path
        self.interface = interface
        self.frames = 0
        self._stop = threading.Event()
        self.bus = can.Bus(interface='socketcan', channel=interface, receive_own_messages=False)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        with open(self.path, 'w', encoding='utf-8') as f:
            while not self._stop.is_set():
                msg = self.bus.recv(timeout=0.2)
                if msg is None:
                    continue
                f.write(renode_fw.candump_line(msg.timestamp, self.interface, msg.arbitration_id,
                                               msg.data, msg.is_extended_id) + '\n')
                self.frames += 1

    def stop(self):
        self._stop.set()
        self.thread.join(timeout=2)
        self.bus.shutdown()


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
    uros_inputs()
    print(pinned_line(renode_fw.FW_PRESET_UROS))
    with open(LOG, 'w', encoding='utf-8') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'openamrobot_sil', 'sil.launch.py', 'stm32:=renode_readonly',
             'capture_file:=%s' % CAPTURE, 'renode_work_dir:=%s' % WORK, 'robot_state:=true'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    spinner = Spinner(probe)
    aux = None
    try:
        # vcan_aux exists once the launch has created the vcan interfaces.
        if not probe.wait(lambda: os.path.exists('/sys/class/net/vcan_aux'), 60.0, 0.2):
            pytest.fail('FAILED: vcan_aux was not created\n%s' % tail_log(), pytrace=False)
        aux = AuxCapture(AUX_CAPTURE)
        probe.aux_capture = aux
        ready = probe.wait(lambda: (
            len(probe.uros_after(0)) >= 5
            and probe.audit is not None
            and probe.last['joints'] is not None
            and probe.last[DRIVE_STATE]['feedback'] == 'gazebo'
            and probe.last[LIFT_STATE]['plant'] == 'gazebo'
            and probe.last[LIFT_HOST]['state'] == IDLE
            and probe.last[HOST_STATE]['configured']
            and probe.last[HOST_STATE]['mode'] == 'command_only'), timeout=300.0, step=0.2)
        if not ready:
            blocked = 'BLOCKED' in tail_log(400)
            pytest.fail('%s: SIL stack with Renode not ready after 300 s. Seen: %s, fw %s\n%s' % (
                'BLOCKED' if blocked else 'FAILED', sorted(probe.last), probe.status,
                tail_log()), pytrace=False)
        time.sleep(2.0)  # let the robot settle on the floor
        yield probe
    finally:
        probe.cmd = None
        if aux is not None:
            aux.stop()
            print('vcan_aux capture %s: %d frames' % (AUX_CAPTURE, aux.frames))
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-f', 'gz sim'], check=False)
        subprocess.run(['pkill', '-f', 'Renode'], check=False)
        subprocess.run(['pkill', '-f', 'micro_ros_agent'], check=False)
        spinner.stop()
        probe.destroy_node()
        rclpy.shutdown()


def test_status_is_the_pinned_uros_build_and_the_three_buses_reach_it(sil, record_property):
    s = sil.last_uros()
    record_property('fw_commit', renode_fw.FW_COMMIT)
    record_property('fw_preset', renode_fw.FW_PRESET_UROS)
    print('%s; status commit %s params %s; buses %s %s' % (
        pinned_line(renode_fw.FW_PRESET_UROS), s['commit'], s['params'],
        sil.status['can_interface'], sil.status['aux_interfaces']))
    assert renode_fw.check_uros_status(s)
    assert sil.status['transport'] == 'uros' and sil.status['agent_running']
    assert sil.status['can_interface'] == 'vcan_drive'
    assert sil.status['aux_interfaces'] == ['vcan_bms', 'vcan_aux']
    # CAN1: heartbeat and SDO answers through the bridge; CAN3 and CAN2 frames arrive too.
    assert sil.wait(lambda: sil.last_hb() == 'alive', 30.0)
    assert sil.wait(lambda: sil.last_uros()['c3_rx'] > 50 and sil.last_uros()['c2_rx'] > 5, 30.0)
    assert sil.wait(lambda: sil.last_uros()['lift']['hb'] == 'alive', 30.0)


def test_firmware_wheel_speeds_match_gazebo_within_20_percent(sil):
    sil.cmd = (0.2, 0.0)
    try:
        time.sleep(3.0)
        fw, gz = sil.fw_wheels(), sil.gz_wheels()
    finally:
        sil.cmd = (0.0, 0.0)
    print('after 3 s: firmware ch1 %.3f ch2 %.3f rad/s, Gazebo left %.3f right %.3f rad/s' % (
        fw[0], fw[1], gz[0], gz[1]))
    # 0.2 m/s on a 0.1 m wheel: 2 rad/s. ch1 is the left axis, ch2 the right (SIL convention).
    assert gz[0] > 1.5 and gz[1] > 1.5
    for f, g in zip(fw, gz):
        assert abs(f - g) <= 0.2 * abs(g), (fw, gz)
    assert sil.last_uros()['tick_miss'] == 0
    assert sil.wait(lambda: abs(sil.gz_wheels()[0]) < 0.1, 5.0)


def test_stopped_drive_gives_heartbeat_lost_and_nan_wheels_never_zero(sil):
    sil.cmd = (0.2, 0.0)
    # The base turns in Gazebo, and the image has read it turning since then: the last valid
    # value before the stop is not 0.
    assert sil.wait(lambda: min(sil.gz_wheels()) > 1.5, 10.0)
    t_moving = time.monotonic()
    assert sil.wait(
        lambda: sil.wheels_since(t_moving) and min(sil.wheels_since(t_moving)[-1]) > 1.5, 20.0)
    t0 = time.monotonic()
    sil.inject_pub.publish(String(data='busoff'))   # the drive emulator falls silent
    try:
        assert sil.wait(lambda: sil.last_hb() == 'lost', 60.0)
        t_lost = time.monotonic()
        time.sleep(3.0)
        with sil.lock:
            wheels = [(t, w) for t, w in sil.wheels if t >= t0]
            raw = [(t, w) for t, w in sil.uros_raw_wheels if t >= t0]
    finally:
        sil.cmd = (0.0, 0.0)
        sil.inject_pub.publish(String(data='busoff-clear'))
    late = [w for t, w in raw if t > t_lost + 1.5]
    assert late and all(is_nan(x) for w in late for x in w), late[-3:]
    assert all(x != 0.0 for _, w in raw for x in w), 'a wheel value of 0 was published'
    late_conv = [w for t, w in wheels if t > t_lost + 1.5]
    assert late_conv and all(is_nan(x) for w in late_conv for x in w)
    # the drive comes back
    assert sil.wait(lambda: sil.last_hb() == 'alive', 60.0)
    assert sil.wait(lambda: not any(is_nan(x) for x in sil.fw_wheels()), 60.0)


def test_lift_position_reported_by_the_firmware_follows_the_gazebo_lift_joint(sil):
    """The CTL-017 emulator reports the Gazebo joint in TPDO1; the image decodes it on CAN3."""
    # 1. CTL-017 frames reach FDCAN3 of the image through the vcan_aux bridge.
    c3_before = sil.last_uros()['c3_rx']
    assert sil.wait(lambda: sil.last_uros()['c3_rx'] > c3_before + 10, 5.0), (
        'no CAN3 frames reach the image', sil.last_uros()['c3_rx'], sil.aux_capture.frames)
    # 2. Readiness, not a first read: the emulated lift is homed and flags its position valid,
    #    and the image reports that position (TPDO1 with position_valid), before the test reads.
    assert sil.wait(lambda: sil.last[LIFT_STATE]['position_mm'] is not None, 30.0), (
        'lift not homed', sil.last[LIFT_STATE])
    assert sil.wait(lambda: renode_fw.lift_mm(sil.last_uros()) is not None
                    and sil.last_uros()['lift']['st'] == 'IDLE', 30.0), sil.last_uros()['lift']
    # 3. Unit and scale: the emulator's position object (0.1 mm in TPDO1, mm in its state), the
    #    firmware's decoded position and the Gazebo joint (m, 0 at L1000 = 5 mm) agree at rest.
    s = sil.last_uros()
    emu_mm = sil.last[LIFT_STATE]['position_mm']
    print('lift at rest: firmware %.1f mm, emulator %.2f mm, Gazebo %.2f mm' % (
        renode_fw.lift_mm(s), emu_mm, sil.gz_lift_mm()))
    assert renode_fw.lift_mm(s) == pytest.approx(emu_mm, abs=2.0), (s['lift'], emu_mm)
    assert renode_fw.lift_mm(s) == pytest.approx(sil.gz_lift_mm(), abs=2.0), s['lift']
    start = renode_fw.lift_mm(s)
    t0 = time.monotonic()
    sil.permit_pub.publish(Bool(data=True))
    assert sil.wait(lambda: sil.last[LIFT_STATE]['permit_fresh'], 3.0)
    sil.lift_pub.publish(Int8(data=1))            # to L1175: joint 0.175 m
    try:
        assert sil.wait(lambda: sil.last[LIFT_HOST]['active_stop'] == 1, 40.0)
        time.sleep(1.0)
    finally:
        sil.permit_pub.publish(Bool(data=False))
    pairs = []
    for t, st in sil.uros_after(t0):
        fw_mm = renode_fw.lift_mm(st)
        age = st['lift']['age_us']
        # A TPDO1 received between the snapshot's clock read and its decode has no age yet
        # ("unknown": the image never invents one); such a sample cannot be placed in time.
        if fw_mm is not None and isinstance(age, int):
            pairs.append((fw_mm, sil.gz_lift_mm(at=t - age / 1e6)))
    moving = [p for p in pairs if abs(p[0] - start) > 5.0]
    print('lift: %d firmware samples, %d while moving; largest difference to Gazebo %.2f mm; '
          'end %.2f mm (Gazebo %.2f mm)' % (len(pairs), len(moving),
                                            max(abs(a - b) for a, b in pairs),
                                            pairs[-1][0], pairs[-1][1]))
    assert len(moving) >= 10, 'the firmware did not report the lift moving'
    # TPDO1 every 100 ms at most (inhibit 10 ms) and 100 ms status period: compared at the
    # sample time of the TPDO1 (age_us), a 10 mm band holds the bus and Gazebo-step jitter.
    for fw_mm, gz_mm in moving:
        assert abs(fw_mm - gz_mm) <= 10.0, (fw_mm, gz_mm)
    assert pairs[-1][0] == pytest.approx(sil.gz_lift_mm(), abs=2.0)
    assert pairs[-1][0] - start > 150.0


def test_battery_values_appear_only_as_locked(sil):
    t0 = time.monotonic()
    before = sil.last_uros()['bms_dec']
    time.sleep(3.0)
    statuses = [s for _, s in sil.uros_after(t0)]
    assert statuses and all(s['bms'] == 'locked' for s in statuses), {s['bms'] for s in statuses}
    # The decoder is fed: Daly answers arrive on CAN2 and are decoded, but stay locked.
    assert statuses[-1]['bms_dec'] > before
    host = sil.last[BMS_HOST]
    print('BMS emulator answers the host client (%s); the image reports %r with %d frames decoded'
          % ({k: host[k] for k in sorted(host)[:4]}, statuses[-1]['bms'], statuses[-1]['bms_dec']))
    for s in statuses:
        assert not isinstance(s['bms'], dict)


def test_ratio_of_renode_time_to_wall_time_is_reported(sil, record_property):
    assert sil.wait(lambda: sil.status['time_ratio'] is not None, 15.0)
    ratio = sil.status['time_ratio']
    record_property('renode_time_ratio', round(ratio, 3))
    print('Renode virtual time / wall time over the last 10 s: %.3f (goal above %.1f%s)' % (
        ratio, TIME_RATIO_GOAL, '' if ratio > TIME_RATIO_GOAL else
        '; below the goal: see docs/architecture/sil-topology.md "Time"'))
    assert 0.0 < ratio < 1.5


def test_agent_talks_to_the_image_through_the_pty_relay(sil):
    """USART3 of the image is not handed to the agent directly. The relay drains Renode's
    pseudo-terminal and carries the XRCE bytes both ways; the agent sits on the relay's link."""
    relay = sil.status['relay']
    print('pty relay: %s' % relay)
    assert sil.status['agent_running'] and relay['opened'] and relay['alive']
    assert relay['link'].endswith('.link') and relay['renode_pty'].endswith('.pty')
    assert relay['to_host_bytes'] > 1000 and relay['to_fw_bytes'] > 50
    assert relay['errors'] == 0, relay
    before = relay['to_host_bytes']
    assert sil.wait(lambda: sil.status['relay']['to_host_bytes'] > before + 1000, 10.0)


def test_bus_loss_on_fdcan1_gives_lost_heartbeat_and_nan_wheels_then_recovers(sil):
    """FDCAN1 is detached from its hub inside Renode: the image is cut off from vcan_drive
    while the drive emulator and the host client go on."""
    assert sil.wait(lambda: sil.last_hb() == 'alive', 60.0)
    assert sil.wait(lambda: not any(is_nan(x) for x in sil.fw_wheels()), 60.0)
    t0 = time.monotonic()
    sil.command('can1_off')
    try:
        assert sil.wait(lambda: sil.last_hb() == 'lost', 60.0)
        t_lost = time.monotonic()
        time.sleep(3.0)
        with sil.lock:
            raw = [(t, w) for t, w in sil.uros_raw_wheels if t >= t0]
    finally:
        sil.command('can1_on')
    late = [w for t, w in raw if t > t_lost + 1.5]
    assert late and all(is_nan(x) for w in late for x in w), late[-3:]
    assert sil.wait(lambda: sil.last_hb() == 'alive', 60.0)
    assert sil.wait(lambda: not any(is_nan(x) for x in sil.fw_wheels()), 60.0)


def test_renode_time_ratio_and_gazebo_real_time_factor_are_reported_separately(
        sil, record_property):
    """Two clocks, two numbers (audit item SW-07). Renode: virtual against wall time, measured
    inside Renode. Gazebo: the plant must keep real time against the emulators' wall-clock
    timers. They are never combined."""
    time.sleep(3.0)
    t = sil.status['time']
    rtf, window = sil.gazebo_rtf(3.0)
    print('Renode virtual/wall ratio %.2f over %.1f s of wall time (recent %.2f, throttle %s; '
          'from the image uptime %.2f); Gazebo real-time factor %.2f over %.1f s of wall time' % (
              t['ratio'], t['wall_s'], t['ratio_recent'], t['throttle'],
              sil.status['time_ratio'], rtf, window))
    record_property('renode_virtual_wall_ratio', '%.3f' % t['ratio'])
    record_property('gazebo_real_time_factor', '%.3f' % rtf)
    # Renode's own time source holds the micro-ROS image at wall rate; 2 % for the sampling.
    assert 0.0 < t['ratio'] <= 1.02, t
    assert window >= 2.5 and rtf <= 1.05
    if rtf < 0.8:
        pytest.fail('BLOCKED: Gazebo real-time factor %.2f is below 0.8: the plant is slow '
                    'against the wall-clock timers of the emulators, results do not count'
                    % rtf, pytrace=False)


def test_robot_state_topics_have_one_writer_each(sil):
    """Audit item SW-08 with robot_state:=true: /odom, odom to base_footprint and
    /joint_states exist and each has one writer. /odom comes from the wheel speeds the image
    reports, so it is coarse; the Gazebo odometry is the reference."""
    sil.cmd = (0.2, 0.0)
    try:
        assert sil.wait(lambda: sil.last.get('/odom') is not None
                        and sil.last.get('/sil/gz/odom') is not None, 30.0)
        x0 = sil.last['/odom'].pose.pose.position.x
        g0 = sil.last['/sil/gz/odom'].pose.pose.position.x
        time.sleep(4.0)
    finally:
        sil.cmd = (0.0, 0.0)
    moved_fw = sil.last['/odom'].pose.pose.position.x - x0
    moved_gz = sil.last['/sil/gz/odom'].pose.pose.position.x - g0
    print('4 s at 0.2 m/s: /odom moved %.2f m, Gazebo odometry %.2f m; odom state %s' % (
        moved_fw, moved_gz, sil.last['odom_state']))
    assert sil.publishers('/odom') == ['sil_fw_odometry']
    assert sil.publishers('/joint_states') == ['sil_fw_odometry']
    assert sil.publishers('/tf') == ['robot_state_publisher', 'sil_fw_odometry']
    assert sil.publishers('/cmd_vel') == ['sil_renode_test_probe']
    assert sil.publishers('/sil/fw/wheel_speeds') == ['sil_renode_firmware']
    assert moved_gz > 0.4
    assert 0.4 * moved_gz <= moved_fw <= 1.6 * moved_gz, (moved_fw, moved_gz)
    assert sil.last['/odom'].child_frame_id == 'base_footprint'
    assert sil.wait(lambda: abs(sil.gz_wheels()[0]) < 0.1, 10.0)


@pytest.fixture(scope='module')
def after_reset(sil):
    before = sil.last_uros()
    t_cmd = time.monotonic()
    sil.command('reset')
    assert sil.wait(lambda: sil.uros_boot(before['boot'] + 1, t_cmd), 120.0), (
        'no status of a new boot 120 s after the reset')
    return before, t_cmd


def test_machine_reset_restarts_the_sequence(sil, after_reset):
    before, t_cmd = after_reset
    assert sil.wait(lambda: len(sil.uros_boot(before['boot'] + 1, t_cmd)) >= 5, 60.0)
    seqs = [s['seq'] for s in sil.uros_boot(before['boot'] + 1, t_cmd)]
    assert seqs[0] < 100 and seqs == sorted(seqs), seqs[:10]


def test_machine_reset_increases_the_boot_id(sil, after_reset):
    before, t_cmd = after_reset
    new = sil.uros_boot(before['boot'] + 1, t_cmd)[0]
    assert new['rec'] == 'retained'
    assert sil.status['renode_starts'] == 1


@pytest.fixture(scope='module')
def after_restart(sil, after_reset):
    t_cmd = time.monotonic()
    sil.command('restart')
    assert sil.wait(lambda: sil.uros_boot(1, t_cmd), 180.0), 'no status 180 s after the restart'
    return t_cmd


def test_fresh_renode_start_restarts_the_sequence_and_the_agent(sil, after_restart):
    assert sil.wait(lambda: len(sil.uros_boot(1, after_restart)) >= 5, 60.0)
    seqs = [s['seq'] for s in sil.uros_boot(1, after_restart)]
    assert seqs[0] < 100 and seqs == sorted(seqs), seqs[:10]
    assert sil.status['agent_running']
    # the new instance's bridges work: the drive and the lift answer again
    assert sil.wait(lambda: sil.last_hb() == 'alive', 60.0)
    assert sil.wait(lambda: sil.last_uros()['lift']['hb'] == 'alive', 60.0)


def test_fresh_renode_start_has_boot_id_1(sil, after_restart):
    new = sil.uros_boot(1, after_restart)[0]
    assert new['rec'] == 'cold'
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
    # Every upload request on the bus is one the image counted as sent ("tx" of its status
    # topic). The status is published every 100 ms and the client reconnects after each reset,
    # so up to a few periods of requests per boot are not counted yet.
    # While FDCAN1 was detached (bus loss test) the image went on counting requests that never
    # reached the bus, so the lower bound allows for that time; the firmware tap gives the
    # exact comparison in the next test.
    extra = s['uploads'] - status['fw_tx_total']
    assert -400 <= extra <= 60 * len(status['boots']), (s['uploads'], status['fw_tx_total'])
    # Two clients on one bus (audit item SW-11): every write on vcan_drive is the host
    # client's. No NMT frame at all, and the receive PDOs on the bus are the ones it counted.
    assert s['nmt'] == 0, s['examples']
    assert -30 <= sil.last[HOST_STATE]['pdo_frames_sent'] - s['pdo_writes'] <= 150, (
        s['pdo_writes'], host['pdo_frames_sent'])
    assert host['mode'] == 'command_only' and host['sdo_frames_refused'] == 0
    assert host['pdo_frames_sent'] > 100
    assert sil.last[DRIVE_STATE]['rpdo_frames'] > 100


def test_whole_run_the_firmware_transmitted_only_sdo_uploads_and_nothing_on_can2_can3(sil):
    """Read-only proof from the firmware side, with the sender known: every frame FDCAN1,
    FDCAN2 or FDCAN3 transmitted, over every boot and Renode start of this run. FDCAN2 and
    FDCAN3 are active here (the image counted frames it received on them), and the audit
    must show zero frames transmitted on those two buses. Any NMT, SDO download, receive PDO
    (so any write to 6040h, 60FFh or 2000h) or other frame fails."""
    tx_total = sil.status['fw_tx_total']     # the image's count first, then the tap
    audit = renode_fw.FirmwareTxAudit()
    starts = sil.status['renode_starts']
    for start in range(1, starts + 1):
        audit.feed_file(os.path.join(WORK, 'fwtx-%d.log' % start))
    a, live, boots = audit.summary(), sil.tx_audit, len(sil.status['boots'])
    _, bus = audit_capture_file(sil.audit['capture_file'])
    u = sil.last_uros()
    print('firmware tap: %d frames by peripheral %s, uploads %d, findings %s; image tx %d over '
          '%d boots and %d Renode starts; uploads on vcan_drive %d; the image received %d '
          'frames on CAN2 and %d on CAN3 in its current boot' % (
              a['frames'], a['by_label'], a['uploads'], a['findings'], tx_total, boots, starts,
              bus['uploads'], u['c2_rx'], u['c3_rx']))
    assert starts == 2 and boots >= 3 and a['frames'] > 300
    assert u['c2_rx'] > 5 and u['c3_rx'] > 50          # the two buses are active
    assert a['by_label'].get('fdcan2', 0) == 0 and a['by_label'].get('fdcan3', 0) == 0
    assert set(a['by_label']) == {'fdcan1'}
    assert a['violations'] == 0 and live['violations'] == 0, a['examples']
    assert all(count == 0 for count in a['findings'].values()), a['findings']
    # The tap agrees with the image's own counter (status every 100 ms, reconnects after a
    # reset) and with the bus, except while FDCAN1 was detached.
    assert -5 <= a['uploads'] - tx_total <= 60 * boots, (a['uploads'], tx_total)
    assert a['uploads'] >= bus['uploads'] > 0.5 * a['uploads']


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


def test_duplicate_sdo_owner_on_vcan_is_detected_and_the_image_keeps_its_values(sil):
    """Last test: it breaks the one-owner rule on purpose. A second client reads 6041h of
    the drive. The bus then carries more uploads than the image transmitted (the firmware tap
    knows the sender), and the wheel values the image publishes stay the drive's."""
    assert sil.wait(lambda: not any(is_nan(x) for x in sil.fw_wheels()), 60.0)
    _, before = audit_capture_file(sil.audit['capture_file'])
    tap_before = sil.tx_audit['uploads']
    t0 = time.monotonic()
    bus = can.Bus(interface='socketcan', channel='vcan_drive')
    sent = 0
    try:
        for _ in range(40):
            bus.send(can.Message(arbitration_id=0x601, is_extended_id=False,
                                 data=bytes([0x40, 0x41, 0x60, 0, 0, 0, 0, 0])))
            sent += 1
            time.sleep(0.05)
    finally:
        bus.shutdown()
    time.sleep(2.0)
    _, after = audit_capture_file(sil.audit['capture_file'])
    foreign = (after['uploads'] - before['uploads']) - (sil.tx_audit['uploads'] - tap_before)
    wheels = sil.wheels_since(t0)
    print('foreign uploads sent %d; uploads on the bus beyond the firmware tap %d; %d wheel '
          'samples from the image meanwhile' % (sent, foreign, len(wheels)))
    assert abs(foreign - sent) <= 15
    # The base is at rest: no foreign 6041h answer (0237h and the like) became a wheel speed.
    assert wheels and all(is_nan(x) or abs(x) < 0.2 for w in wheels for x in w), wheels[-3:]
    assert sil.tx_audit['violations'] == 0
