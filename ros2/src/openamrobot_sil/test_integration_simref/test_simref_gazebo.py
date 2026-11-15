"""End-to-end tests of stm32:=simref with Gazebo and Nav2.

The SIMULATION ONLY reference process of openamr-platform-fw tests/sim_reference (pinned in
launch/sil.launch.py) takes the STM32 role on vcan. Needs Linux vcan, Gazebo Harmonic, Nav2
and the reference binary; when one is missing the tests FAIL with a line starting "BLOCKED:".
Development evidence only: nothing here is about the board, the real drive, the hardwired
stop chain, electrical CAN behaviour or microcontroller timing. The tests run in file order
on one launch; each test leaves the base LOCKED or RUN and the next one re-arms as needed.

A whole-run capture of every virtual bus is written to SIMREF_CAPTURE_DIR.
"""
import json
import os
import shutil
import signal
import subprocess
import threading
import time

import can
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from tf2_msgs.msg import TFMessage

LOG = os.environ.get('SIL_LAUNCH_LOG', '/tmp/sil_simref_launch.log')
CAPTURE_DIR = os.environ.get('SIMREF_CAPTURE_DIR', '/tmp/sil_simref_captures')
BUSES = ('vcan_drive', 'vcan_aux', 'vcan_bms')
WATCHDOG_S = 0.200        # SIMREF_CMD_WATCHDOG_MS, proposed test value, not approved
DRIVE_PERIOD_S = 0.020    # SIMREF_DRIVE_PERIOD_MS, proposed test value, not approved
HB_CONSUMER_S = 0.300     # SIMREF_DRIVE_HB_CONSUMER_MS, proposed test value, not approved
PERMIT_EXPIRY_S = 0.150   # config/sil_simref.yaml lift.permit.timeout_ms
# Receipt of the last /cmd_vel by this test's subscriber and by the bridge differ by DDS
# delivery and thread scheduling; this margin is that measurement tolerance, not a budget of
# the reference.
MEASURE_MARGIN_S = 0.015
REFUSED = 0x80
MOVING = 4                # CTL-017 state MOVING (vendored emulator)


class CanTap:
    """Listens on one vcan bus in this process and keeps (time.time(), id, data)."""

    def __init__(self, channel):
        self.bus = can.Bus(interface='socketcan', channel=channel, receive_own_messages=False)
        self.frames = []
        self.lock = threading.Lock()
        self.run = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.run:
            m = self.bus.recv(timeout=0.1)
            if m is not None and not m.is_extended_id:
                with self.lock:
                    self.frames.append((m.timestamp, m.arbitration_id, bytes(m.data)))

    def since(self, t0, cob=None):
        with self.lock:
            return [f for f in self.frames if f[0] >= t0 and (cob is None or f[1] == cob)]

    def close(self):
        self.run = False
        self.thread.join(timeout=1)
        self.bus.shutdown()


def sdo_writes(frames, index, sub=None):
    """(time, value) of expedited SDO download requests to node 1 for index[:sub]."""
    out = []
    for t, cob, d in frames:
        if cob == 0x601 and len(d) == 8 and d[0] in (0x23, 0x2B, 0x2F) and d[1] | (d[2] << 8) == index \
                and (sub is None or d[3] == sub):
            out.append((t, int.from_bytes(d[4:8], 'little', signed=True)))
    return out


class Probe(Node):
    def __init__(self):
        super().__init__('sil_simref_test_probe')
        self.last, self.reasons, self.cmd_vel_rx = {}, [], []
        self.lock = threading.Lock()
        self.create_subscription(String, '/sil/simref/state', lambda m: self.put('state', json.loads(m.data)), 10)
        self.create_subscription(String, '/sil/simref/reasons', self.on_reason, 100)
        for t in ('/sil/drive/emulator_state', '/sil/lift/emulator_state'):
            self.create_subscription(String, t, lambda m, t=t: self.put(t, json.loads(m.data)), 10)
        self.create_subscription(Odometry, '/sil/gz/odom', lambda m: self.put('gz_odom', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.put('odom', m), 10)
        self.create_subscription(JointState, '/sil/gz/joint_states', lambda m: self.put('joints', m), 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 50)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.pin_pub = self.create_publisher(String, '/sil/simref/pin', 10)
        self.sup_pub = self.create_publisher(String, '/sil/simref/supervisory', 10)
        self.drive_inject = self.create_publisher(String, '/sil/drive/inject', 10)
        self.lift_inject = self.create_publisher(String, '/sil/lift/inject', 10)
        self.lift_pub = self.create_publisher(Int8, '/sil/lift/command', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.odom_tf_seen = 0
        self.teleop = None
        self.create_timer(0.05, self.on_teleop)

    def put(self, key, value):
        with self.lock:
            self.last[key] = value

    def get(self, key):
        with self.lock:
            return self.last.get(key)

    def on_reason(self, m):
        with self.lock:
            self.reasons.append(json.loads(m.data))

    def on_cmd_vel(self, m):
        with self.lock:
            self.cmd_vel_rx.append((time.time(), m.linear.x, m.angular.z))

    def on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                self.odom_tf_seen += 1

    def on_teleop(self):
        if self.teleop is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.teleop
            self.nav_pub.publish(msg)

    # helpers
    def s(self):
        return self.get('state') or {}

    def pins(self, **levels):
        self.pin_pub.publish(String(data=' '.join('%s %d' % kv for kv in levels.items())))

    def sup(self, name):
        self.sup_pub.publish(String(data=name))

    def wait(self, cond, timeout, step=0.01):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return True
            except (KeyError, TypeError, ValueError, AttributeError, IndexError):
                pass
            time.sleep(step)
        return False

    def reasons_after(self, n):
        with self.lock:
            return [r for r in self.reasons if r['n'] > n or r.get('restart')]

    def last_reason_n(self):
        return (self.s().get('reason_log_n') or 0) - 1

    def press_reset(self, hold_s=0.5):
        self.pins(reset=1)
        time.sleep(hold_s)
        self.pins(reset=0)

    def arm(self, timeout=6.0):
        """Documented reset sequence: nominal pins, Jetson Stop released, RESET pressed and
        released (the enable sequence runs while it is held)."""
        if self.s().get('state') == 'RUN':
            return True
        self.teleop = None
        self.pins(y32=1, estop_contact=0, reset=0, charger_fault=0, contactor_fb=1)
        time.sleep(0.3)
        self.sup('jetson_stop_release')
        time.sleep(0.2)
        self.press_reset()
        return self.wait(lambda: self.s()['state'] == 'RUN', timeout)

    def gz_v(self):
        return self.get('gz_odom').twist.twist.linear.x

    def gz_x(self):
        return self.get('gz_odom').pose.pose.position.x

    def lift_pos(self):
        j = self.get('joints')
        return j.position[list(j.name).index('lift_joint')]


def tail_log(lines=80):
    try:
        with open(LOG, encoding='utf-8', errors='replace') as f:
            return ''.join(f.readlines()[-lines:])
    except OSError:
        return '(no launch log)'


def pid_of(pattern, exact=False):
    r = subprocess.run(['pgrep', '-x' if exact else '-f', pattern], capture_output=True, text=True)
    pids = [int(p) for p in r.stdout.split()]
    return pids[0] if pids else None


@pytest.fixture(scope='module')
def sref():
    for tool in ('gz', 'ros2', 'ip'):
        if shutil.which(tool) is None:
            pytest.fail('BLOCKED: %s is not installed' % tool, pytrace=False)
    from ament_index_python.packages import get_package_share_directory
    sil_dir = get_package_share_directory('openamrobot_sil')
    r = subprocess.run(['sh', os.path.join(sil_dir, 'scripts', 'sil_vcan.sh'), 'up'] + list(BUSES),
                       capture_output=True, text=True)
    if r.returncode != 0:
        pytest.fail('BLOCKED: vcan: %s' % r.stderr.strip(), pytrace=False)
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    # whole-run capture of every bus, started before anything sends
    loggers = [subprocess.Popen(['python3', '-m', 'can.logger', '-i', 'socketcan', '-c', bus,
                                 '-f', os.path.join(CAPTURE_DIR, '%s.log' % bus)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
               for bus in BUSES]
    taps = {b: CanTap(b) for b in ('vcan_drive', 'vcan_aux')}
    with open(LOG, 'w', encoding='utf-8') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'openamrobot_sil', 'sil.launch.py', 'stm32:=simref',
                                   'nav2:=true'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    probe.taps = taps
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(probe)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        ready = probe.wait(lambda: (
            probe.get('gz_odom') is not None
            and probe.s()['drive']['phase'] == 'IDLE'
            and probe.get('/sil/drive/emulator_state')['feedback'] == 'gazebo'
            and probe.get('/sil/lift/emulator_state')['plant'] == 'gazebo'
            and probe.s()['lift']['nmt'] == 5
            and probe.get('/sil/lift/emulator_state')['state'] == 3
            and len(probe.get_publishers_info_by_topic('/cmd_vel')) >= 1), timeout=240.0, step=0.2)
        if not ready or not probe.nav.wait_for_server(timeout_sec=120.0):
            blocked = 'BLOCKED' in tail_log(600)
            pytest.fail('%s: simref SIL stack not ready. Seen: %s\n%s' % (
                'BLOCKED' if blocked else 'FAILED', sorted(probe.last), tail_log()), pytrace=False)
        time.sleep(2.0)
        yield probe
    finally:
        probe.teleop = None
        for pid in (pid_of('velocity_smoother'), pid_of('simref_process', exact=True)):
            if pid:
                os.kill(pid, signal.SIGCONT)
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-f', 'gz sim'], check=False)
        for lg in loggers:
            lg.send_signal(signal.SIGINT)
            try:
                lg.wait(timeout=10)
            except subprocess.TimeoutExpired:
                lg.kill()
        for t in taps.values():
            t.close()
        executor.shutdown()
        rclpy.shutdown()


# ---- single writer ------------------------------------------------------------------------
SINGLE_WRITER = {'/cmd_vel': 'collision_monitor', '/odom': 'sil_simref_bridge',
                 '/joint_states': 'sil_simref_bridge'}


def single_writer_violations(node):
    out = []
    for topic, owner in SINGLE_WRITER.items():
        names = sorted(i.node_name for i in node.get_publishers_info_by_topic(topic))
        if names != [owner]:
            out.append('%s: publishers %s, expected exactly [%s]' % (topic, names, owner))
    tf_pubs = sorted({i.node_name for i in node.get_publishers_info_by_topic('/tf')})
    # robot_state_publisher publishes only URDF joints and the URDF has no odom link, so the
    # bridge is the only node that can publish odom -> base_footprint.
    extra = [n for n in tf_pubs if n not in ('robot_state_publisher', 'sil_simref_bridge')]
    if extra or 'sil_simref_bridge' not in tf_pubs:
        out.append('/tf: publishers %s; the odom transform must come from sil_simref_bridge only' % tf_pubs)
    return out


def test_00_single_writer_after_arbitration(sref):
    assert sref.wait(lambda: not single_writer_violations(sref), 30.0), single_writer_violations(sref)
    from ament_index_python.packages import get_package_share_directory
    xacro = os.path.join(get_package_share_directory('openamrobot_sil'), 'urdf', 'sil_robot.urdf.xacro')
    urdf = subprocess.run(['xacro', xacro], capture_output=True, text=True, check=True).stdout
    assert 'link name="odom"' not in urdf
    assert sref.wait(lambda: sref.odom_tf_seen > 10, 5.0)
    # negative control: a second /odom writer must be reported
    intruder = sref.create_publisher(Odometry, '/odom', 10)
    try:
        assert sref.wait(lambda: any('/odom' in v for v in single_writer_violations(sref)), 10.0)
    finally:
        sref.destroy_publisher(intruder)
    assert sref.wait(lambda: not single_writer_violations(sref), 10.0)


# ---- 1. Nav2 goal ---------------------------------------------------------------------------
def test_01_nav2_drives_to_a_goal_3_m_away(sref):
    assert sref.arm(), sref.s()
    x0 = sref.get('odom').pose.pose.position.x
    gz0 = sref.gz_x()
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'odom'
    goal.pose.pose.position.x = x0 + 3.0
    goal.pose.pose.orientation.w = 1.0
    fut = sref.nav.send_goal_async(goal)
    assert sref.wait(lambda: fut.done(), 10.0)
    handle = fut.result()
    assert handle.accepted
    res = handle.get_result_async()
    assert sref.wait(lambda: res.done(), 120.0, 0.1), 'Nav2 did not finish in 120 s: %s' % sref.s().get('state')
    assert res.result().status == 4, 'goal status %d' % res.result().status   # SUCCEEDED
    odom_x = sref.get('odom').pose.pose.position.x - x0
    assert odom_x == pytest.approx(3.0, abs=0.35), odom_x
    assert sref.gz_x() - gz0 == pytest.approx(odom_x, rel=0.2)
    assert sref.s()['state'] == 'RUN'


# ---- 2. watchdog ----------------------------------------------------------------------------
def test_02_cmd_vel_stops_base_stops_within_watchdog_plus_one_period(sref):
    assert sref.arm()
    sref.teleop = (0.15, 0.0)
    assert sref.wait(lambda: sref.gz_v() > 0.12, 8.0)
    sref.teleop = None
    smoother = pid_of('velocity_smoother')
    assert smoother, 'velocity_smoother process not found'
    os.kill(smoother, signal.SIGSTOP)     # the arbitration output stops while non-zero
    try:
        time.sleep(1.5)
        rx = [c for c in sref.cmd_vel_rx if c[1] > 0.05]
        t_last = rx[-1][0]
        zeros = [t for t, v in sdo_writes(sref.taps['vcan_drive'].since(t_last), 0x60FF) if v == 0]
        assert zeros, 'no zero target written after the last /cmd_vel'
        dt = zeros[0] - t_last
        print('last /cmd_vel to first zero 60FFh write: %.3f s' % dt)
        assert WATCHDOG_S - 0.01 <= dt <= WATCHDOG_S + DRIVE_PERIOD_S + MEASURE_MARGIN_S, dt
        assert sref.s()['state'] == 'RUN'          # T11 is not latching
        assert sref.wait(lambda: abs(sref.gz_v()) < 0.01, 3.0)
    finally:
        os.kill(smoother, signal.SIGCONT)


# ---- 3. drive heartbeat lost -----------------------------------------------------------------
def test_03_drive_heartbeat_lost_inhibits_and_reset_needs_new_enable(sref):
    assert sref.arm()
    sref.teleop = (0.1, 0.0)
    assert sref.wait(lambda: sref.gz_v() > 0.08, 8.0)
    n0 = sref.last_reason_n()
    t0 = time.time()
    # the drive keeps answering SDO; only its node 1 heartbeat stops
    sref.drive_inject.publish(String(data='heartbeat-off'))
    assert sref.wait(lambda: sref.s()['state'] == 'LOCKED', 2.0, 0.005)
    t_lock = time.time()
    assert HB_CONSUMER_S - 0.12 <= t_lock - t0 <= HB_CONSUMER_S + 0.25, t_lock - t0
    assert sref.s()['drive']['detail'] == 'HB_TIMEOUT'
    assert any(r['reason'] == 'DRIVE_COMMS' for r in sref.reasons_after(n0))
    time.sleep(1.0)
    late = [f for f in sref.taps['vcan_drive'].since(t_lock + 0.05) if f[1] == 0x601]
    assert late == [], 'frames to the drive after the loss: %d' % len(late)
    assert abs(sref.gz_v()) < 0.01
    # drive comes back and is power-cycled: no enable before a RESET
    sref.drive_inject.publish(String(data='heartbeat-on'))
    time.sleep(0.5)
    t_pc = time.time()
    sref.drive_inject.publish(String(data='power-cycle'))
    time.sleep(2.0)
    assert sref.s()['drive']['phase'] == 'IDLE'
    assert sdo_writes(sref.taps['vcan_drive'].since(t_pc), 0x6040) == []
    assert not sref.get('/sil/drive/emulator_state')['enabled']
    assert abs(sref.gz_v()) < 0.01
    t_arm = time.time()
    assert sref.arm()
    words = [v for _, v in sdo_writes(sref.taps['vcan_drive'].since(t_arm), 0x6040)]
    assert words[:3] == [0x06, 0x07, 0x0F], words
    assert sref.wait(lambda: sref.gz_v() > 0.05, 5.0)
    sref.teleop = None


# ---- 4. E-stop ------------------------------------------------------------------------------
def test_04_estop_stops_motion_and_needs_the_reset_sequence(sref):
    assert sref.arm()
    sref.teleop = (0.15, 0.0)
    assert sref.wait(lambda: sref.gz_v() > 0.12, 8.0)
    n0 = sref.last_reason_n()
    t0 = time.time()
    sref.pins(y32=0, estop_contact=1, contactor_fb=0)   # chain opens, contactor follows
    assert sref.wait(lambda: sref.s()['state'] == 'LOCKED', 1.0, 0.005)
    zeros = [t for t, v in sdo_writes(sref.taps['vcan_drive'].since(t0), 0x60FF) if v == 0]
    assert zeros and zeros[0] - t0 < 0.1
    got = [r['reason'] for r in sref.reasons_after(n0)]
    assert 'HW_SAFETY_NOT_ENABLED' in got and 'ESTOP_DIAG' in got, got
    assert sref.wait(lambda: abs(sref.gz_v()) < 0.01, 3.0)
    time.sleep(1.0)
    assert abs(sref.gz_v()) < 0.01               # /cmd_vel still asks for motion
    sref.pins(y32=1, estop_contact=0, contactor_fb=1)  # E-stop released: still no motion
    time.sleep(1.5)
    assert sref.s()['state'] == 'LOCKED' and abs(sref.gz_v()) < 0.01
    sref.sup('jetson_stop_release')
    sref.press_reset()
    assert sref.wait(lambda: sref.s()['state'] == 'RUN', 5.0)
    assert sref.wait(lambda: sref.gz_v() > 0.05, 5.0)
    sref.teleop = None


# ---- 5. contactor ---------------------------------------------------------------------------
def test_05_contactor_feedback_disagrees_inhibit_with_reason(sref):
    assert sref.arm()
    n0 = sref.last_reason_n()
    t0 = time.time()
    sref.pins(contactor_fb=0)                    # commanded closed (SAFETY_OK high), reads open
    assert sref.wait(lambda: sref.s()['state'] == 'LOCKED', 2.0, 0.005)
    assert 0.09 <= time.time() - t0 <= 0.5
    s = sref.s()
    assert s['input_detail'] == 'CONTACTOR_FEEDBACK'
    assert s['first_reason'] == 'INPUT_INVALID:IMPLAUSIBLE'
    assert any(r['reason'] == 'INPUT_INVALID:IMPLAUSIBLE' for r in sref.reasons_after(n0))
    sref.pins(contactor_fb=1)


# ---- 6. CiA 402 transition delayed, SDO abort -------------------------------------------------
def test_06_delayed_transition_and_sdo_abort_no_motion_reason_reported(sref):
    sref.pins(y32=1, estop_contact=0, reset=0, charger_fault=0, contactor_fb=1)
    time.sleep(0.3)
    if sref.s()['state'] == 'RUN':
        sref.sup('ui_stop')
        sref.pins(y32=0)
        time.sleep(0.2)
        sref.pins(y32=1)
        time.sleep(0.3)
    assert sref.s()['state'] == 'LOCKED'
    sref.teleop = (0.1, 0.0)
    sref.sup('jetson_stop_release')
    for inject, detail, reason in (('delay-transition-300', 'TRANSITION_TIMEOUT', 'DRIVE_COMMS'),
                                   ('abort-enable', 'SDO_ABORT', 'DRIVE_CONFIG')):
        sref.drive_inject.publish(String(data='delay-clear'))
        sref.drive_inject.publish(String(data=inject))
        time.sleep(0.3)
        n0 = sref.last_reason_n()
        sref.press_reset(0.8)
        assert sref.wait(lambda d=detail: sref.s()['drive']['detail'] == d, 3.0), sref.s()['drive']
        time.sleep(0.5)
        s = sref.s()
        assert s['state'] == 'LOCKED' and s['drive']['phase'] == 'FAILED', s['drive']
        assert any(r['reason'] == reason for r in sref.reasons_after(n0))
        if inject == 'abort-enable':
            assert s['drive']['last_abort_index'] == 0x6040 and s['drive']['last_abort'] == 0x08000000
        emu = sref.get('/sil/drive/emulator_state')
        assert not emu['enabled'] and emu['demand_rpm'] == [0.0, 0.0]
        assert abs(sref.gz_v()) < 0.01
    sref.drive_inject.publish(String(data='delay-clear'))
    sref.drive_inject.publish(String(data='abort-clear'))
    time.sleep(0.3)
    sref.press_reset()
    assert sref.wait(lambda: sref.s()['state'] == 'RUN', 5.0)
    sref.teleop = None


# ---- 7. lift permit ---------------------------------------------------------------------------
def test_07_lift_moves_only_with_a_valid_permit_and_expiry_stops_it(sref):
    assert sref.arm()
    # no permit while the base moves: the command is refused and the lift does not move
    sref.teleop = (0.1, 0.0)
    assert sref.wait(lambda: sref.s()['permit']['reason'] == 'BASE_MOVING', 5.0)
    p0 = sref.lift_pos()
    sref.lift_pub.publish(Int8(data=1))
    assert sref.wait(lambda: sref.get('/sil/lift/emulator_state')['cmd_result'] == REFUSED, 2.0)
    time.sleep(0.5)
    assert abs(sref.lift_pos() - p0) < 0.001
    # base stopped for the hold time: permit, then the move starts
    sref.teleop = None
    assert sref.wait(lambda: sref.s()['permit']['permit'], 5.0), sref.s()['permit']
    sref.lift_pub.publish(Int8(data=1))
    assert sref.wait(lambda: sref.get('/sil/lift/emulator_state')['state'] == MOVING, 2.0)
    assert sref.wait(lambda: sref.lift_pos() > p0 + 0.005, 3.0)
    stops0 = sref.get('/sil/lift/emulator_state')['permit_stops']
    # the producer stalls in the middle of the move: no permit frame arrives
    proc = pid_of('simref_process', exact=True)
    t0 = time.time()
    os.kill(proc, signal.SIGSTOP)
    try:
        assert sref.wait(lambda: not sref.get('/sil/lift/emulator_state')['drive_on'], 1.0, 0.005)
        t_stop = time.time() - t0
    finally:
        time.sleep(max(0.0, 0.3 - (time.time() - t0)))
        os.kill(proc, signal.SIGCONT)
    assert t_stop <= PERMIT_EXPIRY_S + 0.15, t_stop
    assert sref.get('/sil/lift/emulator_state')['permit_stops'] == stops0 + 1
    p_stop = sref.lift_pos()
    time.sleep(1.5)                               # permit frames resume: no replay of the move
    assert abs(sref.lift_pos() - p_stop) < 0.002
    assert not sref.get('/sil/lift/emulator_state')['drive_on']


# ---- 8. two faults in one tick ------------------------------------------------------------------
def test_08_two_faults_same_tick_follow_the_priority_order(sref):
    assert sref.arm()
    n0 = sref.last_reason_n()
    # both inputs change in one datagram and pass the same debounce in the same step
    sref.pins(charger_fault=1, estop_contact=1)
    assert sref.wait(lambda: sref.s()['state'] == 'LOCKED', 1.0, 0.005)
    time.sleep(0.2)
    new = sref.reasons_after(n0)
    pair = [r for r in new if r['reason'] in ('CHARGER_FAULT', 'ESTOP_DIAG')]
    assert [r['reason'] for r in pair[:2]] == ['CHARGER_FAULT', 'ESTOP_DIAG'], new
    assert pair[0]['t_us'] == pair[1]['t_us'], 'not the same tick: %s' % pair
    # priority: wcm_fault (W2, recorded in RUN) before estop_diag (E1 starts the lock episode);
    # the other order would give reason_mask [ESTOP_DIAG, CHARGER_FAULT]
    s = sref.s()
    assert s['reason_mask'] == ['ESTOP_DIAG'] and s['first_reason'] == 'ESTOP_DIAG', s
    sref.pins(charger_fault=0, estop_contact=0)
    time.sleep(0.3)


# ---- 9. power restored -----------------------------------------------------------------------
def test_09_power_restored_after_a_cut_no_automatic_reenable(sref):
    assert sref.arm()
    sref.teleop = (0.1, 0.0)
    assert sref.wait(lambda: sref.gz_v() > 0.08, 8.0)
    sref.pins(y32=0, contactor_fb=0)              # power cut: chain and contactor drop
    proc = pid_of('simref_process', exact=True)
    os.kill(proc, signal.SIGKILL)                 # controller loses power
    sref.drive_inject.publish(String(data='power-cycle'))
    sref.lift_inject.publish(String(data='power-cycle'))
    time.sleep(0.5)
    t_restore = time.time()
    sref.pins(y32=1, contactor_fb=1)              # power restored
    assert sref.wait(lambda: pid_of('simref_process', exact=True) not in (None, proc), 10.0)
    assert sref.wait(lambda: sref.s()['uptime_ms'] < 3000 and sref.s()['boot_cause'] == 'POWER_ON', 10.0)
    time.sleep(3.0)                               # /cmd_vel keeps asking for 0.1 m/s
    s = sref.s()
    assert s['state'] == 'LOCKED' and s['drive']['phase'] == 'IDLE', s['drive']
    assert s['first_reason'] == 'BOOT_POWER_ON'
    assert 0x0F not in [v for _, v in sdo_writes(sref.taps['vcan_drive'].since(t_restore), 0x6040)]
    assert not sref.get('/sil/drive/emulator_state')['enabled']
    assert abs(sref.gz_v()) < 0.01
    sref.teleop = None
    assert sref.arm(8.0)


# ---- 10. reference process restart ----------------------------------------------------------
def test_10_restart_of_the_reference_leaves_the_drive_disabled(sref):
    assert sref.arm()
    sref.teleop = (0.1, 0.0)
    assert sref.wait(lambda: sref.gz_v() > 0.08, 8.0)
    assert sref.get('/sil/drive/emulator_state')['enabled']
    proc = pid_of('simref_process', exact=True)
    t_kill = time.time()
    os.kill(proc, signal.SIGKILL)                 # the drive stays powered and enabled
    assert sref.wait(lambda: pid_of('simref_process', exact=True) not in (None, proc), 10.0)
    assert sref.wait(lambda: not sref.get('/sil/drive/emulator_state')['enabled'], 5.0)
    time.sleep(2.0)
    s = sref.s()
    assert s['state'] == 'LOCKED' and s['drive']['phase'] == 'IDLE'
    words = [v for _, v in sdo_writes(sref.taps['vcan_drive'].since(t_kill), 0x6040)]
    assert 0x06 in words and 0x0F not in words, words
    assert abs(sref.gz_v()) < 0.01
    t_arm = time.time()
    assert sref.arm(8.0)
    assert 0x0F in [v for _, v in sdo_writes(sref.taps['vcan_drive'].since(t_arm), 0x6040)]
    assert sref.wait(lambda: sref.gz_v() > 0.05, 5.0)
    sref.teleop = None
    assert sref.wait(lambda: abs(sref.gz_v()) < 0.01, 5.0)
