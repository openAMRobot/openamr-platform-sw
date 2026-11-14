"""End-to-end run of sil.launch.py: vcan, emulators, host test client and Gazebo.

Needs Linux vcan and Gazebo Harmonic. When one is missing the tests FAIL with a line starting
"BLOCKED:". Development evidence only: emulators and simulation, no firmware, no hardware.
"""
import json
import os
import shutil
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, Int8, String

LOG = os.environ.get('SIL_LAUNCH_LOG', '/tmp/sil_launch.log')
LINK_LOSS_S = 0.250       # config/sil.yaml drive.link_loss_ms
REFUSED = 0x80            # config/sil.yaml lift.permit.refused_result
IDLE = 3                  # CTL-017 state IDLE


class Probe(Node):
    def __init__(self):
        super().__init__('sil_test_probe')
        self.last = {}
        self.stamp = {}
        self.cmd = None
        for topic in ('/sil/drive/emulator_state', '/sil/lift/emulator_state',
                      '/sil/drive/status', '/sil/lift/status', '/sil/bms/status'):
            self.create_subscription(String, topic, lambda m, t=topic: self.put(
                t, json.loads(m.data)), 10)
        self.create_subscription(Odometry, '/sil/gz/odom', lambda m: self.put('gz_odom', m), 10)
        self.create_subscription(Odometry, '/sil/base/odom', lambda m: self.put('odom', m), 10)
        self.create_subscription(JointState, '/sil/gz/joint_states',
                                 lambda m: self.put('joints', m), 10)
        self.create_subscription(BatteryState, '/sil/bms/battery_state',
                                 lambda m: self.put('battery', m), 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.silent_pub = self.create_publisher(Bool, '/sil/stm32/can1_silent', 10)
        self.lift_pub = self.create_publisher(Int8, '/sil/lift/command', 10)
        self.permit_pub = self.create_publisher(Bool, '/sil/lift/permit', 10)
        self.create_timer(0.05, self.send_cmd)

    def put(self, key, value):
        self.last[key] = value
        self.stamp[key] = time.monotonic()

    def send_cmd(self):
        if self.cmd is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.cmd
            self.cmd_pub.publish(msg)

    def get(self, key):
        return self.last.get(key)

    def lift_position(self):
        joints = self.get('joints')
        return joints.position[list(joints.name).index('lift_joint')]

    def wait(self, cond, timeout, step=0.01):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return True
            except (KeyError, TypeError, ValueError, AttributeError):
                pass
            time.sleep(step)
        return False


def tail_log(lines=60):
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
    with open(LOG, 'w', encoding='utf-8') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'openamrobot_sil', 'sil.launch.py'],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        ready = probe.wait(lambda: (
            probe.get('gz_odom') is not None
            and probe.get('/sil/drive/status')['configured']
            and probe.get('/sil/drive/emulator_state')['feedback'] == 'gazebo'
            and probe.get('/sil/lift/emulator_state')['plant'] == 'gazebo'
            and probe.get('/sil/lift/status')['state'] == IDLE
            and probe.get('odom') is not None), timeout=180.0, step=0.2)
        if not ready:
            blocked = 'BLOCKED' in tail_log(400)
            pytest.fail('%s: SIL stack not ready after 180 s. Seen: %s\n%s' % (
                'BLOCKED' if blocked else 'FAILED', sorted(probe.last), tail_log()),
                pytrace=False)
        time.sleep(2.0)  # let the robot settle on the floor
        yield probe
    finally:
        probe.cmd = None
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-f', 'gz sim'], check=False)
        executor.shutdown()
        rclpy.shutdown()


def gz_x(probe):
    return probe.get('gz_odom').pose.pose.position.x


def test_bms_values_arrive_over_can2(sil):
    assert sil.wait(lambda: sil.get('battery') is not None, 5.0)
    battery = sil.get('battery')
    assert battery.voltage == pytest.approx(25.6, abs=0.05)
    assert battery.percentage == pytest.approx(0.80, abs=0.001)
    # The client may poll before the emulator has opened its socket at start-up. Once both
    # run, every poll is answered: over 2 s (10 poll cycles at 5 Hz) no new timeout appears.
    before = sil.get('/sil/bms/status')
    time.sleep(2.0)
    after = sil.get('/sil/bms/status')
    assert after['timeouts'] == before['timeouts']
    assert after['answers'] >= before['answers'] + 20
    assert after['age_s'] < 0.5


def test_0_2_m_per_s_command_moves_the_gazebo_base_and_odometry_returns(sil):
    x0, odom0 = gz_x(sil), sil.get('odom').pose.pose.position.x
    assert abs(sil.get('gz_odom').twist.twist.linear.x) < 0.02
    sil.cmd = (0.2, 0.0)
    try:
        # CAN1: the client wrote 19 r/min per wheel (0.2 m/s on a 0.1 m wheel, whole r/min)
        assert sil.wait(lambda: sil.get('/sil/drive/emulator_state')['target_rpm'] == [19, 19], 3.0)
        time.sleep(5.0)
        state = sil.get('/sil/drive/emulator_state')
        gz_odom, odom = sil.get('gz_odom'), sil.get('odom')
    finally:
        sil.cmd = (0.0, 0.0)
    moved = gz_odom.pose.pose.position.x - x0
    assert 0.6 < moved < 1.2, 'Gazebo base moved %.3f m in about 5 s' % moved
    assert abs(gz_odom.pose.pose.position.y) < 0.15
    assert gz_odom.twist.twist.linear.x == pytest.approx(0.199, abs=0.02)
    # feedback path: Gazebo wheel speed -> 606Ch -> CAN1 -> host client odometry
    assert state['feedback'] == 'gazebo'
    assert state['actual_rpm'][0] == pytest.approx(19.0, abs=1.5)
    assert state['actual_rpm'][1] == pytest.approx(19.0, abs=1.5)
    assert odom.twist.twist.linear.x == pytest.approx(0.199, abs=0.02)
    returned = odom.pose.pose.position.x - odom0
    assert returned == pytest.approx(moved, rel=0.2), (returned, moved)
    assert sil.wait(lambda: abs(sil.get('gz_odom').twist.twist.linear.x) < 0.01, 3.0)


def test_link_loss_stops_the_emulated_drive_within_the_configured_time(sil):
    sil.cmd = (0.2, 0.0)
    try:
        assert sil.wait(lambda: sil.get('gz_odom').twist.twist.linear.x > 0.18, 5.0)
        events = sil.get('/sil/drive/emulator_state')['link_loss_events']
        t0 = time.monotonic()
        sil.silent_pub.publish(Bool(data=True))      # the STM32 role goes silent on CAN1
        assert sil.wait(lambda: sil.get('/sil/drive/emulator_state')['link_lost'], 2.0, 0.002)
        detected = time.monotonic() - t0
        assert sil.wait(
            lambda: sil.get('/sil/drive/emulator_state')['demand_rpm'] == [0.0, 0.0], 2.0, 0.002)
        stopped = time.monotonic() - t0
        # Not before the 2000h time. After it: margin for the 20 Hz state topic, DDS and threads.
        assert LINK_LOSS_S - 0.08 <= detected <= LINK_LOSS_S + 0.25, detected
        assert stopped <= LINK_LOSS_S + 0.35, stopped
        assert sil.get('/sil/drive/emulator_state')['link_loss_events'] == events + 1
        # /cmd_vel still asks for 0.2 m/s. The Gazebo base stops and stays stopped.
        assert sil.wait(lambda: abs(sil.get('gz_odom').twist.twist.linear.x) < 0.01, 3.0)
        x = gz_x(sil)
        time.sleep(1.0)
        assert abs(gz_x(sil) - x) < 0.01
    finally:
        sil.cmd = (0.0, 0.0)
        time.sleep(0.3)
        sil.silent_pub.publish(Bool(data=False))
    assert sil.wait(lambda: sil.get('/sil/drive/status')['configured'], 3.0)


def test_lift_moves_only_with_a_fresh_permit(sil):
    lift = '/sil/lift/emulator_state'
    assert abs(sil.lift_position()) < 0.003
    assert sil.get(lift)['permit_gate'] and not sil.get(lift)['permit_fresh']

    # 1. No permit: the command is refused and the Gazebo lift does not move.
    sil.lift_pub.publish(Int8(data=1))
    assert sil.wait(lambda: sil.get('/sil/lift/status')['cmd_result'] == REFUSED, 2.0)
    time.sleep(2.0)
    assert abs(sil.lift_position()) < 0.003 and not sil.get(lift)['drive_on']

    # 2. Permit granted, but no new command: nothing is replayed.
    sil.permit_pub.publish(Bool(data=True))
    assert sil.wait(lambda: sil.get(lift)['permit_fresh'], 2.0)
    time.sleep(1.0)
    assert abs(sil.lift_position()) < 0.003 and not sil.get(lift)['drive_on']

    # 3. Fresh permit and a new command: the lift travels to L1175 (joint 0.175 m).
    sil.lift_pub.publish(Int8(data=1))
    assert sil.wait(lambda: sil.lift_position() > 0.02, 5.0)
    assert sil.wait(lambda: sil.get('/sil/lift/status')['active_stop'] == 1, 25.0)
    assert sil.lift_position() == pytest.approx(0.175, abs=0.004)
    assert sil.get(lift)['faults'] == 0

    # 4. Permit withdrawn on the way to L1350: the lift halts and drops the target.
    sil.lift_pub.publish(Int8(data=2))
    assert sil.wait(lambda: sil.lift_position() > 0.21, 10.0)
    sil.permit_pub.publish(Bool(data=False))
    assert sil.wait(lambda: not sil.get(lift)['drive_on'], 1.0)
    time.sleep(1.0)
    halted = sil.lift_position()
    assert 0.21 < halted < 0.30
    assert sil.get(lift)['permit_stops'] == 1 and sil.get(lift)['target_stop'] == 0xFF

    # 5. Permit back: no motion without a new command.
    sil.permit_pub.publish(Bool(data=True))
    assert sil.wait(lambda: sil.get(lift)['permit_fresh'], 2.0)
    time.sleep(1.5)
    assert sil.lift_position() == pytest.approx(halted, abs=0.002)
    assert sil.get(lift)['faults'] == 0
