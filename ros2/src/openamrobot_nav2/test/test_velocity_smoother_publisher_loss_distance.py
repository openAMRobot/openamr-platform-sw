"""
Real-velocity_smoother publisher-loss distance bound (platform-sw#90, Alex's review).

velocity_timeout: 1.0 (the previous shipped value) let a stopped publisher
keep driving the robot for up to a full second after going silent - too long
for a functional stop. This test runs the REAL nav2_velocity_smoother
executable against the shipped config/nav2_params.yaml, drives it to steady
max velocity, stops publishing entirely (the actual failure this test
exists for - a dead teleop, a crashed gate node, a dropped cmd_vel_gated
publisher), and measures how much commanded distance it keeps emitting
before settling at zero.

Confirmed against the real nav2_velocity_smoother source
(ros-navigation/navigation2, nav2_velocity_smoother/src/velocity_smoother.cpp,
smootherTimer()) before writing this, not assumed: on a velocity_timeout
expiry, command_.twist is reset to a Twist() *target*, which then goes
through the SAME smoothing/clamping step as any other command - i.e. the
robot decelerates to zero at max_decel, it does not snap to zero
instantaneously. So the worst-case commanded distance after a lost
publisher is bounded by two phases:

  1. up to velocity_timeout seconds still commanding the last steady
     velocity (nothing has told the smoother anything is wrong yet);
  2. a decel ramp from that velocity to zero at max_decel.

expected_bound_m (below) computes this directly from the shipped config
(max_velocity[0], max_decel[0], velocity_timeout) rather than a hardcoded
number, so a future config change moves the bound with it instead of
silently testing a stale figure - same convention as
test_twist_mux_startup_and_restart.py's _load_shipped_lock_timeout().

Message type: plain geometry_msgs/Twist on both cmd_vel and cmd_vel_smoothed,
not TwistStamped - checked against the actual installed ROS 2 Jazzy source
(ros-navigation/navigation2, jazzy branch, nav2_util/include/nav2_util/
twist_subscriber.hpp and twist_publisher.hpp), not assumed from a newer
branch: nav2_util::TwistSubscriber/TwistPublisher support either message
type behind the "enable_stamped_cmd_vel" parameter, which defaults to false
on Jazzy. This repo's own config never sets it, so the real node subscribes
to/publishes plain Twist. An earlier draft of this test used TwistStamped
(correct on a newer nav2_util, where that default flipped to true) and the
publisher/subscriber never matched at all - zero messages, not a wrong
value - which is exactly the kind of mismatch that only shows up running
against the real installed package, not by reading the wrong branch.

This is a network-level measurement of the smoother's OUTPUT Twist stream
(received-time integration, the same style test_twist_mux_startup_and_
restart.py uses for its own assertions), not a physical distance measured
from real odometry/Gazebo - there is no robot or simulator in this test.
_MARGIN exists because receipt timestamps carry process/DDS scheduling
jitter this bench has no way to remove; it is a sanity margin on an upper
bound, not a tight physical tolerance.

Needs a full ROS 2 Jazzy workspace with the real `nav2_velocity_smoother`
package installed and `ros2`/`ros2 lifecycle` on PATH - skipped everywhere
else, same convention as the rest of this test package. Run for real with:

    colcon build --packages-select openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v -s \
        src/openamrobot_nav2/test/test_velocity_smoother_publisher_loss_distance.py

New, unreviewed test infrastructure (subprocess-managed, not launch_testing),
same caveat as test_twist_mux_startup_and_restart.py - if it is flaky in your
environment, that is useful information on its own, not just noise to retry
past.
"""
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402,I100

# The actual shipped config, not a hand-maintained copy. Package root is this
# test file's grandparent (test/ -> openamrobot_nav2/).
_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'

_NODE_NAME = 'velocity_smoother'
_IN_TOPIC = 'vsmoother_test_in'
_OUT_TOPIC = 'vsmoother_test_out'
_ODOM_TOPIC = 'vsmoother_test_odom'  # unused (feedback: OPEN_LOOP) but remapped off the real odom

# Sanity margin on the computed worst-case bound (see module docstring) - this
# bench measures over the network via receipt timestamps, not a physical
# distance, so it needs slack for process/DDS scheduling jitter. Kept
# generous on purpose: this is a bound check, not a precision measurement.
_MARGIN = 1.5


def _velocity_smoother_available():
    """Report whether the ros2 CLI AND the real nav2_velocity_smoother package are present."""
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'nav2_velocity_smoother'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


pytestmark = pytest.mark.skipif(
    not _velocity_smoother_available(),
    reason='needs the real ros2 CLI AND the nav2_velocity_smoother package installed')


def _load_shipped_smoother_params():
    with open(_SHIPPED_PARAMS_PATH, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    return params['velocity_smoother']['ros__parameters']


def _start_smoother(params_path):
    """Launch the real nav2_velocity_smoother executable as a subprocess."""
    return subprocess.Popen(
        ['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother',
         '--ros-args', '--params-file', str(params_path),
         '-r', f'cmd_vel:={_IN_TOPIC}',
         '-r', f'cmd_vel_smoothed:={_OUT_TOPIC}',
         '-r', f'odom:={_ODOM_TOPIC}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop_smoother(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def _lifecycle_transition(label, timeout_s=10.0):
    """ros2 lifecycle set <node> <label>, polling until it succeeds or the deadline passes.

    velocity_smoother is a nav2 LifecycleNode - it publishes nothing at all
    until it has been configured and activated, normally done by
    lifecycle_manager. There is no lifecycle_manager here (that is the
    point - an isolated bench test of one node), so this does the same two
    transitions directly via the CLI, retrying while the node is still
    coming up (its lifecycle service is not available yet).
    """
    deadline = time.monotonic() + timeout_s
    last_result = None
    while time.monotonic() < deadline:
        last_result = subprocess.run(
            ['ros2', 'lifecycle', 'set', f'/{_NODE_NAME}', label],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0)
        if last_result.returncode == 0:
            return
        time.sleep(0.3)
    stderr = last_result.stderr.decode(errors='replace') if last_result else '(never ran)'
    raise AssertionError(
        f'ros2 lifecycle set /{_NODE_NAME} {label} never succeeded within '
        f'{timeout_s}s - last stderr: {stderr}')


class TestPublisherLossDistance(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        # Fail loudly and specifically if the shipped config ever drifts back
        # to the slow default, instead of silently testing stale data (same
        # convention as test_twist_mux_startup_and_restart.py).
        params = _load_shipped_smoother_params()
        assert params['velocity_timeout'] <= 0.3, (
            'config/nav2_params.yaml velocity_smoother.velocity_timeout is %r '
            '- this must be <= 0.3 (platform-sw#90, Alex\'s review) or a lost '
            'publisher can keep commanding motion for too long; fix the '
            'shipped config, do not loosen this assertion' % params['velocity_timeout'])
        cls._params = params

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = rclpy.create_node('vsmoother_loss_probe')
        self._out_pub = self.probe.create_publisher(Twist, _IN_TOPIC, 10)
        self._received = []  # (monotonic_receipt_time, linear_x) tuples
        self.probe.create_subscription(
            Twist, _OUT_TOPIC,
            lambda msg: self._received.append((time.monotonic(), msg.linear.x)), 10)
        self._proc = _start_smoother(_SHIPPED_PARAMS_PATH)

    def tearDown(self):
        _stop_smoother(self._proc)
        self.probe.destroy_node()

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def test_commanded_distance_after_a_lost_publisher_stays_bounded(self):
        max_vx = self._params['max_velocity'][0]
        decel_x = abs(self._params['max_decel'][0])
        timeout_s = self._params['velocity_timeout']
        smoothing_hz = self._params['smoothing_frequency']

        _lifecycle_transition('configure')
        _lifecycle_transition('activate')
        self.assertIsNone(
            self._proc.poll(),
            f'velocity_smoother exited early (code {self._proc.poll()}) during startup')

        # Drive to steady max velocity: ramp time is max_vx / max_accel, plus
        # margin, at smoothing_hz - comfortably covered by publishing for 1s.
        steady_cmd = Twist()
        steady_cmd.linear.x = max_vx
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._out_pub.publish(steady_cmd)
            self._spin_for(1.0 / smoothing_hz)
        self.assertTrue(self._received, 'velocity_smoother never produced any output')
        self.assertAlmostEqual(
            self._received[-1][1], max_vx, delta=0.02,
            msg=f'expected steady output near max_velocity[0]={max_vx} before '
            f'cutting the publisher, got {self._received[-1][1]} - the smoother '
            f'may not have had long enough to ramp up, or the sanity check '
            f'itself needs a longer drive window')

        # The actual event under test: the publisher goes silent, with no
        # further message of any kind on _IN_TOPIC. Keep recording _OUT_TOPIC
        # output for a window generous enough to cover the timeout plus a
        # full decel ramp from max velocity, with slack.
        self._received.clear()
        loss_start = time.monotonic()
        settle_window = timeout_s + (max_vx / decel_x) + 1.0
        self._spin_for(settle_window)

        self.assertTrue(
            self._received,
            'expected at least one output message after the publisher stopped '
            '(the timeout zero-target still goes through the normal smoothing '
            'step, which publishes) - got none at all')

        # The LAST message received must be zero - not "the last few", one.
        # Confirmed against the real source (smootherTimer(), same jazzy
        # branch checked in the module docstring): once the ramped command
        # reaches exactly Twist() while still timed out, the NEXT tick sets
        # stopped_ = True and returns before publishing anything at all - the
        # node stops emitting entirely once settled, it does not keep
        # publishing a steady stream of zeros. So the message stream's own
        # last entry is the proof of settling; there is no sustained
        # trailing window of zero messages to sample from (an earlier
        # version of this assertion required the last 5 messages to all be
        # zero and failed on real hardware for exactly this reason - the
        # ramp's own last one or two non-zero steps were still inside that
        # window).
        self.assertLess(
            abs(self._received[-1][1]), 1e-6,
            f'the last output message was not zero ({self._received[-1][1]}) by the '
            f'end of the {settle_window:.2f}s settle window - last few samples: '
            f'{self._received[-5:]}')

        # Integrate |linear.x| over receipt-time gaps from the moment the
        # publisher went silent - trapezoidal rule, network-level measurement
        # (see module docstring), not a physical odometry integration.
        samples = [(loss_start, max_vx)] + self._received
        distance_m = 0.0
        for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
            distance_m += 0.5 * (abs(v0) + abs(v1)) * (t1 - t0)

        expected_bound_m = max_vx * timeout_s + 0.5 * max_vx * (max_vx / decel_x)
        self.assertLess(
            distance_m, expected_bound_m * _MARGIN,
            f'commanded distance after the publisher went silent was '
            f'{distance_m:.4f} m, expected at most '
            f'{expected_bound_m:.4f} m * {_MARGIN} margin = '
            f'{expected_bound_m * _MARGIN:.4f} m (max_velocity[0]={max_vx}, '
            f'velocity_timeout={timeout_s}, max_decel[0]={-decel_x}) - a lost '
            f'publisher is driving the robot further than the configured '
            f'limits should allow (platform-sw#90)')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v', '-s']))
