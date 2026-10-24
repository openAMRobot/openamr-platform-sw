"""Real-mux startup/restart test for the #35 command-arbitration lock.

Platform-sw#90, Mohamed Sayed's review. Every other test in this package
exercises safety_stop_gate_node.py in
isolation - correct, but none of them run an actual twist_mux process, so
none of them could ever have caught the gap Mohamed found: twist_mux's own
LockTopicHandle subscribes with rclcpp::SystemDefaultsQoS() (volatile), so
it never receives this package's TRANSIENT_LOCAL retained lock sample, and
with config/twist_mux.yaml's old `timeout: 0.0` a fresh or restarted mux
defaulted to UNLOCKED (fail-open) until a real lock message happened to
land. The fix is the timeout flip to 0.3s (see twist_mux.yaml's comment)
plus the launch-level startup stagger (navigation_launch.py); this test is
what actually proves the fix against the real mux binary, in both startup
orders and across a restart, by recording its first real downstream
output - not by asserting anything about this package's own nodes.

Needs a full ROS 2 Jazzy workspace with the real `twist_mux` package
installed and `ros2 run` on PATH - skipped everywhere else, same
convention as the rest of this test package. Run for real with:

    colcon build --packages-select openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v -s \
        src/openamrobot_nav2/test/test_twist_mux_startup_and_restart.py

This is new, unreviewed test infrastructure (subprocess-managed, not
launch_testing) - if it is flaky in your environment, that is useful
information on its own, not just noise to retry past.
"""
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')

pytestmark = pytest.mark.skipif(
    shutil.which('ros2') is None,
    reason='needs the real ros2 CLI + twist_mux package on PATH')

from geometry_msgs.msg import Twist  # noqa: E402,I100
from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode  # noqa: E402

# Minimal standalone copy of config/twist_mux.yaml's parameters - kept here
# rather than loading that file directly so this test does not silently
# start passing again if someone reverts the timeout fix there without
# noticing this test exists. If the two drift apart in any other way,
# that is a bug in THIS file, not an excuse to skip checking the real
# shipped config too (a follow-up could load twist_mux.yaml directly and
# assert the two agree - not done here to keep this test self-contained).
_TWIST_MUX_PARAMS = """\
twist_mux:
  ros__parameters:
    use_stamped: false
    topics:
      navigation:
        topic: cmd_vel_nav
        timeout: 0.5
        priority: 10
      software_stop:
        topic: cmd_vel_stop
        timeout: 0.5
        priority: 255
    locks:
      software_stop:
        topic: /safety/stop_lock
        timeout: 0.3
        priority: 255
"""


def _start_mux(params_path):
    """Launch the real twist_mux executable as a subprocess."""
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux',
         '--ros-args', '--params-file', params_path,
         '-r', 'cmd_vel_out:=cmd_vel_mux'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop_mux(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


class TwistMuxLiveTestCase(unittest.TestCase):
    """Shared plumbing for the real-mux tests below.

    A real safety_stop_gate_node plus a real, subprocess-managed twist_mux,
    with probes on the mux's actual output.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls._tmpdir = tempfile.mkdtemp(prefix='twist_mux_live_test_')
        cls._params_path = os.path.join(cls._tmpdir, 'twist_mux.yaml')
        with open(cls._params_path, 'w') as f:
            f.write(_TWIST_MUX_PARAMS)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls._tmpdir, ignore_errors=True)
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode()  # starts locked - see its docstring
        self.probe = rclpy.create_node('twist_mux_live_probe')
        self._nav_pub = self.probe.create_publisher(Twist, 'cmd_vel_nav', 10)
        self._mux_out = []
        self.probe.create_subscription(
            Twist, 'cmd_vel_mux', self._mux_out.append, 10)
        self._mux_proc = None
        self._nonzero_twist = Twist()
        self._nonzero_twist.linear.x = 0.3

    def tearDown(self):
        if self._mux_proc is not None:
            _stop_mux(self._mux_proc)
        self.gate.destroy_node()
        self.probe.destroy_node()

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _publish_nonzero_nav_cmd(self, seconds):
        """Keep a nonzero, lower-priority ('navigation') command flowing.

        Runs for the given duration, interleaved with spinning both nodes.
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._nav_pub.publish(self._nonzero_twist)
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _assert_nothing_nonzero_forwarded(self):
        nonzero = [m for m in self._mux_out if m != Twist()]
        self.assertEqual(
            nonzero, [],
            f'twist_mux forwarded {len(nonzero)} nonzero command(s) while '
            f'the software_stop lock should have been engaged: {nonzero[:3]}')


class TestMuxStartsAfterGate(TwistMuxLiveTestCase):

    def test_mux_starting_after_the_gate_stays_locked(self):
        # Gate first (already locked, already publishing/ticking), THEN the
        # mux - the order this package's docstring always assumed was safe.
        self._spin_for(0.3)  # let the gate get a few ticks out before the mux exists
        self._mux_proc = _start_mux(self._params_path)
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_nothing_nonzero_forwarded()


class TestMuxStartsBeforeGate(TwistMuxLiveTestCase):

    def test_mux_starting_before_the_gate_stays_locked(self):
        # The order Mohamed's review specifically called out: a delayed
        # helper (or a mux that just happens to come up first) must not
        # give nonzero commands a free window before the gate exists.
        self._mux_proc = _start_mux(self._params_path)
        time.sleep(0.5)  # mux fully up, gate does not exist yet
        self._publish_nonzero_nav_cmd(seconds=0.5)  # window before the gate exists at all
        # Now bring the gate up and keep publishing through its startup.
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_nothing_nonzero_forwarded()


class TestMuxRestart(TwistMuxLiveTestCase):

    def test_mux_restart_with_lock_already_engaged_stays_locked(self):
        # Steady state first: mux up, gate up, confirm it is actually
        # forwarding the nav command's lower-priority twin once nothing is
        # locking it - prerequisite check, not the thing under test.
        self.gate.release_lock()
        self._mux_proc = _start_mux(self._params_path)
        self._publish_nonzero_nav_cmd(seconds=1.0)
        self.assertTrue(
            any(m != Twist() for m in self._mux_out),
            'sanity check failed: mux never forwarded the nav command while '
            'unlocked, so this test cannot tell a real pass from a mux that '
            'is not actually running')
        # Re-engage the lock on the OLD mux first, and give it a real
        # settle window before touching anything else: engage_lock() only
        # publishes - the OLD mux (a separate process, on its own executor)
        # still needs actual wall-clock time to receive and act on that
        # message. Without this, nav keeps blasting nonzero commands and
        # the OLD process can legitimately still be forwarding them for a
        # few iterations right up until it is killed, which has nothing to
        # do with the NEW mux's startup behavior this test exists to check
        # - that was exactly the false failure seen here the first time
        # this test was run (leaked commands attributed to a race in the
        # test itself, not to the fix).
        self.gate.engage_lock()
        settle_end = time.monotonic() + 0.5
        while time.monotonic() < settle_end:
            self._nav_pub.publish(self._nonzero_twist)
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)
        recent = self._mux_out[-5:] if len(self._mux_out) >= 5 else self._mux_out
        self.assertTrue(
            all(m == Twist() for m in recent),
            'the OLD mux had not actually re-locked by the end of the '
            'settle window - re-check engage_lock() delivery before '
            'blaming the restart path below')

        # NOW kill and restart the mux - this is the case Mohamed's review
        # named explicitly: the retained sample "lives with this node, not
        # the mux," so nothing about THIS node being long-running helps a
        # brand new mux process that has never received a single message
        # yet.
        self._mux_out.clear()
        _stop_mux(self._mux_proc)
        self._mux_proc = _start_mux(self._params_path)
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_nothing_nonzero_forwarded()
