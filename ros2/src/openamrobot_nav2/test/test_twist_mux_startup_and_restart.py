"""
Real-mux startup/restart test for the #35 command-arbitration lock.

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

Second review round (Mohamed Sayed, platform-sw#90), four findings, all
confirmed against the code and fixed here:

1. [Mux-before-gate ordering] The previous version constructed
   SafetyStopGateNode in the shared setUp() for every test class,
   including the one whose whole point is "no gate exists yet." Since
   SafetyStopGateNode.__init__ publishes the lock on its last line (see
   that file's docstring), the gate's retained sample already existed the
   instant setUp() ran - before _start_mux() was ever called. The
   "gate does not exist yet" comment on that test was simply false
   against the code. Fixed by NOT constructing the gate in setUp() for
   TestMuxStartsBeforeGate; it is constructed inside the test method,
   after the gate-less interval has actually elapsed.
2. [Standalone config copy] The previous version wrote its own
   hand-maintained YAML literal to a scratch file rather than loading
   config/twist_mux.yaml, so reverting the shipped config's timeout fix
   would not move this test at all - it tested a permanently-correct
   copy, forever. Fixed by loading the shipped file directly, with an
   explicit assertion on the loaded lock timeout before using it: that
   assertion is what makes a revert of the shipped value fail loudly,
   in this file, rather than silently testing stale data.
3. [No positive control] TestMuxStartsAfterGate and
   TestMuxStartsBeforeGate only asserted that nothing nonzero was
   forwarded - true vacuously if the mux subprocess failed to launch at
   all. Fixed by checking the mux process is still alive (poll() is
   None) and that its node is actually discoverable over the graph
   before trusting the "nothing nonzero" result - without touching the
   lock state itself, so the "stays locked" scenario under test is not
   disturbed.
4. [CI gap] This file was never in nav-status-tests.yml's explicit
   pytest list, and that workflow never installed ros-jazzy-twist-mux -
   so even added to the list, every case here would have errorred
   against a missing package rather than skipping cleanly, since the
   skip guard below only checked for the `ros2` CLI, not the `twist_mux`
   package specifically. Both the workflow and the guard are fixed
   (workflow in the same patch as this file; guard below).
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


def _twist_mux_available():
    """
    Report whether both the ros2 CLI and the real twist_mux package are present.

    shutil.which('ros2') alone is not enough (finding 4, Mohamed's second
    review): a container can have the CLI without ros-jazzy-twist-mux
    installed, in which case `ros2 run twist_mux twist_mux` errors instead
    of skipping cleanly.
    """
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'twist_mux'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


pytestmark = pytest.mark.skipif(
    not _twist_mux_available(),
    reason='needs the real ros2 CLI AND the twist_mux package installed')

from geometry_msgs.msg import Twist  # noqa: E402,I100
from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode  # noqa: E402

# The actual shipped config, not a hand-maintained copy (finding 2). Package
# root is this test file's grandparent (test/ -> openamrobot_nav2/).
_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'


def _load_shipped_lock_timeout():
    with open(_SHIPPED_PARAMS_PATH, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    return params['twist_mux']['ros__parameters']['locks']['software_stop']['timeout']


def _start_mux(params_path):
    """Launch the real twist_mux executable as a subprocess."""
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux',
         '--ros-args', '--params-file', str(params_path),
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
    """
    Shared plumbing for the real-mux tests below.

    A real, subprocess-managed twist_mux, with probes on its actual
    output. The gate is NOT created here by default (see
    _make_gate_if_default) - TestMuxStartsBeforeGate overrides that to
    control exactly when the gate (and its retained publish) comes into
    existence, which is the whole point of that test (finding 1).
    """

    # Overridden by TestMuxStartsBeforeGate to False.
    _create_gate_in_setup = True

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        # Fail loudly and specifically if the shipped config ever drifts
        # back to the fail-open default - this is what ties this test to
        # the real file instead of a copy that can silently diverge
        # (finding 2).
        shipped_timeout = _load_shipped_lock_timeout()
        assert shipped_timeout > 0.0, (
            'config/twist_mux.yaml locks.software_stop.timeout is %r - this '
            'must be > 0.0 (platform-sw#90 P1) or twist_mux defaults to '
            'UNLOCKED on a fresh/restarted process; fix the shipped config, '
            'do not change this assertion' % shipped_timeout)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode() if self._create_gate_in_setup else None
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
        if self.gate is not None:
            self.gate.destroy_node()
        self.probe.destroy_node()

    def _spin_for(self, seconds, spin_gate=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if spin_gate and self.gate is not None:
                rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _publish_nonzero_nav_cmd(self, seconds):
        """
        Keep a nonzero, lower-priority ('navigation') command flowing.

        Runs for the given duration, interleaved with spinning both nodes
        (the gate only if it exists yet - see finding 1).
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._nav_pub.publish(self._nonzero_twist)
            if self.gate is not None:
                rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _assert_mux_alive_and_discoverable(self):
        """
        Positive control (finding 3, Mohamed's second review of #90).

        A crashed or never-started mux must not let the "nothing nonzero
        forwarded" assertion pass vacuously. Checks the subprocess itself
        and that it actually got far enough into rclcpp init to be
        discoverable - neither depends on the lock state, so it is safe to
        call from a "stays locked" test without disturbing what that test
        is actually proving.
        """
        self.assertIsNotNone(self._mux_proc, 'no mux process was ever started')
        self.assertIsNone(
            self._mux_proc.poll(),
            f'the mux process exited early (code {self._mux_proc.poll()}) - '
            f'a dead process trivially forwards nothing, which must not be '
            f'confused with a live mux correctly withholding everything')
        deadline = time.monotonic() + 3.0
        names = []
        while time.monotonic() < deadline:
            names = self.probe.get_node_names()
            if 'twist_mux' in names:
                break
            rclpy.spin_once(self.probe, timeout_sec=0.05)
        self.assertIn(
            'twist_mux', names,
            f'the twist_mux node never became discoverable (saw {names}) - '
            f'cannot trust an empty output as "correctly locked" if the mux '
            f'itself never came up')

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
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        self._assert_nothing_nonzero_forwarded()


class TestMuxStartsBeforeGate(TwistMuxLiveTestCase):

    _create_gate_in_setup = False  # finding 1: the gate must not exist yet

    def test_mux_starting_before_the_gate_stays_locked(self):
        # The order Mohamed's review specifically called out: a delayed
        # helper (or a mux that just happens to come up first) must not
        # give nonzero commands a free window before the gate exists. The
        # gate genuinely does not exist yet here - self.gate is None until
        # constructed below, not just unspun (finding 1's actual bug).
        self.assertIsNone(
            self.gate, 'the gate must not exist yet at the start of this test')
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        time.sleep(0.5)  # mux fully up, gate genuinely does not exist yet
        self._publish_nonzero_nav_cmd(seconds=0.5)  # window before the gate exists at all
        # Now bring the gate up for real and keep publishing through its startup.
        self.gate = SafetyStopGateNode()
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        self._assert_nothing_nonzero_forwarded()


class TestMuxRestart(TwistMuxLiveTestCase):

    def test_mux_restart_with_lock_already_engaged_stays_locked(self):
        # Steady state first: mux up, gate up, confirm it is actually
        # forwarding the nav command's lower-priority twin once nothing is
        # locking it - prerequisite check, not the thing under test.
        self.gate.release_lock()
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
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
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        self._publish_nonzero_nav_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        self._assert_nothing_nonzero_forwarded()
