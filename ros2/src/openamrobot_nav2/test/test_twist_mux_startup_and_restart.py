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

Third review follow-up (Mohamed Sayed, platform-sw#90, 27 Oct 2026): the
sim-clock cold-start finding he raised earlier - fixed at the config/launch
level, see twist_mux.yaml's locks.software_stop.timeout comment and
navigation_launch.py's delayed_ui_cmd_vel_relay_node - was never actually
exercised by a test, because every class above runs the real wall clock.
Under wall time, rclcpp::Time's default-constructed "stamp_ at epoch 0"
baseline is already billions of seconds in the past the instant any process
starts, so hasExpired() trips essentially instantly and trivially; none of
that is true under use_sim_time, where now() genuinely starts at or near
zero, and must climb the full 0.3s before the same check flips. No test
here ever set use_sim_time or controlled /clock, so none of them could have
told the difference. TestMuxColdStartUnderSimClock below reproduces that
window directly against the real mux with a hand-controlled clock, with the
"forwarding positive control" Mohamed asked for as its first sub-test (see
that class's own docstring) - it does NOT touch or re-verify
navigation_launch.py's own wall-clock mitigation, which is a separate,
launch-level guarantee that a twist_mux-only test cannot exercise.

Fourth review follow-up (live manual repro against the real mux binary, 30
Oct 2026): the "forwarding positive control" sub-test below was written on
the assumption that holding the sim clock near zero is a safe window in
which cmd_vel_nav gets forwarded - true for the lock itself (confirmed via
twist_mux's own /diagnostics: it reports "free"), but false for the actual
forwarding result. In that window, every velocity input that has NEVER
received a single message (cmd_vel_stop prio 255, cmd_vel_dock prio 20,
cmd_vel_joy prio 100) reads "unmasked" too - not because anything is
publishing on them, but because twist_mux's TopicHandle_::hasExpired()
(topic_handle.hpp) is `(now() - stamp_) > timeout`, stamp_ defaults to 0,
and a sim clock held under ~0.25s never ages a never-received handle past
ITS OWN timeout either. Those phantom-valid, higher-priority handles
outrank cmd_vel_nav (priority 10) in twist_mux's arbitration, so nav never
wins - and since none of those higher-priority inputs ever actually
publish anything either, the net result is total silence on cmd_vel_mux.
There is no sim-time value, under this config's current timeouts, where a
safe cold-start forwarding window for cmd_vel_nav actually exists; see
pr90-found-during-review.md for the safety call this raises.
TestMuxColdStartUnderSimClock below is rewritten to prove that silence
directly, not the forwarding its name originally assumed.
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

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402,I100

from geometry_msgs.msg import Twist  # noqa: E402,I100

from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode  # noqa: E402

from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from rosgraph_msgs.msg import Clock  # noqa: E402

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


class _ControlledClockPublisher:
    """
    Publishes a hand-controlled /clock, independent of wall time.

    Mohamed's review (platform-sw#90, third round): none of the real-mux
    tests above ever run with use_sim_time, so none of them could reproduce
    the actual sim-time cold-start window the production fix (twist_mux.yaml
    / navigation_launch.py) targets - under wall time, rclcpp::Time's
    default-constructed "stamp_ at epoch 0" baseline is already billions of
    seconds in the past the instant any process starts, so there is nothing
    to reproduce there. This publishes real /clock messages (QoS: RELIABLE,
    depth 1 - satisfies either a RELIABLE or a BEST_EFFORT reader, so it
    does not matter which one rclcpp's internal TimeSource actually uses)
    at a value this test controls directly, instead of whatever real time
    happens to be.
    """

    def __init__(self, node):
        self._pub = node.create_publisher(
            Clock, '/clock',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._sec = 0
        self._nanosec = 0

    def set_seconds(self, seconds):
        self._sec = int(seconds)
        self._nanosec = int(round((seconds - self._sec) * 1e9))

    def advance(self, delta_seconds):
        self.set_seconds(self._sec + self._nanosec * 1e-9 + delta_seconds)

    def publish_once(self):
        msg = Clock()
        msg.clock = TimeMsg(sec=self._sec, nanosec=self._nanosec)
        self._pub.publish(msg)


class TestMuxColdStartUnderSimClock(unittest.TestCase):
    """
    Reproduces the sim-time cold-start window directly against the real mux.

    Mohamed's review (platform-sw#90, third round): the production fix for
    this (locks.software_stop.timeout: 0.3 in twist_mux.yaml, plus the
    wall-clock TimerAction delay on ui_cmd_vel_relay_node in
    navigation_launch.py - see both files' own comments) was never actually
    exercised under use_sim_time by any test in this file; every class
    above runs on the real wall clock, where the fail-closed default is
    trivially, instantly true (wall-clock now() is already huge), which
    proves nothing about the sim-time path the fix exists for.

    use_sim_time genuinely starts at or near zero and climbs - that is the
    real window this class reproduces, directly against the real twist_mux
    binary, with a hand-controlled /clock (_ControlledClockPublisher) this
    test drives itself rather than trusting whatever wall time happens to
    be.

    This intentionally does NOT re-verify navigation_launch.py's own
    mitigation (the wall-clock TimerAction that keeps every unmanaged raw
    producer from publishing during this window on the real robot) - that
    is a separate, launch-level guarantee a twist_mux-only test cannot
    exercise. What this class proves is narrower and still worth having on
    its own: that the generic timeout mechanism in twist_mux.yaml behaves
    exactly as its own comment says it does, under the clock source it was
    actually written for.

    Fourth review follow-up (live manual repro, 30 Oct 2026): the first
    sub-test below was originally the "forwarding positive control"
    Mohamed asked for - written on the assumption that this window is the
    one condition where a real, expected nonzero forward is known,
    accepted, pre-mitigation behavior. That assumption is false: a live
    manual repro against the real binary (both wall clock and sim clock)
    found nothing is forwarded in this window either, for a different
    reason than the lock (see the module docstring above and that test's
    own docstring for the confirmed mechanism). It no longer serves as a
    positive control for the second sub-test below - that test's docstring
    now carries its own note on what it can and cannot currently prove as
    a result.
    """

    @classmethod
    def setUpClass(cls):
        # Every node constructed below without its own override defaults to
        # sim time - see _ControlledClockPublisher, which must opt out.
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        shipped_timeout = _load_shipped_lock_timeout()
        assert shipped_timeout > 0.0, (
            'config/twist_mux.yaml locks.software_stop.timeout is %r - see '
            'TwistMuxLiveTestCase.setUpClass above for why this must stay '
            '> 0.0' % shipped_timeout)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = rclpy.create_node('sim_clock_probe')
        self.clock_node = rclpy.create_node(
            'sim_clock_driver',
            use_global_arguments=False,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self._clock = _ControlledClockPublisher(self.clock_node)
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
        self.probe.destroy_node()
        self.clock_node.destroy_node()

    def _start_mux_with_sim_time(self):
        return subprocess.Popen(
            ['ros2', 'run', 'twist_mux', 'twist_mux',
             '--ros-args', '--params-file', str(_SHIPPED_PARAMS_PATH),
             '-p', 'use_sim_time:=true',
             '-r', 'cmd_vel_out:=cmd_vel_mux'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _run_for(self, real_seconds, sim_rate=0.0):
        """
        Spin for a fixed wall-clock duration, driving /clock and a nonzero
        nav command every tick.

        sim_rate: simulated seconds added each tick (0.0 keeps the sim
        clock pinned at whatever set_seconds() last set - the cold-start
        hold; a positive value climbs it across the loop instead, to cross
        the 0.3s threshold without needing to wait that long in real time).
        """
        end = time.monotonic() + real_seconds
        while time.monotonic() < end:
            if sim_rate:
                self._clock.advance(sim_rate)
            self._clock.publish_once()
            self._nav_pub.publish(self._nonzero_twist)
            rclpy.spin_once(self.clock_node, timeout_sec=0.0)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _assert_mux_alive_and_discoverable(self):
        self.assertIsNotNone(self._mux_proc, 'no mux process was ever started')
        self.assertIsNone(
            self._mux_proc.poll(),
            f'the mux process exited early (code {self._mux_proc.poll()}) - '
            f'a dead process trivially forwards nothing')
        deadline = time.monotonic() + 3.0
        names = []
        while time.monotonic() < deadline:
            names = self.probe.get_node_names()
            if 'twist_mux' in names:
                break
            rclpy.spin_once(self.probe, timeout_sec=0.05)
        self.assertIn(
            'twist_mux', names,
            f'the twist_mux node never became discoverable (saw {names})')

    def test_nothing_forwarded_under_zero_sim_clock_phantom_handles_mask_nav(self):
        """
        Mux boots with sim clock pinned at 0.05s - never crossing 0.3s.

        Originally written as a "forwarding positive control", on the
        assumption that holding the sim clock near zero is a safe window in
        which cmd_vel_nav gets forwarded (no lock message ever received,
        lock not yet expired-to-locked). That assumption is false, confirmed
        against the real mux binary with a live manual repro (platform-sw#90,
        30 Oct 2026) and with twist_mux's own /diagnostics captured directly
        in this exact window:

            lock locks.software_stop:      free   (as expected)
            velocity topics.navigation:    unmasked, priority #10
            velocity topics.software_stop: unmasked, priority #255  (never received ANY message)
            velocity topics.docking:       unmasked, priority #20   (never received ANY message)
            velocity topics.manual:        unmasked, priority #100  (never received ANY message)

        The lock is correctly free - that is not what blocks forwarding.
        twist_mux's TopicHandle_::hasExpired() (topic_handle.hpp) is
        `(now() - stamp_) > timeout`, and stamp_ defaults to 0 for a topic
        that has never received a single message. With the sim clock held
        near zero, `now() - 0` never exceeds any of the configured timeouts
        (0.25s-0.5s) - so cmd_vel_stop/cmd_vel_dock/cmd_vel_joy, despite
        never publishing anything in this test, read as "unmasked" and
        outrank cmd_vel_nav (priority 10) in twist_mux's own arbitration.
        Nothing from navigation ever wins that comparison, and nothing from
        the phantom-valid higher-priority handles is ever actually
        published either (their own callbacks never fire without a real
        message) - so the result at cmd_vel_mux is total silence, not nav
        being forwarded and not any of the others either.

        This test now proves that silence, not a successful forward: there
        is no sim-time value, under this config's current timeouts, where a
        safe cold-start forwarding window for cmd_vel_nav actually exists.
        See pr90-found-during-review.md for the safety call this raises
        (Mohamed Sayed / platform lead sign-off needed) - this file does
        not change config/twist_mux.yaml or safety_stop_gate_node.py to
        chase a fix on its own.
        """
        self._clock.set_seconds(0.05)
        self._clock.publish_once()
        self._mux_proc = self._start_mux_with_sim_time()
        self._run_for(real_seconds=1.0)  # sim clock held at 0.05s throughout
        self._assert_mux_alive_and_discoverable()
        nonzero = [m for m in self._mux_out if m != Twist()]
        self.assertEqual(
            nonzero, [],
            f'twist_mux forwarded {len(nonzero)} nonzero command(s) while '
            f'the sim clock was held under the 0.3s threshold with no lock '
            f'message ever received: {nonzero[:3]} - if this now fails, '
            f'something about the phantom-handle starvation documented '
            f'above has changed (e.g. a timeout was shortened below '
            f'another input\'s) and needs re-checking against a live '
            f'manual repro before trusting it, not just taken as the '
            f'original "positive control" assumption restored')

    def test_stays_locked_once_sim_clock_crosses_threshold(self):
        """
        Same boot, but the sim clock is allowed to actually climb past 0.3s.

        Confirms the default-fail-closed invariant (already covered under
        wall time by TestMuxStartsAfterGate/TestMuxStartsBeforeGate above)
        also holds once a SIMULATED clock has genuinely had 0.3s to tick
        past epoch zero - not just once some arbitrarily-large wall-clock
        epoch number happens to already be past it, which is true from the
        first instant under wall time and therefore cannot tell these two
        clock sources apart.

        Fourth review follow-up (live manual repro, 30 Oct 2026): this
        test's pass is currently UNFALSIFIABLE and should be read with that
        limitation, not as proof the lock mechanism alone is doing its job
        here. Below the lock's own 0.3s threshold, nothing is forwarded
        either - not because the lock is engaged, but because the phantom
        higher-priority handles documented on
        test_nothing_forwarded_under_zero_sim_clock_phantom_handles_mask_nav
        above dominate first (their timeouts, 0.25s-0.5s, are all >= the
        lock's 0.3s). So this test currently cannot distinguish "the lock
        correctly stayed engaged" from "the phantom-handle effect alone was
        enough to silence everything, lock state notwithstanding" - both
        produce the same observed nonzero=[] result, before AND after the
        threshold this test crosses. Making this test falsifiable needs the
        phantom-handle effect addressed first (see
        pr90-found-during-review.md) - not a change to the assertion
        below, which stays correct as a check on the OLD mux's continued
        lock-after-restart behavior; it is just not currently proof of what
        its own docstring above claims to isolate.
        """
        self._clock.set_seconds(0.0)
        self._clock.publish_once()
        self._mux_proc = self._start_mux_with_sim_time()
        # Let the mux come up at sim t=0 first - the fail-open window the
        # test above measures - then climb well past the threshold.
        self._run_for(real_seconds=0.2)
        self._run_for(real_seconds=1.0, sim_rate=0.02)
        self._mux_out.clear()
        self._run_for(real_seconds=1.0)  # hold steady, now past the threshold
        self._assert_mux_alive_and_discoverable()
        nonzero = [m for m in self._mux_out if m != Twist()]
        self.assertEqual(
            nonzero, [],
            f'twist_mux forwarded {len(nonzero)} nonzero command(s) after '
            f'the sim clock had already climbed past the 0.3s timeout '
            f'threshold with no lock message ever received: {nonzero[:3]}')
