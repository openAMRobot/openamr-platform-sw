"""
Simulated-clock readiness and stop-on-engage, real gate and real twist_mux (#35, #90).

Platform-lead review of #90 decided how the stop gate behaves under
use_sim_time:=true. This file holds the gate and the arbitration chain to those
decisions. Rows refer to the test plan in the design proposal comment on #90.

Decisions under test:
  - release is refused while ROS time is below T_ready = 1.0 s, answering
    success=False with a reason (rows 1a, 2d);
  - once ROS time is past T_ready, release is accepted and navigation and
    docking commands reach cmd_vel_gated unchanged (rows 2a, 2b, 2c);
  - engaging the lock publishes one fresh zero on cmd_vel_gated even when the
    simulated clock is stalled and the mux is silent (row 4a);
  - a late nonzero mux message after engage is still zeroed (row 4b);
  - restarts come back locked and recover only through an explicit release
    (rows 3a, 3b);
  - the mux itself forwards navigation once the clock is past its longest input
    timeout, with no gate involved (row 1c).

A frozen simulated clock is a paused simulation and is out of scope here (it is a
documented simulation-only limitation, not something these tests assert).

What these tests do not show: the Gazebo clock bridge, TF, collision_monitor,
velocity_smoother, hardware, or the hardwired safety chain. They are functional
checks of the stop command path under a hand-driven /clock, not safety evidence.

Rows 1a, 2d and 4a fail on a gate without the readiness refusal and the
engage-time zero; run them first on the unmodified gate, record the result, then
run them again with the gate change in place.

Needs a ROS 2 Jazzy workspace. Rows 1c, 2a-2d and 3a-3b also need the real
twist_mux package and are skipped without it:

    colcon build --packages-select openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v src/openamrobot_nav2/test/test_sim_clock_readiness_full_chain.py
"""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')  # tracking: #35


def _twist_mux_available():
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'twist_mux'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


requires_mux = pytest.mark.skipif(
    not _twist_mux_available(),
    reason='needs the real ros2 CLI AND the twist_mux package installed [#35]')

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402,I100

from geometry_msgs.msg import Twist  # noqa: E402,I100

from openamrobot_nav2.safety_stop_gate_node import SafetyStopGateNode  # noqa: E402

from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from rosgraph_msgs.msg import Clock  # noqa: E402

from std_msgs.msg import Bool  # noqa: E402

from std_srvs.srv import Trigger  # noqa: E402

_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'

# Decision D2 on #90: release is refused below this ROS time.
T_READY_S = 1.0
# A clock value comfortably past T_ready for the "ready" cases.
READY_CLOCK_S = 1.5
# The driver publishes /clock, the lock and commands once per STEP_S of wall time
# (100 Hz), the same pacing as the standalone diagnostic that showed the mux
# forwarding. An unpaced loop publishes far faster than that and is not what any
# real publisher does.
STEP_S = 0.01
# Simulated seconds added per step when the clock is ramped (real-time factor 1).
SIM_PER_TICK_S = 0.01
# Margin on top of the stop input's own timeout for release-to-forward (row 2b).
FORWARD_MARGIN_S = 0.3

_RELEASE = '/safety/stop_lock_release'
_ENGAGE = '/safety/stop_lock_engage'


def _load_shipped_topics():
    with open(_SHIPPED_PARAMS_PATH, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    return params['twist_mux']['ros__parameters']['topics']


def _start_mux_with_sim_time():
    # Own process group: `ros2 run` starts the real twist_mux as a child, and
    # signalling only the wrapper leaves that child running after the test. The
    # whole group is signalled in _stop_mux.
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux',
         '--ros-args', '--params-file', str(_SHIPPED_PARAMS_PATH),
         '-p', 'use_sim_time:=true',
         '-r', 'cmd_vel_out:=cmd_vel_mux'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)


def _signal_group(proc, sig):
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        pass


def _stop_mux(proc):
    _signal_group(proc, signal.SIGINT)
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        _signal_group(proc, signal.SIGKILL)
        proc.wait(timeout=3.0)


class _ControlledClockPublisher:
    """Publishes a /clock whose value this test sets, independent of wall time."""

    def __init__(self, node):
        self._pub = node.create_publisher(
            Clock, '/clock',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._sec = 0
        self._nanosec = 0

    def seconds(self):
        return self._sec + self._nanosec * 1e-9

    def set_seconds(self, seconds):
        self._sec = int(seconds)
        self._nanosec = int(round((seconds - self._sec) * 1e9))

    def advance(self, delta_seconds):
        self.set_seconds(self.seconds() + delta_seconds)

    def publish_once(self):
        msg = Clock()
        msg.clock = TimeMsg(sec=self._sec, nanosec=self._nanosec)
        self._pub.publish(msg)


class SimClockChainCase(unittest.TestCase):
    """
    Shared plumbing: a hand-driven /clock, the real gate, and probes.

    Every node here runs with use_sim_time:=true except the clock driver. The
    real mux, when a test starts one, also runs on simulated time.
    """

    _with_gate = True

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode() if self._with_gate else None
        self.probe = rclpy.create_node('sim_readiness_probe')
        self.clock_node = rclpy.create_node(
            'sim_readiness_clock_driver',
            use_global_arguments=False,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self._clock = _ControlledClockPublisher(self.clock_node)
        self._nav_pub = self.probe.create_publisher(Twist, 'cmd_vel_nav', 10)
        self._dock_pub = self.probe.create_publisher(Twist, 'cmd_vel_dock', 10)
        self._lock_pub = self.probe.create_publisher(Bool, '/safety/stop_lock', 10)
        self._mux_pub = self.probe.create_publisher(Twist, 'cmd_vel_mux', 10)
        # (simulated seconds when received, message)
        self._gated = []
        self._mux_out = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', self._on_gated, 10)
        self.probe.create_subscription(
            Twist, 'cmd_vel_mux', lambda msg: self._mux_out.append(msg), 10)
        self._mux_proc = None
        self._nonzero = Twist()
        self._nonzero.linear.x = 0.3

    def tearDown(self):
        self._stop_current_mux()
        if self.gate is not None:
            self.gate.destroy_node()
        self.probe.destroy_node()
        self.clock_node.destroy_node()

    def _on_gated(self, msg):
        self._gated.append((self._clock.seconds(), msg))

    def _gated_messages(self):
        return [msg for _, msg in self._gated]

    def _nonzero_gated(self):
        return [msg for msg in self._gated_messages() if msg != Twist()]

    def _spin_all(self):
        if self.gate is not None:
            rclpy.spin_once(self.gate, timeout_sec=0.005)
        rclpy.spin_once(self.probe, timeout_sec=0.005)
        rclpy.spin_once(self.clock_node, timeout_sec=0.0)

    def _run_once(self, sim_per_tick=0.0, publish_clock=True,
                  nav=False, dock=False, lock_false=False):
        if publish_clock:
            if sim_per_tick:
                self._clock.advance(sim_per_tick)
            self._clock.publish_once()
        if nav:
            self._nav_pub.publish(self._nonzero)
        if dock:
            self._dock_pub.publish(self._nonzero)
        if lock_false:
            self._lock_pub.publish(Bool(data=False))
        end = time.monotonic() + STEP_S
        while time.monotonic() < end:
            self._spin_all()

    def _run(self, real_seconds, **kwargs):
        end = time.monotonic() + real_seconds
        while time.monotonic() < end:
            self._run_once(**kwargs)

    def _ramp_until(self, target_sim_seconds, max_real_seconds=30.0, **kwargs):
        """Advance the simulated clock in small steps until it reaches the target."""
        end = time.monotonic() + max_real_seconds
        while self._clock.seconds() < target_sim_seconds and time.monotonic() < end:
            self._run_once(sim_per_tick=SIM_PER_TICK_S, **kwargs)
        self.assertGreaterEqual(
            self._clock.seconds(), target_sim_seconds,
            'the simulated clock did not reach the target in time')

    def _trigger(self, service_name):
        client = self.probe.create_client(Trigger, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=3.0), f'{service_name} never came up')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self._spin_all()
        self.assertTrue(future.done(), f'{service_name} call timed out')
        return future.result()

    def _wait_for_subscriber(self, publisher, what):
        end = time.monotonic() + 2.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < end:
            self._spin_all()
        self.assertGreater(
            publisher.get_subscription_count(), 0,
            f'nothing ever subscribed to {what}')

    def _rearm_after_release(self):
        """
        Satisfy the rearm latch after release, before expecting forwarding.

        platform-sw#90 review (Alex, BLOCKER 1): release_lock() alone must
        not let stale motion resume - cmd_vel_gated holds zero until the mux
        output has itself been zero at least once. Publishes directly onto
        cmd_vel_mux (bypassing the real mux, the same synthetic injection
        row 4a already uses) since satisfying the latch is setup here, not
        the subject under test - that is test_safety_stop_gate_node.py's own
        TestReleaseRequiresRearmBeforeResumingMotion.
        """
        self._wait_for_subscriber(self._mux_pub, 'cmd_vel_mux (rearm)')
        self._mux_pub.publish(Twist())
        self._run_once()

    def _start_mux(self):
        self._mux_proc = _start_mux_with_sim_time()

    def _stop_current_mux(self):
        if self._mux_proc is not None:
            _stop_mux(self._mux_proc)
            self._mux_proc = None

    def _assert_mux_alive_and_discoverable(self):
        self.assertIsNotNone(self._mux_proc, 'no mux process was ever started')
        self.assertIsNone(
            self._mux_proc.poll(),
            f'the mux process exited early (code {self._mux_proc.poll()}); a dead '
            f'process forwards nothing, which must not be read as a live mux '
            f'withholding commands')
        end = time.monotonic() + 3.0
        names = []
        while time.monotonic() < end:
            names = self.probe.get_node_names()
            if 'twist_mux' in names:
                break
            self._spin_all()
        self.assertIn(
            'twist_mux', names,
            f'the twist_mux node never became discoverable (saw {names})')


class TestReleaseRefusedWithNoClock(SimClockChainCase):
    """Row 1a: no /clock message ever received."""

    def test_release_refused_and_nothing_nonzero_with_no_clock_ever_received(self):
        # No /clock is published in this test at all, so ROS time is 0.
        self.assertTrue(self.gate._locked)
        response = self._trigger(_RELEASE)
        self.assertFalse(
            response.success,
            'release was accepted with ROS time at 0 (< T_ready): the readiness '
            'refusal decided on #90 is missing')
        self.assertTrue(response.message, 'a refused release must give a reason')
        self.assertRegex(
            response.message.lower(), r'clock|time|ready',
            f'refusal reason does not say why: {response.message!r}')
        self.assertTrue(self.gate._locked, 'a refused release must leave the gate locked')

        self._wait_for_subscriber(self._mux_pub, 'cmd_vel_mux')
        self._mux_pub.publish(self._nonzero)
        self._run(0.3, publish_clock=False)
        received = self._gated_messages()
        self.assertTrue(
            received,
            'expected the message-driven zero on cmd_vel_gated for a mux message')
        for msg in received:
            self.assertEqual(
                msg, Twist(),
                'a nonzero command reached cmd_vel_gated although the gate is locked')


@requires_mux
class TestMuxForwardsOncePastLongestTimeout(SimClockChainCase):
    """
    Row 1c: the mux alone, no gate. With a pinned clock at or past the longest input
    timeout (0.5 s), the never-received higher-priority inputs read as expired and
    navigation is forwarded. This records the measured boundary against the shipped
    config; it does not involve the gate.
    """

    _with_gate = False

    def test_navigation_forwarded_when_clock_is_pinned_past_longest_timeout(self):
        topics = _load_shipped_topics()
        longest = max(t['timeout'] for t in topics.values())
        for pinned in (0.6, 1.0):
            self.assertGreater(
                pinned, longest,
                'the shipped input timeouts changed; revisit the pinned values')
            with self.subTest(pinned_clock_seconds=pinned):
                self._mux_out.clear()
                self._clock.set_seconds(pinned)
                self._start_mux()
                self._run(3.0, nav=True, lock_false=True)
                self._assert_mux_alive_and_discoverable()
                forwarded = [m for m in self._mux_out if m == self._nonzero]
                self.assertTrue(
                    forwarded,
                    f'navigation was not forwarded with the clock pinned at {pinned} s, '
                    f'past the longest input timeout ({longest} s)')
                self._stop_current_mux()
                self._run(0.5, publish_clock=False)


@requires_mux
class TestReadyReleaseForwards(SimClockChainCase):
    """Rows 2a-2d: real mux and real gate, clock driven past T_ready."""

    def _release_when_ready_and_wait_for_forward(self, nav=False, dock=False):
        self._start_mux()
        self._ramp_until(READY_CLOCK_S, nav=nav, dock=dock)
        self._assert_mux_alive_and_discoverable()
        self.assertGreaterEqual(self._clock.seconds(), T_READY_S)
        self.assertEqual(
            self._nonzero_gated(), [],
            'a nonzero command reached cmd_vel_gated before any release')

        response = self._trigger(_RELEASE)
        self.assertTrue(
            response.success,
            f'release was refused with ROS time at {self._clock.seconds():.2f} s '
            f'(>= T_ready): {response.message!r}')
        self.assertFalse(self.gate._locked)
        t_release = self._clock.seconds()

        self._rearm_after_release()
        self._gated.clear()
        self._run(3.0, sim_per_tick=SIM_PER_TICK_S, nav=nav, dock=dock)
        forwarded = [(t, m) for t, m in self._gated if m == self._nonzero]
        self.assertTrue(
            forwarded,
            'the command never reached cmd_vel_gated after a release past T_ready; '
            'the positive control failed, so no silence in this chain can be '
            'read as the stop working')
        return t_release, forwarded[0][0]

    def test_2a_navigation_forwarded_unchanged_after_ready_release(self):
        self._release_when_ready_and_wait_for_forward(nav=True)

    def test_2b_release_to_first_forward_is_bounded_by_the_stop_input_timeout(self):
        stop_timeout = _load_shipped_topics()['software_stop']['timeout']
        t_release, t_first = self._release_when_ready_and_wait_for_forward(nav=True)
        self.assertLessEqual(
            t_first - t_release, stop_timeout + FORWARD_MARGIN_S,
            f'first forward took {t_first - t_release:.2f} simulated seconds after '
            f'release, more than the stop input timeout ({stop_timeout} s) plus '
            f'{FORWARD_MARGIN_S} s: the stop input does not age out as the design '
            f'assumed')

    def test_2c_docking_forwarded_unchanged_after_ready_release(self):
        self._release_when_ready_and_wait_for_forward(dock=True)

    def test_2d_release_refused_below_t_ready_with_real_mux(self):
        self._clock.set_seconds(0.3)
        self._start_mux()
        self._run(1.0, nav=True)
        self._assert_mux_alive_and_discoverable()
        response = self._trigger(_RELEASE)
        self.assertFalse(
            response.success,
            'release was accepted with ROS time at 0.3 s (< T_ready)')
        self.assertTrue(response.message)
        self.assertTrue(self.gate._locked)
        self._gated.clear()
        self._run(1.5, nav=True)
        self.assertEqual(
            self._nonzero_gated(), [],
            'a nonzero command reached cmd_vel_gated after a refused release')


@requires_mux
class TestEngageAndRestartsWithClockRunning(SimClockChainCase):
    """Rows 3a-3b: engage, gate restart and mux restart, with the clock running."""

    def _forwarding_established(self):
        self._start_mux()
        self._ramp_until(READY_CLOCK_S, nav=True)
        response = self._trigger(_RELEASE)
        self.assertTrue(response.success, response.message)
        self._rearm_after_release()
        self._gated.clear()
        self._run(2.0, sim_per_tick=SIM_PER_TICK_S, nav=True)
        self.assertIn(
            self._nonzero, self._gated_messages(),
            'positive control failed: navigation never reached cmd_vel_gated '
            'after release, so the stop checks below would prove nothing')

    def _release_and_expect_forwarding(self):
        response = self._trigger(_RELEASE)
        self.assertTrue(response.success, response.message)
        self._rearm_after_release()
        self._gated.clear()
        self._run(3.0, sim_per_tick=SIM_PER_TICK_S, nav=True)
        self.assertIn(
            self._nonzero, self._gated_messages(),
            'navigation did not reach cmd_vel_gated after the explicit release')

    def _assert_stays_stopped(self, real_seconds):
        self._run(1.0, sim_per_tick=SIM_PER_TICK_S, nav=True)
        self._gated.clear()
        self._run(real_seconds, sim_per_tick=SIM_PER_TICK_S, nav=True)
        self.assertEqual(
            self._nonzero_gated(), [],
            'a nonzero command reached cmd_vel_gated while the lock was engaged')
        self.assertTrue(
            self._gated,
            'cmd_vel_gated carried nothing at all while locked; the gate should '
            'be publishing its zero, so this cannot be told apart from a dead gate')

    def test_3a_engage_while_forwarding_stops_cmd_vel_gated(self):
        self._forwarding_established()
        response = self._trigger(_ENGAGE)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.gate._locked)
        self._assert_stays_stopped(2.0)

    def test_3b_gate_restart_comes_back_locked_and_needs_a_new_release(self):
        self._forwarding_established()
        self.gate.destroy_node()
        self.gate = SafetyStopGateNode()
        self.assertTrue(self.gate._locked)
        self._assert_stays_stopped(2.0)
        self._release_and_expect_forwarding()

    def test_3b_mux_restart_while_locked_stays_locked_until_released(self):
        self._forwarding_established()
        response = self._trigger(_ENGAGE)
        self.assertTrue(response.success, response.message)
        self._run(0.5, sim_per_tick=SIM_PER_TICK_S, nav=True)
        self._stop_current_mux()
        self._start_mux()
        self._assert_mux_alive_and_discoverable()
        self._assert_stays_stopped(2.0)
        self._release_and_expect_forwarding()


class TestEngageUnderStalledClock(SimClockChainCase):
    """
    Rows 4a-4b: engage while /clock has stalled and the mux is silent.

    The gate's periodic tick runs on ROS time and does not fire without /clock, and
    engage_lock() on its own only sets the flag and publishes the lock Bool. Row 4a
    holds the gate to publishing one fresh zero on cmd_vel_gated at engage anyway.
    """

    def _forward_then_stall_the_clock(self):
        self._ramp_until(READY_CLOCK_S)
        response = self._trigger(_RELEASE)
        self.assertTrue(response.success, response.message)
        self._rearm_after_release()
        self._wait_for_subscriber(self._mux_pub, 'cmd_vel_mux')
        self._mux_pub.publish(self._nonzero)
        self._run(0.3, publish_clock=False)
        self.assertIn(
            self._nonzero, self._gated_messages(),
            'positive control failed: the released gate did not forward a mux message')
        # /clock stops here: nothing is published on it from now on.
        self._run(0.3, publish_clock=False)
        self._gated.clear()

    def test_4a_engage_publishes_a_fresh_zero_with_a_stalled_clock_and_silent_mux(self):
        self._forward_then_stall_the_clock()
        response = self._trigger(_ENGAGE)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.gate._locked)
        self._run(0.5, publish_clock=False)
        after_engage = self._gated_messages()
        self.assertTrue(
            after_engage,
            'no message at all reached cmd_vel_gated after engage with /clock stalled '
            'and the mux silent: engage publishes the lock Bool only, so the output '
            'keeps its last command until the clock or the mux wakes up')
        for msg in after_engage:
            self.assertEqual(msg, Twist(), 'a nonzero command followed the engage')

    def test_4b_late_nonzero_mux_message_after_engage_still_comes_out_zero(self):
        self._forward_then_stall_the_clock()
        response = self._trigger(_ENGAGE)
        self.assertTrue(response.success, response.message)
        self._run(0.3, publish_clock=False)
        self._gated.clear()
        self._mux_pub.publish(self._nonzero)
        self._run(0.5, publish_clock=False)
        after_late = self._gated_messages()
        self.assertTrue(
            after_late,
            'a late mux message produced nothing on cmd_vel_gated')
        for msg in after_late:
            self.assertEqual(
                msg, Twist(),
                'a nonzero mux message reached cmd_vel_gated after engage')
