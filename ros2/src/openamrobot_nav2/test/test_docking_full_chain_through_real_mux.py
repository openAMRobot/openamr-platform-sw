"""
Full-chain docking approach/abort test, through the REAL twist_mux (platform-sw#90,
Mohamed Sayed's follow-up review at 710c98b).

Every other real-mux test in this package (test_twist_mux_startup_and_restart.py)
exercises the navigation input (cmd_vel_nav, priority 10). That is not the same
claim as "docking is safe through the mux": docking is a DIFFERENT input
(cmd_vel_dock, priority 20 - see config/twist_mux.yaml) at a DIFFERENT priority,
sitting between navigation (10) and manual (100) in the arbitration order, and
this PR's own dock_trigger.yaml change (pause_collision_monitor_during_dock:
false) only makes sense if cmd_vel_dock really does flow through this exact
chain - cmd_vel_dock -> twist_mux -> cmd_vel_mux -> safety_stop_gate_node ->
cmd_vel_gated - and really does get zeroed at the gate on an abort, the same
way every other input does. Nothing upstream of this test file actually proves
that for docking specifically; it only proved it for navigation.

This does NOT touch dock_trigger.py or run the real 7-phase sequencer - that
node is unmodified by this PR (only its config changed: cmd_vel_topic ->
cmd_vel_dock, pause_collision_monitor_during_dock -> false) and is out of
scope here. These tests publish directly onto cmd_vel_dock, standing in for
"dock_trigger is mid-approach, commanding forward motion" - the mux and gate
cannot tell the difference between that and the real node, which is exactly
the point: the arbitration/safety layer's correctness must not depend on
which node happens to be driving cmd_vel_dock.

Needs a full ROS 2 Jazzy workspace with the real twist_mux package installed
and ros2 on PATH - skipped everywhere else, same convention as
test_twist_mux_startup_and_restart.py, which this file deliberately mirrors
(same subprocess-managed real-mux infrastructure, same shipped-config load,
same positive-control checks) rather than inventing a second pattern. Run for
real with:

    colcon build --packages-select openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v -s \
        src/openamrobot_nav2/test/test_docking_full_chain_through_real_mux.py
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

_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'


def _load_shipped_docking_topic_config():
    with open(_SHIPPED_PARAMS_PATH, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    return params['twist_mux']['ros__parameters']['topics']['docking']


def _start_mux(params_path):
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


class DockingFullChainLiveTestCase(unittest.TestCase):
    """Real mux + real gate, probing cmd_vel_dock -> cmd_vel_mux -> cmd_vel_gated."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        # Tie this file to the shipped docking topic/priority, the same way
        # test_twist_mux_startup_and_restart.py ties itself to the shipped lock
        # timeout - a revert of either value should fail loudly HERE, not pass
        # silently against a hand-copied config.
        docking_cfg = _load_shipped_docking_topic_config()
        assert docking_cfg['topic'] == 'cmd_vel_dock', (
            "config/twist_mux.yaml topics.docking.topic is %r, expected "
            "'cmd_vel_dock' - dock_trigger.yaml's cmd_vel_topic override "
            "(platform-sw#90) only takes effect if this matches; fix the "
            "shipped config, do not change this assertion" % docking_cfg['topic'])
        assert docking_cfg['priority'] == 20, (
            'config/twist_mux.yaml topics.docking.priority is %r, expected 20 '
            '(between navigation=10 and manual=100, per the file\'s own '
            'header comment) - fix the shipped config, do not change this '
            'assertion' % docking_cfg['priority'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode()
        self.probe = rclpy.create_node('docking_full_chain_probe')
        self._dock_pub = self.probe.create_publisher(Twist, 'cmd_vel_dock', 10)
        self._gated_out = []
        self.probe.create_subscription(
            Twist, 'cmd_vel_gated', self._gated_out.append, 10)
        self._mux_proc = None
        self._nonzero_approach_twist = Twist()
        self._nonzero_approach_twist.linear.x = 0.10  # dock_trigger's drive_speed order

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

    def _publish_nonzero_dock_cmd(self, seconds):
        """Stand-in for dock_trigger mid-approach: a steady forward command on
        cmd_vel_dock, interleaved with spinning both real nodes."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._dock_pub.publish(self._nonzero_approach_twist)
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _assert_mux_alive_and_discoverable(self):
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
            f'the twist_mux node never became discoverable (saw {names})')


class TestDockingApproachReachesGatedOutput(DockingFullChainLiveTestCase):
    """The positive case: an in-progress dock approach actually reaches
    cmd_vel_gated, unchanged, through the real mux - priority 20 is not
    masked by anything else when nothing else is commanding and no lock is
    engaged. Without this, the abort test below would be proving the gate
    correctly blocks a signal that was never getting through in the first
    place."""

    def test_nonzero_cmd_vel_dock_forwarded_through_real_mux_while_unlocked(self):
        self.gate.release_lock()
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        self._publish_nonzero_dock_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        self.assertTrue(
            any(m == self._nonzero_approach_twist for m in self._gated_out),
            'the docking approach command never reached cmd_vel_gated through '
            'the real mux while unlocked - the chain is not actually '
            'connected for cmd_vel_dock, so nothing below this proves docking '
            'is safe, only that it is also broken')


class TestDockingAbortZeroesGatedOutput(DockingFullChainLiveTestCase):
    """The actual ask: an abort (stop-lock engaged) while dock_trigger is mid-
    approach must zero the FINAL cmd_vel_gated output, through the real mux,
    for the docking input specifically - not inferred from the nav-priority
    case, which sits at a different priority (10 vs 20) in the same mux."""

    def test_lock_engaged_during_approach_zeroes_cmd_vel_gated(self):
        self.gate.release_lock()
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        self._publish_nonzero_dock_cmd(seconds=1.0)
        self.assertTrue(
            any(m == self._nonzero_approach_twist for m in self._gated_out),
            'sanity check failed: the approach command never reached '
            'cmd_vel_gated while unlocked, so an all-zero result below would '
            'not prove the abort path works - it would just mean cmd_vel_dock '
            'was never connected')

        # The abort: engage the lock while dock_trigger (stood in for by the
        # probe) keeps commanding forward motion, exactly as it would mid-
        # approach if it has not yet reacted to the abort itself.
        #
        # engage_lock() flips self._locked synchronously, but messages
        # published during the unlocked window above are still in flight
        # across the two real DDS hops (dock_pub -> mux -> gate ->
        # cmd_vel_gated -> probe) - clearing the Python-side buffer here
        # cannot un-send what the transport is still delivering. Give it a
        # settle window (spinning, not publishing) to actually drain before
        # clearing, the same lesson test_twist_mux_startup_and_restart.py's
        # TestMuxRestart already documents for the equivalent nav-input
        # case - otherwise stale, pre-lock messages arriving late would be
        # miscounted as the abort failing.
        self.gate.engage_lock()
        self._spin_for(0.5)
        self._gated_out.clear()

        self._publish_nonzero_dock_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        nonzero = [m for m in self._gated_out if m != Twist()]
        self.assertEqual(
            nonzero, [],
            f'cmd_vel_gated carried {len(nonzero)} nonzero command(s) from '
            f'cmd_vel_dock after the stop lock engaged: {nonzero[:3]} - a '
            f'docking abort does not actually stop the robot through the '
            f'real mux')


class TestDockingAlreadyLockedBeforeApproach(DockingFullChainLiveTestCase):
    """Startup-order variant for docking specifically: if the lock is already
    engaged (e.g. the robot started locked, per safety_stop_gate_node's own
    __init__ default) before dock_trigger ever publishes, its first commands
    must still come out zero - covers a dock attempt starting during an
    already-active stop, not only a stop that interrupts one in progress."""

    def test_dock_cmd_published_while_already_locked_comes_out_zero(self):
        # Gate already defaults to locked from construction (see setUp) -
        # confirm that, then start the mux and publish as if docking had
        # started without anyone releasing the lock first.
        self.assertTrue(self.gate._locked)
        self._mux_proc = _start_mux(_SHIPPED_PARAMS_PATH)
        self._publish_nonzero_dock_cmd(seconds=2.0)
        self._assert_mux_alive_and_discoverable()
        nonzero = [m for m in self._gated_out if m != Twist()]
        self.assertEqual(
            nonzero, [],
            f'cmd_vel_gated carried {len(nonzero)} nonzero command(s) from '
            f'cmd_vel_dock while the robot was already locked before the '
            f'approach started: {nonzero[:3]}')
