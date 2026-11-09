"""
D-13 (platform-sw#124) integration tests against a REAL dock_trigger.py
subprocess - closing the "mechanism exists, unit-tested only" gap on PR
#90's own test-status table, rows 1 (clean-boot) and 2 (enabled_producers
exclusion).

Unlike test_stop_contract_full_chain.py (which fakes the dock-trigger
status/abort entirely - see its own docstring on why dock_trigger's phase
loops are out of reach there), this file only needs dock_trigger.py's
constructor and its 2.0s heartbeat timer - neither touches TF/camera/LiDAR,
so a real subprocess is reachable here.

Row 3 (producer restart) is NOT attempted against a real subprocess: D-19
(platform-sw#124, #78) gates every public entry point that could make
dock_trigger send a real undock goal (on_undock, on_goal_pose,
on_trigger(False)'s undock_on_false path), so there is currently no way to
produce a real `sent` event through dock_trigger at all while D-19 holds.
See test_stop_contract_full_chain.py's TestUndockRobotLedgerSurvivesARealRestart
instead, which probe-publishes the real goal_authority_events wire protocol
directly - genuinely exercising the gate's own parsing/ledger-update path,
just not dock_trigger's sending side.
"""
import os
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #124

from action_msgs.msg import GoalStatusArray  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from openamrobot_nav2.safety_stop_gate_node import (  # noqa: E402
    _CANCEL_ACTIONS, SafetyStopGateNode, T_READY)
from rclpy.duration import Duration  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

_LEGACY_ACTIONS = [a for a in _CANCEL_ACTIONS if a != 'undock_robot']


def _start_dock_trigger():
    # start_new_session=True puts the real python child (ros2 run is a thin
    # wrapper around it) in its own process group - SIGINT/kill sent only to
    # the wrapper's own PID does not reach that child, which is exactly the
    # known orphan-process footgun seen all session with twist_mux's own
    # _start_mux()/_stop_mux() in test_stop_contract_full_chain.py. Killing
    # the whole group in _stop_dock_trigger() is what actually reaps it.
    return subprocess.Popen(
        ['ros2', 'run', 'openamrobot_docking', 'dock_trigger.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)


def _stop_dock_trigger(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5.0)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5.0)


class _BaseCase(unittest.TestCase):

    def _spin_both(self, timeout_s=0.5):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _serve_legacy_actions_clear(self):
        # navigate_to_pose/navigate_through_poses/dock_robot stay on the
        # legacy status-topic check - give them a believable "clear"
        # baseline so undock_robot's REAL ledger is what release refusal
        # actually hinges on in these tests.
        self._cancel_services = {}
        for action in _LEGACY_ACTIONS:
            def _cb(request, response):
                del request
                response.return_code = CancelGoal.Response.ERROR_NONE
                return response
            self._cancel_services[action] = self.probe.create_service(
                CancelGoal, f'{action}/_action/cancel_goal', _cb)
            pub = self.probe.create_publisher(GoalStatusArray, f'{action}/_action/status', 10)
            deadline = time.monotonic() + 2.0
            while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                self._spin_both(0.02)
            pub.publish(GoalStatusArray())
            pub.destroy()
        self._spin_both(0.2)

    def _try_release(self):
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self._spin_both(0.02)
        self.assertTrue(future.done(), 'release call timed out')
        client.destroy()
        return future.result()


class TestCleanBootResumeWithRealDockTrigger(_BaseCase):
    """
    D-13 test-status table row 1: clean-boot Resume within ~2s, with no
    goal ever sent, against the REAL dock_trigger.py - not a fake heartbeat.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode()
        self.gate._start -= Duration(seconds=T_READY + 1.0)
        self.probe = rclpy.create_node('d13_real_dock_trigger_clean_boot_probe')
        self._dock_proc = _start_dock_trigger()

    def tearDown(self):
        _stop_dock_trigger(self._dock_proc)
        self.gate.destroy_node()
        self.probe.destroy_node()

    def test_release_succeeds_within_a_few_seconds_of_dock_trigger_construction(self):
        self._serve_legacy_actions_clear()

        # Wait for dock_trigger to be past construction (its ~/abort
        # service exists) before timing the D-13 "within 2s" bound from
        # there - subprocess/interpreter startup latency is a separate,
        # unbounded concern the ruling does not address; the heartbeat
        # timer only starts counting once the node actually exists.
        abort_client = self.probe.create_client(Trigger, '/dock_trigger/abort')
        self.assertTrue(
            abort_client.wait_for_service(timeout_sec=20.0),
            'the real dock_trigger.py subprocess never came up')
        abort_client.destroy()
        t0 = time.monotonic()

        result = None
        deadline = t0 + 10.0
        while time.monotonic() < deadline:
            result = self._try_release()
            if result.success:
                break
            self._spin_both(0.1)
        elapsed = time.monotonic() - t0

        self.assertIsNotNone(result)
        self.assertTrue(
            result.success,
            f'release never succeeded against the real dock_trigger subprocess '
            f'(last message: {result.message if result else "none"})')
        self.assertLess(
            elapsed, 4.0,
            f"D-13: clean-boot Resume should succeed within ~2s of dock_trigger's "
            f'own heartbeat period once it exists, took {elapsed:.2f}s - no goal '
            f'was ever sent to it')


class TestEnabledProducersExcludesUndockRobot(_BaseCase):
    """
    D-13 test-status table row 2: a producer excluded by name from
    enabled_producers must not block release, no matter what its ledger
    state is - proven here by never starting dock_trigger at all, so
    undock_robot's ledger stays UNKNOWN for the entire test.
    """

    @classmethod
    def setUpClass(cls):
        # Global CLI parameter override, same pattern SimClockPostMuxGate
        # already uses in test_safety_stop_gate_node.py for use_sim_time -
        # SafetyStopGateNode's own __init__ takes no kwargs to forward one
        # through directly.
        rclpy.init(args=[
            '--ros-args', '-p',
            f"enabled_producers:=[{','.join(_LEGACY_ACTIONS)}]",
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode()
        self.gate._start -= Duration(seconds=T_READY + 1.0)
        self.probe = rclpy.create_node('d13_exclusion_probe')
        self._dock_proc = None  # deliberately never started

    def tearDown(self):
        self.gate.destroy_node()
        self.probe.destroy_node()

    def test_release_succeeds_with_undock_robot_excluded_despite_no_ledger_contact_ever(self):
        self.assertEqual(self.gate._enabled_producers, set(_LEGACY_ACTIONS))
        self._serve_legacy_actions_clear()

        dock_pub = self.probe.create_publisher(String, 'dock_trigger_status', 10)
        deadline = time.monotonic() + 2.0
        while dock_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self._spin_both(0.02)
        dock_pub.publish(String(data='idle'))
        dock_pub.destroy()

        def _abort_cb(request, response):
            del request
            response.success = True
            response.message = ''
            return response
        self.probe.create_service(Trigger, '/dock_trigger/abort', _abort_cb)
        self._spin_both(0.3)

        result = self._try_release()
        self.assertTrue(
            result.success,
            f'excluded producer undock_robot still blocked release: {result.message}')
