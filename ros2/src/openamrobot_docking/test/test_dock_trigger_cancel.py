"""
Regression tests for making dock_trigger.py cancellable via Stop (software).

Alex's platform item 4: /dock_trigger is a plain std_msgs/Bool topic, not a
ROS 2 action, so there was no way to cancel an in-progress docking/undocking
sequence - Stop (software) only masked cmd_vel_dock at the gate, leaving the
Python state machine itself running, polling for conditions that can never be
satisfied once the robot can't move.

Design (see PROGRESS.md / the design proposal): DockTrigger subscribes to the
existing /safety/stop_lock topic directly - no new contract, no new topic.
When the lock engages while self.busy is True, a cancel flag is set; the
active phase loop is expected to check it and unwind via a DockCancelled
exception, which _run_and_release's existing try/except already catches, so
the existing cleanup (collision_monitor, AMCL, busy reset, status) runs
unchanged.

These tests exercise the subscription wiring and the cancel-flag lifecycle at
the node level - they do not exercise the phase loops themselves (those need
TF/camera/LiDAR and are out of scope for a unit test). Skipped if rclpy isn't
installed, run for real in a ROS 2 workspace.
"""
import importlib.util
import os
import time
import unittest
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: platform item 4

from std_msgs.msg import Bool  # noqa: E402,I100
from std_srvs.srv import Trigger  # noqa: E402,I100

_DOCK_TRIGGER_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'dock_trigger.py')


def _load_dock_trigger_module():
    """
    Load dock_trigger.py as a module.

    openamrobot_docking is a plain ament_cmake package (dock_trigger.py is an
    installed script, not an importable ament_python module), so it is loaded
    directly from its source path - same pattern as
    openamrobot_bringup/test/test_real_bringup_launch.py uses for a launch
    file.
    """
    spec = importlib.util.spec_from_file_location('dock_trigger', _DOCK_TRIGGER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.dock_trigger_module = _load_dock_trigger_module()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = self.dock_trigger_module.DockTrigger()
        self.probe = rclpy.create_node('dock_trigger_cancel_probe')

    def tearDown(self):
        self.node.destroy_node()
        self.probe.destroy_node()

    def _spin_both(self, timeout_s=0.5):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _publish_lock(self, engaged: bool):
        pub = self.probe.create_publisher(Bool, '/safety/stop_lock', 10)
        deadline = time.monotonic() + 1.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(
            pub.get_subscription_count(), 0,
            'dock_trigger never subscribed to /safety/stop_lock')
        pub.publish(Bool(data=engaged))
        pub.destroy()

    def _call_abort(self):
        # platform-sw#119: the gate calls this directly - test the same way,
        # not through the /safety/stop_lock subscription.
        cli = self.probe.create_client(Trigger, '/dock_trigger/abort')
        deadline = time.monotonic() + 2.0
        while not cli.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(cli.service_is_ready(), 'dock_trigger never served ~/abort')
        future = cli.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(future.done(), 'abort call never completed')
        cli.destroy()
        return future.result()


class TestCancelFlagLifecycle(NodeTestCase):

    def test_lock_engaging_while_idle_does_not_set_cancel(self):
        # busy=False at construction (no sequence running) - engaging the
        # lock here is normal startup/idle behaviour, not a mid-sequence
        # abort, so it must not set a stale cancel flag that would
        # immediately abort the very next triggered sequence.
        self.assertFalse(self.node.busy)
        self._publish_lock(True)
        self._spin_both(timeout_s=0.3)
        self.assertFalse(
            getattr(self.node, '_cancel_requested', False),
            'engaging the lock while idle (busy=False) must not set the '
            'cancel flag - platform item 4')

    def test_lock_engaging_while_busy_sets_cancel(self):
        self.node.busy = True
        self._publish_lock(True)
        self._spin_both(timeout_s=0.3)
        self.assertTrue(
            getattr(self.node, '_cancel_requested', False),
            'engaging /safety/stop_lock while a docking/undocking sequence '
            'is active (busy=True) must set a cancel flag the active phase '
            'loop can check (platform item 4)')

    def test_lock_releasing_does_not_clear_an_existing_cancel(self):
        # Once a sequence has been told to cancel, a release must not quietly
        # un-cancel it mid-unwind - the sequence should finish unwinding to
        # idle, not resume. The NEXT trigger (on_trigger/on_undock) is what
        # clears the flag for a fresh sequence, not a bare lock release.
        self.node.busy = True
        self._publish_lock(True)
        self._spin_both(timeout_s=0.2)
        self.assertTrue(getattr(self.node, '_cancel_requested', False))
        self._publish_lock(False)
        self._spin_both(timeout_s=0.2)
        self.assertTrue(
            getattr(self.node, '_cancel_requested', False),
            'releasing the lock must not clear an in-flight cancel request - '
            'only starting a NEW sequence should reset it')


class TestCancelFlagResetsOnNewTrigger(NodeTestCase):

    def test_on_trigger_resets_a_stale_cancel_flag(self):
        # Simulate a previous sequence that was cancelled and has since
        # unwound back to idle (busy=False), leaving _cancel_requested=True
        # behind it. The next /dock_trigger=true must start a clean
        # sequence, not one that is already pre-cancelled.
        #
        # threading.Thread is patched out so the real sequence never
        # actually starts - it would otherwise block on a live Nav2 action
        # server this unit test does not have, as a daemon thread outliving
        # this test. Only the synchronous part of on_trigger (flag reset,
        # setting busy) is under test here.
        self.node._cancel_requested = True
        self.node.busy = False
        with patch('threading.Thread'):
            self.node.on_trigger(Bool(data=True))
        self.assertFalse(
            getattr(self.node, '_cancel_requested', True),
            'on_trigger must reset a stale cancel flag before starting a '
            'new sequence, or the new sequence would abort immediately')

    def test_on_undock_resets_a_stale_cancel_flag(self):
        self.node._cancel_requested = True
        self.node.busy = False
        self.node.is_docked = True
        with patch('threading.Thread'):
            self.node.on_undock(Bool(data=True))
        self.assertFalse(
            getattr(self.node, '_cancel_requested', True),
            'on_undock must reset a stale cancel flag before starting a '
            'new sequence')


class TestAbortService(NodeTestCase):

    def test_abort_sets_the_cancel_flag_while_busy(self):
        self.node.busy = True
        result = self._call_abort()
        self.assertTrue(result.success)
        self.assertTrue(
            getattr(self.node, '_cancel_requested', False),
            '~/abort must set the same cancel flag the loops check, same as '
            'the /safety/stop_lock subscription (platform-sw#119)')

    def test_abort_succeeds_as_a_no_op_while_idle(self):
        # The gate calls this on every tick while locked (#119), including
        # when no sequence is running at all - must not error.
        self.assertFalse(self.node.busy)
        result = self._call_abort()
        self.assertTrue(result.success)

    def test_repeated_abort_calls_while_locked_stay_harmless(self):
        # #119: "repeated on each tick while locked" - idempotent, no error
        # on repetition, flag stays set.
        self.node.busy = True
        for _ in range(3):
            result = self._call_abort()
            self.assertTrue(result.success)
        self.assertTrue(getattr(self.node, '_cancel_requested', False))


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
