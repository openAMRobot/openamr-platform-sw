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

from geometry_msgs.msg import PoseStamped  # noqa: E402,I100
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

    def test_on_undock_no_longer_resets_a_stale_cancel_flag_because_it_is_refused(self):
        # Superseded by D-19 (platform-sw#124, platform-sw#78): on_undock no
        # longer starts a sequence at all while reverse/automatic undock is
        # disabled for Build 1, docked or not - see TestD19ReverseAndAutoUndock
        # Disabled below for the full set of required negative regressions.
        # This test only confirms the refusal happens BEFORE the old
        # cancel-flag-reset/thread-start code would have run; restore a
        # positive "resets the flag" test alongside whatever re-enables D-19.
        self.node._cancel_requested = True
        self.node.busy = False
        self.node.is_docked = True
        with patch('threading.Thread') as thread_cls:
            self.node.on_undock(Bool(data=True))
            thread_cls.assert_not_called()
        self.assertTrue(
            getattr(self.node, '_cancel_requested', False),
            'on_undock is refused under D-19 before it would have reset the '
            'cancel flag or started anything')


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


# #119 test 7 ("a real dock_trigger sequence stopped in each motion phase"):
# the phase loops themselves need TF/camera/LiDAR - a real ROS graph this
# file does not have, same boundary the module docstring above already
# draws. What CAN be verified here, honestly, without that graph: every one
# of the sequence's blocking loops actually contains the cancellation check
# - a static, structural guarantee that complements the dynamic flag-
# lifecycle tests above (TestCancelFlagLifecycle proves the mechanism works;
# this proves every loop that needs it actually calls it). The two together
# are the closest honest substitute for "stopped in each motion phase"
# available without a real sim.
_BLOCKING_LOOP_METHODS = (
    '_estimate_dock',
    '_final_visual_approach',
    '_reverse_distance',
    '_search_for_tag',
    'get_tag_pose_filtered',
    '_spin_to_yaw',
    '_spin_angle_odom',
    '_drive_to_xy',
    '_wait_for_path_clear',
    '_send_action_blocking',
)


class TestEveryBlockingLoopChecksCancellation(unittest.TestCase):
    """
    Static, source-level regression (platform-sw#119 test 7's stand-in).

    Parses dock_trigger.py's AST and confirms each name in
    _BLOCKING_LOOP_METHODS calls self._raise_if_cancelled() somewhere in its
    body - a call anywhere inside the method (including nested blocks),
    never just a literal grep hit that could land in an unrelated method by
    line-number coincidence. Does not require rclpy or a ROS graph.
    """

    @classmethod
    def setUpClass(cls):
        import ast
        with open(_DOCK_TRIGGER_PATH, encoding='utf-8') as f:
            cls._tree = ast.parse(f.read(), filename=_DOCK_TRIGGER_PATH)
        cls._methods_by_name = {}
        for node in ast.walk(cls._tree):
            if isinstance(node, ast.ClassDef) and node.name == 'DockTrigger':
                for item in node.body:
                    if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        cls._methods_by_name[item.name] = item

    def _calls_raise_if_cancelled(self, func_node) -> bool:
        import ast
        for node in ast.walk(func_node):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != '_raise_if_cancelled':
                continue
            if isinstance(node.func.value, ast.Name) and node.func.value.id == 'self':
                return True
        return False

    def test_every_tracked_method_exists(self):
        # Guards against this list silently going stale if a method is
        # ever renamed or removed - the methods below would otherwise
        # pass vacuously (KeyError never raised, just never checked).
        missing = [
            name for name in _BLOCKING_LOOP_METHODS
            if name not in self._methods_by_name]
        self.assertEqual(
            missing, [],
            f'tracked method(s) no longer exist on DockTrigger: {missing} - '
            f'update _BLOCKING_LOOP_METHODS, do not just let this go stale')

    def test_every_tracked_method_checks_cancellation(self):
        tracked = [name for name in _BLOCKING_LOOP_METHODS if name in self._methods_by_name]
        not_checked = [
            name for name in tracked
            if not self._calls_raise_if_cancelled(self._methods_by_name[name])]
        self.assertEqual(
            not_checked, [],
            f'{not_checked} no longer call self._raise_if_cancelled() - a '
            f'Stop (software) during that phase would no longer cancel the '
            f'sequence (platform-sw#119 test 7)')


class TestD19ReverseAndAutoUndockDisabled(NodeTestCase):
    """
    D-19 (platform-sw#124, Alex's 8 Nov ruling, P-03 rev20.6 item 30 A2):
    reverse and automatic undock are disabled for Build 1, for EVERY caller,
    until #78's cancellable, docked-only, rear-checked undock action is
    accepted - panthera-momagdii's #90 review (id 5823873895), reproduced
    and fixed here.

    Required negative regressions per that review: (a) a docked /goal_pose
    never initiates an undock/reverse; (b) an undock request while not
    docked is rejected without motion. This repository's own decision
    (asked and confirmed, not assumed) extends that to EVERY undock
    request, docked or not - reverse itself is disabled, not just the
    automatic path - until #78 lands.
    """

    def test_goal_pose_while_docked_does_not_auto_undock_or_forward(self):
        self.node.is_docked = True
        with patch('threading.Thread') as thread_cls:
            self.node.on_goal_pose(PoseStamped())
            thread_cls.assert_not_called()
        self.assertFalse(self.node.busy)

    def test_undock_request_while_not_docked_is_refused_without_motion(self):
        self.node.is_docked = False
        with patch('threading.Thread') as thread_cls:
            self.node.on_undock(Bool(data=True))
            thread_cls.assert_not_called()
        self.assertFalse(self.node.busy)

    def test_undock_request_while_docked_is_also_refused_without_motion(self):
        # The broader of the two decisions asked and confirmed above:
        # reverse itself is disabled, so even a genuinely docked, explicit
        # undock request must not move the robot until #78 lands.
        self.node.is_docked = True
        with patch('threading.Thread') as thread_cls:
            self.node.on_undock(Bool(data=True))
            thread_cls.assert_not_called()
        self.assertFalse(self.node.busy)

    def test_run_undock_sequence_itself_refuses_as_the_single_chokepoint(self):
        # Defense in depth: whatever eventually calls run_undock_sequence()
        # directly (any caller, present or future) must still get refused
        # here, not just at on_undock/on_goal_pose's own early checks.
        with patch.object(self.node, '_reverse_distance') as reverse_mock:
            result = self.node.run_undock_sequence()
        reverse_mock.assert_not_called()
        self.assertFalse(result)

    def test_any_deliberate_re_enablement_is_a_code_change_not_a_parameter(self):
        # (c) of the review's required set: re-enablement must stay gated on
        # #78's owner acceptance, not something an operator can flip at
        # launch time. Asserting the module constant (not a declared ROS
        # parameter) is the gate is itself part of that guarantee.
        self.assertFalse(
            self.node.has_parameter('build1_reverse_and_auto_undock_disabled'),
            'D-19 must stay a code constant, not a launch/runtime parameter - '
            'an operator could otherwise re-enable reverse without #78')
        self.assertTrue(self.dock_trigger_module.BUILD1_REVERSE_AND_AUTO_UNDOCK_DISABLED)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
