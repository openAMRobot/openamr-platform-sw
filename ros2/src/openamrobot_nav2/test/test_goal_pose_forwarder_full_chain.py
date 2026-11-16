"""
Live proof that /goal_pose reaches the PUBLIC navigate_to_pose action, not bt_navigator's
internal self_client_ (platform-sw#90/#124, Alex's 14 Nov owner-decision review on head
162a19a, following Nikolai's 14 Nov Gazebo reproduction of the defect this replaces).

Real navigation_launch.py (bt_navigator, the real safety_stop_gate_node, twist_mux, both goal
proxies - the full integrated stack) and a real goal_pose_forwarder_node subprocess - exactly
what the nav-only profile launches today (goal_relay.launch.py). A static TF tree
(map->odom->base_link) is enough for bt_navigator to reach ACTIVE and accept a goal (confirmed
by direct experiment: a goal sent this way is accepted and visible on the public action's own
status, then ABORTS for lack of a real costmap/map - that abort is expected and not what any
test here checks for). No Gazebo, map, localization or real motion is involved or claimed -
these tests prove ACTION ROUTING (does the public status/cancel pair see the goal at all), the
literal defect Nikolai's reproduction found; the motion/arbitration side (does a nonzero
command actually reach the real /cmd_vel) is already covered by
test_gate_crash_full_chain_cmd_vel_bound.py and the D-13 clean-boot tests.

The whole stack (TF, navigation_launch.py, the forwarder) comes up ONCE in setUpClass and
stays up for all 5 tests, not once per test - found necessary on hosted CI, not assumed:
restarting the full stack per test (the original design) made later tests in the same job
progressively flakier ("the first goal never appeared" after an already-successful
_wait_for_ready(), on a DIFFERENT test each run) as the job accumulated five full subprocess
tree churns - cumulative resource/discovery cost on a more constrained runner than local, not
a logic bug in any single test. Each test's own setUp instead re-engages Stop first, so every
test still starts from the same known LOCKED baseline regardless of what the previous test left
behind, and clears the accumulated status-sample buffer so assertions never see a prior test's
goal IDs.

Stop/Resume uses the REAL safety_stop_gate_node's own /safety/stop_lock_release and
/safety/stop_lock_engage services - not a faked /safety/stop_state, which was tried first and
does not work: navigation_launch.py always launches the real gate as part of its own stack
(confirmed directly - "ros2 launch navigation_launch.py" starts safety_stop_gate_node too), and
that real gate publishes its own genuine state at TICK_HZ (20Hz, always LOCKED since nothing
ever released it) on the exact same topic a fake publisher would use, racing it - the real
gate's publishes simply win most of the time, which showed up directly as the forwarder logging
"state=0" moments after a fake "RUNNING" publish. launch_arguments=enabled_producers:
navigate_to_pose,navigate_through_poses (same as bringup.launch.py's own nav-only profile) means
no dock_robot/undock_robot fakes are needed to call release for real - confirmed by direct
experiment that dock_trigger_status/abort are not required once those two are excluded.

Release needs two calls, not one, confirmed by direct experiment, not assumed: the gate's own
_status_publisher_matched_at tracking for navigate_to_pose/navigate_through_poses is lazily set
inside _release_blockers() itself (called synchronously from the release service callback), not
on a continuous background tick - so the FIRST release call, however long after boot, is the
call that first discovers bt_navigator's status publisher and starts its own 0.5s discovery-
grace window at that exact instant, and is refused; a SECOND call a moment later succeeds, now
that the window has genuinely elapsed since the first call set it. _release() below retries for
exactly this reason - and since the stack now stays up across tests, only the very first test
to call it actually pays that two-call cost; later tests' own first call already lands well
past the grace window.

Needs the real ros2 CLI AND tf2_ros (static_transform_publisher) installed, plus
openamr_nav_msgs built (StopState) - skipped everywhere else, same convention as the rest of
this test package. Run for real with:

    colcon build --packages-select openamr_nav_msgs openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v -s \
        src/openamrobot_nav2/test/test_goal_pose_forwarder_full_chain.py

New, unreviewed test infrastructure (subprocess-managed, real gate/mux/bt_navigator/forwarder,
plus static TF) - if it is flaky in your environment, that is useful information on its own,
not just noise to retry past.
"""
import os
import shutil
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90/#124


def _pkg_prefix_available(pkg):
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', pkg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


pytestmark = pytest.mark.skipif(
    not _pkg_prefix_available('tf2_ros'),
    reason='needs the real ros2 CLI and tf2_ros installed [#90/#124]')

from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402,I100
from action_msgs.srv import CancelGoal  # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402

from geometry_msgs.msg import PoseStamped  # noqa: E402

from lifecycle_msgs.srv import GetState  # noqa: E402

from std_srvs.srv import Trigger  # noqa: E402


def _start_new_session(args):
    return subprocess.Popen(
        args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def _stop_group(proc, timeout_s=5.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            pass


class TestGoalPoseForwarderFullChain(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.probe = rclpy.create_node('goal_pose_forwarder_probe')

        # Static TF: confirmed by direct experiment (module docstring) - enough for
        # bt_navigator to reach ACTIVE and accept a goal, nothing more claimed.
        cls._tf_procs = [
            _start_new_session(
                ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'odom']),
            _start_new_session(
                ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'odom', 'base_link']),
        ]

        # enabled_producers matches bringup.launch.py's own nav-only profile - no
        # dock_robot/undock_robot, so no dock_trigger_status/abort fakes are needed to
        # call release for real (confirmed by direct experiment).
        share = get_package_share_directory('openamrobot_nav2')
        launch_file = os.path.join(share, 'launch', 'navigation_launch.py')
        cls._nav_proc = _start_new_session(
            ['ros2', 'launch', launch_file, 'use_sim_time:=false',
             'enabled_producers:=navigate_to_pose,navigate_through_poses'])

        cls._forwarder_proc = _start_new_session(
            ['ros2', 'run', 'openamrobot_nav2', 'goal_pose_forwarder_node'])

        cls._goal_pose_pub = cls.probe.create_publisher(PoseStamped, 'goal_pose', 10)

        cls._status_samples = []  # list[GoalStatusArray]
        cls.probe.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            cls._status_samples.append, 10)

        cls._class_wait_for_ready()

    @classmethod
    def tearDownClass(cls):
        _stop_group(cls._forwarder_proc)
        _stop_group(cls._nav_proc)
        for proc in cls._tf_procs:
            _stop_group(proc)
        cls.probe.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _class_spin_for(cls, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.probe, timeout_sec=0.02)

    @classmethod
    def _class_wait_for_ready(cls, timeout_s=40.0):
        end = time.monotonic() + timeout_s
        client = cls.probe.create_client(GetState, '/bt_navigator/get_state')
        while time.monotonic() < end:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                deadline = time.monotonic() + 3.0
                while not future.done() and time.monotonic() < deadline:
                    cls._class_spin_for(0.05)
                if future.done() and future.result().current_state.id == 3:  # ACTIVE
                    client.destroy()
                    break
            cls._class_spin_for(0.2)
        else:
            client.destroy()
            raise AssertionError(f'bt_navigator never reached ACTIVE within {timeout_s}s')

        # bt_navigator's own readiness says nothing about whether the probe has discovered
        # the forwarder node yet (a different, unrelated node pair) - wait for it by name,
        # then for its own goal_pose subscription to actually match: goal_pose is VOLATILE
        # (default QoS), so a publish before the match completes is lost for good, not
        # merely delayed (confirmed directly - a single early publish never reached the
        # forwarder in an earlier version of this test).
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            if 'goal_pose_forwarder' in cls.probe.get_node_names():
                break
            cls._class_spin_for(0.2)
        else:
            raise AssertionError('goal_pose_forwarder node never appeared in the graph')

        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            if cls.probe.get_subscriptions_info_by_topic('goal_pose'):
                return
            cls._class_spin_for(0.2)
        raise AssertionError("'goal_pose' never got a subscriber within 15.0s")

    def setUp(self):
        # Shared stack across all tests in this class (see module docstring) - start every
        # test from the same known LOCKED baseline regardless of what the previous test left
        # behind, and never let one test's own goal IDs leak into the next test's assertions.
        self._engage()
        self._spin_for(0.5)
        self._status_samples.clear()

    def _spin_for(self, seconds):
        self._class_spin_for(seconds)

    def _release(self, timeout_s=15.0):
        # Confirmed by direct experiment (module docstring): the first call primes the
        # gate's own discovery-grace tracking and is refused; a second call after it
        # genuinely succeeds. Retries on ANY refusal reason, not just the discovery-grace
        # one, since this is meant to be a generic "get the real gate released" helper.
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), 'the real gate never came up')
        end = time.monotonic() + timeout_s
        last_message = '(never attempted)'
        while time.monotonic() < end:
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                self._spin_for(0.05)
            self.assertTrue(future.done(), 'stop_lock_release call timed out')
            response = future.result()
            if response.success:
                client.destroy()
                return
            last_message = response.message
            self._spin_for(0.5)
        client.destroy()
        raise AssertionError(
            f'stop_lock_release never succeeded within {timeout_s}s - '
            f'last refusal: {last_message}')

    def _engage(self):
        client = self.probe.create_client(Trigger, '/safety/stop_lock_engage')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            self._spin_for(0.05)
        self.assertTrue(future.done(), 'stop_lock_engage call timed out')
        client.destroy()

    def _latest_status_list(self):
        return self._status_samples[-1].status_list if self._status_samples else []

    def _any_goal_ever_seen(self):
        return any(s.status_list for s in self._status_samples)

    def _wait_until(self, predicate, timeout_s, message):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return
            self._spin_for(0.1)
        raise AssertionError(message)

    def _publish_goal_pose(self, x):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = x
        self._goal_pose_pub.publish(pose)

    def test_goal_pose_reaches_public_action_and_cancel_works(self):
        # Nikolai's own 14 Nov repro, as a test: one /goal_pose, the public status lists the
        # goal, cancel_goal returns it (not cancelled_goals=0 against a server holding no such
        # goal), and the status eventually reflects the cancellation.
        self._release()

        self._publish_goal_pose(1.0)
        self._wait_until(
            lambda: bool(self._latest_status_list()), 15.0,
            'the goal never appeared on the public navigate_to_pose/_action/status - '
            'the forwarder did not relay goal_pose onto the public action')

        cancel_client = self.probe.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.assertTrue(cancel_client.wait_for_service(timeout_sec=5.0))
        future = cancel_client.call_async(CancelGoal.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            self._spin_for(0.05)
        self.assertTrue(future.done(), 'cancel_goal call never completed')
        response = future.result()
        cancel_client.destroy()
        self.assertGreaterEqual(
            len(response.goals_canceling), 1,
            'cancel_goal returned 0 goals canceling - it reached a server holding no such '
            "goal, exactly the defect this forwarder replaces (Nikolai's 14 Nov repro: "
            'cancelled_goals=0 while the goal kept running)')

        self._wait_until(
            lambda: any(
                s.status == GoalStatus.STATUS_CANCELING or s.status == GoalStatus.STATUS_CANCELED
                for s in self._latest_status_list()),
            15.0,
            'the public status never reflected the cancellation')

    def test_pose_rejected_while_stop_is_active_no_goal_ever_sent(self):
        # setUp already engaged (LOCKED) and cleared the sample buffer - never released here.
        self._publish_goal_pose(1.0)
        self._spin_for(3.0)

        self.assertFalse(
            self._any_goal_ever_seen(),
            'a goal appeared on the public action despite Stop being active - the forwarder '
            'must reject a goal_pose while locked, not send it')

    def test_stop_during_a_forwarded_goal_then_resume_sends_nothing_without_a_new_pose(self):
        self._release()

        self._publish_goal_pose(1.0)
        self._wait_until(
            lambda: bool(self._latest_status_list()), 15.0,
            'the first goal never appeared on the public action')
        goal_id_before = bytes(self._latest_status_list()[0].goal_info.goal_id.uuid)
        self._status_samples.clear()

        self._engage()
        self._release()  # Resume - no new goal_pose sent
        self._spin_for(3.0)

        # A GoalStatusArray message reports the server's FULL set of currently-tracked
        # goals at once, not just the newest one - confirmed directly: an earlier test's
        # own goal, only now finally reaching a terminal status (ABORTED/CANCELED/
        # SUCCEEDED) as the server's own housekeeping catches up, legitimately appears
        # alongside this test's own goal in the same message. That is normal action-server
        # behavior, not a replay - only an id OTHER than goal_id_before showing a non-
        # terminal, in-progress status (ACCEPTED/EXECUTING) would mean something actually
        # sent a new goal without a new goal_pose message, which is the real claim here.
        unexpected_active_ids = {
            bytes(s.goal_info.goal_id.uuid)
            for sample in self._status_samples for s in sample.status_list
            if bytes(s.goal_info.goal_id.uuid) != goal_id_before
            and s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)}
        self.assertEqual(
            unexpected_active_ids, set(),
            f'a goal other than {goal_id_before!r} became active after Resume '
            f'({unexpected_active_ids}) although no new goal_pose message was ever sent - '
            'the forwarder must not replay or auto-resend a goal on its own')

    def test_two_poses_in_a_row_exactly_one_active_goal_first_cancelled(self):
        self._release()

        self._publish_goal_pose(1.0)
        self._wait_until(
            lambda: bool(self._latest_status_list()), 15.0, 'the first goal never appeared')
        goal_a_id = bytes(self._latest_status_list()[0].goal_info.goal_id.uuid)
        # Clear here, not just in setUp - same reasoning as the Stop/Resume test above: a
        # straggling late status update from a previous test's own goal must never be
        # mistaken for "a distinct second goal is now active" below.
        self._status_samples.clear()

        self._publish_goal_pose(2.0)

        def _goal_a_reached_terminal():
            for sample in self._status_samples:
                for s in sample.status_list:
                    if bytes(s.goal_info.goal_id.uuid) == goal_a_id and s.status in (
                            GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                            GoalStatus.STATUS_SUCCEEDED):
                        return True
            return False

        self._wait_until(
            _goal_a_reached_terminal, 25.0,
            'the first goal (A) never reached a terminal status after a second goal_pose - '
            'the forwarder must cancel the previous goal before sending the new one')

        def _a_different_goal_is_now_active():
            # Scan every received sample, not just the latest one (same reasoning as the
            # Stop/Resume test above): an action server does not necessarily keep every
            # historical goal in every subsequent array, so goal B's own ACCEPTED/
            # EXECUTING entry could be pruned from later arrays before this predicate
            # happens to be checked against the one sample that had it.
            return any(
                bytes(s.goal_info.goal_id.uuid) != goal_a_id and
                s.status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED)
                for sample in self._status_samples for s in sample.status_list)

        self._wait_until(
            _a_different_goal_is_now_active, 20.0,
            'no distinct second goal (B) ever became active after the first was cancelled')

    def test_zero_publishers_on_bt_navigators_orphaned_goal_topic(self):
        # Alex's review: "a launch test fails if one appears" - this is that test.
        self._spin_for(1.0)
        infos = self.probe.get_publishers_info_by_topic('goal_pose_bt_navigator_unused')
        self.assertEqual(
            infos, [],
            f'goal_pose_bt_navigator_unused has a publisher ({infos}) - something is still '
            "feeding bt_navigator's own internal goal topic directly, bypassing the public "
            'action this forwarder exists to route through')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v', '-s']))
