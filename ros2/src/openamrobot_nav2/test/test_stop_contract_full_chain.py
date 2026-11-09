"""
End-to-end regression tests for the platform-sw#119 stop-path contract
(Alex's Option B decision, 5 Nov 2026): real twist_mux + real gate, with fake
action/dock-trigger servers standing in for the 4 real Nav2/docking actions -
no real Nav2 or opennav_docking graph exists in this workspace. Covers tests
1-6 and 8 from #119's own list.

Test 7 ("a real dock_trigger sequence stopped in each motion phase") is NOT
here: dock_trigger's phase loops need TF/camera/LiDAR, out of reach for a
test file like this one - see test_dock_trigger_cancel.py's own docstring,
which already owns that scope (the cancel-flag/abort-service plumbing at the
node level) and is where that coverage extends instead.

No setting _rearmed (removed entirely by #119 - the attribute no longer
exists) and no injecting a synthetic zero to fake rearm. Every scenario here
drives the REAL release-refusal gate through real cmd_vel_nav/cmd_vel_dock/
cmd_vel_joy traffic over the real mux and real (faked) action-server status -
the exact requirement Alex's/Mohamed's review set for this PR's remaining
tests.
"""
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #119


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
    reason='needs the real ros2 CLI AND the twist_mux package installed [#119]')

from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402,I100
from action_msgs.srv import CancelGoal  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from openamr_nav_msgs.msg import StopState  # noqa: E402
from openamrobot_nav2.safety_stop_gate_node import (  # noqa: E402
    _CANCEL_ACTIONS, SafetyStopGateNode, T_READY)
from rclpy.duration import Duration  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'
_NAV_TIMEOUT_S = 0.5    # cmd_vel_nav, config/twist_mux.yaml
_JOY_TIMEOUT_S = 0.25   # cmd_vel_joy, config/twist_mux.yaml


def _start_mux():
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux',
         '--ros-args', '--params-file', str(_SHIPPED_PARAMS_PATH),
         '-r', 'cmd_vel_out:=cmd_vel_mux'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop_mux(proc):
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


class StopContractFullChainCase(unittest.TestCase):
    """Real twist_mux + real gate; fake action/dock-trigger servers."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyStopGateNode()
        # Real wall clock here (no simulated /clock, same as
        # test_docking_full_chain_through_real_mux.py) - backdate _start so
        # T_READY is never what these tests are actually exercising; that is
        # test_safety_stop_gate_node.py's own TestReleaseReadinessIsRelativeToStartup.
        self.gate._start -= Duration(seconds=T_READY + 1.0)
        self.probe = rclpy.create_node('stop_contract_full_chain_probe')
        self._mux_proc = _start_mux()

        self._nav_pub = self.probe.create_publisher(Twist, 'cmd_vel_nav', 10)
        self._dock_pub = self.probe.create_publisher(Twist, 'cmd_vel_dock', 10)
        self._joy_pub = self.probe.create_publisher(Twist, 'cmd_vel_joy', 10)
        self._gated = []
        self.probe.create_subscription(Twist, 'cmd_vel_gated', self._gated.append, 10)
        self._states = []
        self.probe.create_subscription(
            StopState, '/safety/stop_state', self._states.append, 10)

        self._goal_status_pubs = {}
        self._cancel_calls = {action: [] for action in _CANCEL_ACTIONS}
        # 'accept': the cancel service call succeeds, but the goal's status
        # does NOT instantly clear - a real action server takes a moment (or
        # may never) to actually stop, which is exactly what tests 1/3/6 need
        # to prove release waits for. 'accept_and_clear': the service call
        # succeeds AND immediately reports the goal gone, simulating a
        # server that has already honoured it (tests 2/4). 'reject': the
        # service call itself fails (test 5).
        self._cancel_responses = {action: 'accept' for action in _CANCEL_ACTIONS}
        self._current_goal_status = {action: [] for action in _CANCEL_ACTIONS}
        self._cancel_services = {}
        for action in _CANCEL_ACTIONS:
            self._goal_status_pubs[action] = self.probe.create_publisher(
                GoalStatusArray, f'{action}/_action/status', 10)
            self._cancel_services[action] = self.probe.create_service(
                CancelGoal, f'{action}/_action/cancel_goal', self._make_cancel_cb(action))

        self._dock_status_pub = self.probe.create_publisher(String, 'dock_trigger_status', 10)
        self._abort_calls = []
        self._dock_abort_service = self.probe.create_service(
            Trigger, '/dock_trigger/abort', self._on_abort)

        # D-13 (platform-sw#124): undock_robot is governed by the
        # goal-authority ledger, not the legacy status topic - a heartbeat
        # is what establishes IDLE for it, same as _set_goal_status_all_clear
        # does for the other three actions below.
        self._goal_authority_pub = self.probe.create_publisher(
            String, 'goal_authority_events', 10)
        self._goal_authority_instance_id = 'test-instance'

        self._spin_for(0.5)  # let every client/subscriber above discover its match
        self._set_goal_status_all_clear()
        self._set_dock_status('idle')
        self._publish_goal_authority_heartbeat()
        self._spin_for(0.2)

    def tearDown(self):
        _stop_mux(self._mux_proc)
        self.gate.destroy_node()
        self.probe.destroy_node()

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.gate, timeout_sec=0.01)
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _make_cancel_cb(self, action):
        def _cb(request, response):
            self._cancel_calls[action].append(request)
            outcome = self._cancel_responses[action]
            if outcome == 'reject':
                response.return_code = CancelGoal.Response.ERROR_REJECTED
                return response
            response.return_code = CancelGoal.Response.ERROR_NONE
            if outcome == 'accept_and_clear':
                self._set_goal_status(action, [])
            # 'accept': service call succeeds, status deliberately left as-is -
            # the goal has not actually stopped yet.
            return response
        return _cb

    def _on_abort(self, request, response):
        del request
        self._abort_calls.append(True)
        response.success = True
        response.message = ''
        return response

    def _set_goal_status(self, action, statuses, _track=True):
        if _track:
            self._current_goal_status[action] = list(statuses)
        msg = GoalStatusArray()
        for s in statuses:
            gs = GoalStatus()
            gs.status = s
            msg.status_list.append(gs)
        self._goal_status_pubs[action].publish(msg)

    def _set_goal_status_all_clear(self):
        for action in _CANCEL_ACTIONS:
            self._set_goal_status(action, [])

    def _publish_goal_authority_heartbeat(self, producer='undock_robot'):
        self._goal_authority_pub.publish(
            String(data=f'{producer}|heartbeat|{self._goal_authority_instance_id}'))

    def _set_dock_status(self, status):
        self._dock_status = status
        self._dock_status_pub.publish(String(data=status))

    def _refresh_before_release(self, skip_actions=frozenset(), skip_dock=False):
        """
        Republish every tracked status right before a release attempt.

        Content, not message age, is what the gate judges now (platform-
        sw#90, Mohamed's review: the old STATUS_FRESH_S time window refused
        release on a healthy, idle system whose status simply had not
        changed recently - ROS 2 action status only publishes on a
        transition, not as a heartbeat). This just keeps a test's own
        explicit _set_goal_status/_set_dock_status calls as the only thing
        that actually changes behaviour. skip_actions lets a test (e.g.
        test 5's "server gone") deliberately leave one action's status
        untouched since construction; skip_dock does the same for dock_
        trigger_status.
        """
        if not skip_dock:
            self._set_dock_status(getattr(self, '_dock_status', 'idle'))
        for action in _CANCEL_ACTIONS:
            if action not in skip_actions:
                self._set_goal_status(action, self._current_goal_status[action])
        if 'undock_robot' not in skip_actions:
            self._publish_goal_authority_heartbeat()
        self._spin_for(0.1)

    def _take_down_cancel_service(self, action):
        """Simulate that action's server having crashed/gone away entirely."""
        self.probe.destroy_service(self._cancel_services[action])
        self._spin_for(0.2)  # let discovery catch up with the removal

    def _take_down_dock_abort_service(self):
        self.probe.destroy_service(self._dock_abort_service)
        self._spin_for(0.2)

    def _try_release(self, skip_actions=frozenset(), skip_dock=False):
        self._refresh_before_release(skip_actions=skip_actions, skip_dock=skip_dock)
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self._spin_for(0.02)
        self.assertTrue(future.done(), 'release call timed out')
        client.destroy()
        return future.result()

    def _engage(self):
        client = self.probe.create_client(Trigger, '/safety/stop_lock_engage')
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self._spin_for(0.02)
        self.assertTrue(future.done(), 'engage call timed out')
        client.destroy()
        return future.result()

    def _nonzero_gated(self):
        return [m for m in self._gated if m != Twist()]

    def _nonzero_twist(self):
        t = Twist()
        t.linear.x = 0.2
        return t


class TestContinuousStaleNavBlocksRelease(StopContractFullChainCase):
    """#119 test 1: continuous stale nav through engage and release - release
    refused while the goal is active, no motion."""

    def test_release_refused_and_no_motion_while_nav_goal_stays_active(self):
        self._set_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_for(0.2)
        self._engage()

        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self._nav_pub.publish(self._nonzero_twist())
            self._spin_for(0.02)

        response = self._try_release()
        self.assertFalse(response.success)
        self.assertIn('navigate_to_pose', response.message)

        self._gated.clear()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self._nav_pub.publish(self._nonzero_twist())
            self._spin_for(0.02)
        self.assertEqual(
            self._nonzero_gated(), [],
            'motion reached cmd_vel_gated although release was refused')


class TestSilentSourceReleasesOnceGoalClears(StopContractFullChainCase):
    """#119 test 2: source goes silent without a final zero - release
    accepted once (a)-(c) hold; a new goal moves, no node restart."""

    def test_release_accepted_after_silence_and_new_goal_moves(self):
        self._set_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_for(0.1)
        self._engage()
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self._nav_pub.publish(self._nonzero_twist())
            self._spin_for(0.02)

        # The source goes SILENT - no final zero, just stops publishing - and
        # the (faked) action server reports the goal is actually gone.
        self._set_goal_status('navigate_to_pose', [])
        # Let cmd_vel_nav's own timeout lapse with no new publishes.
        self._spin_for(_NAV_TIMEOUT_S + 0.3)

        response = self._try_release()
        self.assertTrue(response.success, response.message)
        self.assertFalse(self.gate._locked)

        # A fresh goal/command, same gate instance - no restart anywhere.
        self._gated.clear()
        end = time.monotonic() + 1.0
        found = False
        while time.monotonic() < end and not found:
            self._nav_pub.publish(self._nonzero_twist())
            self._spin_for(0.02)
            found = any(m != Twist() for m in self._gated)
        self.assertTrue(found, 'a fresh command never reached cmd_vel_gated after release')


class TestManualZeroDoesNotRearmOverAnActiveGoal(StopContractFullChainCase):
    """#119 test 3: manual zero while an old nav goal is active - no motion."""

    def test_no_motion_when_cmd_vel_joy_is_zero_but_nav_goal_is_active(self):
        self._set_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_for(0.2)
        self._engage()

        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self._joy_pub.publish(Twist())  # manual zero, priority 100 (highest)
            self._spin_for(0.02)

        response = self._try_release()
        self.assertFalse(
            response.success,
            'a manual zero on cmd_vel_joy released the gate while '
            'navigate_to_pose was still reporting an active goal')
        self.assertIn('navigate_to_pose', response.message)


class TestOldGoalStaysGoneAfterPauseAndResume(StopContractFullChainCase):
    """#119 test 4: an old goal that pauses then resumes after release - it
    was actually cancelled and is gone, so its resuming traffic is just a
    fresh, unattributed command, not a surviving authority."""

    def test_old_goal_is_cancelled_and_release_succeeds(self):
        # The fake server honours this cancel and reports the goal gone -
        # the realistic case this test targets (contrast with test 1/3/6,
        # where the default 'accept' leaves status untouched, standing in
        # for a goal that has not actually stopped yet).
        self._cancel_responses['navigate_to_pose'] = 'accept_and_clear'
        self._set_goal_status('navigate_to_pose', [GoalStatus.STATUS_EXECUTING])
        self._spin_for(0.1)
        self._engage()
        self._spin_for(0.3)
        self.assertTrue(
            self._cancel_calls['navigate_to_pose'],
            'engage never called cancel_goal on navigate_to_pose')
        self.assertEqual(
            self._current_goal_status['navigate_to_pose'], [],
            'the fake server never actually cleared the goal status')

        # The old goal pauses (the stream stops).
        self._spin_for(_NAV_TIMEOUT_S + 0.3)
        response = self._try_release()
        self.assertTrue(response.success, response.message)

        # The old goal's stream "resumes" - but it was genuinely cancelled at
        # the action level, so this is indistinguishable from a fresh command
        # and is allowed through, same as any other post-release traffic.
        self._gated.clear()
        end = time.monotonic() + 1.0
        found = False
        while time.monotonic() < end and not found:
            self._nav_pub.publish(self._nonzero_twist())
            self._spin_for(0.02)
            found = any(m != Twist() for m in self._gated)
        self.assertTrue(found)


class TestCancelFailureKeepsReleaseRefused(StopContractFullChainCase):
    """#119 test 5: cancel rejected, timed out, or server gone - release
    stays refused with a reason."""

    def test_release_refused_when_the_cancel_is_rejected(self):
        self._cancel_responses['dock_robot'] = 'reject'  # reject every cancel
        self._set_goal_status('dock_robot', [GoalStatus.STATUS_EXECUTING])
        self._spin_for(0.1)
        self._engage()
        self._spin_for(0.3)
        self.assertTrue(self._cancel_calls['dock_robot'], 'cancel_goal was never called')
        self.assertEqual(
            self._current_goal_status['dock_robot'], [GoalStatus.STATUS_EXECUTING],
            'a rejected cancel must not have cleared the goal status')

        response = self._try_release()
        self.assertFalse(response.success)
        self.assertIn('dock_robot', response.message)

    @pytest.mark.xfail(
        strict=True,
        reason=(
            'known gap, contract decision pending: service_is_ready() does not prove a '
            "cancel was accepted, so a rejected cancel against an already-'clear' status "
            'is invisible - tracking: #122'))
    def test_release_refused_when_the_cancel_is_rejected_and_status_was_already_clear(self):
        # Mohamed's 7 Nov 05:23 counterexample to the P1 fix: if the LAST
        # content we ever saw for an action was already empty (no goal ever
        # reported active - e.g. a goal accepted a moment ago has not yet
        # published its first status), a rejected cancel is invisible to a
        # purely content-based check. service_is_ready() alone does not
        # prove the cancel took.
        self._cancel_responses['navigate_to_pose'] = 'reject'
        self._set_goal_status('navigate_to_pose', [])  # already "clear" before engage
        self._spin_for(0.1)
        self._engage()
        self._spin_for(0.3)
        self.assertTrue(
            self._cancel_calls['navigate_to_pose'], 'cancel_goal was never called')

        response = self._try_release()
        self.assertFalse(
            response.success,
            "release succeeded despite navigate_to_pose's cancel_goal having been "
            'rejected - content-plus-liveness alone does not prove the cancel took '
            "(platform-sw#90, Mohamed's review, 7 Nov 05:23)")
        self.assertIn('navigate_to_pose', response.message)

    def test_release_refused_when_the_action_server_is_gone(self):
        # The last content we ever got from navigate_through_poses was
        # setUp's all-clear sample - by content alone that would satisfy
        # condition (a). What must still block release is that its
        # cancel_goal service is no longer reachable at all: the same,
        # deliberately undifferentiated "status stale or never received"
        # reason this gate gives for a server that never answered in the
        # first place. navigate_through_poses, not undock_robot: undock_robot
        # moved to the goal-authority ledger (D-13, platform-sw#124), which
        # does not consult this client at all, so this legacy mechanism no
        # longer applies to it.
        self._take_down_cancel_service('navigate_through_poses')
        response = self._try_release(skip_actions={'navigate_through_poses'})
        self.assertFalse(response.success)
        self.assertIn('navigate_through_poses', response.message)

    def test_release_refused_when_dock_trigger_is_gone(self):
        # Same reasoning as the action case above, for the dock_trigger
        # abort service instead of a cancel_goal client.
        self._take_down_dock_abort_service()
        response = self._try_release(skip_dock=True)
        self.assertFalse(response.success)
        self.assertIn('dock_trigger', response.message)


class TestReleaseSucceedsOnAHealthyIdleSystemDespiteStaleStatus(StopContractFullChainCase):
    """
    Covers platform-sw#90, Mohamed's review (P1).

    A healthy idle system must not be unable to Resume indefinitely. ROS 2
    action status only publishes on a transition, not as a periodic
    heartbeat, so an idle server's last sample ages past any fixed
    freshness window with nothing wrong. Release must still succeed once
    its cancel_goal service (or, for docking, the abort service) is
    confirmed reachable right now - content, not message age, is what
    actually proves no old goal has authority.
    """

    def test_release_succeeds_with_an_old_all_clear_sample_if_the_server_is_reachable(self):
        self._take_down_cancel_service('navigate_to_pose')  # discard setUp's own service
        self._cancel_services['navigate_to_pose'] = self.probe.create_service(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal',
            self._make_cancel_cb('navigate_to_pose'))
        self._spin_for(0.2)

        # the only status navigate_to_pose ever gets is this one, published
        # well before the real sleep below - no refresh, unlike every other
        # test in this file via _refresh_before_release.
        self._set_goal_status('navigate_to_pose', [])
        time.sleep(1.3)  # real elapsed time past any STATUS_FRESH_S-style window

        response = self._try_release(skip_actions={'navigate_to_pose'})
        self.assertTrue(response.success, response.message)

    def test_release_succeeds_with_an_old_idle_dock_sample_if_the_abort_service_is_reachable(self):
        self._set_dock_status('idle')
        time.sleep(1.3)

        response = self._try_release(skip_dock=True)
        self.assertTrue(response.success, response.message)


class TestNewGoalSentWhileLockedIsCancelledToo(StopContractFullChainCase):
    """#119 test 6: a new goal sent while locked is cancelled too; release
    stays refused until it is gone."""

    def test_a_goal_accepted_after_engage_is_still_cancelled_and_blocks_release(self):
        self._engage()
        self._spin_for(0.2)
        before = len(self._cancel_calls['navigate_through_poses'])

        # A goal is accepted WHILE already locked (e.g. sent a moment before
        # the UI's own block takes effect).
        self._set_goal_status('navigate_through_poses', [GoalStatus.STATUS_ACCEPTED])
        self._spin_for(0.3)
        self.assertGreater(
            len(self._cancel_calls['navigate_through_poses']), before,
            'the gate never re-cancelled a goal accepted while already locked')

        response = self._try_release()
        self.assertFalse(response.success)
        self.assertIn('navigate_through_poses', response.message)

        # Once that goal genuinely clears, release succeeds.
        self._set_goal_status('navigate_through_poses', [])
        response = self._try_release()
        self.assertTrue(response.success, response.message)


class TestRepeatedStopResumeAndStopState(StopContractFullChainCase):
    """#119 test 8: repeated Stop and Resume, and a browser reload - no
    implicit restart, stop_state correct throughout."""

    def test_repeated_engage_release_cycles_stay_correct(self):
        for _ in range(3):
            self._engage()
            self._spin_for(0.1)
            response = self._try_release()
            self.assertTrue(response.success, response.message)
            self.assertFalse(self.gate._locked)

    def test_a_late_joining_subscriber_sees_the_current_state_without_a_restart(self):
        # "Browser reload": a brand new subscriber, same gate instance, no
        # restart anywhere - the retained TRANSIENT_LOCAL sample must carry
        # the CURRENT state immediately, not a stale or missing one.
        self._engage()
        self._spin_for(0.2)
        late = []
        qos_probe = rclpy.create_node('late_joining_probe')
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_probe.create_subscription(StopState, '/safety/stop_state', late.append, qos)
        end = time.monotonic() + 2.0
        while not late and time.monotonic() < end:
            rclpy.spin_once(qos_probe, timeout_sec=0.05)
        self.assertTrue(late, 'a late-joining subscriber never received /safety/stop_state')
        self.assertEqual(late[0].state, StopState.STATE_LOCKED)
        qos_probe.destroy_node()


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
