"""
Tests for the goal, recovery and lifecycle trackers.

The goal tests cover late messages from an old goal, the lifecycle tests cover
a node that disappears and comes back, and the recovery tests cover the reset
at the start of a new goal.

Skipped if openamr_nav_msgs (openamrobot-interfaces) isn't installed.
"""

from types import SimpleNamespace
import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')

from openamr_nav_msgs.msg import NavigationStatus, NavTaskStatus, RecoveryStatus  # noqa: E402,I100
from openamrobot_nav2.status_trackers import (  # noqa: E402
    LifecycleTracker,
    RecoveryTracker,
    ResultFetcher,
    TaskTracker,
)

EXECUTING = 2
SUCCEEDED = 4
ABORTED = 6

ACTIVE = 3
UNCONFIGURED = 1


def goal(n):
    return bytes([n]) * 16


def entry(n, status, sec):
    """Build one GoalStatus entry for goal number n."""
    return SimpleNamespace(
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=goal(n)),
            stamp=SimpleNamespace(sec=sec, nanosec=0)),
        status=status)


class TestTaskTracker(unittest.TestCase):

    def test_empty_status_changes_nothing(self):
        tracker = TaskTracker()
        self.assertEqual(tracker.on_status([]), (False, None))
        self.assertEqual(tracker.state, NavTaskStatus.STATE_UNKNOWN)

    def test_new_goal_is_tracked(self):
        tracker = TaskTracker()
        new_goal, to_fetch = tracker.on_status([entry(1, EXECUTING, 10)])
        self.assertTrue(new_goal)
        self.assertIsNone(to_fetch)
        self.assertEqual(tracker.goal_id, goal(1))
        self.assertEqual(tracker.state, NavTaskStatus.STATE_ACTIVE)
        self.assertEqual(tracker.goal_stamp, (10, 0))

    def test_terminal_status_offers_the_result_to_fetch(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, EXECUTING, 10)])
        _, to_fetch = tracker.on_status([entry(1, SUCCEEDED, 10)])
        self.assertEqual(bytes(to_fetch.uuid), goal(1))
        self.assertEqual(tracker.state, NavTaskStatus.STATE_SUCCEEDED)

    def test_stops_offering_it_once_the_request_is_marked_sent(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, SUCCEEDED, 10)])
        tracker.mark_result_requested(goal(1))
        _, to_fetch = tracker.on_status([entry(1, SUCCEEDED, 10)])
        self.assertIsNone(to_fetch)

    def test_keeps_offering_it_if_never_marked_sent(self):
        # the exact bug this guards: a terminal goal must not be treated as
        # "handled" just because a status update for it arrived
        tracker = TaskTracker()
        tracker.on_status([entry(1, SUCCEEDED, 10)])
        for _ in range(3):
            _, to_fetch = tracker.on_status([entry(1, SUCCEEDED, 10)])
            self.assertEqual(bytes(to_fetch.uuid), goal(1))

    def test_newest_goal_wins_when_the_list_has_several(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, ABORTED, 10), entry(2, EXECUTING, 20)])
        self.assertEqual(tracker.goal_id, goal(2))
        self.assertEqual(tracker.state, NavTaskStatus.STATE_ACTIVE)

    def test_feedback_and_result_for_the_tracked_goal_are_applied(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, EXECUTING, 10)])
        self.assertTrue(tracker.on_feedback(goal(1), 3.5))
        self.assertEqual(tracker.distance_remaining, 3.5)
        tracker.on_status([entry(1, ABORTED, 10)])
        self.assertTrue(tracker.on_result(goal(1), 204))
        self.assertEqual(tracker.native_error_code, 204)
        self.assertEqual(tracker.reason, NavigationStatus.GOAL_OUTSIDE_MAP)

    def test_late_feedback_from_an_old_goal_is_ignored(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, EXECUTING, 10)])
        tracker.on_status([entry(1, SUCCEEDED, 10), entry(2, EXECUTING, 20)])
        tracker.on_feedback(goal(2), 7.0)
        # goal 1 was replaced, but one more feedback message from it shows up
        self.assertFalse(tracker.on_feedback(goal(1), 0.1))
        self.assertEqual(tracker.distance_remaining, 7.0)

    def test_late_result_from_an_old_goal_is_ignored(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, EXECUTING, 10)])
        tracker.on_status([entry(1, ABORTED, 10)])
        # the result request for goal 1 is still in flight when goal 2 starts
        tracker.on_status([entry(1, ABORTED, 10), entry(2, EXECUTING, 20)])
        self.assertFalse(tracker.on_result(goal(1), 208))
        self.assertEqual(tracker.native_error_code, 0)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)
        self.assertEqual(tracker.state, NavTaskStatus.STATE_ACTIVE)

    def test_new_goal_clears_the_previous_goals_leftovers(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, EXECUTING, 10)])
        tracker.on_feedback(goal(1), 2.0)
        tracker.on_status([entry(1, ABORTED, 10)])
        tracker.on_result(goal(1), 208)
        tracker.on_status([entry(1, ABORTED, 10), entry(2, EXECUTING, 20)])
        self.assertEqual(tracker.distance_remaining, 0.0)
        self.assertEqual(tracker.native_error_code, 0)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_result_codes_map_to_reasons(self):
        tracker = TaskTracker()
        tracker.on_status([entry(1, ABORTED, 10)])
        tracker.on_result(goal(1), 208)
        self.assertEqual(tracker.reason, NavigationStatus.NO_VALID_PATH)
        tracker.on_result(goal(1), 999)
        self.assertEqual(tracker.reason, NavigationStatus.NAV_UNKNOWN_FAULT)
        tracker.on_result(goal(1), 0)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_start_outside_map_maps_to_start_blocked_not_goal_outside_map(self):
        # error 203 is START_OUTSIDE_MAP, a different failure than 204
        # (GOAL_OUTSIDE_MAP) and must not share its reason
        tracker = TaskTracker()
        tracker.on_status([entry(1, ABORTED, 10)])
        tracker.on_result(goal(1), 203)
        self.assertEqual(tracker.reason, NavigationStatus.START_BLOCKED)
        self.assertNotEqual(tracker.reason, NavigationStatus.GOAL_OUTSIDE_MAP)


class TestRecoveryTracker(unittest.TestCase):

    def test_counts_recovery_actions_that_start_running(self):
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('Spin', 'RUNNING')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_SPIN)
        self.assertEqual(tracker.attempt, 1)
        self.assertEqual(tracker.reason, NavigationStatus.RECOVERY_IN_PROGRESS)
        tracker.on_bt_event('Spin', 'SUCCESS')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_other_behavior_tree_nodes_are_ignored(self):
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('ComputePathToPose', 'RUNNING')
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)

    def test_an_unrelated_clear_prefixed_node_is_not_counted(self):
        # the match is the four exact costmap-clear names, not a "Clear"
        # prefix - a differently-named node that happens to start the same
        # way must not be mistaken for one
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('ClearSomethingUnrelated', 'RUNNING')
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)

    def test_the_subtree_clear_pair_counts_as_one_outer_cycle(self):
        # platform-sw#91, Mohamed, Option A: the real Jazzy BT's "ClearingActions"
        # is one RoundRobin pick containing two sequential leaves
        # (ClearLocalCostmap-Subtree then ClearGlobalCostmap-Subtree) - that
        # is one outer recovery cycle, not two.
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('ClearLocalCostmap-Subtree', 'RUNNING')
        self.assertEqual(tracker.attempt, 1)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_CLEAR_COSTMAP)
        tracker.on_bt_event('ClearLocalCostmap-Subtree', 'SUCCESS')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)

        tracker.on_bt_event('ClearGlobalCostmap-Subtree', 'RUNNING')
        self.assertEqual(tracker.attempt, 1, 'the pair must not count as a second attempt')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_CLEAR_COSTMAP)
        tracker.on_bt_event('ClearGlobalCostmap-Subtree', 'SUCCESS')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)
        self.assertEqual(tracker.attempt, 1)

    def test_the_context_clear_pair_is_inline_and_never_counted(self):
        # platform-sw#91, Mohamed: ClearLocalCostmap-Context/ClearGlobalCostmap-
        # Context run inside ComputePathToPose's/FollowPath's own one-retry
        # RecoveryNode, inline with the primary - they are not an outer
        # recovery cycle at all and must not add to the displayed attempt,
        # even though they still show up as the currently-running action.
        tracker = RecoveryTracker(6)
        for name in ('ClearLocalCostmap-Context', 'ClearGlobalCostmap-Context'):
            tracker.on_bt_event(name, 'RUNNING')
            self.assertEqual(tracker.action, RecoveryStatus.ACTION_CLEAR_COSTMAP, name)
            self.assertEqual(tracker.attempt, 0, name)
            tracker.on_bt_event(name, 'SUCCESS')
            self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE, name)
            self.assertEqual(tracker.attempt, 0, name)

    def test_count_starts_over_with_a_new_goal(self):
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('BackUp', 'RUNNING')
        tracker.on_bt_event('Wait', 'RUNNING')
        self.assertEqual(tracker.attempt, 2)
        tracker.on_new_goal()
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_limit_is_kept_in_a_sane_range(self):
        self.assertEqual(RecoveryTracker(0).attempt_limit, 1)
        self.assertEqual(RecoveryTracker(1000).attempt_limit, 255)

    def test_the_named_inline_waits_are_not_counted(self):
        # platform-sw#91/#92 review (Mohamed): navigate_to_pose.xml's two inline
        # 2.0 s Waits (before each bounded per-action clear) are explicitly
        # named WaitBeforeClear-GlobalContext / WaitBeforeClear-LocalContext,
        # specifically so they never match the bare 'Wait' key - an outer
        # round-robin recovery action, counted toward the displayed attempt -
        # the way an unnamed <Wait> would.
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('WaitBeforeClear-GlobalContext', 'RUNNING')
        tracker.on_bt_event('WaitBeforeClear-LocalContext', 'RUNNING')
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)

    def test_inline_recovery_does_not_exhaust_the_real_retry_budget(self):
        # platform-sw#91's original concern (each inline recovery counting
        # toward the shared attempt_limit) is now moot for READINESS by
        # construction: exhaustion comes from primary-failure counting
        # (TestRecoveryTrackerExhaustion), not from attempt at all, and the
        # Context clears are categorically inline
        # (test_the_context_clear_pair_is_inline_and_never_counted) regardless
        # of how the paired Wait is named. This only confirms the two facts
        # still hold together: the named Waits still do not display as a
        # recovery action, and inline Context clears still never raise
        # RECOVERY_LIMIT_REACHED on their own.
        tracker = RecoveryTracker(6)
        for name in ('WaitBeforeClear-GlobalContext', 'ClearGlobalCostmap-Context',
                     'WaitBeforeClear-LocalContext', 'ClearLocalCostmap-Context'):
            tracker.on_bt_event(name, 'RUNNING')
            tracker.on_bt_event(name, 'SUCCESS')
        self.assertEqual(tracker.attempt, 0)
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)


class TestRecoveryTrackerExhaustion(unittest.TestCase):
    """
    Covers platform-sw#91, Mohamed's review.

    Readiness is a separate decision from the displayed attempt count.
    RECOVERY_LIMIT_REACHED no longer comes from attempt >= attempt_limit (a
    diagnostic leaf-action counter that both overcounts paired clears and
    undercounts nothing, but was never actually tracking Nav2's own retry
    budget). It comes from counting how many times NavigateWithReplanning
    (the primary) itself has failed: per Jazzy's recovery_node.cpp, the
    primary is ticked number_of_retries + 1 times before RecoveryNode gives
    up, so reaching that count on its own FAILURE is exhaustion; failing
    short of it means the recovery branch (or an earlier diagnostic check)
    cut this goal's attempt short instead - a different, lesser outcome.
    """

    def _fail_primary(self, tracker, times):
        for _ in range(times):
            tracker.on_bt_event('NavigateWithReplanning', 'FAILURE')

    def test_primary_exhausting_its_retries_reaches_the_limit(self):
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 7)  # number_of_retries + 1
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_an_early_recovery_branch_failure_is_not_the_limit(self):
        # Real capture (platform-sw#91): an unreachable goal makes the
        # diagnostic Fallback (WouldAControllerRecoveryHelp /
        # WouldAPlannerRecoveryHelp) fail immediately, so NavigateRecovery
        # itself FAILUREs on the very FIRST primary failure - zero retries
        # consumed. RecoveryFallback (the ReactiveFallback one level further
        # in) never even ran in that capture, which is exactly why this no
        # longer tries to identify which named node on the recovery side
        # failed.
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 1)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_a_mid_budget_recovery_branch_failure_is_not_the_limit(self):
        # Same outcome as the early case above, but after a few successful
        # retries first - still short of the budget when it gives up.
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 3)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_the_primary_succeeding_after_the_last_recovery_is_not_the_limit(self):
        # Mohamed: "Nav2 runs the primary once more after the sixth
        # successful recovery" - if THAT attempt succeeds, NavigateRecovery
        # returns SUCCESS, not FAILURE, and must never have been flagged.
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 6)
        tracker.on_bt_event('NavigateWithReplanning', 'SUCCESS')
        tracker.on_bt_event('NavigateRecovery', 'SUCCESS')
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_a_navigate_recovery_success_clears_a_previous_exhaustion_flag(self):
        # not sticky forever within a goal - a SUCCESS on NavigateRecovery
        # itself always clears it, whatever happened on an earlier cycle.
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 7)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)
        tracker.on_new_goal()
        self._fail_primary(tracker, 1)
        tracker.on_bt_event('NavigateWithReplanning', 'SUCCESS')
        tracker.on_bt_event('NavigateRecovery', 'SUCCESS')
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_the_bt_s_own_goal_start_resets_a_previous_goal_s_accounting(self):
        # platform-sw#90, Mohamed's 11:17 review: the goal boundary must come
        # from goal-start EVIDENCE, not from when a status callback happened
        # to be processed. NavigateRecovery entering RUNNING is that
        # evidence, in the BT's own ordered stream (confirmed in a real
        # Jazzy capture: every goal's run begins with exactly that
        # transition), so no cross-stream clock comparison is involved.
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 7)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

        # a new goal's BT run begins - the previous goal's exhaustion and
        # its primary-failure count must not carry into it
        tracker.on_bt_event('NavigateRecovery', 'RUNNING', previous_status='IDLE')
        self.assertEqual(tracker.reason, NavigationStatus.NONE)
        self.assertEqual(tracker.attempt, 0)

        # and the new goal needs its own full budget again, not one more
        # failure on top of the old count
        self._fail_primary(tracker, 1)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_a_running_transition_that_is_not_an_entry_does_not_reset(self):
        # only an entry INTO running is a goal start; a repeated RUNNING
        # report mid-goal must not wipe the count being accumulated
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('NavigateRecovery', 'RUNNING', previous_status='IDLE')
        self._fail_primary(tracker, 3)
        tracker.on_bt_event('NavigateRecovery', 'RUNNING', previous_status='RUNNING')
        self._fail_primary(tracker, 4)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertEqual(
            tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            'the 7 primary failures all belong to this one goal')

    def test_goal_boundary_isolation_does_not_depend_on_event_timestamps(self):
        # The BT log's own timestamps are documented as "typically wall
        # clock" (nav2_msgs/BehaviorTreeStatusChange) and a real capture
        # confirmed wall-clock values under use_sim_time:=true, while a
        # node's own clock there is simulated - so comparing the two is
        # comparing different clocks, and any such filter is either inert
        # or wrong depending on direction. on_bt_event therefore takes no
        # timestamp at all (platform-sw#90, Mohamed's 11:17 review).
        tracker = RecoveryTracker(6)
        with self.assertRaises(TypeError):
            tracker.on_bt_event('NavigateWithReplanning', 'FAILURE', event_time_s=50.0)

    def test_a_real_captured_unreachable_goal_sequence_is_not_the_limit(self):
        """
        Replay a real captured Jazzy /behavior_tree_log.

        Captured for an out-of-map goal (platform-sw#91), node_name/status
        pairs only. NavigateRecovery fails here with zero retries consumed
        - the diagnostic Fallback decided no recovery would help before
        RecoveryFallback ever ran. The event list is the real capture
        verbatim (irrelevant IDLE transitions included), confirming the
        fix against real Nav2/Jazzy behavior, not just a hand-written
        scenario.
        """
        tracker = RecoveryTracker(6)
        real_capture = [
            ('ComputePathToPose', 'FAILURE'), ('ComputePathToPose', 'IDLE'),
            ('Sequence', 'RUNNING'),
            ('WouldAPlannerRecoveryHelp', 'FAILURE'), ('WouldAPlannerRecoveryHelp', 'IDLE'),
            ('Sequence', 'FAILURE'), ('Sequence', 'IDLE'),
            ('ComputePathToPose', 'IDLE'), ('ComputePathToPose', 'FAILURE'),
            ('RateController', 'FAILURE'), ('ComputePathToPose', 'IDLE'),
            ('ControllerSelector', 'IDLE'), ('PlannerSelector', 'IDLE'),
            ('RateController', 'IDLE'),
            ('NavigateWithReplanning', 'FAILURE'), ('NavigateWithReplanning', 'IDLE'),
            ('Sequence', 'RUNNING'), ('Fallback', 'RUNNING'),
            ('WouldAControllerRecoveryHelp', 'FAILURE'),
            ('WouldAPlannerRecoveryHelp', 'FAILURE'),
            ('WouldAControllerRecoveryHelp', 'IDLE'), ('WouldAPlannerRecoveryHelp', 'IDLE'),
            ('Fallback', 'FAILURE'), ('Fallback', 'IDLE'),
            ('Sequence', 'FAILURE'), ('Sequence', 'IDLE'),
            ('NavigateRecovery', 'IDLE'), ('NavigateRecovery', 'FAILURE'),
        ]
        for node_name, status in real_capture:
            tracker.on_bt_event(node_name, status)
        self.assertNotEqual(
            tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            'a single-shot "no recovery would help" bailout is not retry exhaustion')

    def test_a_cancelled_goal_s_trailing_primary_failure_can_leak_into_the_next_goal(self):
        """
        Document a known, open residual gap in cross-topic reset ordering.

        platform-sw#90, Mohamed's re-review at b259349; tracked separately
        as platform-sw#127. on_new_goal() and on_bt_event() are driven by
        two different topics (goal status, /behavior_tree_log) that are each
        internally ordered but not ordered relative to each other. If a
        client cancels a goal mid-recovery and immediately sends the next
        one, the new goal's status can reach this tracker (resetting it)
        before the cancelled goal's own last in-flight primary-failure tick
        is delivered - and nav2_msgs/BehaviorTreeLog carries no goal
        correlation id, so that stray tick cannot be told apart from the new
        goal's own first failure.

        This is NOT fixed here: nothing in the current BT log message lets a
        subscriber attribute an event to a goal, so the fix needs either a
        Nav2 interface change upstream or a different mitigation, not a
        tweak to this tracker's reset timing (moving the reset later would
        just trade this gap for the one #92 already fixed - a leftover
        exhaustion flag persisting into a goal that has no BT activity of
        its own yet). Recorded as a test, not an xfail, because this is the
        tracker's actual current behavior, not a bug being masked.
        """
        tracker = RecoveryTracker(6)
        self._fail_primary(tracker, 5)  # the cancelled goal, deep but not exhausted

        # the next goal's own status reaches this tracker first and resets it
        tracker.on_new_goal()

        # the cancelled goal's last in-flight primary tick, delivered late,
        # is counted as the new goal's first failure instead of being
        # discarded
        tracker.on_bt_event('NavigateWithReplanning', 'FAILURE')

        # the new goal then fails 6 more times entirely on its own
        self._fail_primary(tracker, 6)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertEqual(
            tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            'exhausted one failure early: a stray tick from the cancelled '
            "goal, not from the new goal's own 6 failures")

    def test_a_late_new_goal_reset_can_erase_the_new_goal_s_own_first_failure(self):
        """
        Document the opposite-direction case of the same residual gap.

        platform-sw#127 (Mohamed's follow-up comment, 2026-11-08): a Python
        replay of the cross-topic race the first test above documents, run
        in the other delivery order. on_new_goal() is driven by the goal
        status topic; on_bt_event()'s own goal-start detection
        (NavigateRecovery entering RUNNING) is driven by /behavior_tree_log.
        The two topics are each internally ordered but not ordered relative
        to each other, so either can arrive after the other for the SAME
        goal, not only across a goal boundary.

        Here the new goal's BT run has already started (NavigateRecovery
        RUNNING) and already recorded one genuine primary failure of its
        own, before its own status-topic "new goal" notification is
        delivered late and calls on_new_goal() - wiping that real failure
        rather than a stray one from a previous goal. The net effect is the
        mirror image of the first test: instead of exhausting one failure
        early, this goal is one failure short of exhaustion when it
        actually should have reached it.

        Not fixed here, for the same reason as the first test: neither
        topic carries a goal-correlation id a subscriber could use to
        reject this specific late message instead of the legitimate one
        it looks identical to.
        """
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('NavigateRecovery', 'RUNNING', previous_status='IDLE')
        tracker.on_bt_event('NavigateWithReplanning', 'FAILURE')  # the new goal's own 1st failure

        # this goal's own status-topic "new goal" notification, delivered
        # late relative to the BT log, erases the failure just recorded
        tracker.on_new_goal()

        self._fail_primary(tracker, 6)
        tracker.on_bt_event('NavigateRecovery', 'FAILURE')
        self.assertNotEqual(
            tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED,
            "one failure short of exhaustion: the goal's own first failure "
            'was erased by its own late-delivered reset')


class FakeResultClient:
    """Stands in for the get_result action-client service."""

    def __init__(self, ready=False):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        fut = FakeFuture()
        self.calls.append((req, fut))
        return fut


class FakeFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._exc = None
        self._cbs = []

    def done(self):
        return self._done

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._result

    def add_done_callback(self, cb):
        self._cbs.append(cb)

    def finish(self, result):
        self._done = True
        self._result = result
        for cb in self._cbs:
            cb(self)

    def fail(self, exc):
        """Finish as a failed call, the way a service going away mid-call would."""
        self._done = True
        self._exc = exc
        for cb in self._cbs:
            cb(self)


class FakeLogger:

    def __init__(self):
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)


class TestResultFetcher(unittest.TestCase):
    """
    Regression tests for a review finding.

    A terminal goal's result must be retried, not treated as handled, if the
    get_result service was not ready the first time a status update for it
    arrived.
    """

    def make(self, ready=False):
        task = TaskTracker()
        client = FakeResultClient(ready=ready)
        logger = FakeLogger()
        fetcher = ResultFetcher(task, client, lambda: SimpleNamespace(goal_id=None), logger)
        return task, client, logger, fetcher

    def test_service_unavailable_then_available_then_applied(self):
        # Nav2 publishes goal status only on change, so the retry must not
        # depend on another status update ever arriving - this test drives it
        # with retry_if_pending() alone, the way the node's timer does, and
        # never calls task.on_status() a second time for the same goal.
        task, client, logger, fetcher = self.make(ready=False)

        # 1. a terminal goal status arrives while the service is unavailable
        _, to_fetch = task.on_status([entry(1, ABORTED, 10)])
        fetcher.request(to_fetch)
        # 2. verify no result is applied yet, and nothing was actually sent
        self.assertEqual(client.calls, [])
        self.assertEqual(task.native_error_code, 0)
        self.assertEqual(task.reason, NavigationStatus.NONE)
        self.assertTrue(logger.warnings)

        # the timer fires again while still unavailable: still nothing sent,
        # with no status update involved at all
        fetcher.retry_if_pending()
        self.assertEqual(client.calls, [])

        # 3. make the service available, the timer fires again
        client.ready = True
        # 4. verify the result is requested
        fetcher.retry_if_pending()
        self.assertEqual(len(client.calls), 1)

        # and applied once the response comes back
        _, future = client.calls[0]
        future.finish(SimpleNamespace(result=SimpleNamespace(error_code=208)))
        # 5. verify native_error_code and reason are updated correctly
        self.assertEqual(task.native_error_code, 208)
        self.assertEqual(task.reason, NavigationStatus.NO_VALID_PATH)

        # nothing left to retry, and a further status update for the same
        # goal does not ask again either
        fetcher.retry_if_pending()
        self.assertEqual(len(client.calls), 1)
        _, to_fetch = task.on_status([entry(1, ABORTED, 10)])
        self.assertIsNone(to_fetch)

    def test_available_from_the_start_asks_once(self):
        task, client, logger, fetcher = self.make(ready=True)
        _, to_fetch = task.on_status([entry(1, SUCCEEDED, 10)])
        fetcher.request(to_fetch)
        self.assertEqual(len(client.calls), 1)
        self.assertFalse(logger.warnings)

    def test_retry_if_pending_does_nothing_when_nothing_is_owed(self):
        task, client, logger, fetcher = self.make(ready=True)
        fetcher.retry_if_pending()
        self.assertEqual(client.calls, [])

    def test_async_failure_is_retried_once_the_service_recovers(self):
        # the request was sent, but the future fails instead of returning a
        # result (the service disappeared mid-call) - the result is still
        # owed, and the tick-driven retry must pick it back up
        task, client, logger, fetcher = self.make(ready=True)
        _, to_fetch = task.on_status([entry(1, ABORTED, 10)])
        fetcher.request(to_fetch)
        self.assertEqual(len(client.calls), 1)

        _, future = client.calls[0]
        future.fail(RuntimeError('service disappeared'))
        self.assertEqual(task.native_error_code, 0)
        self.assertEqual(task.reason, NavigationStatus.NONE)

        fetcher.retry_if_pending()
        self.assertEqual(len(client.calls), 2)
        _, future2 = client.calls[1]
        future2.finish(SimpleNamespace(result=SimpleNamespace(error_code=208)))
        self.assertEqual(task.native_error_code, 208)
        self.assertEqual(task.reason, NavigationStatus.NO_VALID_PATH)

    def test_async_failure_for_a_goal_no_longer_tracked_is_not_retried(self):
        task, client, logger, fetcher = self.make(ready=True)
        _, to_fetch = task.on_status([entry(1, ABORTED, 10)])
        fetcher.request(to_fetch)
        _, future = client.calls[0]

        # a new goal replaces goal 1 before the async failure comes back
        task.on_status([entry(1, ABORTED, 10), entry(2, EXECUTING, 20)])
        future.fail(RuntimeError('service disappeared'))

        # goal 1 is stale now: nothing is re-armed, nothing new is sent for it
        fetcher.retry_if_pending()
        self.assertEqual(len(client.calls), 1)


class TestLifecycleTracker(unittest.TestCase):

    def test_unknown_until_confirmed(self):
        tracker = LifecycleTracker(['planner_server'], stale_after=3.0)
        self.assertIsNone(tracker.state('planner_server', 0.0))
        self.assertEqual(tracker.lost(0.0), [])

    def test_confirmed_state_holds_while_it_is_fresh(self):
        tracker = LifecycleTracker(['planner_server'], stale_after=3.0)
        tracker.confirm('planner_server', ACTIVE, now=10.0)
        self.assertEqual(tracker.state('planner_server', 12.0), ACTIVE)

    def test_node_that_disappears_does_not_stay_active(self):
        tracker = LifecycleTracker(['planner_server'], stale_after=3.0)
        tracker.confirm('planner_server', ACTIVE, now=10.0)
        # nothing confirms it any more
        self.assertIsNone(tracker.state('planner_server', 14.0))
        self.assertEqual(tracker.lost(14.0), ['planner_server'])

    def test_unreachable_drops_the_state_immediately(self):
        tracker = LifecycleTracker(['planner_server'], stale_after=3.0)
        tracker.confirm('planner_server', ACTIVE, now=10.0)
        tracker.unreachable('planner_server')
        self.assertIsNone(tracker.state('planner_server', 10.1))
        self.assertEqual(tracker.lost(10.1), ['planner_server'])

    def test_restarted_node_is_picked_up_again(self):
        tracker = LifecycleTracker(['planner_server'], stale_after=3.0)
        tracker.confirm('planner_server', ACTIVE, now=10.0)
        tracker.unreachable('planner_server')
        # it comes back unconfigured, then the lifecycle manager activates it
        tracker.confirm('planner_server', UNCONFIGURED, now=20.0)
        self.assertEqual(tracker.state('planner_server', 20.5), UNCONFIGURED)
        self.assertEqual(tracker.lost(20.5), [])
        tracker.confirm('planner_server', ACTIVE, now=22.0)
        self.assertEqual(tracker.state('planner_server', 22.5), ACTIVE)

    def test_only_nodes_that_were_seen_count_as_lost(self):
        tracker = LifecycleTracker(['amcl', 'planner_server'], stale_after=3.0)
        tracker.confirm('amcl', ACTIVE, now=10.0)
        self.assertEqual(tracker.lost(30.0), ['amcl'])

    def test_seen_any_turns_true_after_the_first_confirmation(self):
        tracker = LifecycleTracker(['amcl'], stale_after=3.0)
        self.assertFalse(tracker.seen_any())
        tracker.confirm('amcl', ACTIVE, now=1.0)
        self.assertTrue(tracker.seen_any())


if __name__ == '__main__':
    unittest.main(verbosity=2)
