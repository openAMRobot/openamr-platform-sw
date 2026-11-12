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

    def test_real_nav2_costmap_clear_node_names_are_counted(self):
        # the default Nav2 BT uses these exact names, not the single
        # 'ClearEntireCostmap' name this used to only match
        for name in ('ClearLocalCostmap-Context', 'ClearGlobalCostmap-Context',
                     'ClearLocalCostmap-Subtree', 'ClearGlobalCostmap-Subtree'):
            tracker = RecoveryTracker(6)
            tracker.on_bt_event(name, 'RUNNING')
            self.assertEqual(tracker.action, RecoveryStatus.ACTION_CLEAR_COSTMAP, name)
            self.assertEqual(tracker.attempt, 1, name)

    def test_an_unrelated_clear_prefixed_node_is_not_counted(self):
        # the match is the four exact costmap-clear names, not a "Clear"
        # prefix - a differently-named node that happens to start the same
        # way must not be mistaken for one
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('ClearSomethingUnrelated', 'RUNNING')
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)

    def test_a_costmap_clear_finishing_returns_to_no_action(self):
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('ClearLocalCostmap-Context', 'RUNNING')
        tracker.on_bt_event('ClearLocalCostmap-Context', 'SUCCESS')
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)
        self.assertEqual(tracker.attempt, 1)  # the attempt itself still counted

    def test_count_starts_over_with_a_new_goal(self):
        tracker = RecoveryTracker(6)
        tracker.on_bt_event('BackUp', 'RUNNING')
        tracker.on_bt_event('Wait', 'RUNNING')
        self.assertEqual(tracker.attempt, 2)
        tracker.on_new_goal()
        self.assertEqual(tracker.attempt, 0)
        self.assertEqual(tracker.action, RecoveryStatus.ACTION_NONE)
        self.assertEqual(tracker.reason, NavigationStatus.NONE)

    def test_limit_comes_from_configuration(self):
        tracker = RecoveryTracker(2)
        tracker.on_bt_event('Spin', 'RUNNING')
        self.assertNotEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)
        tracker.on_bt_event('Spin', 'RUNNING')
        self.assertEqual(tracker.reason, NavigationStatus.RECOVERY_LIMIT_REACHED)

    def test_limit_is_kept_in_a_sane_range(self):
        self.assertEqual(RecoveryTracker(0).attempt_limit, 1)
        self.assertEqual(RecoveryTracker(1000).attempt_limit, 255)


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
