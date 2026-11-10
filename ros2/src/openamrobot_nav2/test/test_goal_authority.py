"""
Unit tests for goal_authority.py (platform-sw#124, D-13).

Pure Python, no rclpy needed - the ledger itself has no ROS dependency.
"""
import unittest

from openamrobot_nav2.goal_authority import GoalAuthority, ProducerState


class TestGoalAuthority(unittest.TestCase):

    def test_never_heard_from_at_all_is_unknown_not_idle(self):
        # D-13 rejects inferring "clear" from silence - before this
        # producer's own FIRST heartbeat or notify ever arrives, there is
        # no evidence at all, not "evidence of idle."
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.UNKNOWN)

    def test_a_heartbeat_with_no_goal_ever_sent_is_idle(self):
        # D-13: "Clean-boot Resume within 2 s with no goal ever sent." This
        # is what makes that possible - a producer's own heartbeat, not a
        # goal, establishes liveness and an outstanding count of zero.
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.heartbeat('undock_robot', instance_id='a', now_s=0.5)
        self.assertEqual(authority.state('undock_robot', now_s=0.5), ProducerState.IDLE)

    def test_a_send_then_done_from_the_same_instance_returns_to_idle(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)
        authority.notify_done('undock_robot', instance_id='a', now_s=1.0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.IDLE)

    def test_an_outstanding_goal_blocks_until_done(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)

    def test_stale_contact_degrades_back_to_unknown(self):
        # the producer heartbeated once, then stopped (crashed, hung,
        # network partition) - old evidence must not stay "clear" forever.
        authority = GoalAuthority(['undock_robot'], stale_after_s=2.0)
        authority.heartbeat('undock_robot', instance_id='a', now_s=0.0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.IDLE)
        self.assertEqual(
            authority.state('undock_robot', now_s=2.5), ProducerState.UNKNOWN,
            'no contact for longer than stale_after_s must not read as idle')

    def test_a_restart_with_nothing_outstanding_is_fine(self):
        # the process restarted, but it had already reported done before it
        # went away - no orphaned count, the new instance starts clean
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.notify_done('undock_robot', instance_id='a', now_s=0.5)
        authority.notify_sent('undock_robot', instance_id='b', now_s=1.0)  # restarted instance
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.ACTIVE)
        authority.notify_done('undock_robot', instance_id='b', now_s=1.5)
        self.assertEqual(authority.state('undock_robot', now_s=1.5), ProducerState.IDLE)

    def test_a_restart_with_something_outstanding_is_unknown_not_cleared(self):
        # platform-sw#124 D-13: "missing, stale, restarted... is UNKNOWN and
        # blocks." The old instance sent a goal and never reported it done
        # (crashed mid-goal) - the new instance's first contact must not
        # silently wipe that unaccounted-for goal into a clean IDLE.
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        # instance 'a' is gone; instance 'b' starts up and heartbeats
        authority.heartbeat('undock_robot', instance_id='b', now_s=1.0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.UNKNOWN)

    def test_the_new_instance_clearing_its_own_goal_does_not_clear_the_orphan(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.notify_sent('undock_robot', instance_id='b', now_s=1.0)
        authority.notify_done('undock_robot', instance_id='b', now_s=1.5)
        self.assertEqual(
            authority.state('undock_robot', now_s=1.5), ProducerState.UNKNOWN,
            "clearing instance b's own goal must not un-orphan instance a's")

    def test_a_done_from_a_stale_instance_is_ignored_not_misapplied(self):
        # instance 'a's late, stale notify_done arrives after 'b' has
        # already taken over - must not decrement b's count
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.notify_sent('undock_robot', instance_id='b', now_s=1.0)
        authority.notify_done('undock_robot', instance_id='a', now_s=1.5)  # stale, from 'a'
        self.assertEqual(authority.state('undock_robot', now_s=1.5), ProducerState.UNKNOWN)

    def test_multiple_outstanding_goals_need_matching_done_calls(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.1)
        authority.notify_done('undock_robot', instance_id='a', now_s=0.5)
        self.assertEqual(
            authority.state('undock_robot', now_s=0.5), ProducerState.ACTIVE,
            'one of two outstanding goals is still unaccounted for')
        authority.notify_done('undock_robot', instance_id='a', now_s=1.0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.IDLE)

    def test_reset_producer_clears_an_orphan_and_requires_fresh_contact(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.heartbeat('undock_robot', instance_id='b', now_s=1.0)  # orphaned
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.UNKNOWN)

        authority.reset_producer('undock_robot')
        self.assertEqual(
            authority.state('undock_robot', now_s=1.0), ProducerState.UNKNOWN,
            'reset discards the instance too - the next contact must be treated as fresh')

        authority.heartbeat('undock_robot', instance_id='b', now_s=1.1)
        self.assertEqual(authority.state('undock_robot', now_s=1.1), ProducerState.IDLE)

    def test_producers_are_tracked_independently(self):
        authority = GoalAuthority(['undock_robot', 'dock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)
        self.assertEqual(authority.state('dock_robot', now_s=0.0), ProducerState.UNKNOWN)

    def test_a_second_restart_does_not_erase_the_first_restarts_orphan(self):
        # panthera-momagdii's #90 review, reproducer B: A sends a goal and
        # never reports it done (gone). B restarts, correctly orphaned/
        # UNKNOWN. C then restarts again with nothing of its OWN
        # outstanding - _touch()'s orphan flag must stay sticky across that
        # second transition, not get silently overwritten back to False
        # just because C's own outstanding count happens to be zero. A's
        # goal is still unaccounted for; only an explicit reset_producer()
        # may clear it (see test_reset_producer_clears_an_orphan...).
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        authority.heartbeat('undock_robot', instance_id='b', now_s=1.0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.UNKNOWN)
        authority.heartbeat('undock_robot', instance_id='c', now_s=2.0)
        self.assertEqual(
            authority.state('undock_robot', now_s=2.0), ProducerState.UNKNOWN,
            "a second restart (c) must not un-orphan a's unresolved goal just "
            "because c's own outstanding count is zero")

    def test_is_orphaned_is_false_for_a_live_instances_own_outstanding_goal(self):
        # panthera-momagdii's #90 review, reproducer A: engage_lock() must
        # be able to tell "this instance's own, currently-outstanding goal"
        # apart from "an orphan left by a gone instance" - only the latter
        # may ever be reset_producer()'d. A live instance mid-goal is not
        # orphaned, even while locked/engaged.
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self.assertFalse(authority.is_orphaned('undock_robot'))
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)

    def test_is_orphaned_is_true_only_after_a_restart_leaves_something_outstanding(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.notify_sent('undock_robot', instance_id='a', now_s=0.0)
        self.assertFalse(authority.is_orphaned('undock_robot'))
        authority.heartbeat('undock_robot', instance_id='b', now_s=1.0)
        self.assertTrue(authority.is_orphaned('undock_robot'))
        authority.reset_producer('undock_robot')
        self.assertFalse(authority.is_orphaned('undock_robot'))

    def test_a_heartbeat_reporting_a_nonzero_outstanding_count_is_active(self):
        # platform-sw#90, panthera-momagdii's P1 review: a gate that
        # restarts between a producer's own notify_sent() and notify_done()
        # never saw the sent event on this (new) ledger object - only the
        # producer's own next heartbeat carries the truth after that. A
        # fresh ledger's first-ever contact with a producer reporting
        # outstanding=1 must read ACTIVE, not IDLE.
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.heartbeat('undock_robot', instance_id='a', now_s=0.0, outstanding=1)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)

    def test_heartbeat_outstanding_count_updates_on_every_call(self):
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        authority.heartbeat('undock_robot', instance_id='a', now_s=0.0, outstanding=1)
        self.assertEqual(authority.state('undock_robot', now_s=0.0), ProducerState.ACTIVE)
        authority.heartbeat('undock_robot', instance_id='a', now_s=1.0, outstanding=0)
        self.assertEqual(authority.state('undock_robot', now_s=1.0), ProducerState.IDLE)

    def test_heartbeat_rejects_a_negative_outstanding_count(self):
        # panthera-momagdii's #90 review, P2: state() treats any value <= 0
        # as IDLE, so a negative count would silently read the same as a
        # genuinely idle report if merely accepted.
        authority = GoalAuthority(['undock_robot'], stale_after_s=5.0)
        with self.assertRaises(ValueError):
            authority.heartbeat('undock_robot', instance_id='a', now_s=0.0, outstanding=-1)

    def test_managed_lists_exactly_the_configured_producers(self):
        authority = GoalAuthority(['undock_robot', 'dock_robot'], stale_after_s=5.0)
        self.assertEqual(set(authority.managed()), {'undock_robot', 'dock_robot'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
