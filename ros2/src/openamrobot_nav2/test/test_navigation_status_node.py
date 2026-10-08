"""
Node-level regression tests for two review findings.

Both were bugs in navigation_status_node.py itself, not in the pure trackers
or rules modules, so they needed a test that instantiates the real node. This
replaces only the two external service boundaries (the lifecycle get_state
clients and the nav2 get_result client) with fakes; everything else, _tick and
_poll_lifecycle included, is the real node code.

Skipped if openamr_nav_msgs (openamrobot-interfaces) isn't installed. Needs a
real rclpy, which is always present in a ROS 2 workspace.
"""

import time
from types import SimpleNamespace
import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')
rclpy = pytest.importorskip('rclpy')

from openamr_nav_msgs.msg import NavigationStatus, NavStackStatus  # noqa: E402,I100
from openamrobot_nav2.navigation_status_node import (  # noqa: E402
    LIFECYCLE_ACTIVE,
    MANAGED_NAV_NODES,
    NavigationStatusNode,
)
from openamrobot_nav2.status_rules import health_to_diagnostic_level  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

ABORTED = 6
ACTIVE = 3


class FakeFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._cbs = []

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        self._cbs.append(cb)

    def finish(self, result):
        self._done = True
        self._result = result
        for cb in self._cbs:
            cb(self)


class FakeClient:
    """Stands in for an rclpy service client: get_state or get_result."""

    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        fut = FakeFuture()
        self.calls.append((req, fut))
        return fut


def goal(n):
    return bytes([n]) * 16


def status_entry(n, status, sec=10):
    return SimpleNamespace(
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=goal(n)),
            stamp=SimpleNamespace(sec=sec, nanosec=0)),
        status=status)


class NodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = NavigationStatusNode()

    def tearDown(self):
        self.node.destroy_node()


class TestLifecyclePollTimeout(NodeTestCase):
    """
    Covers a pending-request bug.

    A pending get_state request that never completes must not block polling
    that node forever - a node can crash mid-request, and service_is_ready()
    can keep saying yes for a while afterwards since discovery lags behind.
    """

    def test_a_request_that_never_completes_is_abandoned_after_its_timeout(self):
        # Real time, not a fake clock: the node reads get_clock().now(), so the
        # timeout is shortened here and the test sleeps past it for real,
        # rather than poking a clock attribute the real node does not use.
        name = 'planner_server'
        client = FakeClient(ready=True)
        self.node._state_clients[name] = client
        self.node._state_request_timeout_s = 0.3

        self.node._poll_lifecycle()
        self.assertEqual(len(client.calls), 1)  # first request sent, left pending

        # well within the timeout: no new request while the old one is still out
        self.node._poll_lifecycle()
        self.assertEqual(len(client.calls), 1)

        # past the timeout, and the first request still never completed
        time.sleep(0.4)
        self.node._poll_lifecycle()
        self.assertEqual(len(client.calls), 2)  # abandoned, retried

        # the retried request succeeds: state is picked up again
        _, fut = client.calls[1]
        fut.finish(SimpleNamespace(current_state=SimpleNamespace(id=ACTIVE)))
        now_s = self.node._now_s()
        self.assertEqual(self.node._lifecycle.state(name, now_s), ACTIVE)

    def test_a_late_stale_response_does_not_overwrite_newer_state(self):
        # request A times out and is abandoned, request B is sent and
        # succeeds first, then A's late (stale) response finally arrives - it
        # must be ignored, not overwrite what B already confirmed
        name = 'planner_server'
        client = FakeClient(ready=True)
        self.node._state_clients[name] = client
        self.node._state_request_timeout_s = 0.3

        self.node._poll_lifecycle()
        self.assertEqual(len(client.calls), 1)
        _, fut_a = client.calls[0]

        time.sleep(0.4)
        self.node._poll_lifecycle()
        self.assertEqual(len(client.calls), 2)
        _, fut_b = client.calls[1]

        fut_b.finish(SimpleNamespace(current_state=SimpleNamespace(id=ACTIVE)))
        now_s = self.node._now_s()
        self.assertEqual(self.node._lifecycle.state(name, now_s), ACTIVE)

        stale_state = 1  # TransitionState UNCONFIGURED - not what is true now
        fut_a.finish(SimpleNamespace(current_state=SimpleNamespace(id=stale_state)))
        now_s = self.node._now_s()
        self.assertEqual(self.node._lifecycle.state(name, now_s), ACTIVE)


class TestResultRetryDrivenByTick(NodeTestCase):
    """
    Covers a retry-timing bug.

    The retry must not depend on Nav2 sending another goal-status update,
    since Nav2 only publishes it on change, not periodically.
    """

    def test_retry_fires_from_the_tick_with_no_further_status_update(self):
        client = FakeClient(ready=False)
        self.node._nav_result_client = client
        self.node._result_fetcher._client = client

        # the only status update Nav2 ever sends for this goal
        self.node._on_nav_status(SimpleNamespace(status_list=[status_entry(1, ABORTED)]))
        self.assertEqual(client.calls, [])
        self.assertEqual(self.node._task.native_error_code, 0)

        # ticks while the service is still down: nothing drives this but the tick
        for _ in range(3):
            self.node._tick()
        self.assertEqual(client.calls, [])

        # the service comes up: the very next tick sends the request, with no
        # new goal-status message at all
        client.ready = True
        self.node._tick()
        self.assertEqual(len(client.calls), 1)

        _, fut = client.calls[0]
        fut.finish(SimpleNamespace(result=SimpleNamespace(error_code=208)))
        self.assertEqual(self.node._task.native_error_code, 208)
        self.assertEqual(self.node._task.reason, NavigationStatus.NO_VALID_PATH)


class TestFailedStackState(NodeTestCase):
    """
    Covers the FAILED stack state.

    A stack stuck with some but not all nodes active must eventually report
    FAILED, not stay RESETTING forever - but a normal, brief bounce must not
    be mistaken for FAILED.
    """

    def _confirm_all_but_one(self, state_of_rest, now_s):
        for i, name in enumerate(MANAGED_NAV_NODES):
            state = LIFECYCLE_ACTIVE if i == 0 else state_of_rest
            self.node._lifecycle.confirm(name, state, now_s)

    def test_a_brief_bounce_stays_resetting_not_failed(self):
        self.node._state_request_timeout_s = 0.3
        self.node.set_parameters(
            [Parameter('stack_failed_after_s', value=0.3)])
        now_s = self.node._now_s()
        self._confirm_all_but_one(1, now_s)  # 1 = UNCONFIGURED, not ACTIVE

        state, reason = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_RESETTING)
        self.assertEqual(reason, NavigationStatus.NAV_STACK_RESETTING)

    def test_stuck_past_the_timeout_becomes_failed(self):
        self.node.set_parameters(
            [Parameter('stack_failed_after_s', value=0.3)])
        now_s = self.node._now_s()
        self._confirm_all_but_one(1, now_s)
        self.node._stack_state(self.node._now_s())  # starts the resetting_since clock

        time.sleep(0.4)
        # the stuck nodes are still not confirmed active, so refresh them so
        # they do not also go stale/unknown for an unrelated reason
        self._confirm_all_but_one(1, self.node._now_s())
        state, reason = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_FAILED)
        self.assertEqual(reason, NavigationStatus.NAV_STACK_RESETTING)

    def test_recovering_to_active_clears_the_failed_state(self):
        self.node.set_parameters(
            [Parameter('stack_failed_after_s', value=0.3)])
        now_s = self.node._now_s()
        self._confirm_all_but_one(1, now_s)
        self.node._stack_state(self.node._now_s())
        time.sleep(0.4)
        self._confirm_all_but_one(1, self.node._now_s())
        state, _ = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_FAILED)

        # all nodes come back
        now_s = self.node._now_s()
        for name in MANAGED_NAV_NODES:
            self.node._lifecycle.confirm(name, LIFECYCLE_ACTIVE, now_s)
        state, reason = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_ACTIVE)
        self.assertEqual(reason, NavigationStatus.NONE)

        # and if it gets stuck again later, the clock must have restarted,
        # not still be counting from the first time
        now_s = self.node._now_s()
        self._confirm_all_but_one(1, now_s)
        state, _ = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_RESETTING)

    def test_going_unknown_clears_the_clock_for_a_later_stall(self):
        self.node.set_parameters(
            [Parameter('stack_failed_after_s', value=0.3)])
        now_s = self.node._now_s()
        self._confirm_all_but_one(1, now_s)
        self.node._stack_state(self.node._now_s())  # first stall begins, clock starts

        time.sleep(0.2)  # into the stall, still under the 0.3s timeout

        # every node drops out of confirmation (e.g. their services become
        # unreachable) - the stack reports UNKNOWN, and must forget the clock
        # it was keeping for the stall above, not just pause it
        for name in MANAGED_NAV_NODES:
            self.node._lifecycle.unreachable(name)
        state, _ = self.node._stack_state(self.node._now_s())
        self.assertEqual(state, NavStackStatus.STATE_UNKNOWN)

        # total elapsed since the FIRST stall is now ~0.4s, past the 0.3s
        # timeout - but that old clock is gone
        time.sleep(0.2)

        # a brand new, separate stall starts now
        self._confirm_all_but_one(1, self.node._now_s())
        state, reason = self.node._stack_state(self.node._now_s())
        # freshly RESETTING, not immediately FAILED from inherited time
        self.assertEqual(state, NavStackStatus.STATE_RESETTING)
        self.assertEqual(reason, NavigationStatus.NAV_STACK_RESETTING)


class FakePub:
    """Stands in for an rclpy publisher, capturing every message it is given."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class TestDiagnosticsMirror(NodeTestCase):
    """
    The /diagnostics mirror publishes every tick, independent of status gating.

    /navigation/status itself may not be due yet (PublishGate only gates that
    topic), but a monitoring tool watching /diagnostics should not have to
    understand NavigationStatus's own change/heartbeat gating to get a fresh
    reading.
    """

    def test_diagnostics_publishes_every_tick_even_with_no_change(self):
        diag_pub = FakePub()
        status_pub = FakePub()
        self.node._diag_pub = diag_pub
        self.node._pub = status_pub

        for _ in range(3):
            self.node._tick()

        # unconditional: one /diagnostics message per tick, no gating
        self.assertEqual(len(diag_pub.published), 3)
        # meanwhile /navigation/status is still gated: nothing changed between
        # these back-to-back ticks and no heartbeat period has elapsed, so
        # only the first tick (nothing published yet) is due
        self.assertEqual(len(status_pub.published), 1)

    def test_overall_entry_reflects_navigation_status_health(self):
        diag_pub = FakePub()
        self.node._diag_pub = diag_pub
        self.node._tick()

        published = diag_pub.published[-1]
        overall = published.status[0]
        self.assertEqual(overall.name, 'navigation_status_node: health')

        built = self.node._build_status(self.node.get_clock().now())
        self.assertEqual(overall.level, health_to_diagnostic_level(built.health))

    def test_one_diagnostic_entry_per_tracked_sensor(self):
        diag_pub = FakePub()
        self.node._diag_pub = diag_pub
        self.node._tick()

        published = diag_pub.published[-1]
        sensor_entries = [
            s for s in published.status if s.name != 'navigation_status_node: health']
        self.assertEqual(len(sensor_entries), len(self.node._sensors))
        self.assertEqual(
            {s.hardware_id for s in sensor_entries}, set(self.node._sensors.keys()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
