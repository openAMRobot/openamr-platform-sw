"""
Node-level regression tests for several review findings.

These were bugs in navigation_status_node.py itself, not in the pure trackers
or rules modules, so they needed a test that instantiates the real node. This
replaces only the two external service boundaries (the lifecycle get_state
clients and the nav2 get_result client) with fakes; everything else, _tick and
_poll_lifecycle included, is the real node code.

Skipped if openamr_nav_msgs (openamrobot-interfaces) isn't installed. Needs a
real rclpy, which is always present in a ROS 2 workspace.
"""

import math
import os
import tempfile
import time
from types import SimpleNamespace
import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')
rclpy = pytest.importorskip('rclpy')

from nav_msgs.msg import OccupancyGrid  # noqa: E402
from openamr_nav_msgs.msg import (  # noqa: E402,I100
    NavigationStatus,
    NavStackStatus,
    SiteConfigStatus,
)
from openamrobot_nav2.config_identity import config_identity_hash  # noqa: E402
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


class TestSiteConfigStatus(NodeTestCase):
    """
    Covers platform-sw#73 review findings: map identity, config identity and
    zone presence/schema must not report a pass when there is nothing real
    to confirm them - a missing or merely-plausible-looking input has to
    keep readiness blocked, the same way a real mismatch would.

    Each test sets the node's loaded site_config/_required_zones/
    _optional_zones/_zones_schema_valid/_map_hash directly, the same way
    other tests here drive _lifecycle or _task directly, rather than depend
    on whatever site_config.yaml happens to be installed.

    config_version defaults to the real config_identity_hash() of whatever
    site_id/map_id/config_id/zones this test declares, so "a positive test
    that proves a real content match" (Mohamed Sayed's review) is the
    default path every test here takes unless it explicitly overrides
    config_version to something else - most directly in
    test_wrong_but_nonempty_config_version_still_mismatches below.
    """

    def _configure(self, *, config_version=None, required_zones=(), optional_zones=(),
                   zones_schema_valid=True, map_hash='declared-hash', map_matches=True,
                   site_id='s', map_id='m', config_id='c'):
        required_zones = frozenset(required_zones)
        optional_zones = frozenset(optional_zones)
        if config_version is None:
            config_version = config_identity_hash(
                site_id=site_id, map_id=map_id, config_id=config_id,
                required_zones=required_zones, optional_zones=optional_zones)
        self.node._site_config = {
            'site_id': site_id, 'map_id': map_id, 'map_version': map_hash,
            'config_id': config_id, 'config_version': config_version,
        }
        self.node._required_zones = required_zones
        self.node._optional_zones = optional_zones
        self.node._zones_schema_valid = zones_schema_valid
        self.node._map_hash = map_hash if map_matches else 'actual-hash-does-not-match'
        self.node._map_reload_pending = False

    def test_blank_config_version_mismatches_even_when_the_map_matches(self):
        self._configure(config_version='')
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.CONFIG_ID_MISMATCH)

    def test_wrong_but_nonempty_config_version_still_mismatches(self):
        # Mohamed Sayed's platform-sw#73 review: bool(config_version) alone
        # used to pass here, since "CFG-1" is nonempty - this is the
        # negative test proving a real content comparison is now happening.
        self._configure(config_version='CFG-1-this-is-not-the-real-hash')
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.CONFIG_ID_MISMATCH)

    def test_correct_config_version_matches_real_declared_content(self):
        # the positive half of the above: the real hash of this site's own
        # declared identity/zone content is computed and matched exactly,
        # not merely "is this field present".
        self._configure(
            site_id='sim', map_id='sim-floor-1', config_id='sim-floor-1-config',
            required_zones=['charging_dock'], optional_zones=['loading_bay'])
        out = self.node._site_config_status(self.node._now_s())
        self.assertNotEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertNotEqual(out.reason, NavigationStatus.CONFIG_ID_MISMATCH)

    def test_editing_a_zone_list_without_regenerating_config_version_mismatches(self):
        # the whole point of hashing real content: a declared config_version
        # that was correct for a different set of zones must not still read
        # as matched once the zones below it change.
        self._configure(config_version=None, required_zones=['charging_dock'])
        stale_version = self.node._site_config['config_version']
        self.node._required_zones = frozenset(['charging_dock', 'new_zone'])
        self.node._site_config['config_version'] = stale_version
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.CONFIG_ID_MISMATCH)

    def test_populated_config_version_with_empty_zone_lists_is_fully_matched(self):
        self._configure()
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MATCHED)
        self.assertEqual(out.floor_coverage, SiteConfigStatus.COVERAGE_FULL)
        self.assertEqual(out.reason, NavigationStatus.NONE)

    def test_declared_required_zone_without_confirmation_blocks_readiness(self):
        # roll_up()'s own tests (test_status_rules.py) already cover that
        # COVERAGE_INSUFFICIENT blocks readiness in general; this is the
        # node-level half - that an unconfirmed required zone actually
        # produces COVERAGE_INSUFFICIENT, not a silent pass.
        self._configure(required_zones=['charging_dock'])
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MATCHED)
        self.assertEqual(out.floor_coverage, SiteConfigStatus.COVERAGE_INSUFFICIENT)
        self.assertEqual(out.reason, NavigationStatus.FLOOR_COVERAGE_INSUFFICIENT)

    def test_declared_optional_zone_without_confirmation_degrades_not_blocks(self):
        # same split as above: roll_up()'s tests cover that COVERAGE_PARTIAL
        # degrades without blocking; this confirms the node-level input that
        # feeds it - an unconfirmed optional zone, not a required one.
        self._configure(optional_zones=['loading_bay'])
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MATCHED)
        self.assertEqual(out.floor_coverage, SiteConfigStatus.COVERAGE_PARTIAL)
        self.assertEqual(out.reason, NavigationStatus.FLOOR_COVERAGE_OPTIONAL_GAP)

    def test_map_hash_mismatch_still_blocks_regardless_of_config_version(self):
        self._configure(map_matches=False)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.MAP_ID_MISMATCH)

    def test_invalid_zones_schema_blocks_with_unknown_not_a_silent_pass(self):
        # Mohamed Sayed's platform-sw#73 review: a missing/malformed zone
        # declaration must not silently evaluate as "no zones required".
        self._configure(zones_schema_valid=False)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_UNKNOWN)
        self.assertEqual(out.floor_coverage, SiteConfigStatus.COVERAGE_UNKNOWN)
        self.assertEqual(out.reason, NavigationStatus.SITE_CONFIG_UNKNOWN)
        # STATE_UNKNOWN blanks the identity fields too, per CONTRACT.md's
        # field-validity note - they are not meaningful once the schema
        # itself can't be trusted.
        self.assertEqual(out.map_id, '')
        self.assertEqual(out.config_id, '')

    # --- second-pass review (re-review of 364f01e): identity-field validation ---

    def test_omitted_identity_fields_block_with_unknown_not_a_silent_pass(self):
        # Mohamed Sayed's second-pass review: a profile missing
        # site_id/map_id/config_id entirely used to hash three empty
        # strings and could read as MATCHED/FULL against a declared
        # config_version of that same hash - this proves it can't any more.
        self._configure()
        del self.node._site_config['site_id']
        del self.node._site_config['map_id']
        del self.node._site_config['config_id']
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_UNKNOWN)
        self.assertEqual(out.reason, NavigationStatus.SITE_CONFIG_UNKNOWN)
        self.assertEqual(out.map_id, '')
        self.assertEqual(out.config_id, '')

    def test_wrong_type_identity_field_blocks_not_silently_stringified(self):
        # a numeric id used to be stringified for the hash and then assigned
        # unchanged to the output's string fields - this is the producer-
        # level case for exactly that.
        self._configure(map_id=7)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_UNKNOWN)
        self.assertEqual(out.reason, NavigationStatus.SITE_CONFIG_UNKNOWN)

    def test_a_null_identity_field_blocks_the_same_as_an_absent_one(self):
        self._configure()
        self.node._site_config['site_id'] = None
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_UNKNOWN)
        self.assertEqual(out.reason, NavigationStatus.SITE_CONFIG_UNKNOWN)


# A tiny, valid 2x2 PGM, same fixture test_map_identity.py uses - enough to
# exercise load_map_identity_source's file-reading path.
_TINY_PGM = b'P5\n2 2\n255\n' + bytes([10, 20, 30, 40])


def _occupancy_grid(*, resolution, origin_x, origin_y, origin_yaw=0.0, data=(0, 0, 0, 0)):
    """Build a real OccupancyGrid with just the fields _on_map reads."""
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.orientation.z = math.sin(origin_yaw / 2.0)
    msg.info.origin.orientation.w = math.cos(origin_yaw / 2.0)
    msg.data = list(data)
    return msg


class TestLiveMapReloadDetection(NodeTestCase):
    """
    Mohamed Sayed's platform-sw#73 review: live-loading map B through Nav2's
    LoadMap service while map_yaml_path still names file A left this node
    rereading A's unchanged content and reporting it MATCHED/FULL forever -
    the exact callback/producer-method reproduction his review describes.

    These write two real map.yaml + image fixtures (A, B) to disk and drive
    _on_map with a real OccupancyGrid built from B's own declared fields,
    with the node's map_yaml_path left pointed at A throughout - exactly the
    "A-to-B live reload regression with A unchanged" his review asked for.
    """

    def setUp(self):
        super().setUp()
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        with open(os.path.join(self.tmpdir.name, 'map.pgm'), 'wb') as f:
            f.write(_TINY_PGM)
        self.map_a_path = self._write_yaml('map_a.yaml', resolution=0.05, origin=[-1.0, -2.0, 0.0])
        self.map_b_path = self._write_yaml('map_b.yaml', resolution=0.1, origin=[3.0, 4.0, 0.0])

    def _write_yaml(self, name, *, resolution, origin):
        path = os.path.join(self.tmpdir.name, name)
        with open(path, 'w') as f:
            f.write('image: "map.pgm"\n')
            f.write(f'resolution: {resolution}\n')
            f.write(f'origin: {origin}\n')
            f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: "trinary"\n')
        return path

    def test_a_to_b_reload_with_a_unchanged_stops_reporting_a_as_matched(self):
        self.node._map_yaml_path = self.map_a_path
        self.node._map_hash = self.node._compute_map_hash()
        hash_a = self.node._map_hash
        self.assertIsNotNone(hash_a)  # sanity: A itself is readable
        self.node._site_config = {
            'site_id': 's', 'map_id': 'm', 'map_version': hash_a,
            'config_id': 'c', 'config_version': ''}
        self.node._required_zones = frozenset()
        self.node._optional_zones = frozenset()
        self.node._zones_schema_valid = True
        self.node._map_reload_pending = False

        # Nav2 LoadMap is told to load B; map_yaml_path (launch-time only,
        # never updated by a live reload) still names A. map_server
        # republishes on /map with B's own declared resolution/origin.
        self.node._on_map(_occupancy_grid(resolution=0.1, origin_x=3.0, origin_y=4.0))

        self.assertIsNone(self.node._map_hash)
        self.assertTrue(self.node._map_reload_pending)

        # consume the one-tick STALE pulse (same as any other reload)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_STALE)

        # map_server keeps the active map (B) on /map; map_yaml_path is
        # still unrevised, still naming A. The bug reported MATCHED again
        # here, since rereading A's unchanged file always reproduces hash_a.
        self.node._on_map(_occupancy_grid(resolution=0.1, origin_x=3.0, origin_y=4.0))
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.MAP_ID_MISMATCH)
        self.assertNotEqual(out.map_version, hash_a)

    def test_map_identity_is_not_matched_before_any_live_map_has_arrived(self):
        # Mohamed Sayed's platform-sw#73 review, fourth pass: self._map_hash
        # used to be populated straight from disk at __init__, so a freshly
        # started node reported MATCHED immediately - before _on_map had
        # ever run once - whenever declared_map_version happened to equal
        # the disk hash, which it normally does (both come from hashing the
        # same file). declared_map_version here is deliberately set to the
        # real disk hash of map_a, to show this isn't fixed by it merely
        # being wrong; the point is that NO live message has been checked.
        self.node._map_yaml_path = self.map_a_path
        hash_a = self.node._compute_map_hash()
        self.assertIsNotNone(hash_a)  # sanity: A itself is readable
        self.node._site_config = {
            'site_id': 's', 'map_id': 'm', 'map_version': hash_a,
            'config_id': 'c', 'config_version': ''}
        self.node._required_zones = frozenset()
        self.node._optional_zones = frozenset()
        self.node._zones_schema_valid = True
        self.node._map_reload_pending = False

        # no _on_map call at all - this is the very first tick after startup
        self.assertIsNone(self.node._map_hash)
        out = self.node._site_config_status(self.node._now_s())
        self.assertNotEqual(out.state, SiteConfigStatus.STATE_MATCHED)
        self.assertEqual(out.reason, NavigationStatus.MAP_ID_MISMATCH)

    def test_live_map_matching_the_declared_file_still_confirms_normally(self):
        # the positive case: no reload has happened, the live message's own
        # resolution/origin agree with map_yaml_path's file, so the existing
        # match behavior is unaffected by this check.
        self.node._map_yaml_path = self.map_a_path
        self.node._map_hash = None
        self.node._map_reload_pending = False
        self.node._on_map(_occupancy_grid(resolution=0.05, origin_x=-1.0, origin_y=-2.0))
        self.assertEqual(self.node._map_hash, self.node._compute_map_hash())

    def test_a_simple_repeated_publish_of_the_same_map_stays_matched(self):
        # map_server republishes transient-local on every new subscriber,
        # not only on a real reload - the exact same content arriving twice
        # must never be mistaken for a content change.
        self.node._map_yaml_path = self.map_a_path
        self.node._map_hash = None
        self.node._map_reload_pending = False
        same_data = (5, 10, 15, 20)
        self.node._on_map(_occupancy_grid(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, data=same_data))
        hash_after_first = self.node._map_hash
        self.assertIsNotNone(hash_after_first)
        self.node._on_map(_occupancy_grid(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, data=same_data))
        self.assertEqual(self.node._map_hash, hash_after_first)
        self.assertFalse(self.node._map_reload_pending)

    def test_same_geometry_different_content_stops_reporting_matched(self):
        # Mohamed Sayed's platform-sw#73 review, third pass: his
        # reproduction used two valid 2x2 maps with identical dimensions/
        # resolution/origin and one changed pixel - resolution/origin
        # agreement alone let this through before, since nothing compared
        # content. map_yaml_path still names A throughout; only the live
        # message's own cell data changes.
        self.node._map_yaml_path = self.map_a_path
        self.node._map_hash = self.node._compute_map_hash()
        hash_a = self.node._map_hash
        self.assertIsNotNone(hash_a)
        # a real computed config_version, not '' - an empty one would mismatch
        # on config identity alone regardless of the map, which would make
        # the MATCHED check below meaningless (config_identity_matches would
        # fail even once the map genuinely does).
        config_version = config_identity_hash(
            site_id='s', map_id='m', config_id='c',
            required_zones=frozenset(), optional_zones=frozenset())
        self.node._site_config = {
            'site_id': 's', 'map_id': 'm', 'map_version': hash_a,
            'config_id': 'c', 'config_version': config_version}
        self.node._required_zones = frozenset()
        self.node._optional_zones = frozenset()
        self.node._zones_schema_valid = True
        self.node._map_reload_pending = False

        # first message: same geometry as A, and this is the first live
        # confirmation, so its content is the one trusted/recorded
        self.node._on_map(_occupancy_grid(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, data=(0, 0, 0, 0)))
        self.assertEqual(self.node._map_hash, hash_a)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MATCHED)

        # second message: same geometry, but different cell content - a
        # same-dimensions replacement map, exactly Mohamed's reproduction.
        # map_yaml_path is unrevised and still reads as A either way, so
        # only the content fingerprint can catch this. Invalidating the hash
        # here holds STATE_STALE for exactly one tick (the reload_pending
        # pulse, same as any other map change) before the mismatch itself is
        # what the next evaluation reports.
        self.node._on_map(_occupancy_grid(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, data=(100, 0, 0, 0)))
        self.assertIsNone(self.node._map_hash)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_STALE)  # one-tick pulse
        self.assertEqual(out.reason, NavigationStatus.MAP_STALE)

        # readiness is actually blocked, not just STATE_STALE for one tick
        self.node._on_map(_occupancy_grid(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, data=(100, 0, 0, 0)))
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertNotEqual(out.map_version, hash_a)


class TestSiteConfigOutputSanitization(NodeTestCase):
    """
    Mohamed Sayed's platform-sw#73 review, third pass: identity_fields_valid
    is checked after reload_pending/map_identity_matches in
    evaluate_site_config's precedence order, so a malformed map_id/config_id
    could still reach _site_config_status's output assignment whenever one
    of those earlier faults was the one reported - the fault precedence
    itself is correct and unchanged (see evaluate_site_config's docstring),
    but the output fields must be sanitized independently of which fault won.
    """

    def _configure(self, *, map_id, config_id='c', map_matches=True, config_version=''):
        self.node._site_config = {
            'site_id': 's', 'map_id': map_id, 'map_version': 'declared-hash',
            'config_id': config_id, 'config_version': config_version}
        self.node._required_zones = frozenset()
        self.node._optional_zones = frozenset()
        self.node._zones_schema_valid = True
        self.node._map_hash = 'declared-hash' if map_matches else 'actual-hash-does-not-match'
        self.node._map_reload_pending = False

    def test_malformed_map_id_is_sanitized_even_under_a_map_mismatch(self):
        self._configure(map_id=7, map_matches=False)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.reason, NavigationStatus.MAP_ID_MISMATCH)
        self.assertEqual(out.map_id, '')
        self.assertIsInstance(out.map_id, str)

    def test_malformed_config_id_is_sanitized_even_under_a_map_mismatch(self):
        self._configure(map_id='m', config_id=None, map_matches=False)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_MISMATCHED)
        self.assertEqual(out.config_id, '')
        self.assertIsInstance(out.config_id, str)

    def test_malformed_map_id_is_sanitized_under_a_reload_pending_stale(self):
        self._configure(map_id=7)
        self.node._map_reload_pending = True
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.state, SiteConfigStatus.STATE_STALE)
        self.assertEqual(out.map_id, '')
        self.assertIsInstance(out.map_id, str)

    def test_wrong_type_config_version_never_reaches_the_output_field(self):
        # a non-string config_version used to pass `or ''` unchanged (any
        # truthy value does), reaching the ROS string field as-is.
        self._configure(map_id='m', config_version=7)
        out = self.node._site_config_status(self.node._now_s())
        self.assertEqual(out.config_version, '')
        self.assertIsInstance(out.config_version, str)


if __name__ == '__main__':
    unittest.main(verbosity=2)
