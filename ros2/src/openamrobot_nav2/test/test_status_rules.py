"""
Tests for the readiness roll-up, reason rules, publish gate and profile checks.

The readiness tests are mostly negative: a stale or missing required sensor, or
localization that is lost, unknown or degraded, must keep navigation NOT_READY
even when everything else, including the I8 dependency, is fine.

Skipped if openamr_nav_msgs (openamrobot-interfaces) isn't installed.
"""

import copy
from types import SimpleNamespace
import unittest

import pytest

pytest.importorskip('openamr_nav_msgs')

from openamr_nav_msgs.msg import (  # noqa: E402,I100
    LocalizationStatus,
    MotionSourceCoverage,
    NavigationStatus,
    NavStackStatus,
    ProtectionStatus,
    SensorStatus,
)
from openamrobot_nav2.status_rules import (  # noqa: E402
    evaluate_localization,
    motion_source_reason,
    PublishGate,
    rear_coverage_reason,
    roll_up,
    sensor_reason,
    status_signature,
    validate_profile,
)

NONE = NavigationStatus.NONE
READY = NavigationStatus.NAVIGATION_READINESS_READY
NOT_READY = NavigationStatus.NAVIGATION_READINESS_NOT_READY

THRESHOLDS = {
    'pose_age_moving_s': 5.0,
    'covariance_xy_max': 0.5,
    'covariance_yaw_max': 0.3,
}


def good(**overrides):
    """Return roll_up arguments for a healthy navigation stack."""
    args = {
        'config_fault': False,
        'stack_state': NavStackStatus.STATE_ACTIVE,
        'stack_reason': NONE,
        'sensors': [
            (True, SensorStatus.STATE_OK, NONE),
            (True, SensorStatus.STATE_OK, NONE),
            (False, SensorStatus.STATE_ABSENT, NavigationStatus.SENSOR_NOT_CONFIGURED),
        ],
        'localization_state': LocalizationStatus.STATE_OK,
        'localization_reason': NONE,
        'other_reasons': [],
        'base_reasons': [],
    }
    args.update(overrides)
    return args


class TestSensorReason(unittest.TestCase):

    def test_ok_carries_none(self):
        self.assertEqual(sensor_reason(SensorStatus.STATE_OK, True), NONE)

    def test_stale_and_absent(self):
        self.assertEqual(sensor_reason(SensorStatus.STATE_STALE, True),
                         NavigationStatus.SENSOR_DATA_STALE)
        self.assertEqual(sensor_reason(SensorStatus.STATE_ABSENT, False),
                         NavigationStatus.SENSOR_NOT_CONFIGURED)

    def test_required_sensor_that_never_published_says_no_data_yet(self):
        self.assertEqual(sensor_reason(SensorStatus.STATE_UNKNOWN, False),
                         NavigationStatus.SENSOR_NO_DATA_YET)

    def test_data_that_can_not_be_judged_is_a_config_fault(self):
        self.assertEqual(sensor_reason(SensorStatus.STATE_UNKNOWN, True),
                         NavigationStatus.NAV_CONFIG_FAULT)

    def test_no_state_other_than_ok_carries_none(self):
        for state in (SensorStatus.STATE_UNKNOWN, SensorStatus.STATE_ABSENT,
                      SensorStatus.STATE_STALE, SensorStatus.STATE_DEGRADED):
            for ever_seen in (False, True):
                self.assertNotEqual(sensor_reason(state, ever_seen), NONE)


class TestProtectionReasons(unittest.TestCase):

    def test_only_protected_carries_none(self):
        self.assertEqual(
            motion_source_reason(MotionSourceCoverage.COVERAGE_PROTECTED), NONE)
        for coverage in (MotionSourceCoverage.COVERAGE_UNKNOWN,
                         MotionSourceCoverage.COVERAGE_UNPROTECTED,
                         MotionSourceCoverage.COVERAGE_PARTIAL):
            self.assertNotEqual(motion_source_reason(coverage), NONE)

    def test_partial_is_reported_as_the_stricter_reason(self):
        self.assertEqual(
            motion_source_reason(MotionSourceCoverage.COVERAGE_PARTIAL),
            NavigationStatus.MOTION_SOURCE_UNPROTECTED)

    def test_unknown_coverage_says_the_state_is_unknown(self):
        self.assertEqual(
            motion_source_reason(MotionSourceCoverage.COVERAGE_UNKNOWN),
            NavigationStatus.PROTECTION_STATE_UNKNOWN)

    def test_rear_coverage(self):
        self.assertEqual(
            rear_coverage_reason(ProtectionStatus.REAR_COVERAGE_AVAILABLE), NONE)
        self.assertEqual(
            rear_coverage_reason(ProtectionStatus.REAR_COVERAGE_UNAVAILABLE),
            NavigationStatus.REAR_COVERAGE_UNAVAILABLE)
        self.assertEqual(
            rear_coverage_reason(ProtectionStatus.REAR_COVERAGE_UNKNOWN),
            NavigationStatus.PROTECTION_STATE_UNKNOWN)


def localize(**overrides):
    args = {
        'pose_received': True, 'pose_age_s': 1.0,
        'covariance_xy': 0.1, 'covariance_yaw': 0.1,
        'tf_ok': True, 'correction_paused': False, 'moving': False,
        'thresholds': THRESHOLDS,
    }
    args.update(overrides)
    return evaluate_localization(**args)


class TestEvaluateLocalization(unittest.TestCase):

    def test_good_pose_is_ok(self):
        self.assertEqual(
            localize(),
            (LocalizationStatus.STATE_OK, LocalizationStatus.HEALTH_OK, NONE))

    def test_no_pose_yet(self):
        self.assertEqual(
            localize(pose_received=False),
            (LocalizationStatus.STATE_UNKNOWN, LocalizationStatus.HEALTH_UNKNOWN,
             NavigationStatus.LOCALIZATION_NOT_INITIALIZED))

    def test_missing_transform_is_lost(self):
        self.assertEqual(
            localize(tf_ok=False),
            (LocalizationStatus.STATE_LOST, LocalizationStatus.HEALTH_FAULT,
             NavigationStatus.TF_UNAVAILABLE))

    def test_paused_for_docking_is_not_reported_as_ok(self):
        state, _, reason = localize(correction_paused=True)
        self.assertEqual(state, LocalizationStatus.STATE_UNKNOWN)
        self.assertEqual(reason, NavigationStatus.LOCALIZATION_PAUSED_FOR_DOCKING)

    def test_old_pose_while_moving_is_degraded(self):
        state, _, reason = localize(moving=True, pose_age_s=6.0)
        self.assertEqual(state, LocalizationStatus.STATE_DEGRADED)
        self.assertEqual(reason, NavigationStatus.LOCALIZATION_STALE_WHILE_MOVING)

    def test_old_pose_while_parked_is_fine(self):
        state, _, _ = localize(moving=False, pose_age_s=600.0)
        self.assertEqual(state, LocalizationStatus.STATE_OK)

    def test_large_covariance_is_degraded(self):
        for kwargs in ({'covariance_xy': 0.9}, {'covariance_yaw': 0.9}):
            state, _, reason = localize(**kwargs)
            self.assertEqual(state, LocalizationStatus.STATE_DEGRADED)
            self.assertEqual(reason, NavigationStatus.LOCALIZATION_UNCERTAIN)

    def test_unset_covariance_limits_leave_it_unknown_never_ok(self):
        limits = {'pose_age_moving_s': 5.0}
        state, health, reason = localize(thresholds=limits)
        self.assertEqual(state, LocalizationStatus.STATE_UNKNOWN)
        self.assertEqual(health, LocalizationStatus.HEALTH_UNKNOWN)
        self.assertEqual(reason, NavigationStatus.LOCALIZATION_UNCERTAIN)

    def test_one_unset_covariance_limit_is_enough_to_stay_unknown(self):
        limits = dict(THRESHOLDS)
        del limits['covariance_yaw_max']
        self.assertEqual(localize(thresholds=limits)[0], LocalizationStatus.STATE_UNKNOWN)

    def test_unset_pose_age_limit_or_unknown_motion_leaves_it_unknown(self):
        limits = {k: v for k, v in THRESHOLDS.items() if k != 'pose_age_moving_s'}
        self.assertEqual(localize(thresholds=limits)[0], LocalizationStatus.STATE_UNKNOWN)
        self.assertEqual(localize(moving=None)[0], LocalizationStatus.STATE_UNKNOWN)

    def test_no_threshold_at_all_can_never_produce_ok(self):
        self.assertNotEqual(localize(thresholds={})[0], LocalizationStatus.STATE_OK)

    def test_nothing_but_ok_carries_none(self):
        for pose_received in (False, True):
            for tf_ok in (False, True):
                for paused in (False, True):
                    for moving in (None, False, True):
                        for age in (1.0, 9.0):
                            for cov in (0.1, 0.9):
                                for limits in (THRESHOLDS, {}):
                                    state, _, reason = localize(
                                        pose_received=pose_received, tf_ok=tf_ok,
                                        correction_paused=paused, moving=moving,
                                        pose_age_s=age, covariance_xy=cov,
                                        thresholds=limits)
                                    if state != LocalizationStatus.STATE_OK:
                                        self.assertNotEqual(reason, NONE)


class TestRollUp(unittest.TestCase):

    def test_everything_fine_is_ready(self):
        result = roll_up(**good())
        self.assertEqual(result.readiness, READY)
        self.assertEqual(result.health, NavigationStatus.HEALTH_OK)
        self.assertEqual(result.not_ready_reasons, [])

    def test_base_link_lost_blocks_readiness(self):
        result = roll_up(**good(base_reasons=[NavigationStatus.BASE_LINK_LOST]))
        self.assertEqual(result.readiness, NOT_READY)
        self.assertEqual(result.health, NavigationStatus.HEALTH_DEGRADED)
        self.assertIn(NavigationStatus.BASE_LINK_LOST, result.not_ready_reasons)

    def test_stale_required_sensor_blocks_readiness(self):
        sensors = [(True, SensorStatus.STATE_STALE, NavigationStatus.SENSOR_DATA_STALE),
                   (True, SensorStatus.STATE_OK, NONE)]
        result = roll_up(**good(sensors=sensors))
        self.assertEqual(result.readiness, NOT_READY)
        self.assertIn(NavigationStatus.SENSOR_DATA_STALE, result.not_ready_reasons)

    def test_required_sensor_that_never_published_blocks_readiness(self):
        sensors = [(True, SensorStatus.STATE_UNKNOWN, NavigationStatus.SENSOR_NO_DATA_YET)]
        result = roll_up(**good(sensors=sensors))
        self.assertEqual(result.readiness, NOT_READY)
        self.assertIn(NavigationStatus.SENSOR_NO_DATA_YET, result.not_ready_reasons)

    def test_absent_required_sensor_blocks_readiness(self):
        sensors = [(True, SensorStatus.STATE_ABSENT, NavigationStatus.SENSOR_NOT_CONFIGURED)]
        self.assertEqual(roll_up(**good(sensors=sensors)).readiness, NOT_READY)

    def test_optional_sensor_problems_do_not_block_but_are_still_active(self):
        sensors = [(True, SensorStatus.STATE_OK, NONE),
                   (False, SensorStatus.STATE_STALE, NavigationStatus.SENSOR_DATA_STALE)]
        result = roll_up(**good(sensors=sensors))
        self.assertEqual(result.readiness, READY)
        self.assertNotIn(NavigationStatus.SENSOR_DATA_STALE, result.not_ready_reasons)
        self.assertIn(NavigationStatus.SENSOR_DATA_STALE, result.active_reasons)

    def test_lost_unknown_or_degraded_localization_blocks_readiness(self):
        for state, reason in (
                (LocalizationStatus.STATE_LOST, NavigationStatus.TF_UNAVAILABLE),
                (LocalizationStatus.STATE_UNKNOWN,
                 NavigationStatus.LOCALIZATION_NOT_INITIALIZED),
                (LocalizationStatus.STATE_DEGRADED, NavigationStatus.LOCALIZATION_UNCERTAIN)):
            result = roll_up(**good(localization_state=state, localization_reason=reason))
            self.assertEqual(result.readiness, NOT_READY)
            self.assertIn(reason, result.not_ready_reasons)

    def test_stack_that_is_not_active_blocks_readiness(self):
        for state, reason in (
                (NavStackStatus.STATE_RESETTING, NavigationStatus.NAV_STACK_RESETTING),
                (NavStackStatus.STATE_UNKNOWN, NavigationStatus.NAV_STACK_NOT_STARTED)):
            result = roll_up(**good(stack_state=state, stack_reason=reason))
            self.assertEqual(result.readiness, NOT_READY)

    def test_unknown_stack_makes_health_unknown(self):
        result = roll_up(**good(stack_state=NavStackStatus.STATE_UNKNOWN,
                                stack_reason=NavigationStatus.NAV_STACK_NOT_STARTED))
        self.assertEqual(result.health, NavigationStatus.HEALTH_UNKNOWN)

    def test_failed_stack_is_a_fault_not_just_degraded(self):
        result = roll_up(**good(stack_state=NavStackStatus.STATE_FAILED,
                                stack_reason=NavigationStatus.NAV_STACK_RESETTING))
        self.assertEqual(result.health, NavigationStatus.HEALTH_FAULT)
        self.assertEqual(result.readiness, NOT_READY)
        self.assertIn(NavigationStatus.NAV_STACK_RESETTING, result.not_ready_reasons)

    def test_config_fault_is_not_ready_and_a_fault(self):
        result = roll_up(**good(config_fault=True))
        self.assertEqual(result.readiness, NOT_READY)
        self.assertEqual(result.health, NavigationStatus.HEALTH_FAULT)
        self.assertIn(NavigationStatus.NAV_CONFIG_FAULT, result.not_ready_reasons)
        self.assertIn(NavigationStatus.NAV_CONFIG_FAULT, result.active_reasons)

    def test_readiness_does_not_depend_on_the_reason_being_filled_in(self):
        # a blocked component blocks even if its reason were NONE by mistake
        result = roll_up(**good(sensors=[(True, SensorStatus.STATE_STALE, NONE)]))
        self.assertEqual(result.readiness, NOT_READY)

    def test_active_reasons_cover_every_group(self):
        result = roll_up(**good(
            stack_state=NavStackStatus.STATE_RESETTING,
            stack_reason=NavigationStatus.NAV_STACK_RESETTING,
            sensors=[(True, SensorStatus.STATE_STALE, NavigationStatus.SENSOR_DATA_STALE),
                     (False, SensorStatus.STATE_ABSENT,
                      NavigationStatus.SENSOR_NOT_CONFIGURED)],
            localization_state=LocalizationStatus.STATE_UNKNOWN,
            localization_reason=NavigationStatus.LOCALIZATION_NOT_INITIALIZED,
            other_reasons=[NavigationStatus.RECOVERY_IN_PROGRESS,
                           NavigationStatus.COLLISION_LAYER_INACTIVE,
                           NavigationStatus.MOTION_SOURCE_UNPROTECTED,
                           NavigationStatus.REAR_COVERAGE_UNAVAILABLE],
            base_reasons=[NavigationStatus.BASE_LINK_LOST]))
        for reason in (NavigationStatus.BASE_LINK_LOST,
                       NavigationStatus.NAV_STACK_RESETTING,
                       NavigationStatus.SENSOR_DATA_STALE,
                       NavigationStatus.SENSOR_NOT_CONFIGURED,
                       NavigationStatus.LOCALIZATION_NOT_INITIALIZED,
                       NavigationStatus.RECOVERY_IN_PROGRESS,
                       NavigationStatus.COLLISION_LAYER_INACTIVE,
                       NavigationStatus.MOTION_SOURCE_UNPROTECTED,
                       NavigationStatus.REAR_COVERAGE_UNAVAILABLE):
            self.assertIn(reason, result.active_reasons)

    def test_active_reasons_have_no_duplicates_and_no_none(self):
        result = roll_up(**good(
            other_reasons=[NavigationStatus.MOTION_SOURCE_UNPROTECTED] * 4 + [NONE],
            base_reasons=[NavigationStatus.BASE_LINK_LOST]))
        self.assertEqual(len(result.active_reasons), len(set(result.active_reasons)))
        self.assertNotIn(NONE, result.active_reasons)
        self.assertNotIn(NONE, result.not_ready_reasons)


class TestPublishGate(unittest.TestCase):

    def test_first_status_goes_out(self):
        self.assertTrue(PublishGate(0.5).should_publish('a', 0.0))

    def test_unchanged_status_waits_for_the_heartbeat(self):
        gate = PublishGate(0.5)
        gate.should_publish('a', 0.0)
        self.assertFalse(gate.should_publish('a', 0.1))
        self.assertFalse(gate.should_publish('a', 0.4))
        self.assertTrue(gate.should_publish('a', 0.5))

    def test_a_change_goes_out_right_away(self):
        gate = PublishGate(0.5)
        gate.should_publish('a', 0.0)
        self.assertTrue(gate.should_publish('b', 0.1))

    def test_heartbeat_counts_from_the_last_publish(self):
        gate = PublishGate(0.5)
        gate.should_publish('a', 0.0)
        gate.should_publish('b', 0.3)
        self.assertFalse(gate.should_publish('b', 0.6))
        self.assertTrue(gate.should_publish('b', 0.8))

    def test_heartbeat_tolerates_a_little_timer_jitter(self):
        gate = PublishGate(0.5, tolerance=0.05)
        gate.should_publish('a', 0.0)
        self.assertFalse(gate.should_publish('a', 0.4))
        self.assertTrue(gate.should_publish('a', 0.49))

    def test_heartbeat_keeps_its_rate_when_the_timer_runs_a_little_fast(self):
        # a 10 Hz check that ticks every 0.0996 s reaches 0.498 s after five
        # ticks, just short of the 0.5 s heartbeat
        gate = PublishGate(0.5, tolerance=0.05)
        published = sum(gate.should_publish('a', k * 0.0996) for k in range(201))
        self.assertGreaterEqual(published / (200 * 0.0996), 1.95)


def make_msg():
    return SimpleNamespace(
        contract_version=1, profile_id='sim', thresholds_id='sim-v1',
        health=2, navigation_readiness=0,
        not_ready_reasons=[9001], active_reasons=[9001], sensor_gaps=1,
        stack=SimpleNamespace(state=3, reason=0, inactive_nodes=[]),
        localization=SimpleNamespace(
            state=0, health=0, reason=3001, tf_available=True, correction_paused=False,
            pose_age=SimpleNamespace(sec=0, nanosec=0),
            covariance_xy=0.0, covariance_yaw=0.0),
        sensors=[SimpleNamespace(id='odom', state=4, reason=0,
                                 data_age=SimpleNamespace(sec=0, nanosec=0),
                                 rate_hz=50.0)],
        task=SimpleNamespace(
            state=0, reason=0, native_error_code=0,
            goal_stamp=SimpleNamespace(sec=0, nanosec=0), distance_remaining=0.0),
        recovery=SimpleNamespace(action=0, attempt=0, attempt_limit=6, reason=0),
        protection=SimpleNamespace(
            collision_monitor=2, collision_monitor_reason=0,
            motion_sources=[SimpleNamespace(source=1, coverage=2, reason=6002)],
            rear_coverage=1, rear_coverage_reason=6003),
        constraints=[])


class TestStatusSignature(unittest.TestCase):

    def test_values_that_move_on_every_message_do_not_count_as_a_change(self):
        a = make_msg()
        b = copy.deepcopy(a)
        b.sensors[0].data_age = SimpleNamespace(sec=0, nanosec=20000000)
        b.sensors[0].rate_hz = 49.7
        b.localization.pose_age = SimpleNamespace(sec=9, nanosec=0)
        b.localization.covariance_xy = 0.4
        b.localization.covariance_yaw = 0.2
        b.task.distance_remaining = 3.2
        self.assertEqual(status_signature(a), status_signature(b))

    def test_real_changes_do_count(self):
        base = status_signature(make_msg())
        changes = {
            'health': lambda m: setattr(m, 'health', 1),
            'readiness': lambda m: setattr(m, 'navigation_readiness', 1),
            'active reasons': lambda m: setattr(m, 'active_reasons', [9001, 2001]),
            'stack state': lambda m: setattr(m.stack, 'state', 4),
            'stack nodes': lambda m: setattr(m.stack, 'inactive_nodes', ['amcl']),
            'localization': lambda m: setattr(m.localization, 'state', 3),
            'sensor state': lambda m: setattr(m.sensors[0], 'state', 2),
            'sensor reason': lambda m: setattr(m.sensors[0], 'reason', 2001),
            'task state': lambda m: setattr(m.task, 'state', 2),
            'task goal': lambda m: setattr(m.task.goal_stamp, 'sec', 12),
            'error code': lambda m: setattr(m.task, 'native_error_code', 204),
            'recovery attempt': lambda m: setattr(m.recovery, 'attempt', 1),
            'collision monitor': lambda m: setattr(m.protection, 'collision_monitor', 1),
            'coverage': lambda m: setattr(m.protection.motion_sources[0], 'coverage', 3),
        }
        for name, change in changes.items():
            msg = make_msg()
            change(msg)
            self.assertNotEqual(status_signature(msg), base, name)


class TestValidateProfile(unittest.TestCase):

    SENSOR = {'id': 'odom', 'kind': 'ODOMETRY', 'topic': '/odom', 'frame_id': 'odom',
              'required': True, 'nominal_period_s': 0.02}

    def test_a_usable_profile_has_no_problems(self):
        self.assertEqual(validate_profile({'sensors': [self.SENSOR]}), [])

    def test_empty_or_broken_profiles_are_problems(self):
        for profile in (None, {}, [], 'text'):
            self.assertTrue(validate_profile(profile))

    def test_a_profile_without_sensors_is_a_problem(self):
        self.assertTrue(validate_profile({'sensors': []}))
        self.assertTrue(validate_profile({'profile_id': 'sim'}))

    def test_a_profile_without_a_required_sensor_is_a_problem(self):
        optional = dict(self.SENSOR, required=False)
        problems = validate_profile({'sensors': [optional]})
        self.assertIn('no required sensor declared', problems)

    def test_a_sensor_missing_a_key_is_a_problem(self):
        broken = {k: v for k, v in self.SENSOR.items() if k != 'topic'}
        problems = validate_profile({'sensors': [self.SENSOR, broken]})
        self.assertTrue(any('topic' in problem for problem in problems))


if __name__ == '__main__':
    unittest.main(verbosity=2)
