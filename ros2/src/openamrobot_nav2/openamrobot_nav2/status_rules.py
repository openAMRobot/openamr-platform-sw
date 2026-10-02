"""
Rules that turn tracker output into the values in NavigationStatus.

Plain functions over plain values, so the readiness, reason and publishing rules
can be tested without a running graph.
"""

from collections import namedtuple

from openamr_nav_msgs.msg import (
    LocalizationStatus,
    MotionSourceCoverage,
    NavigationStatus,
    NavStackStatus,
    ProtectionStatus,
    SensorStatus,
)

# keys every sensor entry in the profile needs
SENSOR_KEYS = ('id', 'kind', 'topic', 'frame_id', 'required', 'nominal_period_s')

Rollup = namedtuple('Rollup', 'health readiness not_ready_reasons active_reasons')


def _unique(reasons):
    """Return the reasons that are not NONE, once each, in order."""
    seen = []
    for reason in reasons:
        if reason != NavigationStatus.NONE and reason not in seen:
            seen.append(reason)
    return seen


def sensor_reason(state, ever_seen):
    """Return the reason for a sensor state. Only OK carries NONE."""
    if state == SensorStatus.STATE_OK:
        return NavigationStatus.NONE
    if state == SensorStatus.STATE_STALE:
        return NavigationStatus.SENSOR_DATA_STALE
    if state == SensorStatus.STATE_ABSENT:
        return NavigationStatus.SENSOR_NOT_CONFIGURED
    if state == SensorStatus.STATE_UNKNOWN:
        # data that can not be judged means the profile has no stale threshold
        if ever_seen:
            return NavigationStatus.NAV_CONFIG_FAULT
        return NavigationStatus.SENSOR_NO_DATA_YET
    return NavigationStatus.SENSOR_DATA_INVALID


def motion_source_reason(coverage):
    """Return the reason for a motion source's collision-monitor coverage."""
    if coverage == MotionSourceCoverage.COVERAGE_PROTECTED:
        return NavigationStatus.NONE
    if coverage in (MotionSourceCoverage.COVERAGE_UNPROTECTED,
                    MotionSourceCoverage.COVERAGE_PARTIAL):
        # There is no reason for "partly protected" yet, so PARTIAL reports the
        # stricter one until a MOTION_SOURCE_PARTIALLY_PROTECTED code exists.
        return NavigationStatus.MOTION_SOURCE_UNPROTECTED
    return NavigationStatus.PROTECTION_STATE_UNKNOWN


def rear_coverage_reason(rear_coverage):
    """Return the reason for the rear coverage value."""
    if rear_coverage == ProtectionStatus.REAR_COVERAGE_AVAILABLE:
        return NavigationStatus.NONE
    if rear_coverage == ProtectionStatus.REAR_COVERAGE_UNAVAILABLE:
        return NavigationStatus.REAR_COVERAGE_UNAVAILABLE
    return NavigationStatus.PROTECTION_STATE_UNKNOWN


def evaluate_localization(*, pose_received, pose_age_s, covariance_xy, covariance_yaw,
                          tf_ok, correction_paused, moving, thresholds):
    """
    Return (state, health, reason) for localization.

    moving is True, False or None when it can not be told. A threshold that is
    missing leaves the check it belongs to undecided, so the result is UNKNOWN,
    never OK.
    """
    unknown = (LocalizationStatus.STATE_UNKNOWN, LocalizationStatus.HEALTH_UNKNOWN)
    if not pose_received:
        return unknown + (NavigationStatus.LOCALIZATION_NOT_INITIALIZED,)
    if not tf_ok:
        return (LocalizationStatus.STATE_LOST, LocalizationStatus.HEALTH_FAULT,
                NavigationStatus.TF_UNAVAILABLE)
    if correction_paused:
        # correction is off while docking, so the pose is not being checked
        return unknown + (NavigationStatus.LOCALIZATION_PAUSED_FOR_DOCKING,)

    max_age = thresholds.get('pose_age_moving_s')
    if moving is None or max_age is None:
        return unknown + (NavigationStatus.LOCALIZATION_UNCERTAIN,)
    if moving and pose_age_s > max_age:
        return (LocalizationStatus.STATE_DEGRADED, LocalizationStatus.HEALTH_DEGRADED,
                NavigationStatus.LOCALIZATION_STALE_WHILE_MOVING)

    max_xy = thresholds.get('covariance_xy_max')
    max_yaw = thresholds.get('covariance_yaw_max')
    if max_xy is None or max_yaw is None:
        return unknown + (NavigationStatus.LOCALIZATION_UNCERTAIN,)
    if covariance_xy > max_xy or covariance_yaw > max_yaw:
        return (LocalizationStatus.STATE_DEGRADED, LocalizationStatus.HEALTH_DEGRADED,
                NavigationStatus.LOCALIZATION_UNCERTAIN)
    return (LocalizationStatus.STATE_OK, LocalizationStatus.HEALTH_OK,
            NavigationStatus.NONE)


def roll_up(*, config_fault, stack_state, stack_reason, sensors,
            localization_state, localization_reason, other_reasons, base_reasons):
    """
    Combine the group results into health, readiness and the two reason lists.

    sensors is a list of (required, state, reason). other_reasons are the reasons
    from the task, recovery and protection groups, and base_reasons are the ones
    from the I8 dependency. Readiness does not depend on the reason lists: a
    blocked component blocks it even if its reason were missing.
    """
    blocked = bool(
        config_fault
        or base_reasons
        or stack_state != NavStackStatus.STATE_ACTIVE
        or any(required and state != SensorStatus.STATE_OK
               for required, state, _ in sensors)
        or localization_state != LocalizationStatus.STATE_OK)

    blocking_reasons = []
    if config_fault:
        blocking_reasons.append(NavigationStatus.NAV_CONFIG_FAULT)
    blocking_reasons.extend(base_reasons)
    if stack_state != NavStackStatus.STATE_ACTIVE:
        blocking_reasons.append(stack_reason)
    for required, state, reason in sensors:
        if required and state != SensorStatus.STATE_OK:
            blocking_reasons.append(reason)
    if localization_state != LocalizationStatus.STATE_OK:
        blocking_reasons.append(localization_reason)
    not_ready_reasons = _unique(blocking_reasons)

    active_reasons = _unique(
        not_ready_reasons
        + [stack_reason, localization_reason]
        + [reason for _, _, reason in sensors]
        + list(other_reasons))

    if config_fault:
        health = NavigationStatus.HEALTH_FAULT
    elif stack_state == NavStackStatus.STATE_UNKNOWN:
        health = NavigationStatus.HEALTH_UNKNOWN
    elif blocked:
        health = NavigationStatus.HEALTH_DEGRADED
    else:
        health = NavigationStatus.HEALTH_OK

    if blocked:
        readiness = NavigationStatus.NAVIGATION_READINESS_NOT_READY
    else:
        readiness = NavigationStatus.NAVIGATION_READINESS_READY
    return Rollup(health, readiness, not_ready_reasons, active_reasons)


def status_signature(msg):
    """
    Return the parts of a NavigationStatus that count as a change.

    Ages, rates, covariance and distance move on every message, so they are left
    out: they would make every status look new. They still go out with each
    heartbeat.
    """
    return (
        msg.contract_version, msg.profile_id, msg.thresholds_id,
        msg.health, msg.navigation_readiness,
        tuple(msg.not_ready_reasons), tuple(msg.active_reasons), msg.sensor_gaps,
        msg.stack.state, msg.stack.reason, tuple(msg.stack.inactive_nodes),
        msg.localization.state, msg.localization.health, msg.localization.reason,
        msg.localization.tf_available, msg.localization.correction_paused,
        tuple((s.id, s.state, s.reason) for s in msg.sensors),
        msg.task.state, msg.task.reason, msg.task.native_error_code,
        (msg.task.goal_stamp.sec, msg.task.goal_stamp.nanosec),
        msg.recovery.action, msg.recovery.attempt, msg.recovery.attempt_limit,
        msg.recovery.reason,
        msg.protection.collision_monitor, msg.protection.collision_monitor_reason,
        tuple((m.source, m.coverage, m.reason) for m in msg.protection.motion_sources),
        msg.protection.rear_coverage, msg.protection.rear_coverage_reason,
        tuple((c.type, c.value, c.reason) for c in msg.constraints),
    )


class PublishGate:
    """
    Publish right away when something changed, otherwise once per heartbeat.

    The check runs on a timer, so an elapsed time a hair under the heartbeat
    period still counts as due. Without that allowance the heartbeat slips to the
    next timer tick and runs slower than asked.
    """

    def __init__(self, heartbeat_period, tolerance=0.0):
        self.heartbeat_period = float(heartbeat_period)
        self.tolerance = float(tolerance)
        self._last_signature = None
        self._last_time = None

    def should_publish(self, signature, now):
        """Return True if the status should go out now, and remember that it did."""
        due = (
            self._last_time is None
            or signature != self._last_signature
            or now - self._last_time >= self.heartbeat_period - self.tolerance)
        if due:
            self._last_signature = signature
            self._last_time = now
        return due


def validate_profile(profile):
    """Return the problems that make a profile unusable. Empty means it is fine."""
    if not isinstance(profile, dict) or not profile:
        return ['the profile is empty or not a mapping']
    problems = []
    sensors = profile.get('sensors') or []
    if not sensors:
        problems.append('no sensors declared')
        return problems
    for sensor in sensors:
        missing = [key for key in SENSOR_KEYS if key not in sensor]
        if missing:
            problems.append(
                f"sensor {sensor.get('id', '?')} is missing {', '.join(missing)}")
    if not any(sensor.get('required') for sensor in sensors):
        problems.append('no required sensor declared')
    return problems
