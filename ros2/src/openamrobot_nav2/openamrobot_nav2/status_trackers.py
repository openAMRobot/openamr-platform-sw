"""
State trackers behind navigation_status_node.

They hold no ROS handles and take time as plain seconds, so they can be tested
without a running graph.
"""

from collections import deque

from openamr_nav_msgs.msg import (
    NavigationStatus,
    NavTaskStatus,
    RecoveryStatus,
    SensorStatus,
)

# GoalStatus codes from action_msgs
_ACCEPTED = 1
_EXECUTING = 2
_SUCCEEDED = 4
_CANCELED = 5
_ABORTED = 6

_TERMINAL = (_SUCCEEDED, _CANCELED, _ABORTED)

_TASK_STATE = {
    _ACCEPTED: NavTaskStatus.STATE_ACTIVE,
    _EXECUTING: NavTaskStatus.STATE_ACTIVE,
    _SUCCEEDED: NavTaskStatus.STATE_SUCCEEDED,
    _CANCELED: NavTaskStatus.STATE_CANCELED,
    _ABORTED: NavTaskStatus.STATE_ABORTED,
}

# Nav2 result codes mapped to the reasons in NavigationStatus. Only 204 and 208
# have actually shown up in the sim; the rest come from the interface files.
NATIVE_CODE_TO_REASON = {
    203: NavigationStatus.GOAL_OUTSIDE_MAP,
    204: NavigationStatus.GOAL_OUTSIDE_MAP,
    205: NavigationStatus.START_BLOCKED,
    206: NavigationStatus.GOAL_BLOCKED,
    208: NavigationStatus.NO_VALID_PATH,
    207: NavigationStatus.PLANNER_TIMEOUT,
    107: NavigationStatus.CONTROLLER_TIMEOUT,
    104: NavigationStatus.STUCK,
    105: NavigationStatus.STUCK,
    106: NavigationStatus.LOCAL_BLOCKED,
    102: NavigationStatus.TF_UNAVAILABLE,
    202: NavigationStatus.TF_UNAVAILABLE,
    101: NavigationStatus.NAV_CONFIG_FAULT,
    201: NavigationStatus.NAV_CONFIG_FAULT,
    100: NavigationStatus.NAV_UNKNOWN_FAULT,
    200: NavigationStatus.NAV_UNKNOWN_FAULT,
}

# Nav2 behavior tree nodes that count as a recovery action
_RECOVERY_NODES = {
    'Spin': RecoveryStatus.ACTION_SPIN,
    'BackUp': RecoveryStatus.ACTION_BACKUP,
    'Wait': RecoveryStatus.ACTION_WAIT,
    'ClearEntireCostmap': RecoveryStatus.ACTION_CLEAR_COSTMAP,
}


class SensorTracker:
    """
    Track freshness and rate for one sensor from the profile.

    Times are plain seconds. A sensor with no stale threshold can be seen but not
    judged, so once it has data it is UNKNOWN, never OK.
    """

    def __init__(self, cfg):
        self.id = cfg['id']
        self.kind = cfg['kind']
        self.topic = cfg['topic']
        self.frame_id = cfg['frame_id']
        self.required = bool(cfg['required'])
        nominal = cfg.get('nominal_period_s')
        multiple = cfg.get('stale_multiple')
        if nominal is None or multiple is None:
            self.stale_after = None
        else:
            self.stale_after = nominal * multiple
        self._last = None
        self._intervals = deque(maxlen=20)

    @property
    def ever_seen(self):
        """Return True once a message has arrived."""
        return self._last is not None

    def on_message(self, stamp):
        """Record the stamp of a message, in seconds."""
        if self._last is not None:
            dt = stamp - self._last
            if dt > 0:
                self._intervals.append(dt)
        self._last = stamp

    def rate_hz(self):
        """Return the rolling average rate."""
        if not self._intervals:
            return 0.0
        avg = sum(self._intervals) / len(self._intervals)
        return 1.0 / avg if avg > 0 else 0.0

    def age(self, now):
        """Return seconds since the last message, 0 if there was none."""
        if self._last is None:
            return 0.0
        return max(0.0, now - self._last)

    def state(self, now):
        """Return the SensorStatus state."""
        if self._last is None:
            # nothing received yet: a required sensor is UNKNOWN, an optional one
            # is ABSENT
            if self.required:
                return SensorStatus.STATE_UNKNOWN
            return SensorStatus.STATE_ABSENT
        if self.stale_after is None:
            return SensorStatus.STATE_UNKNOWN
        if now - self._last > self.stale_after:
            return SensorStatus.STATE_STALE
        return SensorStatus.STATE_OK


class TaskTracker:
    """
    Follow the navigate_to_pose goal that is currently being tracked.

    Status, feedback and results all carry a goal id. Anything that does not
    match the tracked goal is dropped, so a late message from an old goal can
    not overwrite the state of a newer one.
    """

    def __init__(self):
        self.goal_id = None  # bytes
        self.goal_stamp = (0, 0)  # (sec, nanosec)
        self.state = NavTaskStatus.STATE_UNKNOWN
        self.native_error_code = 0
        self.reason = NavigationStatus.NONE
        self.distance_remaining = 0.0
        self._result_requested_for = None

    def on_status(self, entries):
        """
        Update from the status_list of a GoalStatusArray.

        The newest goal in the list is the tracked one. Returns a pair: whether
        a new goal just started, and the goal id message to fetch a result for
        (None when there is nothing to fetch).
        """
        if not entries:
            return False, None
        latest = max(
            reversed(entries),
            key=lambda e: (e.goal_info.stamp.sec, e.goal_info.stamp.nanosec))
        goal_id = bytes(latest.goal_info.goal_id.uuid)

        new_goal = goal_id != self.goal_id
        if new_goal:
            # drop whatever the previous goal left behind
            self.goal_id = goal_id
            self.native_error_code = 0
            self.reason = NavigationStatus.NONE
            self.distance_remaining = 0.0
            self._result_requested_for = None
        self.goal_stamp = (latest.goal_info.stamp.sec, latest.goal_info.stamp.nanosec)
        self.state = _TASK_STATE.get(latest.status, NavTaskStatus.STATE_UNKNOWN)

        to_fetch = None
        if latest.status in _TERMINAL and goal_id != self._result_requested_for:
            # not marked as requested here - only once a request is actually
            # sent (see mark_result_requested). Otherwise, if the service was
            # not ready, this goal would never be asked for again.
            to_fetch = latest.goal_info.goal_id
        return new_goal, to_fetch

    def mark_result_requested(self, goal_id):
        """Record that a get_result request was actually sent for this goal."""
        self._result_requested_for = goal_id

    def on_feedback(self, goal_id, distance_remaining):
        """Take the distance if the feedback belongs to the tracked goal."""
        if goal_id != self.goal_id:
            return False
        self.distance_remaining = float(distance_remaining)
        return True

    def on_result(self, goal_id, error_code):
        """Take the error code if the result belongs to the tracked goal."""
        if goal_id != self.goal_id:
            return False
        self.native_error_code = int(error_code)
        if error_code == 0:
            self.reason = NavigationStatus.NONE
        else:
            self.reason = NATIVE_CODE_TO_REASON.get(
                error_code, NavigationStatus.NAV_UNKNOWN_FAULT)
        return True


class ResultFetcher:
    """
    Ask for a terminal goal's result, retrying on a timer until it is sent.

    Nav2 publishes goal status only on change, not periodically - so if the
    service was not ready when the terminal status first arrived, waiting for
    another status update to retry could wait forever (that update may never
    come, if no further goal is ever sent). Instead this remembers the goal id
    still owed a request and tries again whenever the node calls
    retry_if_pending(), meant to be driven by the node's own periodic tick
    rather than by incoming status messages.
    """

    def __init__(self, task, client, request_factory, logger):
        self._task = task
        self._client = client
        self._request_factory = request_factory
        self._logger = logger
        self._pending_goal_id = None  # the GoalId message still owed a request

    def request(self, goal_id):
        """Ask for goal_id's result now, remembering it if it can not be sent."""
        self._pending_goal_id = goal_id
        self._try_send(goal_id)

    def retry_if_pending(self):
        """Retry the outstanding request, if any. Call this from a timer."""
        if self._pending_goal_id is not None:
            self._try_send(self._pending_goal_id)

    def _try_send(self, goal_id):
        if not self._client.service_is_ready():
            self._logger.warn('get_result service not ready, will retry')
            return
        req = self._request_factory()
        req.goal_id = goal_id
        goal_bytes = bytes(goal_id.uuid)
        future = self._client.call_async(req)
        self._task.mark_result_requested(goal_bytes)
        self._pending_goal_id = None
        future.add_done_callback(lambda f: self._on_result(f, goal_bytes))

    def _on_result(self, future, goal_bytes):
        try:
            response = future.result()
        except Exception as exc:
            self._logger.warn(f'get_result call failed: {exc}')
            return
        self._task.on_result(goal_bytes, response.result.error_code)


class RecoveryTracker:
    """
    Count recovery actions for the current goal.

    The count starts over when a new goal starts, and the limit comes from
    configuration, not from Nav2.
    """

    def __init__(self, attempt_limit):
        self.attempt_limit = max(1, min(int(attempt_limit), 255))
        self.action = RecoveryStatus.ACTION_NONE
        self.attempt = 0

    def on_new_goal(self):
        """Start the count over."""
        self.action = RecoveryStatus.ACTION_NONE
        self.attempt = 0

    def on_bt_event(self, node_name, status):
        """Count a recovery action when its behavior tree node starts running."""
        action = _RECOVERY_NODES.get(node_name)
        if action is None:
            return
        if status == 'RUNNING':
            self.action = action
            self.attempt += 1
        elif status in ('SUCCESS', 'FAILURE'):
            self.action = RecoveryStatus.ACTION_NONE

    @property
    def reason(self):
        """Reason code for the current recovery state."""
        if self.attempt >= self.attempt_limit:
            return NavigationStatus.RECOVERY_LIMIT_REACHED
        if self.action != RecoveryStatus.ACTION_NONE:
            return NavigationStatus.RECOVERY_IN_PROGRESS
        return NavigationStatus.NONE


class LifecycleTracker:
    """
    Remember the last confirmed lifecycle state of each watched node.

    A state only counts while it keeps being confirmed. If nothing confirms it
    within stale_after seconds the node is treated as unknown, so a node that
    crashed can not stay ACTIVE forever.
    """

    def __init__(self, names, stale_after):
        self.stale_after = float(stale_after)
        self._state = dict.fromkeys(names)
        self._confirmed_at = dict.fromkeys(names)
        self._seen = set()

    def confirm(self, name, state, now):
        """Record a state seen at time now, from a query or a transition event."""
        self._state[name] = state
        self._confirmed_at[name] = now
        self._seen.add(name)

    def unreachable(self, name):
        """Drop the state right away, e.g. when the node's service is gone."""
        self._confirmed_at[name] = None

    def state(self, name, now):
        """Return the lifecycle state id, or None if unknown or gone quiet."""
        confirmed_at = self._confirmed_at.get(name)
        if confirmed_at is None or now - confirmed_at > self.stale_after:
            return None
        return self._state[name]

    def seen_any(self):
        """Return True once any watched node has been confirmed."""
        return bool(self._seen)

    def lost(self, now):
        """Return the nodes that were seen before but are not confirmed any more."""
        return [
            name for name in self._state
            if name in self._seen and self.state(name, now) is None
        ]
