"""
Publish NavigationStatus on /navigation/status.

Watches Nav2 (lifecycle state, AMCL, the navigate_to_pose action, the behavior
tree log), the sensor topics in the profile and the collision monitor, and
rolls them into one status message. It never sends goals or velocity commands.
A status goes out as soon as something changes, and otherwise once per heartbeat.

Not covered yet: mission-state mapping (waiting on N1) and base/I8 status (no
adapter yet, so navigation stays NOT_READY with BASE_LINK_LOST).
"""

import math
import os

from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from nav_msgs.msg import Odometry
from openamr_nav_msgs.msg import (
    LocalizationStatus,
    MotionSourceCoverage,
    NavigationStatus,
    NavStackStatus,
    NavTaskStatus,
    ProtectionStatus,
    RecoveryStatus,
    SensorStatus,
)
from openamrobot_nav2.status_rules import (
    evaluate_localization,
    motion_source_reason,
    PublishGate,
    rear_coverage_reason,
    roll_up,
    SENSOR_KEYS,
    sensor_reason,
    status_signature,
    validate_profile,
)
from openamrobot_nav2.status_trackers import (
    LifecycleTracker,
    RecoveryTracker,
    ResultFetcher,
    SensorTracker,
    TaskTracker,
)
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import yaml


# Nav2 nodes whose lifecycle state feeds the stack status. Keep in sync with
# nav2_params.yaml.
MANAGED_NAV_NODES = [
    'amcl',
    'planner_server',
    'controller_server',
    'smoother_server',
    'bt_navigator',
    'behavior_server',
    'waypoint_follower',
    'velocity_smoother',
    'map_server',
    'collision_monitor',
]

LIFECYCLE_ACTIVE = 3  # PRIMARY_STATE_ACTIVE

# Message type to subscribe to for each sensor kind in the profile. ToF,
# ultrasonic and depth camera are not here yet; a profile entry with a kind
# that is not listed gets skipped with a warning.
SENSOR_KIND_TO_MSG_TYPE = {
    'LIDAR': LaserScan,
    'ODOMETRY': Odometry,
    'IMU': Imu,
}


class NavigationStatusNode(Node):

    def _load_profile(self, path: str):
        """
        Load the sensor and motion-source profile.

        Returns the profile and a list of problems. A missing, broken or unusable
        profile does not crash the node: it reports NAV_CONFIG_FAULT and stays
        NOT_READY, so an empty profile can never pass the required-sensor check.
        """
        try:
            with open(path, 'r') as f:
                profile = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(f'failed to load profile at {path}: {exc}')
            return {}, [f'could not load {path}']
        problems = validate_profile(profile)
        for problem in problems:
            self.get_logger().error(f'profile problem: {problem}')
        if not problems:
            self.get_logger().info(f'loaded profile from {path}')
        if not isinstance(profile, dict):
            profile = {}
        return profile, problems

    def __init__(self):
        super().__init__('navigation_status_node')

        default_profile_path = os.path.join(
            get_package_share_directory('openamrobot_nav2'),
            'config', 'navigation_status_profile.yaml')
        self.declare_parameter('profile_path', default_profile_path)
        profile_path = self.get_parameter('profile_path').value
        profile, problems = self._load_profile(profile_path)
        self._config_fault = bool(problems)
        self._thresholds = profile.get('thresholds') or {}

        # defaults come from the profile but can still be overridden with -p
        self.declare_parameter('profile_id', profile.get('profile_id', 'UNKNOWN'))
        self.declare_parameter('thresholds_id', profile.get('thresholds_id', 'UNSET'))
        self.declare_parameter('heartbeat_hz', float(profile.get('heartbeat_hz', 2.0)))
        self.declare_parameter(
            'recovery_attempt_limit', int(profile.get('recovery_attempt_limit', 6)))
        self.declare_parameter('lifecycle_poll_period_s', 1.0)
        self.declare_parameter('lifecycle_stale_after_s', 3.0)
        self.declare_parameter('lifecycle_request_timeout_s', 2.0)
        self.declare_parameter('change_check_hz', 10.0)

        self._profile_id = self.get_parameter('profile_id').value
        self._thresholds_id = self.get_parameter('thresholds_id').value

        # transient-local so a late subscriber still gets the last status
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(NavigationStatus, '/navigation/status', qos)

        # sensors from the profile; the stale limits come from its thresholds
        self._sensors = {}
        stale_multiples = self._thresholds.get('sensor_stale_multiple') or {}
        for cfg in profile.get('sensors') or []:
            if any(key not in cfg for key in SENSOR_KEYS):
                continue  # already reported as a profile problem
            tracker_cfg = {
                'id': cfg['id'], 'kind': cfg['kind'], 'topic': cfg['topic'],
                'frame_id': cfg['frame_id'], 'required': bool(cfg['required']),
                'nominal_period_s': float(cfg['nominal_period_s']),
                'stale_multiple': stale_multiples.get(cfg['id']),
            }
            self._sensors[cfg['id']] = SensorTracker(tracker_cfg)
            msg_type = SENSOR_KIND_TO_MSG_TYPE.get(cfg['kind'])
            if msg_type is None:
                self.get_logger().warn(
                    f"no message type for sensor kind '{cfg['kind']}' "
                    f"({cfg['id']}), skipping it")
                continue
            self.create_subscription(
                msg_type, cfg['topic'], self._make_sensor_cb(cfg['id']), 10)

        # separate odom subscription, only used for the is-moving check
        self.create_subscription(Odometry, '/odom', self._on_odom_for_motion, 10)

        # localization
        self._amcl_pose = None
        self._amcl_pose_stamp = None
        self._correction_paused = False  # from /dock_trigger_status
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # nav task: we only watch, never send goals. Feedback comes on a topic, but
        # the result does not: it has to be fetched from the action's get_result
        # service using the goal id from the status topic.
        self._task = TaskTracker()
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
            self._on_nav_feedback, 10)
        self._nav_result_client = self.create_client(
            NavigateToPose.Impl.GetResultService, '/navigate_to_pose/_action/get_result')
        self._result_fetcher = ResultFetcher(
            self._task, self._nav_result_client,
            lambda: NavigateToPose.Impl.GetResultService.Request(), self.get_logger())

        # our own attempt counter, since number_of_recoveries from Nav2 overcounts
        self._recovery = RecoveryTracker(self.get_parameter('recovery_attempt_limit').value)
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self._on_bt_log, 10)

        # protection
        self._collision_monitor_state = None  # never seen publishing in the sim
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._on_collision_monitor_state, 10)
        coverage_str_to_enum = {
            'UNKNOWN': MotionSourceCoverage.COVERAGE_UNKNOWN,
            'UNPROTECTED': MotionSourceCoverage.COVERAGE_UNPROTECTED,
            'PARTIAL': MotionSourceCoverage.COVERAGE_PARTIAL,
            'PROTECTED': MotionSourceCoverage.COVERAGE_PROTECTED,
        }
        self._motion_sources = {
            entry['source']: coverage_str_to_enum.get(
                entry.get('coverage'), MotionSourceCoverage.COVERAGE_UNKNOWN)
            for entry in profile.get('motion_sources') or []
        }

        rear_str_to_enum = {
            'UNKNOWN': ProtectionStatus.REAR_COVERAGE_UNKNOWN,
            'UNAVAILABLE': ProtectionStatus.REAR_COVERAGE_UNAVAILABLE,
            'AVAILABLE': ProtectionStatus.REAR_COVERAGE_AVAILABLE,
        }
        self._rear_coverage = rear_str_to_enum.get(
            profile.get('rear_coverage', 'UNKNOWN'),
            ProtectionStatus.REAR_COVERAGE_UNKNOWN)

        # read only: used to know when AMCL correction is paused for docking.
        # DockingStatus itself is #33.
        self.create_subscription(
            String, '/dock_trigger_status', self._on_dock_trigger_status, 10)

        # lifecycle watchers
        poll_period = float(self.get_parameter('lifecycle_poll_period_s').value)
        stale_after = max(
            float(self.get_parameter('lifecycle_stale_after_s').value), 2 * poll_period)
        self._lifecycle = LifecycleTracker(MANAGED_NAV_NODES, stale_after)
        self._init_lifecycle_watchers(poll_period)

        # no I8 adapter yet, see _base_reasons
        self._base_ever_seen = False

        # check for changes often, publish on a change or once per heartbeat
        heartbeat_hz = float(self.get_parameter('heartbeat_hz').value)
        check_hz = max(float(self.get_parameter('change_check_hz').value), heartbeat_hz)
        self._gate = PublishGate(1.0 / heartbeat_hz, tolerance=0.5 / check_hz)
        self.create_timer(1.0 / check_hz, self._tick)

        self.get_logger().info(
            f'navigation_status_node up, profile={self._profile_id}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _init_lifecycle_watchers(self, poll_period):
        self._state_clients = {}
        self._state_pending = {}
        self._state_pending_since = {}
        self._state_request_timeout_s = max(
            float(self.get_parameter('lifecycle_request_timeout_s').value), 2 * poll_period)
        for name in MANAGED_NAV_NODES:
            self.create_subscription(
                TransitionEvent, f'/{name}/transition_event',
                self._make_transition_cb(name), 10)
            self._state_clients[name] = self.create_client(GetState, f'/{name}/get_state')
        # transition_event only fires on changes, so ask every node for its state
        # regularly; a node that stops answering ages out of the tracker
        self.create_timer(poll_period, self._poll_lifecycle)
        self._poll_lifecycle()

    def _poll_lifecycle(self):
        now_s = self._now_s()
        for name, client in self._state_clients.items():
            if not client.service_is_ready():
                self._lifecycle.unreachable(name)
                self._state_pending.pop(name, None)
                continue
            pending = self._state_pending.get(name)
            if pending is not None and not pending.done():
                if now_s - self._state_pending_since[name] <= self._state_request_timeout_s:
                    continue
                # the node stopped answering mid-request - service_is_ready()
                # can still say yes for a while after a crash (discovery lags
                # behind), so abandon the request rather than wait forever
                self.get_logger().warn(f'{name} get_state request timed out, retrying')
                self._lifecycle.unreachable(name)
            future = client.call_async(GetState.Request())
            self._state_pending[name] = future
            self._state_pending_since[name] = now_s
            future.add_done_callback(self._make_get_state_cb(name, future))

    def _make_transition_cb(self, name):
        def _cb(msg: TransitionEvent):
            self._lifecycle.confirm(name, msg.goal_state.id, self._now_s())
        return _cb

    def _make_get_state_cb(self, name, future):
        def _cb(fut):
            if self._state_pending.get(name) is not future:
                # superseded by a newer request (this one timed out and was
                # abandoned) - a late response here would be stale and must
                # not overwrite whatever the newer request already confirmed
                return
            try:
                state = fut.result().current_state.id
            except Exception as exc:
                self.get_logger().warn(f'get_state failed for {name}: {exc}')
                return
            self._lifecycle.confirm(name, state, self._now_s())
        return _cb

    def _stack_state(self, now_s):
        states = [self._lifecycle.state(name, now_s) for name in MANAGED_NAV_NODES]
        if any(s is None for s in states):
            if self._lifecycle.lost(now_s):
                reason = NavigationStatus.NAV_NODE_INACTIVE
            elif self._lifecycle.seen_any():
                reason = NavigationStatus.NAV_STACK_STARTING
            else:
                reason = NavigationStatus.NAV_STACK_NOT_STARTED
            return NavStackStatus.STATE_UNKNOWN, reason
        if all(s == LIFECYCLE_ACTIVE for s in states):
            return NavStackStatus.STATE_ACTIVE, NavigationStatus.NONE
        # Some but not all nodes active: call it resetting rather than failed, since
        # a node bouncing is normal. FAILED (stuck like this for too long) is not
        # implemented.
        return NavStackStatus.STATE_RESETTING, NavigationStatus.NAV_STACK_RESETTING

    def _make_sensor_cb(self, sensor_id):
        def _cb(msg):
            self._sensors[sensor_id].on_message(self._stamp_seconds(msg.header.stamp))
        return _cb

    def _stamp_to_time(self, stamp) -> Time:
        return Time.from_msg(stamp) if (stamp.sec or stamp.nanosec) else self.get_clock().now()

    def _stamp_seconds(self, stamp) -> float:
        return self._stamp_to_time(stamp).nanoseconds / 1e9

    def _on_odom_for_motion(self, msg: Odometry):
        # only for the is-moving check below; kept apart from the sensor loop so it
        # still works if a profile leaves odom out
        self._latest_odom = msg

    def _is_moving(self):
        """Return True or False, or None when it can not be told."""
        limit = self._thresholds.get('moving_speed_mps')
        odom = getattr(self, '_latest_odom', None)
        if limit is None or odom is None:
            return None
        v = odom.twist.twist.linear
        return math.sqrt(v.x ** 2 + v.y ** 2) > limit

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self._amcl_pose = msg
        self._amcl_pose_stamp = self._stamp_to_time(msg.header.stamp)

    def _on_dock_trigger_status(self, msg: String):
        # dock_trigger pauses AMCL correction while docking; we only read the status
        self._correction_paused = msg.data in ('docking', 'undocking')

    def _localization_status(self, now: Time) -> LocalizationStatus:
        out = LocalizationStatus()
        try:
            tf_ok = self._tf_buffer.can_transform(
                'map', 'odom', rclpy.time.Time())
        except Exception:
            tf_ok = False
        # the message setters only accept a real bool, and can_transform() is not
        # guaranteed to return one
        out.tf_available = bool(tf_ok)
        out.correction_paused = bool(self._correction_paused)

        pose_received = self._amcl_pose is not None
        age_s = 0.0
        covariance_xy = 0.0
        covariance_yaw = 0.0
        if pose_received:
            age_s = max(0.0, (now - self._amcl_pose_stamp).nanoseconds / 1e9)
            out.pose_age = RclpyDuration(seconds=age_s).to_msg()
            cov = self._amcl_pose.pose.covariance
            # 6x6 row-major covariance: 0 = xx, 7 = yy, 35 = yaw
            covariance_xy = max(cov[0], cov[7])
            covariance_yaw = cov[35]
            out.covariance_xy = covariance_xy
            out.covariance_yaw = covariance_yaw

        out.state, out.health, out.reason = evaluate_localization(
            pose_received=pose_received, pose_age_s=age_s,
            covariance_xy=covariance_xy, covariance_yaw=covariance_yaw,
            tf_ok=bool(tf_ok), correction_paused=bool(self._correction_paused),
            moving=self._is_moving(), thresholds=self._thresholds)
        return out

    def _on_nav_status(self, msg: GoalStatusArray):
        new_goal, to_fetch = self._task.on_status(msg.status_list)
        if new_goal:
            self._recovery.on_new_goal()
        if to_fetch is not None:
            self._result_fetcher.request(to_fetch)

    def _on_nav_feedback(self, msg):
        self._task.on_feedback(bytes(msg.goal_id.uuid), msg.feedback.distance_remaining)

    def _on_bt_log(self, msg: BehaviorTreeLog):
        for event in msg.event_log:
            self._recovery.on_bt_event(event.node_name, event.current_status)

    def _on_collision_monitor_state(self, msg: CollisionMonitorState):
        self._collision_monitor_state = msg

    def _protection_status(self, now_s) -> ProtectionStatus:
        out = ProtectionStatus()
        cm_state = self._lifecycle.state('collision_monitor', now_s)
        if cm_state == LIFECYCLE_ACTIVE:
            out.collision_monitor = ProtectionStatus.COLLISION_MONITOR_ACTIVE
            out.collision_monitor_reason = NavigationStatus.NONE
        elif cm_state is not None:
            out.collision_monitor = ProtectionStatus.COLLISION_MONITOR_INACTIVE
            out.collision_monitor_reason = NavigationStatus.COLLISION_LAYER_INACTIVE
        else:
            out.collision_monitor = ProtectionStatus.COLLISION_MONITOR_UNKNOWN
            out.collision_monitor_reason = NavigationStatus.PROTECTION_STATE_UNKNOWN

        source_enum = {
            'NAV_CONTROLLER': MotionSourceCoverage.SOURCE_NAV_CONTROLLER,
            'RECOVERY': MotionSourceCoverage.SOURCE_RECOVERY,
            'DOCKING': MotionSourceCoverage.SOURCE_DOCKING,
            'UNDOCK_REVERSE': MotionSourceCoverage.SOURCE_UNDOCK_REVERSE,
            'TELEOP': MotionSourceCoverage.SOURCE_TELEOP,
            'PARKING': MotionSourceCoverage.SOURCE_PARKING,
        }
        for name, coverage in self._motion_sources.items():
            msc = MotionSourceCoverage()
            msc.source = source_enum.get(name, MotionSourceCoverage.SOURCE_UNKNOWN)
            msc.coverage = coverage
            msc.reason = motion_source_reason(coverage)
            out.motion_sources.append(msc)

        out.rear_coverage = self._rear_coverage
        out.rear_coverage_reason = rear_coverage_reason(self._rear_coverage)
        return out

    def _base_reasons(self):
        """
        Return BASE_LINK_LOST until an I8 status adapter exists.

        There is no adapter on main yet (openamr-platform-fw#6), so this only
        covers the "never received" case. ESTOP_ACTIVE and battery state are
        left alone: they need explicit, fresh I8 telemetry.
        """
        if self._base_ever_seen:
            return []
        return [NavigationStatus.BASE_LINK_LOST]

    def _tick(self):
        self._result_fetcher.retry_if_pending()
        now = self.get_clock().now()
        msg = self._build_status(now)
        if self._gate.should_publish(status_signature(msg), now.nanoseconds / 1e9):
            self._pub.publish(msg)

    def _build_status(self, now):
        now_s = now.nanoseconds / 1e9
        msg = NavigationStatus()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ''
        msg.contract_version = NavigationStatus.CONTRACT_VERSION
        msg.profile_id = self._profile_id
        msg.thresholds_id = self._thresholds_id

        stack = NavStackStatus()
        stack.state, stack.reason = self._stack_state(now_s)
        stack.inactive_nodes = [
            name for name in MANAGED_NAV_NODES
            if self._lifecycle.state(name, now_s) != LIFECYCLE_ACTIVE
        ]
        msg.stack = stack

        msg.localization = self._localization_status(now)

        msg.sensors = []
        for tracker in self._sensors.values():
            s = SensorStatus()
            s.id = tracker.id
            s.kind = getattr(SensorStatus, f'KIND_{tracker.kind}', SensorStatus.KIND_UNKNOWN)
            s.frame_id = tracker.frame_id
            s.state = tracker.state(now_s)
            s.reason = sensor_reason(s.state, tracker.ever_seen)
            s.data_age = RclpyDuration(seconds=tracker.age(now_s)).to_msg()
            s.rate_hz = tracker.rate_hz()
            s.required = tracker.required
            msg.sensors.append(s)

        task = NavTaskStatus()
        task.state = self._task.state
        task.reason = self._task.reason
        task.native_error_code = self._task.native_error_code
        task.distance_remaining = self._task.distance_remaining
        task.goal_stamp.sec, task.goal_stamp.nanosec = self._task.goal_stamp
        msg.task = task

        recovery = RecoveryStatus()
        recovery.action = self._recovery.action
        recovery.attempt = min(self._recovery.attempt, 255)
        recovery.attempt_limit = self._recovery.attempt_limit
        recovery.reason = self._recovery.reason
        msg.recovery = recovery

        msg.protection = self._protection_status(now_s)

        msg.sensor_gaps = sum(
            1 for s in msg.sensors
            if s.state in (SensorStatus.STATE_ABSENT, SensorStatus.STATE_UNKNOWN)
        )

        result = roll_up(
            config_fault=self._config_fault,
            stack_state=stack.state, stack_reason=stack.reason,
            sensors=[(s.required, s.state, s.reason) for s in msg.sensors],
            localization_state=msg.localization.state,
            localization_reason=msg.localization.reason,
            other_reasons=(
                [task.reason, recovery.reason, msg.protection.collision_monitor_reason]
                + [m.reason for m in msg.protection.motion_sources]
                + [msg.protection.rear_coverage_reason]),
            base_reasons=self._base_reasons())
        msg.health = result.health
        msg.navigation_readiness = result.readiness
        msg.not_ready_reasons = result.not_ready_reasons
        msg.active_reasons = result.active_reasons
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C has already shut the context down
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
