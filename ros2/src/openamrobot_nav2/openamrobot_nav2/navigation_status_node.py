"""
Publish NavigationStatus on /navigation/status.

Watches Nav2 (lifecycle state, AMCL, the navigate_to_pose action, the behavior
tree log), the sensor topics in the profile and the collision monitor, and
rolls them into one status message. It never sends goals or velocity commands.

Not covered yet: mission-state mapping (waiting on N1) and base/I8 status (no
adapter yet, so navigation stays NOT_READY with BASE_LINK_LOST).
"""

from collections import deque
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
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.parameter import Parameter
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


class SensorTracker:
    """Track freshness and rate for one sensor from the profile."""

    def __init__(self, cfg):
        self.id = cfg['id']
        self.kind = cfg['kind']
        self.topic = cfg['topic']
        self.frame_id = cfg['frame_id']
        self.required = cfg['required']
        self.nominal_period_s = cfg['nominal_period_s']
        self.stale_after_s = cfg['nominal_period_s'] * cfg['stale_multiple']
        self.last_stamp = None
        self.recent_intervals = deque(maxlen=20)

    def on_message(self, stamp: Time):
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds / 1e9
            if dt > 0:
                self.recent_intervals.append(dt)
        self.last_stamp = stamp

    def rate_hz(self):
        if not self.recent_intervals:
            return 0.0
        avg = sum(self.recent_intervals) / len(self.recent_intervals)
        return 1.0 / avg if avg > 0 else 0.0

    def state(self, now: Time):
        if self.last_stamp is None:
            # nothing received yet: a required sensor is UNKNOWN, an optional one is
            # ABSENT
            return SensorStatus.STATE_ABSENT if not self.required else SensorStatus.STATE_UNKNOWN
        age_s = (now - self.last_stamp).nanoseconds / 1e9
        if age_s > self.stale_after_s:
            return SensorStatus.STATE_STALE
        return SensorStatus.STATE_OK

    def age_duration(self, now: Time):
        if self.last_stamp is None:
            return RclpyDuration(seconds=0).to_msg()
        age_s = max(0.0, (now - self.last_stamp).nanoseconds / 1e9)
        return RclpyDuration(seconds=age_s).to_msg()


class NavigationStatusNode(Node):

    def _load_profile(self, path: str) -> dict:
        """
        Load the sensor and motion-source profile.

        A missing or broken file logs an error and gives an empty profile
        instead of crashing the node. That is easy to spot, since no sensors
        show up.
        """
        try:
            with open(path, 'r') as f:
                profile = yaml.safe_load(f) or {}
            self.get_logger().info(f'loaded profile from {path}')
            return profile
        except Exception as exc:
            self.get_logger().error(
                f'failed to load profile at {path}: {exc}, using an empty profile')
            return {}

    def __init__(self):
        super().__init__('navigation_status_node')

        # rclpy already declares use_sim_time, so set it instead of declaring it.
        # Message stamps in the sim are sim time; without this "now" would be wall
        # time and every age would come out huge. Must be False on a real robot.
        if not self.get_parameter('use_sim_time').value:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        default_profile_path = os.path.join(
            get_package_share_directory('openamrobot_nav2'),
            'config', 'navigation_status_profile.yaml')
        self.declare_parameter('profile_path', default_profile_path)
        profile_path = self.get_parameter('profile_path').value
        profile = self._load_profile(profile_path)

        # defaults come from the profile but can still be overridden with -p
        self.declare_parameter('profile_id', profile.get('profile_id', 'UNKNOWN'))
        self.declare_parameter('thresholds_id', profile.get('thresholds_id', 'UNSET'))
        self.declare_parameter('heartbeat_hz', float(profile.get('heartbeat_hz', 2.0)))
        self.declare_parameter('contract_version', 1)

        self._profile_id = self.get_parameter('profile_id').value
        self._thresholds_id = self.get_parameter('thresholds_id').value
        self._contract_version = self.get_parameter('contract_version').value

        # transient-local so a late subscriber still gets the last status
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(NavigationStatus, '/navigation/status', qos)

        # sensors from the profile
        self._sensors = {}
        for cfg in profile.get('sensors', []):
            tracker_cfg = {
                'id': cfg['id'], 'kind': cfg['kind'], 'topic': cfg['topic'],
                'frame_id': cfg['frame_id'], 'required': bool(cfg['required']),
                'nominal_period_s': float(cfg['nominal_period_s']),
                'stale_multiple': float(cfg['stale_multiple']),
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
        self._task_state = NavTaskStatus.STATE_UNKNOWN
        self._task_native_code = 0
        self._task_reason = NavigationStatus.NONE
        self._task_distance_remaining = 0.0
        self._current_goal_id_bytes = None  # goal being tracked
        self._result_requested_for = set()  # goals we already asked get_result for
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
            self._on_nav_feedback, 10)
        self._nav_result_client = self.create_client(
            NavigateToPose.Impl.GetResultService, '/navigate_to_pose/_action/get_result')

        # our own attempt counter, since number_of_recoveries from Nav2 overcounts
        self._recovery_action = RecoveryStatus.ACTION_NONE
        self._recovery_attempt = 0
        self._recovery_attempt_limit = 6  # placeholder, should come from config
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
            entry['source']: coverage_str_to_enum[entry['coverage']]
            for entry in profile.get('motion_sources', [])
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
        self._node_states = dict.fromkeys(MANAGED_NAV_NODES)
        self._init_lifecycle_watchers()

        # no I8 adapter yet, see _update_base_dependency
        self._base_ever_seen = False

        heartbeat_hz = self.get_parameter('heartbeat_hz').value
        self.create_timer(1.0 / heartbeat_hz, self._publish_status)

        self.get_logger().info(
            f'navigation_status_node up, profile={self._profile_id}')

    def _init_lifecycle_watchers(self):
        for name in MANAGED_NAV_NODES:
            self.create_subscription(
                TransitionEvent, f'/{name}/transition_event',
                self._make_transition_cb(name), 10)
            client = self.create_client(GetState, f'/{name}/get_state')
            # fetch the state once, since transition_event only fires on changes; a
            # node that is not up yet stays None until its first event
            if client.wait_for_service(timeout_sec=2.0):
                req = GetState.Request()
                future = client.call_async(req)
                future.add_done_callback(self._make_get_state_cb(name))

    def _make_transition_cb(self, name):
        def _cb(msg: TransitionEvent):
            self._node_states[name] = msg.goal_state.id
        return _cb

    def _make_get_state_cb(self, name):
        def _cb(future):
            try:
                result = future.result()
                self._node_states[name] = result.current_state.id
            except Exception as exc:
                self.get_logger().warn(f'get_state failed for {name}: {exc}')
        return _cb

    def _stack_state(self):
        states = self._node_states.values()
        if any(s is None for s in states):
            return NavStackStatus.STATE_UNKNOWN, NavigationStatus.NONE
        if all(s == LIFECYCLE_ACTIVE for s in states):
            return NavStackStatus.STATE_ACTIVE, NavigationStatus.NONE
        # Some but not all nodes active: call it resetting rather than failed, since
        # a node bouncing is normal. FAILED (stuck like this for too long) is not
        # implemented.
        return NavStackStatus.STATE_RESETTING, NavigationStatus.NAV_STACK_RESETTING

    def _make_sensor_cb(self, sensor_id):
        def _cb(msg):
            self._sensors[sensor_id].on_message(self._stamp_to_time(msg.header.stamp))
        return _cb

    def _stamp_to_time(self, stamp) -> Time:
        return Time.from_msg(stamp) if (stamp.sec or stamp.nanosec) else self.get_clock().now()

    def _on_odom_for_motion(self, msg: Odometry):
        # only for the is-moving check below; kept apart from the sensor loop so it
        # still works if a profile leaves odom out
        self._latest_odom = msg

    def _is_moving(self) -> bool:
        odom = getattr(self, '_latest_odom', None)
        if odom is None:
            return False
        v = odom.twist.twist.linear
        speed = math.sqrt(v.x ** 2 + v.y ** 2)
        return speed > 0.02  # m/s, placeholder threshold

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

        if self._amcl_pose is None:
            out.state = LocalizationStatus.STATE_UNKNOWN
            out.health = LocalizationStatus.HEALTH_UNKNOWN
            out.reason = NavigationStatus.LOCALIZATION_NOT_INITIALIZED
            return out

        age_s = max(0.0, (now - self._amcl_pose_stamp).nanoseconds / 1e9)
        out.pose_age = RclpyDuration(seconds=age_s).to_msg()
        cov = self._amcl_pose.pose.covariance
        # 6x6 row-major covariance: 0 = xx, 7 = yy, 35 = yaw
        out.covariance_xy = max(cov[0], cov[7])
        out.covariance_yaw = cov[35]

        if not tf_ok:
            out.state = LocalizationStatus.STATE_LOST
            out.health = LocalizationStatus.HEALTH_FAULT
            out.reason = NavigationStatus.TF_UNAVAILABLE
            return out

        if self._correction_paused:
            out.state = LocalizationStatus.STATE_OK
            out.health = LocalizationStatus.HEALTH_OK
            out.reason = NavigationStatus.LOCALIZATION_PAUSED_FOR_DOCKING
            return out

        if self._is_moving() and age_s > 5.0:  # seconds, placeholder threshold
            out.state = LocalizationStatus.STATE_DEGRADED
            out.health = LocalizationStatus.HEALTH_DEGRADED
            out.reason = NavigationStatus.LOCALIZATION_STALE_WHILE_MOVING
            return out

        # no covariance cutoff yet, that needs real robot data
        out.state = LocalizationStatus.STATE_OK
        out.health = LocalizationStatus.HEALTH_OK
        out.reason = NavigationStatus.NONE
        return out

    def _on_nav_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        goal_id_bytes = bytes(latest.goal_info.goal_id.uuid)

        # GoalStatus codes: 1 accepted, 2 executing, 4 succeeded, 5 canceled, 6 aborted
        mapping = {
            1: NavTaskStatus.STATE_ACTIVE,
            2: NavTaskStatus.STATE_ACTIVE,
            4: NavTaskStatus.STATE_SUCCEEDED,
            5: NavTaskStatus.STATE_CANCELED,
            6: NavTaskStatus.STATE_ABORTED,
        }
        terminal_statuses = (4, 5, 6)

        if goal_id_bytes != self._current_goal_id_bytes:
            # new goal: drop the last goal's error code and distance
            self._current_goal_id_bytes = goal_id_bytes
            self._task_native_code = 0
            self._task_reason = NavigationStatus.NONE
            self._task_distance_remaining = 0.0

        self._task_state = mapping.get(latest.status, NavTaskStatus.STATE_UNKNOWN)

        if (latest.status in terminal_statuses
                and goal_id_bytes not in self._result_requested_for):
            self._result_requested_for.add(goal_id_bytes)
            self._request_nav_result(latest.goal_info.goal_id)

    def _on_nav_feedback(self, msg):
        self._task_distance_remaining = msg.feedback.distance_remaining

    def _request_nav_result(self, goal_id):
        if not self._nav_result_client.service_is_ready():
            self.get_logger().warn(
                'get_result service not ready, no error code for this goal')
            return
        req = NavigateToPose.Impl.GetResultService.Request()
        req.goal_id = goal_id
        future = self._nav_result_client.call_async(req)
        future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'get_result call failed: {exc}')
            return
        code = response.result.error_code
        self._task_native_code = code
        if code == 0:
            self._task_reason = NavigationStatus.NONE
        else:
            self._task_reason = NATIVE_CODE_TO_REASON.get(
                code, NavigationStatus.NAV_UNKNOWN_FAULT)

    def _on_bt_log(self, msg: BehaviorTreeLog):
        recovery_names = {
            'Spin': RecoveryStatus.ACTION_SPIN,
            'BackUp': RecoveryStatus.ACTION_BACKUP,
            'Wait': RecoveryStatus.ACTION_WAIT,
            'ClearEntireCostmap': RecoveryStatus.ACTION_CLEAR_COSTMAP,
        }
        for event in msg.event_log:
            if event.current_status == 'RUNNING' and event.node_name in recovery_names:
                self._recovery_action = recovery_names[event.node_name]
                self._recovery_attempt += 1
            elif event.current_status in ('SUCCESS', 'FAILURE') and \
                    event.node_name in recovery_names:
                self._recovery_action = RecoveryStatus.ACTION_NONE

    def _on_collision_monitor_state(self, msg: CollisionMonitorState):
        self._collision_monitor_state = msg

    def _protection_status(self) -> ProtectionStatus:
        out = ProtectionStatus()
        cm_state = self._node_states.get('collision_monitor')
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
            msc.source = source_enum[name]
            msc.coverage = coverage
            msc.reason = (NavigationStatus.MOTION_SOURCE_UNPROTECTED
                          if coverage == MotionSourceCoverage.COVERAGE_UNPROTECTED
                          else NavigationStatus.NONE)
            out.motion_sources.append(msc)

        out.rear_coverage = self._rear_coverage
        out.rear_coverage_reason = (
            NavigationStatus.REAR_COVERAGE_UNAVAILABLE
            if self._rear_coverage == ProtectionStatus.REAR_COVERAGE_UNAVAILABLE
            else NavigationStatus.NONE)
        return out

    def _update_base_dependency(self, not_ready_reasons: list):
        """
        Add BASE_LINK_LOST until an I8 status adapter exists.

        There is no adapter on main yet (openamr-platform-fw#6), so this only
        covers the "never received" case. ESTOP_ACTIVE and battery state are
        left alone: they need explicit, fresh I8 telemetry.
        """
        if not self._base_ever_seen:
            not_ready_reasons.append(NavigationStatus.BASE_LINK_LOST)

    def _publish_status(self):
        now = self.get_clock().now()
        msg = NavigationStatus()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ''
        msg.contract_version = self._contract_version
        msg.profile_id = self._profile_id
        msg.thresholds_id = self._thresholds_id

        stack = NavStackStatus()
        stack.state, stack_reason = self._stack_state()
        stack.reason = stack_reason
        stack.inactive_nodes = [
            name for name, s in self._node_states.items() if s != LIFECYCLE_ACTIVE
        ]
        msg.stack = stack

        msg.localization = self._localization_status(now)

        msg.sensors = []
        for tracker in self._sensors.values():
            s = SensorStatus()
            s.id = tracker.id
            s.kind = getattr(SensorStatus, f'KIND_{tracker.kind}')
            s.frame_id = tracker.frame_id
            s.state = tracker.state(now)
            s.reason = (NavigationStatus.SENSOR_DATA_STALE
                        if s.state == SensorStatus.STATE_STALE
                        else NavigationStatus.SENSOR_NOT_CONFIGURED
                        if s.state == SensorStatus.STATE_ABSENT
                        else NavigationStatus.NONE)
            s.data_age = tracker.age_duration(now)
            s.rate_hz = tracker.rate_hz()
            s.required = tracker.required
            msg.sensors.append(s)

        task = NavTaskStatus()
        task.state = self._task_state
        task.reason = self._task_reason
        task.native_error_code = self._task_native_code
        task.distance_remaining = self._task_distance_remaining
        msg.task = task

        recovery = RecoveryStatus()
        recovery.action = self._recovery_action
        recovery.attempt = min(self._recovery_attempt, 255)
        recovery.attempt_limit = self._recovery_attempt_limit
        recovery.reason = (NavigationStatus.RECOVERY_LIMIT_REACHED
                           if self._recovery_attempt >= self._recovery_attempt_limit
                           else NavigationStatus.RECOVERY_IN_PROGRESS
                           if self._recovery_action != RecoveryStatus.ACTION_NONE
                           else NavigationStatus.NONE)
        msg.recovery = recovery

        msg.protection = self._protection_status()

        not_ready_reasons = []
        self._update_base_dependency(not_ready_reasons)

        sensor_gaps = sum(
            1 for s in msg.sensors
            if s.state in (SensorStatus.STATE_ABSENT, SensorStatus.STATE_UNKNOWN)
        )
        msg.sensor_gaps = sensor_gaps

        required_sensors_ok = all(
            s.state == SensorStatus.STATE_OK for s in msg.sensors if s.required
        )
        if (stack.state == NavStackStatus.STATE_ACTIVE
                and required_sensors_ok
                and msg.localization.state == LocalizationStatus.STATE_OK
                and not not_ready_reasons):
            msg.health = NavigationStatus.HEALTH_OK
        elif stack.state == NavStackStatus.STATE_UNKNOWN:
            msg.health = NavigationStatus.HEALTH_UNKNOWN
        else:
            msg.health = NavigationStatus.HEALTH_DEGRADED

        if not_ready_reasons or stack.state != NavStackStatus.STATE_ACTIVE:
            msg.navigation_readiness = NavigationStatus.NAVIGATION_READINESS_NOT_READY
        else:
            msg.navigation_readiness = NavigationStatus.NAVIGATION_READINESS_READY
        msg.not_ready_reasons = not_ready_reasons

        active_reasons = [
            r for r in (
                stack.reason, msg.localization.reason, task.reason,
                recovery.reason, msg.protection.collision_monitor_reason,
            ) if r != NavigationStatus.NONE
        ]
        msg.active_reasons = active_reasons

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
