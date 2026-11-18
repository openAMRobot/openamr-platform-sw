# SPDX-License-Identifier: MIT
"""station_alignment: station goal refinement from the marker, with Nav2 as the only command authority.

Action server (Proposed): /station_alignment/navigate_to_station, type nav2_msgs/action/NavigateToPose. The goal is
the nominal station pose in the map. The node:

1. sends that goal to Nav2's navigate_to_pose with behavior_tree = navigate_to_station.xml (platform-sw #164);
2. after the robot has stopped, averages `samples` station poses from /station_alignment/station_pose (base frame,
   independent of AMCL);
3. if the measured residual is within accept_along, accept_lateral and accept_yaw it succeeds; otherwise it fixes
   the measured station's position in odom and its heading in map and reaches them (maneuver.py): translations
   with behavior_server's DriveOnHeading or BackUp in odom, rotations as rotation-only goals of the station tree;
   a lateral residual above accept_lateral first backs up `backoff`. Then it measures again, up to `max_iterations`
   corrections. A behavior that fails is measured again like any correction.

It is an action client of Nav2 only: it publishes no velocity, runs no controller and starts no navigator. Motion is
controller_server's StationApproach controller and station_goal_checker for the approach, and behavior_server for
the correction, both through Nav2's velocity smoother and collision monitor. A residual larger than
max_correction_* aborts instead of moving. A diagnostic JSON line per goal goes to /station_alignment/report (Proposed).
"""
import json
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from builtin_interfaces.msg import Duration as DurationMsg

from openamrobot_station_alignment import geometry, maneuver, sampling, sources

STATUS_SUCCEEDED = 4
# Proposed result code (NavigateToPose.Result.error_code, outside Nav2's 9000 to 9011): the station goal of #164
# succeeded but the marker gave no usable sample, so no correction was made.
NOT_ALIGNED = 9900


def _pose_of(msg_pose):
    q = msg_pose.orientation
    return (msg_pose.position.x, msg_pose.position.y, geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))


def default_station_tree():
    try:
        share = get_package_share_directory("openamrobot_nav2")
        return os.path.join(share, "behavior_trees", "navigate_to_station.xml")
    except Exception:  # noqa: BLE001 - package not installed: the parameter must then be set
        return ""


class StationAlignment(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("station_alignment", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("action_name", "/station_alignment/navigate_to_station")
        self.declare_parameter("behavior_tree", default_station_tree())
        self.declare_parameter("station_pose_topic", "/station_alignment/station_pose")
        self.declare_parameter("report_topic", "/station_alignment/report")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("samples", 8)
        self.declare_parameter("min_samples", 4)
        self.declare_parameter("not_seen_retries", 2)
        self.declare_parameter("sample_timeout_s", 6.0)
        self.declare_parameter("max_latency_s", 15.0)
        self.declare_parameter("on_marker_not_seen", "succeed_unaligned")
        self.declare_parameter("max_iterations", 3)
        self.declare_parameter("accept_along", 0.03)
        self.declare_parameter("accept_lateral", 0.02)
        self.declare_parameter("accept_yaw", 0.06)
        self.declare_parameter("max_correction_xy", 0.15)
        self.declare_parameter("max_correction_yaw", 0.30)
        self.declare_parameter("max_sample_spread_xy", 0.01)
        self.declare_parameter("navigation_timeout_s", 300.0)
        self.declare_parameter("backoff", 0.15)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_max_age_s", 0.5)
        self.declare_parameter("tf_wait_s", 5.0)
        self.declare_parameter("station_pose_source", "topic")
        self.declare_parameter("behavior_speed", 0.05)
        self.declare_parameter("behavior_time_allowance_s", 30.0)
        self.declare_parameter("backup_action", "backup")
        self.declare_parameter("drive_on_heading_action", "drive_on_heading")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.samples = []
        self.sequence = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value,
                                callback_group=self.group)
        # Nav2 behavior_server translations (odom frame, through the velocity smoother and collision monitor)
        self.behaviors = {
            maneuver.BACKUP: ActionClient(self, BackUp, self.get_parameter("backup_action").value,
                                          callback_group=self.group),
            maneuver.DRIVE: ActionClient(self, DriveOnHeading, self.get_parameter("drive_on_heading_action").value,
                                         callback_group=self.group)}
        self.report = self.create_publisher(String, self.get_parameter("report_topic").value, 10)
        self.frame_rejected = 0
        self.latest_odom = None
        if self.get_parameter("station_pose_source").value == "topic":
            self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("station_pose_topic").value,
                                     self.on_station_pose, 20, callback_group=self.group)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 20,
                                 callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, self.get_parameter("action_name").value,
                                   execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)

    # ------------------------------------------------------------------ inputs
    def on_station_pose(self, msg):
        self.ingest(Time.from_msg(msg.header.stamp).nanoseconds, msg.header.frame_id, _pose_of(msg.pose.pose))

    def ingest(self, stamp_ns, frame, pose):
        """A station pose, from the topic or handed over in process by station_alignment_stack."""
        if frame != self.get_parameter("base_frame").value:
            self.frame_rejected += 1
            return
        received = self.get_clock().now().nanoseconds
        with self.lock:
            self.sequence += 1
            self.samples.append((self.sequence, stamp_ns, received, pose))
            del self.samples[:-200]

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))
        with self.lock:
            self.latest_odom = (Time.from_msg(msg.header.stamp).nanoseconds, msg.header.frame_id, msg.child_frame_id,
                                pose)

    def _sleep_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            time.sleep(0.02)

    def _collect(self, after):
        """Sample window on simulation time (sampling.SampleWindow): (status, window)."""
        window = sampling.SampleWindow(after.nanoseconds, int(self.get_parameter("samples").value),
                                       float(self.get_parameter("sample_timeout_s").value),
                                       float(self.get_parameter("max_latency_s").value),
                                       int(self.get_parameter("min_samples").value))
        seen = 0
        while rclpy.ok():
            with self.lock:
                new = [s for s in self.samples if s[0] > seen]
                seen = self.sequence
            for _, stamp_ns, received_ns, pose in new:
                if received_ns >= after.nanoseconds or stamp_ns > after.nanoseconds:
                    window.add(stamp_ns, received_ns, pose)
            status = window.poll(self.get_clock().now().nanoseconds)
            if status != sampling.WAITING:
                return status, window
            time.sleep(0.05)
        return sampling.NOT_SEEN, window

    # ------------------------------------------------------------------ Nav2
    def _navigate(self, goal_handle, pose, frame):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        qx, qy, qz, qw = geometry.quaternion_of_yaw(pose[2])
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        goal.behavior_tree = self.get_parameter("behavior_tree").value
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return False, "navigate_to_pose not available"
        sent = self.nav.send_goal_async(goal, feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback))
        timeout_ns = int(float(self.get_parameter("navigation_timeout_s").value) * 1e9)
        deadline = self.get_clock().now().nanoseconds + timeout_ns
        while rclpy.ok() and not sent.done():
            time.sleep(0.02)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False, "Nav2 rejected the goal"
        result = handle.get_result_async()
        while rclpy.ok() and not result.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
            if self.get_clock().now().nanoseconds > deadline:
                handle.cancel_goal_async()
                return False, "Nav2 goal timed out"
            time.sleep(0.05)
        status = result.result().status
        return status == STATUS_SUCCEEDED, f"Nav2 status {status}"

    def _tf_pose(self, target):
        """(x, y, yaw) of base_frame in `target` from /tf, waiting up to tf_wait_s of simulation time."""
        base = self.get_parameter("base_frame").value

        def get():
            try:
                t = self.tf_buffer.lookup_transform(target, base, Time())
            except TransformException:
                return None
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))
        pose, waited = sources.wait_for(get, lambda: self.get_clock().now().nanoseconds,
                                        float(self.get_parameter("tf_wait_s").value), lambda: time.sleep(0.02))
        if pose is None:
            raise TransformException(f"no {target} -> {base} after {waited:.1f} s of simulation time; "
                                     f"{self._tf_diagnosis(target, base)}")
        return pose

    def _tf_diagnosis(self, *frames):
        """The parent of each frame in this node's buffer and the /tf publishers this node sees."""
        try:
            tree = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        except Exception:  # noqa: BLE001 - diagnosis only
            tree = {}
        parents = {f: (tree.get(f) or {}).get("parent", "absent") for f in frames}
        writers = sorted({i.node_name for i in self.get_publishers_info_by_topic("/tf")})
        return f"parents in buffer {parents}, /tf publishers seen {writers}"

    def _map_from_base(self):
        return self._tf_pose(self.get_parameter("global_frame").value)

    def _odom_from_base(self):
        """From the latest /odom message (the transform's own writer) if fresh, else from /tf."""
        with self.lock:
            latest = self.latest_odom
        pose = sources.fresh_odom(latest, self.get_clock().now().nanoseconds,
                                  float(self.get_parameter("odom_max_age_s").value),
                                  self.get_parameter("odom_frame").value, self.get_parameter("base_frame").value)
        return pose if pose is not None else self._tf_pose(self.get_parameter("odom_frame").value)

    def _behavior(self, goal_handle, kind, value):
        """Run one behavior_server action; (ok, text)."""
        allowance = float(self.get_parameter("behavior_time_allowance_s").value)
        speed = float(self.get_parameter("behavior_speed").value)
        goal = (DriveOnHeading if kind == maneuver.DRIVE else BackUp).Goal()
        goal.target.x = float(abs(value)) if kind == maneuver.DRIVE else -float(abs(value))
        goal.speed = speed if kind == maneuver.DRIVE else -speed
        goal.time_allowance = DurationMsg(sec=int(allowance), nanosec=int((allowance % 1.0) * 1e9))
        client = self.behaviors[kind]
        if not client.wait_for_server(timeout_sec=30.0):
            return False, f"{kind} action not available"
        sent = client.send_goal_async(goal)
        while rclpy.ok() and not sent.done():
            time.sleep(0.02)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False, f"{kind} rejected"
        result = handle.get_result_async()
        while rclpy.ok() and not result.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
            time.sleep(0.02)
        status = result.result().status
        return status == STATUS_SUCCEEDED, f"{kind} {value:+.4f} status {status}"

    def _rotate(self, goal_handle, delta):
        """Rotation-only goal of #164's station tree: the current map position, the map heading turned by `delta`.
        Closed on the AMCL heading, which sees the turn the base makes after the wheels stop (odometry does not)."""
        x, y, yaw = self._map_from_base()
        frame = self.get_parameter("global_frame").value
        ok, why = self._navigate(goal_handle, (x, y, geometry.wrap(yaw + delta)), frame)
        return ok, f"rotate {delta:+.4f} {why}"

    def _correct(self, goal_handle, residual, step):
        """Reach the measured station: position target fixed in odom, yaw target in map, see maneuver.py."""
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        start, map_pose = self._odom_from_base(), self._map_from_base()
        plan = maneuver.Plan(maneuver.target_in_odom(start, residual),
                             maneuver.target_yaw_in_map(map_pose[2], residual),
                             residual, float(p("accept_lateral")), float(p("backoff")))
        step["mode"] = "lateral" if plan.lateral else "direct"
        step["target_odom"] = plan.target
        step["target_yaw_map"] = plan.target_yaw_map
        step["moves"] = []
        while True:
            primitive = plan.next(self._odom_from_base(), self._map_from_base()[2])
            if primitive is None:
                return True
            if primitive[0] == maneuver.ROTATE:
                ok, text = self._rotate(goal_handle, primitive[1])
            else:
                ok, text = self._behavior(goal_handle, *primitive)
            step["moves"].append(text)
            if not ok or goal_handle.is_cancel_requested:
                return False

    # ------------------------------------------------------------------ action
    def execute(self, goal_handle):
        request = goal_handle.request
        frame = request.pose.header.frame_id or self.get_parameter("global_frame").value
        target = _pose_of(request.pose.pose)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        report = {"nominal_goal": target, "iterations": [], "outcome": None}
        result = NavigateToPose.Result()

        def not_aligned(why):
            """Marker not usable after the #164 approach: the base is where #164 stopped it. Proposed result:
            SUCCEEDED with error_code NOT_ALIGNED and outcome "not aligned", unless on_marker_not_seen is abort."""
            if p("on_marker_not_seen") == "abort":
                return finish(False, why)
            report["outcome"] = f"not aligned: {why}"
            result.error_code = NOT_ALIGNED
            if hasattr(result, "error_msg"):
                result.error_msg = report["outcome"]
            self.report.publish(String(data=json.dumps(report)))
            self.get_logger().warn(f"station alignment {report['outcome']}")
            goal_handle.succeed()
            return result

        def finish(ok, why):
            report["outcome"] = "aligned" if ok else f"aborted: {why}"
            self.report.publish(String(data=json.dumps(report)))
            self.get_logger().info(f"station alignment {report['outcome']}")
            if ok:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        nominal_ok, why = self._navigate(goal_handle, target, frame)
        report["iterations"].append({"goal": target, "nav2": why})
        if goal_handle.is_cancel_requested:
            return finish(False, "canceled")
        # A failed #164 goal is not the end: the base may stop a few centimetres off (for example the progress
        # checker on a goal it already almost reached). The marker decides; without a marker the failure stands.
        nominal_why = why
        for iteration in range(int(p("max_iterations")) + 1):
            stopped = self.get_clock().now()
            self._sleep_sim(float(p("settle_s")))
            attempt = 0
            while True:
                opened = self.get_clock().now()
                status, window = self._collect(opened)
                if not sampling.reopen(status, attempt, int(p("not_seen_retries"))):
                    break
                attempt += 1
            step = {"iteration": iteration, "stopped_s": stopped.nanoseconds * 1e-9,
                    "window_opened_s": opened.nanoseconds * 1e-9, "window_status": status,
                    "window_attempts": attempt + 1, **window.summary()}
            report["iterations"].append(step)
            if status != sampling.DONE:
                why = ("marker not seen" if status == sampling.NOT_SEEN else "no fresh marker sample") + \
                    f": {window.received} received, {len(window.fresh)} fresh, simulation time"
                step["station_pose_source"] = p("station_pose_source")
                step["frame_rejected"] = self.frame_rejected
                step["station_pose_publishers"] = sorted(
                    {i.node_name for i in self.get_publishers_info_by_topic(p("station_pose_topic"))})
                if sampling.without_marker(iteration, nominal_ok) == "not_aligned":
                    return not_aligned(why)
                return finish(False, why if nominal_ok or iteration else f"{nominal_why}; {why}")
            residual, spread = window.result()
            step.update({"residual": residual, "spread_xy": spread})
            if spread > float(p("max_sample_spread_xy")):
                return finish(False, f"marker samples spread {spread:.4f} m")
            limits = (float(p("accept_along")), float(p("accept_lateral")), float(p("accept_yaw")))
            if geometry.accepted(residual, *limits):
                return finish(True, "")
            if not geometry.within(residual, float(p("max_correction_xy")), float(p("max_correction_yaw"))):
                return finish(False, f"residual {residual} beyond the correction limit")
            if iteration == int(p("max_iterations")):
                return finish(False, f"residual {residual} after {iteration} corrections")
            try:
                step["moves_ok"] = self._correct(goal_handle, residual, step)
            except TransformException as e:
                return finish(False, f"no {p('odom_frame')} pose: {e}")
            if goal_handle.is_cancel_requested:
                return finish(False, "canceled")
        return finish(False, "unreachable")


def main(args=None):
    rclpy.init(args=args)
    node = StationAlignment()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
