# SPDX-License-Identifier: MIT
"""station_alignment: station goal refinement from the marker, with Nav2 as the only command authority.

Action server (Proposed): /station_alignment/navigate_to_station, type nav2_msgs/action/NavigateToPose. The goal is
the nominal station pose in the map. The node:

1. sends that goal to Nav2's navigate_to_pose with behavior_tree = navigate_to_station.xml (platform-sw #164);
2. after the robot has stopped, averages `samples` station poses from /station_alignment/station_pose (base frame,
   independent of AMCL);
3. if the measured residual is within the station goal checker tolerance it succeeds; otherwise it composes the
   robot's current map pose with the measured station pose and sends that corrected goal to the same tree, up to
   `max_iterations` times.

It is an action client of Nav2 only: it publishes no velocity, runs no controller and starts no navigator. Motion is
controller_server's StationApproach controller and station_goal_checker, through the velocity smoother and the
collision monitor, as for every other goal. A residual larger than max_correction_* aborts instead of moving.
A diagnostic JSON line per goal goes to /station_alignment/report (Proposed).
"""
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_station_alignment import geometry

STATUS_SUCCEEDED = 4


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
    def __init__(self):
        super().__init__("station_alignment")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("action_name", "/station_alignment/navigate_to_station")
        self.declare_parameter("behavior_tree", default_station_tree())
        self.declare_parameter("station_pose_topic", "/station_alignment/station_pose")
        self.declare_parameter("report_topic", "/station_alignment/report")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("samples", 8)
        self.declare_parameter("sample_timeout_s", 6.0)
        self.declare_parameter("max_iterations", 3)
        self.declare_parameter("accept_xy", 0.025)
        self.declare_parameter("accept_yaw", 0.05)
        self.declare_parameter("max_correction_xy", 0.15)
        self.declare_parameter("max_correction_yaw", 0.30)
        self.declare_parameter("max_sample_spread_xy", 0.01)
        self.declare_parameter("navigation_timeout_s", 300.0)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.samples = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value,
                                callback_group=self.group)
        self.report = self.create_publisher(String, self.get_parameter("report_topic").value, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("station_pose_topic").value,
                                 self.on_station_pose, 20, callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, self.get_parameter("action_name").value,
                                   execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)

    # ------------------------------------------------------------------ inputs
    def on_station_pose(self, msg):
        if msg.header.frame_id != self.get_parameter("base_frame").value:
            return
        with self.lock:
            self.samples.append((Time.from_msg(msg.header.stamp), _pose_of(msg.pose.pose)))
            del self.samples[:-100]

    def _sleep_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            time.sleep(0.02)

    def _collect(self, after):
        """Mean of `samples` station poses stamped after `after`, or None on timeout or spread."""
        n = int(self.get_parameter("samples").value)
        deadline = time.monotonic() + 10.0 * float(self.get_parameter("sample_timeout_s").value)
        sim_deadline = after.nanoseconds + int(float(self.get_parameter("sample_timeout_s").value) * 1e9)
        while rclpy.ok():
            with self.lock:
                fresh = [p for t, p in self.samples if t.nanoseconds > after.nanoseconds]
            if len(fresh) >= n:
                fresh = fresh[-n:]
                m = geometry.mean_pose(fresh)
                spread = max(math.hypot(p[0] - m[0], p[1] - m[1]) for p in fresh)
                return m, spread, len(fresh)
            if self.get_clock().now().nanoseconds > sim_deadline or time.monotonic() > deadline:
                return None, None, len(fresh)
            time.sleep(0.05)
        return None, None, 0

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
        deadline = time.monotonic() + float(self.get_parameter("navigation_timeout_s").value) * 10.0
        while rclpy.ok() and not sent.done():
            time.sleep(0.02)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False, "Nav2 rejected the goal"
        result = handle.get_result_async()
        while rclpy.ok() and not result.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                return False, "Nav2 goal timed out"
            time.sleep(0.05)
        status = result.result().status
        return status == STATUS_SUCCEEDED, f"Nav2 status {status}"

    def _map_from_base(self):
        t = self.tf_buffer.lookup_transform(self.get_parameter("global_frame").value,
                                            self.get_parameter("base_frame").value, Time())
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------ action
    def execute(self, goal_handle):
        request = goal_handle.request
        frame = request.pose.header.frame_id or self.get_parameter("global_frame").value
        target = _pose_of(request.pose.pose)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        report = {"nominal_goal": target, "iterations": [], "outcome": None}
        result = NavigateToPose.Result()

        def finish(ok, why):
            report["outcome"] = "succeeded" if ok else f"aborted: {why}"
            self.report.publish(String(data=json.dumps(report)))
            self.get_logger().info(f"station alignment {report['outcome']}")
            if ok:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        ok, why = self._navigate(goal_handle, target, frame)
        report["iterations"].append({"goal": target, "nav2": why})
        if not ok:
            return finish(False, why)
        for iteration in range(int(p("max_iterations")) + 1):
            stopped = self.get_clock().now()
            self._sleep_sim(float(p("settle_s")))
            residual, spread, count = self._collect(self.get_clock().now())
            step = {"iteration": iteration, "samples": count, "stopped_s": stopped.nanoseconds * 1e-9}
            report["iterations"].append(step)
            if residual is None:
                return finish(False, f"marker not seen: {count} samples in {p('sample_timeout_s')} s")
            step.update({"residual": residual, "spread_xy": spread})
            if spread > float(p("max_sample_spread_xy")):
                return finish(False, f"marker samples spread {spread:.4f} m")
            if geometry.within(residual, float(p("accept_xy")), float(p("accept_yaw"))):
                return finish(True, "")
            if not geometry.within(residual, float(p("max_correction_xy")), float(p("max_correction_yaw"))):
                return finish(False, f"residual {residual} beyond the correction limit")
            if iteration == int(p("max_iterations")):
                return finish(False, f"residual {residual} after {iteration} corrections")
            try:
                corrected = geometry.corrected_goal(self._map_from_base(), residual)
            except TransformException as e:
                return finish(False, f"no {p('global_frame')} pose: {e}")
            step["corrected_goal"] = corrected
            ok, why = self._navigate(goal_handle, corrected, p("global_frame"))
            step["nav2"] = why
            if not ok:
                return finish(False, why)
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
