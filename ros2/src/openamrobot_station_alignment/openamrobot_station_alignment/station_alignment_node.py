# SPDX-License-Identifier: MIT
"""station_alignment: station goal refinement from the marker, with Nav2 as the only command authority.

Action server (Proposed): /station_alignment/navigate_to_station, type nav2_msgs/action/NavigateToPose. The goal is
the nominal station pose in the map. The node:

1. sends that goal to Nav2's navigate_to_pose with behavior_tree = navigate_to_station.xml (platform-sw #164);
2. after the robot has stopped, averages `samples` station poses from /station_alignment/station_pose (base frame,
   independent of AMCL);
3. if the measured residual is within accept_along, accept_lateral and accept_yaw it succeeds; otherwise it composes the
   robot's current map pose with the measured station pose and sends that corrected goal to the same tree, up to
   `max_iterations` times. A differential base cannot step sideways: a lateral residual up to `accept_lateral`
   is left (the reach tolerates -0.20 to +0.15 m along the shelf) and only the distance and the yaw are corrected in
   place; a larger one is corrected by a goal `backoff` behind the corrected station, then the corrected station, so
   that the controller approaches it on a curve. A correction Nav2 fails is measured again and retried by backing off.

It is an action client of Nav2 only: it publishes no velocity, runs no controller and starts no navigator. Motion is
controller_server's StationApproach controller and station_goal_checker, through the velocity smoother and the
collision monitor, as for every other goal. A residual larger than max_correction_* aborts instead of moving.
A diagnostic JSON line per goal goes to /station_alignment/report (Proposed).
"""
import json
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

from openamrobot_station_alignment import geometry, sampling

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
        self.declare_parameter("backoff", 0.30)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.samples = []
        self.sequence = 0
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
        received = self.get_clock().now().nanoseconds
        with self.lock:
            self.sequence += 1
            self.samples.append((self.sequence, Time.from_msg(msg.header.stamp).nanoseconds, received,
                                 _pose_of(msg.pose.pose)))
            del self.samples[:-200]

    def _sleep_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            time.sleep(0.02)

    def _collect(self, after):
        """Sample window on simulation time (sampling.SampleWindow): (status, window)."""
        window = sampling.SampleWindow(after.nanoseconds, int(self.get_parameter("samples").value),
                                       float(self.get_parameter("sample_timeout_s").value),
                                       float(self.get_parameter("max_latency_s").value))
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
        failed = not nominal_ok
        for iteration in range(int(p("max_iterations")) + 1):
            stopped = self.get_clock().now()
            self._sleep_sim(float(p("settle_s")))
            opened = self.get_clock().now()
            status, window = self._collect(opened)
            step = {"iteration": iteration, "stopped_s": stopped.nanoseconds * 1e-9,
                    "window_opened_s": opened.nanoseconds * 1e-9, "window_status": status, **window.summary()}
            report["iterations"].append(step)
            if status != sampling.DONE:
                why = ("marker not seen" if status == sampling.NOT_SEEN else "no fresh marker sample") + \
                    f": {window.received} received, {len(window.fresh)} fresh, simulation time"
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
            direct = abs(residual[1]) <= float(p("accept_lateral")) and not failed
            target_in_base = (residual[0], 0.0, residual[2]) if direct else residual
            try:
                map_from_base = self._map_from_base()
            except TransformException as e:
                return finish(False, f"no {p('global_frame')} pose: {e}")
            corrected = geometry.corrected_goal(map_from_base, target_in_base)
            step["corrected_goal"] = corrected
            step["mode"] = "direct" if direct else "backoff"
            failed = False
            if not direct:
                staging = geometry.compose(corrected, (-float(p("backoff")), 0.0, 0.0))
                ok, why = self._navigate(goal_handle, staging, p("global_frame"))
                step["staging_nav2"] = why
                if goal_handle.is_cancel_requested:
                    return finish(False, "canceled")
            ok, why = self._navigate(goal_handle, corrected, p("global_frame"))
            step["nav2"] = why
            if goal_handle.is_cancel_requested:
                return finish(False, "canceled")
            failed = not ok
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
