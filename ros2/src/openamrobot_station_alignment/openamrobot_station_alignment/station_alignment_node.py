# SPDX-License-Identifier: MIT
"""station_alignment: station goal refinement from the marker, with Nav2 as the only command authority.

Action server (Proposed): /station_alignment/navigate_to_station, type nav2_msgs/action/NavigateToPose. The goal is
the nominal station pose in the map. The node:

1. sends that goal to Nav2's navigate_to_pose with behavior_tree = navigate_to_station.xml (platform-sw #164);
2. after the robot has stopped, averages `samples` station poses from /station_alignment/station_pose (base frame,
   independent of AMCL);
3. if the measured residual is within accept_along, accept_lateral and accept_yaw it succeeds; otherwise it fixes
   the measured station's position in odom and its heading in map and reaches them (maneuver.py), forward only
   (D-19): rotation-only goals and forward drives, all NavigateToPose goals of the station tree. A lateral residual
   above accept_lateral, or a target behind the robot, is reached by turning to it first. Then it measures again,
   up to `max_iterations` corrections. A correction goal that fails is measured again like any correction.

It is an action client of Nav2's navigate_to_pose only: it publishes no velocity, runs no controller, starts no
navigator and calls no behavior_server action. Motion is controller_server's StationApproach controller and
station_goal_checker, through Nav2's velocity smoother and collision monitor. Stop (/safety/stop_state not RUNNING,
or a sub-goal CANCELED by another client) ends the station goal and nothing more is sent. A residual larger than
max_correction_* aborts instead of moving. A diagnostic JSON line per goal goes to /station_alignment/report (Proposed).
"""
import collections
import json
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from openamr_nav_msgs.msg import StopState
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


from openamrobot_station_alignment import geometry, maneuver, sampling, sources

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
# Proposed result code (NavigateToPose.Result.error_code, outside Nav2's 9000 to 9011): the station goal of #164
# succeeded but the marker gave no usable sample, so no correction was made.
NOT_ALIGNED = 9900


class Halted(Exception):
    """Stop engaged (a Nav2 sub-goal CANCELED that this node did not cancel, or /safety/stop_state not RUNNING):
    the station goal ends and nothing more is sent (software lead's review of aa8fd11, item b; the rule of
    openamrobot-interfaces #40, item 3)."""


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
        self.declare_parameter("on_marker_not_seen", "abort")
        self.declare_parameter("max_iterations", 3)
        self.declare_parameter("accept_along", 0.025)
        self.declare_parameter("accept_lateral", 0.02)
        self.declare_parameter("accept_yaw", 0.06)
        self.declare_parameter("max_correction_xy", 0.15)
        self.declare_parameter("max_correction_yaw", 0.30)
        self.declare_parameter("max_sample_spread_xy", 0.003)
        self.declare_parameter("max_sample_spread_yaw", 0.01)
        self.declare_parameter("odom_history_tolerance_s", 0.5)
        self.declare_parameter("navigation_timeout_s", 300.0)
        self.declare_parameter("correction_timeout_s", 60.0)
        self.declare_parameter("drive_lead", 0.025)
        self.declare_parameter("stop_state_topic", "/safety/stop_state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_max_age_s", 0.5)
        self.declare_parameter("tf_wait_s", 5.0)
        self.declare_parameter("station_pose_source", "topic")
        samples, minimum = int(self.get_parameter("samples").value), int(self.get_parameter("min_samples").value)
        if not 1 <= minimum <= samples:
            raise ValueError(f"min_samples {minimum} must be between 1 and samples {samples}")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.samples = []
        self.sequence = 0
        self.window_stamp_ns = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value,
                                callback_group=self.group)
        self.report = self.create_publisher(String, self.get_parameter("report_topic").value, 10)
        self.frame_rejected = 0
        self.odom_history = collections.deque(maxlen=400)
        self.active = False
        self.latest_odom = None
        self.stop_state = None    # last /safety/stop_state; None until the gate (platform-sw #90) publishes one
        if self.get_parameter("station_pose_source").value == "topic":
            self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("station_pose_topic").value,
                                     self.on_station_pose, 20, callback_group=self.group)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 20,
                                 callback_group=self.group)
        # The gate publishes reliable, transient local (safety_stop_gate_node, platform-sw #90).
        stop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(StopState, self.get_parameter("stop_state_topic").value, self.on_stop_state,
                                 stop_qos, callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, self.get_parameter("action_name").value,
                                   execute_callback=self.execute, goal_callback=self._goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)

    # ------------------------------------------------------------------ inputs
    def on_station_pose(self, msg):
        self.ingest(Time.from_msg(msg.header.stamp).nanoseconds, msg.header.frame_id, _pose_of(msg.pose.pose))

    def on_stop_state(self, msg):
        with self.lock:
            self.stop_state = int(msg.state)

    def _check_stop(self):
        """Raise Halted unless the stop gate is RUNNING. No message yet (no gate on this stack) counts as running."""
        with self.lock:
            state = self.stop_state
        if state is not None and state != StopState.STATE_RUNNING:
            raise Halted(f"/safety/stop_state {state}, not RUNNING")

    def ingest(self, stamp_ns, frame, pose):
        """A station pose, from the topic or handed over in process by station_alignment_stack."""
        if frame != self.get_parameter("base_frame").value:
            with self.lock:
                self.frame_rejected += 1
            return
        received = self.get_clock().now().nanoseconds
        with self.lock:
            self.sequence += 1
            self.samples.append((self.sequence, stamp_ns, received, pose))
            del self.samples[:-200]

    @property
    def latest_odom(self):
        return self._latest_odom

    @latest_odom.setter
    def latest_odom(self, value):
        """The latest /odom (stamp_ns, frame, child, pose); every value is also kept in odom_history."""
        self._latest_odom = value
        if value is not None:
            self.odom_history.append((value[0], value[3]))

    def _goal(self, _request):
        """One goal at a time: a second goal while one runs is rejected (review of c4666a1, 5a)."""
        with self.lock:
            if self.active:
                return GoalResponse.REJECT
            self.active = True
            return GoalResponse.ACCEPT

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))
        with self.lock:
            self.latest_odom = (Time.from_msg(msg.header.stamp).nanoseconds, msg.header.frame_id, msg.child_frame_id,
                                pose)

    def _sleep_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            self._check_stop()
            time.sleep(0.02)

    def _collect(self, after):
        """Sample window on simulation time (sampling.SampleWindow): (status, window)."""
        window = sampling.SampleWindow(after.nanoseconds, int(self.get_parameter("samples").value),
                                       float(self.get_parameter("sample_timeout_s").value),
                                       float(self.get_parameter("max_latency_s").value),
                                       int(self.get_parameter("min_samples").value))
        seen = 0
        while rclpy.ok():
            self._check_stop()
            now = self.get_clock().now().nanoseconds
            with self.lock:
                self.samples = sources.drop_other_generation(self.samples, now)
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
    def _navigate(self, goal_handle, pose, frame, timeout_s=None):
        """One NavigateToPose goal; (ok, text). Deadline on simulation time: navigation_timeout_s, or timeout_s for
        a correction. Raises Halted when Stop is engaged before or during the goal, or when Nav2 reports the goal
        CANCELED without this node having cancelled it (the gate cancels on Stop): nothing more is sent."""
        self._check_stop()
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
        timeout = float(self.get_parameter("navigation_timeout_s").value) if timeout_s is None else float(timeout_s)
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and not sent.done():
            time.sleep(0.02)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False, "Nav2 rejected the goal"
        result = handle.get_result_async()
        own_cancel = False
        while rclpy.ok() and not result.done():
            if goal_handle.is_cancel_requested:
                own_cancel = True
                handle.cancel_goal_async()
            try:
                self._check_stop()
            except Halted:
                handle.cancel_goal_async()
                raise
            if self.get_clock().now().nanoseconds > deadline:
                handle.cancel_goal_async()
                return False, f"Nav2 goal timed out after {timeout:.0f} s (simulation time)"
            time.sleep(0.05)
        status = result.result().status
        if status == STATUS_CANCELED and not (own_cancel or goal_handle.is_cancel_requested):
            raise Halted("Nav2 sub-goal CANCELED (status 5) by another client, the stop gate on Stop")
        return status == STATUS_SUCCEEDED, f"Nav2 status {status}"

    def _tf_pose(self, target, max_age_s=None):
        """(x, y, yaw) of base_frame in `target` from /tf, waiting up to tf_wait_s of simulation time. With
        max_age_s, a transform older than that (or stamped in the future) is not used (review of c4666a1, 2c)."""
        base = self.get_parameter("base_frame").value

        def get():
            try:
                t = self.tf_buffer.lookup_transform(target, base, Time())
            except TransformException:
                return None
            if max_age_s is not None:
                stamp = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
                if not sources.within_age(stamp, self.get_clock().now().nanoseconds, max_age_s):
                    return None
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))
        pose, waited = sources.wait_for(get, lambda: self.get_clock().now().nanoseconds,
                                        float(self.get_parameter("tf_wait_s").value), lambda: time.sleep(0.02))
        if pose is None:
            age = f" not older than {max_age_s} s" if max_age_s is not None else ""
            raise TransformException(f"no {target} -> {base}{age} after {waited:.1f} s of simulation time; "
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
        if pose is not None:
            return pose
        return self._tf_pose(self.get_parameter("odom_frame").value, float(self.get_parameter("odom_max_age_s").value))

    def _drive(self, goal_handle, distance):
        """Forward drive of `distance` on the current map heading: a goal of #164's station tree `drive_lead` beyond
        the point (maneuver.forward_goal). Forward only (D-19; review of aa8fd11, item a)."""
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        goal = maneuver.forward_goal(self._map_from_base(), distance, float(p("drive_lead")))
        ok, why = self._navigate(goal_handle, goal, p("global_frame"), float(p("correction_timeout_s")))
        return ok, f"drive {distance:+.4f} {why}"

    def _rotate(self, goal_handle, target_yaw):
        """Rotation-only goal of #164's station tree: the current map position and the absolute map heading
        `target_yaw` (from the plan's own read; Kunal's review of aa8fd11, point 4). Closed on the AMCL heading,
        which sees the turn the base makes after the wheels stop (odometry does not)."""
        x, y, _ = self._map_from_base()
        frame = self.get_parameter("global_frame").value
        ok, why = self._navigate(goal_handle, (x, y, geometry.wrap(target_yaw)), frame,
                                 float(self.get_parameter("correction_timeout_s").value))
        return ok, f"rotate to {target_yaw:+.4f} {why}"

    def _correct(self, goal_handle, residual, step):
        """Reach the measured station: position target fixed in odom, yaw target in map, see maneuver.py."""
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        now_odom, map_pose = self._odom_from_base(), self._map_from_base()
        # The marker measured the station relative to the robot at the image stamps: compose with the odom pose
        # at those stamps, not with a later one (review of c4666a1, 2d). The map heading is taken now and moved
        # back by the odom turn since the images.
        with self.lock:
            history = list(self.odom_history)
        at_images = None
        if self.window_stamp_ns is not None:
            at_images = sources.odom_at(history, self.window_stamp_ns, float(p("odom_history_tolerance_s")))
        start = at_images if at_images is not None else now_odom
        step["odom_at_images"] = at_images is not None
        yaw_since = geometry.wrap(now_odom[2] - start[2])
        plan = maneuver.Plan(maneuver.target_in_odom(start, residual),
                             maneuver.target_yaw_in_map(geometry.wrap(map_pose[2] - yaw_since), residual),
                             residual, float(p("accept_lateral")))
        step["mode"] = "turn_first" if plan.turn_first else "direct"
        step["target_odom"] = plan.target
        step["target_yaw_map"] = plan.target_yaw_map
        step["moves"] = []
        while True:
            primitive = plan.next(self._odom_from_base(), self._map_from_base()[2])
            if primitive is None:
                return True
            if primitive[0] == maneuver.ROTATE:
                ok, text = self._rotate(goal_handle, primitive[2])
            else:
                ok, text = self._drive(goal_handle, primitive[1])
            step["moves"].append(text)
            if not ok or goal_handle.is_cancel_requested:
                return False

    # ------------------------------------------------------------------ action
    def execute(self, goal_handle):
        try:
            return self._execute(goal_handle)
        finally:
            with self.lock:
                self.active = False

    def _execute(self, goal_handle):
        request = goal_handle.request
        frame = request.pose.header.frame_id or self.get_parameter("global_frame").value
        target = _pose_of(request.pose.pose)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        report = {"nominal_goal": target, "iterations": [], "outcome": None}
        result = NavigateToPose.Result()

        def not_aligned(why):
            """Marker not usable after the #164 approach: the base is where #164 stopped it. error_code NOT_ALIGNED
            and outcome "not aligned", with status ABORTED (on_marker_not_seen: abort, the default) or SUCCEEDED
            (succeed_unaligned, opt-in). Abort is the default because most clients read only the status (software
            lead's review of aa8fd11, item c)."""
            report["outcome"] = f"not aligned: {why}"
            result.error_code = NOT_ALIGNED
            if hasattr(result, "error_msg"):
                result.error_msg = report["outcome"]
            self.report.publish(String(data=json.dumps(report)))
            self.get_logger().warn(f"station alignment {report['outcome']}")
            if p("on_marker_not_seen") == "succeed_unaligned":
                goal_handle.succeed()
            else:
                goal_handle.abort()
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

        def run():
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
                    with self.lock:
                        step["frame_rejected"] = self.frame_rejected
                    step["station_pose_publishers"] = sorted(
                        {i.node_name for i in self.get_publishers_info_by_topic(p("station_pose_topic"))})
                    if sampling.without_marker(iteration, nominal_ok) == "not_aligned":
                        return not_aligned(why)
                    return finish(False, why if nominal_ok or iteration else f"{nominal_why}; {why}")
                residual, spread = window.result()
                self.window_stamp_ns = window.stamp_ns
                step.update({"residual": residual, "spread_xy": spread, "spread_yaw": window.spread_yaw})
                if spread > float(p("max_sample_spread_xy")):
                    return finish(False, f"marker samples spread {spread:.4f} m")
                if window.spread_yaw > float(p("max_sample_spread_yaw")):
                    return finish(False, f"marker samples yaw spread {window.spread_yaw:.4f} rad")
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

        try:
            return run()
        except Halted as e:   # Stop: end the station goal, send nothing more
            report["stopped"] = str(e)
            return finish(False, f"stopped: {e}")


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
