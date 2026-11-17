# SPDX-License-Identifier: MIT
"""station_marker_detector: AprilTag detections of the station marker plate -> station pose in base_footprint.

Input: apriltag_ros (/apriltag/detections and the camera -> tag transforms it broadcasts), the robot's TF tree.
Output (Proposed): /station_alignment/station_pose, geometry_msgs/PoseWithCovarianceStamped in `base_frame`, stamped
with the image time: where the station is relative to the robot, independent of AMCL. Covariance: sample covariance
of the last `window` estimates (reset when the estimate jumps by more than the gate, that is when the robot moves),
plus a floor. Perception only: this node publishes no velocity and sends no goal.
"""
import collections

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from openamrobot_station_alignment import geometry


class StationMarkerDetector(Node):
    def __init__(self):
        super().__init__("station_marker_detector")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("detections_topic", "/apriltag/detections")
        self.declare_parameter("station_pose_topic", "/station_alignment/station_pose")
        self.declare_parameter("tag_ids", [3, 4])
        self.declare_parameter("tag_frames", ["station_marker_3", "station_marker_4"])
        self.declare_parameter("tag_offsets", [-0.10, 0.10])
        self.declare_parameter("standoff", 0.713)
        self.declare_parameter("lateral", 0.0)
        self.declare_parameter("window", 10)
        self.declare_parameter("gate_xy", 0.02)
        self.declare_parameter("gate_yaw", 0.04)
        self.declare_parameter("floor_xy", 0.003)
        self.declare_parameter("floor_yaw", 0.005)
        self.declare_parameter("single_tag_floor_yaw", 0.03)
        self.declare_parameter("min_tags", 2)
        self.declare_parameter("tf_timeout", 1.0)
        p = self.get_parameter
        ids = [int(i) for i in p("tag_ids").value]
        frames = list(p("tag_frames").value)
        offsets = [float(o) for o in p("tag_offsets").value]
        if not (len(ids) == len(frames) == len(offsets)):
            raise ValueError("tag_ids, tag_frames and tag_offsets differ in length")
        self.frames = dict(zip(ids, frames))
        self.layout = geometry.Layout(dict(zip(ids, offsets)), p("standoff").value, p("lateral").value)
        self.base_frame = p("base_frame").value
        self.window = collections.deque(maxlen=int(p("window").value))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = collections.deque()
        self.pub = self.create_publisher(PoseWithCovarianceStamped, p("station_pose_topic").value, 10)
        self.create_subscription(AprilTagDetectionArray, p("detections_topic").value, self.on_detections, 10)
        self.stats = collections.Counter()
        self.create_timer(10.0, self.log_stats)
        self.create_timer(0.05, self.process_pending)

    def log_stats(self):
        """Counts of the last 10 s (node clock): detection messages, ids seen, TF failures, poses published."""
        if self.stats:
            self.get_logger().info("stats " + " ".join(f"{k}={v}" for k, v in sorted(self.stats.items())))
        self.stats.clear()

    def _tag(self, frame, stamp):
        t = self.tf_buffer.lookup_transform(self.base_frame, frame, stamp)
        q = t.transform.rotation
        v = t.transform.translation
        return (v.x, v.y, v.z), geometry.rotation_matrix(q.x, q.y, q.z, q.w)

    def on_detections(self, msg):
        """Queue the detection: apriltag_ros publishes the detections before the tag transforms of the same image."""
        self.stats["messages"] += 1
        seen = sorted({int(d.id) for d in msg.detections if int(d.id) in self.frames})
        self.stats[f"seen_{len(seen)}"] += 1
        if len(seen) >= int(self.get_parameter("min_tags").value):
            self.pending.append((Time.from_msg(msg.header.stamp), msg.header.stamp, seen))

    def process_pending(self):
        """Estimate every queued image whose tag transforms have arrived; drop those older than tf_timeout."""
        now = self.get_clock().now()
        keep = collections.deque()
        timeout = Duration(seconds=float(self.get_parameter("tf_timeout").value))
        while self.pending:
            stamp, stamp_msg, seen = self.pending.popleft()
            ready = [i for i in seen if self.tf_buffer.can_transform(self.base_frame, self.frames[i], stamp)]
            if len(ready) == len(seen):
                self.estimate(stamp_msg, {i: self._tag(self.frames[i], stamp) for i in ready})
            elif now - stamp > timeout:
                self.stats["tf_failed"] += 1
                if len(ready) >= int(self.get_parameter("min_tags").value):
                    self.estimate(stamp_msg, {i: self._tag(self.frames[i], stamp) for i in ready})
            else:
                keep.append((stamp, stamp_msg, seen))
        self.pending = keep

    def estimate(self, stamp_msg, tags):
        try:
            pose, mode = geometry.station_from_tags(self.layout, tags)
        except ValueError as e:
            self.get_logger().warn(f"station from tags {sorted(tags)}: {e}")
            return
        if self.window:
            m = geometry.mean_pose(list(self.window))
            if not geometry.within((pose[0] - m[0], pose[1] - m[1], pose[2] - m[2]),
                                   self.get_parameter("gate_xy").value, self.get_parameter("gate_yaw").value):
                self.window.clear()
        self.window.append(pose)
        floor_yaw = self.get_parameter("floor_yaw").value
        if mode == "single":
            floor_yaw = max(floor_yaw, self.get_parameter("single_tag_floor_yaw").value)
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp_msg
        out.header.frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = pose[0], pose[1]
        qx, qy, qz, qw = geometry.quaternion_of_yaw(pose[2])
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        out.pose.covariance = geometry.covariance(list(self.window), self.get_parameter("floor_xy").value, floor_yaw)
        self.pub.publish(out)
        self.stats["published"] += 1
        # stamp age at publication on this node's clock, ms (simulation time with use_sim_time)
        age_ms = int((self.get_clock().now() - Time.from_msg(stamp_msg)).nanoseconds * 1e-6)
        self.stats["age_ms_max"] = max(self.stats["age_ms_max"], age_ms)


def main(args=None):
    rclpy.init(args=args)
    node = StationMarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
