# SPDX-License-Identifier: MIT
"""station_alignment_trials: N trials in the station alignment world, #164 alone against #164 plus the marker.

Each trial: drive to a start pose with the default tree; send the station goal with navigate_to_station.xml (#164
alone) and record the Gazebo ground truth error; then send the same goal to /station_alignment/navigate_to_station
(#164 plus marker refinement) and record the ground truth error again. The runner also records AMCL against ground
truth, the marker estimate against ground truth, and every publisher of /cmd_vel and /cmd_vel_nav.

Writes JSON and a Markdown table. Exit status 0 only when every trial completed, every aligned pose is within the reach
budget (0.04 m, 0.08 rad, openamr-upperbody-sw docs/integration/station_approach.md) and the only /cmd_vel publisher
is collision_monitor. Simulation evidence only: not robot, site or safety acceptance.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from openamrobot_station_alignment import evaluation, geometry, scene
from openamrobot_station_alignment.evaluation import (COMMAND_PUBLISHERS, REACH_XY, REACH_YAW, START_POSES,
                                                      command_publishers_ok, evaluate, summary)
from openamrobot_station_alignment.station_alignment_node import default_station_tree


class Runner(Node):
    def __init__(self):
        super().__init__("station_alignment_trials",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.latest = {}
        self.markers = []
        self.create_subscription(Odometry, "/station_alignment/ground_truth/odom", self._store("truth"), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._store("amcl"), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/station_alignment/station_pose",
                                 self._store("marker"), 10)
        self.create_subscription(String, "/station_alignment/report", self._store("report"), 10)
        self.counts = {}
        for topic, kind in (("/base_camera/color/image_raw", Image),
                            ("/base_camera/color/camera_info_synced", CameraInfo),
                            ("/apriltag/detections", AprilTagDetectionArray)):
            self.create_subscription(kind, topic, self._count(topic), qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.align = ActionClient(self, NavigateToPose, "/station_alignment/navigate_to_station")

    def _store(self, key):
        def cb(msg):
            with self.lock:
                self.latest[key] = msg
                if key == "marker":
                    self.counts["station_pose"] = self.counts.get("station_pose", 0) + 1
                    q = msg.pose.pose.orientation
                    self.markers.append((Time.from_msg(msg.header.stamp).nanoseconds,
                                         (msg.pose.pose.position.x, msg.pose.pose.position.y,
                                          geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))))
                    del self.markers[:-200]
        return cb

    def _count(self, topic):
        def cb(msg):
            with self.lock:
                self.counts[topic] = self.counts.get(topic, 0) + 1
                if topic == "/apriltag/detections":
                    ids = tuple(sorted(int(d.id) for d in msg.detections))
                    key = f"detections {ids}"
                    self.counts[key] = self.counts.get(key, 0) + 1
        return cb

    def take_counts(self):
        with self.lock:
            out, self.counts = self.counts, {}
        return out

    def get(self, key):
        with self.lock:
            return self.latest.get(key)

    def truth(self):
        m = self.get("truth")
        return None if m is None else (m.pose.pose.position.x, m.pose.pose.position.y,
                                       geometry.yaw_of_quaternion(m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                                                  m.pose.pose.orientation.z, m.pose.pose.orientation.w))

    def amcl(self):
        m = self.get("amcl")
        if m is None:
            return None
        p = m.pose.pose
        return (p.position.x, p.position.y,
                geometry.yaw_of_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def sleep_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        wall = time.monotonic() + seconds * 20 + 5
        while self.get_clock().now().nanoseconds < end and time.monotonic() < wall:
            time.sleep(0.05)

    def wait_for(self, what, condition, wall_s):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.2)
        self.get_logger().error(f"timed out waiting for {what}")
        return False

    def send(self, client, pose, tree, wall_s):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        qx, qy, qz, qw = geometry.quaternion_of_yaw(pose[2])
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        goal.behavior_tree = tree
        started = time.monotonic()
        for _ in range(60):
            # Nav2's action server exists before bt_navigator is active, and rejects goals until then.
            future = client.send_goal_async(goal)
            while not future.done():
                time.sleep(0.05)
            handle = future.result()
            if handle.accepted:
                break
            time.sleep(5.0)
        if not handle.accepted:
            return "rejected", time.monotonic() - started
        result = handle.get_result_async()
        while not result.done():
            if time.monotonic() - started > wall_s:
                handle.cancel_goal_async()
                return "timeout", time.monotonic() - started
            time.sleep(0.1)
        return {4: "succeeded", 5: "canceled", 6: "aborted"}.get(result.result().status, str(result.result().status)), \
            time.monotonic() - started

    def release_software_stop(self, present_wall_s=20.0, release_wall_s=90.0):
        """Operator release of the software Stop (openamrobot_nav2 safety_stop_gate_node, #90) before the first goal.
        The gate starts locked and refuses goals until /safety/stop_lock_release succeeds. -> record for the result
        file. Only the gate's public service is called. A functional stop of the test stack, not the hardwired E-stop."""
        client = self.create_client(Trigger, "/safety/stop_lock_release")
        record = {"gate_present": client.wait_for_service(timeout_sec=present_wall_s), "released": None, "message": ""}
        if not record["gate_present"]:
            return record
        end = time.monotonic() + release_wall_s
        while time.monotonic() < end:
            future = client.call_async(Trigger.Request())
            wait = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < wait:
                time.sleep(0.05)
            if future.done() and future.result() is not None:
                record["released"], record["message"] = future.result().success, future.result().message
                if record["released"]:
                    break
            time.sleep(1.0)
        record["released"] = bool(record["released"])
        return record

    def measure(self, station):
        settled = self.get_clock().now().nanoseconds + int(0.5 * 1e9)
        self.sleep_sim(2.5)
        truth, amcl = self.truth(), self.amcl()
        out = {"truth": truth, "amcl": amcl}
        if truth is not None:
            exy, eyaw, along, left = geometry.pose_error(truth, station)
            out.update(error_xy=exy, error_yaw=eyaw, error_along=along, error_left=left)
        if truth is not None and amcl is not None:
            out["amcl_error_xy"] = math.hypot(amcl[0] - truth[0], amcl[1] - truth[1])
            out["amcl_error_yaw"] = abs(geometry.wrap(amcl[2] - truth[2]))
        # The marker estimate of the stopped robot: the mean of the estimates stamped after it settled.
        with self.lock:
            fresh = [p for t, p in self.markers if t > settled]
        out["marker_samples"] = len(fresh)
        if truth is not None and fresh:
            measured = geometry.mean_pose(fresh)
            true_rel = geometry.between(truth, station)
            e = geometry.between(true_rel, measured)
            out.update(marker_measured=measured, marker_true=true_rel, marker_error_xy=math.hypot(e[0], e[1]),
                       marker_error_yaw=abs(e[2]),
                       marker_spread_xy=max(math.hypot(p[0] - measured[0], p[1] - measured[1]) for p in fresh))
        return out

    def publishers(self):
        out = {}
        for topic in COMMAND_PUBLISHERS:
            out[topic] = sorted({i.node_name for i in self.get_publishers_info_by_topic(topic)})
        return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--out", default="station_alignment_trials.json")
    parser.add_argument("--markdown", default="station_alignment_trials.md")
    parser.add_argument("--startup-timeout", type=float, default=900.0)
    parser.add_argument("--goal-timeout", type=float, default=1800.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Runner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    station = scene.station()
    tree = default_station_tree()
    record = {"station": station, "behavior_tree": tree, "reach_budget": [REACH_XY, REACH_YAW], "trials": []}
    ready = (node.wait_for("ground truth", lambda: node.truth() is not None, args.startup_timeout)
             and node.wait_for("navigate_to_pose", lambda: node.nav.wait_for_server(timeout_sec=1.0),
                               args.startup_timeout)
             and node.wait_for("station alignment", lambda: node.align.wait_for_server(timeout_sec=1.0), 120.0)
             and node.wait_for("AMCL pose", lambda: node.amcl() is not None, 300.0))
    if ready:
        record["software_stop"] = node.release_software_stop()
        print(json.dumps({"software_stop": record["software_stop"]}), flush=True)
        ready = not record["software_stop"]["gate_present"] or record["software_stop"]["released"]
    record["ready"] = ready
    if ready:
        for index in range(args.trials):
            start = START_POSES[index % len(START_POSES)]
            trial = {"trial": index + 1, "start": start}
            t0 = time.monotonic()
            sim0 = node.get_clock().now().nanoseconds
            node.take_counts()
            status, _ = node.send(node.nav, start, "", args.goal_timeout)
            trial["to_start"] = status
            status, wall = node.send(node.nav, station, tree, args.goal_timeout)
            trial["baseline"] = {"nav2": status, "wall_s": wall, **node.measure(station)}
            with node.lock:
                node.latest.pop("report", None)
            status, wall = node.send(node.align, station, tree, args.goal_timeout)
            aligned = {"nav2": status, "wall_s": wall, **node.measure(station)}
            report = node.get("report")
            if report is not None:
                aligned["report"] = json.loads(report.data)
                aligned["corrections"] = evaluation.count_corrections(aligned["report"])
            trial["aligned"] = aligned
            trial["trial_wall_s"] = time.monotonic() - t0
            trial["trial_sim_s"] = (node.get_clock().now().nanoseconds - sim0) * 1e-9
            trial["message_counts"] = node.take_counts()
            record["trials"].append(trial)
            print(json.dumps(trial, default=str), flush=True)
            with open(args.out, "w", encoding="utf-8") as f:
                json.dump(record, f, indent=1, default=str)
    record["command_publishers"] = node.publishers()
    pubs_ok, pubs_why = command_publishers_ok(record["command_publishers"])
    passed, why = evaluate(record["trials"], args.trials)
    aligned_ok = all(t["aligned"]["nav2"] == "succeeded"
                     and (t["aligned"].get("report") or {}).get("outcome") == "aligned" for t in record["trials"])
    if not aligned_ok:
        why.append("an alignment did not end aligned")
    record["passed"] = ready and passed and pubs_ok and aligned_ok
    record["reasons"] = ([] if ready else ["stack not ready"]) + why + pubs_why
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=1, default=str)
    text = summary(record["trials"]) + f"\n\ncommand publishers: {record['command_publishers']}\n" \
        f"result: {'PASS' if record['passed'] else 'FAIL'} {record['reasons']}\n"
    with open(args.markdown, "w", encoding="utf-8") as f:
        f.write(text)
    print(text, flush=True)
    code = 0 if record["passed"] else 1
    # Exit without interpreter teardown: tearing down rclpy with the executor thread still spinning ended the
    # process with status 245 after "result: PASS" (run 44168522141, rtf1). The result files are written above.
    sys.stdout.flush()
    os._exit(code)


if __name__ == "__main__":
    sys.exit(main())
