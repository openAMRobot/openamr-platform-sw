# SPDX-License-Identifier: MIT
"""clearance_trials: drive through the clearance arena past obstacles that are not in the map, measure from ground truth.

Each trial spawns one scenario's obstacles (scene.scenarios), sends one NavigateToPose goal with the default tree
through the unchanged command chain, and records from the Gazebo ground truth: the smallest distance between the
chassis collision box and every obstacle, contacts, path, time, speeds, standstill time, and from Nav2 the result,
the recovery nodes that ran and what collision_monitor did. The runner publishes no velocity command.

Exit status 0 when no trial had a contact and every trial ended as its scenario expects. The 0.10 m clearance target
is reported per trial, not enforced here. Simulation evidence only: not robot, site, physical or safety evidence.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger

from openamrobot_clearance_trials import geometry, scene
from openamrobot_clearance_trials.trials_constants import FOOTPRINT

CLEARANCE_TARGET = 0.10
CONTACT = 0.001
RECOVERY_NODES = ("Spin", "Wait", "BackUp", "DriveOnHeading", "ClearLocalCostmap-Subtree",
                  "ClearGlobalCostmap-Subtree", "ClearLocalCostmap-Context", "ClearGlobalCostmap-Context",
                  "WaitBeforeClear-LocalContext", "WaitBeforeClear-GlobalContext")
CM_ACTIONS = {0: "none", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}


class Runner(Node):
    def __init__(self):
        super().__init__("clearance_trials",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.truth = None            # (t, x, y, yaw, v, w)
        self.amcl = None
        self.cmd_out = (0.0, 0.0)
        self.cmd_in = (0.0, 0.0)
        self.cm = "none"
        self.rec = None              # the running trial's record, or None
        self.obstacles = {}          # name -> shape, the spawned ones
        self.walls = scene.shapes(scene.mapped())
        self.create_subscription(Odometry, "/clearance/ground_truth/odom", self._truth, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 10)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self._cmd("cmd_out", m), 20)
        self.create_subscription(Twist, "/cmd_vel_smoothed", lambda m: self._cmd("cmd_in", m), 20)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._cm, 10)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self._bt, 50)
        # Field of view of the scans as the stack receives them: per 10 degree sector of bearing from the LiDAR
        # (0 = robot forward, lidar_joint yaw pi), the share of beams with a valid range and the smallest valid range.
        self.fov = {"/scan": {}, "/scan_filtered": {}}
        for topic in self.fov:
            self.create_subscription(LaserScan, topic, lambda m, k=topic: self._scan(k, m), 5)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _cmd(self, key, msg):
        with self.lock:
            setattr(self, key, (msg.linear.x, msg.angular.z))

    def _cm(self, msg):
        with self.lock:
            self.cm = CM_ACTIONS.get(msg.action_type, str(msg.action_type))
            if self.rec is not None:
                self.rec["cm_events"].append((round(self.truth[0], 2) if self.truth else None, self.cm,
                                              msg.polygon_name))

    def _bt(self, msg):
        with self.lock:
            if self.rec is None:
                return
            for e in msg.event_log:
                if e.node_name in RECOVERY_NODES and e.current_status == "RUNNING" and e.previous_status == "IDLE":
                    self.rec["recoveries"][e.node_name] = self.rec["recoveries"].get(e.node_name, 0) + 1

    def _scan(self, key, msg):
        table = self.fov[key]
        table["scans"] = table.get("scans", 0) + 1
        table["range_min"], table["range_max"] = msg.range_min, msg.range_max
        if table["scans"] % 5:
            return
        sectors = table.setdefault("sectors", {})
        for i, r in enumerate(msg.ranges):
            bearing = math.degrees(geometry.wrap(msg.angle_min + i * msg.angle_increment + math.pi))
            cell = sectors.setdefault(int(math.floor(bearing / 10.0)) * 10, [0, 0, math.inf])
            cell[0] += 1
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                cell[1] += 1
                cell[2] = min(cell[2], r)

    def fov_report(self):
        lines = []
        with self.lock:
            for key, table in self.fov.items():
                lines.append(f"{key}: {table.get('scans', 0)} scans, range_min {table.get('range_min')},"
                             f" range_max {table.get('range_max')}")
                for sector in sorted(table.get("sectors", {})):
                    n, valid, nearest = table["sectors"][sector]
                    near = "none" if math.isinf(nearest) else f"{nearest:.2f}"
                    lines.append(f"  bearing {sector:+4d}..{sector + 10:+4d} deg: valid {100.0 * valid / max(n, 1):5.1f} %,"
                                 f" nearest valid {near} m")
        return "\n".join(lines)

    def _amcl(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.amcl = (p.position.x, p.position.y,
                         geometry.yaw_of_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def _truth(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        pose = (p.x, p.y, geometry.yaw_of_quaternion(q.x, q.y, q.z, q.w))
        speed = math.hypot(v.linear.x, v.linear.y)
        with self.lock:
            previous = self.truth
            self.truth = (t, pose[0], pose[1], pose[2], speed, v.angular.z)
            rec = self.rec
            if rec is None:
                return
            chassis = geometry.place(geometry.CHASSIS, pose)
            footprint = geometry.place(FOOTPRINT, pose)
            sample = [round(t, 2), round(pose[0], 3), round(pose[1], 3), round(pose[2], 3), round(speed, 3),
                      round(v.angular.z, 3), None, round(self.cmd_in[0], 3), round(self.cmd_out[0], 3), self.cm]
            for name, shape in self.obstacles.items():
                d = geometry.shape_distance(chassis, shape)
                if sample[6] is None or d < sample[6]:
                    sample[6] = round(d, 4)
                if d < rec["min_clear"]:
                    rec["min_clear"], rec["min_at"] = d, {"t": t, "pose": pose, "v": speed, "w": v.angular.z,
                                                          "obstacle": name, "cm": self.cm}
                rec["min_clear_footprint"] = min(rec["min_clear_footprint"], geometry.shape_distance(footprint, shape))
                if d < CONTACT:
                    rec["contact_samples"] += 1
                    rec["contact_with"].add(name)
            for name, shape in self.walls.items():
                d = geometry.shape_distance(chassis, shape)
                if d < rec["min_clear_mapped"]:
                    rec["min_clear_mapped"], rec["min_mapped_with"] = d, name
                if d < CONTACT:
                    rec["contact_samples"] += 1
                    rec["contact_with"].add(name)
            if previous is not None and rec["t0"] is not None:
                dt = t - previous[0]
                if 0.0 < dt < 1.0:
                    rec["path"] += math.hypot(pose[0] - previous[1], pose[1] - previous[2])
                    rec["turned"] += abs(geometry.wrap(pose[2] - previous[3]))
                    rec["v_max"] = max(rec["v_max"], speed)
                    if speed < 0.01 and abs(v.angular.z) < 0.02:
                        rec["still"] += dt
                        rec["still_run"] += dt
                    else:
                        rec["still_run"] = 0.0
                    rec["still_run_max"] = max(rec["still_run_max"], rec["still_run"])
                    if self.cm != "none":
                        rec["cm_time"][self.cm] = rec["cm_time"].get(self.cm, 0.0) + dt
                    if self.cmd_in[0] > 0.02 and self.cmd_out[0] < 0.5 * self.cmd_in[0]:
                        rec["cm_cut_s"] += dt
            if len(rec["series"]) == 0 or t - rec["series"][-1][0] >= 0.2:
                rec["series"].append(sample)

    # --- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, seconds):
        end = self.now() + seconds
        wall = time.monotonic() + seconds * 30 + 10
        while self.now() < end and time.monotonic() < wall:
            time.sleep(0.05)

    def wait_for(self, what, condition, wall_s):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.2)
        self.get_logger().error(f"timed out waiting for {what}")
        return False

    def pose(self):
        with self.lock:
            return None if self.truth is None else self.truth[1:4]

    def release_software_stop(self, present_wall_s=30.0, release_wall_s=120.0):
        """Operator release of the software Stop gate (openamrobot_nav2 safety_stop_gate_node) before the first goal.
        Only the gate's public service is called. A functional stop of the test stack, not the hardwired E-stop."""
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

    def spawn(self, item):
        name, kind, data, height = item
        x, y, yaw = scene.model_pose(kind, data)
        cmd = ["ros2", "run", "ros_gz_sim", "create", "-world", scene.WORLD_NAME, "-string",
               scene.model_sdf(name, kind, data, height), "-name", name, "-x", str(x), "-y", str(y), "-z", "0",
               "-Y", str(yaw)]
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        ok = done.returncode == 0
        if not ok:
            self.get_logger().error(f"spawn {name} failed: {done.stdout[-300:]} {done.stderr[-300:]}")
        with self.lock:
            self.obstacles[name] = scene.shapes([item])[name]
        return ok

    def remove(self, name):
        cmd = ["gz", "service", "-s", f"/world/{scene.WORLD_NAME}/remove", "--reqtype", "gz.msgs.Entity",
               "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", f'name: "{name}" type: MODEL']
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        ok = done.returncode == 0 and "true" in done.stdout
        if not ok:
            self.get_logger().error(f"remove {name} failed: {done.stdout[-200:]} {done.stderr[-200:]}")
        with self.lock:
            self.obstacles.pop(name, None)
        return ok

    def send(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = geometry.quaternion_of_yaw(pose[2])
        handle = None
        for _ in range(60):
            future = self.nav.send_goal_async(goal)
            while not future.done():
                time.sleep(0.05)
            handle = future.result()
            if handle.accepted:
                return handle
            time.sleep(5.0)
        return None

    def drive(self, pose, sim_timeout, wall_timeout, on_tick=None):
        """-> (status, sim seconds)."""
        handle = self.send(pose)
        if handle is None:
            return "rejected", 0.0
        t0, wall0 = self.now(), time.monotonic()
        result = handle.get_result_async()
        while not result.done():
            if self.now() - t0 > sim_timeout or time.monotonic() - wall0 > wall_timeout:
                cancel = handle.cancel_goal_async()
                end = time.monotonic() + 20.0
                while not cancel.done() and time.monotonic() < end:
                    time.sleep(0.05)
                return "timeout", self.now() - t0
            if on_tick is not None:
                on_tick(self.now() - t0)
            time.sleep(0.1)
        status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(result.result().status, str(result.result().status))
        return status, self.now() - t0

    def trial(self, name, spec, target, direction, sim_timeout, wall_timeout):
        rec = {"scenario": name, "direction": direction, "expect": spec["expect"], "note": spec["note"],
               "min_clear": math.inf, "min_at": None, "min_clear_footprint": math.inf, "min_clear_mapped": math.inf,
               "min_mapped_with": None, "contact_samples": 0, "contact_with": set(), "path": 0.0, "turned": 0.0,
               "v_max": 0.0, "still": 0.0, "still_run": 0.0, "still_run_max": 0.0, "cm_time": {}, "cm_cut_s": 0.0,
               "cm_events": [], "recoveries": {}, "series": [], "t0": None, "removed": None, "spawn_ok": True}
        for item in spec["obstacles"]:
            rec["spawn_ok"] = self.spawn(item) and rec["spawn_ok"]
        self.sleep_sim(3.0)
        rec["start"] = self.pose()
        with self.lock:
            rec["t0"] = self.truth[0]
            self.rec = rec
        state = {"removed": False}

        def on_tick(elapsed):
            delay = spec.get("remove_after_stop_s")
            if delay is None or state["removed"]:
                return
            with self.lock:
                moved, still = rec["path"], rec["still_run"]
            if moved > 1.0 and still >= delay:
                self.remove("cart")
                state["removed"] = True
                with self.lock:
                    rec["removed"] = {"elapsed_s": elapsed, "path_m": moved}

        status, sim_s = self.drive(target, sim_timeout, wall_timeout, on_tick)
        self.sleep_sim(1.0)
        with self.lock:
            self.rec = None
            end = self.truth
        rec.update(status=status, sim_s=sim_s, end=end[1:4], end_speed=end[4],
                   goal_error=math.hypot(end[1] - target[0], end[2] - target[1]),
                   amcl_error=None if self.amcl is None else math.hypot(self.amcl[0] - end[1], self.amcl[1] - end[2]))
        if rec["removed"] is not None:
            rec["recovery_s"] = sim_s - rec["removed"]["elapsed_s"] if status == "succeeded" else None
        for item in list(spec["obstacles"]):
            if item[0] in self.obstacles:
                self.remove(item[0])
        rec["contact"] = rec["contact_samples"] > 0
        rec["contact_with"] = sorted(rec["contact_with"])
        for key in ("min_clear", "min_clear_footprint", "min_clear_mapped"):
            rec[key] = None if math.isinf(rec[key]) else rec[key]
        rec["mean_speed"] = rec["path"] / sim_s if sim_s > 0 else None
        if spec["expect"] == "pass":
            rec["as_expected"] = status == "succeeded" and not rec["contact"]
        elif spec["expect"] == "blocked":
            rec["as_expected"] = status in ("aborted", "timeout") and not rec["contact"] and rec["end_speed"] < 0.01
        else:
            rec["as_expected"] = not rec["contact"]
        rec["below_target"] = rec["min_clear"] is not None and rec["min_clear"] < CLEARANCE_TARGET
        series = rec.pop("series")
        if rec["min_at"] is not None:
            tm = rec["min_at"]["t"]
            rec["closest_window"] = [s for s in series if tm - 4.0 <= s[0] <= tm + 4.0][::2]
        rec.pop("t0"), rec.pop("still_run")
        return rec, series


def summary(trials):
    rows = ["| # | scenario | dir | expect | Nav2 | as expected | min clearance, chassis (m) | at speed (m/s) | contact |"
            " min to mapped walls (m) | time (s) | path (m) | still (s) | recoveries | collision_monitor (s) |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, t in enumerate(trials):
        clear = "n/a" if t["min_clear"] is None else f"{t['min_clear']:.3f}"
        speed = "n/a" if t["min_at"] is None else f"{t['min_at']['v']:.2f}"
        rec = ", ".join(f"{k} {v}" for k, v in sorted(t["recoveries"].items())) or "none"
        cm = ", ".join(f"{k} {v:.1f}" for k, v in sorted(t["cm_time"].items())) or "none"
        rows.append(f"| {i + 1} | {t['scenario']} | {t['direction']} | {t['expect']} | {t['status']} |"
                    f" {'yes' if t['as_expected'] else 'NO'} | {clear} | {speed} | {'YES' if t['contact'] else 'no'} |"
                    f" {t['min_clear_mapped']:.3f} | {t['sim_s']:.0f} | {t['path']:.1f} | {t['still']:.0f} | {rec} |"
                    f" {cm} |")
    return "\n".join(rows)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", default="free,open_box,gap100,gap080,gap066,gap050,aisle_blocked")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--aisle-width", type=float, default=scene.AISLE_WIDTH)
    parser.add_argument("--out", default="clearance_trials.json")
    parser.add_argument("--markdown", default="clearance_trials.md")
    parser.add_argument("--label", default="")
    parser.add_argument("--startup-timeout", type=float, default=900.0)
    parser.add_argument("--sim-timeout", type=float, default=420.0)
    parser.add_argument("--wall-timeout", type=float, default=2400.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Runner()
    node.walls = scene.shapes(scene.mapped(args.aisle_width))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    specs = scene.scenarios(args.aisle_width)
    names = [n for n in args.scenarios.split(",") if n]
    unknown = [n for n in names if n not in specs]
    if unknown:
        print(f"unknown scenarios {unknown}; known: {sorted(specs)}")
        os._exit(2)
    record = {"label": args.label, "aisle_width": args.aisle_width, "clearance_target": CLEARANCE_TARGET, "trials": []}
    ready = (node.wait_for("ground truth", lambda: node.pose() is not None, args.startup_timeout)
             and node.wait_for("navigate_to_pose", lambda: node.nav.wait_for_server(timeout_sec=1.0),
                               args.startup_timeout)
             and node.wait_for("AMCL pose", lambda: node.amcl is not None, 300.0))
    if ready:
        record["software_stop"] = node.release_software_stop()
        print(json.dumps({"software_stop": record["software_stop"]}), flush=True)
        ready = not record["software_stop"]["gate_present"] or record["software_stop"]["released"]
    record["ready"] = ready
    east, west = scene.GOAL, (scene.START[0], scene.START[1], math.pi)
    if ready:
        for repeat in range(args.repeats):
            for name in names:
                here = node.pose()
                direction = "east" if abs(here[0] - scene.START[0]) < abs(here[0] - scene.GOAL[0]) else "west"
                target = east if direction == "east" else west
                rec, series = node.trial(name, specs[name], target, direction, args.sim_timeout, args.wall_timeout)
                rec["repeat"] = repeat + 1
                record["trials"].append(rec)
                print("TRIAL " + json.dumps(rec, default=str), flush=True)
                with open(args.out, "w", encoding="utf-8") as f:
                    json.dump(record, f, indent=1, default=str)
                with open(f"{args.out}.series.{len(record['trials'])}.json", "w", encoding="utf-8") as f:
                    json.dump(series, f)
                if rec["status"] != "succeeded":
                    # Clean-up leg with the obstacles removed, so that the next trial starts from an end of the arena.
                    node.sleep_sim(5.0)
                    status, sim_s = node.drive(target, args.sim_timeout, args.wall_timeout)
                    print(json.dumps({"cleanup_leg": status, "sim_s": sim_s, "pose": node.pose()}), flush=True)
                    if status != "succeeded":
                        record["aborted_series"] = f"clean-up leg after {name} ended {status}"
                        break
            if "aborted_series" in record:
                break
    trials = record["trials"]
    planned = len(names) * args.repeats
    reasons = []
    if not ready:
        reasons.append("stack not ready")
    if len(trials) < planned:
        reasons.append(f"{len(trials)} of {planned} trials ran")
    reasons += [f"trial {i + 1} {t['scenario']}: contact with {t['contact_with']}" for i, t in enumerate(trials)
                if t["contact"]]
    reasons += [f"trial {i + 1} {t['scenario']}: expected {t['expect']}, Nav2 {t['status']}"
                for i, t in enumerate(trials) if not t["as_expected"] and not t["contact"]]
    record["passed"] = not reasons
    record["reasons"] = reasons
    clear = [t["min_clear"] for t in trials if t["min_clear"] is not None]
    record["min_clearance_all"] = min(clear) if clear else None
    record["below_target"] = [f"{t['scenario']} {t['direction']} {t['min_clear']:.3f}" for t in trials
                              if t["below_target"]]
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=1, default=str)
    text = (f"label: {args.label}\n\n" + summary(trials) + "\n\n"
            f"smallest chassis clearance to an unmapped obstacle: {record['min_clearance_all']}\n"
            f"below the {CLEARANCE_TARGET} m target: {record['below_target']}\n"
            f"result: {'PASS' if record['passed'] else 'FAIL'} {reasons}\n")
    with open(args.markdown, "w", encoding="utf-8") as f:
        f.write(text)
    print(text, flush=True)
    print("SCAN FIELD OF VIEW (bearing from the LiDAR, 0 = robot forward)\n" + node.fov_report(), flush=True)
    sys.stdout.flush()
    os._exit(0 if record["passed"] else 1)


if __name__ == "__main__":
    sys.exit(main())
