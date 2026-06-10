#!/usr/bin/env python3
"""
Bundle-driven, camera-centric docking trigger.

The dock carries a 3-tag AprilTag bundle (family 36h11, IDs 0/1/2 — outer tags
at y = ±0.45 m, centre tag at y = 0). The 90 cm baseline between the outer
tags gives a stable surface normal that is independent of single-tag yaw
jitter.

Sequence on /dock_trigger=true:
  1. NavigateToPose → staging zone (Nav2/RPP)
  2. Centring scan: rotate in place until the bundle midpoint is centred in
     the camera image (image-frame P-controller)
  3. Estimate the dock surface normal from the outer tags' wide baseline
     (proximity-weighted EMA); back off if the robot arrived too close
  4. Pure-pursuit the normal axis in the camera/tag frame (independent of
     map drift and wheel slip), with a re-verification step against
     normal_tolerance_deg
  5. Two-regime final approach:
       FAR  (camera→centre-tag depth > freeze_axis_distance): EMA-average
            the live axis and pure-pursuit it
       NEAR (depth ≤ freeze_axis_distance): freeze the averaged axis and
            finish on a visual corrector on the centre tag
       — stops when camera→centre-tag depth ≤ docking_distance.

Also supports:
  - /undock_robot → reverse undock_reverse_distance, then spin 180°
  - /goal_pose gate → if a navigation goal arrives while docked, the robot
    undocks first, then republishes the goal on goal_pose_forward_topic
  - obstacle guard on /scan during forward drive and undock reverse
    (skipped during the final IBVS approach to the dock itself)
  - /dock_trigger_status (idle | docking | docked | undocking | failed),
    with a 2 s heartbeat for UI sync

See docs/14_docking_research.md for the full design rationale and
docs/13_perception_and_line.md for the perception pipeline.
"""

import math
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import (
    NavigateToPose,
    UndockRobot,
)
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_rotate_z(qx, qy, qz, qw):
    """Apply quaternion rotation to the unit vector (0, 0, 1).

    Returns the tag's local +Z axis expressed in the parent (map) frame.
    For an apriltag, this is approximately the tag's normal.
    """
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    nz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return nx, ny, nz


class DockTrigger(Node):
    def __init__(self):
        super().__init__('dock_trigger')

        # ── Params ──────────────────────────────────────────────────────────
        self.declare_parameter('trigger_topic', 'dock_trigger')
        self.declare_parameter('undock_on_false', False)
        self.declare_parameter('dock_type', 'openamrobot_dock')

        # ── Undock ───────────────────────────────────────────────────────────
        # Publish std_msgs/Bool true on undock_trigger_topic to run the undock
        # maneuver: reverse undock_reverse_distance metres in a straight line,
        # then spin 180° in place. The robot ends up facing away from the dock,
        # free to navigate.
        #
        # goal_pose_topic / goal_pose_forward_topic implement the
        # "undock-before-navigate" gate. Nav2's bt_navigator is remapped (in
        # the nav2 launch) to listen on goal_pose_forward_topic instead of
        # goal_pose, so this node owns goal_pose: when a navigation goal
        # arrives while docked it undocks first, then republishes the goal on
        # goal_pose_forward_topic for Nav2 to act on. When not docked the goal
        # is forwarded immediately (pass-through).
        self.declare_parameter('undock_trigger_topic', 'undock_robot')
        self.declare_parameter('goal_pose_topic', 'goal_pose')
        self.declare_parameter('goal_pose_forward_topic', 'goal_pose_nav')
        self.declare_parameter('undock_reverse_distance', 1.5)   # m straight back
        self.declare_parameter('undock_reverse_speed', 0.10)     # m/s (magnitude)

        # Dock pose in map frame (must match nav2_sim_full.yaml docks/home_dock)
        self.declare_parameter('dock_pose_x', 0.0)
        self.declare_parameter('dock_pose_y', 4.9)
        self.declare_parameter('dock_pose_yaw', 1.5707)

        # Staging
        self.declare_parameter('staging_distance', 1.5)        # m in front of dock
        self.declare_parameter('staging_hold_seconds', 1.0)    # robot stationary

        # Docking
        self.declare_parameter('docking_distance', 0.6)        # final distance from tag (m)
        self.declare_parameter('drive_speed', 0.10)            # m/s forward
        self.declare_parameter('drive_yaw_kp', 1.5)            # angular P gain during drive
        self.declare_parameter('drive_yaw_max_omega', 0.5)     # rad/s clamp on correction

        # Line-tracking advance — pure-pursuit-style controller. Each iteration
        # we compute a desired heading
        #
        #     desired_yaw = perp_yaw − atan2(lateral, line_lookahead_distance)
        #
        # so the robot aims at a point on the perpendicular line that is
        # line_lookahead_distance metres ahead of its current foot-of-
        # perpendicular. The atan is bounded by ±π/2, so this also naturally
        # bounds the heading deviation. The control law then drives the
        # robot to this heading with a yaw P-loop:
        #
        #     omega = line_yaw_kp × yaw_err, saturated by drive_yaw_max_omega.
        #
        # As the robot approaches the line, lateral → 0 and desired_yaw →
        # perp_yaw, so the robot arrives perpendicular to the tag.
        #
        # Tuning intuition:
        #   - smaller line_lookahead_distance = more aggressive lateral
        #     convergence (steeper desired heading at the same offset);
        #   - line_yaw_kp controls how fast the robot tracks the desired
        #     heading;
        #   - to dampen oscillations, increase line_lookahead_distance or
        #     decrease line_yaw_kp.
        self.declare_parameter('line_yaw_kp', 2.5)
        self.declare_parameter('line_lookahead_distance', 0.4)

        # Image-frame visual servo (camera-frame closed-loop on the centre
        # tag) used by the Phase 5 NEAR regime once the axis is frozen.
        #
        #     omega = -visual_servo_kp · atan2(X_optical, Z_optical)
        #
        # Map-frame solvePnP carries a systematic bias in the near field
        # (corners hugging the bottom of the FOV), but the image-frame
        # angle is self-consistent — keeping the tag centred in the
        # image = aiming straight at the dock. The low-pass filter
        # alpha rejects single-frame solvePnP spikes (alpha = 0.2 →
        # time constant ≈ 5 frames; alpha = 1.0 disables filtering).
        self.declare_parameter('visual_servo_kp', 1.0)            # rad/s per rad
        self.declare_parameter('visual_servo_filter_alpha', 0.2)

        # Initial tag-search scan. After Nav2 reaches the staging zone the
        # tag may not be in the camera frame (Nav2 goal yaw tolerance plus
        # parking precision). We open-loop rotate at scan_rotation_speed
        # until the tag is in view, then close the loop on the camera-frame
        # angle to centre it. Scan ends when the tag has been within
        # scan_centring_tolerance of image centre for scan_consecutive_target
        # consecutive frames.
        self.declare_parameter('scan_rotation_speed', 0.3)     # rad/s
        self.declare_parameter('scan_consecutive_target', 5)   # centred frames in a row
        self.declare_parameter('scan_centring_tolerance', 0.035)  # rad ≈ 2°
        self.declare_parameter('scan_centring_kp', 1.0)        # rad/s per rad of image angle
        self.declare_parameter('drive_rate_hz', 20.0)          # control loop frequency
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        # Spin (manual closed-loop, no nav2_behaviors costmap check)
        self.declare_parameter('spin_kp', 2.0)
        self.declare_parameter('spin_max_omega', 0.6)
        self.declare_parameter('spin_yaw_tolerance', 0.02)     # ~1.1°

        # Tag detection
        self.declare_parameter('detection_topic', '/detected_dock_pose')
        self.declare_parameter('detection_max_age', 1.5)       # s — drop stale msgs

        # Temporal filtering of tag pose: collect N samples, average them
        self.declare_parameter('filter_num_samples', 20)
        self.declare_parameter('filter_max_collect_time', 1.5) # s

        # Debug visualisation: publish the perpendicular approach line and the
        # running-average tag centre as RViz markers (visualization_msgs/
        # MarkerArray) on debug_marker_topic. See docs/13_perception_and_line.md.
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('debug_marker_topic', '/docking/debug_markers')

        # Gazebo marker mirror: draw the same line + tag centre INSIDE the
        # Gazebo GUI. Gazebo does not consume ROS markers, so we push a
        # gz.msgs.Marker_V to the gz transport service via the `gz` CLI (no
        # Python gz bindings on this system). The call is throttled and run in
        # a daemon thread so it never stalls the control loop. Map ≡ world in
        # this sim, so map coords are world coords. See docs/13.
        self.declare_parameter('publish_gz_marker', True)
        self.declare_parameter('gz_marker_service', '/marker_array')
        self.declare_parameter('gz_marker_period', 0.4)   # s — throttle (subprocess is heavy)

        # ── Camera-centric approach (two-sided normal estimation) ───────────
        # See docs/13_perception_and_line.md §6. The robot estimates the tag
        # normal from two sides (cancels solvePnP view-angle bias), goes to a
        # point on the normal, re-verifies, then does a camera-only final
        # approach (immune to wheel slip).
        self.declare_parameter('too_close_distance', 1.0)        # m — back off if closer
        self.declare_parameter('predock_distance', 1.5)          # m on the normal (P1)
        self.declare_parameter('refined_predock_distance', 1.30) # m on the normal (P2)
        self.declare_parameter('normal_tolerance_deg', 5.0)      # deg — N vs N' agreement
        self.declare_parameter('obs_lateral', 0.5)               # m — side offset for obs B
        self.declare_parameter('obs_distance', 2.0)              # m from tag for obs B
        # Phase 5 two regimes: FAR (> freeze_axis_distance) the robot averages
        # the dock axis from the 3 tags and pure-pursuits it; NEAR (≤ this) the
        # axis is frozen (no more averaging — close-range estimates are noisy)
        # and the robot finishes on the centre-tag visual corrector. Camera→tag
        # depth.
        self.declare_parameter('freeze_axis_distance', 0.70)     # m camera→tag
        # EMA weight for the live axis estimate (Phase 5 FAR). A cumulative mean
        # would freeze the early (off-axis, wrong) estimates; an EMA keeps
        # following the recent, better ones as the robot centres up. Higher =
        # more reactive (noisier), lower = smoother (slower).
        self.declare_parameter('axis_filter_alpha', 0.15)

        # ── Obstacle avoidance during drive phases ─────────────────────────
        # The sequencer publishes cmd_vel straight to the robot, bypassing
        # Nav2's collision_monitor. We re-add a simple obstacle guard inside
        # the forward-drive phase (pure-pursuit onto the dock normal) and
        # the undock reverse: before and during each of those phases, we
        # check the LIDAR in a forward or backward cone. If the closest
        # return inside the cone is closer than the configured threshold,
        # the robot stops and waits for the path to clear. After
        # `obstacle_wait_timeout` seconds it aborts the phase.
        #
        # Phase 5 (IBVS final approach to the dock) deliberately skips this
        # check — the dock itself is "an obstacle" we are approaching on
        # purpose. The centring scan / spin-in-place phases also skip it
        # (the robot is rotating in place, not translating into anything).
        self.declare_parameter('obstacle_check_enabled', True)
        self.declare_parameter('obstacle_scan_topic', '/scan_filtered')
        self.declare_parameter('obstacle_forward_distance', 0.6)        # m — stop if obstacle within this distance ahead
        self.declare_parameter('obstacle_backward_distance', 0.6)       # m — stop if obstacle within this distance behind
        self.declare_parameter('obstacle_arc_half_width_deg', 30.0)     # deg — half-width of the detection cone
        self.declare_parameter('obstacle_wait_timeout', 10.0)           # s — max wait before aborting
        self.declare_parameter('obstacle_check_period', 0.2)            # s — poll period while waiting
        # Range floor — LIDAR returns shorter than this are IGNORED.
        # Default 0.0 (disabled): self-reflections from the robot's own body
        # are removed UPSTREAM by Nav2's scan_body_filter (angle-based), and
        # we subscribe to /scan_filtered. A >0 floor would silently mask
        # real obstacles closer than the floor inside the kept angular
        # sector — angular filtering is the correct primitive.
        # Set >0 ONLY if /scan_filtered isn't available on your stack and
        # you must fall back to /scan — and then pick a value clearly
        # below the closest legitimate obstacle distance.
        self.declare_parameter('obstacle_min_range', 0.0)               # m — 0 = disabled

        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.undock_on_false = self.get_parameter('undock_on_false').value
        self.dock_type = self.get_parameter('dock_type').value
        self.undock_trigger_topic = self.get_parameter('undock_trigger_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.goal_pose_forward_topic = self.get_parameter('goal_pose_forward_topic').value
        self.undock_reverse_distance = float(self.get_parameter('undock_reverse_distance').value)
        self.undock_reverse_speed = float(self.get_parameter('undock_reverse_speed').value)
        self.dock_x = float(self.get_parameter('dock_pose_x').value)
        self.dock_y = float(self.get_parameter('dock_pose_y').value)
        self.dock_yaw = float(self.get_parameter('dock_pose_yaw').value)
        self.staging_distance = float(self.get_parameter('staging_distance').value)
        self.staging_hold_seconds = float(self.get_parameter('staging_hold_seconds').value)
        self.docking_distance = float(self.get_parameter('docking_distance').value)
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.drive_yaw_kp = float(self.get_parameter('drive_yaw_kp').value)
        self.drive_yaw_max_omega = float(self.get_parameter('drive_yaw_max_omega').value)
        self.line_yaw_kp = float(self.get_parameter('line_yaw_kp').value)
        self.line_lookahead_distance = float(self.get_parameter('line_lookahead_distance').value)
        self.visual_servo_kp = float(self.get_parameter('visual_servo_kp').value)
        self.visual_servo_filter_alpha = float(self.get_parameter('visual_servo_filter_alpha').value)
        self.scan_rotation_speed = float(self.get_parameter('scan_rotation_speed').value)
        self.scan_consecutive_target = int(self.get_parameter('scan_consecutive_target').value)
        self.scan_centring_tolerance = float(self.get_parameter('scan_centring_tolerance').value)
        self.scan_centring_kp = float(self.get_parameter('scan_centring_kp').value)
        self.drive_rate_hz = float(self.get_parameter('drive_rate_hz').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.spin_kp = float(self.get_parameter('spin_kp').value)
        self.spin_max_omega = float(self.get_parameter('spin_max_omega').value)
        self.spin_yaw_tolerance = float(self.get_parameter('spin_yaw_tolerance').value)
        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_max_age = float(self.get_parameter('detection_max_age').value)
        self.filter_num_samples = int(self.get_parameter('filter_num_samples').value)
        self.filter_max_collect_time = float(self.get_parameter('filter_max_collect_time').value)
        self.publish_debug_markers = bool(self.get_parameter('publish_debug_markers').value)
        self.debug_marker_topic = self.get_parameter('debug_marker_topic').value
        self.publish_gz_marker = bool(self.get_parameter('publish_gz_marker').value)
        self.gz_marker_service = self.get_parameter('gz_marker_service').value
        self.gz_marker_period = float(self.get_parameter('gz_marker_period').value)
        self._last_gz_marker_t = 0.0
        self._gz_marker_inflight = False
        self.too_close_distance = float(self.get_parameter('too_close_distance').value)
        self.predock_distance = float(self.get_parameter('predock_distance').value)
        self.refined_predock_distance = float(self.get_parameter('refined_predock_distance').value)
        self.normal_tolerance = math.radians(
            float(self.get_parameter('normal_tolerance_deg').value))
        self.obs_lateral = float(self.get_parameter('obs_lateral').value)
        self.obs_distance = float(self.get_parameter('obs_distance').value)
        self.freeze_axis_distance = float(self.get_parameter('freeze_axis_distance').value)
        self.axis_filter_alpha = float(self.get_parameter('axis_filter_alpha').value)
        self.obstacle_check_enabled = bool(self.get_parameter('obstacle_check_enabled').value)
        self.obstacle_scan_topic = self.get_parameter('obstacle_scan_topic').value
        self.obstacle_forward_distance = float(self.get_parameter('obstacle_forward_distance').value)
        self.obstacle_backward_distance = float(self.get_parameter('obstacle_backward_distance').value)
        self.obstacle_arc_half_width = math.radians(
            float(self.get_parameter('obstacle_arc_half_width_deg').value))
        self.obstacle_wait_timeout = float(self.get_parameter('obstacle_wait_timeout').value)
        self.obstacle_check_period = float(self.get_parameter('obstacle_check_period').value)
        self.obstacle_min_range = float(self.get_parameter('obstacle_min_range').value)

        # ── Multi-threaded callback group so the long-running sequence can
        #    run while subscriptions and TF still get processed. ────────────
        self.cb_group = ReentrantCallbackGroup()

        # ── Action clients ──────────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=self.cb_group)
        self.undock_client = ActionClient(self, UndockRobot, 'undock_robot',
                                          callback_group=self.cb_group)

        # ── cmd_vel publisher (closed-loop drive phase) ────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ── TF listener ─────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Tag detection subscription ──────────────────────────────────────
        self.detected_pose = None
        self.create_subscription(
            PoseStamped, self.detection_topic, self.on_detection, 10,
            callback_group=self.cb_group,
        )

        # ── Laser scan subscription (obstacle avoidance) ────────────────────
        self.latest_scan = None
        if self.obstacle_check_enabled:
            self.create_subscription(
                LaserScan, self.obstacle_scan_topic, self.on_scan, 10,
                callback_group=self.cb_group,
            )

        # ── State ───────────────────────────────────────────────────────────
        # busy: a long-running maneuver (dock or undock) is in progress; new
        #       triggers are ignored until it finishes.
        # is_docked: the robot completed a docking sequence and has not yet
        #            undocked. Gates the undock-before-navigate behaviour.
        self.busy = False
        self.is_docked = False

        # ── Goal-pose gate publisher (forwards to Nav2 after undock) ────────
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, self.goal_pose_forward_topic, 10)

        # ── Debug marker publisher (perpendicular line + tag centre) ────────
        self.marker_pub = self.create_publisher(
            MarkerArray, self.debug_marker_topic, 10)

        # ── Docking status publisher (consumed by the web UI, from upstream) ─
        # Publishes one of: idle | docking | docked | undocking | failed
        # Kept minimal here — the bundle sequencer doesn't yet emit every
        # transient state; the 2 s heartbeat reports docked/idle and is
        # enough to keep the UI in sync. Wire fine-grained transitions in a
        # follow-up.
        self.dock_status_pub = self.create_publisher(String, 'dock_trigger_status', 10)
        self.create_timer(2.0, self._heartbeat_status, callback_group=self.cb_group)

        # ── Triggers ────────────────────────────────────────────────────────
        self.create_subscription(
            Bool, self.trigger_topic, self.on_trigger, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Bool, self.undock_trigger_topic, self.on_undock, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PoseStamped, self.goal_pose_topic, self.on_goal_pose, 10,
            callback_group=self.cb_group,
        )
        self.get_logger().info(
            f"Dock trigger ready on '{self.trigger_topic}'. "
            f"staging={self.staging_distance}m, hold={self.staging_hold_seconds}s, "
            f"dock_dist={self.docking_distance}m"
        )
        self.get_logger().info(
            f"Undock ready on '{self.undock_trigger_topic}' "
            f"(reverse {self.undock_reverse_distance}m + spin 180°); "
            f"goal gate '{self.goal_pose_topic}' → '{self.goal_pose_forward_topic}'"
        )

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
    def _pub_status(self, state: str) -> None:
        self.dock_status_pub.publish(String(data=state))

    def _heartbeat_status(self) -> None:
        if not self.busy:
            self._pub_status('docked' if self.is_docked else 'idle')

    def on_detection(self, msg: PoseStamped):
        self.detected_pose = msg

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_trigger(self, msg: Bool):
        if msg.data:
            if self.busy:
                self.get_logger().warn('Sequence already in progress — ignoring trigger')
                return
            self.busy = True
            t = threading.Thread(target=self._run_and_release, daemon=True)
            t.start()
        elif self.undock_on_false:
            self._send_undock()

    def _run_and_release(self):
        self._pub_status('docking')
        try:
            self.run_docking_sequence()
        except Exception as e:
            self.get_logger().error(f'Docking sequence error: {e}')
        finally:
            self._pub_status('docked' if self.is_docked else 'failed')
            self.busy = False

    def on_undock(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warn('Sequence already in progress — ignoring undock')
            return
        if not self.is_docked:
            self.get_logger().warn('Not docked — running undock maneuver anyway')
        self.busy = True
        t = threading.Thread(target=self._undock_and_release, daemon=True)
        t.start()

    def _undock_and_release(self):
        self._pub_status('undocking')
        try:
            self.run_undock_sequence()
        except Exception as e:
            self.get_logger().error(f'Undock sequence error: {e}')
        finally:
            self._pub_status('idle' if not self.is_docked else 'failed')
            self.busy = False

    def on_goal_pose(self, msg: PoseStamped):
        # If not docked, the robot is free to navigate — pass the goal straight
        # through to Nav2 (which listens on goal_pose_forward_topic).
        if not self.is_docked:
            self.goal_pose_pub.publish(msg)
            return
        # Docked: undock first, then forward the goal.
        if self.busy:
            self.get_logger().warn('Sequence in progress — ignoring navigation goal')
            return
        self.busy = True
        t = threading.Thread(target=self._undock_then_forward, args=(msg,), daemon=True)
        t.start()

    def _undock_then_forward(self, goal_msg: PoseStamped):
        self._pub_status('undocking')
        try:
            self.get_logger().info('Navigation goal received while docked — undocking first')
            if self.run_undock_sequence():
                self.get_logger().info('   undock complete — forwarding goal to Nav2')
                self._pub_status('idle')
                self.goal_pose_pub.publish(goal_msg)
            else:
                self.get_logger().error('   undock failed — navigation goal NOT forwarded')
                self._pub_status('failed')
        except Exception as e:
            self.get_logger().error(f'Undock-then-navigate error: {e}')
            self._pub_status('failed')
        finally:
            self.busy = False

    # ──────────────────────────────────────────────────────────────────────
    # Main sequence
    # ──────────────────────────────────────────────────────────────────────
    def run_docking_sequence(self):
        # Camera-centric pipeline — see docs/13_perception_and_line.md §6.

        # ── Phase 1: Nav2 coarse approach to the staging zone. ────────────
        self.get_logger().info('── Phase 1: NavigateToPose → approach zone')
        if not self.navigate_to_staging():
            self.get_logger().error('NavigateToPose failed — aborting')
            return
        self._publish_cmd_vel(0.0, 0.0)
        time.sleep(self.staging_hold_seconds)

        # See both tags and estimate the dock (centre + normal from baseline).
        if not self._search_for_tag():
            self.get_logger().error('   tags not detected during scan — aborting')
            return
        est = self._estimate_dock()
        if est is None:
            self.get_logger().error('   could not estimate dock from both tags — aborting')
            return
        cx, cy, normal_yaw = est
        pose = self.lookup_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        d0 = math.hypot(cx - rx, cy - ry)
        self.get_logger().info(
            f'   dock centre ({cx:.2f}, {cy:.2f}), normal '
            f'{math.degrees(normal_yaw):.1f}°, distance {d0:.2f}m')

        # ── Phase 1.5: back off if too close. ─────────────────────────────
        if d0 < self.too_close_distance:
            self.get_logger().info(
                f'── Phase 1.5: too close ({d0:.2f}m) — backing off to '
                f'{self.predock_distance:.2f}m on the normal')
            if not self._goto_point_on_normal(cx, cy, normal_yaw,
                                              self.predock_distance):
                return
            if not self._search_for_tag():
                return
            est = self._estimate_dock()
            if est is None:
                return
            cx, cy, normal_yaw = est

        # ── Phase 3: go to the pre-dock point on the normal. ──────────────
        self.get_logger().info(
            f'── Phase 3: pre-dock point ({self.predock_distance:.2f}m on normal)')
        if not self._goto_point_on_normal(cx, cy, normal_yaw, self.predock_distance):
            return

        # ── Phase 4: re-estimate head-on and confirm/refine. ──────────────
        self.get_logger().info('── Phase 4: re-estimate dock normal')
        if not self._search_for_tag():
            return
        est = self._estimate_dock()
        if est is None:
            return
        cx2, cy2, normal_yaw2 = est
        err = abs(normalize_angle(normal_yaw2 - normal_yaw))
        self.get_logger().info(
            f'   re-estimated normal {math.degrees(normal_yaw2):.1f}° '
            f'(was {math.degrees(normal_yaw):.1f}°, diff {math.degrees(err):.1f}°)')
        cx, cy, normal_yaw = cx2, cy2, normal_yaw2
        if err > self.normal_tolerance:
            self.get_logger().info(
                f'   disagree (> {math.degrees(self.normal_tolerance):.1f}°) — '
                f'repositioning to {self.refined_predock_distance:.2f}m')
            if not self._goto_point_on_normal(cx, cy, normal_yaw,
                                              self.refined_predock_distance):
                return
        else:
            self.get_logger().info('   normals agree — confirmed')

        # ── Phase 5: final visual approach (camera-only). ─────────────────
        self.get_logger().info(
            f'── Phase 5: visual approach to {self.docking_distance:.2f}m (camera depth)')
        if not self._final_visual_approach(cx, cy, normal_yaw):
            self.get_logger().error('   final approach failed')
            return

        self.is_docked = True
        self.get_logger().info('Docking sequence complete ✓')

    # ──────────────────────────────────────────────────────────────────────
    # Camera-centric helpers (see docs/13_perception_and_line.md §6)
    # ──────────────────────────────────────────────────────────────────────
    def _estimate_dock(self, num_samples=None):
        """Average num_samples joint observations of the three tags' map
        positions, then derive the dock target and normal:

          - dock target = the CENTRE tag (id1), the point we drive onto;
          - dock normal = perpendicular to the wide baseline between the OUTER
            tags (id0→id2), disambiguated toward the robot.

        Using the outer tag *centres* and their wide baseline gives a far more
        robust dock orientation than a single-tag solvePnP normal.

        Returns (cx, cy, normal_yaw) or None.
        """
        if num_samples is None:
            num_samples = self.filter_num_samples
        s = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]   # sums for tag 0,1,2
        n = 0
        last_key = None
        deadline = time.time() + self.filter_max_collect_time
        while time.time() < deadline and n < num_samples:
            p0 = self._lookup_tag_map('charging_dock_tag_0')
            p1 = self._lookup_tag_map('charging_dock_tag_1')
            p2 = self._lookup_tag_map('charging_dock_tag_2')
            if p0 is not None and p1 is not None and p2 is not None:
                key = (round(p0[0], 4), round(p1[0], 4), round(p2[0], 4))
                if key != last_key:        # don't count an unchanged TF twice
                    for i, p in enumerate((p0, p1, p2)):
                        s[i][0] += p[0]
                        s[i][1] += p[1]
                    n += 1
                    last_key = key
            time.sleep(0.05)
        if n == 0:
            self.get_logger().error('   never saw all three tags together')
            return None
        c0 = (s[0][0] / n, s[0][1] / n)
        c1 = (s[1][0] / n, s[1][1] / n)
        c2 = (s[2][0] / n, s[2][1] / n)
        pose = self.lookup_robot_pose()
        if pose is None:
            return None
        rx, ry, _ = pose
        return self._dock_pose_from_tags(c0, c1, c2, rx, ry)

    def _dock_pose_from_tags(self, c0, c1, c2, rx, ry):
        """Dock target + heading-to-dock from the three tag centres.

        Target = midpoint of the OUTER tags (c0+c2)/2. The outer tags are big
        and well localised, so their midpoint pins the dock centre more
        precisely than the small centre tag (and equals it geometrically). The
        normal is perpendicular to the wide baseline c0→c2, disambiguated toward
        the robot; the returned yaw faces the robot toward the dock.
        """
        cx = 0.5 * (c0[0] + c2[0])          # midpoint of the precise outer tags
        cy = 0.5 * (c0[1] + c2[1])
        dx, dy = c2[0] - c0[0], c2[1] - c0[1]   # wide baseline
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return None
        nx, ny = -dy / L, dx / L            # perpendicular to the tag row
        if nx * (rx - cx) + ny * (ry - cy) < 0.0:
            nx, ny = -nx, -ny               # point toward the robot
        return cx, cy, math.atan2(-ny, -nx)

    def _goto_point_on_normal(self, tag_x, tag_y, normal_yaw, dist):
        """Drive to the point `dist` metres in front of the tag along its
        normal (robot side), then face the tag. normal_yaw is the heading from
        that point toward the tag.
        """
        px = tag_x - dist * math.cos(normal_yaw)
        py = tag_y - dist * math.sin(normal_yaw)
        self.get_logger().info(
            f'   → ({px:.2f}, {py:.2f}) then face the tag')
        # Garde-fou: verify the forward path is clear before committing to the
        # drive. This catches "someone is directly in front of the robot when
        # the trigger arrives" up front, before the drive loop starts.
        if not self._wait_for_path_clear(
            0.0, self.obstacle_forward_distance,
            self.obstacle_wait_timeout, 'goto-point pre-check'
        ):
            return False
        if not self._drive_to_xy(px, py):
            return False
        pose = self.lookup_robot_pose()
        if pose is None:
            return False
        rx, ry, _ = pose
        return self._spin_to_yaw(math.atan2(tag_y - ry, tag_x - rx))


    def _final_visual_approach(self, cx, cy, normal_yaw, max_time=90.0):
        """Two-regime final approach.

        FAR (camera depth > freeze_axis_distance): the axis (dock centre +
        normal) is re-derived every iteration from the live 3-tag perception and
        folded into a **running average**, and the robot pure-pursuits that axis
        — so it converges onto the perpendicular line while still far, where the
        estimate is clean.

        NEAR (≤ freeze_axis_distance): the averaged axis is **frozen** (close-up
        estimates are noisy and the outer tags start leaving the FOV — averaging
        them in caused the end-of-approach zig-zag). The robot finishes on the
        **centre-tag visual corrector** (keep the centre tag centred in the
        image), which from an already-aligned pose only trims the residual.

        Stops when camera→centre-tag depth ≤ docking_distance.
        """
        period = 1.0 / self.drive_rate_hz
        deadline = time.time() + max_time
        camera_forward_offset = 0.35   # camera_link is +0.35 m ahead of base_link (URDF)

        acx, acy = cx, cy
        asin, acos = math.sin(normal_yaw), math.cos(normal_yaw)
        n = 1
        frozen = False
        filtered_angle = None

        pose0 = self.lookup_robot_pose()
        if pose0 is None:
            return False
        x0, y0, _ = pose0
        max_travel = math.hypot(cx - x0, cy - y0) + 0.5

        while time.time() < deadline:
            pose = self.lookup_robot_pose()
            if pose is None:
                time.sleep(period)
                continue
            rx, ry, ryaw = pose

            c1cam = self._lookup_tag_cam('charging_dock_tag_1')
            if c1cam is not None and c1cam[2] > 0.05:
                depth = c1cam[2]
            else:
                depth = max(0.0, math.hypot(cx - rx, cy - ry)
                            - camera_forward_offset)
            if depth <= self.docking_distance:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(
                    f'   docked: depth {depth:.3f}m ≤ {self.docking_distance:.2f}m')
                return True

            if math.hypot(rx - x0, ry - y0) > max_travel:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().error('   exceeded travel safety')
                return False

            if depth > self.freeze_axis_distance:
                # FAR: refine the axis with a proximity-weighted EMA. The EMA
                # weight grows as the robot gets closer, so the samples taken
                # while advancing (nearer = tags bigger in the image = more
                # accurate) count MUCH more than the early far ones.
                est = self._estimate_dock_once()
                if est is not None:
                    ecx, ecy, eyaw = est
                    a = min(0.6, self.axis_filter_alpha
                            * (self.predock_distance / max(depth, 0.4)))
                    acx = (1.0 - a) * acx + a * ecx
                    acy = (1.0 - a) * acy + a * ecy
                    asin = (1.0 - a) * asin + a * math.sin(eyaw)
                    acos = (1.0 - a) * acos + a * math.cos(eyaw)
                    n += 1
                cx, cy = acx, acy
                normal_yaw = math.atan2(asin, acos)
                lateral = (-(rx - cx) * math.sin(normal_yaw)
                           + (ry - cy) * math.cos(normal_yaw))
                desired_yaw = normalize_angle(
                    normal_yaw - math.atan2(lateral, self.line_lookahead_distance))
                yaw_err = normalize_angle(desired_yaw - ryaw)
                omega = self.line_yaw_kp * yaw_err
            else:
                # NEAR: axis frozen, finish on the centre-tag visual corrector.
                if not frozen:
                    frozen = True
                    self.get_logger().info(
                        f'   depth {depth:.2f}m ≤ {self.freeze_axis_distance:.2f}m '
                        f'— freezing axis ({n} samples), visual corrector')
                if c1cam is not None and c1cam[2] > 0.0:
                    raw_angle = math.atan2(c1cam[0], c1cam[2])
                    if filtered_angle is None:
                        filtered_angle = raw_angle
                    else:
                        a = self.visual_servo_filter_alpha
                        filtered_angle = a * raw_angle + (1.0 - a) * filtered_angle
                    omega = -self.visual_servo_kp * filtered_angle
                else:
                    omega = 0.0   # tag out of view in the last cm — go straight

            omega = max(-self.drive_yaw_max_omega,
                        min(self.drive_yaw_max_omega, omega))
            v = self.drive_speed
            taper = 2.0 * self.docking_distance
            if depth < taper:
                v = max(0.03, self.drive_speed * depth / taper)

            self._publish_cmd_vel(v, omega)
            self._publish_line_markers(cx, cy, normal_yaw)
            self._publish_gz_line_marker(cx, cy, normal_yaw)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().error('   final approach timeout')
        return False

    def _estimate_dock_once(self):
        """One-shot dock estimate (cx, cy, normal_yaw) in the map frame from a
        single read of the three tags, or None if the outer tags aren't both
        visible. Used to keep the approach axis adapting in real time."""
        p0 = self._lookup_tag_map('charging_dock_tag_0')
        p2 = self._lookup_tag_map('charging_dock_tag_2')
        if p0 is None or p2 is None:
            return None
        p1 = self._lookup_tag_map('charging_dock_tag_1')
        pose = self.lookup_robot_pose()
        if pose is None:
            return None
        rx, ry, _ = pose
        return self._dock_pose_from_tags(p0, p1, p2, rx, ry)

    # ──────────────────────────────────────────────────────────────────────
    # Undock sequence
    # ──────────────────────────────────────────────────────────────────────
    def run_undock_sequence(self) -> bool:
        """Reverse undock_reverse_distance metres in a straight line, then
        spin 180° in place. Clears is_docked on success.
        """
        self.get_logger().info(
            f'── UNDOCK: reverse {self.undock_reverse_distance:.2f}m, then spin 180°'
        )
        # Garde-fou: verify the rear path is clear before starting the reverse.
        # The 180° in-place spin afterwards does not move the chassis through
        # space and is not guarded here.
        if not self._wait_for_path_clear(
            math.pi, self.obstacle_backward_distance,
            self.obstacle_wait_timeout, 'undock pre-check'
        ):
            self.get_logger().error('   undock aborted: rear path blocked')
            return False
        if not self._reverse_distance(self.undock_reverse_distance):
            self.get_logger().error('   undock reverse failed')
            return False

        pose = self.lookup_robot_pose()
        if pose is None:
            self.get_logger().error('   could not read robot pose for 180° spin')
            return False
        _, _, ryaw = pose
        target_yaw = normalize_angle(ryaw + math.pi)
        self.get_logger().info(
            f'   spinning 180° (from {ryaw:.3f} to {target_yaw:.3f})'
        )
        # 180° is the longest spin — give it more time than the default 15 s
        # (a slow spin_max_omega or collision-monitor clamping can stretch it).
        if not self._spin_to_yaw(target_yaw, max_time=30.0):
            self.get_logger().error('   undock 180° spin failed')
            return False

        self.is_docked = False
        self.get_logger().info('Undock complete ✓ — robot is free to navigate')
        return True

    def _reverse_distance(self, dist: float, max_extra_time: float = 10.0) -> bool:
        """Drive straight backward until the robot has travelled `dist` metres
        from its starting position (measured in the map frame). No steering.
        """
        period = 1.0 / self.drive_rate_hz
        pose0 = self.lookup_robot_pose()
        if pose0 is None:
            return False
        x0, y0, _ = pose0
        speed = max(0.02, self.undock_reverse_speed)
        deadline = time.time() + dist / speed + max_extra_time

        while time.time() < deadline:
            # Obstacle guard (backward cone). Important during undock —
            # someone behind the robot is the prototypical surprise case.
            if not self._wait_for_path_clear(
                math.pi, self.obstacle_backward_distance,
                self.obstacle_wait_timeout, 'undock reverse'
            ):
                self._publish_cmd_vel(0.0, 0.0)
                return False

            pose = self.lookup_robot_pose()
            if pose is None:
                time.sleep(period)
                continue
            rx, ry, _ = pose
            travelled = math.hypot(rx - x0, ry - y0)
            if travelled >= dist:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(f'   reversed {travelled:.2f}m')
                return True
            self._publish_cmd_vel(-speed, 0.0)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().error('   reverse timeout')
        return False

    def _has_fresh_detection(self) -> bool:
        """True if /detected_dock_pose carries a message younger than
        detection_max_age."""
        msg = self.detected_pose
        if msg is None:
            return False
        age = (self.get_clock().now()
               - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        return age < self.detection_max_age

    def _search_for_tag(self) -> bool:
        """Rotate in place until BOTH tags are visible and their midpoint is
        centred in the image for scan_consecutive_target consecutive frames.
        Bounded by ~one rotation.

        Centring uses the midpoint of the two tags in camera_optical_frame:
        with +X right and +Z forward, the horizontal offset is atan2(X, Z).
        If only one tag is visible we steer toward it to bring the other into
        view; if neither is visible we rotate open-loop.
        """
        period = 1.0 / self.drive_rate_hz
        timeout_s = 2.0 * math.pi / max(0.05, self.scan_rotation_speed) + 15.0
        deadline = time.time() + timeout_s
        centred_count = 0

        self.get_logger().info(
            f'   scanning for BOTH tags (tolerance '
            f'±{math.degrees(self.scan_centring_tolerance):.1f}°, '
            f'need {self.scan_consecutive_target} consecutive frames)'
        )

        while time.time() < deadline:
            both = self._dock_center_cam(require_all=True)
            if both is not None and both[2] > 0.05:
                image_angle = math.atan2(both[0], both[2])
                if abs(image_angle) < self.scan_centring_tolerance:
                    centred_count += 1
                    if centred_count >= self.scan_consecutive_target:
                        self._publish_cmd_vel(0.0, 0.0)
                        self.get_logger().info(
                            f'   both tags centred '
                            f'(angle={math.degrees(image_angle):+.1f}°)')
                        return True
                    self._publish_cmd_vel(0.0, 0.0)
                else:
                    centred_count = 0
                    omega = -self.scan_centring_kp * image_angle
                    omega = max(-self.scan_rotation_speed,
                                min(self.scan_rotation_speed, omega))
                    self._publish_cmd_vel(0.0, omega)
            else:
                centred_count = 0
                one = self._any_tag_cam()
                if one is not None and one[2] > 0.05:
                    # One tag visible — steer toward it to reveal the others.
                    omega = -self.scan_centring_kp * math.atan2(one[0], one[2])
                    omega = max(-self.scan_rotation_speed,
                                min(self.scan_rotation_speed, omega))
                    self._publish_cmd_vel(0.0, omega)
                else:
                    self._publish_cmd_vel(0.0, self.scan_rotation_speed)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        return False

    def _perpendicular_yaw_from_latest_tag(self, rx: float, ry: float):
        """Read the latest /detected_dock_pose and return the yaw the robot
        should have to be perpendicular to the tag plane (facing the tag).

        Returns None if no fresh detection is available.
        """
        msg = self.detected_pose
        if msg is None:
            return None
        age = (self.get_clock().now() -
               rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > self.detection_max_age:
            return None

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        tx = msg.pose.position.x
        ty = msg.pose.position.y

        nx, ny, _ = quat_rotate_z(qx, qy, qz, qw)
        n_norm = math.hypot(nx, ny)
        if n_norm < 1e-6:
            return None
        nx /= n_norm
        ny /= n_norm
        # Disambiguate sign so normal points from tag toward robot
        if nx * (rx - tx) + ny * (ry - ty) < 0:
            nx, ny = -nx, -ny
        # Yaw to face the tag = opposite of normal direction
        return math.atan2(-ny, -nx)

    # ──────────────────────────────────────────────────────────────────────
    # Temporal filtering of tag detection
    # ──────────────────────────────────────────────────────────────────────
    def get_tag_pose_filtered(self):
        """Collect N fresh tag detections and average them.

        Returns (x, y, qx, qy, qz, qw) in map frame, or None if no samples.
        - Position averaged component-wise (works since variations are small)
        - Quaternion averaged component-wise then re-normalised (valid for
          small perturbations around a nominal orientation, which is our case)
        """
        samples = []
        deadline = time.time() + self.filter_max_collect_time
        last_stamp = None

        while time.time() < deadline and len(samples) < self.filter_num_samples:
            msg = self.detected_pose
            if msg is not None:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if stamp != last_stamp:
                    last_stamp = stamp
                    age = (self.get_clock().now() -
                           rclpy.time.Time.from_msg(msg.header.stamp)
                          ).nanoseconds * 1e-9
                    if age < self.detection_max_age:
                        samples.append(msg)
            time.sleep(0.05)

        if not samples:
            return None

        n = len(samples)
        avg_x = sum(m.pose.position.x for m in samples) / n
        avg_y = sum(m.pose.position.y for m in samples) / n

        avg_qx = sum(m.pose.orientation.x for m in samples) / n
        avg_qy = sum(m.pose.orientation.y for m in samples) / n
        avg_qz = sum(m.pose.orientation.z for m in samples) / n
        avg_qw = sum(m.pose.orientation.w for m in samples) / n
        norm = math.sqrt(avg_qx**2 + avg_qy**2 + avg_qz**2 + avg_qw**2)
        if norm > 1e-9:
            avg_qx /= norm
            avg_qy /= norm
            avg_qz /= norm
            avg_qw /= norm

        self.get_logger().info(f'   averaged {n} samples')
        return avg_x, avg_y, avg_qx, avg_qy, avg_qz, avg_qw

    # ──────────────────────────────────────────────────────────────────────
    # Parallel spot from tag normal
    # ──────────────────────────────────────────────────────────────────────
    def compute_parallel_spot(self, tx, ty, qx, qy, qz, qw, distance):
        """Compute the spot perpendicular to the tag plane, at `distance`
        meters from the tag, on the SAME SIDE as the robot.

        Returns (spot_x, spot_y, target_yaw, nx, ny) where:
          - (spot_x, spot_y) is the spot in map frame
          - target_yaw makes the robot face the tag (perpendicular to tag plane)
          - (nx, ny) is the unit normal pointing FROM tag TOWARD robot
        """
        pose = self.lookup_robot_pose()
        if pose is None:
            return None
        rx, ry, _ = pose

        # Tag's +Z axis in map frame (apriltag convention: out of tag face)
        nx, ny, _ = quat_rotate_z(qx, qy, qz, qw)
        n_norm = math.hypot(nx, ny)
        if n_norm < 1e-6:
            self.get_logger().warn('   tag normal has zero xy magnitude — '
                                   'falling back to dock_yaw direction')
            nx = -math.cos(self.dock_yaw)
            ny = -math.sin(self.dock_yaw)
            n_norm = 1.0
        nx /= n_norm
        ny /= n_norm

        # The apriltag ±Z convention is ambiguous (some conventions point
        # toward camera, others into wall). Disambiguate by checking which
        # direction is on the robot's side: dot product with (robot - tag).
        if nx * (rx - tx) + ny * (ry - ty) < 0:
            nx = -nx
            ny = -ny

        spot_x = tx + distance * nx
        spot_y = ty + distance * ny
        target_yaw = math.atan2(-ny, -nx)
        return spot_x, spot_y, target_yaw, nx, ny

    # ──────────────────────────────────────────────────────────────────────
    # Phase implementations
    # ──────────────────────────────────────────────────────────────────────
    def navigate_to_staging(self) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action not available')
            return False
        sx = self.dock_x - self.staging_distance * math.cos(self.dock_yaw)
        sy = self.dock_y - self.staging_distance * math.sin(self.dock_yaw)
        self.get_logger().info(f'   → staging ({sx:.2f}, {sy:.2f}, yaw={self.dock_yaw:.2f})')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = sx
        goal.pose.pose.position.y = sy
        goal.pose.pose.orientation.z = math.sin(self.dock_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.dock_yaw / 2.0)
        return self._send_action_blocking(self.nav_client, goal)

    def _spin_to_yaw(self, target_yaw: float, max_time: float = 15.0) -> bool:
        """Spin in place until the robot's yaw in map frame matches target_yaw.

        Bypass nav2's Spin action (which has an eager costmap collision check)
        — publish cmd_vel directly, P-controlled.
        """
        period = 1.0 / self.drive_rate_hz
        deadline = time.time() + max_time
        stable_required = 5
        stable_count = 0

        while time.time() < deadline:
            pose = self.lookup_robot_pose()
            if pose is None:
                time.sleep(period)
                continue
            _, _, ryaw = pose
            err = normalize_angle(target_yaw - ryaw)

            if abs(err) < self.spin_yaw_tolerance:
                stable_count += 1
                if stable_count >= stable_required:
                    self._publish_cmd_vel(0.0, 0.0)
                    self.get_logger().info(
                        f'   spin done: yaw={ryaw:.3f} target={target_yaw:.3f} err={err:.3f}'
                    )
                    return True
                self._publish_cmd_vel(0.0, 0.0)
                time.sleep(period)
                continue

            stable_count = 0
            omega = self.spin_kp * err
            omega = max(-self.spin_max_omega, min(self.spin_max_omega, omega))
            self._publish_cmd_vel(0.0, omega)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().error('   _spin_to_yaw timeout')
        return False

    def _drive_to_xy(self, tx: float, ty: float, max_time: float = 60.0) -> bool:
        """Drive forward toward (tx, ty) in map frame, correcting heading along
        the way. Stops when robot is within position_tolerance of (tx, ty)."""
        period = 1.0 / self.drive_rate_hz
        deadline = time.time() + max_time
        pose0 = self.lookup_robot_pose()
        if pose0 is None:
            return False
        x0, y0, _ = pose0
        max_travel = math.hypot(tx - x0, ty - y0) + 0.5
        position_tolerance = 0.05

        while time.time() < deadline:
            # Obstacle guard: if the forward cone has a return closer than
            # obstacle_forward_distance, stop, wait for it to clear, then
            # resume. If still blocked after obstacle_wait_timeout, abort.
            if not self._wait_for_path_clear(
                0.0, self.obstacle_forward_distance,
                self.obstacle_wait_timeout, 'forward drive'
            ):
                self._publish_cmd_vel(0.0, 0.0)
                return False

            pose = self.lookup_robot_pose()
            if pose is None:
                time.sleep(period)
                continue
            rx, ry, ryaw = pose
            distance = math.hypot(tx - rx, ty - ry)
            if distance < position_tolerance:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(f'   reached spot: dist={distance:.3f}m')
                return True
            if math.hypot(rx - x0, ry - y0) > max_travel:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().error('   exceeded travel safety bound')
                return False

            # Heading correction
            target_yaw = math.atan2(ty - ry, tx - rx)
            yaw_err = normalize_angle(target_yaw - ryaw)
            omega = self.drive_yaw_kp * yaw_err
            omega = max(-self.drive_yaw_max_omega, min(self.drive_yaw_max_omega, omega))

            # Taper near goal
            taper = 5.0 * position_tolerance
            v = self.drive_speed
            if distance < taper:
                v = max(0.03, self.drive_speed * (distance / taper))
            # If yaw way off, slow down to rotate first
            if abs(yaw_err) > 0.3:
                v *= 0.3

            self._publish_cmd_vel(v, omega)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().error('   _drive_to_xy timeout')
        return False

    # ──────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────
    def _latest_tag_xy(self, fallback_x: float, fallback_y: float):
        msg = self.detected_pose
        if msg is None:
            return fallback_x, fallback_y
        age = (self.get_clock().now() -
               rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > self.detection_max_age:
            return fallback_x, fallback_y
        return msg.pose.position.x, msg.pose.position.y

    def _lateral_offset_to_axis(self, rx: float, ry: float) -> float:
        """Signed perpendicular distance from the robot to the dock's
        perpendicular axis (the line through the configured dock pose in
        the direction of dock_yaw).

        Sign convention: positive when the robot is to the LEFT of the axis
        (looking toward the tag, i.e. along +dock_yaw). The dock pose used
        here is the canonical one from config — using the noisy live tag
        detection here would let detection jitter spuriously trigger the
        reverse-and-realign behavior.
        """
        dx = rx - self.dock_x
        dy = ry - self.dock_y
        return -dx * math.sin(self.dock_yaw) + dy * math.cos(self.dock_yaw)

    def _compute_realign_target(self, rx: float, ry: float):
        """Compute the realign target point: the projection of (rx, ry) onto
        the dock axis, moved further back (away from the dock along
        −dock_yaw direction) by realign_reverse_distance.

        Returns (target_x, target_y). Reaching this point with the body
        oriented perpendicular to the tag will leave the robot on the axis
        (lateral offset = 0), behind its current axial position, ready to
        re-advance perpendicular to the tag plane.
        """
        cos_y = math.cos(self.dock_yaw)
        sin_y = math.sin(self.dock_yaw)
        # Project robot onto the axis: signed distance from dock along +dock_yaw.
        axial = (rx - self.dock_x) * cos_y + (ry - self.dock_y) * sin_y
        # Foot of perpendicular from robot onto the axis.
        foot_x = self.dock_x + axial * cos_y
        foot_y = self.dock_y + axial * sin_y
        # Step further back along −dock_yaw direction.
        target_x = foot_x - self.realign_reverse_distance * cos_y
        target_y = foot_y - self.realign_reverse_distance * sin_y
        return target_x, target_y

    # ──────────────────────────────────────────────────────────────────────
    # Obstacle avoidance (forward/backward cone in the laser scan)
    # ──────────────────────────────────────────────────────────────────────
    def _min_range_in_arc(self, center_angle: float, half_width: float) -> float:
        """Return the minimum LIDAR range inside the angular cone centred on
        `center_angle` (radians, in the scan frame: 0 = forward, π = backward)
        with half-width `half_width` (radians). Returns +inf if no laser data
        is available or no finite return is inside the cone.

        Returns shorter than `obstacle_min_range` are discarded as
        self-reflections from the robot's own body (the LIDAR sits on top of
        the chassis and rays heading rearward in particular hit the
        enclosure). Without this floor the backward cone always reads ~0.2 m
        and the undock phase blocks forever.

        Note: angles are taken in the scan's frame, not base_link. For a LIDAR
        mounted near the geometric centre of the robot with its forward axis
        aligned with base_link forward, this is a fine approximation. A
        significantly off-centre LIDAR would need a TF-based projection.
        """
        scan = self.latest_scan
        if scan is None:
            return float('inf')
        a_lo = normalize_angle(center_angle - half_width)
        a_hi = normalize_angle(center_angle + half_width)
        wrap = a_lo > a_hi  # cone spans the ±π wrap (e.g. backward at π)
        floor = max(self.obstacle_min_range, scan.range_min)
        min_r = float('inf')
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < floor or r > scan.range_max:
                continue
            a = normalize_angle(scan.angle_min + i * scan.angle_increment)
            inside = (a_lo <= a <= a_hi) if not wrap else (a >= a_lo or a <= a_hi)
            if inside and r < min_r:
                min_r = r
        return min_r

    def _wait_for_path_clear(self, center_angle: float, distance: float,
                             max_wait: float, label: str) -> bool:
        """Stop the robot and poll the laser scan until the path in the given
        direction is clear (no return closer than `distance` inside the cone),
        or `max_wait` seconds elapse.

        Returns True if the path became clear (or was already clear, or
        checking is disabled / no scan available — see fall-back below).
        Returns False on timeout — the caller should abort the phase.
        """
        if not self.obstacle_check_enabled:
            return True
        if self.latest_scan is None:
            # No /scan received yet — proceed but warn. This avoids deadlocking
            # the docking sequence if the LIDAR pipeline is degraded; the
            # standard navigation safety layer still applies upstream.
            self.get_logger().warn(
                f'   {label}: no /scan received yet; proceeding without check'
            )
            return True
        min_r = self._min_range_in_arc(center_angle, self.obstacle_arc_half_width)
        if min_r > distance:
            return True  # path already clear, nothing to do
        # First contact — stop, log, then poll until clear or timeout
        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().warn(
            f'   {label}: obstacle at {min_r:.2f} m (threshold {distance:.2f} m), '
            f'waiting up to {max_wait:.0f} s for it to clear…'
        )
        deadline = time.time() + max_wait
        while time.time() < deadline:
            time.sleep(self.obstacle_check_period)
            min_r = self._min_range_in_arc(center_angle, self.obstacle_arc_half_width)
            if min_r > distance:
                self.get_logger().info(
                    f'   {label}: path cleared (now {min_r:.2f} m), resuming'
                )
                return True
        self.get_logger().error(
            f'   {label}: still blocked after {max_wait:.0f} s, aborting phase'
        )
        return False

    def _publish_cmd_vel(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_vel_pub.publish(msg)

    def _publish_line_markers(self, cx: float, cy: float, perp_yaw: float):
        """Publish the perpendicular approach line and the dock centre as RViz
        markers in the map frame. See docs/13_perception_and_line.md.
        """
        if not self.publish_debug_markers:
            return
        now = self.get_clock().now().to_msg()
        dirx, diry = math.cos(perp_yaw), math.sin(perp_yaw)
        arr = MarkerArray()

        # Green line: the perpendicular approach axis through the dock centre.
        # perp_yaw points from the robot toward the dock, so the robot side of
        # the line is at centre − dir; draw from there to just past the dock.
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = now
        line.ns = 'docking_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.g = 1.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0
        line.points = [
            Point(x=cx - 4.0 * dirx, y=cy - 4.0 * diry, z=0.15),
            Point(x=cx + 0.3 * dirx, y=cy + 0.3 * diry, z=0.15),
        ]
        arr.markers.append(line)

        # Red sphere: the dock centre.
        tag = Marker()
        tag.header.frame_id = 'map'
        tag.header.stamp = now
        tag.ns = 'docking_tag'
        tag.id = 1
        tag.type = Marker.SPHERE
        tag.action = Marker.ADD
        tag.pose.position.x = cx
        tag.pose.position.y = cy
        tag.pose.position.z = 0.30
        tag.pose.orientation.w = 1.0
        tag.scale.x = tag.scale.y = tag.scale.z = 0.12
        tag.color.r = 1.0
        tag.color.a = 1.0
        arr.markers.append(tag)

        self.marker_pub.publish(arr)

    def _publish_gz_line_marker(self, cx_in: float, cy_in: float, perp_yaw: float):
        """Mirror the line + dock centre into the Gazebo GUI via the gz marker
        service. Throttled and run off-thread so the control loop never blocks.
        Map ≡ world in this sim, so (cx_in, cy_in) are world coordinates.
        """
        if not self.publish_gz_marker or self._gz_marker_inflight:
            return
        now = time.time()
        if now - self._last_gz_marker_t < self.gz_marker_period:
            return
        self._last_gz_marker_t = now

        # map ≡ world in this sim (robot spawns at the world origin, AMCL is
        # initialised there), so the map-frame estimate is also the world pose.
        wx, wy, wyaw = cx_in, cy_in, perp_yaw

        # A thin CYLINDER (not LINE_STRIP) — gz renders line markers as 1-px
        # lines that are nearly invisible in the 3D view. A cylinder is a real
        # mesh: always visible, properly coloured, thickness controllable.
        dirx, diry = math.cos(wyaw), math.sin(wyaw)
        x1, y1 = wx - 4.0 * dirx, wy - 4.0 * diry
        x2, y2 = wx + 0.3 * dirx, wy + 0.3 * diry
        cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        length = math.hypot(x2 - x1, y2 - y1)
        # Quaternion rotating the cylinder's +Z axis onto the (horizontal) line
        # direction: a 90° rotation about the axis Z × dir = (−sin, cos, 0).
        qw = 0.70710678
        qx = -0.70710678 * diry
        qy = 0.70710678 * dirx
        req = (
            'marker { action: ADD_MODIFY ns: "docking_line" id: 1 type: CYLINDER '
            'material { ambient { r: 0 g: 1 b: 0 a: 1 } diffuse { r: 0 g: 1 b: 0 a: 1 } } '
            f'pose {{ position {{ x: {cx:.3f} y: {cy:.3f} z: 0.15 }} '
            f'orientation {{ x: {qx:.5f} y: {qy:.5f} z: 0.0 w: {qw:.5f} }} }} '
            f'scale {{ x: 0.04 y: 0.04 z: {length:.3f} }} '
            '} '
            'marker { action: ADD_MODIFY ns: "docking_tag" id: 2 type: SPHERE '
            'material { ambient { r: 1 g: 0 b: 0 a: 1 } diffuse { r: 1 g: 0 b: 0 a: 1 } } '
            f'pose {{ position {{ x: {wx:.3f} y: {wy:.3f} z: 0.30 }} '
            'orientation { w: 1 } } '
            'scale { x: 0.08 y: 0.08 z: 0.08 } }'
        )
        self._gz_marker_inflight = True
        threading.Thread(target=self._run_gz_marker, args=(req,), daemon=True).start()

    def _run_gz_marker(self, req: str):
        try:
            subprocess.run(
                ['gz', 'service', '-s', self.gz_marker_service,
                 '--reqtype', 'gz.msgs.Marker_V',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=2.0,
            )
        except Exception:
            pass
        finally:
            self._gz_marker_inflight = False

    def lookup_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = quat_to_yaw(t.transform.rotation)
        return x, y, yaw

    # ── Multi-tag perception (3 tags: id0/id2 outer, id1 centre) ──────────
    def _lookup_tag_cam(self, frame):
        """3D position (x, y, z) of `frame` in camera_optical_frame, or None if
        the TF is missing or stale."""
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_optical_frame', frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        age = (self.get_clock().now()
               - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > self.detection_max_age:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z)

    def _lookup_tag_map(self, frame):
        """(x, y) of `frame` in the map frame, or None if missing/stale."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        age = (self.get_clock().now()
               - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > self.detection_max_age:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _dock_center_cam(self, require_all=True):
        """The CENTRE tag (id1) in camera_optical_frame — the thing we centre
        on and drive onto. With require_all=True, return it only when all three
        tags are visible (so the dock can be estimated); with require_all=False
        return it as soon as the centre tag is visible (near field, where the
        outer tags leave the FOV first)."""
        c1 = self._lookup_tag_cam('charging_dock_tag_1')
        if c1 is None:
            return None
        if require_all:
            if (self._lookup_tag_cam('charging_dock_tag_0') is None or
                    self._lookup_tag_cam('charging_dock_tag_2') is None):
                return None
        return c1

    def _any_tag_cam(self):
        """Any visible tag in camera_optical_frame (for the search scan)."""
        for f in ('charging_dock_tag_1', 'charging_dock_tag_0',
                  'charging_dock_tag_2'):
            c = self._lookup_tag_cam(f)
            if c is not None:
                return c
        return None

    def _send_action_blocking(self, client, goal) -> bool:
        send_future = client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.05)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Goal rejected')
            return False
        result_future = gh.get_result_async()
        while not result_future.done():
            time.sleep(0.05)
        result = result_future.result()
        if result is None:
            self.get_logger().error('No result returned')
            return False
        if result.status != 4:
            self.get_logger().error(f'Action ended with status {result.status}')
            return False
        return True

    def _send_undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('undock_robot action not available')
            return
        goal = UndockRobot.Goal()
        goal.dock_type = str(self.dock_type)
        self.undock_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = DockTrigger()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
