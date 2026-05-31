#!/usr/bin/env python3
"""
Custom rotate-then-drive docking trigger.

Sequence on /dock_trigger=true:
  1. NavigateToPose → staging zone (Nav2/RPP)
  2. Hold N seconds (robot still)
  3. Read /detected_dock_pose (or fall back to static dock pose)
  4. Spin in place until robot heading points at the tag (closed-loop)
  5. Drive forward in a straight line, continuously correcting heading from
     live AprilTag detection, until docking_distance from the tag
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import (
    NavigateToPose,
    UndockRobot,
)
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


class TagRunningAverage:
    """Incremental running mean of the tag's pose in the map frame.

    update() is called once per fresh detection. Positions average naturally
    (true running mean). Quaternions are averaged componentwise after sign
    alignment to the existing mean, then renormalised — this is the
    well-known "naive quaternion mean", valid when all samples lie close to
    each other on the sphere (which is the case here: the tag is static and
    detections cluster around the true pose).
    """

    def __init__(self):
        self.count = 0
        self.total_weight = 0.0   # sum of weights, for the weighted mean
        self.x = 0.0
        self.y = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0

    def update(self, x, y, qx, qy, qz, qw, weight=1.0):
        """Fold a new sample into the running mean with the given weight.

        weight = 1.0 reproduces the original true-mean behaviour (each
        sample contributes equally). For a distance-weighted mean (used
        during Phase 4 so noisier near-field detections influence the
        mean less than the clean far-field ones), pass a weight that
        increases with the distance at which the sample was taken.
        """
        if weight <= 0.0:
            return
        self.count += 1
        new_total = self.total_weight + weight
        # Sign-align the new quaternion to the existing mean so q and -q
        # don't cancel each other.
        if self.count > 1 and (qx * self.qx + qy * self.qy
                               + qz * self.qz + qw * self.qw) < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        # Weighted incremental mean:
        # m_new = (W_old * m_old + w * x) / (W_old + w)
        #       = m_old + (w / W_new) * (x - m_old)
        k = weight / new_total
        self.x += k * (x - self.x)
        self.y += k * (y - self.y)
        self.qx += k * (qx - self.qx)
        self.qy += k * (qy - self.qy)
        self.qz += k * (qz - self.qz)
        self.qw += k * (qw - self.qw)
        self.total_weight = new_total
        # Renormalise the quaternion.
        norm = math.sqrt(self.qx * self.qx + self.qy * self.qy
                         + self.qz * self.qz + self.qw * self.qw)
        if norm > 1e-9:
            self.qx /= norm
            self.qy /= norm
            self.qz /= norm
            self.qw /= norm

    def perpendicular_yaw(self, rx, ry):
        """Yaw the robot should have to face the tag perpendicular to its
        plane, using the current running-average quaternion. Returns None
        until at least one sample has been collected.
        """
        if self.count == 0:
            return None
        nx, ny, _ = quat_rotate_z(self.qx, self.qy, self.qz, self.qw)
        n_norm = math.hypot(nx, ny)
        if n_norm < 1e-6:
            return None
        nx /= n_norm
        ny /= n_norm
        if nx * (rx - self.x) + ny * (ry - self.y) < 0.0:
            nx, ny = -nx, -ny
        return math.atan2(-ny, -nx)

    def signed_lateral_offset(self, rx, ry, perp_yaw):
        """Signed perpendicular distance from (rx, ry) to the line passing
        through the running-average tag centre in direction perp_yaw.

        Positive = robot is to the LEFT of the line (looking along perp_yaw
        toward the tag). The robot reduces this offset by steering CW.
        """
        return (-(rx - self.x) * math.sin(perp_yaw)
                + (ry - self.y) * math.cos(perp_yaw))


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

        # Distance below which Phase 4 switches from map-frame line-tracking
        # to camera-frame visual servoing. Above this distance, the
        # running-average filter and the pure-pursuit on the perpendicular
        # line do the work (they handle long-range alignment well, with
        # plenty of clean detections far from the tag). Below it, the
        # running average is frozen (no more updates) and the heading
        # controller switches to a direct closed-loop on the image-frame
        # angle to the tag:
        #
        #     omega = -visual_servo_kp · atan2(X_optical, Z_optical)
        #
        # This works because map-frame solvePnP carries a systematic bias
        # in the near field (corners at the bottom of the FOV are noisy),
        # but the image-frame angle is self-consistent: it directly
        # describes where the tag IS in the camera, not where solvePnP
        # thinks it is in the map. Keeping the tag centred in the image
        # = aiming straight at the dock.
        #
        # Linear forward speed continues with the taper.
        # Set to 0 to disable visual servoing (line-tracking all the way).
        self.declare_parameter('visual_servo_distance', 1.0)
        self.declare_parameter('visual_servo_kp', 1.0)         # rad/s per rad of image-frame angle

        # Number of refinement samples to accept in Phase 4 before
        # declaring the line "stabilised" and handing control over to
        # the visual servo. Once this many fresh detections have been
        # folded into the running average (the cleanest ones, taken
        # while the robot is still relatively far from the dock), the
        # line is locked-in and the robot stops refining it. The visual
        # servo then handles the final approach using the live camera-
        # frame angle (which doesn't depend on the noisy near-field
        # solvePnP outputs).
        #
        # Set to 0 to disable the count-based trigger (only the distance
        # threshold visual_servo_distance will then drive the handover).
        self.declare_parameter('line_stabilization_samples', 25)
        # Low-pass smoothing on the image-frame angle to reject noisy
        # solvePnP spikes near the dock. alpha = 0.2 gives a time
        # constant of ~5 frames (≈ 0.35 s at 14 Hz detection rate);
        # alpha = 1.0 disables filtering.
        self.declare_parameter('visual_servo_filter_alpha', 0.2)

        # Outlier rejection on Phase 4 running-average updates.
        # A new detection that lands more than this many metres from
        # the current running-average position in the XY plane is almost
        # certainly a single-frame solvePnP glitch and is SKIPPED (not
        # folded into the mean). Typical legitimate jitter is ~5 cm; 0.30
        # m is a generous threshold that still catches the gross outliers.
        # Set to 0 to disable outlier rejection.
        self.declare_parameter('refinement_outlier_threshold', 0.30)

        # Distance-based weight floor: how much weight near-field samples
        # contribute to the running mean relative to far-field ones. With
        # weight_min = 0.1 and weight_full_distance = 1.5:
        #   sample at 1.5 m → weight 1.0 (full, like Phase 2)
        #   sample at 0.75 m → weight 0.5
        #   sample at <= 0.15 m → weight 0.1 (clamped floor)
        # This keeps near-field (noisier) detections in the mean but lets
        # the clean far-field samples dominate.
        self.declare_parameter('refinement_weight_min', 0.1)
        self.declare_parameter('refinement_weight_full_distance', 1.5)

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
        self.visual_servo_distance = float(self.get_parameter('visual_servo_distance').value)
        self.visual_servo_kp = float(self.get_parameter('visual_servo_kp').value)
        self.visual_servo_filter_alpha = float(self.get_parameter('visual_servo_filter_alpha').value)
        self.line_stabilization_samples = int(self.get_parameter('line_stabilization_samples').value)
        self.refinement_outlier_threshold = float(self.get_parameter('refinement_outlier_threshold').value)
        self.refinement_weight_min = float(self.get_parameter('refinement_weight_min').value)
        self.refinement_weight_full_distance = float(self.get_parameter('refinement_weight_full_distance').value)
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

        # ── Docking status publisher (consumed by the web UI) ────────────────
        # Publishes one of: idle | docking | docked | undocking | failed
        self.dock_status_pub = self.create_publisher(String, 'dock_trigger_status', 10)

        # Heartbeat: re-publish current state every 2 s so the UI syncs on connect.
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
        # ── Phase 1: Nav2 to the canonical staging zone. ──────────────────
        self.get_logger().info('── Phase 1/4: NavigateToPose → staging zone')
        if not self.navigate_to_staging():
            self.get_logger().error('NavigateToPose failed — aborting')
            return

        # ── Phase 2: scan + initial filter of the tag pose. ───────────────
        # The scan rotates the robot in place until the tag has been
        # consistently visible. The filter then averages N samples while
        # stationary to seed the running-average tag estimate.
        self.get_logger().info(
            f'── Phase 2/4: tag search + initial filter ({self.filter_num_samples} samples)'
        )
        self._publish_cmd_vel(0.0, 0.0)
        time.sleep(self.staging_hold_seconds)

        if not self._search_for_tag():
            self.get_logger().error('   tag never detected during scan — aborting')
            return

        tag_avg = TagRunningAverage()
        if not self._collect_initial_samples(tag_avg, self.filter_num_samples):
            return

        self.get_logger().info(
            f'   tag at ({tag_avg.x:.3f}, {tag_avg.y:.3f}) after {tag_avg.count} samples'
        )

        # ── Phase 3: spin to the running-average perpendicular yaw. ──────
        pose = self.lookup_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        perp_yaw = tag_avg.perpendicular_yaw(rx, ry)
        if perp_yaw is None:
            self.get_logger().error('   could not compute perpendicular yaw — aborting')
            return

        self.get_logger().info(
            f'── Phase 3/4: ALIGN (spin to perpendicular yaw {perp_yaw:.3f})'
        )
        if not self._spin_to_yaw(perp_yaw):
            self.get_logger().error('   alignment spin failed')
            return

        # ── Phase 4: line-tracking advance. ──────────────────────────────
        # Drive forward while continuously updating the running average and
        # steering to stay on the perpendicular line through the averaged
        # tag. No discrete realign, no separate auto-cal — one continuous
        # closed loop. Stop when distance to the averaged tag ≤ docking
        # distance.
        self.get_logger().info(
            f'── Phase 4/4: line-tracking advance to {self.docking_distance:.2f}m'
        )
        if not self._advance_with_line_tracking(tag_avg):
            self.get_logger().error('   advance failed')
            return

        self.is_docked = True
        self.get_logger().info('Docking sequence complete ✓')

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
        if not self._spin_to_yaw(target_yaw):
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
        """Rotate in place until the tag has been freshly detected AND
        centred in the camera image for scan_consecutive_target consecutive
        frames. Bounded by one full rotation.

        The centring uses TF camera_optical_frame → charging_dock_apriltag
        directly: with +X right and +Z forward in the optical frame, the
        horizontal angular offset to centre is atan2(X, Z). The scan
        rotates at scan_rotation_speed when no detection is available, then
        switches to a yaw P-loop (omega = −scan_centring_kp × atan2(X, Z))
        once the tag is in view. We declare "centred" when |angle| <
        scan_centring_tolerance for the required consecutive frames.
        """
        period = 1.0 / self.drive_rate_hz
        timeout_s = 2.0 * math.pi / max(0.05, self.scan_rotation_speed) + 10.0
        deadline = time.time() + timeout_s
        centred_count = 0

        self.get_logger().info(
            f'   scanning to centre tag in camera '
            f'(tolerance ±{math.degrees(self.scan_centring_tolerance):.1f}°, '
            f'need {self.scan_consecutive_target} consecutive frames)'
        )

        while time.time() < deadline:
            cam_tag = self.lookup_tag_in_camera_optical()
            if cam_tag is not None and cam_tag[2] > 0.05:
                tx_cam, _, tz_cam = cam_tag
                image_angle = math.atan2(tx_cam, tz_cam)
                if abs(image_angle) < self.scan_centring_tolerance:
                    centred_count += 1
                    if centred_count >= self.scan_consecutive_target:
                        self._publish_cmd_vel(0.0, 0.0)
                        self.get_logger().info(
                            f'   tag centred in camera '
                            f'(image_angle={math.degrees(image_angle):+.1f}°, '
                            f'consecutive={centred_count})'
                        )
                        return True
                    # Hold (no command this iteration to keep the camera
                    # steady for the next centring check).
                    self._publish_cmd_vel(0.0, 0.0)
                else:
                    centred_count = 0
                    # Yaw P-loop to drive image_angle toward 0.
                    omega = -self.scan_centring_kp * image_angle
                    omega = max(-self.scan_rotation_speed,
                                min(self.scan_rotation_speed, omega))
                    self._publish_cmd_vel(0.0, omega)
            else:
                # No detection — open-loop slow rotation until we find it.
                centred_count = 0
                self._publish_cmd_vel(0.0, self.scan_rotation_speed)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        return False

    def _collect_initial_samples(self, avg: TagRunningAverage,
                                 num_samples: int) -> bool:
        """Block while stationary until avg has accumulated num_samples fresh
        detections, or filter_max_collect_time elapses. Each detection is
        identified by its header stamp so we never count the same message
        twice.
        """
        period = 0.05
        deadline = time.time() + self.filter_max_collect_time
        last_stamp_ns = -1

        while time.time() < deadline and avg.count < num_samples:
            msg = self.detected_pose
            if msg is not None:
                stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
                if stamp_ns != last_stamp_ns:
                    age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
                    if age < self.detection_max_age:
                        avg.update(
                            msg.pose.position.x, msg.pose.position.y,
                            msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w,
                        )
                    last_stamp_ns = stamp_ns
            time.sleep(period)

        if avg.count < num_samples:
            self.get_logger().error(
                f'   only got {avg.count}/{num_samples} samples in '
                f'{self.filter_max_collect_time:.1f}s — aborting'
            )
            return False
        return True

    def _advance_with_line_tracking(self, avg: TagRunningAverage,
                                    max_time: float = 90.0) -> bool:
        """Drive forward toward the running-average tag while steering to
        stay on the perpendicular line through the averaged tag centre.

        Each iteration:
          1) read robot pose;
          2) if a fresh detection is available, fold it into the running
             average — this REFINES both the tag centre (avg.x, avg.y) and
             the perpendicular line that the robot is tracking, even while
             the robot is in motion;
          3) compute perpendicular_yaw from the current average and the
             signed lateral offset from the perpendicular line;
          4) emit omega = line_yaw_kp × yaw_err, saturated by
             drive_yaw_max_omega — pure-pursuit on the perpendicular line;
          5) emit v = drive_speed, with a linear taper inside 2 ×
             docking_distance so the robot eases into the final stop;
          6) stop when distance(robot, avg) ≤ docking_distance.

        Detection-lost behaviour:
          - While fresh detections arrive: keep refining the line (avg
            gets updated every iteration with new detection data).
          - When detections stop arriving (tag out of FOV in the near
            field, occluded, etc.): freeze avg at its last value and KEEP
            following the line. No corrector kicks in at the end.

        The final visual-servo / one-shot align block from the original
        4-phase pipeline is intentionally removed — the running-average
        line is now refined all the way down to docking_distance, so the
        line itself converges on the true dock axis.
        """
        period = 1.0 / self.drive_rate_hz
        deadline = time.time() + max_time

        pose0 = self.lookup_robot_pose()
        if pose0 is None:
            return False
        x0, y0, _ = pose0
        d0 = math.hypot(avg.x - x0, avg.y - y0)
        max_travel = max(0.5, d0 + 0.5)

        last_stamp_ns = -1
        had_detection = True   # we entered Phase 4 with the 40-sample seed
        samples_in_phase4 = 0
        in_visual_servo_logged = False  # log the visual-servo transition once
        filtered_image_angle = None     # low-pass state for visual servo

        self.get_logger().info(
            f'   start d_to_tag={d0:.3f}m, forward to travel ≈ '
            f'{max(0.0, d0 - self.docking_distance):.3f}m'
            f' (visual servo activates at d ≤ {self.visual_servo_distance:.2f}m)'
        )

        while time.time() < deadline:
            pose = self.lookup_robot_pose()
            if pose is None:
                time.sleep(period)
                continue
            rx, ry, ryaw = pose

            # Distance to the running-average tag, computed BEFORE folding
            # the new detection. Drives one of the two visual-servo
            # triggers (the other is the refinement sample count).
            distance_to_avg = math.hypot(avg.x - rx, avg.y - ry)

            # The line is "stabilised" — and we hand over to the visual
            # servo — as soon as EITHER of these is true:
            #   • we've accepted line_stabilization_samples fresh
            #     detections in Phase 4 (the line is built from clean
            #     far-field samples, no point refining it further with
            #     the noisier near-field ones), OR
            #   • we've crossed the distance threshold (fallback,
            #     guards against the case where outlier rejection kills
            #     most samples and we never reach the count).
            line_stabilised_by_count = (
                self.line_stabilization_samples > 0
                and samples_in_phase4 >= self.line_stabilization_samples
            )
            line_stabilised_by_distance = (
                self.visual_servo_distance > 0.0
                and distance_to_avg <= self.visual_servo_distance
            )
            in_visual_servo = line_stabilised_by_count or line_stabilised_by_distance

            if in_visual_servo and not in_visual_servo_logged:
                in_visual_servo_logged = True
                trigger = ('count' if line_stabilised_by_count
                           else 'distance')
                self.get_logger().info(
                    f'   line stabilised ({trigger}: '
                    f'{samples_in_phase4} samples, d={distance_to_avg:.2f}m) '
                    f'— switching to camera-frame visual servo'
                )

            # Fold any new detection into the running average ONLY while
            # we're in the line-tracking regime. After visual-servo
            # handover, the running average is frozen — the heading
            # controller uses the live camera-frame angle directly.
            #
            # Two safeguards on Phase 4 updates:
            #   1. OUTLIER REJECTION — if the sample lands too far from
            #      the current mean, skip it (likely a 1-frame solvePnP
            #      glitch).
            #   2. DISTANCE-WEIGHTED MEAN — far-field samples (cleaner,
            #      tag is well inside the FOV) contribute more weight
            #      than near-field samples (noisier corner detection at
            #      the edges of the FOV).
            msg = self.detected_pose
            fresh = False
            outliers_rejected = 0
            if msg is not None and not in_visual_servo:
                stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
                if stamp_ns != last_stamp_ns:
                    age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
                    if age < self.detection_max_age:
                        new_x = msg.pose.position.x
                        new_y = msg.pose.position.y
                        sample_offset = math.hypot(new_x - avg.x, new_y - avg.y)
                        is_outlier = (
                            self.refinement_outlier_threshold > 0.0
                            and sample_offset > self.refinement_outlier_threshold
                        )
                        if is_outlier:
                            outliers_rejected += 1
                            self.get_logger().warning(
                                f'   refinement outlier rejected '
                                f'(offset={sample_offset*100:.1f}cm > '
                                f'{self.refinement_outlier_threshold*100:.0f}cm)'
                            )
                        else:
                            # Linear distance weight, clamped to [min, 1.0].
                            weight = max(
                                self.refinement_weight_min,
                                min(1.0, distance_to_avg
                                    / self.refinement_weight_full_distance),
                            )
                            avg.update(
                                new_x, new_y,
                                msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z, msg.pose.orientation.w,
                                weight=weight,
                            )
                            fresh = True
                            samples_in_phase4 += 1
                    last_stamp_ns = stamp_ns

            if not in_visual_servo:
                # Detection-tracking state machine only matters in the
                # line-tracking regime. In visual-servo we use the live
                # camera lookup instead.
                if not fresh and had_detection:
                    had_detection = False
                    self.get_logger().info(
                        f'   tag lost — continuing on last-known line '
                        f'(refined with {samples_in_phase4} extra samples '
                        f'during Phase 4)'
                    )
                elif fresh and not had_detection:
                    had_detection = True
                    self.get_logger().info('   tag reacquired — resuming line refinement')

            perp_yaw = avg.perpendicular_yaw(rx, ry)
            if perp_yaw is None:
                # Should never happen — avg always has the 40 Phase-2 samples
                # by the time we get here. Defensive guard only.
                self.get_logger().warning('   lost tag estimate — holding')
                self._publish_cmd_vel(0.0, 0.0)
                time.sleep(period)
                continue

            lateral = avg.signed_lateral_offset(rx, ry, perp_yaw)
            distance = math.hypot(avg.x - rx, avg.y - ry)

            if distance <= self.docking_distance:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(
                    f'   reached: d_to_tag={distance:.3f}m, lateral='
                    f'{lateral*100:+.1f}cm ({avg.count} total samples '
                    f'averaged, {samples_in_phase4} during Phase 4)'
                )
                return True

            travelled = math.hypot(rx - x0, ry - y0)
            if travelled > max_travel:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().error(
                    f'   exceeded travel safety ({travelled:.2f}m > '
                    f'{max_travel:.2f}m)'
                )
                return False

            # Heading control: line-tracking (far) → visual servo (near).
            if in_visual_servo:
                # Camera-frame closed-loop. omega keeps the tag centred
                # in the image. The image_angle is low-pass filtered to
                # reject single-frame solvePnP noise spikes that would
                # otherwise produce one bad correction per noisy frame
                # (and force the robot to "un-correct" the next frame).
                tag_cam = self.lookup_tag_in_camera_optical()
                if tag_cam is not None:
                    tx_cam, _, tz_cam = tag_cam
                    if tz_cam > 0.0:
                        raw_angle = math.atan2(tx_cam, tz_cam)
                        if filtered_image_angle is None:
                            # First valid sample seeds the filter.
                            filtered_image_angle = raw_angle
                        else:
                            a = self.visual_servo_filter_alpha
                            filtered_image_angle = (
                                a * raw_angle
                                + (1.0 - a) * filtered_image_angle
                            )
                        omega = -self.visual_servo_kp * filtered_image_angle
                    else:
                        # Tag behind the camera (shouldn't happen here).
                        omega = 0.0
                else:
                    # No fresh TF — drive straight rather than blind-steer.
                    omega = 0.0
            else:
                # Line-tracking pure-pursuit on the perpendicular line
                # through the running-average tag centre.
                desired_yaw = normalize_angle(
                    perp_yaw - math.atan2(lateral, self.line_lookahead_distance))
                yaw_err = normalize_angle(desired_yaw - ryaw)
                omega = self.line_yaw_kp * yaw_err

            omega = max(-self.drive_yaw_max_omega,
                        min(self.drive_yaw_max_omega, omega))

            v = self.drive_speed
            taper = 2.0 * self.docking_distance
            if distance < taper:
                v = max(0.03, self.drive_speed *
                        (distance - self.docking_distance) /
                        (taper - self.docking_distance))

            self._publish_cmd_vel(v, omega)
            time.sleep(period)

        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().error('   line-tracking advance timeout')
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

    def _publish_cmd_vel(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_vel_pub.publish(msg)

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

    def lookup_tag_in_camera_optical(self):
        """Return the tag's 3D position (x, y, z) in camera_optical_frame,
        or None if the TF is not available or stale.

        This is the right quantity for true visual centring: in optical
        frame, +X is image-right and +Z is forward (into the scene). The
        horizontal angular offset from the image centre is atan2(x, z).
        Using this bypasses the map-frame solvePnP bias and centres the
        tag as it actually appears in the camera image.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_optical_frame', 'charging_dock_apriltag',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None
        return (t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z)

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
