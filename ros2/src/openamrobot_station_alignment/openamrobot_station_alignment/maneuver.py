# SPDX-License-Identifier: MIT
"""Station correction as a short sequence of Nav2 behaviors, planned in the odom frame. No ROS import.

Why (Proposed, 18 Nov 2026):
- A corrected NavigateToPose goal is planned in map and followed in odom through the current map -> odom, so every
  AMCL update during the move shifts the target (AMCL updates every 0.05 m or 0.05 rad since #164).
- #164's StationApproach controller (nav2_graceful_controller) has initial_rotation and allow_backward both true;
  the controller then disables allow_backward ("setting allow backward to false"), so a goal behind the robot was
  driven as a turn-around, out and back (run 44164871954 / 44188740454 rtf1 trial 4: 3 corrections, 0.0485 m).
- The progress checker (0.05 m in 12 s) aborts short moves that are mostly rotation.

- behavior_server's Spin stops on the odom heading, and the base keeps turning after the wheels stop: in run
  44225167178 (rtf1) a Spin of +0.082 rad gave +0.110 rad in odom and about +0.23 rad by ground truth and marker.

The position target is fixed in odom when the marker is measured (robot stopped): odom <- base at that time composed
with the measured station. Translations go along the heading with behavior_server's DriveOnHeading and BackUp
(local frame odom, no progress checker). Rotations are closed on the map heading, which AMCL takes from the scan and
which therefore sees the turn odometry misses: the yaw target is the AMCL yaw plus the measured yaw residual, and a
rotation is a rotation-only goal (position = the current one) of #164's station tree. Each step is planned from the
current pose, so an error of one step is taken up by the next. The marker is measured again afterwards; nothing here
decides acceptance.
"""
import math

from openamrobot_station_alignment import geometry

BACKUP, ROTATE, DRIVE = "backup", "rotate", "drive"


class Plan:
    """Stages towards an odom-frame target. Lateral residuals above `lateral_direct` use a short reverse first:
    back up `backoff`, turn to face the target point, drive to it, turn to the target yaw. Otherwise: drive along
    the heading to the target's distance, turn to the target yaw."""

    def __init__(self, target_odom, target_yaw_map, residual, lateral_direct, backoff, min_move=0.003, min_turn=0.01,
                 yaw_deadband=0.02):
        self.target = target_odom
        self.target_yaw_map = target_yaw_map
        self.backoff = backoff
        self.min_move, self.min_turn, self.yaw_deadband = min_move, min_turn, yaw_deadband
        self.lateral = abs(residual[1]) > lateral_direct
        self.stages = ["backoff", "face", "approach", "align"] if self.lateral else ["approach", "align"]

    def next(self, current_odom, current_yaw_map):
        """(primitive, magnitude) to run now, or None when the plan is done. Positions from odom, yaw from map.
        A rotation is (ROTATE, delta, absolute target yaw in map): the goal sent is the absolute yaw, so an AMCL update
        between this read and the goal does not move the target (Kunal's review of aa8fd11, point 4)."""
        while self.stages:
            stage = self.stages.pop(0)
            step = self._step(stage, current_odom, current_yaw_map)
            if step is not None:
                return step
        return None

    def _step(self, stage, current, yaw_map):
        rel = geometry.between(current, self.target)
        if stage == "backoff":
            return (BACKUP, self.backoff)
        if stage == "face":
            angle = math.atan2(rel[1], rel[0]) if math.hypot(rel[0], rel[1]) > self.min_move else 0.0
            return self._turn(angle, geometry.wrap(yaw_map + angle))
        if stage == "approach":
            # along the current heading only; a lateral remainder is left to the next marker measurement
            if abs(rel[0]) < self.min_move:
                return None
            return (DRIVE, rel[0]) if rel[0] > 0 else (BACKUP, -rel[0])
        if stage == "align":
            delta = geometry.wrap(self.target_yaw_map - yaw_map)
            return None if abs(delta) <= self.yaw_deadband else self._turn(delta, self.target_yaw_map)
        raise ValueError(stage)

    def _turn(self, angle, target_yaw_map):
        return None if abs(angle) < self.min_turn else (ROTATE, angle, target_yaw_map)


def behavior_allowance(distance, speed, factor=2.0, margin_s=5.0):
    """Time allowance of a BackUp or DriveOnHeading of `distance` at `speed`: twice the nominal time plus a margin
    (0.15 m at 0.05 m/s: 11 s), instead of a fixed 30 s (Kunal's review of aa8fd11, point 5)."""
    return factor * abs(distance) / max(abs(speed), 1e-6) + margin_s


class ProgressWatch:
    """Stall detection of a translation on odometry, on simulation time: stalled when the base has moved less than
    `min_move` in the last `window_s`. Defaults 0.005 m in 4 s: at 0.05 m/s the base covers 0.2 m in 4 s, so this
    only fires when it is held; the approach's progress checker allows 12 s for 0.05 m."""

    def __init__(self, start_pose, start_ns, window_s=4.0, min_move=0.005):
        self.ref, self.ref_ns = start_pose, start_ns
        self.window_ns, self.min_move = int(window_s * 1e9), min_move

    def update(self, pose, now_ns):
        if math.hypot(pose[0] - self.ref[0], pose[1] - self.ref[1]) >= self.min_move:
            self.ref, self.ref_ns = pose, now_ns
            return False
        return now_ns - self.ref_ns > self.window_ns


def target_in_odom(odom_from_base, residual):
    """The station in odom: where the robot is in odom at the measurement, composed with the measured station."""
    return geometry.compose(odom_from_base, residual)


def target_yaw_in_map(map_yaw, residual):
    """The station heading in map: the AMCL heading at the measurement plus the measured yaw residual."""
    return geometry.wrap(map_yaw + residual[2])
