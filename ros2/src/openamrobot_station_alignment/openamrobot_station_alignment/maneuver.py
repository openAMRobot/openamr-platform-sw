# SPDX-License-Identifier: MIT
"""Station correction as a short sequence of Nav2 behaviors, planned in the odom frame. No ROS import.

Why (Proposed, 18 Nov 2026):
- A corrected NavigateToPose goal is planned in map and followed in odom through the current map -> odom, so every
  AMCL update during the move shifts the target (AMCL updates every 0.05 m or 0.05 rad since #164).
- #164's StationApproach controller (nav2_graceful_controller) has initial_rotation and allow_backward both true;
  the controller then disables allow_backward ("setting allow backward to false"), so a goal behind the robot was
  driven as a turn-around, out and back (run 44164871954 / 44188740454 rtf1 trial 4: 3 corrections, 0.0485 m).
- The progress checker (0.05 m in 12 s) aborts short moves that are mostly rotation.

The target is fixed in odom when the marker is measured (robot stopped): odom <- base at that time composed with
the measured station. It is then reached with behavior_server's BackUp, Spin and DriveOnHeading, which run in its
local frame (odom), publish on cmd_vel_nav like every Nav2 command, and use no progress checker. Each step is
planned from the current odom pose, so an overshoot of one step is taken up by the next. The marker is measured
again afterwards; nothing here decides acceptance.
"""
import math

from openamrobot_station_alignment import geometry

BACKUP, SPIN, DRIVE = "backup", "spin", "drive"


class Plan:
    """Stages towards an odom-frame target. Lateral residuals above `lateral_direct` use a short reverse first:
    back up `backoff`, turn to face the target point, drive to it, turn to the target yaw. Otherwise: drive along
    the heading to the target's distance, turn to the target yaw."""

    def __init__(self, target_odom, residual, lateral_direct, backoff, min_move=0.003, min_turn=0.01,
                 yaw_deadband=0.02, spin_overshoot=0.0):
        self.target = target_odom
        self.backoff = backoff
        self.min_move, self.min_turn, self.yaw_deadband = min_move, min_turn, yaw_deadband
        self.lateral = abs(residual[1]) > lateral_direct
        self.stages = ["backoff", "face", "approach", "align"] if self.lateral else ["approach", "align"]
        self.spin_overshoot = spin_overshoot
        self.spins = []

    def next(self, current_odom):
        """(primitive, magnitude) to run now from `current_odom`, or None when the plan is done."""
        while self.stages:
            stage = self.stages.pop(0)
            step = self._step(stage, current_odom)
            if step is not None:
                return step
        return None

    def _step(self, stage, current):
        rel = geometry.between(current, self.target)
        if stage == "backoff":
            return (BACKUP, self.backoff)
        if stage == "face":
            return self._spin(math.atan2(rel[1], rel[0]) if math.hypot(rel[0], rel[1]) > self.min_move else 0.0)
        if stage == "approach":
            # along the current heading only; a lateral remainder is left to the next marker measurement
            if abs(rel[0]) < self.min_move:
                return None
            return (DRIVE, rel[0]) if rel[0] > 0 else (BACKUP, -rel[0])
        if stage == "align":
            if abs(rel[2]) <= self.yaw_deadband:
                return None
            return self._spin(rel[2])
        raise ValueError(stage)

    def _spin(self, angle):
        """Spin by `angle`, shortened by the overshoot observed so far in this plan."""
        command = math.copysign(max(abs(angle) - self.spin_overshoot, 0.0), angle)
        if abs(command) < self.min_turn:
            return None
        return (SPIN, command)

    def observe_spin(self, commanded, achieved):
        """Record a spin: the overshoot (achieved beyond commanded, in its direction) shortens the next ones."""
        self.spins.append((commanded, achieved))
        over = [math.copysign(1.0, c) * a - abs(c) for c, a in self.spins]
        self.spin_overshoot = max(sum(over) / len(over), 0.0)


def target_in_odom(odom_from_base, residual):
    """The station in odom: where the robot is in odom at the measurement, composed with the measured station."""
    return geometry.compose(odom_from_base, residual)
