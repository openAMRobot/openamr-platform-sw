# SPDX-License-Identifier: MIT
"""Station correction as a short sequence of forward-only Nav2 goals, planned in the odom frame. No ROS import.

Forward only (software lead's review of aa8fd11, item a): D-19 (8 Nov) allows no reverse in Build 1 because the rear
sectors are masked, and platform-sw #90 removes backup, drive_on_heading and assisted_teleop from behavior_server and
clamps min_vel_x to 0. The correction therefore never reverses and uses no behavior_server action:

- target ahead and lateral residual within `lateral_direct`: drive forward along the heading, turn to the target yaw;
- otherwise (lateral residual above `lateral_direct`, or the target behind the robot): turn to face the target point,
  drive forward to it, turn to the target yaw.

Every primitive is a NavigateToPose goal of #164's station tree: a rotation is a rotation-only goal (the current map
position, the absolute target heading), a drive is a goal straight ahead on the current map heading. Why the yaw is
closed on the map heading: the sim base keeps turning after the wheels stop, so odometry misses part of a turn
(run 44225167178, rtf1: a Spin of +0.082 rad gave +0.110 rad in odom and about +0.23 rad by ground truth); AMCL takes
the heading from the scan. The position target is fixed in odom when the marker is measured (robot stopped), so an
AMCL update between the measurement and the move does not move it; each drive is planned from the current pose. The
marker is measured again afterwards; nothing here decides acceptance.
"""
import math

from openamrobot_station_alignment import geometry

ROTATE, DRIVE = "rotate", "drive"


class Plan:
    """Stages towards an odom-frame target, forward motion only (module docstring)."""

    def __init__(self, target_odom, target_yaw_map, residual, lateral_direct, min_move=0.003, min_turn=0.01,
                 yaw_deadband=0.02):
        self.target = target_odom
        self.target_yaw_map = target_yaw_map
        self.min_move, self.min_turn, self.yaw_deadband = min_move, min_turn, yaw_deadband
        self.turn_first = abs(residual[1]) > lateral_direct or residual[0] < -min_move
        self.stages = ["face", "approach", "align"] if self.turn_first else ["approach", "align"]

    def next(self, current_odom, current_yaw_map):
        """The primitive to run now, or None when the plan is done. Positions from odom, yaw from map.
        (DRIVE, distance > 0) or (ROTATE, delta, absolute target yaw in map): the goal sent is the absolute yaw, so
        an AMCL update between this read and the goal does not move the target (review of aa8fd11, point 4)."""
        while self.stages:
            stage = self.stages.pop(0)
            step = self._step(stage, current_odom, current_yaw_map)
            if step is not None:
                return step
        return None

    def _step(self, stage, current, yaw_map):
        rel = geometry.between(current, self.target)
        if stage == "face":
            angle = math.atan2(rel[1], rel[0]) if math.hypot(rel[0], rel[1]) > self.min_move else 0.0
            return self._turn(angle, geometry.wrap(yaw_map + angle))
        if stage == "approach":
            # forward along the current heading only; never a reverse. A target behind or beside the robot was
            # faced first; a remainder is left to the next marker measurement.
            return (DRIVE, rel[0]) if rel[0] >= self.min_move else None
        if stage == "align":
            delta = geometry.wrap(self.target_yaw_map - yaw_map)
            return None if abs(delta) <= self.yaw_deadband else self._turn(delta, self.target_yaw_map)
        raise ValueError(stage)

    def _turn(self, angle, target_yaw_map):
        return None if abs(angle) < self.min_turn else (ROTATE, angle, target_yaw_map)


def forward_goal(map_pose, distance, lead):
    """Map goal of a forward drive of `distance` from `map_pose`, on its heading. The goal lies `lead` beyond the
    point: #164's station_goal_checker ends the goal as soon as the base is within its xy tolerance (0.025 m), so a
    goal at the point itself would stop the base that much short of it, and a move shorter than the tolerance would
    end at once. With `lead` equal to the tolerance the base stops at the point."""
    if distance <= 0.0:
        raise ValueError(f"forward drive only, got {distance}")
    x, y, yaw = map_pose
    reach = distance + lead
    return (x + reach * math.cos(yaw), y + reach * math.sin(yaw), yaw)


def target_in_odom(odom_from_base, residual):
    """The station in odom: where the robot is in odom at the measurement, composed with the measured station."""
    return geometry.compose(odom_from_base, residual)


def target_yaw_in_map(map_yaw, residual):
    """The station heading in map: the AMCL heading at the measurement plus the measured yaw residual."""
    return geometry.wrap(map_yaw + residual[2])
