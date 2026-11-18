# SPDX-License-Identifier: MIT
"""Station correction: positions in odom with Nav2 translations, rotations closed on the map heading
(openamrobot_station_alignment.maneuver).

Robot model from the CI runs:
- a rotation stopped on odometry turns `coast` further than odometry sees (run 44225167178, rtf1: Spin +0.082 rad,
  odom +0.110, ground truth and marker about +0.23); a rotation closed on the map heading (AMCL from the scan) ends
  within `map_rotation_error` of its target;
- a translation goes `drive_over` beyond its distance along the true heading; odometry follows it.
Reproduces run 44188740454 rtf1 trial 4 (map-frame goals, 0.0485 m after 3 corrections) and run 44225167178 rtf1
trials 2 and 5 (odometry-stopped spins, 0.105 and 0.098 rad).
"""
import itertools
import math

import pytest

from openamrobot_station_alignment import geometry as g
from openamrobot_station_alignment import maneuver as m

ACCEPT = (0.025, 0.02, 0.06)           # config/station_alignment.yaml
STATION = (0.0, -2.085, -math.pi / 2)


class Robot:
    def __init__(self, truth, coast=0.14, map_rotation_error=0.03, drive_over=0.003, amcl_yaw_error=0.02):
        self.truth = truth
        self.odom = truth             # odom starts on truth; only its heading drifts
        self.coast, self.map_error, self.drive_over = coast, map_rotation_error, drive_over
        self.amcl_yaw_error = amcl_yaw_error

    def map_yaw(self):
        return g.wrap(self.truth[2] + self.amcl_yaw_error)

    def rotate_closed_on_map(self, delta):
        target = g.wrap(self.map_yaw() + delta)
        reached = g.wrap(target + math.copysign(self.map_error, delta))        # true heading it stops at, + AMCL
        turned = g.wrap(reached - self.map_yaw())
        self.truth = (self.truth[0], self.truth[1], g.wrap(self.truth[2] + turned))
        self.odom = (self.odom[0], self.odom[1], g.wrap(self.odom[2] + turned - math.copysign(self.coast, delta)))

    def spin_on_odom(self, delta):
        """behavior_server Spin: stops when odometry has turned `delta`; the base turns `coast` further."""
        self.odom = (self.odom[0], self.odom[1], g.wrap(self.odom[2] + delta))
        self.truth = (self.truth[0], self.truth[1], g.wrap(self.truth[2] + delta + math.copysign(self.coast, delta)))

    def translate(self, distance):
        d = distance + math.copysign(self.drive_over, distance)
        self.truth = g.compose(self.truth, (d, 0.0, 0.0))
        self.odom = g.compose(self.odom, (d, 0.0, 0.0))


def align(robot, iterations=3, rotation="map"):
    """The alignment loop: the marker measures the station in the true robot frame; correct; measure again."""
    for it in range(iterations + 1):
        residual = g.between(robot.truth, STATION)
        if g.accepted(residual, *ACCEPT):
            return it
        if it == iterations:
            return None
        plan = m.Plan(m.target_in_odom(robot.odom, residual), m.target_yaw_in_map(robot.map_yaw(), residual),
                      residual, lateral_direct=ACCEPT[1], backoff=0.15)
        while (step := plan.next(robot.odom, robot.map_yaw())) is not None:
            if step[0] == m.ROTATE:
                (robot.rotate_closed_on_map if rotation == "map" else robot.spin_on_odom)(step[1])
            elif step[0] == m.DRIVE:
                robot.translate(step[1])
            else:
                robot.translate(-step[1])
    return None


def start(residual):
    """True robot pose whose measured station residual is `residual` (distance, left, yaw)."""
    return g.compose(STATION, g.inverse(residual))


def test_rtf1_trial_4_of_run_44188740454_converges():
    robot = Robot(start((0.045, 0.033, -0.019)))
    assert align(robot) is not None
    exy, eyaw, _, _ = g.pose_error(robot.truth, STATION)
    assert exy <= 0.04 and eyaw <= 0.08


@pytest.mark.parametrize("residual", [(0.018, 0.011, 0.082), (0.003, -0.028, 0.01)])
def test_rtf1_trials_2_and_5_of_run_44225167178(residual):
    # stopped on odometry (behavior_server Spin): the yaw never settles
    assert align(Robot(start(residual)), rotation="odom") is None
    # closed on the map heading: aligned
    robot = Robot(start(residual))
    assert align(robot) is not None
    assert g.accepted(g.between(robot.truth, STATION), *ACCEPT)


GRID = list(itertools.product((-0.06, 0.03, 0.06), (-0.05, -0.025, 0.0, 0.035), (-0.08, 0.0, 0.07)))


@pytest.mark.parametrize("dx,dy,dyaw", GRID)
def test_every_start_within_the_errors_seen_converges(dx, dy, dyaw):
    robot = Robot(g.compose(STATION, (dx, dy, dyaw)))
    assert align(robot) is not None, g.between(robot.truth, STATION)


def test_lateral_residual_uses_a_short_reverse_and_small_turns():
    residual = (0.01, 0.035, 0.0)
    robot = Robot((0.0, 0.0, 0.0), coast=0.0, map_rotation_error=0.0, drive_over=0.0, amcl_yaw_error=0.0)
    plan = m.Plan(m.target_in_odom(robot.odom, residual), m.target_yaw_in_map(0.0, residual), residual, 0.02, 0.15)
    steps = []
    while (step := plan.next(robot.odom, robot.map_yaw())) is not None:
        steps.append(step)
        if step[0] == m.ROTATE:
            robot.rotate_closed_on_map(step[1])
        else:
            robot.translate(step[1] if step[0] == m.DRIVE else -step[1])
    assert [s[0] for s in steps] == [m.BACKUP, m.ROTATE, m.DRIVE, m.ROTATE]
    assert steps[0][1] == 0.15 and all(abs(s[1]) < 0.3 for s in steps if s[0] == m.ROTATE)
    assert g.pose_error(robot.truth, residual)[0] < 1e-9


def test_small_lateral_residual_drives_along_the_heading_only():
    residual = (0.025, 0.01, 0.04)
    plan = m.Plan(residual, 0.04, residual, lateral_direct=0.02, backoff=0.15)
    assert plan.next((0.0, 0.0, 0.0), 0.0) == (m.DRIVE, pytest.approx(0.025))
    assert plan.next((0.025, 0.0, 0.0), 0.0) == (m.ROTATE, pytest.approx(0.04), pytest.approx(0.04))


def test_the_yaw_target_is_taken_from_map_not_odom():
    plan = m.Plan((0.0, 0.0, 0.0), 0.3, (0.0, 0.0, 0.3), lateral_direct=0.02, backoff=0.15)
    # odometry says the yaw is reached, the map heading says 0.1 rad are still missing
    assert plan.next((0.0, 0.0, 0.3), 0.2) == (m.ROTATE, pytest.approx(0.1), pytest.approx(0.3))


def test_an_amcl_update_moves_a_map_goal_but_not_an_odom_target():
    residual = (0.045, 0.03, 0.0)
    target = m.target_in_odom((0.0, 0.0, 0.0), residual)
    jump = (0.02, -0.015, 0.0)  # map -> odom changes by this during the move (AMCL update)
    map_goal_in_odom_after_jump = g.compose(g.inverse(jump), target)
    assert g.pose_error(map_goal_in_odom_after_jump, target)[0] == pytest.approx(math.hypot(0.02, 0.015))
    plan = m.Plan(target, 0.0, residual, lateral_direct=0.02, backoff=0.15)
    assert plan.target == target
