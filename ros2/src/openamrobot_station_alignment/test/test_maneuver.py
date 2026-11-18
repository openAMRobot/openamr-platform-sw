# SPDX-License-Identifier: MIT
"""Odom-frame station correction with Nav2 behaviors (openamrobot_station_alignment.maneuver).

Reproduces the rtf1 trial 4 failure of run 44188740454 (lateral residual 0.033 to 0.040 m, three corrections, final
0.0485 m) and the effect of an AMCL update during a map-frame correction.
"""
import itertools
import math

import pytest

from openamrobot_station_alignment import geometry as g
from openamrobot_station_alignment import maneuver as m

ACCEPT = (0.03, 0.02, 0.06)            # config/station_alignment.yaml, unchanged
STATION = (0.0, -2.085, -math.pi / 2)


def execute(pose, step, spin_over=0.05, drive_over=0.003):
    """Robot model: a spin goes `spin_over` beyond its command, a drive `drive_over` beyond (both odometry exact)."""
    kind, value = step
    if kind == m.SPIN:
        achieved = value + math.copysign(spin_over, value)
        return g.compose(pose, (0.0, 0.0, achieved)), achieved
    d = value + drive_over if kind == m.DRIVE else -(value + drive_over)
    return g.compose(pose, (d, 0.0, 0.0)), d


def align(start_true, amcl_error=(0.0, 0.0, 0.0), iterations=3, **robot):
    """The alignment loop: measure the station in the robot frame (exact marker), accept or correct in odom."""
    pose = start_true            # odom = world here; AMCL is not used by the correction
    overshoot = 0.0
    for it in range(iterations + 1):
        residual = g.between(pose, STATION)
        if g.accepted(residual, *ACCEPT):
            return pose, it
        if it == iterations:
            return pose, None
        plan = m.Plan(m.target_in_odom(pose, residual), residual, lateral_direct=ACCEPT[1], backoff=0.15,
                      spin_overshoot=overshoot)
        while True:
            step = plan.next(pose)
            if step is None:
                break
            pose, achieved = execute(pose, step, **robot)
            if step[0] == m.SPIN:
                plan.observe_spin(step[1], achieved)
        overshoot = plan.spin_overshoot
    return pose, None


def test_trial_4_of_run_44188740454_converges_in_odom():
    # residual of the first correction there: 0.045 towards the shelf, 0.033 left, -0.019 rad
    start = g.compose(STATION, g.inverse((0.045, 0.033, -0.019)))
    pose, iterations = align(start)
    exy, eyaw, _, _ = g.pose_error(pose, STATION)
    assert iterations is not None and iterations <= 3
    assert exy <= 0.04 and eyaw <= 0.08


GRID = list(itertools.product((-0.06, 0.03, 0.06), (-0.05, -0.025, 0.0, 0.035), (-0.07, 0.0, 0.05)))


@pytest.mark.parametrize("dx,dy,dyaw", GRID)
def test_every_start_within_the_amcl_error_seen_converges(dx, dy, dyaw):
    start = g.compose(STATION, (dx, dy, dyaw))
    pose, iterations = align(start)
    assert iterations is not None, g.between(pose, STATION)
    assert g.accepted(g.between(pose, STATION), *ACCEPT)


def test_lateral_residual_uses_a_short_reverse_and_small_turns():
    residual = (0.01, 0.035, 0.0)
    plan = m.Plan(m.target_in_odom((0.0, 0.0, 0.0), residual), residual, lateral_direct=0.02, backoff=0.15)
    pose, steps = (0.0, 0.0, 0.0), []
    while (step := plan.next(pose)) is not None:
        steps.append(step)
        pose, _ = execute(pose, step, spin_over=0.0, drive_over=0.0)
    assert [s[0] for s in steps] == [m.BACKUP, m.SPIN, m.DRIVE, m.SPIN]
    assert steps[0][1] == 0.15 and all(abs(s[1]) < 0.3 for s in steps if s[0] == m.SPIN)
    assert g.pose_error(pose, residual)[0] < 1e-9


def test_small_lateral_residual_drives_along_the_heading_only():
    residual = (0.025, 0.01, 0.04)
    plan = m.Plan(residual, residual, lateral_direct=0.02, backoff=0.15)
    assert plan.next((0.0, 0.0, 0.0)) == (m.DRIVE, pytest.approx(0.025))
    assert plan.next((0.025, 0.0, 0.0))[0] == m.SPIN


def test_spin_overshoot_is_learned_and_subtracted():
    plan = m.Plan((0.0, 0.0, 0.2), (0.0, 0.0, 0.2), lateral_direct=0.02, backoff=0.15)
    plan.observe_spin(0.2, 0.25)
    assert plan.spin_overshoot == pytest.approx(0.05)
    assert plan._spin(0.2) == (m.SPIN, pytest.approx(0.15))
    assert plan._spin(0.04) is None          # shorter than the overshoot: left to the next measurement


def test_an_amcl_update_moves_a_map_goal_but_not_an_odom_target():
    residual = (0.045, 0.03, 0.0)
    robot_odom = (0.0, 0.0, 0.0)
    target = m.target_in_odom(robot_odom, residual)
    jump = (0.02, -0.015, 0.0)  # map -> odom changes by this during the move (AMCL update)
    # map-frame goal (the previous correction): the controller follows it through the new map -> odom
    map_goal_in_odom_after_jump = g.compose(g.inverse(jump), target)
    assert g.pose_error(map_goal_in_odom_after_jump, target)[0] == pytest.approx(math.hypot(0.02, 0.015))
    # odom target: unchanged by construction; the plan never reads map
    plan = m.Plan(target, residual, lateral_direct=0.02, backoff=0.15)
    assert plan.target == target
