# SPDX-License-Identifier: MIT
"""Station pose from marker tags, covariance and goal correction (openamrobot_station_alignment.geometry)."""
import math
import random

import pytest

from openamrobot_station_alignment import geometry as g

LAYOUT = g.Layout({3: -0.10, 4: 0.10}, standoff=0.713)


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def tags_seen_from(robot_in_station, height=0.32, ids=(3, 4)):
    """Tag centres and rotations in the robot frame, for a robot at `robot_in_station` (station frame pose)."""
    # Station frame: station at origin facing +x; plate origin at (+standoff, 0); shelf axis u = +y.
    base_from_station = g.inverse(robot_in_station)
    out = {}
    for tag in ids:
        sx, sy = LAYOUT.standoff, LAYOUT.tag_offsets[tag]
        x, y, _ = g.compose(base_from_station, (sx, sy, 0.0))
        # Tag z axis (face normal) points from the plate towards the robot: -x of the station frame.
        yaw = base_from_station[2]
        nx, ny = -math.cos(yaw), -math.sin(yaw)
        rotation = ((0.0, 0.0, nx), (0.0, 0.0, ny), (0.0, 0.0, 0.0))
        out[tag] = ((x, y, height), rotation)
    return out


def test_wrap_compose_inverse():
    assert g.wrap(3 * math.pi) == pytest.approx(math.pi)
    assert g.wrap(-math.pi) == pytest.approx(math.pi)
    a, b = (1.0, -2.0, 0.7), (0.3, 0.4, -1.9)
    assert close(g.compose(a, g.inverse(a)), (0.0, 0.0, 0.0), 1e-12)
    assert close(g.between(a, g.compose(a, b)), b, 1e-12)


def test_robot_at_the_station_sees_the_station_at_its_origin():
    pose, mode = g.station_from_tags(LAYOUT, tags_seen_from((0.0, 0.0, 0.0)))
    assert mode == "baseline"
    assert close(pose, (0.0, 0.0, 0.0), 1e-12)


@pytest.mark.parametrize("offset", [(0.05, 0.0, 0.0), (-0.04, 0.03, 0.0), (0.0, 0.0, 0.08), (0.03, -0.05, -0.1)])
def test_baseline_estimate_is_the_inverse_of_the_robot_offset(offset):
    pose, _ = g.station_from_tags(LAYOUT, tags_seen_from(offset))
    assert close(pose, g.inverse(offset), 1e-9)


@pytest.mark.parametrize("tag", [3, 4])
def test_single_tag_uses_its_face_normal(tag):
    offset = (0.03, -0.02, 0.06)
    pose, mode = g.station_from_tags(LAYOUT, tags_seen_from(offset, ids=(tag,)))
    assert mode == "single"
    assert close(pose, g.inverse(offset), 1e-9)


def test_single_tag_normal_sign_does_not_matter():
    tags = tags_seen_from((0.02, 0.01, 0.05), ids=(3,))
    position, rotation = tags[3]
    flipped = tuple(tuple(-v for v in row) for row in rotation)
    a, _ = g.station_from_tags(LAYOUT, tags)
    b, _ = g.station_from_tags(LAYOUT, {3: (position, flipped)})
    assert close(a, b, 1e-12)


def test_unknown_tags_are_ignored_and_none_is_an_error():
    tags = tags_seen_from((0.0, 0.0, 0.0))
    tags[7] = ((5.0, 5.0, 0.0), None)
    pose, mode = g.station_from_tags(LAYOUT, tags)
    assert mode == "baseline" and close(pose, (0.0, 0.0, 0.0), 1e-12)
    with pytest.raises(ValueError):
        g.station_from_tags(LAYOUT, {7: ((1.0, 0.0, 0.0), None)})


def test_baseline_yaw_error_is_small_for_millimetre_tag_noise():
    rng = random.Random(1)
    worst = 0.0
    for _ in range(200):
        tags = tags_seen_from((0.0, 0.0, 0.0))
        noisy = {i: ((p[0] + rng.gauss(0, 0.002), p[1] + rng.gauss(0, 0.002), p[2]), r) for i, (p, r) in tags.items()}
        pose, _ = g.station_from_tags(LAYOUT, noisy)
        worst = max(worst, abs(pose[2]))
    # 0.20 m baseline, 2 mm per tag: sqrt(2) * 0.002 / 0.20 = 0.014 rad standard deviation of the yaw from one frame;
    # the worst of 200 frames stays inside the 0.08 rad budget. The alignment averages 8 frames.
    assert worst < 0.06


def test_corrected_goal_is_the_true_station_when_the_measurement_is_exact():
    true_station = (0.0, -2.085, -math.pi / 2)
    robot_true = g.compose(true_station, (-0.03, 0.02, 0.04))
    # AMCL's error is a map-frame offset (a wrong map -> odom): the map pose it reports is error o truth.
    amcl_error = (0.045, -0.02, 0.01)
    robot_amcl = g.compose(amcl_error, robot_true)
    measured = g.between(robot_true, true_station)  # what the camera sees: station in the robot frame
    goal = g.corrected_goal(robot_amcl, measured)
    # Driving until AMCL reports `goal` puts the true robot exactly on the station while the AMCL error stays.
    reached_true = g.compose(g.inverse(amcl_error), goal)
    assert close(reached_true, true_station, 1e-9)
    # The nominal goal (#164 alone) leaves the AMCL error on the station.
    nominal_true = g.compose(g.inverse(amcl_error), true_station)
    assert g.pose_error(nominal_true, true_station)[0] > 0.04


def test_covariance_layout_and_floor():
    poses = [(0.0, 0.0, 0.0)] * 5
    cov = g.covariance(poses, floor_xy=0.003, floor_yaw=0.005)
    assert len(cov) == 36
    assert cov[0] == pytest.approx(0.003 ** 2) and cov[7] == pytest.approx(0.003 ** 2)
    assert cov[35] == pytest.approx(0.005 ** 2)
    assert cov[14] == cov[21] == cov[28] == 1e6
    spread = [(0.01, 0.0, 0.0), (-0.01, 0.0, 0.0)]
    assert g.covariance(spread, 0.0, 0.0)[0] == pytest.approx(0.0002)


def test_mean_pose_wraps_yaw():
    m = g.mean_pose([(0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01)])
    assert abs(g.wrap(m[2] - math.pi)) < 1e-9


def test_accepted_is_per_axis():
    assert g.accepted((0.03, -0.02, 0.06), 0.03, 0.02, 0.06)
    assert not g.accepted((0.031, 0.0, 0.0), 0.03, 0.02, 0.06)
    assert not g.accepted((0.0, 0.021, 0.0), 0.03, 0.02, 0.06)
    assert not g.accepted((0.0, 0.0, -0.061), 0.03, 0.02, 0.06)


def test_within_and_pose_error():
    assert g.within((0.02, 0.01, 0.04), 0.025, 0.05)
    assert not g.within((0.02, 0.02, 0.0), 0.025, 0.05)
    assert not g.within((0.0, 0.0, 0.06), 0.025, 0.05)
    exy, eyaw, along, left = g.pose_error((1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))
    assert exy == pytest.approx(1.0) and eyaw == pytest.approx(math.pi / 2)
    assert along == pytest.approx(0.0, abs=1e-12) and left == pytest.approx(-1.0)
