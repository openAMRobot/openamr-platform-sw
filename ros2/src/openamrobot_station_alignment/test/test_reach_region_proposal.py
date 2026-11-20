# SPDX-License-Identifier: MIT
"""Proposed reach region for the station trials verdict (review of #167). Proposed, not adopted: evaluation.py still
uses REACH_XY 0.04 m and REACH_YAW 0.08 rad, and this file changes no threshold.

The scn04 straight-reach boundary comes from the reach model in fixtures/station_reach_region.json: the most negative
along-heading error (base too far back) at which the reach still completes, for left -0.04..0.04 m and yaw
-0.08..0.08 rad. It shrinks with positive yaw (base yaw minus station yaw, counter-clockwise), grows with negative
yaw and moves at most 5 mm with the lateral offset. The current budget, hypot <= 0.04 m and |yaw| <= 0.08 rad,
accepts poses behind that boundary (up to 4.7 mm). The proposed region:

    -along <= 0.0459 - MARGIN - 0.19 * max(yaw, 0),   |left| <= 0.04,   |yaw| <= 0.08,   along <= 0.04

0.0459 m and 0.19 m/rad bound the boundary from inside at every probed lateral offset; MARGIN 0.003 m is kept to it.
Simulation-derived kinematics only: not a site, robot or safety tolerance.
"""
import json
import math
import os

import pytest
import yaml

from openamrobot_station_alignment import evaluation as e

HERE = os.path.dirname(os.path.abspath(__file__))
MARGIN = 0.003

with open(os.path.join(HERE, "fixtures", "station_reach_region.json"), encoding="utf-8") as f:
    DATA = json.load(f)
B = DATA["reach_boundary"]
LEFTS, YAWS, TABLE = B["left_m"], B["yaw_rad"], B["most_negative_along_m"]
POSES = DATA["aligned_final_poses"]["rows"]


def boundary(left, yaw):
    """Most negative along (m) at which the reach completes, bilinear in the probed table."""
    assert LEFTS[0] <= left <= LEFTS[-1] and YAWS[0] <= yaw <= YAWS[-1]
    i = min(int((left - LEFTS[0]) / (LEFTS[1] - LEFTS[0])), len(LEFTS) - 2)
    j = min(int((yaw - YAWS[0]) / (YAWS[1] - YAWS[0])), len(YAWS) - 2)
    tl = (left - LEFTS[i]) / (LEFTS[i + 1] - LEFTS[i])
    ty = (yaw - YAWS[j]) / (YAWS[j + 1] - YAWS[j])
    return (TABLE[i][j] * (1 - tl) * (1 - ty) + TABLE[i + 1][j] * tl * (1 - ty)
            + TABLE[i][j + 1] * (1 - tl) * ty + TABLE[i + 1][j + 1] * tl * ty)


def proposed(along, left, yaw):
    return (-along <= 0.0459 - MARGIN - 0.19 * max(yaw, 0.0) and abs(left) <= 0.04 and abs(yaw) <= 0.08
            and along <= 0.04)


def current(along, left, yaw):
    """The budget evaluation.py applies today."""
    return math.hypot(along, left) <= e.REACH_XY and abs(yaw) <= e.REACH_YAW


GRID = [(a / 1000, lat / 1000, y / 1000)
        for a in range(-60, 41, 1) for lat in range(-40, 41, 4) for y in range(-80, 81, 4)]


def test_the_boundary_table_is_the_reach_figures_of_upperbody_test_reach_margin():
    assert boundary(0.0, 0.0) == pytest.approx(-0.0484, abs=2e-4)      # test_reach_margin: -0.0485 m
    assert boundary(0.0, 0.08) == pytest.approx(-0.0353, abs=2e-4)     # test_reach_margin: -0.0353 m
    worst = min(-row[-1] for row in TABLE)
    assert worst == pytest.approx(0.0308, abs=1e-4)                    # left +0.04 m, yaw +0.08 rad


def test_the_current_budget_accepts_poses_behind_the_reach_boundary():
    """Consequence of 0.04 m / 0.08 rad: accepted poses from which the straight reach does not complete."""
    short = [boundary(lat, y) - a for a, lat, y in GRID if current(a, lat, y) and a < boundary(lat, y)]
    assert short and round(max(short) * 1000, 1) == 4.7


def test_the_proposed_region_keeps_the_margin_to_the_reach_boundary_everywhere():
    accepted = [(a, lat, y) for a, lat, y in GRID if proposed(a, lat, y)]
    assert accepted
    assert min(a - boundary(lat, y) for a, lat, y in accepted) >= MARGIN - 1e-4


def test_every_recorded_aligned_station_pose_is_inside_the_proposed_region():
    """Consequence on parking success: none of the 96 aligned trials would fail the proposed region."""
    assert len(POSES) == 96
    outside = [p for p in POSES if not proposed(p[0], p[1], p[2])]
    assert outside == []
    assert all(current(p[0], p[1], p[2]) for p in POSES)


def test_the_alignment_acceptance_corner_is_inside_the_proposed_region():
    """accept_along 0.025 + bias 0.0054 behind, accept_lateral 0.02 + 0.0027, accept_yaw 0.06 (config comment): the
    alignment can end there; the proposed region must accept it, the isotropic 0.034 m and 0.030 m radii do not."""
    with open(os.path.join(HERE, "..", "config", "station_alignment.yaml"), encoding="utf-8") as f:
        p = yaml.safe_load(f)["station_alignment"]["ros__parameters"]
    along, left, yaw = -(p["accept_along"] + 0.0054), p["accept_lateral"] + 0.0027, p["accept_yaw"]
    assert (round(-along, 4), round(left, 4), yaw) == (0.0304, 0.0227, 0.06)
    assert proposed(along, left, yaw)
    assert along - boundary(left, yaw) > MARGIN
    assert math.hypot(along, left) > 0.034


def test_the_margin_at_every_corner_of_the_lateral_heading_box():
    """Review of the coupled rule (Kunal's review of ea8ed13): margin = depth of the reach boundary minus the depth the
    rule allows, at the four corners of |left| <= 0.04 m x |yaw| <= 0.08 rad, both lateral signs, and its minimum
    over the whole box (0.5 mm x 0.5 mrad grid; bilinear in the probed table). Proposed only: no threshold changes."""
    allowed = lambda yaw: 0.0459 - MARGIN - 0.19 * max(yaw, 0.0)    # noqa: E731
    corners = {(lat, y): round(-boundary(lat, y) - allowed(y), 4) for lat in (-0.04, 0.04) for y in (-0.08, 0.08)}
    assert corners == {(-0.04, -0.08): 0.0119, (-0.04, 0.08): 0.0069, (0.04, -0.08): 0.0168, (0.04, 0.08): 0.0031}
    box = [(-boundary(lat / 2e4, y / 1e4) - allowed(y / 1e4), lat / 2e4, y / 1e4)
           for lat in range(-800, 801, 10) for y in range(-800, 801, 5)]
    worst = min(box)
    assert round(worst[0], 4) == 0.0030 and worst[1:] == (-0.04, 0.0)       # an edge, not a corner
    assert (e.REACH_XY, e.REACH_YAW) == (0.04, 0.08)                          # thresholds in code unchanged
