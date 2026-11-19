# SPDX-License-Identifier: MIT
"""The 0.04 m / 0.08 rad reach budget against the straight reach of scn04 (review of #167). Characterisation: no
threshold is changed here.

Straight reach (0.10 m along the fingers, then 0.02 m down, MoveIt in 5 mm steps with no position tolerance), from
openamr-upperbody-sw ros2/src/openamrobot_build1_scene/test/test_reach_margin.py and docs/integration/station_approach.md
(demo/control-room, ee36e87): it completes for a base down to 0.0485 m too far back along the heading at 0 rad and to
0.0353 m at 0.08 rad; beyond that openarm_left_joint4 is at its lower limit 0 rad. Only these two yaw values were
probed; the sign of the yaw at 0.08 rad and any lateral offset are not in those figures.
"""
import os

import yaml

from openamrobot_station_alignment import evaluation as e

REACH_BACK_AT_0_RAD = 0.0485     # m, base too far from the shelf along the heading
REACH_BACK_AT_0_08_RAD = 0.0353  # m
MARKER_ALONG_BIAS = 0.0054       # m, largest of 20 CI trials (config/station_alignment.yaml)
HERE = os.path.dirname(os.path.abspath(__file__))


def alignment_parameters():
    with open(os.path.join(HERE, "..", "config", "station_alignment.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)["station_alignment"]["ros__parameters"]


def test_the_reach_budget_passes_a_pose_from_which_the_straight_reach_does_not_complete():
    """Consequence: a trial that ends 0.038 m too far back at 0.08 rad is inside the budget, so the trials verdict
    passes it, while the straight reach stops 2.7 mm short of completing there (elbow at its limit)."""
    error_back, error_yaw = 0.038, 0.08
    ok, why = e.evaluate([{"trial": 1, "aligned": {"error_xy": error_back, "error_yaw": error_yaw}}], 1)
    assert ok and not why
    assert error_back > REACH_BACK_AT_0_08_RAD
    # The along-heading room the budget claims at its yaw limit is 4.7 mm more than the reach has.
    assert round((e.REACH_XY - REACH_BACK_AT_0_08_RAD) * 1000, 1) == 4.7
    assert e.REACH_XY < REACH_BACK_AT_0_RAD   # at 0 rad the budget is inside the reach


def test_the_alignment_acceptance_along_the_heading_stays_inside_the_straight_reach_at_0_08_rad():
    """The alignment itself accepts at most accept_along plus the marker bias along the heading (0.0304 m) and
    accept_yaw 0.06 rad. Against the 0.08 rad reach figure (assumed the tighter of the two probed yaws, because the
    room shrinks from 0 to 0.08 rad) that leaves 4.9 mm. A lateral residual up to accept_lateral is not in the reach
    figures and is not checked here."""
    p = alignment_parameters()
    worst_back = p["accept_along"] + MARKER_ALONG_BIAS
    assert p["accept_yaw"] <= 0.08
    assert worst_back < REACH_BACK_AT_0_08_RAD
    assert round((REACH_BACK_AT_0_08_RAD - worst_back) * 1000, 1) == 4.9
