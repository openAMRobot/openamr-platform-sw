# SPDX-License-Identifier: MIT
"""Pass criteria of the simulated trials (openamrobot_station_alignment.evaluation)."""
from openamrobot_station_alignment import evaluation as e


def trial(n, xy, yaw):
    return {"trial": n, "baseline": {"error_xy": 0.05, "error_yaw": 0.01},
            "aligned": {"error_xy": xy, "error_yaw": yaw, "corrections": 1}}


def test_all_within_the_budget_passes():
    ok, why = e.evaluate([trial(i, 0.02, 0.03) for i in range(1, 6)], 5)
    assert ok and not why


def test_one_trial_beyond_the_budget_fails():
    trials = [trial(i, 0.02, 0.03) for i in range(1, 5)] + [trial(5, 0.041, 0.0)]
    ok, why = e.evaluate(trials, 5)
    assert not ok and "trial 5" in why[0]
    ok, _ = e.evaluate([trial(1, 0.0, 0.081)], 1)
    assert not ok


def test_missing_trials_fail():
    ok, why = e.evaluate([trial(1, 0.01, 0.01)], 5)
    assert not ok and "1 of 5" in why[0]
    ok, _ = e.evaluate([{"trial": 1, "aligned": {"error_xy": None}}], 1)
    assert not ok


def test_command_publishers():
    good = {"/cmd_vel": ["collision_monitor"], "/cmd_vel_nav": ["behavior_server", "controller_server"]}
    assert e.command_publishers_ok(good)[0]
    assert not e.command_publishers_ok({**good, "/cmd_vel": ["collision_monitor", "station_alignment"]})[0]
    assert not e.command_publishers_ok({**good, "/cmd_vel_nav": ["controller_server", "dock_trigger"]})[0]
    assert not e.command_publishers_ok({"/cmd_vel": [], "/cmd_vel_nav": []})[0]


def test_summary_lists_every_trial():
    text = e.summary([trial(i, 0.02, 0.03) for i in range(1, 4)])
    assert text.count("\n| ") == 3 and "#164 + marker, position: mean 0.0200" in text


def repetition(n, nav2="succeeded", xy=0.02, yaw=0.03, start_xy=0.03, start_yaw=0.40, pre="succeeded"):
    return {"repetition": n, "pre_nav2": pre, "start": {"amcl_goal_xy": start_xy, "amcl_goal_yaw": start_yaw},
            "nav2": nav2, "error_xy": xy, "error_yaw": yaw}


def test_short_correction_offsets_alternate_and_are_inside_the_regressions_region():
    a, b = e.short_correction_offset(0), e.short_correction_offset(1)
    assert a == (0.0, e.SHORT_CORRECTION_LEFT, e.SHORT_CORRECTION_YAW) and b == (0.0, -a[1], -a[2])
    # platform-sw #192: shorter than the general progress radius, further off than initial_rotation_tolerance.
    assert e.SHORT_CORRECTION_LEFT < e.SHORT_CORRECTION_MAX_XY == 0.05
    assert e.SHORT_CORRECTION_YAW > e.SHORT_CORRECTION_MIN_YAW == 0.30


def test_short_corrections_pass_only_when_every_exercised_repetition_succeeds_inside_the_budget():
    ok, why = e.evaluate_short_corrections([repetition(i) for i in range(1, 6)], 5)
    assert ok and not why
    ok, why = e.evaluate_short_corrections([repetition(i) for i in range(1, 5)] + [repetition(5, nav2="aborted")], 5)
    assert not ok and "repetition 5: Nav2 aborted" in why[0]
    ok, why = e.evaluate_short_corrections([repetition(1, yaw=0.081)], 1)
    assert not ok and "beyond the reach budget" in why[0]
    ok, why = e.evaluate_short_corrections([repetition(1, xy=0.041)], 1)
    assert not ok


def test_a_repetition_outside_the_region_is_not_counted_as_evidence():
    far = repetition(1, start_xy=0.06)
    straight = repetition(2, start_yaw=0.29)
    no_pre = repetition(3, pre="aborted")
    for r in (far, straight, no_pre):
        assert not e.short_correction_exercised(r)
    ok, why = e.evaluate_short_corrections([far, straight, no_pre, repetition(4)], 2)
    assert not ok and "1 of 2" in why[0]
    # An unexercised repetition that failed does not fail the regression, and does not pass it either.
    ok, why = e.evaluate_short_corrections([repetition(1, nav2="aborted", start_xy=0.06), repetition(2)], 1)
    assert ok and not why


def test_short_correction_summary_has_a_row_per_repetition():
    text = e.short_correction_summary([repetition(1), repetition(2, nav2="aborted", start_xy=0.06)])
    assert text.count("\n") == 3 and "| 2 | succeeded | 0.0600 |" in text and "| no | aborted |" in text
