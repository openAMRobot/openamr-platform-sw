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
