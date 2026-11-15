"""Odometry from the wheel speeds the image reports (SW-08): unknown is never zero motion."""
import math

import pytest

from openamrobot_sil.fw_odometry import FwOdometry

NAN = math.nan


def test_straight_line_from_equal_wheel_speeds():
    odo = FwOdometry(0.1, 0.4)
    for i in range(11):
        assert odo.update(2.0, 2.0, i * 0.1)
    assert odo.x == pytest.approx(0.2) and odo.y == pytest.approx(0.0)
    assert odo.v == pytest.approx(0.2) and odo.valid


def test_turn_on_the_spot_from_opposite_wheel_speeds():
    odo = FwOdometry(0.1, 0.4)
    for i in range(11):
        odo.update(-1.0, 1.0, i * 0.1)
    assert odo.x == pytest.approx(0.0, abs=1e-9) and odo.yaw == pytest.approx(0.5)


def test_unknown_samples_move_nothing_and_are_not_zero_speed():
    odo = FwOdometry(0.1, 0.4)
    odo.update(2.0, 2.0, 0.0)
    odo.update(2.0, 2.0, 0.1)
    x = odo.x
    for t in (0.2, 0.3, 0.4):
        assert not odo.update(NAN, 2.0, t)
    assert odo.x == x and not odo.valid and math.isnan(odo.v)
    assert odo.unknown_samples == 3
    odo.update(2.0, 2.0, 0.5)             # first sample after the gap: no step across it
    assert odo.x == x and odo.valid
    odo.update(2.0, 2.0, 0.6)
    assert odo.x == pytest.approx(x + 0.02)


def test_a_long_gap_between_valid_samples_is_not_integrated():
    odo = FwOdometry(0.1, 0.4, max_gap_s=1.0)
    odo.update(2.0, 2.0, 0.0)
    odo.update(2.0, 2.0, 5.0)
    assert odo.x == 0.0 and odo.gaps == 1


def test_signs_follow_the_configuration():
    odo = FwOdometry(0.1, 0.4, left_sign=1, right_sign=-1)
    odo.update(2.0, -2.0, 0.0)
    odo.update(2.0, -2.0, 1.0)
    assert odo.x == pytest.approx(0.2)
