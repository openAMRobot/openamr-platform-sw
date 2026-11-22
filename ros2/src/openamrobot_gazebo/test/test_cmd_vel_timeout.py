# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""Command timeout of the simulated base (#79): the decision logic, without ROS."""

from openamrobot_gazebo.cmd_vel_timeout import (
    CommandTimeout, DEFAULT_TIMEOUT_S, DEFAULT_ZERO_REPEAT_S, validate_tick_period,
    validate_zero_repeat_period, zero_is_due)
import pytest


def test_default_matches_the_real_base_controller():
    # Teensy firmware and Build 1 STM32: 200 ms without a fresh /cmd_vel.
    assert DEFAULT_TIMEOUT_S == 0.2
    assert CommandTimeout().timeout_s == 0.2


@pytest.mark.parametrize('bad', [0.0, -0.2, float('nan')])
def test_timeout_must_be_positive(bad):
    with pytest.raises(ValueError):
        CommandTimeout(bad)


def test_nothing_is_sent_before_the_first_command():
    t = CommandTimeout()
    assert t.on_tick(0.0) == (False, False)
    assert t.on_tick(100.0) == (False, False)
    assert t.timeouts == 0


def test_fresh_command_is_not_stale():
    t = CommandTimeout()
    t.on_command(10.0)
    assert t.on_tick(10.0) == (False, False)
    assert t.on_tick(10.199) == (False, False)


def test_zero_from_the_timeout_on_and_every_tick_after():
    t = CommandTimeout()
    t.on_command(10.0)
    assert t.on_tick(10.2) == (True, True)
    assert t.on_tick(10.22) == (True, False)
    assert t.on_tick(50.0) == (True, False)
    assert t.timeouts == 1 and t.stale


def test_next_command_resumes_non_latching():
    t = CommandTimeout()
    t.on_command(10.0)
    t.on_tick(10.3)
    assert t.on_command(10.31) is True
    assert not t.stale
    assert t.on_tick(10.4) == (False, False)
    assert t.on_command(10.41) is False


def test_each_timeout_is_counted_once():
    t = CommandTimeout()
    for start in (0.0, 1.0, 2.0):
        t.on_command(start)
        t.on_tick(start + 0.25)
        t.on_tick(start + 0.5)
    assert t.timeouts == 3


def test_sub_microsecond_clock_jitter_does_not_look_like_a_reset():
    t = CommandTimeout()
    t.on_command(10.0)
    assert t.on_tick(10.0 - 5e-7) == (False, False)
    assert t.on_tick(10.0 + 0.199) == (False, False)


@pytest.mark.parametrize('period', [0.0, -0.01, 0.2, 0.21, float('nan')])
def test_tick_period_must_be_positive_and_shorter_than_timeout(period):
    with pytest.raises(ValueError):
        validate_tick_period(0.2, period)


def test_tick_period_can_be_shorter_than_timeout():
    validate_tick_period(0.2, 0.02)


def test_clock_going_backwards_counts_as_stale():
    # A simulation reset moves the clock back; the command from before it must not keep driving.
    t = CommandTimeout()
    t.on_command(30.0)
    assert t.on_tick(0.5) == (True, True)


def test_age_follows_the_given_clock():
    t = CommandTimeout(0.5)
    assert t.age(3.0) is None
    t.on_command(3.0)
    assert t.age(3.25) == pytest.approx(0.25)
    assert t.on_tick(3.49) == (False, False)
    assert t.on_tick(3.5) == (True, True)


def test_stale_message_names_the_age_and_never_prints_a_negative_one():
    from openamrobot_gazebo.cmd_vel_timeout import describe_stale
    normal = describe_stale(0.2, 0.2)
    assert '0.200 s' in normal and 'holding the simulated base at zero' in normal
    reset = describe_stale(-4.98, 0.2)
    assert 'went back 4.980 s' in reset and 'holding the simulated base at zero' in reset
    assert '-' not in reset.replace('(simulation reset?)', '')


def test_zero_is_due_at_once_then_every_repeat_period():
    assert zero_is_due(10.2, None, 0.1)
    assert not zero_is_due(10.25, 10.2, 0.1)
    assert zero_is_due(10.3, 10.2, 0.1)        # the exact period counts (float tolerance)


def test_zero_is_due_again_when_the_clock_went_backwards():
    # A reset puts the clock before the last zero; waiting for it to catch up would suppress zeros.
    assert zero_is_due(5.0, 60.4, 0.1)


@pytest.mark.parametrize('repeat', [0.01, 0.0, 0.25, float('nan')])
def test_repeat_period_must_lie_between_tick_and_timeout(repeat):
    with pytest.raises(ValueError):
        validate_zero_repeat_period(0.2, 0.02, repeat)


def test_default_repeat_period_is_valid_with_the_default_timeout():
    validate_zero_repeat_period(DEFAULT_TIMEOUT_S, 0.02, DEFAULT_ZERO_REPEAT_S)
