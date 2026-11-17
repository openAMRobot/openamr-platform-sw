# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Behaviour of the simulation clock interface (clock contract, rules C1, C5, C6 and gate D3)."""
import pytest

from openamrobot_sil.sim_time import FedClock, ManualClock, SteadyClock, Stamp, is_fresh


def test_fresh_within_age_and_stale_after():
    clk = ManualClock()
    s = Stamp(clk)
    clk.advance(0.149)
    assert is_fresh(s, clk, 0.150)
    clk.advance(0.002)
    assert not is_fresh(s, clk, 0.150)


def test_pause_freezes_age():
    clk = ManualClock(10.0)
    s = Stamp(clk)
    clk.paused = True
    clk.advance(5.0)
    assert clk.now() == 10.0
    assert is_fresh(s, clk, 0.150)


def test_reset_invalidates_every_older_stamp():
    clk = ManualClock(10.0)
    s = Stamp(clk)
    clk.reset(10.0)
    assert clk.generation == 1
    assert not is_fresh(s, clk, 100.0)


def test_backward_feed_is_a_new_generation():
    clk = FedClock(5.0)
    s = Stamp(clk)
    clk.update(5.1)
    assert is_fresh(s, clk, 0.2)
    clk.update(0.0)
    assert clk.generation == 1
    assert not is_fresh(s, clk, 100.0)


def test_non_finite_time_is_refused():
    with pytest.raises(ValueError):
        FedClock().update(float('nan'))


def test_none_stamp_is_never_fresh():
    assert not is_fresh(None, ManualClock(), 1.0)


def test_steady_clock_advances_and_is_its_own_domain():
    clk = SteadyClock()
    assert clk.domain == 'steady' and clk.generation == 0
    assert clk.now() <= clk.now()
