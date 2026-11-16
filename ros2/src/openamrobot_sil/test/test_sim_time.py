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


# --- coordinator (rule C5) and watch -----------------------------------------------------------

from openamrobot_sil.sim_time import STALL_BOUND_S, ClockCoordinator, ClockWatch  # noqa: E402


def coordinator():
    wall = ManualClock()
    return ClockCoordinator(FedClock(), steady=wall), wall


def test_no_stall_before_the_first_clock_message():
    coord, wall = coordinator()
    wall.advance(60.0)
    assert coord.check() is None and not coord.clock.paused


def test_stall_is_declared_after_the_bound_on_steady_time_and_reported_once():
    coord, wall = coordinator()
    coord.feed(3.0)
    wall.advance(STALL_BOUND_S)
    assert coord.check() is None                 # not before the bound
    wall.advance(0.01)
    report = coord.check()
    assert report.startswith('simulation stalled') and 'not a device fault' in report
    assert '3.000 s' in report                   # the last simulation time
    assert coord.clock.paused and coord.clock.stalls == 1
    wall.advance(5.0)
    assert coord.check() is None                 # one report per stall


def test_advancing_clock_never_stalls_and_resume_clears_pause():
    coord, wall = coordinator()
    for i in range(50):
        wall.advance(0.1)
        coord.feed(i * 0.05)                     # real-time factor 0.5
        assert coord.check() is None
    wall.advance(1.5)
    coord.check()
    assert coord.feed(10.0) == 'resumed' and not coord.clock.paused
    assert coord.clock.stalls == 1


def test_backward_clock_is_a_counted_reset():
    coord, _wall = coordinator()
    coord.feed(8.0)
    assert coord.feed(0.0) == 'reset'
    assert coord.resets == 1 and coord.clock.generation == 1


def test_a_stamp_from_before_a_stall_is_never_fresh():
    coord, wall = coordinator()
    coord.feed(1.0)
    s = Stamp(coord.clock)
    wall.advance(2.0)
    coord.check()
    coord.feed(1.001)
    assert not is_fresh(s, coord.clock, 100.0)
    assert is_fresh(Stamp(coord.clock), coord.clock, 0.1)


def test_watch_reports_reset_before_stall_before_resume():
    clk = ManualClock(5.0)
    watch = ClockWatch(clk)
    assert watch.poll() is None
    clk.declare_stall()
    clk.reset(0.0)
    assert watch.poll() == 'reset'
    clk.declare_stall()
    clk.declare_resume()
    assert watch.poll() == 'stall'
    clk.declare_stall()
    assert watch.poll() == 'stall'
    clk.declare_resume()
    assert watch.poll() == 'resume' and watch.poll() is None
