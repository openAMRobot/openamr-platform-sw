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

from openamrobot_sil.sim_time import (JUMP_BOUND_S, STALL_BOUND_S, ClockCoordinator,  # noqa: E402
                                      ClockWatch)


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


# --- temporal discontinuities (senior review 17 Nov 2026) --------------------------------------

def test_forward_jump_is_reported_never_a_new_generation_and_never_makes_a_stamp_fresh():
    coord, wall = coordinator()
    coord.feed(5.0)
    s = Stamp(coord.clock)
    wall.advance(0.01)
    assert coord.feed(15.0) == 'jump'                 # +10 s in one /clock message
    assert coord.jumps == 1 and coord.clock.generation == 0 and coord.resets == 0
    report = coord.jump_report()
    assert 'jumped forward by 10.000 s' in report and 'not a device fault' in report
    assert not is_fresh(s, coord.clock, 0.5)          # the elapsed 10 s count as age
    assert coord.check() is None                      # a jump is an advance, not a stall


def test_steps_up_to_the_jump_bound_are_not_jumps():
    coord, _wall = coordinator()
    coord.feed(1.0)
    assert coord.feed(1.0 + JUMP_BOUND_S) is None
    assert coord.jumps == 0 and coord.jump_report() is None


def test_a_jump_that_ends_a_stall_resumes_and_is_counted():
    coord, wall = coordinator()
    coord.feed(2.0)
    wall.advance(1.5)
    assert coord.check().startswith('simulation stalled')
    assert coord.feed(30.0) == 'resumed'
    assert coord.jumps == 1 and not coord.clock.paused
    assert coord.status()['jumps'] == 1


def test_duplicate_clock_times_change_nothing_and_raise_no_stall():
    coord, wall = coordinator()
    watch = ClockWatch(coord.clock)
    t = 4.0
    for _ in range(100):                              # every time published twice, 10 s wall
        wall.advance(0.05)
        t += 0.05
        assert coord.feed(t) is None
        assert coord.feed(t) is None                  # duplicate
        assert coord.check() is None
    assert coord.clock.generation == 0 and coord.clock.stalls == 0
    assert coord.resets == 0 and coord.jumps == 0 and watch.poll() is None


def test_duplicates_alone_are_no_advance_so_they_do_not_hide_a_stall():
    coord, wall = coordinator()
    coord.feed(4.0)
    for _ in range(11):                               # a paused Gazebo republishes 4.0 s
        wall.advance(0.1)
        coord.feed(4.0)
    assert coord.check().startswith('simulation stalled')
    assert coord.clock.stalls == 1 and coord.clock.generation == 0


def test_delayed_clock_below_the_stall_bound_is_no_stall():
    coord, wall = coordinator()
    coord.feed(1.0)
    wall.advance(STALL_BOUND_S - 0.05)                # /clock late on steady time
    assert coord.check() is None
    assert coord.feed(1.001) is None                  # simulation time continues normally
    assert coord.clock.stalls == 0 and coord.jumps == 0


def test_delayed_clock_above_the_stall_bound_is_one_stall_and_no_jump_on_resume():
    coord, wall = coordinator()
    coord.feed(1.0)
    wall.advance(STALL_BOUND_S + 0.05)
    report = coord.check()
    assert report.startswith('simulation stalled') and '1.000 s' in report
    assert '1.05 s wall time' in report               # wall time and last simulation time (C5)
    assert coord.check() is None
    assert coord.feed(1.001) == 'resumed'             # the same simulation step continues
    assert coord.jumps == 0 and coord.clock.generation == 0 and coord.clock.stalls == 1


def test_fed_clock_is_not_fed_before_its_first_time():
    clk = FedClock()
    assert not clk.fed
    clk.update(0.0)
    assert clk.fed and clk.generation == 0
    assert ManualClock().fed and SteadyClock().fed


# --- atomic stamp (review finding, 17 Nov 2026) ------------------------------------------------

class ResetOnFirstRead(ManualClock):
    """Deterministic stand-in for a /clock reset arriving on another thread: the first read of
    the clock state returns the pre-reset value and then a reset to 0 s lands, before any
    second read the reader makes."""

    def __init__(self, start):
        super().__init__(start)
        self.armed = False

    def _fire(self):
        if self.armed:
            self.armed = False
            self.reset(0.0)

    def now(self):
        t = super().now()
        self._fire()
        return t

    def snapshot(self):
        state = super().snapshot()
        self._fire()
        return state


def test_stamp_takes_time_and_generation_together_across_a_reset():
    clk = ResetOnFirstRead(100.0)
    clk.armed = True
    s = Stamp(clk)
    assert clk.generation == 1 and clk.now() == 0.0       # the reset landed during the stamp
    # never a pre-reset time paired with the new generation, or the reverse
    assert (s.t, s.generation) in ((100.0, 0), (0.0, 1))
    clk.advance(100.05)
    assert not is_fresh(s, clk, 0.1)    # a pre-reset value never becomes fresh after the reset


def test_snapshot_is_time_and_generation_of_every_clock():
    clk = FedClock(2.0)
    assert clk.snapshot() == (2.0, 0)
    clk.update(1.0)
    assert clk.snapshot() == (1.0, 1) == (clk.now(), clk.generation)
    man = ManualClock(3.0)
    man.reset(3.0)
    man.advance(0.5)
    assert man.snapshot() == (3.5, 1)
    t, gen = SteadyClock().snapshot()
    assert gen == 0 and t <= SteadyClock().now()
