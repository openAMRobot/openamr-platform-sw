# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Simulation clock interface of the clock contract (docs/architecture/sil-topology.md section 6).

Every device model takes a clock object instead of reading host time. A clock gives:

- ``now()``: seconds in its domain;
- ``generation``: an integer that increases on every reset or backward jump of that time.
  A command, enable or permit stamped in an older generation is never fresh again (gate D3);
- ``paused``: True while the domain does not advance;
- ``stalls``: an integer that increases every time a stall is declared (a pause longer than the
  stall bound). A command, enable or permit received before a stall is never fresh after it.

``Stamp`` and ``is_fresh`` implement freshness by receipt time in the receiver's domain (rule C6).
Host-process health (rule C5) uses ``SteadyClock`` only and never raises a device fault.

``ClockCoordinator`` is the one coordinator of a process: it feeds a ``FedClock`` from ``/clock``
and declares a stall when the fed time has not advanced for longer than the stall bound measured
on host steady time (C5). ``ClockWatch`` lets a device model see a reset or a stall at its next
step, on its own thread.
"""
import math
import time


class SimClock:
    """Interface. Subclasses implement ``now``; ``generation`` and ``paused`` are attributes."""

    domain = 'abstract'

    def __init__(self):
        self.generation = 0
        self.paused = False
        self.stalls = 0

    def now(self):
        raise NotImplementedError


class SteadyClock(SimClock):
    """Host monotonic time: domain ``steady``. Used for an internal plant and for health checks."""

    domain = 'steady'

    def now(self):
        return time.monotonic()


class FedClock(SimClock):
    """Time fed from outside, for example from each ``/clock`` message of Gazebo (domain ``gazebo``).

    A time earlier than the last one is a reset: the generation increases. A time equal to the
    last one for longer than the caller's stall bound is a pause; the caller decides that with
    host time (rule C5) and sets ``paused``.
    """

    domain = 'gazebo'

    def __init__(self, start=0.0, domain='gazebo'):
        super().__init__()
        self.domain = domain
        self._t = float(start)

    def update(self, t):
        t = float(t)
        if not math.isfinite(t):
            raise ValueError('simulation time must be finite')
        if t < self._t:
            self.generation += 1
        self._t = t
        return self.generation

    def now(self):
        return self._t

    def declare_stall(self):
        """The coordinator saw no advance for longer than the stall bound (C5)."""
        self.paused = True
        self.stalls += 1

    def declare_resume(self):
        self.paused = False


class ManualClock(FedClock):
    """Test clock: advance, jump back and pause by hand."""

    def __init__(self, start=0.0):
        super().__init__(start, domain='manual')

    def advance(self, dt):
        if dt < 0:
            raise ValueError('use reset() to go back in time')
        if not self.paused:
            self.update(self._t + dt)

    def reset(self, t=0.0):
        """Backward jump (simulation reset). Always a new generation, even to the same time."""
        if float(t) >= self._t:
            self.generation += 1
            self._t = float(t)
        else:
            self.update(t)


class Stamp:
    """When a value was received, in the receiver's clock domain and generation."""

    __slots__ = ('t', 'generation', 'stalls')

    def __init__(self, clock):
        self.t = clock.now()
        self.generation = clock.generation
        self.stalls = clock.stalls


def is_fresh(stamp, clock, max_age):
    """True only for a stamp of the current generation, received after the last stall, that is
    not older than ``max_age`` seconds."""
    if stamp is None or stamp.generation != clock.generation or stamp.stalls != clock.stalls:
        return False
    age = clock.now() - stamp.t
    return 0.0 <= age <= max_age


# Contract section 6.3: a pause longer than 1 s wall time counts as a stall (proposal, software
# lead). A process-health bound (C5), not a device or firmware timeout (C3).
STALL_BOUND_S = 1.0


class ClockCoordinator:
    """One per process: feeds ``clock`` (a ``FedClock``) from ``/clock`` and supervises it.

    ``feed(t)`` is called with every ``/clock`` time. ``check()`` is called from a host steady
    timer (C5); when the fed time has not advanced for longer than ``stall_bound_s`` of
    ``steady`` time it declares a stall and returns the "simulation stalled" report. A stall is
    never a device fault: it is reported here, and the device models drop motion and permits
    through ``ClockWatch``. Before the first ``/clock`` nothing is declared.
    """

    def __init__(self, clock=None, stall_bound_s=STALL_BOUND_S, steady=None):
        self.clock = clock if clock is not None else FedClock()
        self.stall_bound_s = float(stall_bound_s)
        self.steady = steady if steady is not None else SteadyClock()
        self.fed = False
        self.resets = 0
        self._last_advance = None

    def feed(self, t):
        """Take one ``/clock`` time. Returns 'reset', 'resumed' or None."""
        clk = self.clock
        before_t, before_gen = clk.now(), clk.generation
        clk.update(t)
        event = None
        if clk.generation != before_gen:
            self.resets += 1
            event = 'reset'
        if not self.fed or event or clk.now() > before_t:
            self.fed = True
            self._last_advance = self.steady.now()
            if clk.paused:
                clk.declare_resume()
                event = event or 'resumed'
        return event

    def check(self):
        """Health check on steady time. Returns the stall report once per stall, else None."""
        if not self.fed or self.clock.paused:
            return None
        idle = self.steady.now() - self._last_advance
        if idle <= self.stall_bound_s:
            return None
        self.clock.declare_stall()
        return ('simulation stalled: /clock has not advanced for %.2f s wall time, last '
                'simulation time %.3f s (stall %d; not a device fault)' % (
                    idle, self.clock.now(), self.clock.stalls))

    def status(self):
        clk = self.clock
        return {'domain': clk.domain, 'now': round(clk.now(), 3), 'fed': self.fed,
                'generation': clk.generation, 'paused': clk.paused, 'stalls': clk.stalls,
                'resets': self.resets, 'stall_bound_s': self.stall_bound_s}


class ClockWatch:
    """What changed on a clock since the last ``poll()``: 'reset', 'stall', 'resume' or None.

    A device model polls at the start of each step on its own thread. 'reset' (a new generation)
    wins over 'stall', and 'stall' over 'resume'.
    """

    def __init__(self, clock):
        self.clock = clock
        self._seen = self._state()

    def _state(self):
        return (self.clock.generation, self.clock.stalls, self.clock.paused)

    def poll(self):
        gen, stalls, paused = now = self._state()
        seen_gen, seen_stalls, seen_paused = self._seen
        self._seen = now
        if gen != seen_gen:
            return 'reset'
        if stalls != seen_stalls:
            return 'stall'
        if seen_paused and not paused:
            return 'resume'
        return None
