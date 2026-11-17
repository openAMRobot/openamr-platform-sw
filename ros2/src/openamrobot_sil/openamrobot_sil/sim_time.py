# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Simulation clock interface of the clock contract (docs/architecture/sil-topology.md section 6).

Every device model takes a clock object instead of reading host time. A clock gives:

- ``now()``: seconds in its domain;
- ``generation``: an integer that increases on every reset or backward jump of that time.
  A command, enable or permit stamped in an older generation is never fresh again (gate D3);
- ``snapshot()``: ``(now, generation)`` read together. A ``FedClock`` keeps both in one tuple
  that a writer replaces in a single assignment, so a reader on another thread never pairs a
  new time with an old generation;
- ``paused``: True while the domain does not advance;
- ``stalls``: an integer that increases every time a stall is declared (a pause longer than the
  stall bound). A command, enable or permit received before a stall is never fresh after it;
- ``fed``: False only for a ``FedClock`` that has not received its first time yet. A device
  model or stand-in does not act before it (first-clock startup).

``Stamp`` and ``is_fresh`` implement freshness by receipt time in the receiver's domain (rule C6).
Host-process health (rule C5) uses ``SteadyClock`` only and never raises a device fault.

``ClockCoordinator`` is the one coordinator of a process: it feeds a ``FedClock`` from ``/clock``
and declares a stall when the fed time has not advanced for longer than the stall bound measured
on host steady time (C5). ``ClockWatch`` lets a device model see a reset or a stall at its next
step, on its own thread.
"""
import math
import threading
import time


class SimClock:
    """Interface. Subclasses implement ``now``; ``generation`` and ``paused`` are attributes."""

    domain = 'abstract'

    def __init__(self):
        self.generation = 0
        self.paused = False
        self.stalls = 0
        self.fed = True

    def now(self):
        raise NotImplementedError

    def snapshot(self):
        """``(now, generation)`` together. A clock whose time and generation can change on
        another thread overrides this to read both in one step."""
        return self.now(), self.generation


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
        # (t, generation) is one tuple, replaced in a single assignment under the writer lock,
        # so now(), generation and snapshot() each read one consistent pair.
        self._lock = threading.Lock()
        self._state = (float(start), 0)
        super().__init__()
        self.domain = domain
        self.fed = False

    @property
    def generation(self):
        return self._state[1]

    @generation.setter
    def generation(self, value):
        with self._lock:
            self._state = (self._state[0], int(value))

    def update(self, t):
        t = float(t)
        if not math.isfinite(t):
            raise ValueError('simulation time must be finite')
        with self._lock:
            last_t, gen = self._state
            if t < last_t:
                gen += 1
            self._state = (t, gen)
        self.fed = True
        return gen

    def now(self):
        return self._state[0]

    def snapshot(self):
        return self._state

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
        self.fed = True

    def advance(self, dt):
        if dt < 0:
            raise ValueError('use reset() to go back in time')
        if not self.paused:
            with self._lock:
                last_t, gen = self._state
                t = last_t + float(dt)
                if not math.isfinite(t):
                    raise ValueError('simulation time must be finite')
                self._state = (t, gen)

    def reset(self, t=0.0):
        """Backward jump (simulation reset). Always a new generation, even to the same time."""
        t = float(t)
        if not math.isfinite(t):
            raise ValueError('simulation time must be finite')
        with self._lock:
            last_t, gen = self._state
            self._state = (t, gen + 1)


class Stamp:
    """When a value was received, in the receiver's clock domain and generation."""

    __slots__ = ('t', 'generation', 'stalls')

    def __init__(self, clock):
        self.t, self.generation = clock.snapshot()
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
# A forward step of the fed time larger than this between two /clock messages is reported as a
# jump (proposal, software lead; same value as the stall bound). Report only: no device decision
# reads it, and no device timeout changes (C3).
JUMP_BOUND_S = STALL_BOUND_S


class ClockCoordinator:
    """One per process: feeds ``clock`` (a ``FedClock``) from ``/clock`` and supervises it.

    ``feed(t)`` is called with every ``/clock`` time. ``check()`` is called from a host steady
    timer (C5); when the fed time has not advanced for longer than ``stall_bound_s`` of
    ``steady`` time it declares a stall and returns the "simulation stalled" report. A stall is
    never a device fault: it is reported here, and the device models drop motion and permits
    through ``ClockWatch``. Before the first ``/clock`` nothing is declared.

    A forward step larger than ``jump_bound_s`` is a jump: the device models see the elapsed
    time, so their own timeouts expire as they would on the device (C3), and ``jump_report()``
    gives the report that goes next to any fault they raise. A jump is never a new generation.
    A duplicate time (the same time twice) is not an advance: it neither stalls nor resets.
    """

    def __init__(self, clock=None, stall_bound_s=STALL_BOUND_S, steady=None,
                 jump_bound_s=JUMP_BOUND_S):
        self.clock = clock if clock is not None else FedClock()
        self.stall_bound_s = float(stall_bound_s)
        self.jump_bound_s = float(jump_bound_s)
        self.steady = steady if steady is not None else SteadyClock()
        self.fed = False
        self.resets = 0
        self.jumps = 0
        self.last_jump = None     # (from_s, to_s) of the last jump
        self._last_advance = None

    def feed(self, t):
        """Take one ``/clock`` time. Returns 'reset', 'resumed', 'jump' or None.

        A jump that also ends a stall returns 'resumed'; ``jumps`` counts it either way."""
        clk = self.clock
        before_t, before_gen = clk.now(), clk.generation
        clk.update(t)
        event = None
        jumped = False
        if clk.generation != before_gen:
            self.resets += 1
            event = 'reset'
        elif self.fed and clk.now() - before_t > self.jump_bound_s:
            self.jumps += 1
            self.last_jump = (before_t, clk.now())
            jumped = True
        if not self.fed or event or clk.now() > before_t:
            self.fed = True
            self._last_advance = self.steady.now()
            if clk.paused:
                clk.declare_resume()
                event = event or 'resumed'
        if event is None and jumped:
            event = 'jump'
        return event

    def jump_report(self):
        """Report of the last forward jump, or None. Not a device fault."""
        if self.last_jump is None:
            return None
        before, after = self.last_jump
        return ('simulation time jumped forward by %.3f s, from %.3f s to %.3f s (jump %d): '
                'device timers see the elapsed time, a device timeout that expires in it is the '
                'device behaviour; the jump itself is not a device fault' % (
                    after - before, before, after, self.jumps))

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
                'resets': self.resets, 'jumps': self.jumps, 'stall_bound_s': self.stall_bound_s}


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
