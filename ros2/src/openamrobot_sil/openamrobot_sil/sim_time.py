# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Simulation clock interface of the clock contract (docs/architecture/sil-topology.md section 6).

Every device model takes a clock object instead of reading host time. A clock gives:

- ``now()``: seconds in its domain;
- ``generation``: an integer that increases on every reset or backward jump of that time.
  A command, enable or permit stamped in an older generation is never fresh again (gate D3);
- ``paused``: True while the domain does not advance.

``Stamp`` and ``is_fresh`` implement freshness by receipt time in the receiver's domain (rule C6).
Host-process health (rule C5) uses ``SteadyClock`` only and never raises a device fault.
"""
import math
import time


class SimClock:
    """Interface. Subclasses implement ``now``; ``generation`` and ``paused`` are attributes."""

    domain = 'abstract'

    def __init__(self):
        self.generation = 0
        self.paused = False

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

    __slots__ = ('t', 'generation')

    def __init__(self, clock):
        self.t = clock.now()
        self.generation = clock.generation


def is_fresh(stamp, clock, max_age):
    """True only for a stamp of the current generation that is not older than ``max_age`` seconds."""
    if stamp is None or stamp.generation != clock.generation:
        return False
    age = clock.now() - stamp.t
    return 0.0 <= age <= max_age
