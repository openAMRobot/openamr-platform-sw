"""Start-up trace helpers of the simref fixture: when does each observer first see the lift joint
state, and what else is the machine doing. No ROS import, so it is unit tested without Gazebo
(test_startup_trace.py). Diagnostics only: nothing here changes the stack under test.

All times are time.monotonic() in milliseconds (mono_ms) so that they correlate with the
LIFTDIAG lines of the lift node and with each other, plus the wall clock for the vcan captures.
Simulation (header) timestamps are kept as separate fields and never mixed with these clocks.
"""
import threading
import time


class StartupTrace:
    """Ordered list of (mono_ms, wall, who, text) events, safe to call from several threads."""

    def __init__(self, clock=time.monotonic, wall=time.time):
        self.clock, self.wall = clock, wall
        self.lock = threading.Lock()
        self.events = []

    def mark(self, who, text):
        with self.lock:
            self.events.append((int(self.clock() * 1000), self.wall(), who, text))

    def format(self):
        with self.lock:
            events = sorted(self.events, key=lambda e: e[0])
        if not events:
            return 'startup trace: no events'
        t0 = events[0][0]
        lines = ['startup trace: %d events (mono_ms absolute, +ms from the first event)' % len(events)]
        lines += ['  mono_ms=%d +%-6d wall=%.3f %-12s %s' % (m, m - t0, w, who, text)
                  for m, w, who, text in events]
        return '\n'.join(lines)


class FirstSecondCounter:
    """Counts messages in the first window_s seconds after the first one, keeping their stamps."""

    def __init__(self, window_s=1.0):
        self.window_s, self.first, self.count, self.stamps = window_s, None, 0, []

    def add(self, now_s, stamp):
        """Record one message; returns True for the very first one."""
        if self.first is None:
            self.first = now_s
        if now_s - self.first <= self.window_s:
            self.count += 1
            self.stamps.append(stamp)
        return self.count == 1 and len(self.stamps) == 1

    def done(self, now_s):
        return self.first is not None and now_s - self.first > self.window_s

    def summary(self):
        if not self.stamps:
            return 'no messages'
        return '%d messages in the first %.1f s, header stamps %s .. %s' % (
            self.count, self.window_s, self.stamps[0], self.stamps[-1])


class GraphWatcher:
    """Reports when the visible endpoints of a topic change (count or who/QoS)."""

    def __init__(self):
        self.last = None

    def observe(self, count, who):
        """who: tuple of 'node reliability durability' strings. True when it differs from before."""
        state = (count, tuple(sorted(who)))
        if state == self.last:
            return False
        self.last = state
        return True


def cpu_percent(prev, cur):
    """Busy, iowait and steal percentages between two /proc/stat 'cpu' lines (None when unusable)."""
    try:
        a = [int(x) for x in prev.split()[1:9]]
        b = [int(x) for x in cur.split()[1:9]]
    except (ValueError, IndexError):
        return None
    d = [y - x for x, y in zip(a, b)]
    total = sum(d)
    if total <= 0:
        return None
    idle = d[3] + d[4]               # idle + iowait
    return {'busy': 100.0 * (total - idle) / total, 'iowait': 100.0 * d[4] / total,
            'steal': 100.0 * d[7] / total}


class LineCapture:
    """Keeps (mono_ms, line) pairs of a text stream, up to max_lines, for offline analysis."""

    def __init__(self, max_lines=20000):
        self.max_lines, self.lines, self.dropped = max_lines, [], 0

    def add(self, mono_ms, line):
        if len(self.lines) < self.max_lines:
            self.lines.append((mono_ms, line.rstrip('\n')))
        else:
            self.dropped += 1

    def format(self):
        out = ['%d %s' % (m, ln) for m, ln in self.lines]
        if self.dropped:
            out.append('# %d further lines dropped (cap %d)' % (self.dropped, self.max_lines))
        return '\n'.join(out)
