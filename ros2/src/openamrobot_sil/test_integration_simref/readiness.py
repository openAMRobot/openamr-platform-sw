"""Readiness gate of the simref end-to-end fixture, with a per-condition failure report.

No ROS import: the conditions read values through callables, so the gate and its report are
unit tested without Gazebo or Nav2 (test_readiness.py). Development evidence only.

The gate passes only when EVERY condition holds at the same time, exactly like the single
boolean expression it replaces. Nothing here changes a timeout or relaxes a condition. The
STALE_AFTER_S threshold only picks the word "stale" instead of "wrong" in the report of a
condition that is already failing; it never makes a condition pass or fail.
"""
import time

STALE_AFTER_S = 5.0     # report label only, see the module docstring

MISSING, STALE, WRONG, OK = 'missing', 'stale', 'wrong', 'ok'


class Condition:
    """One readiness requirement.

    observe() returns (value, updated) where value is None when nothing was ever received and
    updated is the time.monotonic() of the last update, or None when it has no time (a count).
    holds(value) is the requirement. expected is the text shown in the report.
    """

    def __init__(self, name, expected, observe, holds):
        self.name, self.expected, self.observe, self.holds = name, expected, observe, holds


def evaluate(conditions, now):
    """One report row per condition; row['ok'] is the requirement itself, unaffected by age."""
    rows = []
    for c in conditions:
        try:
            value, updated = c.observe()
        except (KeyError, TypeError, ValueError, AttributeError, IndexError):
            value, updated = None, None
        try:
            ok = value is not None and bool(c.holds(value))
        except (KeyError, TypeError, ValueError, AttributeError, IndexError):
            ok = False
        age = None if updated is None else max(0.0, now - updated)
        if ok:
            status = OK
        elif value is None:
            status = MISSING
        elif age is not None and age > STALE_AFTER_S:
            status = STALE
        else:
            status = WRONG
        rows.append({'name': c.name, 'expected': c.expected, 'observed': value, 'age': age,
                     'status': status, 'ok': ok})
    return rows


def wait_ready(conditions, timeout, step=0.2, clock=time.monotonic, sleep=time.sleep):
    """Poll until all conditions hold at once or timeout elapses. Returns (ok, rows, waited)."""
    start = clock()
    while True:
        rows = evaluate(conditions, clock())
        if all(r['ok'] for r in rows):
            return True, rows, clock() - start
        if clock() - start >= timeout:
            return False, rows, clock() - start
        sleep(step)


def format_report(rows, waited, timeout):
    lines = ['readiness gate: %d of %d conditions hold after waiting %.1f s (limit %.1f s)'
             % (sum(r['ok'] for r in rows), len(rows), waited, timeout),
             '  %-34s %-8s %-26s %-30s %s' % ('condition', 'status', 'expected', 'last observed',
                                              'last update')]
    for r in rows:
        age = 'no update time' if r['age'] is None else '%.1f s ago' % r['age']
        if r['status'] == MISSING:
            age = 'never received'
        lines.append('  %-34s %-8s %-26s %-30s %s' % (r['name'], r['status'].upper(), r['expected'],
                                                      _short(r['observed']), age))
    return '\n'.join(lines)


def _short(value, limit=30):
    text = '-' if value is None else repr(value)
    return text if len(text) <= limit else text[:limit - 3] + '...'
