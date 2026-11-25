"""Unit tests of the simref readiness gate and its failure report (no ROS, no Gazebo)."""
from readiness import Condition, evaluate, format_report, wait_ready


class Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


def cond(name, state, expected, holds):
    """Condition reading state[name] = (value, updated)."""
    return Condition(name, expected, lambda: state[name], holds)


def test_missing_stale_wrong_and_ok_are_told_apart():
    state = {'a': (None, None), 'b': ('RUN', 99.0), 'c': ('RUN', 80.0), 'd': ('IDLE', 99.0)}
    conds = [cond(n, state, "'IDLE'", lambda v: v == 'IDLE') for n in 'abcd']
    rows = {r['name']: r for r in evaluate(conds, now=100.0)}
    assert rows['a']['status'] == 'missing' and rows['a']['age'] is None
    assert rows['b']['status'] == 'wrong' and abs(rows['b']['age'] - 1.0) < 1e-9
    assert rows['c']['status'] == 'stale' and abs(rows['c']['age'] - 20.0) < 1e-9
    assert rows['d']['status'] == 'ok' and rows['d']['ok']


def test_a_stale_value_that_satisfies_the_requirement_is_still_ok():
    # age only chooses the label of a failing condition, it never fails a passing one
    state = {'a': ('IDLE', 1.0)}
    rows = evaluate([cond('a', state, "'IDLE'", lambda v: v == 'IDLE')], now=100.0)
    assert rows[0]['ok'] and rows[0]['status'] == 'ok'


def test_gate_needs_every_condition_at_once_and_does_not_fake_success():
    clock = Clock()
    state = {'a': ('IDLE', 100.0), 'b': (None, None)}
    conds = [cond('a', state, 'IDLE', lambda v: v == 'IDLE'), cond('b', state, '5', lambda v: v == 5)]
    ok, rows, waited = wait_ready(conds, timeout=10.0, step=0.5, clock=clock.now, sleep=clock.sleep)
    assert not ok and not rows[1]['ok'] and rows[0]['ok']
    assert 10.0 <= waited < 10.6        # gave up at the limit it was given, no extension


def test_gate_passes_as_soon_as_the_last_condition_holds():
    clock = Clock()
    state = {'a': (None, None)}

    def observe():
        if clock.t >= 103.0:
            state['a'] = (5, clock.t)
        return state['a']

    ok, rows, waited = wait_ready([Condition('a', '5', observe, lambda v: v == 5)], timeout=10.0,
                                  step=0.5, clock=clock.now, sleep=clock.sleep)
    assert ok and rows[0]['ok'] and 3.0 <= waited < 3.6


def test_a_failing_observer_is_a_missing_condition_not_a_crash():
    def broken():
        raise KeyError('drive')

    rows = evaluate([Condition('x', '1', broken, lambda v: True)], now=1.0)
    assert rows[0]['status'] == 'missing' and not rows[0]['ok']


def test_report_names_every_condition_with_expected_observed_and_age():
    state = {'drive phase': ('LOCKED', 97.5), 'lift NMT state': (None, None),
             'drive feedback': ('gazebo', 99.0)}
    conds = [cond('drive phase', state, "'IDLE'", lambda v: v == 'IDLE'),
             cond('lift NMT state', state, '5', lambda v: v == 5),
             cond('drive feedback', state, "'gazebo'", lambda v: v == 'gazebo')]
    text = format_report(evaluate(conds, now=100.0), waited=240.2, timeout=240.0)
    assert '1 of 3 conditions hold after waiting 240.2 s (limit 240.0 s)' in text
    line = {n: [ln for ln in text.splitlines() if n in ln][0] for n in state}
    assert 'WRONG' in line['drive phase'] and "'IDLE'" in line['drive phase'] \
        and "'LOCKED'" in line['drive phase'] and '2.5 s ago' in line['drive phase']
    assert 'MISSING' in line['lift NMT state'] and 'never received' in line['lift NMT state']
    assert 'OK' in line['drive feedback']
