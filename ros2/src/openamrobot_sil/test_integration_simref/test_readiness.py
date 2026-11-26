"""Unit tests of the simref readiness gate and its failure report (no ROS, no Gazebo)."""
from readiness import Condition, evaluate, format_report, nav_row, wait_ready


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


def test_first_satisfied_time_is_recorded_per_condition():
    clock = Clock()
    state = {'a': ('x', 100.0), 'b': (None, None)}

    def observe_b():
        if clock.t >= 102.0:
            state['b'] = (5, clock.t)
        return state['b']

    conds = [cond('a', state, 'x', lambda v: v == 'x'), Condition('b', '5', observe_b, lambda v: v == 5)]
    ok, rows, waited = wait_ready(conds, timeout=10.0, step=0.5, clock=clock.now, sleep=clock.sleep)
    first = {r['name']: r['first_ok'] for r in rows}
    assert ok and first['a'] == 0.0 and 2.0 <= first['b'] < 2.6


def test_first_satisfied_time_is_kept_when_a_condition_later_drops_and_the_gate_fails():
    clock = Clock()

    def observe():
        return ('IDLE', clock.t) if clock.t < 101.0 else ('LOCKED', clock.t)

    never = Condition('never', '5', lambda: (None, None), lambda v: v == 5)
    ok, rows, waited = wait_ready([Condition('a', 'IDLE', observe, lambda v: v == 'IDLE'), never],
                                  timeout=5.0, step=0.5, clock=clock.now, sleep=clock.sleep)
    by = {r['name']: r for r in rows}
    assert not ok                                       # needs both at once, 'never' did not hold
    assert by['a']['first_ok'] == 0.0 and not by['a']['ok']     # held once, not at the end
    assert by['never']['first_ok'] is None


def test_success_report_shows_every_condition_ok_with_first_held_time():
    clock = Clock()
    state = {'a': ('IDLE', 100.0), 'b': (5, 100.0)}
    conds = [cond('a', state, "'IDLE'", lambda v: v == 'IDLE'), cond('b', state, '5', lambda v: v == 5)]
    ok, rows, waited = wait_ready(conds, timeout=10.0, step=0.5, clock=clock.now, sleep=clock.sleep)
    rows.append(nav_row(True, 1.2, 120.0))
    text = format_report(rows, waited, 240.0, passed=ok)
    lines = text.splitlines()
    assert lines[0].startswith('readiness gate PASSED: 3 of 3 conditions hold after waiting 0.0 s')
    assert all('OK' in ln for ln in lines[2:]) and 'FAILED' not in text
    assert 'first held' in lines[1] and '0.00 s' in lines[2]
    assert 'appeared after 1.2 s of 120.0 s' in lines[4]


def test_failure_report_says_failed_and_keeps_the_last_observed_values():
    clock = Clock()
    state = {'a': ('LOCKED', 100.0)}
    ok, rows, waited = wait_ready([cond('a', state, "'IDLE'", lambda v: v == 'IDLE')], timeout=2.0,
                                  step=0.5, clock=clock.now, sleep=clock.sleep)
    rows.append(nav_row(False, None, 120.0))
    text = format_report(rows, waited, 2.0, passed=ok)
    assert text.startswith('readiness gate FAILED: 0 of 2 conditions hold after waiting 2.0 s')
    assert "'LOCKED'" in text and 'WRONG' in text and 'not waited for: the gate failed first' in text


def test_nav_server_that_is_not_there_is_missing_whether_waited_for_or_not():
    waited = nav_row(False, 120.0, 120.0)
    peeked = nav_row(False, None, 120.0)
    ready = nav_row(True, 0.4, 120.0)
    assert waited['status'] == peeked['status'] == 'missing' and not waited['ok'] and not peeked['ok']
    assert 'did not appear in 120.0 s (limit 120.0 s)' in waited['note']
    assert ready['status'] == 'ok' and ready['ok']
