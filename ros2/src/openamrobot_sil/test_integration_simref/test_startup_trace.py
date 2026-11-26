"""Unit tests of the start-up trace helpers (no ROS, no Gazebo)."""
from startup_trace import (cpu_percent, FirstSecondCounter, GraphWatcher, LineCapture,
                           StartupTrace)


class Ticks:
    def __init__(self):
        self.t = 5.0

    def clock(self):
        self.t += 0.25
        return self.t


def test_trace_is_ordered_and_names_each_observer_with_absolute_and_relative_time():
    ticks = Ticks()
    trace = StartupTrace(clock=ticks.clock, wall=lambda: 1000.5)
    trace.mark('fixture', 'launch starting')
    trace.mark('probe', 'first joint_states stamp=0.020 lift_m=0.000000')
    text = trace.format().splitlines()
    assert text[0].startswith('startup trace: 2 events')
    assert 'mono_ms=5250 +0' in text[1] and 'fixture' in text[1] and 'launch starting' in text[1]
    assert 'mono_ms=5500 +250' in text[2] and 'wall=1000.500' in text[2] and 'probe' in text[2]


def test_trace_with_no_events_says_so():
    assert StartupTrace().format() == 'startup trace: no events'


def test_first_second_counter_counts_only_the_first_window_and_keeps_stamps():
    c = FirstSecondCounter(window_s=1.0)
    assert c.add(10.0, '1.000') is True
    for i in range(1, 100):
        assert c.add(10.0 + i * 0.01, '%.3f' % (1.0 + i * 0.01)) is False
    c.add(11.5, '2.500')                      # outside the window, not counted
    assert c.count == 100 and c.done(11.5) and not c.done(10.9)
    assert c.summary() == '100 messages in the first 1.0 s, header stamps 1.000 .. 1.990'
    assert FirstSecondCounter().summary() == 'no messages'


def test_graph_watcher_reports_only_changes_and_ignores_order():
    g = GraphWatcher()
    assert g.observe(0, ()) is True
    assert g.observe(0, ()) is False
    assert g.observe(1, ('parameter_bridge RELIABLE VOLATILE',)) is True
    assert g.observe(1, ('parameter_bridge RELIABLE VOLATILE',)) is False
    assert g.observe(2, ('b RELIABLE VOLATILE', 'a RELIABLE VOLATILE')) is True
    assert g.observe(2, ('a RELIABLE VOLATILE', 'b RELIABLE VOLATILE')) is False


def test_cpu_percent_from_two_proc_stat_lines():
    prev = 'cpu  100 0 100 700 0 0 0 0 0 0'
    cur = 'cpu  200 0 200 800 100 0 0 100 0 0'        # +100 user, +100 system, +100 idle, +100 iowait, +100 steal
    got = cpu_percent(prev, cur)
    assert abs(got['busy'] - 60.0) < 1e-9 and abs(got['iowait'] - 20.0) < 1e-9
    assert abs(got['steal'] - 20.0) < 1e-9
    assert cpu_percent(prev, prev) is None and cpu_percent('x', cur) is None


def test_line_capture_keeps_timestamps_and_respects_its_cap():
    cap = LineCapture(max_lines=2)
    cap.add(10, 'header {\n')
    cap.add(11, 'sec: 0')
    cap.add(12, 'dropped')
    cap.add(13, 'dropped too')
    assert cap.format().splitlines() == ['10 header {', '11 sec: 0',
                                         '# 2 further lines dropped (cap 2)']
