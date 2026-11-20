# SPDX-License-Identifier: MIT
"""Inputs of the alignment and their waits (openamrobot_station_alignment.sources).

Baseline 1 runs 44247720886 (odom -> base_footprint "not part of the same tree", lookup without wait) and
44247729473 (0 station poses received in the alignment while the detector, another process, published them).
"""
import ast
import os

from openamrobot_station_alignment import sources

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
S = 1_000_000_000


class SimClock:
    """Simulation time that advances 0.01 s per poll (a real-time factor of 0.2 with 50 ms polls)."""

    def __init__(self):
        self.ns = 100 * S

    def now(self):
        return self.ns

    def sleep(self):
        self.ns += S // 100


def test_odom_from_the_message_when_fresh_and_of_the_right_frames():
    latest = (10 * S, "odom", "base_footprint", (1.0, 2.0, 0.3))
    assert sources.fresh_odom(latest, 10 * S + S // 4, 0.5, "odom", "base_footprint") == (1.0, 2.0, 0.3)
    assert sources.fresh_odom(latest, 11 * S, 0.5, "odom", "base_footprint") is None
    assert sources.fresh_odom((10 * S, "odom", "base_link", (0, 0, 0)), 10 * S, 0.5, "odom", "base_footprint") is None
    assert sources.fresh_odom(None, 0, 0.5, "odom", "base_footprint") is None


def test_a_lookup_that_succeeds_late_is_waited_for_on_simulation_time():
    clock = SimClock()
    calls = []

    def get():
        calls.append(clock.now())
        return (0.0, 0.0, 0.0) if len(calls) > 120 else None   # available after 1.2 s of simulation time

    value, waited = sources.wait_for(get, clock.now, 5.0, clock.sleep)
    assert value == (0.0, 0.0, 0.0) and 1.1 < waited < 1.3


def test_a_lookup_that_never_succeeds_fails_after_the_timeout_in_simulation_time():
    clock = SimClock()
    value, waited = sources.wait_for(lambda: None, clock.now, 5.0, clock.sleep)
    assert value is None and 5.0 < waited < 5.1


def test_local_sink_hands_over_every_pose():
    got = []
    sink = sources.LocalSink()
    sink.connect(lambda stamp, frame, pose: got.append((stamp, frame, pose)))
    sink(5 * S, "base_footprint", (0.01, 0.0, 0.0))
    assert got == [(5 * S, "base_footprint", (0.01, 0.0, 0.0))]


def _calls(path):
    with open(path, encoding="utf-8") as f:
        return ast.parse(f.read())


def test_the_stack_wires_the_detector_to_the_alignment_in_process():
    source = ast.unparse(_calls(os.path.join(PKG, "openamrobot_station_alignment", "stack.py")))
    assert "sink.connect(alignment.ingest)" in source and "detector.local_sink = sink" in source
    assert "'station_pose_source', Parameter.Type.STRING, 'local'" in source


def test_the_detector_hands_over_and_the_alignment_waits():
    detector = ast.unparse(_calls(os.path.join(PKG, "openamrobot_station_alignment", "station_marker_detector.py")))
    assert "self.local_sink(" in detector
    node = ast.unparse(_calls(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py")))
    assert "sources.wait_for(" in node and "sources.fresh_odom(" in node
