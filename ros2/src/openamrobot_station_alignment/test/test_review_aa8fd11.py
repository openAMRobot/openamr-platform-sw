# SPDX-License-Identifier: MIT
# Tests for the review of platform-sw #167 at aa8fd11 (skunal3318, CHANGES_REQUESTED). Each test named
# test_kN_* covers point N of the review; all of them except k2 fail on aa8fd11 (k2 documents behaviour that
# aa8fd11 already had). Run without ROS: rosstub replaces the ROS imports.
import ast
import inspect
import json
import os
import types

import pytest

import rosstub

rosstub.install()

from test_review_c4666a1 import S, GoalHandle, Harness, camera  # noqa: E402

from openamrobot_station_alignment import evaluation, geometry, maneuver  # noqa: E402
from openamrobot_station_alignment import station_alignment_node as san  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- 1. correction count
def _report_from_node():
    """A real report from the node: 0.05 m along residual for 3 s, corrected, then aligned."""
    h = Harness(start_s=100.0)
    state = {"pose": (0.05, 0.0, 0.0)}
    h.camera = camera(5.0, 0.3, lambda: state["pose"], start_s=100.0)

    def correct(gh, residual, step):     # the node's own step bookkeeping, then the marker reads aligned
        step.update({"mode": "drive", "target_odom": [0.05, 0.0, 0.0], "moves": ["drive +0.0500 status 4"]})
        state["pose"] = (0.0, 0.0, 0.0)
        return True
    h._correct = correct
    gh = GoalHandle()
    h.execute(gh)
    return json.loads(h.report.msgs[-1].data)


def test_k1_corrections_are_counted_from_what_the_node_writes():
    report = _report_from_node()
    assert report["outcome"] == "aligned"
    assert evaluation.count_corrections(report) == 1
    assert not any("corrected_goal" in s for s in report["iterations"])


def test_k1_trials_uses_the_counting_function():
    src = open(os.path.join(PKG, "openamrobot_station_alignment", "trials.py")).read()
    assert "corrected_goal" not in src and "evaluation.count_corrections(" in src


def test_k1_count_from_the_real_correct_step():
    """_correct writes target_odom and moves into the step before it moves."""
    src = inspect.getsource(san.StationAlignment._correct)
    assert 'step["target_odom"]' in src or "target_odom=" in src


# ---------------------------------------------------------------- 2. overlapping goals
def test_k2_a_second_goal_is_rejected_in_goal_callback_and_accepted_after_the_first_ends():
    h = Harness()
    goal_cb = h.servers[0]["goal_callback"]
    assert goal_cb(object()) == rosstub.GoalResponse.ACCEPT
    assert goal_cb(object()) == rosstub.GoalResponse.REJECT
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.0)
    h.execute(GoalHandle())                       # the accepted goal runs and ends
    assert goal_cb(object()) == rosstub.GoalResponse.ACCEPT


# ---------------------------------------------------------------- 3 and 5. behaviour bounds
class FakeFuture:
    def __init__(self, value=None, done=False, clock=None, give_up_s=None):
        self.value, self._done = value, done
        self.clock, self.give_up_s = clock, give_up_s

    def done(self):
        if self.clock is not None and self.clock.ns > self.give_up_s * S:   # a node without a deadline waits forever
            raise AssertionError("still waiting for the behaviour result after 60 s of simulation time")
        return self._done

    def result(self):
        return self.value


class FakeHandle:
    def __init__(self, result_future):
        self.accepted = True
        self.cancelled = 0
        self.result_future = result_future

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled += 1


class FakeClient:
    def __init__(self, result_future):
        self.handle = FakeHandle(result_future)
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return FakeFuture(self.handle, done=True)


class Goal:
    def __init__(self):
        self.target = types.SimpleNamespace(x=None)
        self.speed = None
        self.time_allowance = None


def _behavior_harness(odom, monkeypatch, overrides=None):
    h = Harness(start_s=100.0, rtf=1.0, overrides=overrides)
    client = FakeClient(FakeFuture(done=False, clock=h.clock, give_up_s=160.0))   # accepted, never answers
    h.behaviors = {maneuver.BACKUP: client, maneuver.DRIVE: client}
    monkeypatch.setattr(san, "BackUp", types.SimpleNamespace(Goal=Goal))
    monkeypatch.setattr(san, "DriveOnHeading", types.SimpleNamespace(Goal=Goal))
    monkeypatch.setattr(san, "DurationMsg", lambda sec, nanosec: (sec, nanosec))
    h._odom_from_base = lambda: odom(h.clock.ns)
    return h, client


def test_k3_a_behaviour_without_result_is_cancelled_on_simulation_time(monkeypatch):
    # the base moves 0.05 m/s (no stall), but behavior_server never returns a result
    h, client = _behavior_harness(lambda ns: (0.05 * (ns - 100 * S) / S, 0.0, 0.0), monkeypatch)
    ok, text = h._behavior(GoalHandle(), maneuver.DRIVE, 0.15)
    assert not ok and "timed out" in text
    assert client.handle.cancelled >= 1
    allowance = maneuver.behavior_allowance(0.15, 0.05)
    elapsed = (h.clock.ns - 100 * S) / S
    assert allowance + 5.0 <= elapsed < allowance + 5.0 + 0.1


def test_k5_a_held_base_is_cancelled_as_stalled(monkeypatch):
    h, client = _behavior_harness(lambda ns: (0.001, 0.0, 0.0), monkeypatch)
    ok, text = h._behavior(GoalHandle(), maneuver.BACKUP, 0.15)
    assert not ok and "stalled" in text and client.handle.cancelled >= 1
    assert (h.clock.ns - 100 * S) / S < 4.5


def test_k5_time_allowance_follows_the_move_length(monkeypatch):
    h, client = _behavior_harness(lambda ns: (0.001, 0.0, 0.0), monkeypatch)
    h._behavior(GoalHandle(), maneuver.BACKUP, 0.15)
    sec, nanosec = client.goals[0].time_allowance
    assert sec + nanosec / 1e9 == pytest.approx(2 * 0.15 / 0.05 + 5.0)
    assert maneuver.behavior_allowance(0.01, 0.05) == pytest.approx(5.4)
    assert maneuver.behavior_allowance(10.0, 0.05) > 30.0     # the node caps it at behavior_time_allowance_s


def test_k5_progress_watch():
    w = maneuver.ProgressWatch((0.0, 0.0, 0.0), 0, window_s=4.0, min_move=0.005)
    assert not w.update((0.004, 0.0, 0.0), 3 * S)
    assert not w.update((0.006, 0.0, 0.0), 3.9 * S)       # progress resets the window
    assert not w.update((0.007, 0.0, 0.0), 7.8 * S)
    assert w.update((0.008, 0.0, 0.0), 8.0 * S)


# ---------------------------------------------------------------- 4. absolute rotation target
def test_k4_an_amcl_update_between_plan_and_goal_does_not_move_the_target_yaw():
    h = Harness()
    reads = iter([(1.0, 2.0, 0.20), (1.0, 2.0, 0.25)])     # plan reads 0.20, AMCL moves the heading to 0.25
    h._map_from_base = lambda: next(reads)
    plan = maneuver.Plan((0.0, 0.0, 0.0), 0.30, (0.0, 0.0, 0.1), lateral_direct=0.02, backoff=0.15)
    primitive = plan.next((0.0, 0.0, 0.0), h._map_from_base()[2])
    assert primitive[0] == maneuver.ROTATE
    h._rotate(GoalHandle(), primitive[2])
    assert h.nav_calls[-1] == (1.0, 2.0, pytest.approx(0.30))


def test_k4_face_turn_carries_its_absolute_target():
    plan = maneuver.Plan((0.0, 0.1, 0.0), 0.0, (0.0, 0.1, 0.0), lateral_direct=0.02, backoff=0.15)
    plan.stages = ["face", "approach", "align"]      # after the back-off
    kind, delta, target = plan.next((0.0, 0.0, 0.0), 0.5)
    assert kind == maneuver.ROTATE and target == pytest.approx(geometry.wrap(0.5 + delta))


# ---------------------------------------------------------------- 6. detector lookup, counter lock
def test_k6_detector_does_one_lookup_per_tag_and_no_can_transform():
    src = open(os.path.join(PKG, "openamrobot_station_alignment", "station_marker_detector.py")).read()
    assert ".can_transform(" not in src   # the detector imports apriltag_msgs, so it is read, not imported
    fn = next(f for f in ast.walk(ast.parse(src)) if isinstance(f, ast.FunctionDef) and f.name == "process_pending")
    tries = [n for n in ast.walk(fn) if isinstance(n, ast.Try)]
    assert tries and any("self._tag(" in ast.unparse(t.body[0]) for t in tries)
    assert [h.type.id for t in tries for h in t.handlers] == ["TransformException"]


def test_k6_frame_rejected_is_incremented_under_the_lock():
    t = ast.parse(inspect.getsource(san.StationAlignment.ingest).strip().replace("\n    ", "\n"))
    for w in (n for n in ast.walk(t) if isinstance(n, ast.With)):
        if "frame_rejected" in ast.unparse(w):
            assert ast.unparse(w.items[0].context_expr) == "self.lock"
            return
    pytest.fail("frame_rejected is not changed inside `with self.lock`")
