# SPDX-License-Identifier: MIT
# Tests for the software lead's review of platform-sw #167 at aa8fd11 (items a, b, c). Every test named test_a*,
# test_b*, test_c* and the deadline test fail on a2bf64d; test_keep_* hold on both. Run without ROS: rosstub
# replaces the ROS imports; the node's own _navigate runs against a fake Nav2 on the fake simulation clock.
import ast
import glob
import importlib.util
import json
import os
import shutil
import types

import pytest
import yaml

import rosstub

rosstub.install()

from test_review_c4666a1 import S, GoalHandle, Harness, camera  # noqa: E402

from openamrobot_station_alignment import maneuver  # noqa: E402
from openamrobot_station_alignment import station_alignment_node as san  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2 = os.path.join(os.path.dirname(PKG), "openamrobot_nav2")
PR90 = os.path.join(PKG, "test", "fixtures", "pr90_4fad3d8_nav2_subset.yaml")
LOCKED, RUNNING = rosstub.StopState.STATE_LOCKED, rosstub.StopState.STATE_RUNNING


# ---------------------------------------------------------------- fake Nav2
class Goal:
    def __init__(self):
        ns = types.SimpleNamespace
        self.pose = ns(header=ns(frame_id=None, stamp=None),
                       pose=ns(position=ns(x=None, y=None), orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.behavior_tree = None


class Future:
    def __init__(self, done=lambda: True, value=None):
        self._done, self.value = done, value

    def done(self):
        return self._done()

    def result(self):
        return self.value() if callable(self.value) else self.value


class FakeNav:
    """navigate_to_pose: each goal is accepted; `script(index, nav)` returns (duration_s, status) or None (never
    ends). A cancel ends the goal with status 5."""

    def __init__(self, harness, script):
        self.h, self.script = harness, script
        self.goals, self.cancels = [], []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        index = len(self.goals)
        self.goals.append((self.h.clock.ns, goal.pose.pose.position.x, goal.pose.pose.position.y))
        plan = self.script(index, self)
        state = {"end": None if plan is None else self.h.clock.ns + int(plan[0] * S),
                 "status": None if plan is None else plan[1]}
        nav = self

        class Handle:
            accepted = True

            def get_result_async(self):
                return Future(lambda: state["end"] is not None and nav.h.clock.ns >= state["end"],
                              lambda: types.SimpleNamespace(status=state["status"]))

            def cancel_goal_async(self):
                nav.cancels.append(index)
                state["end"], state["status"] = nav.h.clock.ns, san.STATUS_CANCELED
        return Future(value=Handle())


def real_nav_harness(monkeypatch, script, start_s=100.0, overrides=None):
    h = Harness(start_s=start_s, overrides=overrides)
    h._navigate = types.MethodType(san.StationAlignment._navigate, h)      # the node's own, not the stub
    monkeypatch.setattr(san, "NavigateToPose", types.SimpleNamespace(Goal=Goal, Result=rosstub.Result))
    clock = h.clock
    h.clock.now = lambda: types.SimpleNamespace(nanoseconds=clock.ns, to_msg=lambda: None)
    h._map_from_base = lambda: (0.0, 0.0, 0.0)
    h._odom_from_base = lambda: (0.0, 0.0, 0.0)
    h.nav = FakeNav(h, script)
    return h


def run(h):
    gh = GoalHandle()
    result = h.execute(gh)
    return gh, result, json.loads(h.report.msgs[-1].data)


# ---------------------------------------------------------------- a. forward only, #90's behaviour set
def test_a_the_node_has_no_behavior_server_client_and_no_reverse_parameter():
    src = open(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py")).read()
    for name in ("BackUp", "DriveOnHeading", "AssistedTeleop", "backup_action", "drive_on_heading_action", "backoff"):
        assert name not in src, name
    config = yaml.safe_load(open(os.path.join(PKG, "config", "station_alignment.yaml")))
    params = config["station_alignment"]["ros__parameters"]
    assert not {"backoff", "backup_action", "drive_on_heading_action", "spin_action"} & set(params)


def test_a_a_lateral_correction_sends_only_navigate_to_pose_goals_ahead_of_the_robot(monkeypatch):
    poses = {"p": (0.02, 0.045, 0.0)}           # lateral residual above accept_lateral
    h = real_nav_harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.camera = camera(5.0, 0.3, lambda: poses["p"], start_s=100.0)
    drives = []
    real_drive = h._drive

    def drive(gh, distance):
        drives.append(distance)
        poses["p"] = (0.0, 0.0, 0.0)            # reached: the next measurement is aligned
        return real_drive(gh, distance)
    h._drive = drive
    gh, result, report = run(h)
    step = report["iterations"][1]
    assert report["outcome"] == "aligned" and step["mode"] == "turn_first"
    assert [m.split()[0] for m in step["moves"]] == ["rotate", "drive"]   # the stubbed map heading never turns
    assert drives and all(d > 0 for d in drives)
    # the drive goal lies ahead of the robot on its heading (the robot faces +x here), drive_lead beyond the point
    _, x, y = h.nav.goals[2]
    assert x == pytest.approx(drives[0] + 0.025) and y == pytest.approx(0.0)


def _load_overlay():
    path = os.path.join(PKG, "scripts", "apply_pr90_behaviour_set.py")
    spec = importlib.util.spec_from_file_location("overlay", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a_pr90_behaviour_set_leaves_every_action_the_alignment_and_its_trees_need(tmp_path):
    nav2 = tmp_path / "openamrobot_nav2"
    shutil.copytree(os.path.join(NAV2, "config"), nav2 / "config")
    shutil.copytree(os.path.join(NAV2, "behavior_trees"), nav2 / "behavior_trees")
    changes = _load_overlay().apply(PR90, str(nav2))
    params = yaml.safe_load(open(nav2 / "config" / "nav2_params.yaml"))
    behaviors = params["behavior_server"]["ros__parameters"]
    assert behaviors["behavior_plugins"] == ["spin", "wait"] and "backup" not in behaviors
    assert params["velocity_smoother"]["ros__parameters"]["min_velocity"][0] == 0.0
    assert params["controller_server"]["ros__parameters"]["FollowPath"]["min_vel_x"] == 0.0
    assert any("navigate_to_station.xml: 1 BackUp removed" == c for c in changes), changes
    # the BT nodes that call behavior_server must name a registered behaviour
    calls = {"Spin": "spin", "Wait": "wait", "BackUp": "backup", "DriveOnHeading": "drive_on_heading",
             "AssistedTeleop": "assisted_teleop"}
    for tree in glob.glob(str(nav2 / "behavior_trees" / "*.xml")):
        used = {calls[n] for n in calls if f"<{n} " in open(tree).read()}
        assert used <= set(behaviors["behavior_plugins"]), (tree, used)
    station = open(nav2 / "behavior_trees" / "navigate_to_station.xml").read()
    assert 'controller_id="StationApproach"' in station and 'goal_checker_id="station_goal_checker"' in station
    # the alignment itself needs navigate_to_pose only
    t = ast.parse(open(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py")).read())
    clients = [ast.unparse(c.args[1]) for c in ast.walk(t)
               if isinstance(c, ast.Call) and ast.unparse(c.func) == "ActionClient"]
    assert clients == ["NavigateToPose"]


def test_a_drive_lead_matches_the_station_goal_checker():
    params = yaml.safe_load(open(os.path.join(NAV2, "config", "nav2_params.yaml")))
    checker = params["controller_server"]["ros__parameters"]["station_goal_checker"]["xy_goal_tolerance"]
    config = yaml.safe_load(open(os.path.join(PKG, "config", "station_alignment.yaml")))
    assert config["station_alignment"]["ros__parameters"]["drive_lead"] == checker


# ---------------------------------------------------------------- b. Stop
def _correction_needed(h):
    h.camera = camera(5.0, 0.3, (0.0, 0.045, 0.0), start_s=100.0)      # lateral: a correction follows


def test_b_stop_mid_correction_ends_the_goal_and_nothing_is_sent_after_release(monkeypatch):
    def script(i, nav):
        if i == 1:                               # the first correction goal: Stop engages, the gate cancels it
            nav.h.stop_state = LOCKED
            nav.h.release_at = nav.h.clock.ns + 2 * S
            return (0.5, san.STATUS_CANCELED)
        return (1.0, san.STATUS_SUCCEEDED)
    h = real_nav_harness(monkeypatch, script)
    h.release_at = None
    _correction_needed(h)
    sleep = h._sleep

    def sleep_and_release(dt):                  # Stop is released 2 s of simulation time later
        sleep(dt)
        if h.release_at is not None and h.clock.ns >= h.release_at:
            h.stop_state = RUNNING
    san.time = types.SimpleNamespace(sleep=sleep_and_release)
    gh, result, report = run(h)
    assert gh.state == "aborted" and report["outcome"].startswith("aborted: stopped:"), report["outcome"]
    assert len(h.nav.goals) == 2
    for _ in range(2000):                       # keep the clock running well past the release: no goal
        sleep_and_release(0.05)
    assert h.stop_state == RUNNING and len(h.nav.goals) == 2


def test_b_a_canceled_sub_goal_ends_the_station_goal_without_a_stop_state(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: (0.5, san.STATUS_CANCELED if i == 1 else san.STATUS_SUCCEEDED))
    _correction_needed(h)
    gh, result, report = run(h)
    assert gh.state == "aborted" and "CANCELED" in report["outcome"] and len(h.nav.goals) == 2


def test_b_a_canceled_nominal_goal_is_not_measured(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: (0.5, san.STATUS_CANCELED))
    _correction_needed(h)
    gh, result, report = run(h)
    assert gh.state == "aborted" and len(report["iterations"]) == 0 and len(h.nav.goals) == 1


def test_b_stop_during_the_measurement_sends_no_correction(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    _correction_needed(h)
    sleep = h._sleep

    def sleep_and_stop(dt):
        sleep(dt)
        if h.clock.ns >= 102 * S:              # in the settle time / sample window after the approach
            h.stop_state = LOCKED
    san.time = types.SimpleNamespace(sleep=sleep_and_stop)
    gh, result, report = run(h)
    assert gh.state == "aborted" and "stop_state" in report["outcome"] and len(h.nav.goals) == 1


def test_b_stop_engaged_before_the_goal_sends_nothing(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.stop_state = LOCKED
    gh, result, report = run(h)
    assert gh.state == "aborted" and h.nav.goals == []


def test_keep_running_stop_state_does_not_stop(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.stop_state = RUNNING
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.0)
    gh, result, report = run(h)
    assert report["outcome"] == "aligned" and gh.state == "succeeded"


def test_b_own_cancel_is_a_cancel_not_a_stop(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: None)       # never ends unless cancelled
    gh = GoalHandle()
    gh.is_cancel_requested = True
    h.execute(gh)
    report = json.loads(h.report.msgs[-1].data)
    assert gh.state == "canceled" and "stopped" not in report["outcome"]


# ---------------------------------------------------------------- c. 9900
def test_c_marker_not_seen_aborts_with_9900_by_default():
    h = Harness(start_s=100.0)
    gh, result, report = run(h)
    assert gh.state == "aborted" and result.error_code == san.NOT_ALIGNED
    assert report["outcome"].startswith("not aligned")
    config = yaml.safe_load(open(os.path.join(PKG, "config", "station_alignment.yaml")))
    assert config["station_alignment"]["ros__parameters"]["on_marker_not_seen"] == "abort"


def test_keep_succeed_unaligned_is_opt_in():
    h = Harness(start_s=100.0, overrides={"on_marker_not_seen": "succeed_unaligned"})
    gh, result, report = run(h)
    assert gh.state == "succeeded" and result.error_code == san.NOT_ALIGNED


def test_correction_goals_use_the_correction_deadline(monkeypatch):
    h = real_nav_harness(monkeypatch, lambda i, nav: None)       # Nav2 never answers
    ok, text = h._drive(GoalHandle(), 0.03)
    assert not ok and "timed out after 60 s" in text and h.nav.cancels == [0]
    assert 60.0 <= (h.clock.ns - 100 * S) / S < 60.2
    assert maneuver.DRIVE == "drive"
