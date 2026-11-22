# SPDX-License-Identifier: MIT
# Tests for Kunal's review of platform-sw #167 at ea8ed13 (skunal3318, COMMENTED, review 5986320877). A test named
# test_pN_* covers point N and fails with the package code of ea8ed13 (the CI revert step); test_keep_* documents
# behaviour that ea8ed13 already had and holds on both. Points 6 (test_command_authority.py) and the reach region
# (test_reach_region_proposal.py) are test-only and live in their own files. Run without ROS: rosstub replaces the
# ROS imports; the node's own methods run against a fake Nav2 on the fake simulation clock.
import glob
import importlib.util
import json
import math
import os
import shutil
import types

import pytest

import rosstub

rosstub.install()

from test_review_c4666a1 import BASE, S, GoalHandle, Harness, camera  # noqa: E402
from test_review_aa8fd11_lead import NAV2, PR90, Future, real_nav_harness, run  # noqa: E402

from openamrobot_station_alignment import geometry, sampling, scene, sources  # noqa: E402
from openamrobot_station_alignment import station_alignment_node as san  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATION_TREE = os.path.join(NAV2, "behavior_trees", "navigate_to_station.xml")
FIXTURES = os.path.join(PKG, "test", "fixtures", "trees_with_backup")


def _yaw(goal):
    o = goal.pose.pose.orientation
    return geometry.yaw_of_quaternion(o.x, o.y, o.z, o.w)


class Nav:
    """navigate_to_pose with the behaviour of bt_navigator around a cancel: `script(index, nav)` gives
    (duration_s, status) or None (never ends). A cancel is processed `cancel_delay_s` later (then the cancel response
    and the CANCELED status arrive); a goal sent while a cancel is pending is rejected."""

    def __init__(self, harness, script, cancel_delay_s=0.0, cancel_never=False):
        self.h, self.script = harness, script
        self.cancel_delay_ns, self.cancel_never = int(cancel_delay_s * S), cancel_never
        self.goals, self.rejected, self.cancels, self.pending = [], [], [], []

    def wait_for_server(self, timeout_sec):
        return True

    def _cancel_pending(self):
        now = self.h.clock.ns
        return any(self.cancel_never or now < t for t in self.pending)

    def send_goal_async(self, goal, feedback_callback=None):
        if self._cancel_pending():
            self.rejected.append((self.h.clock.ns, goal))
            return Future(value=types.SimpleNamespace(accepted=False))
        index = len(self.goals)
        self.goals.append((self.h.clock.ns, goal))
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
                done_at = nav.h.clock.ns + nav.cancel_delay_ns
                nav.cancels.append((index, nav.h.clock.ns))
                nav.pending.append(done_at)
                if not nav.cancel_never:
                    state["end"], state["status"] = done_at, san.STATUS_CANCELED
                return Future(lambda: not nav.cancel_never and nav.h.clock.ns >= done_at)
        return Future(value=Handle())


def harness(monkeypatch, script, overrides=None, **nav):
    h = real_nav_harness(monkeypatch, script, overrides=overrides)
    h.nav = Nav(h, script, **nav)
    return h


class TreeGoalHandle(GoalHandle):
    def __init__(self, tree):
        super().__init__()
        self.request.behavior_tree = tree


def errors(h):
    return [m for level, m in getattr(h, "logs", []) if level == "error"]


def warnings(h):
    return [m for level, m in getattr(h, "logs", []) if level == "warn"]


# ---------------------------------------------------------------- 1. station tree
def test_p1_only_a_missing_package_gives_the_empty_default_and_nothing_else_is_swallowed(monkeypatch):
    def missing(name):
        raise rosstub.PackageNotFoundError(name)
    monkeypatch.setattr(san, "get_package_share_directory", missing)
    assert san.default_station_tree() == ""

    def broken(name):
        raise RuntimeError("ament index unreadable")
    monkeypatch.setattr(san, "get_package_share_directory", broken)
    with pytest.raises(RuntimeError):
        san.default_station_tree()


@pytest.mark.parametrize("tree", ["", "/nonexistent/behavior_trees/navigate_to_station.xml"])
def test_p1_without_a_station_tree_the_node_logs_an_error_and_aborts_9901_before_nav2(monkeypatch, tree):
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED), overrides={"behavior_tree": tree})
    assert errors(h) and "9901" in errors(h)[0]              # at start
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.0)
    gh, result, report = run(h)
    assert gh.state == "aborted" and result.error_code == san.STATION_TREE_MISSING == 9901
    assert report["outcome"].startswith("rejected:") and h.nav.goals == []
    assert len(errors(h)) == 2                               # and for the goal


def test_p1_a_goal_naming_another_behavior_tree_is_refused_9902_before_nav2(monkeypatch):
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.0)
    gh = TreeGoalHandle("/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
                        "navigate_to_pose_w_replanning_and_recovery.xml")
    result = h.execute(gh)
    report = json.loads(h.report.msgs[-1].data)
    assert gh.state == "aborted" and result.error_code == san.BEHAVIOR_TREE_MISMATCH == 9902
    assert "is not the configured station tree" in report["outcome"] and h.nav.goals == []
    assert errors(h)


@pytest.mark.parametrize("tree", ["", STATION_TREE,
                                  os.path.join(NAV2, "behavior_trees", ".", "navigate_to_station.xml")])
def test_keep_an_empty_or_the_configured_tree_runs_with_the_configured_tree(monkeypatch, tree):
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.0)
    gh = TreeGoalHandle(tree)
    h.execute(gh)
    report = json.loads(h.report.msgs[-1].data)
    assert gh.state == "succeeded" and report["outcome"] == "aligned"
    assert [g.behavior_tree for _, g in h.nav.goals] == [STATION_TREE]


# ---------------------------------------------------------------- 2. failed rotation
def _lateral_correction(h, heading):
    """A lateral residual (turn first); the map heading follows `heading` (what AMCL reports)."""
    poses = {"p": (0.02, 0.045, 0.0)}
    h.camera = camera(5.0, 0.3, lambda: poses["p"], start_s=100.0)
    h._map_from_base = lambda: (0.0, 0.0, heading["yaw"])
    return poses


def test_p2_a_face_turn_aborted_mid_turn_turns_back_to_the_measurement_heading(monkeypatch):
    heading = {"yaw": 0.0}

    def script(i, nav):
        if i == 1:                                # the face turn: aborted part way (progress checker), at 0.4 rad
            heading["yaw"] = 0.4
            return (2.0, 6)
        if i == 2:                                # the recovery turn reaches the measurement heading
            heading["yaw"] = _yaw(nav.goals[-1][1])
            poses["p"] = (0.0, 0.0, 0.0)          # marker in view again, and aligned
        return (1.0, san.STATUS_SUCCEEDED)
    h = harness(monkeypatch, script)
    poses = _lateral_correction(h, heading)
    gh, result, report = run(h)
    step = report["iterations"][1]
    assert step["mode"] == "turn_first" and "status 6" in step["moves"][0]
    assert len(step["moves"]) == 2 and step["moves"][1].startswith("rotate to +0.0000")
    assert step["recovery"] == {"target_yaw_map": 0.0, "attempts": 1, "ok": True}
    face, back = h.nav.goals[1][1], h.nav.goals[2][1]
    assert _yaw(face) == pytest.approx(math.atan2(0.045, 0.02)) and _yaw(back) == pytest.approx(0.0)
    assert back.pose.pose.position.x == 0.0 and back.pose.pose.position.y == 0.0   # a turn in place, no drive
    assert report["outcome"] == "aligned" and gh.state == "succeeded"


def test_p2_recovery_is_bounded_and_ends_the_goal_9903(monkeypatch):
    heading = {"yaw": 0.0}

    def script(i, nav):
        if i >= 1:                                # every turn after the approach aborts part way
            heading["yaw"] = 0.4
            return (2.0, 6)
        return (1.0, san.STATUS_SUCCEEDED)
    h = harness(monkeypatch, script)
    _lateral_correction(h, heading)
    gh, result, report = run(h)
    assert len(h.nav.goals) == 1 + 1 + 2          # approach, face turn, two recovery turns; then nothing
    assert "heading not recovered to +0.0000 after 2 turns" in report["outcome"]
    assert gh.state == "aborted" and result.error_code == san.HEADING_NOT_RECOVERED == 9903
    assert all(g.pose.pose.position.x == 0.0 for _, g in h.nav.goals[1:])
    assert errors(h)


# ---------------------------------------------------------------- 3. stale windows
def test_p3_a_stale_window_is_reopened_and_the_next_window_measures(monkeypatch):
    first = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=101.5, stop_s=102.1)    # 3 fresh images, then a gap
    later = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=123.5)                  # images again after the stale end
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.camera = lambda now: first(now) + later(now)
    gh, result, report = run(h)
    assert report["outcome"] == "aligned"
    step = report["iterations"][1]
    assert step["window_statuses"] == [sampling.STALE, sampling.DONE] and step["window_attempts"] == 2


def test_p3_stale_and_not_seen_share_one_retry_budget(monkeypatch):
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    state = {"next": 100 * S}

    def old_images(now):                          # images keep arriving, all stamped long before the stop
        out = []
        while state["next"] <= now:
            out.append((state["next"], 50 * S, (0.0, 0.0, 0.0)))
            state["next"] += S // 5
        return out
    h.camera = old_images
    gh, result, report = run(h)
    step = report["iterations"][1]
    assert step["window_attempts"] == 3 and step["window_statuses"] == [sampling.STALE] * 3
    assert result.error_code == san.NOT_ALIGNED and report["outcome"].startswith("not aligned")


# ---------------------------------------------------------------- 4. odom at the image stamps
def test_p4_the_history_reaches_back_over_the_longest_documented_window():
    h = Harness(start_s=100.0)
    for k in range(int(30 * 50)):                 # 30 s of /odom at 50 Hz (400 entries held 8 s)
        h.latest_odom = (100 * S + k * S // 50, "odom", BASE, (k * 1e-3, 0.0, 0.0))
    newest = h.odom_history[-1][0]
    image = newest - int(21.0 * S)                # an image stamp at the far end of a 6 s + 15 s window
    pose = sources.odom_at(list(h.odom_history), image, 0.5)
    assert pose is not None and pose[0] == pytest.approx((image - 100 * S) / S * 50 * 1e-3, abs=1e-9)
    assert h.odom_horizon_s == pytest.approx(6.0 + 15.0 + 0.5 + 2.0)
    assert h.odom_history[0][0] >= newest - int(h.odom_horizon_s * S)


def test_p4_a_clock_reset_clears_the_odom_history():
    from collections import deque
    history = deque()
    for t in (100.0, 100.02, 100.04):
        sources.append_odom(history, int(t * S), (t, 0.0, 0.0), 23.5)
    sources.append_odom(history, int(1.0 * S), (9.0, 0.0, 0.0), 23.5)      # Gazebo restarted: the clock is back
    assert list(history) == [(1 * S, (9.0, 0.0, 0.0))]


def test_p4_without_odom_at_the_images_it_warns_measures_again_and_fails_9904(monkeypatch):
    h = harness(monkeypatch, lambda i, nav: (1.0, san.STATUS_SUCCEEDED))
    h.odom_rate_hz = None                         # no /odom pose recorded at all
    h.camera = camera(5.0, 0.3, (0.0, 0.045, 0.0), start_s=100.0)          # a correction is needed
    gh, result, report = run(h)
    assert len(h.nav.goals) == 1                  # the approach only: no correction from the current pose
    misses = [s for s in report["iterations"][1:] if "odom_miss" in s]
    assert len(misses) == 2 and all(s["odom_at_images"] is False for s in misses)
    assert misses[1]["window_opened_s"] > misses[0]["window_opened_s"]      # a new measurement window
    assert gh.state == "aborted" and result.error_code == san.ODOM_AT_IMAGES_MISSING == 9904
    assert len([w for w in warnings(h) if "no /odom pose within 0.5 s" in w]) == 2


def test_p4_a_miss_then_odom_again_corrects_from_the_pose_at_the_images(monkeypatch):
    poses = {"p": (0.0, 0.045, 0.0)}

    def script(i, nav):
        if i >= 1:
            poses["p"] = (0.0, 0.0, 0.0)
        return (1.0, san.STATUS_SUCCEEDED)
    h = harness(monkeypatch, script)
    h.odom_rate_hz = None
    h.camera = camera(5.0, 0.3, lambda: poses["p"], start_s=100.0)
    warn = h.get_logger

    def logger():                                 # /odom comes back right after the first miss is reported
        lg = warn()

        def w(m):
            lg.warn(m)
            h.odom_rate_hz = 50.0
        return types.SimpleNamespace(info=lg.info, warn=w, error=lg.error)
    h.get_logger = logger
    gh, result, report = run(h)
    first, second = report["iterations"][1], report["iterations"][2]
    assert "odom_miss" in first and "moves" not in first
    assert second["odom_at_images"] is True and second["moves"] and second["iteration"] == first["iteration"]
    assert report["outcome"] == "aligned"


# ---------------------------------------------------------------- 5. cancel, then the next goal
def test_p5_after_a_timed_out_goal_the_next_goal_waits_for_the_cancel_and_is_accepted(monkeypatch):
    h = harness(monkeypatch, lambda i, nav: None if i == 0 else (1.0, san.STATUS_SUCCEEDED),
                overrides={"correction_timeout_s": 5.0}, cancel_delay_s=2.0)
    gh = GoalHandle()
    ok, text = h._drive(gh, 0.05)                 # never answers: timed out, cancelled
    assert not ok and text.endswith("timed out after 5 s (simulation time)")
    (index, cancelled_at), = h.nav.cancels
    assert h.clock.ns >= cancelled_at + 2 * S     # returned only once bt_navigator processed the cancel
    ok, text = h._rotate(gh, 0.0)                 # what _correct sends next (the turn back)
    assert ok and h.nav.rejected == [] and len(h.nav.goals) == 2


def test_p5_a_cancel_that_is_never_confirmed_is_bounded_and_reported(monkeypatch):
    h = harness(monkeypatch, lambda i, nav: None, overrides={"correction_timeout_s": 5.0, "cancel_timeout_s": 3.0},
                cancel_never=True)
    ok, text = h._drive(GoalHandle(), 0.05)
    assert not ok and "cancel not confirmed in 3 s" in text
    (index, cancelled_at), = h.nav.cancels
    assert 3 * S <= h.clock.ns - cancelled_at < 3.2 * S
    assert [w for w in warnings(h) if "cancel not confirmed" in w]


# ---------------------------------------------------------------- 7. cylinders and the LiDAR plane
def _cell(grid, x, y):
    height = len(grid)
    col = int((x + scene.FLOOR[0] / 2) / scene.MAP_RESOLUTION)
    row = height - 1 - int((y + scene.FLOOR[1] / 2) / scene.MAP_RESOLUTION)
    return grid[row][col]


def test_p7_a_cylinder_below_the_lidar_plane_is_not_on_the_map_and_a_tall_one_is():
    short = (("pylon", 0.10, scene.LIDAR_PLANE_Z - 0.07, (1.5, 1.2)),)     # a 0.152 m pylon: under the scan
    tall = (("post", 0.10, 1.00, (1.5, 1.2)),)
    _, _, grid_short = scene.occupancy(boxes=(), cylinders=short)
    _, _, grid_tall = scene.occupancy(boxes=(), cylinders=tall)
    assert _cell(grid_short, 1.5, 1.2) == 254 and _cell(grid_tall, 1.5, 1.2) == 0
    assert all(v == 254 for row in grid_short for v in row)


# ---------------------------------------------------------------- 8. the #90 overlay strips a real BackUp
def _overlay():
    path = os.path.join(PKG, "scripts", "apply_pr90_behaviour_set.py")
    spec = importlib.util.spec_from_file_location("overlay", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nav2_copy(tmp_path, tree):
    nav2 = tmp_path / "openamrobot_nav2"
    shutil.copytree(os.path.join(NAV2, "config"), nav2 / "config")
    (nav2 / "behavior_trees").mkdir()
    shutil.copy(os.path.join(FIXTURES, tree), nav2 / "behavior_trees" / tree)
    return nav2


def test_keep_p8_the_overlay_removes_one_line_and_multi_line_backup_and_keeps_the_rest(tmp_path):
    nav2 = _nav2_copy(tmp_path, "recovery_with_backup.xml")
    before = open(os.path.join(FIXTURES, "recovery_with_backup.xml")).read()
    assert before.count("<BackUp ") == 2
    changes = _overlay().apply(PR90, str(nav2))
    after = open(nav2 / "behavior_trees" / "recovery_with_backup.xml").read()
    assert "<BackUp" not in after and "recovery_with_backup.xml: 2 BackUp removed" in changes
    for node in ("<Spin ", "<Wait ", "<GoalUpdated/>", "</RoundRobin>", "<ComputePathToPose "):
        assert after.count(node) == before.count(node) == 1, node
    import xml.etree.ElementTree as ET
    ET.fromstring(after.split("-->", 2)[-1])     # still well-formed XML


def test_p8_a_backup_the_pattern_cannot_remove_fails_the_overlay(tmp_path):
    nav2 = _nav2_copy(tmp_path, "paired_backup.xml")
    with pytest.raises(ValueError, match="BackUp left after stripping"):
        _overlay().apply(PR90, str(nav2))


def test_keep_p8_the_fixtures_are_the_only_trees_with_backup():
    trees = glob.glob(os.path.join(NAV2, "behavior_trees", "*.xml"))
    assert trees and not [t for t in trees if "<BackUp" in open(t).read()]
