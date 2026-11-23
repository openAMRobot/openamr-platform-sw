# SPDX-License-Identifier: MIT
"""What the alignment node returns when Nav2 ABORTS the station goal (platform-sw #192; rtf1 trial 5 on bc8aeae:
"Failed to make progress", Nav2 status 6, 0.65 rad of yaw left).

These pin behaviour the node already has; none of them needed a code change. Run without ROS (rosstub).

The contract they pin (Proposed, owner decision): after an aborted station goal only a fresh marker measurement
inside accept_along, accept_lateral and accept_yaw ends the goal SUCCEEDED. No marker, a stale marker, a residual
beyond the correction limit or one that corrections do not close all end it ABORTED, with or without
on_marker_not_seen: succeed_unaligned.
"""
import json

import rosstub

rosstub.install()

from test_review_c4666a1 import GoalHandle, Harness, camera  # noqa: E402

from openamrobot_station_alignment import station_alignment_node as san  # noqa: E402

ABORTED = "Nav2 status 6"


class AbortedStationGoal(Harness):
    """The station goal itself (the first Nav2 goal) is aborted; correction goals succeed."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self._map_from_base = lambda: (0.0, 0.0, 0.0)
        self._odom_from_base = lambda: (0.0, 0.0, 0.0)

    def _navigate(self, goal_handle, pose, frame, timeout_s=None):
        self.nav_calls.append(pose)
        return (False, ABORTED) if len(self.nav_calls) == 1 else (True, "Nav2 status 4")


def run(h):
    gh = GoalHandle()
    result = h.execute(gh)
    return gh, result, json.loads(h.report.msgs[-1].data)


def limits(h):
    return {k: h.get_parameter(k).value for k in ("accept_along", "accept_lateral", "accept_yaw",
                                                   "max_correction_xy", "max_correction_yaw")}


def test_an_aborted_station_goal_without_a_marker_is_aborted_with_both_reasons():
    h = AbortedStationGoal(start_s=100.0)
    gh, result, report = run(h)
    assert gh.state == "aborted"
    assert report["outcome"].startswith(f"aborted: {ABORTED}; marker not seen")
    assert report["iterations"][0]["nav2"] == ABORTED
    assert len(h.nav_calls) == 1, "nothing is commanded after the aborted goal without a measurement"


def test_succeed_unaligned_does_not_turn_an_aborted_station_goal_into_a_success():
    h = AbortedStationGoal(start_s=100.0, overrides={"on_marker_not_seen": "succeed_unaligned"})
    gh, result, report = run(h)
    assert gh.state == "aborted" and result.error_code != san.NOT_ALIGNED
    assert report["outcome"].startswith(f"aborted: {ABORTED}")


def test_an_aborted_station_goal_with_only_stale_marker_poses_is_aborted():
    h = AbortedStationGoal(start_s=100.0)
    # A perfect pose, but every image arrives 30 s after its stamp: older than max_latency_s (15 s).
    h.camera = camera(5.0, 30.0, (0.0, 0.0, 0.0), start_s=60.0)
    gh, result, report = run(h)
    assert gh.state == "aborted" and report["outcome"].startswith(f"aborted: {ABORTED}")
    assert "no fresh marker sample" in report["outcome"] or "marker not seen" in report["outcome"]
    assert len(h.nav_calls) == 1


def test_after_an_abort_only_a_fresh_marker_residual_inside_the_acceptance_limits_succeeds():
    h = AbortedStationGoal(start_s=100.0)
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)
    gh, result, report = run(h)
    assert gh.state == "succeeded" and report["outcome"] == "aligned"
    # The abort stays on record for the client and the evidence.
    assert report["iterations"][0]["nav2"] == ABORTED
    assert len(h.nav_calls) == 1


def test_the_yaw_an_aborted_goal_left_is_never_accepted_and_not_corrected_beyond_the_limit():
    """rtf1 trial 5 left 0.65 rad. max_correction_yaw (0.30) bounds what the marker correction may command; it is
    not a bound on what Nav2 leaves. Beyond it the node moves nothing and aborts."""
    h = AbortedStationGoal(start_s=100.0)
    assert 0.65 > limits(h)["max_correction_yaw"] > limits(h)["accept_yaw"]
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.65), start_s=100.5)
    gh, result, report = run(h)
    assert gh.state == "aborted" and "beyond the correction limit" in report["outcome"]
    assert len(h.nav_calls) == 1


def test_a_residual_between_acceptance_and_the_correction_limit_is_corrected_and_not_accepted_as_it_is():
    h = AbortedStationGoal(start_s=100.0)
    yaw = 0.10
    assert limits(h)["accept_yaw"] < yaw < limits(h)["max_correction_yaw"]
    h.camera = camera(5.0, 0.3, (0.0, 0.0, yaw), start_s=100.5)     # the marker never reports the error closed
    gh, result, report = run(h)
    assert gh.state == "aborted" and "corrections" in report["outcome"]
    assert len(h.nav_calls) > 1, "corrections were commanded"
    measured = [s for s in report["iterations"] if "residual" in s]
    assert len(measured) == int(h.get_parameter("max_iterations").value) + 1


def test_a_position_error_beyond_the_acceptance_limits_is_not_accepted_after_an_abort():
    h = AbortedStationGoal(start_s=100.0)
    along = limits(h)["accept_along"] + 0.005
    h.camera = camera(5.0, 0.3, (along, 0.0, 0.0), start_s=100.5)
    gh, result, report = run(h)
    assert gh.state == "aborted"
    assert len(h.nav_calls) > 1
