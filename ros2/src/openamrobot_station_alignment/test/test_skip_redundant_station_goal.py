# SPDX-License-Identifier: MIT
"""The nominal station goal is skipped only when the base is already within the correction range (platform-sw
#192: a goal this close is exactly the one the graceful controller's path-length latch and the shared progress
checker fail on - rtf1 trial 5 and the traces on #192/#194). Verified on #192 that every correction the node sends
is itself a NavigateToPose goal on navigate_to_station.xml, through the same controller_server, velocity smoother,
collision monitor and Stop gate as the nominal goal - skipping the redundant nominal goal changes which goals are
sent, not what commands the base or which safety checks they go through.

The check itself is a single, non-blocking TF lookup, not _map_from_base / _tf_pose (which retries for up to
tf_wait_s): a 5 s stall deciding whether to skip a goal would defeat the point. test_the_check_does_not_use_the_
blocking_tf_wait pins that directly.

Run without ROS (rosstub). 2 Hz trace / rtf1 simulation verification is tracked separately on #192; this file pins
the decision logic only.
"""
import types

import rosstub

rosstub.install()

from openamrobot_station_alignment import geometry  # noqa: E402
from test_review_c4666a1 import GoalHandle, Harness, camera, run  # noqa: E402


def _tf(x, y, yaw):
    q = geometry.quaternion_of_yaw(yaw)
    return types.SimpleNamespace(
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y, z=0.0),
            rotation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])))


def test_the_nominal_goal_is_sent_when_the_base_is_far_from_the_target():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(1.0, 0.0, 0.0)   # well beyond max_correction_xy (0.15 m)
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)
    gh, result, report = run(h)
    assert len(h.nav_calls) == 1, "the nominal goal must still be sent when the base is not already close"
    assert "skipped" not in report["iterations"][0]


def test_the_nominal_goal_is_sent_when_the_request_frame_is_not_global_frame():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.0, 0.0, 0.0)   # exactly at the target
    gh = GoalHandle()
    gh.request.pose.header.frame_id = "odom"     # global_frame defaults to "map"
    result = h.execute(gh)
    assert len(h.nav_calls) == 1


def test_the_nominal_goal_is_sent_when_tf_is_not_available():
    h = Harness(start_s=100.0)
    # h.tf_buffer.lookup_transform is left as the Harness default, which raises TransformException.
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)
    gh, result, report = run(h)
    assert len(h.nav_calls) == 1


def test_the_check_does_not_use_the_blocking_tf_wait():
    """tf_wait_s (default 5.0 s simulation time) must not be spent deciding whether to skip - that would add a
    real wait to every goal just to decide whether to skip one. Confirmed by elapsed simulation time, not by
    reading the source: this is exactly the regression a blocking lookup caused when first tried."""
    h = Harness(start_s=100.0)
    # No TF: the check must return immediately, not retry for tf_wait_s before giving up.
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)
    before = h.clock.ns
    skip, why = h._skip_nominal_goal((0.0, 0.0, 0.0), "map")
    assert skip is False
    assert h.clock.ns == before, "the opportunistic check must not advance the clock waiting for TF"


def test_a_base_already_within_the_correction_range_skips_the_nominal_goal_and_a_fresh_marker_still_succeeds():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.0, 0.0)   # 0.05 m from target, inside 0.15 m
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)   # marker residual inside acceptance
    gh, result, report = run(h)
    assert len(h.nav_calls) == 0, "no Nav2 goal should be sent at all when the base is already this close"
    assert report["iterations"][0]["skipped"] is True
    assert gh.state == "succeeded" and report["outcome"] == "aligned"


def test_without_a_usable_marker_the_skip_is_withdrawn_and_the_nominal_goal_is_sent():
    """Review of 7e4251c: a skip with no marker left the base up to max_correction_xy from the station with no
    Nav2 goal sent. "not aligned" must keep meaning "where #164 stopped the base"."""
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.0, 0.0)
    # no camera: no marker sample ever arrives
    gh, result, report = run(h)
    assert len(h.nav_calls) == 1, "no usable marker: the nominal goal is sent as before the skip existed"
    assert "skipped" not in report["iterations"][0] and "no usable marker" in report["skip_withdrawn"]
    assert report["outcome"].startswith("not aligned")
    import openamrobot_station_alignment.station_alignment_node as san
    assert gh.state == "aborted" and result.error_code == san.NOT_ALIGNED


def test_succeed_unaligned_never_succeeds_on_a_skip_without_a_nominal_goal():
    h = Harness(start_s=100.0, overrides={"on_marker_not_seen": "succeed_unaligned"})
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.14, 0.0, 0.29)   # just inside the correction range
    gh, result, report = run(h)
    assert len(h.nav_calls) == 1, "the opt-in may only report success after Nav2 has run the station goal"


def test_only_stale_marker_poses_withdraw_the_skip():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.0, 0.0)
    h.camera = camera(5.0, 30.0, (0.0, 0.0, 0.0), start_s=60.0)   # every image 30 s old on arrival
    gh, result, report = run(h)
    assert len(h.nav_calls) == 1 and "skip_withdrawn" in report


def test_an_engaged_stop_ends_a_skipped_goal_before_anything_is_measured_or_sent():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.0, 0.0)
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=100.5)
    h.stop_state = rosstub.StopState.STATE_LOCKED
    gh, result, report = run(h)
    assert gh.state == "aborted" and report["outcome"].startswith("aborted: stopped")
    assert len(h.nav_calls) == 0


def test_a_base_within_the_correction_range_but_outside_acceptance_is_corrected_not_skipped_entirely():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.0, 0.0)
    # marker residual outside accept_along (0.025) but inside max_correction_xy (0.15): a real correction drive.
    h.camera = camera(5.0, 0.3, (0.05, 0.0, 0.0), start_s=100.5)
    corrections = []
    h._correct = lambda gh, residual, step: corrections.append(residual) or True
    gh, result, report = run(h)
    assert len(h.nav_calls) == 0, "the nominal goal is still skipped; the correction itself is the next call"
    assert corrections, "skipping the nominal goal must still reach the normal measure-and-correct loop"


def test_the_skip_residual_is_the_map_frame_offset_between_target_and_the_actual_pose():
    h = Harness(start_s=100.0)
    h.tf_buffer.lookup_transform = lambda *a: _tf(0.05, 0.02, 0.1)
    skip, why = h._skip_nominal_goal((0.0, 0.0, 0.0), "map")
    assert skip is True
    assert why.startswith("skipped: base already within the correction range ")
    # The yaw in the report round-trips through a quaternion, so compare the reported numbers loosely rather
    # than the exact formatted string.
    along, lateral, yaw = geometry.between((0.0, 0.0, 0.0), (0.05, 0.02, 0.1))
    assert round(along, 3) == 0.05 and round(lateral, 3) == 0.02 and round(yaw, 3) == 0.1
