# SPDX-License-Identifier: MIT
"""The sample window kept open at the measured rate (Proposed, 20 Nov 2026; review of #167).

Fast DDS rtf02 (7 s test image delay, RTF 0.2): runs 44464798587, 44464850846 and 44588689142 each ended one trial
"not aligned: no fresh marker sample", for example 6 received, 3 fresh, latency 7.05 s, at
timeout_s + max_latency_s = 21 s of simulation time. Only about 17 % of the camera images reached
/base_camera/color/camera_info_synced on that leg (43 of 259 in run 44265339608), against about 87 % with CycloneDDS
(234 of 268 in run 44227126188). So the station poses arrive every 3 to 5 s of simulation time and the fourth fresh
one is due about one period after the window closes. The rule keeps waiting while fresh samples still arrive at the
window's own rate; it never counts a stale sample, never lowers min_samples and stops at max_extension_s.
"""
import os

import yaml

from openamrobot_station_alignment import sampling

S = 1_000_000_000
POSE = (0.03, 0.0, 0.01)
AFTER = 31.3
LATENCY = 7.05
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def window(**kw):
    return sampling.SampleWindow(int(AFTER * S), 8, 6.0, 15.0, min_samples=4, **kw)


def drive(w, arrivals, until_s, step_s=0.01):
    """Feed (received_s after AFTER, fresh) samples with the 7.05 s latency and poll every step; -> (status, t)."""
    pending = sorted(arrivals)
    t, i = 0.0, 0
    while t <= until_s:
        while i < len(pending) and pending[i][0] <= t:
            received, fresh = pending[i]
            stamp = AFTER + received - LATENCY if fresh else min(AFTER + received - LATENCY, AFTER - 0.5)
            assert (stamp > AFTER) == fresh
            w.add(int(stamp * S), int((AFTER + received) * S), POSE)
            i += 1
        status = w.poll(int((AFTER + t) * S))
        if status != sampling.WAITING:
            return status, t
        t += step_s
    return sampling.WAITING, t


# The recorded shape: 3 stale arrivals, then 3 fresh ones before 21 s, the 4th at 23.5 s.
RECORDED = [(1.0, False), (3.5, False), (6.0, False), (9.0, True), (14.0, True), (19.5, True), (23.5, True)]


def test_without_the_rule_the_recorded_window_ends_stale_with_3_fresh_samples():
    status, t = drive(window(), RECORDED, 40.0)
    w = window()
    drive(w, RECORDED, 21.0)
    assert status == sampling.STALE and 21.0 < t < 21.1
    assert (w.received, len(w.fresh)) == (6, 3)


def test_with_the_rule_the_fourth_fresh_sample_is_taken():
    """revert: sampling.py without period_margin ends this window STALE at 21 s (test above)."""
    w = window(period_margin=1.5, max_extension_s=15.0)
    status, t = drive(w, RECORDED, 40.0)
    assert status == sampling.DONE and 23.5 <= t < 23.6
    assert len(w.fresh) == 4 and abs(w.summary()["period_s"] - 3.75) < 1e-6   # (23.5 - 1.0) / 6


def test_a_stream_of_stale_samples_is_not_extended():
    old = [(1.0 + 0.5 * k, False) for k in range(13)]                   # stamps all before AFTER
    status, t = drive(window(period_margin=1.5, max_extension_s=15.0), old, 60.0)
    assert status == sampling.STALE and 21.0 < t < 21.1


def test_nothing_received_is_still_not_seen_at_the_timeout():
    status, t = drive(window(period_margin=1.5, max_extension_s=15.0), [], 60.0)
    assert status == sampling.NOT_SEEN and 6.0 < t < 6.1


def test_fresh_samples_that_stop_end_stale_one_and_a_half_periods_after_the_last():
    stopped = RECORDED[:-1]                                             # no 4th fresh sample comes
    status, t = drive(window(period_margin=1.5, max_extension_s=15.0), stopped, 60.0)
    assert status == sampling.STALE
    assert abs(t - (19.5 + 1.5 * 3.7)) < 0.02                           # 25.05 s, not 21 s and not 36 s


def test_the_extension_never_passes_max_extension_s():
    slow = [(1.0, False), (8.0, True)] + [(8.0 + 12.0 * k, True) for k in range(1, 4)]   # 12 s period
    status, t = drive(window(period_margin=1.5, max_extension_s=15.0), slow, 80.0)
    assert status == sampling.STALE and 36.0 < t < 36.1                 # 6 + 15 + 15, the 4th fresh is at 44 s
    status, t = drive(window(period_margin=1.5, max_extension_s=5.0), slow, 80.0)
    assert status == sampling.STALE and 26.0 < t < 26.1                 # 6 + 15 + 5


def test_a_stale_sample_during_the_extension_does_not_count():
    late_stale = RECORDED[:-1] + [(22.0, False), (23.0, False)]
    w = window(period_margin=1.5, max_extension_s=15.0)
    status, _ = drive(w, late_stale, 60.0)
    assert status == sampling.STALE and len(w.fresh) == 3


def test_the_rule_is_off_unless_configured_and_configured_in_the_alignment():
    assert window().period_margin == 0.0 and window().max_extension_ns == 0
    params = yaml.safe_load(open(os.path.join(PKG, "config", "station_alignment.yaml")))
    params = params["station_alignment"]["ros__parameters"]
    assert (params["period_margin"], params["max_extension_s"], params["min_samples"]) == (1.5, 15.0, 4)
    node = open(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py")).read()
    assert 'float(self.get_parameter("period_margin").value)' in node
    assert 'float(self.get_parameter("max_extension_s").value)' in node
