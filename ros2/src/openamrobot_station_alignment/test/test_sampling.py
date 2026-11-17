# SPDX-License-Identifier: MIT
"""The alignment's marker sample window at a low real-time factor (openamrobot_station_alignment.sampling).

Reproduces Baseline 1 runs 44155397328, 44155429472 and 44155442866 (real-time factor about 0.2): the detector
published at 5 Hz during the window, but the station poses reached the alignment with image stamps more than 6 s
of simulation time old, and the 6 s window starting at `after` saw no fresh sample.
"""
from openamrobot_station_alignment import sampling

S = 1_000_000_000
POSE = (0.03, 0.0, 0.01)


def pipeline(after_s, latency_s, rate_hz=5.0, until_s=60.0, start_s=20.0):
    """(stamp_ns, received_ns) of station poses whose images are `latency_s` of simulation time old on arrival."""
    out, t = [], start_s
    while t < until_s:
        out.append((int(t * S), int((t + latency_s) * S)))
        t += 1.0 / rate_hz
    return out


def run_window(samples, after_s, rtf=0.2, wall_step_s=0.05, **kw):
    """Drive a SampleWindow the way the node does: poll every 50 ms of wall time, sim = rtf * wall."""
    w = sampling.SampleWindow(int(after_s * S), kw.get("n", 8), kw.get("timeout_s", 6.0), kw.get("max_latency_s", 15.0))
    now = after_s
    pending = sorted(samples, key=lambda s: s[1])
    i = 0
    while True:
        now += rtf * wall_step_s
        while i < len(pending) and pending[i][1] <= now * S:
            stamp, received = pending[i]
            if received >= after_s * S or stamp > after_s * S:
                w.add(stamp, received, POSE)
            i += 1
        status = w.poll(int(now * S))
        if status != sampling.WAITING:
            return status, w, now - after_s


def legacy_count(samples, after_s, timeout_s=6.0):
    """The rule of 353e3d6: samples stamped after `after` that arrived within timeout_s of `after`."""
    return sum(1 for stamp, received in samples if stamp > after_s * S and received <= (after_s + timeout_s) * S)


def test_baseline1_latency_gave_the_old_rule_no_sample():
    samples = pipeline(after_s=31.3, latency_s=7.0)
    assert legacy_count(samples, 31.3) == 0
    assert sum(1 for _, r in samples if 31.3 * S <= r <= 37.3 * S) == 30  # 5 Hz were arriving all the time


def test_the_window_waits_for_fresh_samples_at_rtf_0_2():
    status, w, waited = run_window(pipeline(after_s=31.3, latency_s=7.0), after_s=31.3)
    assert status == sampling.DONE
    assert len(w.fresh) == 8 and w.summary()["latency_s_min"] >= 6.9
    assert 7.0 < waited < 9.0  # latency plus 8 frames at 5 Hz, in simulation time


def test_without_latency_it_finishes_in_two_seconds_of_simulation_time():
    status, _, waited = run_window(pipeline(after_s=31.3, latency_s=0.05), after_s=31.3, rtf=1.0)
    assert status == sampling.DONE and waited < 2.0


def test_nothing_received_is_not_seen_after_the_timeout_in_simulation_time():
    status, w, waited = run_window([], after_s=31.3, rtf=0.05)
    assert status == sampling.NOT_SEEN and w.received == 0
    assert 6.0 < waited < 6.1  # simulation seconds, whatever the real-time factor


def test_only_stale_samples_end_as_stale_after_the_latency_allowance():
    old = [(int(10 * S), int(r * S)) for r in [31.5 + 0.2 * k for k in range(200)]]
    status, w, waited = run_window(old, after_s=31.3)
    assert status == sampling.STALE and w.received > 0 and not w.fresh
    assert 21.0 < waited < 21.2


def test_result_is_the_mean_of_the_last_samples():
    w = sampling.SampleWindow(0, 2, 6.0, 15.0)
    for k, pose in enumerate([(9.0, 9.0, 0.0), (0.01, 0.0, 0.0), (0.03, 0.0, 0.0)]):
        w.add((k + 1) * S, (k + 1) * S, pose)
    mean, spread = w.result()
    assert abs(mean[0] - 0.02) < 1e-12 and abs(spread - 0.01) < 1e-12
