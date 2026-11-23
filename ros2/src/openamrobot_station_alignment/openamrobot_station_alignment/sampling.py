# SPDX-License-Identifier: MIT
"""The marker sample window of the alignment, on simulation (ROS) time only. No ROS import.

A sample is a station pose with the stamp of its image. Only samples whose image was taken after `after` (the
robot has stopped and settled) count. The pipeline from the camera to the station pose has a latency: at a
real-time factor of 0.2 in Baseline 1 the stamps arriving at the alignment were more than 6 s of simulation time
old, so a fixed 6 s window that starts at `after` never saw a fresh sample although the detector published at 5 Hz
(openamr-upperbody-sw runs 44155397328, 44155429472, 44155442866).

Rules, all on the node clock (simulation time with use_sim_time):
- done: `samples` fresh samples arrived;
- not seen: nothing at all arrived in `timeout_s` after `after` (the marker is not in view);
- stale: samples arrive but none is fresh yet: keep waiting up to `timeout_s + max_latency_s` after `after`;
- fewer than `samples` but at least `min_samples` fresh samples once `timeout_s` has passed: done with those (Fast
  DDS delivered about 1 image in 7 to the detector in run 44261511618 rtf02, so 8 frames may not come in time);
- the latency (receive time minus stamp) of every sample is recorded for the report.
"""
import math

from openamrobot_station_alignment import geometry

DONE, WAITING, NOT_SEEN, STALE = "done", "waiting", "not_seen", "stale"


def pending_expired(received_ns, now_ns, timeout_s):
    """The detector's wait for the tag transforms of one image, counted from when the detection arrived, never from
    the image stamp: a stamp several seconds old (low real-time factor, a lagging camera pipeline) is not a reason to
    drop a detection whose transforms are on their way."""
    return now_ns - received_ns > int(timeout_s * 1e9)


def reopen(status, attempt, retries):
    """Open the window once more after `status`? Only when nothing at all arrived (images come in bursts over Fast
    DDS: run 44265339608 rtf02 trial 1, 0 poses in the 6 s after a correction, then the base was 0.012 m away)."""
    return status == NOT_SEEN and attempt < retries


def without_marker(iteration, nominal_ok):
    """What the alignment does when the window ends without a usable marker pose.

    "not_aligned" only right after a #164 approach that Nav2 reported succeeded and before any correction moved the
    base: the base is where #164 put it. Otherwise "abort": Nav2 failed (the base may be anywhere), or a correction
    already moved the base without a confirming measurement.
    """
    return "not_aligned" if iteration == 0 and nominal_ok else "abort"


class SampleWindow:
    def __init__(self, after_ns, samples, timeout_s, max_latency_s, min_samples=None):
        self.after_ns = int(after_ns)
        self.samples = int(samples)
        # never fewer than one: min_samples 0 would end "done" with no sample (review of c4666a1, 4f)
        self.min_samples = max(1, min(int(samples if min_samples is None else min_samples), self.samples))
        self.timeout_ns = int(timeout_s * 1e9)
        self.max_latency_ns = int(max_latency_s * 1e9)
        self.fresh = []
        self.fresh_stamps = []
        self.received = 0
        self.latencies = []

    def add(self, stamp_ns, received_ns, pose):
        """A sample received at `received_ns`; only samples received after the window opened are given here."""
        self.received += 1
        self.latencies.append((received_ns - stamp_ns) * 1e-9)
        if stamp_ns > self.after_ns:
            self.fresh.append(pose)
            self.fresh_stamps.append(stamp_ns)

    def poll(self, now_ns):
        if len(self.fresh) >= self.samples:
            return DONE
        elapsed = now_ns - self.after_ns
        if elapsed > self.timeout_ns and len(self.fresh) >= self.min_samples:
            return DONE
        if self.received == 0 and elapsed > self.timeout_ns:
            return NOT_SEEN
        if elapsed > self.timeout_ns + self.max_latency_ns:
            return STALE
        return WAITING

    def result(self):
        """(mean pose, xy spread) of the last `samples` fresh samples (fewer when done on min_samples). Also sets
        `spread_yaw`, the largest yaw deviation from the circular mean, and `stamp_ns`, the mean image stamp."""
        fresh = self.fresh[-self.samples:]
        m = geometry.mean_pose(fresh)
        self.spread_yaw = max(abs(geometry.wrap(p[2] - m[2])) for p in fresh)
        stamps = self.fresh_stamps[-self.samples:]
        self.stamp_ns = sum(stamps) // len(stamps)
        return m, max(math.hypot(p[0] - m[0], p[1] - m[1]) for p in fresh)

    def summary(self):
        lat = self.latencies
        return {"received": self.received, "fresh": len(self.fresh),
                "latency_s_min": min(lat) if lat else None, "latency_s_max": max(lat) if lat else None}
