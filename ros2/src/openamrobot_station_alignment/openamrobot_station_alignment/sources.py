# SPDX-License-Identifier: MIT
"""Where the alignment takes its inputs from, and how long it waits for them. No ROS import.

Baseline 1 (Fast DDS, about 84 processes, real-time factor about 0.1) lost one input of the alignment node in two of
three runs while the publisher was running: the station poses of the detector (upperbody-sw run 44247729473: 0
received while the detector published 30 and 50 per 10 s, and the node's clock advanced), and the odom ->
base_footprint transform (run 44247720886: "not part of the same tree"; base_state_adapter is its only writer).
So:
- the detector and the alignment run in one process and the station pose is handed over in process
  (`station_alignment_stack`); the topic is still published for others;
- the odom pose comes from the /odom message of the same writer, falling back to /tf;
- every lookup waits, on simulation time, before it fails, and the failure says what was missing.
"""


MAX_FUTURE_S = 0.1   # a stamp further ahead of the clock than this is from another /clock generation


def within_age(stamp_ns, now_ns, max_age_s):
    """A stamp at most max_age_s old and not from the future (a /clock reset leaves old stamps ahead of now)."""
    age = now_ns - stamp_ns
    return -int(MAX_FUTURE_S * 1e9) <= age <= int(max_age_s * 1e9)


def fresh_odom(latest, now_ns, max_age_s, odom_frame, base_frame):
    """(x, y, yaw) of the latest /odom message if it is odom_frame -> base_frame, at most max_age_s old and not
    stamped in the future."""
    if latest is None:
        return None
    stamp_ns, frame, child, pose = latest
    if frame != odom_frame or child != base_frame:
        return None
    if not within_age(stamp_ns, now_ns, max_age_s):
        return None
    return pose


def drop_other_generation(samples, now_ns):
    """Station poses stamped or received after `now`: after a /clock reset (Gazebo reset or restart while the stack
    runs) the poses of the old generation are ahead of the new clock and would look fresh. Keeps the others."""
    limit = now_ns + int(MAX_FUTURE_S * 1e9)
    return [s for s in samples if s[1] <= limit and s[2] <= limit]


def history_horizon_s(sample_timeout_s, max_latency_s, tolerance_s, margin_s=2.0):
    """How far back the /odom history must reach when a correction starts: a fresh image is stamped after its window
    opened and the window closes at most sample_timeout_s + max_latency_s later (sampling.py); the pose may lie
    tolerance_s before the stamp; margin_s covers the time from the window's end to the lookup. Sized from the
    longest documented latency instead of a fixed count (400 entries held 8 s at 50 Hz; Kunal's review of ea8ed13,
    point 4)."""
    return float(sample_timeout_s) + float(max_latency_s) + float(tolerance_s) + float(margin_s)


def append_odom(history, stamp_ns, pose, horizon_s):
    """Add one /odom pose to `history` (a deque of (stamp_ns, pose)) and drop what lies more than horizon_s behind
    it. A stamp more than MAX_FUTURE_S behind the newest one is a new /clock generation: the old history is cleared."""
    if history and stamp_ns < history[-1][0] - int(MAX_FUTURE_S * 1e9):
        history.clear()
    history.append((stamp_ns, pose))
    oldest = stamp_ns - int(horizon_s * 1e9)
    while history and history[0][0] < oldest:
        history.popleft()


def odom_at(history, stamp_ns, tolerance_s):
    """The odom pose at the image stamp: the /odom pose nearest to `stamp_ns` within tolerance_s, else None.
    history: (stamp_ns, pose) in arrival order."""
    best = None
    for stamp, pose in history:
        gap = abs(stamp - stamp_ns)
        if gap <= int(tolerance_s * 1e9) and (best is None or gap < best[0]):
            best = (gap, pose)
    return None if best is None else best[1]


def wait_for(get, now_ns, timeout_s, sleep):
    """get() until it returns a value or `timeout_s` of simulation time (now_ns()) passed; (value or None, waited s)."""
    start = now_ns()
    while True:
        value = get()
        if value is not None:
            return value, (now_ns() - start) * 1e-9
        if now_ns() - start > int(timeout_s * 1e9):
            return None, (now_ns() - start) * 1e-9
        sleep()


class LocalSink:
    """In-process hand-over of station poses from the detector to the alignment (same process, no DDS)."""

    def __init__(self):
        self.targets = []

    def connect(self, target):
        self.targets.append(target)

    def __call__(self, stamp_ns, frame, pose):
        for target in self.targets:
            target(stamp_ns, frame, pose)
