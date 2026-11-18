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


def fresh_odom(latest, now_ns, max_age_s, odom_frame, base_frame):
    """(x, y, yaw) of the latest /odom message if it is odom_frame -> base_frame and at most max_age_s old."""
    if latest is None:
        return None
    stamp_ns, frame, child, pose = latest
    if frame != odom_frame or child != base_frame:
        return None
    if now_ns - stamp_ns > int(max_age_s * 1e9):
        return None
    return pose


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
