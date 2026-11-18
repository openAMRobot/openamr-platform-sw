# SPDX-License-Identifier: MIT
# Independent review tests for platform-sw #167 at c4666a1 (stale data, retries, accuracy with min_samples).
# Run without ROS: rosstub replaces the ROS imports; the reviewed methods of StationAlignment run unchanged on a
# fake simulation clock. Tests named *_FINDING are expected to fail on c4666a1.
import math
import random
import types

import pytest

import rosstub

rosstub.install()

from openamrobot_station_alignment import geometry, sampling, sources  # noqa: E402
from openamrobot_station_alignment import station_alignment_node as san  # noqa: E402

S = 1_000_000_000
BASE = "base_footprint"


class FakeClock:
    def __init__(self, start_s):
        self.ns = int(start_s * S)

    def now(self):
        return types.SimpleNamespace(nanoseconds=self.ns)


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Harness(san.StationAlignment):
    """The real StationAlignment.__init__ and methods on a fake clock; Nav2 replaced by an instant success."""

    def __init__(self, start_s=100.0, rtf=0.1, overrides=None):
        self._params = {}
        self._over = dict(overrides or {})
        self.clock = FakeClock(start_s)
        self.rtf = rtf
        self.events = []          # (deliver_at_ns, stamp_ns, pose)
        self.camera = None        # callable(now_ns) -> list of (deliver_at_ns, stamp_ns, pose)
        self.servers = []
        san.ActionServer = lambda node, typ, name, **kw: self.servers.append(kw) or object()
        san.time = types.SimpleNamespace(sleep=self._sleep)
        super().__init__()
        self.tf_buffer = types.SimpleNamespace(lookup_transform=self._no_tf, all_frames_as_yaml=lambda: "{}")
        self.nav_calls = []

    # Node API used by the node
    def declare_parameter(self, name, value):
        self._params[name] = self._over.get(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, *a, **k):
        return Recorder()

    def create_subscription(self, *a, **k):
        return None

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)

    def get_publishers_info_by_topic(self, topic):
        return []

    def _no_tf(self, *a):
        raise rosstub.TransformException("no transform")

    # fake time: wall sleep dt advances simulation by rtf * dt and delivers due poses
    def _sleep(self, dt):
        self.clock.ns += int(self.rtf * dt * S)
        if self.camera:
            self.events.extend(self.camera(self.clock.ns))
        due = [e for e in self.events if e[0] <= self.clock.ns]
        self.events = [e for e in self.events if e[0] > self.clock.ns]
        for _, stamp, pose in sorted(due):
            self.ingest(stamp, BASE, pose)

    def _navigate(self, goal_handle, pose, frame, timeout_s=None):
        self.nav_calls.append(pose)
        return True, "Nav2 status 4"


def camera(rate_hz, latency_s, pose, start_s=0.0, stop_s=1e9, keep=1.0, seed=0):
    """Images stamped on the sim clock at rate_hz from start_s, delivered latency_s later."""
    state = {"next": int(start_s * S)}
    rng = random.Random(seed)

    def tick(now_ns):
        out = []
        while state["next"] <= now_ns and state["next"] < stop_s * S:
            if rng.random() < keep:
                p = pose() if callable(pose) else pose
                out.append((state["next"] + int(latency_s * S), state["next"], p))
            state["next"] += int(S / rate_hz)
        return out
    return tick


class GoalHandle:
    def __init__(self):
        q = geometry.quaternion_of_yaw(0.0)
        self.request = types.SimpleNamespace(pose=types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="map"),
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                                       orientation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))))
        self.is_cancel_requested = False
        self.state = None

    def succeed(self):
        self.state = "succeeded"

    def abort(self):
        self.state = "aborted"

    def canceled(self):
        self.state = "canceled"

    def publish_feedback(self, fb):
        pass


def run(h):
    import json
    gh = GoalHandle()
    result = h.execute(gh)
    report = json.loads(h.report.msgs[-1].data)
    return gh, result, report


# ---------------------------------------------------------------- 1. freshness
def _prefill(h, start_s, end_s, pose=(0.0, 0.0, 0.0)):
    """Fill the node's buffer with poses stamped and received in [start_s, end_s) on the current clock."""
    saved = h.clock.ns
    t = start_s
    while t < end_s:
        h.clock.ns = int(t * S)
        h.ingest(int(t * S), BASE, pose)
        t += 0.2
    h.clock.ns = saved


def test_1a_reopened_windows_do_not_use_poses_from_before_the_stop():
    h = Harness(start_s=100.0)
    _prefill(h, 60.0, 100.0)              # 200 perfect poses, all from before the stop
    gh, result, report = run(h)
    step = report["iterations"][1]
    assert result.error_code == san.NOT_ALIGNED and report["outcome"].startswith("not aligned")
    assert step["window_attempts"] == 3 and step["fresh"] == 0 and step["received"] == 0


def test_1b_a_later_iteration_does_not_reuse_the_previous_window():
    h = Harness(start_s=100.0)
    # 0.05 m residual (needs a correction) only until 110 s, then the marker stream stops
    h.camera = camera(5.0, 0.3, (0.05, 0.0, 0.0), start_s=100.0, stop_s=103.0)
    h._correct = lambda gh, residual, step: True     # correction "done", no motion
    gh, result, report = run(h)
    assert report["iterations"][1]["window_status"] == sampling.DONE
    assert report["iterations"][2]["fresh"] == 0
    assert report["outcome"].startswith("aborted") and gh.state == "aborted"


def test_1c_FINDING_old_generation_poses_after_a_clock_reset_are_not_fresh():
    h = Harness(start_s=10.0)
    _prefill(h, 500.0, 540.0)             # previous /clock generation: stamps and receive times near 500 s
    gh, result, report = run(h)           # the clock restarted at 10 s; no marker pose of this generation
    assert report["outcome"] != "aligned", report["iterations"][1]


def test_1d_reopened_window_freshness_is_counted_from_its_own_opening():
    w1 = sampling.SampleWindow(10 * S, 8, 6.0, 15.0, 4)
    assert w1.poll(16 * S + 1) == sampling.NOT_SEEN
    w2 = sampling.SampleWindow(16 * S + 1, 8, 6.0, 15.0, 4)
    for t in range(11, 16):               # stamped inside window 1, arriving during window 2
        w2.add(t * S, 17 * S, (0.0, 0.0, 0.0))
    assert len(w2.fresh) == 0


# ---------------------------------------------------------------- 2. odom input
def test_2a_FINDING_an_odom_message_stamped_in_the_future_is_not_fresh():
    latest = (500 * S, "odom", BASE, (1.0, 0.0, 0.0))     # from before a /clock reset to 10 s
    assert sources.fresh_odom(latest, 10 * S, 0.5, "odom", BASE) is None


def _tf(stamp_s, x):
    q = geometry.quaternion_of_yaw(0.0)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=int(stamp_s), nanosec=0)),
        transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=x, y=0.0, z=0.0),
                                        rotation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])))


def test_2b_stale_odom_message_falls_back_and_fails_without_tf():
    h = Harness(start_s=100.0)
    h.latest_odom = (99 * S, "odom", BASE, (1.0, 0.0, 0.0))   # 1 s old > odom_max_age_s 0.5
    with pytest.raises(rosstub.TransformException):
        h._odom_from_base()
    assert h.clock.ns - 100 * S >= 5 * S                       # waited tf_wait_s of simulation time


def test_2c_FINDING_a_stale_tf_fallback_is_not_used():
    h = Harness(start_s=100.0)
    h.latest_odom = None                                       # /odom writer lost (Baseline 1 failure mode)
    h.tf_buffer.lookup_transform = lambda *a: _tf(70.0, 1.0)   # last odom -> base 30 s old
    with pytest.raises(rosstub.TransformException):
        h._odom_from_base()


def test_2d_FINDING_odom_pose_is_taken_at_the_image_stamps_not_later():
    h = Harness(start_s=100.0)
    h.latest_odom = (100 * S, "odom", BASE, (1.000, 0.0, 0.0))  # pose while the images were taken
    h._map_from_base = lambda: (0.0, 0.0, 0.0)
    step = {}

    def stop(*a):
        return False, "stop"
    h._drive = stop   # adapted on the branch: forward drives replaced behavior_server translations (aa8fd11 review a)
    h._rotate = stop
    h.window_stamp_ns = 100 * S   # adapted on the branch: what _collect records, the mean image stamp of the window
    h.clock.ns = 101 * S
    h.latest_odom = (101 * S, "odom", BASE, (1.010, 0.0, 0.0))  # 10 mm odom change after the window
    h._correct(GoalHandle(), (0.05, 0.0, 0.0), step)
    assert abs(step["target_odom"][0] - 1.050) < 1e-6, step["target_odom"]


# ---------------------------------------------------------------- 3. accuracy with 4 samples
SIGMA = 3e-5          # per axis, conservative: the 8-frame spread (max distance from the mean) was below 3e-5 m
BIAS_MAX = 0.0052     # marker - truth, Baseline table max (constant sign: a bias)


def _mc(n, trials=20000, seed=1):
    rng = random.Random(seed)
    errs = []
    for _ in range(trials):
        xs = [rng.gauss(0, SIGMA) for _ in range(n)]
        ys = [rng.gauss(0, SIGMA) for _ in range(n)]
        errs.append(math.hypot(sum(xs) / n, sum(ys) / n))
    errs.sort()
    return errs[int(0.997 * trials)]


def test_3a_noise_of_a_4_sample_mean_is_negligible_against_the_budget():
    e4, e8 = _mc(4), _mc(8)
    print(f"99.7% radial error of the mean: 4 samples {e4 * 1e3:.4f} mm, 8 samples {e8 * 1e3:.4f} mm")
    assert e4 < 1e-4 and e8 < e4 < 2 * e8


def test_3b_xy_spread_check_rejects_a_single_inconsistent_sample_among_4():
    w = sampling.SampleWindow(0, 8, 6.0, 15.0, 4)
    for p in [(0.0, 0.0, 0.0)] * 3 + [(0.014, 0.0, 0.0)]:
        w.add(1, 2, p)
    _, spread = w.result()
    assert spread > 0.01                      # 3/4 * 14 mm = 10.5 mm: rejected
    w = sampling.SampleWindow(0, 8, 6.0, 15.0, 4)
    for p in [(0.0, 0.0, 0.0)] * 3 + [(0.013, 0.0, 0.0)]:
        w.add(1, 2, p)
    mean, spread = w.result()
    assert spread <= 0.01 and abs(mean[0] - 0.00325) < 1e-9   # undetected: mean moved 3.25 mm


def test_3c_FINDING_inconsistent_yaw_samples_are_rejected():
    w = sampling.SampleWindow(0, 8, 6.0, 15.0, 4)
    for p in [(0.0, 0.0, 0.0)] * 3 + [(0.0, 0.0, 0.20)]:      # one frame 0.2 rad off (PnP flip-like)
        w.add(1, 2, p)
    mean, spread = w.result()
    # adapted on the branch: the yaw spread is its own value (window.spread_yaw), checked against
    # max_sample_spread_yaw (0.01 rad) next to the xy spread
    rejected = spread > 0.01 or w.spread_yaw > 0.01
    assert rejected, f"accepted with mean yaw {mean[2]:.3f} rad, xy spread {spread}"


def test_3d_FINDING_worst_accepted_pose_plus_marker_bias_is_inside_0_04_m():
    # adapted on the branch: the acceptance limits are read from the configuration instead of 0.03 / 0.02
    import os
    import yaml
    import openamrobot_station_alignment as pkg
    cfg = os.path.join(os.path.dirname(os.path.dirname(pkg.__file__)), "config", "station_alignment.yaml")
    params = yaml.safe_load(open(cfg))["station_alignment"]["ros__parameters"]
    worst = math.hypot(params["accept_along"], params["accept_lateral"]) + BIAS_MAX + 3 * SIGMA / 2
    print(f"worst true error {worst * 1e3:.2f} mm against 40 mm")
    assert worst <= 0.04


def test_3e_yaw_budget_margin():
    assert 0.06 + 0.0023 <= 0.08


# ---------------------------------------------------------------- 4. retries
def test_4a_no_samples_reports_not_aligned_9900_within_a_bounded_time():
    h = Harness(start_s=100.0)
    gh, result, report = run(h)
    elapsed = (h.clock.ns - 100 * S) / S
    assert result.error_code == san.NOT_ALIGNED and report["outcome"].startswith("not aligned")
    # adapted on the branch: ABORTED with 9900 is the default (software lead's review of aa8fd11, item c)
    assert gh.state == "aborted" and report["iterations"][1]["window_attempts"] == 3
    assert elapsed <= 1.0 + 3 * 6.0 + 0.1, elapsed


def test_4b_fewer_than_min_samples_is_never_aligned():
    h = Harness(start_s=100.0)
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=101.5, stop_s=102.1)   # 3 images after opening
    gh, result, report = run(h)
    step = report["iterations"][1]
    assert step["fresh"] == 3 and step["window_status"] == sampling.STALE
    assert result.error_code == san.NOT_ALIGNED and report["outcome"] != "aligned"


def test_4c_done_implies_at_least_min_samples_fresh():
    rng = random.Random(7)
    for _ in range(3000):
        n, m = 8, rng.randint(1, 8)
        w = sampling.SampleWindow(10 * S, n, 6.0, 15.0, m)
        t = 10 * S
        while True:
            t += rng.randint(1, 4) * S // 10
            if rng.random() < 0.3:
                stamp = t - rng.randint(0, 20) * S // 2
                w.add(stamp, t, (0.0, 0.0, 0.0))
            st = w.poll(t)
            if st != sampling.WAITING:
                break
        if st == sampling.DONE:
            assert len(w.fresh) >= m


def test_4d_worst_case_time_with_two_empty_windows_then_stale_poses():
    h = Harness(start_s=100.0)
    # nothing for the first two windows, then only stale images (30 s latency, stamped before the window)
    h.camera = camera(5.0, 30.0, (0.0, 0.0, 0.0), start_s=90.0)
    gh, result, report = run(h)
    elapsed = (h.clock.ns - 100 * S) / S
    assert report["outcome"] != "aligned"
    assert elapsed <= 1.0 + 6.0 + 6.0 + 21.0 + 0.2, elapsed


def test_4e_four_fresh_samples_after_the_timeout_align():
    h = Harness(start_s=100.0)
    h.camera = camera(5.0, 0.3, (0.0, 0.0, 0.0), start_s=101.5, stop_s=102.3)   # 4 images
    gh, result, report = run(h)
    assert report["outcome"] == "aligned" and report["iterations"][1]["fresh"] == 4


def test_4f_FINDING_min_samples_zero_is_rejected_not_done_with_no_sample():
    w = sampling.SampleWindow(0, 8, 6.0, 15.0, 0)
    assert w.poll(7 * S) == sampling.NOT_SEEN


# ---------------------------------------------------------------- 5. combined executable
def test_5a_FINDING_a_second_goal_is_not_accepted_while_one_runs():
    h = Harness()
    goal_cb = h.servers[0]["goal_callback"]
    first = goal_cb(object())
    second = goal_cb(object())          # while the first executes
    assert first == rosstub.GoalResponse.ACCEPT and second != rosstub.GoalResponse.ACCEPT


def test_5b_alignment_callbacks_are_reentrant_and_tf_and_clock_are_not_behind_execute():
    import ast
    import inspect
    src = ast.parse(inspect.getsource(san.StationAlignment.__init__).strip().replace("\n    ", "\n"))
    calls = [c for c in ast.walk(src) if isinstance(c, ast.Call)
             and ast.unparse(c.func) in ("ActionClient", "ActionServer", "self.create_subscription")]
    assert len(calls) == 5   # adapted on the branch: no behavior_server clients, a /safety/stop_state subscription
    for c in calls:
        assert any(k.arg == "callback_group" and ast.unparse(k.value) == "self.group" for k in c.keywords), \
            ast.unparse(c)


def test_5c_stack_creates_no_publisher_and_no_client():
    import ast
    import os
    import openamrobot_station_alignment as pkg
    t = ast.parse(open(os.path.join(os.path.dirname(pkg.__file__), "stack.py")).read())
    names = {ast.unparse(c.func) for c in ast.walk(t) if isinstance(c, ast.Call)}
    assert not [n for n in names if "publisher" in n or "Client" in n or "Twist" in n]
