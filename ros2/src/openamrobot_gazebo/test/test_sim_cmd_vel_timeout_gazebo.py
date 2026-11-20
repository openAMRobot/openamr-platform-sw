# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
The simulated base stops when its /cmd_vel publisher dies (#79), measured in Gazebo.

Starts gz_simulator.launch.py headless, drives the base at 0.2 m/s from a `ros2 topic pub`
process, SIGKILLs that process (no final zero: a lost publisher) and follows the robot's world pose
from Gazebo itself (gz-transport, /world/walled_world/dynamic_pose/info), not /odom, because
odometry from DiffDrive reports the commanded motion even when the body does not follow.

Before the fix the base drove on at 0.2 m/s for as long as the simulation ran (1.00 m in the 5 s
after the kill on main 69847ab). With the command timeout it stops within the timeout plus
DiffDrive's deceleration.

Needs Gazebo Harmonic (`gz`). Without gz the test skips unless OPENAMR_REQUIRE_GZ=1 is set, as
for test_gazebo_friction.py; then a missing gz fails. Simulation only: no hardware, not safety
evidence.
"""

import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time

import pytest

MODEL = 'openamrobot'
POSES = '/world/walled_world/dynamic_pose/info'
SPEED = 0.2                 # m/s commanded before the publisher dies
DRIVE_S = 4.0               # simulation seconds of driving before the kill
WATCH_S = 5.0               # simulation seconds watched after the kill
TIMEOUT_S = 0.2             # config/sim_cmd_vel_timeout.yaml
DECEL = 0.5                 # m/s^2, DiffDrive min_linear_acceleration (gazebo_control.xacro)
# Travel allowed after the kill: the timeout at full speed, the deceleration ramp, and 0.1 s of
# bridge and tick latency at full speed: 0.04 + 0.04 + 0.02 = 0.10 m; 0.15 m leaves margin.
MAX_TRAVEL_AFTER_KILL_M = 0.15
MAX_STOP_DELAY_S = 1.0      # simulation time from the kill until the body is still
STILL_M_PER_S = 0.005


def _gz_or_skip():
    if shutil.which('gz') is None:
        if os.environ.get('OPENAMR_REQUIRE_GZ') == '1':
            pytest.fail('gz (Gazebo) is not installed and OPENAMR_REQUIRE_GZ=1')
        pytest.skip('gz (Gazebo) is not installed; set OPENAMR_REQUIRE_GZ=1 to fail instead (#79)')
    try:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as exc:  # the gz Python bindings come with the gz vendor packages
        if os.environ.get('OPENAMR_REQUIRE_GZ') == '1':
            pytest.fail(f'gz Python bindings missing and OPENAMR_REQUIRE_GZ=1: {exc}')
        pytest.skip(f'gz Python bindings missing ({exc}); '
                    'set OPENAMR_REQUIRE_GZ=1 to fail instead (#79)')
    return Pose_V, Node


class WorldPose:
    """The robot's world pose from Gazebo, stamped with simulation time."""

    def __init__(self, pose_v, gz_node):
        self.lock = threading.Lock()
        self.samples = []  # (sim_s, x, y)
        self.node = gz_node()
        assert self.node.subscribe(pose_v, POSES, self._on_poses)

    def _on_poses(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == MODEL:
                with self.lock:
                    self.samples.append((t, p.position.x, p.position.y))

    def now(self):
        with self.lock:
            return self.samples[-1][0] if self.samples else None

    def wait_until(self, sim_s, wall_limit_s):
        end = time.monotonic() + wall_limit_s
        while time.monotonic() < end:
            now = self.now()
            if now is not None and now >= sim_s:
                return now
            time.sleep(0.01)
        raise TimeoutError(f'simulation time did not reach {sim_s} s (now {self.now()})')

    def between(self, t0, t1):
        with self.lock:
            return [(t, x, y) for t, x, y in self.samples if t0 <= t <= t1]


def _travel(pts):
    return math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2]) if len(pts) > 1 else 0.0


def _still_from(pts, window_s=0.1):
    """First simulation time from which every `window_s` window moves slower than STILL_M_PER_S."""
    still, k = None, 0
    for j, (tj, xj, yj) in enumerate(pts):
        k = max(k, j)
        while k < len(pts) and pts[k][0] < tj + window_s:
            k += 1
        if k == len(pts):
            break
        speed = math.hypot(pts[k][1] - xj, pts[k][2] - yj) / (pts[k][0] - tj)
        if speed > STILL_M_PER_S:
            still = None
        elif still is None:
            still = tj
    return still


def _kill_group(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


@pytest.fixture(scope='module')
def sim():
    pose_v, gz_node = _gz_or_skip()
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 50)       # isolated from other graphs
    env['GZ_PARTITION'] = f'openamr_test79_{os.getpid()}'     # isolated gz-transport partition
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']          # for the in-process pose subscriber
    log = tempfile.NamedTemporaryFile('w+', prefix='test79_launch_', suffix='.log', delete=False)
    launch = subprocess.Popen(['ros2', 'launch', 'openamrobot_gazebo', 'gz_simulator.launch.py',
                               'gui:=false'], env=env, stdout=log, stderr=subprocess.STDOUT,
                              stdin=subprocess.DEVNULL, start_new_session=True)
    pose = WorldPose(pose_v, gz_node)
    try:
        end = time.monotonic() + 120
        while pose.now() is None or pose.now() < 2.0:   # robot spawned and settled
            if launch.poll() is not None or time.monotonic() > end:
                log.seek(0)
                tail = log.read()[-2000:]
                pytest.fail(f'simulation did not start (exit {launch.poll()}): {tail}')
            time.sleep(0.2)
        yield pose, env, log
    finally:
        _kill_group(launch, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            _kill_group(launch, signal.SIGKILL)
            launch.wait(timeout=10)


def test_base_stops_after_its_cmd_vel_publisher_is_killed(sim):
    pose, env, log = sim
    pub = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '20', '/cmd_vel',
                            'geometry_msgs/msg/Twist', f'{{linear: {{x: {SPEED}}}}}'],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        t_start = pose.now()
        pose.wait_until(t_start + DRIVE_S, wall_limit_s=120)
        t_kill = pose.now()
        _kill_group(pub, signal.SIGKILL)   # no final zero: the publisher is simply gone
        pose.wait_until(t_kill + WATCH_S, wall_limit_s=120)
    finally:
        _kill_group(pub, signal.SIGKILL)
        pub.wait(timeout=10)

    driven = _travel(pose.between(t_start, t_kill))
    after = pose.between(t_kill, t_kill + WATCH_S)
    travel_after = _travel(after)
    still = _still_from(after)
    last_2s = _travel(pose.between(t_kill + WATCH_S - 2.0, t_kill + WATCH_S))
    log.seek(0)
    timeouts_logged = log.read().count('holding the simulated base at zero')
    report = {'commanded_m_per_s': SPEED, 'driven_before_kill_m': round(driven, 4),
              'travel_after_kill_m': round(travel_after, 4),
              'still_after_kill_s': None if still is None else round(still - t_kill, 3),
              'travel_last_2s_m': round(last_2s, 4), 'timeouts_logged': timeouts_logged,
              'samples_after_kill': len(after), 'kill_sim_s': round(t_kill, 3)}
    out = os.environ.get('OPENAMR_TEST79_REPORT')
    if out:
        Path(out).write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(report))

    # Positive control: the base really moved in Gazebo under the command.
    assert driven > 0.3, f'base did not move under {SPEED} m/s: {report}'
    assert len(after) > 50, f'too few pose samples after the kill: {report}'
    assert travel_after <= MAX_TRAVEL_AFTER_KILL_M, f'base kept driving after the kill: {report}'
    assert still is not None and still - t_kill <= MAX_STOP_DELAY_S, f'base not still: {report}'
    assert last_2s < 0.002, f'base still moving at the end: {report}'
    assert timeouts_logged >= 1, f'the command timeout did not report firing: {report}'
