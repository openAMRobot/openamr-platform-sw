# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
# SPDX-License-Identifier: MIT
"""Component comparison of the drive wheel contact shapes in Gazebo: straight run and spin.

For each wheel_contact shape the base alone (robo_urdf.urdf.xacro, no upper body) runs headless in
an empty world with a ground plane, 1 ms step and the default physics engine, as walled_world.sdf.
Visuals and sensors are removed (they do not take part in contact); DiffDrive and its limits are
unchanged. An OdometryPublisher on the model gives the ground-truth pose; DiffDrive /odom gives the
wheel odometry. Two manoeuvres, each from rest, commanded through DiffDrive on /cmd_vel:

  straight  v 0.20 m/s for 10 s, then 0 for 1 s
  spin      w 0.50 rad/s for one commanded revolution (4 pi s), then 0 for 1 s

Reported per shape: distance and heading from odometry and from ground truth, yaw turned by
odometry and by ground truth, and the drift of the centre during the spin. The numbers are written
to WHEEL_CONTACT_EVIDENCE_DIR (JSON and Markdown). The test asserts that each run happened (the
base moved as commanded); it sets no acceptance limit: which shape to use is the decision of the
robot-description owner and the software lead. Simulation only; no tyre or floor was measured.

Needs `gz sim` (Gazebo Harmonic). Without it the test fails as BLOCKED, never a skip or a pass.
"""

import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest

import xacro

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'robo_urdf.urdf.xacro'
SHAPES = ('cylinder', 'sphere', 'narrow_cylinder')
WORLD = 'wheel_contact'
STEP = 0.001
KEEP_PLUGINS = ('gz-sim-diff-drive-system', 'gz-sim-joint-state-publisher-system')

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world}">
    <physics name="1ms" type="ignored">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
      </link>
    </model>
    {model}
  </world>
</sdf>
"""

TRUTH = """<gazebo>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>truth</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>/truth</odom_topic>
      <odom_publish_frequency>100</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>"""


def _robot_urdf(shape):
    root = ET.fromstring(xacro.process_file(str(XACRO), mappings={'wheel_contact': shape}).toxml())
    for link in root.findall('link'):
        for v in link.findall('visual'):
            link.remove(v)
    for g in list(root.findall('gazebo')):
        for s in g.findall('sensor'):
            g.remove(s)
        for p in g.findall('plugin'):
            if p.get('filename') not in KEEP_PLUGINS:
                g.remove(p)
        if g.get('reference') is None and len(g) == 0:
            root.remove(g)
    root.append(ET.fromstring(TRUTH))
    return ET.tostring(root, encoding='unicode')


def _parse_text(text):
    """Protobuf text format to nested dicts (repeated keys become lists)."""
    stack = [{}]
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.endswith('{'):
            node = {}
            key = line[:-1].strip()
            parent = stack[-1]
            if key in parent:
                parent[key] = parent[key] if isinstance(parent[key], list) else [parent[key]]
                parent[key].append(node)
            else:
                parent[key] = node
            stack.append(node)
        elif line == '}':
            stack.pop()
        elif ':' in line:
            k, v = line.split(':', 1)
            v = v.strip().strip('"')
            try:
                v = float(v)
            except ValueError:
                pass
            stack[-1][k.strip()] = v
    return stack[0]


def _messages(path):
    """Split an `gz topic -e` log into messages (each starts with a top-level header block)."""
    text = Path(path).read_text()
    parts = re.split(r'(?m)^(?=header \{)', text)
    out = []
    for p in parts:
        if not p.strip().startswith('header'):
            continue
        m = _parse_text(p)
        if 'pose' not in m or 'header' not in m:
            continue
        st = m['header'].get('stamp', {})
        t = st.get('sec', 0.0) + st.get('nsec', 0.0) * 1e-9
        pos = m['pose'].get('position', {})
        q = m['pose'].get('orientation', {})
        qx, qy, qz, qw = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        out.append((t, pos.get('x', 0.0), pos.get('y', 0.0), yaw))
    out.sort()
    return out


def _at(samples, t):
    """The last sample at or before t."""
    best = None
    for s in samples:
        if s[0] <= t + 1e-9:
            best = s
        else:
            break
    if best is None:
        raise AssertionError(f'no sample at or before t {t:.3f}')
    return best


def _unwrapped_yaw(samples, t0, t1):
    total, prev = 0.0, None
    for s in samples:
        if s[0] < t0 - 1e-9 or s[0] > t1 + 1e-9:
            continue
        if prev is not None:
            total += math.atan2(math.sin(s[3] - prev), math.cos(s[3] - prev))
        prev = s[3]
    return total


class Sim:
    def __init__(self, world_file, workdir):
        self.workdir = workdir
        env = dict(os.environ)
        self.env = env
        self.server = subprocess.Popen(
            ['gz', 'sim', '-s', '-v', '2', '-z', '1000000', str(world_file)],
            stdout=open(workdir / 'server.log', 'w'),
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        self.echo = []
        for topic in ('/odom', '/truth'):
            f = open(workdir / f'{topic.strip("/")}.log', 'w')
            self.echo.append(
                subprocess.Popen(
                    ['stdbuf', '-oL', 'gz', 'topic', '-e', '-t', topic],
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    env=env,
                    start_new_session=True,
                )
            )
        self.pub = None
        deadline = time.time() + 60
        while time.time() < deadline:
            if self.server.poll() is not None:
                raise AssertionError(
                    'gz sim exited: ' + (workdir / 'server.log').read_text()[-2000:]
                )
            r = subprocess.run(
                ['gz', 'service', '-l'], capture_output=True, text=True, env=env, timeout=30
            )
            if f'/world/{WORLD}/control' in r.stdout:
                break
            time.sleep(0.5)
        else:
            raise AssertionError('world control service did not appear')
        time.sleep(2.0)  # let the echo subscribers connect before the first step

    def stats(self):
        r = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{WORLD}/stats'],
            capture_output=True,
            text=True,
            env=self.env,
            timeout=30,
        )
        m = _parse_text(r.stdout)
        st = m.get('sim_time', {})
        return (
            st.get('sec', 0.0) + st.get('nsec', 0.0) * 1e-9,
            m.get('paused') == 'true' or m.get('paused') is True,
        )

    def command(self, v, w):
        if self.pub is not None:
            os.killpg(self.pub.pid, signal.SIGTERM)
            self.pub.wait()
        msg = f'linear: {{x: {v}}}, angular: {{z: {w}}}'
        loop = f'while true; do gz topic -t /cmd_vel -m gz.msgs.Twist -p "{msg}"; sleep 0.1; done'
        self.pub = subprocess.Popen(
            ['bash', '-c', loop],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=self.env,
            start_new_session=True,
        )
        time.sleep(1.5)  # paused: the command is delivered before any step

    def step(self, seconds):
        t0, _ = self.stats()
        n = int(round(seconds / STEP))
        r = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                f'/world/{WORLD}/control',
                '--reqtype',
                'gz.msgs.WorldControl',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '5000',
                '--req',
                f'pause: true, multi_step: {n}',
            ],
            capture_output=True,
            text=True,
            env=self.env,
            timeout=60,
        )
        assert 'data: true' in r.stdout, f'world control refused: {r.stdout} {r.stderr}'
        deadline = time.time() + 600
        while time.time() < deadline:
            t, paused = self.stats()
            if t >= t0 + n * STEP - 1e-6 and paused:
                return t
            time.sleep(0.2)
        raise AssertionError(f'simulation did not reach t {t0 + n * STEP:.3f}')

    def close(self):
        procs = ([self.pub] if self.pub else []) + self.echo + [self.server]
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)


def _run(shape, workdir):
    urdf = workdir / 'robot.urdf'
    urdf.write_text(_robot_urdf(shape))
    sdf = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True, timeout=120
    )
    assert sdf.returncode == 0 and sdf.stdout.strip(), f'gz sdf -p failed: {sdf.stderr[-1000:]}'
    model = ET.fromstring(sdf.stdout).find('model')
    model.set('name', 'base')
    world = workdir / 'world.sdf'
    world.write_text(
        WORLD_SDF.format(world=WORLD, step=STEP, model=ET.tostring(model, encoding='unicode'))
    )
    sim = Sim(world, workdir)
    try:
        sim.command(0.0, 0.0)
        t_settle = sim.step(1.0)
        sim.command(0.20, 0.0)
        sim.step(10.0)
        sim.command(0.0, 0.0)
        t_straight = sim.step(1.0)
        sim.command(0.0, 0.50)
        sim.step(4 * math.pi)
        sim.command(0.0, 0.0)
        t_spin = sim.step(1.0)
    finally:
        sim.close()
    odom, truth = _messages(workdir / 'odom.log'), _messages(workdir / 'truth.log')
    assert odom and truth, f'no odometry ({len(odom)}) or ground truth ({len(truth)}) recorded'

    def leg(t0, t1):
        o0, o1, g0, g1 = _at(odom, t0), _at(odom, t1), _at(truth, t0), _at(truth, t1)
        od = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
        gd = math.hypot(g1[1] - g0[1], g1[2] - g0[2])
        return {
            'odom_distance_m': od,
            'truth_distance_m': gd,
            'odom_yaw_rad': _unwrapped_yaw(odom, t0, t1),
            'truth_yaw_rad': _unwrapped_yaw(truth, t0, t1),
            'truth_heading_of_travel_rad': (
                math.atan2(g1[2] - g0[2], g1[1] - g0[1]) - g0[3] if gd > 1e-6 else 0.0
            ),
        }

    straight, spin = leg(t_settle, t_straight), leg(t_straight, t_spin)
    return {
        'shape': shape,
        'times_s': [t_settle, t_straight, t_spin],
        'samples': [len(odom), len(truth)],
        'straight': straight,
        'spin': spin,
    }


@pytest.fixture(scope='module')
def results():
    if shutil.which('gz') is None:
        pytest.fail('BLOCKED: gz (Gazebo Harmonic) is not installed; this comparison needs gz sim')
    version = subprocess.run(
        ['gz', 'sim', '--versions'], capture_output=True, text=True
    ).stdout.strip()
    out = {'gz_sim': version, 'step_s': STEP, 'runs': {}}
    base = Path(
        os.environ.get('WHEEL_CONTACT_EVIDENCE_DIR') or tempfile.mkdtemp(prefix='wheel_contact_')
    )
    for shape in SHAPES:
        d = base / shape
        d.mkdir(parents=True, exist_ok=True)
        out['runs'][shape] = _run(shape, d)
    (base / 'wheel_contact.json').write_text(json.dumps(out, indent=1) + '\n')
    rows = [
        '| Shape | Straight: odom m | truth m | odom/truth | truth yaw change rad '
        '| Spin: odom yaw rad | truth yaw rad | truth/odom | centre drift m |',
        '|---|---|---|---|---|---|---|---|---|',
    ]
    for shape, r in out['runs'].items():
        s, p = r['straight'], r['spin']
        rows.append(
            f"| {shape} | {s['odom_distance_m']:.4f} | {s['truth_distance_m']:.4f} | "
            f"{s['odom_distance_m'] / s['truth_distance_m']:.4f} | {s['truth_yaw_rad']:+.4f} | "
            f"{p['odom_yaw_rad']:.4f} | {p['truth_yaw_rad']:.4f} | "
            f"{p['truth_yaw_rad'] / p['odom_yaw_rad']:.4f} | "
            f"{p['truth_distance_m']:.4f} |"
        )
    table = '\n'.join(rows)
    (base / 'wheel_contact.md').write_text(f'gz sim {version}, step {STEP} s\n\n{table}\n')
    print(table)
    return out


@pytest.mark.parametrize('shape', SHAPES)
def test_straight_run_happened(results, shape):
    s = results['runs'][shape]['straight']
    # Commanded 0.20 m/s for 10 s with DiffDrive's 0.5 m/s2 ramp: about 2 m by odometry.
    assert 1.8 < s['odom_distance_m'] < 2.1, s
    assert s['truth_distance_m'] > 1.0, s


@pytest.mark.parametrize('shape', SHAPES)
def test_spin_happened(results, shape):
    p = results['runs'][shape]['spin']
    # Commanded one revolution; DiffDrive's 1 rad/s2 ramp makes it a little less by odometry.
    assert 5.5 < p['odom_yaw_rad'] < 6.5, p
    assert abs(p['truth_yaw_rad']) > 1.0, p
