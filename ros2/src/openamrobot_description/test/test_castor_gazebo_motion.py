# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
# SPDX-License-Identifier: MIT
"""In Gazebo the castors swivel to trail the motion and their wheels roll.

Castor-swivel proposal (15 Nov 2026). The expanded URDF is converted with `gz sdf -p`, placed
on a flat plane (mu 1) in a server-only world without sensors or meshes, and driven through
the DiffDrive plugin of gazebo_control.xacro. Checks:
  - at rest the base is level and on the floor (same as test_ground_contact.py, in physics);
  - driving forward the castors trail behind their swivel axes (angle near 0);
  - reversing with a small yaw the castors flip and trail the other way (angle near pi);
  - the castor wheels roll at v / r, so the base is not dragging fixed spheres;
  - the base covers the commanded distance.
This is a simulation behaviour check of the model, not a measurement of the real castor.

Needs `gz sim` (Gazebo Harmonic). Without gz it skips, unless OPENAMR_REQUIRE_GZ=1 is set:
then a missing gz fails. Runs about 30 s of wall time (real-time factor 1).
"""
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import pytest

import yaml

PKG = Path(__file__).resolve().parents[1]
URDF = PKG / 'urdf' / 'robo_urdf.urdf.xacro'
with open(PKG / 'config' / 'castor_model.yaml', encoding='utf-8') as _f:
    CASTOR = yaml.safe_load(_f)['castor']
PREFIXES = ('fl', 'fr', 'bl', 'br')
SPEED = 0.3  # m/s, below the 0.5 m/s ceiling


def _world_sdf(tmp):
    import xacro
    urdf = tmp / 'robot.urdf'
    urdf.write_text(xacro.process_file(str(URDF)).toxml())
    res = subprocess.run(['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True,
                         timeout=120)
    if res.returncode != 0 or not res.stdout.strip():
        pytest.fail('gz sdf -p failed: %s' % res.stderr[-500:])
    model = ET.fromstring(res.stdout).find('model')
    model.set('name', 'robot')
    for link in model.findall('link'):
        for tag in ('visual', 'sensor'):
            for e in link.findall(tag):
                link.remove(e)
    for plugin in model.findall('plugin'):
        if 'imu' in plugin.get('filename', ''):
            model.remove(plugin)
    js = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                       name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(js, 'topic').text = '/castor_test/joints'
    for p in PREFIXES:
        for kind in ('caster', 'wheel'):
            ET.SubElement(js, 'joint_name').text = '%s_%s_joint' % (p, kind)
    pp = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system',
                       name='gz::sim::systems::PosePublisher')
    for key, val in (('publish_link_pose', 'false'), ('publish_model_pose', 'true'),
                     ('use_pose_vector_msg', 'false'), ('update_frequency', '50')):
        ET.SubElement(pp, key).text = val
    ET.SubElement(model, 'pose').text = '0 0 0.002 0 0 0'
    world = tmp / 'castor_world.sdf'
    world.write_text(
        '<?xml version="1.0"?><sdf version="1.9"><world name="castor_test">'
        '<physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>'
        '<real_time_factor>1</real_time_factor></physics>'
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
        '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'
        '<plugin filename="gz-sim-scene-broadcaster-system"'
        ' name="gz::sim::systems::SceneBroadcaster"/>'
        '<model name="ground_plane"><static>true</static><link name="l"><collision name="c">'
        '<geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>'
        '<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>'
        '</collision></link></model>' + ET.tostring(model).decode() + '</world></sdf>')
    return world


class _Sim:

    def __init__(self, world):
        self.env = dict(os.environ, GZ_PARTITION='castor_' + uuid.uuid4().hex[:8])
        self.proc = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world)],
                                     env=self.env, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)

    def echo(self, topic):
        res = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', topic], env=self.env,
                             capture_output=True, text=True, timeout=20)
        return res.stdout

    def cmd(self, vx, wz):
        subprocess.run(['gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist', '-p',
                        'linear: {x: %g}, angular: {z: %g}' % (vx, wz)],
                       env=self.env, capture_output=True, timeout=20, check=True)

    def joints(self):
        """{joint name: (position, velocity)} from one JointStatePublisher message."""
        text = self.echo('/castor_test/joints')
        out = {}
        for block in text.split('\njoint {')[1:]:
            name = re.search(r'name: "(\w+)"', block)
            axis = block.split('axis1 {', 1)
            if not name or len(axis) < 2:
                continue
            pos = re.search(r'\n\s*position: ([-\w.+]+)', axis[1])
            vel = re.search(r'\n\s*velocity: ([-\w.+]+)', axis[1])
            out[name.group(1)] = (float(pos.group(1)) if pos else 0.0,
                                  float(vel.group(1)) if vel else 0.0)
        return out

    def pose(self):
        """(x, y, z, roll, pitch, yaw) of the model from one PosePublisher message."""
        text = self.echo('/model/robot/pose')

        def block(name):
            body = re.search(name + r' \{(.*?)\}', text, re.S).group(1)
            return {k: float(v) for k, v in re.findall(r'(\w): ([-\d.e+]+)', body)}
        p, q = block('position'), block('orientation')
        qw, qx, qy, qz = q.get('w', 1.0), q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0)
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0), roll, pitch, yaw

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _swivel(joints):
    return {p: _wrap(joints['%s_caster_joint' % p][0]) for p in PREFIXES}


@pytest.fixture(scope='module')
def sim(tmp_path_factory):
    if shutil.which('gz') is None:
        if os.environ.get('OPENAMR_REQUIRE_GZ') == '1':
            pytest.fail('gz (Gazebo) is not installed and OPENAMR_REQUIRE_GZ=1')
        pytest.skip('gz (Gazebo) is not installed; set OPENAMR_REQUIRE_GZ=1 to fail instead')
    s = _Sim(_world_sdf(tmp_path_factory.mktemp('castor')))
    try:
        deadline = time.time() + 60
        while time.time() < deadline:
            if s.proc.poll() is not None:
                pytest.fail('gz sim exited with %s' % s.proc.returncode)
            try:
                if len(s.joints()) == 8:
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
        else:
            pytest.fail('no castor joint states from gz sim within 60 s')
        time.sleep(1.0)
        yield s
    finally:
        s.stop()


def test_at_rest_the_base_is_level_on_the_floor(sim):
    x, y, z, roll, pitch, _ = sim.pose()
    assert abs(z) < 0.002, 'model origin (base_footprint) %.4f m off the floor' % z
    assert abs(math.degrees(roll)) < 0.1 and abs(math.degrees(pitch)) < 0.1, (roll, pitch)


def _drive(sim, vx, wz_kick, settle):
    """Command a small yaw kick, then straight travel; repeat so a lost message does no harm."""
    for _ in range(2):
        sim.cmd(vx, wz_kick)
    time.sleep(0.6)
    end = time.time() + settle
    while time.time() < end:
        sim.cmd(vx, 0.0)
        time.sleep(0.3)


def _wheels_roll_at(joints, speed):
    rate = speed / CASTOR['wheel_radius']['value']
    return all(abs(joints['%s_wheel_joint' % p][1] - rate) <= 0.1 * abs(rate) for p in PREFIXES)


def test_forward_castors_trail_and_wheels_roll(sim):
    x0 = sim.pose()[0]
    _drive(sim, SPEED, 0.05, 2.0)
    joints = sim.joints()
    x1 = sim.pose()[0]
    for p, a in _swivel(joints).items():
        assert abs(a) < 0.15, '%s castor at %.2f rad, should trail (0) going forward' % (p, a)
    assert _wheels_roll_at(joints, SPEED), \
        'castor wheels do not roll at v / r: %s' % {k: v[1] for k, v in joints.items()}
    assert x1 - x0 > 0.5, 'base covered only %.3f m forward' % (x1 - x0)


def test_reverse_castors_flip(sim):
    # from 0.3 m/s forward at the DiffDrive limit of 0.5 m/s2: 0.6 s to stop, 0.6 s back up to
    # speed, so 3.0 s of reverse command gives about 0.5 m net reverse travel
    x0 = sim.pose()[0]
    _drive(sim, -SPEED, 0.05, 3.0)
    joints = sim.joints()
    for p, a in _swivel(joints).items():
        assert abs(a) > math.pi - 0.15, '%s castor at %.2f rad, should trail (pi) reversing' % (
            p, a)
    assert _wheels_roll_at(joints, SPEED), \
        'castor wheels do not roll at v / r: %s' % {k: v[1] for k, v in joints.items()}
    assert x0 - sim.pose()[0] > 0.2, 'base did not reverse'


def test_forward_again_castors_flip_back(sim):
    _drive(sim, SPEED, -0.05, 2.0)
    for p, a in _swivel(sim.joints()).items():
        assert abs(a) < 0.15, '%s castor at %.2f rad after forward again' % (p, a)
    for _ in range(3):
        sim.cmd(0.0, 0.0)
    time.sleep(1.5)
    _, _, z, roll, pitch, _ = sim.pose()
    assert abs(z) < 0.002 and abs(math.degrees(pitch)) < 0.2 and abs(math.degrees(roll)) < 0.2
