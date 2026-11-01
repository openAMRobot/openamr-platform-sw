# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Near-field sensor frames of the expanded URDF against the register (PSW-08).

The expected us_* and tof_* origins are parsed from decisions.yaml NEAR-FIELD-FRAMES in the
pinned copy under tests/fixtures/register at the repository root (commit and sha256 in
PIN.yaml). No register value is written in this file: a register change reaches the test
only by refreshing the fixture.
"""

import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
URDF = PKG / 'urdf' / 'robo_urdf.urdf.xacro'
# Pinned at the repository root: the register excludes tests/fixtures/** from its own
# text check, so the copy is not reported as contradicting itself.
REGISTER_DIR = Path(__file__).resolve().parents[4] / 'tests' / 'fixtures' / 'register'
REGISTER = REGISTER_DIR / 'decisions.yaml'
PIN = REGISTER_DIR / 'PIN.yaml'
DECISION = 'NEAR-FIELD-FRAMES'

# Tolerances of the task (PSW-08): 0.1 mm on position, 0.06 degrees on orientation.
POS_TOL_M = 0.0001
ANGLE_TOL_DEG = 0.06

FRAME_RE = re.compile(r'\b((?:us|tof)_[a-z_]+) \(([^)]*)\)')
PITCH_RE = re.compile(r'pitched (\d+(?:\.\d+)?) degrees down')
TILT_RE = re.compile(r'\btilt (\d+(?:\.\d+)?)\b')
NAMES = ['us_front_left', 'us_front_right', 'us_rear_left', 'us_rear_right',
         'tof_front', 'tof_rear']


def load_entry():
    with open(REGISTER, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    entries = [d for d in data['decisions'] if d['id'] == DECISION]
    assert len(entries) == 1, 'register has %d %s entries' % (len(entries), DECISION)
    return entries[0]


def value_text(value):
    """
    One register value as text.

    An unquoted ': ' in the register makes YAML read the item as a one-key mapping
    (the tof_* item at 71c439e); the key and value are joined back into the original text.
    """
    if isinstance(value, dict):
        return ' '.join('%s: %s' % (k, v) for k, v in value.items())
    return str(value)


def parse_frames(entry):
    """{frame: {xyz, yaw, pitch}} in metres and radians from the register value strings."""
    frames = {}
    for value in entry['values']:
        text = ' '.join(value_text(value).split())
        pitch = PITCH_RE.search(text)
        tilt = TILT_RE.search(text)
        for name, inner in FRAME_RE.findall(text):
            parts = [p.strip() for p in inner.split(',')]
            xyz = [float(p) for p in parts[:3]]
            yaw = 0.0
            for extra in parts[3:]:
                assert extra == 'yaw pi', '%s: unknown term %r' % (name, extra)
                yaw = math.pi
            frames[name] = {'xyz': xyz, 'yaw': yaw,
                            'pitch_text': pitch.group(1) if pitch else None}
        if tilt:
            for f in frames.values():
                f.setdefault('tilt_text', tilt.group(1))
    for name, f in frames.items():
        deg = f['pitch_text'] if name.startswith('tof_') else f.get('tilt_text')
        assert deg is not None, '%s: no pitch or tilt in the register' % name
        f['pitch'] = math.radians(float(deg))
    return frames


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


@pytest.fixture(scope='module')
def register_frames():
    return parse_frames(load_entry())


@pytest.fixture(scope='module')
def joints():
    import xacro
    robot = ET.fromstring(xacro.process_file(str(URDF)).toxml())
    out = {}
    for joint in robot.findall('joint'):
        if joint.find('parent').get('link') != 'base_link':
            continue
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        out[joint.find('child').get('link')] = (np.array(xyz), rpy_matrix(*rpy))
    return out


def test_register_fixture_matches_its_pin():
    pin = yaml.safe_load(PIN.read_text(encoding='utf-8'))
    assert pin['repository'] == 'openAMRobot/.github' and pin['path'] == 'decisions.yaml'
    assert re.fullmatch(r'[0-9a-f]{40}', pin['commit']), pin['commit']
    assert hashlib.sha256(REGISTER.read_bytes()).hexdigest() == pin['sha256']


def test_register_lists_every_near_field_frame(register_frames):
    assert sorted(register_frames) == sorted(NAMES), sorted(register_frames)


@pytest.mark.parametrize('name', NAMES)
def test_joint_origin_equals_register(name, register_frames, joints):
    """Position within 0.1 mm and orientation within 0.06 degrees of the register."""
    exp = register_frames[name]
    assert name in joints, '%s is not a child of base_link in the URDF' % name
    xyz, rot = joints[name]
    err_mm = np.abs(xyz - np.array(exp['xyz'])) * 1000.0
    assert np.all(err_mm <= POS_TOL_M * 1000.0 + 1e-9), \
        '%s: URDF xyz %s, register %s (mm error %s)' % (
            name, xyz.tolist(), exp['xyz'], err_mm.round(3).tolist())
    rel = rpy_matrix(0.0, exp['pitch'], exp['yaw']).T @ rot
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1.0) / 2.0))))
    assert angle <= ANGLE_TOL_DEG, (
        '%s: orientation differs from the register by %.3f deg (register pitch %.1f deg '
        'down, yaw %.4f rad)' % (name, angle, math.degrees(exp['pitch']), exp['yaw']))


@pytest.fixture(scope='module')
def sdf_frames():
    """{frame: (xyz, rotation)} of the base_link-relative frames in the legacy urdf/robot.sdf."""
    root = ET.parse(PKG / 'urdf' / 'robot.sdf').getroot()
    out = {}
    for frame in root.iter('frame'):
        pose = frame.find('pose')
        if pose is None or pose.get('relative_to') != 'base_link':
            continue
        v = [float(t) for t in pose.text.split()]
        out[frame.get('name')] = (np.array(v[:3]), rpy_matrix(*v[3:6]))
    return out


@pytest.mark.parametrize('name', NAMES)
def test_sdf_frame_equals_register(name, register_frames, sdf_frames):
    """The legacy robot.sdf frames carry the same register values as the URDF."""
    exp = register_frames[name]
    assert name in sdf_frames, '%s is not a base_link frame in robot.sdf' % name
    xyz, rot = sdf_frames[name]
    err = np.abs(xyz - np.array(exp['xyz']))
    assert np.all(err <= POS_TOL_M + 1e-9), (name, xyz.tolist(), exp['xyz'])
    rel = rpy_matrix(0.0, exp['pitch'], exp['yaw']).T @ rot
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1.0) / 2.0))))
    assert angle <= ANGLE_TOL_DEG, (name, angle)
