# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
# SPDX-License-Identifier: MIT
"""The four castors are passive swivel castors with the geometry of their source.

Castor-swivel proposal (15 Nov 2026): each Blickle LPA-VSTH 35K castor (BOM MEC-002) is a
vertical swivel joint at the mount (<c>_caster_joint) and a horizontal roll joint
(<c>_wheel_joint) one trail behind the swivel axis. Every number lives in
config/castor_model.yaml with its source or its estimation method; these tests check the
expanded URDF against that file and check the file against its sources.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

import pytest

import xacro

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
MODEL_FILE = os.path.join(HERE, os.pardir, 'config', 'castor_model.yaml')
EXPORT_FILE = os.path.join(HERE, os.pardir, 'config', 'roadmap_mass_export.yaml')
with open(MODEL_FILE, encoding='utf-8') as f:
    CASTOR = yaml.safe_load(f)['castor']
PREFIXES = ('fl', 'fr', 'bl', 'br')
# Datasheet values, written here a second time on purpose: a change of castor_model.yaml that
# is not a change of the part fails this check. BOM Issue 7.15 MEC-002 and the castor options
# sheet: LPA-VSTH 35K, wheel D35 x 27 mm, swivel offset (trail) 15 mm.
DATASHEET = {'trail': 0.015, 'wheel_radius': 0.035 / 2, 'wheel_width': 0.027}
# Mount positions and heights are unchanged by this proposal (legacy SolidWorks positions,
# z kept per Alex decision 2026-10-12); the M-02 castor layout will replace them.
MOUNTS = {'fl': (0.27505, 0.215), 'fr': (0.27505, -0.215),
          'bl': (-0.27495, 0.215), 'br': (-0.27495, -0.215)}
MOUNT_Z = -0.043833
BASE_LINK_HEIGHT = 0.098
STATUSES = {'datasheet', 'cad', 'decision', 'estimated'}


def _robot():
    return ET.fromstring(xacro.process_file(XACRO).toxml())


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(origin):
    m = np.eye(4)
    if origin is not None:
        m[:3, :3] = _rpy(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
        m[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    return m


def _joint_rot(joint, angle):
    """Homogeneous rotation of `angle` about the joint axis (joint frame)."""
    axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
    return m


def _world(robot, link, q=None):
    """Transform of `link` in base_footprint with joint positions `q` (default all zero)."""
    q = q or {}
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    m = np.eye(4)
    while link in joints:
        j = joints[link]
        local = _tf(j.find('origin'))
        if j.get('type') != 'fixed':
            local = local @ _joint_rot(j, q.get(j.get('name'), 0.0))
        m = local @ m
        link = j.find('parent').get('link')
    return m


def _joint(robot, name):
    found = robot.find("joint[@name='%s']" % name)
    assert found is not None, 'no joint %s' % name
    return found


def _axis_in_base(robot, joint):
    """Joint axis expressed in base_footprint at zero joint positions."""
    parent = _world(robot, joint.find('parent').get('link'))
    rot = parent[:3, :3] @ _tf(joint.find('origin'))[:3, :3]
    axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
    return rot @ (axis / np.linalg.norm(axis))


@pytest.fixture(scope='module')
def robot():
    return _robot()


def test_castor_model_values_have_a_source_or_a_method():
    """No unexplained number: a source for measured or decided values, a method for estimates."""
    for key, entry in CASTOR.items():
        if not isinstance(entry, dict):
            continue
        assert {'value', 'unit', 'status'} <= set(entry), key
        assert entry['status'] in STATUSES, key
        if entry['status'] == 'estimated':
            assert len(str(entry.get('method', ''))) > 40, '%s: estimate without a method' % key
            assert entry.get('owner') and entry.get('measure'), '%s: estimate without owner' % key
        else:
            assert len(str(entry.get('source', ''))) > 10, '%s: no source' % key


def test_castor_model_geometry_matches_the_datasheet():
    for key, want in DATASHEET.items():
        assert CASTOR[key]['status'] == 'datasheet', key
        assert CASTOR[key]['value'] == pytest.approx(want, abs=1e-9), key


@pytest.mark.parametrize('prefix', PREFIXES)
def test_swivel_axis_is_vertical_and_at_the_mount(robot, prefix):
    joint = _joint(robot, '%s_caster_joint' % prefix)
    assert joint.get('type') == 'continuous', 'the swivel must turn freely through 360 deg'
    assert joint.find('parent').get('link') == 'base_link'
    axis = _axis_in_base(robot, joint)
    assert abs(abs(axis[2]) - 1.0) <= 1e-9, '%s swivel axis %s is not parallel to z' % (
        prefix, axis)
    origin = _world(robot, '%s_caster_link' % prefix)[:3, 3]
    x, y = MOUNTS[prefix]
    assert origin == pytest.approx([x, y, BASE_LINK_HEIGHT + MOUNT_Z], abs=1e-9), \
        'castor mount moved'


@pytest.mark.parametrize('prefix', PREFIXES)
def test_roll_axis_is_horizontal(robot, prefix):
    joint = _joint(robot, '%s_wheel_joint' % prefix)
    assert joint.get('type') == 'continuous', 'the castor wheel must roll'
    assert joint.find('parent').get('link') == '%s_caster_link' % prefix
    swivel = _joint(robot, '%s_caster_joint' % prefix)
    for angle in (0.0, 0.7, math.pi / 2, 2.5, math.pi):
        q = {swivel.get('name'): angle}
        parent = _world(robot, '%s_caster_link' % prefix, q)
        rot = parent[:3, :3] @ _tf(joint.find('origin'))[:3, :3]
        axis = rot @ np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        assert abs(axis[2]) <= 1e-9, '%s roll axis not horizontal at swivel %.2f rad' % (
            prefix, angle)
        # the roll axis is perpendicular to the trail arm, so the wheel rolls along the trail
        arm = _world(robot, '%s_wheel_link' % prefix, q)[:3, 3] - parent[:3, 3]
        assert abs(float(np.dot(axis, arm))) <= 1e-9


@pytest.mark.parametrize('prefix', PREFIXES)
def test_trail_matches_the_source(robot, prefix):
    """Horizontal distance swivel axis to wheel axle = trail, at every swivel angle."""
    swivel = _joint(robot, '%s_caster_joint' % prefix)
    for angle in (0.0, 1.0, math.pi / 2, math.pi, -2.0):
        q = {swivel.get('name'): angle}
        mount = _world(robot, '%s_caster_link' % prefix, q)[:3, 3]
        wheel = _world(robot, '%s_wheel_link' % prefix, q)[:3, 3]
        horizontal = math.hypot(*(wheel - mount)[:2])
        assert horizontal == pytest.approx(CASTOR['trail']['value'], abs=1e-9)
        assert horizontal == pytest.approx(DATASHEET['trail'], abs=1e-9)
    # at zero swivel the wheel trails behind the axis (stable for forward travel)
    zero = _world(robot, '%s_wheel_link' % prefix)[:3, 3] - \
        _world(robot, '%s_caster_link' % prefix)[:3, 3]
    assert zero[0] == pytest.approx(-DATASHEET['trail'], abs=1e-9)
    assert zero[1] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('prefix', PREFIXES)
def test_contact_stays_on_the_floor_at_every_swivel_angle(robot, prefix):
    """Swivelling and rolling do not lift or sink the base: the contact sphere touches z 0."""
    link = robot.find("link[@name='%s_wheel_link']" % prefix)
    (col,) = link.findall('collision')
    radius = float(col.find('geometry/sphere').get('radius'))
    assert radius == pytest.approx(DATASHEET['wheel_radius'], abs=1e-9)
    swivel = '%s_caster_joint' % prefix
    roll = '%s_wheel_joint' % prefix
    for a in (0.0, 0.9, math.pi / 2, math.pi):
        for b in (0.0, 1.3, math.pi):
            centre = (_world(robot, '%s_wheel_link' % prefix, {swivel: a, roll: b}) @
                      _tf(col.find('origin')))[:3, 3]
            assert centre[2] - radius == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('prefix', PREFIXES)
def test_joint_dynamics_come_from_the_castor_model(robot, prefix):
    for joint, kind in (('%s_caster_joint' % prefix, 'swivel'),
                        ('%s_wheel_joint' % prefix, 'roll')):
        dyn = _joint(robot, joint).find('dynamics')
        assert dyn is not None, joint
        assert float(dyn.get('friction')) == pytest.approx(CASTOR[kind + '_friction']['value'])
        assert float(dyn.get('damping')) == pytest.approx(CASTOR[kind + '_damping']['value'])
        assert float(dyn.get('friction')) > 0 and float(dyn.get('damping')) > 0


def test_castor_masses_and_links_are_unchanged(robot):
    """No new link; the castor and castor-wheel links keep the mass export split."""
    with open(EXPORT_FILE, encoding='utf-8') as f:
        links = yaml.safe_load(f)['groups']['base']['links']
    names = {link.get('name') for link in robot.findall('link')}
    for prefix in PREFIXES:
        for kind, kg in (('caster', 0.22), ('wheel', 0.08)):
            name = '%s_%s_link' % (prefix, kind)
            assert name in names
            mass = float(robot.find("link[@name='%s']/inertial/mass" % name).get('value'))
            assert mass == pytest.approx(links[name]['mass_kg']) == pytest.approx(kg)
    castor_links = {n for n in names if n.endswith(('_caster_link', '_wheel_link'))
                    and n[:2] in PREFIXES}
    assert len(castor_links) == 8, sorted(castor_links)


def test_swivel_aligns_and_wheel_rolls_with_the_estimated_values():
    """Plausibility of the estimates: contact grip turns the fork and spins the wheel.

    Per-castor normal load N from the castor_model.yaml method (robot 86.3 kg, drive pair
    67.5 kg, four castors). The aligning torque mu N trail must exceed the swivel friction,
    and the rolling force roll_friction / r must be far below the grip mu N; otherwise the
    castor would drag in simulation like the old fixed model.
    """
    normal = (86.3 - 67.5) / 4 * 9.81
    mu = CASTOR['contact_mu']['value']
    aligning = mu * normal * CASTOR['trail']['value']
    assert aligning > 5 * CASTOR['swivel_friction']['value']
    rolling_force = CASTOR['roll_friction']['value'] / CASTOR['wheel_radius']['value']
    assert rolling_force < 0.1 * mu * normal
