# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
# SPDX-License-Identifier: MIT
"""The wheel_contact argument selects the drive wheel collision shape; the default is unchanged.

cylinder (default) is the 46 mm wide cylinder of the loaded radius. sphere and narrow_cylinder are
the two candidates of platform-sw#168 finding F1. Every shape touches the floor plane at the same
height, so the frames, DiffDrive and the castors do not change with the argument.
"""
import os
import xml.etree.ElementTree as ET

import pytest

from test_ground_contact import _world, WHEELS

import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
LOADED_RADIUS = 0.098
SHAPES = {
    'cylinder': ('cylinder', {'radius': LOADED_RADIUS, 'length': 0.046}),
    'sphere': ('sphere', {'radius': LOADED_RADIUS}),
    'narrow_cylinder': ('cylinder', {'radius': LOADED_RADIUS, 'length': 0.004}),
}


def _robot(**args):
    return ET.fromstring(xacro.process_file(XACRO, mappings=args).toxml())


def _wheel_geometry(robot, link):
    (el,) = [e for e in robot.findall('link') if e.get('name') == link]
    (col,) = el.findall('collision')
    (geom,) = list(col.find('geometry'))
    return col, geom.tag, {k: float(v) for k, v in geom.attrib.items()}


def _strip(robot):
    """The robot without the drive wheel collisions, as text, to compare everything else."""
    for link in robot.findall('link'):
        if link.get('name') in WHEELS:
            for col in link.findall('collision'):
                link.remove(col)
    return ET.tostring(robot)


def test_default_is_the_46_mm_cylinder():
    robot = _robot()
    for w in WHEELS:
        col, tag, attrs = _wheel_geometry(robot, w)
        assert (tag, attrs) == SHAPES['cylinder'], (w, tag, attrs)
        assert col.find('origin').get('rpy') == '1.5707963 0 0'


def test_default_equals_explicit_cylinder():
    assert ET.tostring(_robot()) == ET.tostring(_robot(wheel_contact='cylinder'))


@pytest.mark.parametrize('shape', sorted(SHAPES))
def test_shape_and_radius(shape):
    robot = _robot(wheel_contact=shape)
    for w in WHEELS:
        _col, tag, attrs = _wheel_geometry(robot, w)
        assert (tag, attrs) == SHAPES[shape], (shape, w, tag, attrs)


@pytest.mark.parametrize('shape', sorted(SHAPES))
def test_every_shape_rests_on_the_floor_plane(shape):
    robot = _robot(wheel_contact=shape)
    world = _world(robot)
    for w in WHEELS:
        col, _tag, attrs = _wheel_geometry(robot, w)
        z = world[w][2, 3] + float(col.find('origin').get('xyz').split()[2])
        assert abs(z - attrs['radius']) <= 1e-9, (shape, w, z, attrs['radius'])


@pytest.mark.parametrize('shape', ['sphere', 'narrow_cylinder'])
def test_only_the_drive_wheel_collision_changes(shape):
    assert _strip(_robot(wheel_contact=shape)) == _strip(_robot())


def test_diff_drive_radius_is_the_loaded_radius_for_every_shape():
    for shape in SHAPES:
        robot = _robot(wheel_contact=shape)
        assert [float(e.text) for e in robot.iter('wheel_radius')] == [LOADED_RADIUS], shape


def test_unknown_shape_is_an_error():
    with pytest.raises(Exception):
        _robot(wheel_contact='box')
