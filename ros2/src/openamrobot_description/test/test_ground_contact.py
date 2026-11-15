# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
# SPDX-License-Identifier: MIT
"""At rest the drive wheels and the castors stand on one plane and the base is level.

Owner decision 15 Nov 2026 (Alex): the 200 mm tyre is mounted 2 mm below the castor plane on
purpose. Under the robot weight the rubber compresses about 2 mm and the rigid castors do not,
so in the loaded state every contact is on the floor plane and base_link is not pitched. The
description models that loaded state: wheel collision radius = loaded radius = axle height.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
TOL = 0.0005  # m, half a millimetre: CAD placement precision of the castor housings
WHEELS = ('left_wheel_link', 'right_wheel_link')


def _robot():
    return ET.fromstring(xacro.process_file(XACRO).toxml())


def _tf(origin):
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = xyz
    return m


def _world(robot):
    """Transform of every link in base_footprint at zero joint positions."""
    parent = {}
    for j in robot.findall('joint'):
        parent[j.find('child').get('link')] = (j.find('parent').get('link'), _tf(j.find('origin')))
    cache = {}

    def get(link):
        if link not in cache:
            if link not in parent:
                cache[link] = np.eye(4)
            else:
                up, t = parent[link]
                cache[link] = get(up) @ t
        return cache[link]
    return {link.get('name'): get(link.get('name')) for link in robot.findall('link')}


def _lowest_points(robot):
    """Lowest z of each wheel cylinder and each castor sphere, in base_footprint."""
    world = _world(robot)
    out = {}
    for link in robot.findall('link'):
        name = link.get('name')
        for col in link.findall('collision'):
            geom = col.find('geometry')
            t = world[name] @ _tf(col.find('origin'))
            sphere = geom.find('sphere')
            cyl = geom.find('cylinder')
            if sphere is not None and name not in WHEELS:
                out[name] = t[2, 3] - float(sphere.get('radius'))
            elif cyl is not None and name in WHEELS:
                # cylinder axis is the local z of the collision frame, horizontal here
                axis = t[:3, 2]
                assert abs(axis[2]) < 1e-6, '%s cylinder axis is not horizontal' % name
                out[name] = t[2, 3] - float(cyl.get('radius'))
    return out


def test_drive_wheels_rest_on_the_floor_plane():
    low = _lowest_points(_robot())
    for w in WHEELS:
        assert w in low, 'no cylinder collision on %s' % w
        assert abs(low[w]) <= 1e-6, '%s bottom at %.4f m, expected 0 (loaded radius)' % (w, low[w])


def test_castors_rest_on_the_same_plane():
    low = _lowest_points(_robot())
    castors = {k: v for k, v in low.items() if k not in WHEELS}
    assert len(castors) == 4, 'expected four castor contact spheres, found %s' % sorted(castors)
    for name, z in castors.items():
        # One-sided: a castor below the floor would lift the drive wheels off it.
        assert z >= 0.0, '%s contact %.4f m below the floor; it would lift the drive wheels' % (name, -z)
        assert z <= TOL, '%s contact at %.4f m above the floor (tolerance %.4f)' % (name, z, TOL)


def test_diff_drive_radius_equals_loaded_collision_radius():
    robot = _robot()
    radius = {float(c.find('geometry/cylinder').get('radius'))
              for link in robot.findall('link') if link.get('name') in WHEELS
              for c in link.findall('collision')}
    plugin = [float(e.text) for e in robot.iter('wheel_radius')]
    assert len(radius) == 1 and plugin, 'wheel radius or DiffDrive wheel_radius missing'
    assert plugin == [radius.pop()], 'DiffDrive wheel_radius differs from the collision radius'
