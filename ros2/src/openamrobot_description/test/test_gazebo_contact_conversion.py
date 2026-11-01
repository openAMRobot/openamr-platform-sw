# Copyright 2026 OpenAMRobot contributors
# Licensed under the MIT License (see LICENSE).

"""Verify actual SDFormat contact properties, including fixed-joint lumping."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro

PKG = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def sdf(tmp_path_factory):
    path = tmp_path_factory.mktemp('contact_conversion') / 'robot.urdf'
    path.write_text(xacro.process_file(str(PKG / 'urdf/robo_urdf.urdf.xacro')).toxml())
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True,
                            text=True, check=True, timeout=30)
    return ET.fromstring(result.stdout)


@pytest.mark.parametrize('link,mu,mu2', [
    ('base_link', 0.2, 0.2),
    ('left_wheel_link', 1.0, 1.0), ('right_wheel_link', 1.0, 1.0),
    ('fl_wheel_link', 0.0, 0.0), ('fr_wheel_link', 0.0, 0.0),
    ('bl_wheel_link', 0.0, 0.0), ('br_wheel_link', 0.0, 0.0),
])
def test_contact_coefficients_reach_collision_surface(sdf, link, mu, mu2):
    """The existing intended coefficients must survive URDF conversion/lumping."""
    collisions = [c for c in sdf.findall('.//collision')
                  if f'{link}_collision' in c.get('name', '')]
    assert len(collisions) == 1, (link, [c.get('name') for c in collisions])
    ode = collisions[0].find('surface/friction/ode')
    assert ode is not None
    assert ode.findtext('mu') is not None, f'{link}: contact friction was discarded'
    assert float(ode.findtext('mu')) == pytest.approx(mu)
    assert float(ode.findtext('mu2')) == pytest.approx(mu2)
    bullet = collisions[0].find('surface/friction/bullet')
    assert bullet is not None, f'{link}: Bullet Featherstone cannot read ODE coefficients'
    assert float(bullet.findtext('friction')) == pytest.approx(mu)
    assert float(bullet.findtext('friction2')) == pytest.approx(mu2)
    assert float(collisions[0].findtext('surface/friction/torsional/coefficient')) == 0.0
    if link in ('left_wheel_link', 'right_wheel_link'):
        assert [float(v) for v in ode.findtext('fdir1').split()] == [1.0, 0.0, 0.0]
        assert float(ode.findtext('slip1')) == pytest.approx(0.001)
        assert float(ode.findtext('slip2')) == pytest.approx(0.001)


def test_surface_is_never_inserted_as_a_link_child(sdf):
    """SDF has collision/surface, never link/surface (silently ignored by Gazebo)."""
    assert not sdf.findall('.//link/surface')


@pytest.mark.parametrize('prefix', ['fl', 'fr', 'bl', 'br'])
def test_castor_keeps_its_own_contact_body(sdf, prefix):
    """Bullet has one friction setting per link; castors must not inherit chassis friction."""
    link = sdf.find(f".//link[@name='{prefix}_wheel_link']")
    assert link is not None, f'{prefix}: lumped into chassis collider'
    assert float(link.findtext('inertial/mass')) > 0
    joint = sdf.find(f".//joint[@name='{prefix}_wheel_joint']")
    assert joint is not None and joint.get('type') == 'fixed'


def test_legacy_camera_info_reaches_its_existing_bridge_topic(sdf):
    sensor = sdf.find(".//sensor[@name='camera_sensor']")
    assert sensor.findtext('camera/camera_info_topic') == '/camera/camera_info'
