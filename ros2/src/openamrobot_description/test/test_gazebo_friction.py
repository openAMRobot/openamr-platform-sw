# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Wheel and caster friction must reach the converted SDF, not only the URDF.

gazebo_control.xacro used to write the friction as a raw <surface> under
<gazebo reference>. The URDF to SDF conversion does not apply that to the link's
collisions, so every wheel contact ran at the default mu 1 and the four caster
spheres, lumped into base_footprint by their fixed joints, dragged at mu 1.
<mu1>, <mu2> and <fdir1> are applied to every collision of the link.
"""

from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

PKG = Path(__file__).resolve().parents[1]
URDF = PKG / 'urdf' / 'robo_urdf.urdf.xacro'

# The values gazebo_control.xacro already carried before this change.
DRIVE = {'mu': 1.5, 'mu2': 0.5, 'fdir1': [1.0, 0.0, 0.0]}
CASTER = {'mu': 0.0, 'mu2': 0.0}
DRIVE_LINKS = ('left_wheel_link', 'right_wheel_link')
CASTER_LINKS = ('fl_wheel_link', 'bl_wheel_link', 'fr_wheel_link', 'br_wheel_link')


@pytest.fixture(scope='module')
def urdf_xml():
    """Return the expanded URDF text."""
    import xacro
    return xacro.process_file(str(URDF)).toxml()


@pytest.fixture(scope='module')
def sdf_root(urdf_xml, tmp_path_factory):
    """Return the SDF from gz sdf -p; skip if gz is missing."""
    if shutil.which('gz') is None:
        pytest.skip('gz (Gazebo) is not installed')
    path = tmp_path_factory.mktemp('friction') / 'robot.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True,
                            text=True, timeout=120)
    if result.returncode != 0 or not result.stdout.strip():
        pytest.skip(f'gz sdf -p is not usable here: {result.stderr[-200:]}')
    return ET.fromstring(result.stdout)


def _gazebo_tags(urdf_xml, link):
    root = ET.fromstring(urdf_xml)
    out = {}
    for g in root.findall('gazebo'):
        if g.get('reference') == link:
            out.update({c.tag: (c.text or '').strip() for c in g})
    return out


def _collision_ode(sdf_root, link):
    """Friction of the collision that came from `link`, lumped or not."""
    found = [c for c in sdf_root.iter('collision')
             if c.get('name', '').endswith(f'{link}_collision')
             or f'{link}_collision_' in c.get('name', '')]
    assert len(found) == 1, (link, [c.get('name') for c in found])
    ode = found[0].find('surface/friction/ode')
    assert ode is not None, f'{link}: no friction on its collision, the conversion dropped it'
    return ode


def test_no_raw_surface_under_a_wheel_reference(urdf_xml):
    """A <surface> directly under <gazebo reference> is dropped by the conversion."""
    root = ET.fromstring(urdf_xml)
    for g in root.findall('gazebo'):
        if g.get('reference') in DRIVE_LINKS + CASTER_LINKS:
            assert g.find('surface') is None, g.get('reference')


@pytest.mark.parametrize('link', DRIVE_LINKS)
def test_drive_wheel_friction_tags_in_urdf(urdf_xml, link):
    """The drive wheels carry mu1, mu2 and fdir1."""
    tags = _gazebo_tags(urdf_xml, link)
    assert float(tags['mu1']) == DRIVE['mu']
    assert float(tags['mu2']) == DRIVE['mu2']
    assert [float(v) for v in tags['fdir1'].split()] == DRIVE['fdir1']


@pytest.mark.parametrize('link', CASTER_LINKS)
def test_caster_friction_tags_in_urdf(urdf_xml, link):
    """The casters carry mu1 0 and mu2 0."""
    tags = _gazebo_tags(urdf_xml, link)
    assert float(tags['mu1']) == CASTER['mu']
    assert float(tags['mu2']) == CASTER['mu2']


def test_caster_geometry_is_unchanged(urdf_xml):
    """The casters stay 26 mm collision spheres on fixed joints."""
    root = ET.fromstring(urdf_xml)
    for link in CASTER_LINKS:
        sphere = root.find(f"link[@name='{link}']/collision/geometry/sphere")
        assert sphere is not None and float(sphere.get('radius')) == pytest.approx(0.026)
        joint = root.find(f"joint[@name='{link[:2]}_wheel_joint']")
        assert joint is not None and joint.get('type') == 'fixed'


@pytest.mark.parametrize('link', DRIVE_LINKS)
def test_drive_wheel_friction_reaches_sdf_collision(sdf_root, link):
    """The drive wheel collisions carry the friction in the SDF."""
    ode = _collision_ode(sdf_root, link)
    assert float(ode.findtext('mu')) == DRIVE['mu']
    assert float(ode.findtext('mu2')) == DRIVE['mu2']
    assert [float(v) for v in ode.findtext('fdir1').split()] == DRIVE['fdir1']


@pytest.mark.parametrize('link', CASTER_LINKS)
def test_caster_friction_reaches_sdf_collision(sdf_root, link):
    """Every lumped caster collision carries mu 0 in the SDF."""
    ode = _collision_ode(sdf_root, link)
    assert float(ode.findtext('mu')) == CASTER['mu']
    assert float(ode.findtext('mu2')) == CASTER['mu2']
