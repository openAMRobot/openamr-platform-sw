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

Since the castor-swivel proposal (15 Nov 2026) the castor joints are passive continuous
joints, so the castor wheel links are no longer lumped into base_footprint; their friction
comes from config/castor_model.yaml.

The SDF tests need `gz sdf` (Gazebo Harmonic). Without gz they skip, unless
OPENAMR_REQUIRE_GZ=1 is set, as in the description-gazebo-tests CI job: then a
missing gz fails. A non-zero or empty `gz sdf -p` result always fails.
"""

import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

import yaml

PKG = Path(__file__).resolve().parents[1]
URDF = PKG / 'urdf' / 'robo_urdf.urdf.xacro'

# The values gazebo_control.xacro already carried before this change.
DRIVE = {'mu': 1.5, 'mu2': 0.5, 'fdir1': [1.0, 0.0, 0.0]}
# Castor-swivel proposal (15 Nov 2026): the castors swivel and roll on passive joints, so their
# contact grips. The values are read from config/castor_model.yaml, their single source; the
# literal checks below pin them so that a silent edit of that file fails here as well.
# Was {'mu': 0.0, 'mu2': 0.0}, the frictionless slider on fixed joints.
with open(PKG / 'config' / 'castor_model.yaml', encoding='utf-8') as _f:
    CASTOR_MODEL = yaml.safe_load(_f)['castor']
CASTER = {'mu': CASTOR_MODEL['contact_mu']['value'], 'mu2': CASTOR_MODEL['contact_mu']['value'],
          'fdir1': [float(v) for v in CASTOR_MODEL['contact_fdir1']['value']]}
assert CASTER == {'mu': 0.5, 'mu2': 0.5, 'fdir1': [0.0, 1.0, 0.0]}, CASTER
DRIVE_LINKS = ('left_wheel_link', 'right_wheel_link')
CASTER_LINKS = ('fl_wheel_link', 'bl_wheel_link', 'fr_wheel_link', 'br_wheel_link')


@pytest.fixture(scope='module')
def urdf_xml():
    """Return the expanded URDF text."""
    import xacro
    return xacro.process_file(str(URDF)).toxml()


@pytest.fixture(scope='module')
def sdf_root(urdf_xml, tmp_path_factory):
    """Return the SDF from gz sdf -p; fail on a broken conversion."""
    if shutil.which('gz') is None:
        if os.environ.get('OPENAMR_REQUIRE_GZ') == '1':
            pytest.fail('gz (Gazebo) is not installed and OPENAMR_REQUIRE_GZ=1')
        pytest.skip('gz (Gazebo) is not installed; set OPENAMR_REQUIRE_GZ=1 to fail instead')
    path = tmp_path_factory.mktemp('friction') / 'robot.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True,
                            text=True, timeout=120)
    if result.returncode != 0 or not result.stdout.strip():
        pytest.fail(f'gz sdf -p failed (exit {result.returncode}) or printed nothing: '
                    f'{result.stderr[-500:]}')
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


def _ode_value(ode, tag, link):
    """Return a friction value; an empty <ode/> means Gazebo uses its default mu 1."""
    text = ode.findtext(tag)
    assert text is not None, f'{link}: <{tag}> missing in the SDF, Gazebo uses its default'
    return text


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
    """The castor wheels carry mu1, mu2 and fdir1 of castor_model.yaml (rolling castor)."""
    tags = _gazebo_tags(urdf_xml, link)
    assert float(tags['mu1']) == CASTER['mu']
    assert float(tags['mu2']) == CASTER['mu2']
    assert [float(v) for v in tags['fdir1'].split()] == CASTER['fdir1']


def test_caster_geometry_is_the_castor_model(urdf_xml):
    """Castor wheels are D35 contact spheres on passive swivel and roll joints."""
    root = ET.fromstring(urdf_xml)
    for link in CASTER_LINKS:
        sphere = root.find(f"link[@name='{link}']/collision/geometry/sphere")
        assert sphere is not None
        assert float(sphere.get('radius')) == pytest.approx(CASTOR_MODEL['wheel_radius']['value'])
        assert float(sphere.get('radius')) == pytest.approx(0.0175)
        for joint_name in (f'{link[:2]}_wheel_joint', f'{link[:2]}_caster_joint'):
            joint = root.find(f"joint[@name='{joint_name}']")
            assert joint is not None and joint.get('type') == 'continuous', joint_name


@pytest.mark.parametrize('link', DRIVE_LINKS)
def test_drive_wheel_friction_reaches_sdf_collision(sdf_root, link):
    """The drive wheel collisions carry the friction in the SDF."""
    ode = _collision_ode(sdf_root, link)
    assert float(_ode_value(ode, 'mu', link)) == DRIVE['mu']
    assert float(_ode_value(ode, 'mu2', link)) == DRIVE['mu2']
    assert [float(v) for v in _ode_value(ode, 'fdir1', link).split()] == DRIVE['fdir1']


@pytest.mark.parametrize('link', CASTER_LINKS)
def test_caster_friction_reaches_sdf_collision(sdf_root, link):
    """Every castor wheel collision carries its friction and direction in the SDF."""
    ode = _collision_ode(sdf_root, link)
    assert float(_ode_value(ode, 'mu', link)) == CASTER['mu']
    assert float(_ode_value(ode, 'mu2', link)) == CASTER['mu2']
    assert [float(v) for v in _ode_value(ode, 'fdir1', link).split()] == CASTER['fdir1']
