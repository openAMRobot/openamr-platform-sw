# SPDX-License-Identifier: MIT
"""Collision support points must match the CAD-derived visual component map."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro
import yaml
PKG=Path(__file__).resolve().parents[1]
MAP=PKG.parents[2]/'tools/cad/base_mesh_map.yaml'
@pytest.fixture(scope='module')
def robot():return ET.fromstring(xacro.process_file(str(PKG/'urdf/robo_urdf.urdf.xacro')).toxml())
@pytest.mark.parametrize('side',['left','right'])
def test_drive_contact_uses_the_full_cad_tyre_width(robot,side):
    profile=yaml.safe_load(MAP.read_text())['components']['hub_motor_wheel']['parts'][0]['profile_mm']
    radius=max(p[1] for p in profile); axial=[p[0] for p in profile if p[1]==radius]
    tyre=robot.find(f"link[@name='{side}_wheel_link']/collision/geometry/cylinder")
    assert float(tyre.get('radius'))==pytest.approx(radius/1000)
    assert float(tyre.get('length'))==pytest.approx((max(axial)-min(axial))/1000)
@pytest.mark.parametrize('prefix',['fl','fr','bl','br'])
def test_castor_contact_and_housing_follow_cad_without_moving_joint_frames(robot,prefix):
    parts=yaml.safe_load(MAP.read_text())['components']['castor']['parts']
    joint=robot.find(f"joint[@name='{prefix}_wheel_joint']")
    origin=[float(v) for v in joint.find('origin').get('xyz').split()]
    assert origin==pytest.approx([-.015,0,-.027746])
    collision=robot.find(f"link[@name='{prefix}_wheel_link']/collision")
    offset=[float(v) for v in collision.find('origin').get('xyz').split()]
    assert [a+b for a,b in zip(origin,offset)]==pytest.approx([v/1000 for v in parts[2]['xyz_mm']])
    assert float(collision.find('geometry/sphere').get('radius'))==pytest.approx(parts[2]['radius_mm']/1000)
    housing=robot.findall(f"link[@name='{prefix}_caster_link']/collision")
    assert len(housing)==2
    for collision,part in zip(housing,parts[:2]):
        assert [float(v) for v in collision.find('geometry/box').get('size').split()]==pytest.approx([v/1000 for v in part['size_mm']])
