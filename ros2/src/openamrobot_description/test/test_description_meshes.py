# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""Checks of the 2.0 base description: URDF, visual meshes, frames and mesh provenance."""

import hashlib
import math
from pathlib import Path
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
URDF = PKG / 'urdf' / 'robo_urdf.urdf.xacro'
VISUAL = PKG / 'meshes' / 'visual'
PROVENANCE = PKG / 'meshes' / 'PROVENANCE.yaml'
FRAMES = PKG / 'config' / 'sensor_frames.yaml'
PREFIX = 'package://openamrobot_description/'

# The 2.0 base components of BOM Issue 7.11 that the model must account for.
BOM_COMPONENTS = [
    'hub_motor_wheel', 'castor', 'motor_driver', 'ultrasonic', 'tof', 'lidar', 'base_camera',
    'imu', 'compute', 'base_controller', 'battery', 'safety_relay_set', 'estop', 'reset_button',
    'wireless_charging_receiver',
]
# Release 001 drivetrain parts that must be excluded from every mesh.
RELEASE_001 = ['GEVN_200', 'Мотор-редуктор', 'Rexnord-UCFL', 'MMP.03.00.00.001',
               'MMP.03.00.00.004', 'PT1.00.00.240']

FLOOR_Z = -0.098
FLOOR_TOL = 0.002 + 1e-6


@pytest.fixture(scope='module')
def urdf_xml():
    import xacro
    return xacro.process_file(str(URDF)).toxml()


@pytest.fixture(scope='module')
def robot(urdf_xml):
    return ET.fromstring(urdf_xml)


@pytest.fixture(scope='module')
def provenance():
    with open(PROVENANCE, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(element):
    o = element.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(rpy)
    m[:3, 3] = xyz
    return m


def read_stl(path):
    data = Path(path).read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    assert len(data) == 84 + 50 * count, '%s is not a binary STL' % path
    dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    return rec['v'].reshape(-1, 3).astype(np.float64)


def mesh_file(mesh):
    name = mesh.get('filename')
    assert name.startswith(PREFIX), name
    return PKG / name[len(PREFIX):]


def link_points_in_base_link(robot, link_name):
    """Visual mesh vertices of a link, in base_link (the link must hang from base_link)."""
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    pose = np.eye(4)
    name = link_name
    while name != 'base_link':
        joint = joints[name]
        pose = origin(joint) @ pose
        name = joint.find('parent').get('link')
    link = next(lk for lk in robot.findall('link') if lk.get('name') == link_name)
    pts = []
    for vis in link.findall('visual'):
        mesh = vis.find('geometry/mesh')
        if mesh is None:
            continue
        v = read_stl(mesh_file(mesh))
        m = pose @ origin(vis)
        pts.append(v @ m[:3, :3].T + m[:3, 3])
    assert pts, 'no visual mesh on %s' % link_name
    return np.concatenate(pts)


def test_xacro_expands_and_parses(urdf_xml, tmp_path):
    robot = ET.fromstring(urdf_xml)
    assert robot.tag == 'robot'
    names = [lk.get('name') for lk in robot.findall('link')]
    assert len(names) == len(set(names))
    check = shutil.which('check_urdf')
    if check is None:
        pytest.fail('check_urdf not found; install liburdfdom-tools (ros-jazzy-urdfdom)')
    path = tmp_path / 'robot.urdf'
    path.write_text(urdf_xml, encoding='utf-8')
    out = subprocess.run([check, str(path)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    assert 'Successfully Parsed XML' in out.stdout


def test_referenced_meshes_exist_and_none_unreferenced(robot):
    referenced = set()
    for mesh in robot.iter('mesh'):
        path = mesh_file(mesh)
        assert path.is_file(), 'missing mesh %s' % path
        if path.parent == VISUAL:
            referenced.add(path.name)
    on_disk = {p.name for p in VISUAL.iterdir() if p.is_file()}
    unreferenced = sorted(on_disk - referenced)
    assert not unreferenced, 'unreferenced visual meshes: %s' % unreferenced
    sdf = (PKG / 'urdf' / 'robot.sdf').read_text(encoding='utf-8')
    for uri in ET.fromstring(sdf).iter('uri'):
        assert (PKG / uri.text.strip()[len(PREFIX):]).is_file(), 'robot.sdf: missing %s' % uri.text


def test_base_link_visual_bounding_box(robot):
    pts = link_points_in_base_link(robot, 'base_link')
    lo, hi = pts.min(0), pts.max(0)
    assert lo[0] >= -0.393 and hi[0] <= 0.393, (lo, hi)
    assert lo[1] >= -0.294 and hi[1] <= 0.294, (lo, hi)
    assert lo[2] >= -0.050 and hi[2] <= 0.300, (lo, hi)
    assert lo[0] <= -0.385 and hi[0] >= 0.385, 'must span the panel faces: %s %s' % (lo, hi)


@pytest.mark.parametrize('link', ['left_wheel_link', 'right_wheel_link'])
def test_wheel_lowest_point_on_the_floor(robot, link):
    low = link_points_in_base_link(robot, link)[:, 2].min()
    assert abs(low - FLOOR_Z) <= FLOOR_TOL, '%s lowest point %.4f' % (link, low)


@pytest.mark.parametrize(
    'link', ['fl_caster_link', 'fr_caster_link', 'bl_caster_link', 'br_caster_link'])
def test_castor_contact_on_the_floor(robot, link):
    low = link_points_in_base_link(robot, link)[:, 2].min()
    assert abs(low - FLOOR_Z) <= FLOOR_TOL, '%s lowest point %.4f' % (link, low)


def test_sensor_joint_origins_equal_sensor_frames(robot):
    with open(FRAMES, encoding='utf-8') as stream:
        frames = yaml.safe_load(stream)['frames']
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    for name, frame in frames.items():
        joint = joints[name]
        assert joint.find('parent').get('link') == 'base_link', name
        m = origin(joint)
        np.testing.assert_allclose(m[:3, 3], frame['xyz'], atol=1e-6, err_msg=name)
        np.testing.assert_allclose(m[:3, :3], rpy_matrix(frame['rpy']), atol=1e-6, err_msg=name)


def test_component_inventory(provenance):
    comps = provenance['components']
    for cid in BOM_COMPONENTS:
        assert cid in comps, 'PROVENANCE.yaml does not list %s' % cid
        comp = comps[cid]
        assert comp['status'] in ('modelled', 'not modelled'), cid
        if comp['status'] == 'modelled':
            assert comp['envelope'] is True and comp['dimension_source'].strip(), cid
        else:
            assert comp.get('reason', '').strip(), '%s is not modelled without a reason' % cid
    for group in provenance['groups']:
        if 'output' in group:
            assert group['class'] == 'chassis', group['group']
        if group['class'] == 'release_001':
            assert 'output' not in group, group['group']
    groups = provenance['groups']
    release = [i for g in groups if g['class'] == 'release_001' for i in g['instances']]
    meshed = [i for g in groups if 'output' in g for i in g['instances']]
    for name in RELEASE_001:
        assert any(name in i for i in release), '%s is not listed as release 001' % name
        assert not any(name in i for i in meshed), '%s is in a mesh group' % name


def test_visual_mesh_size_budget():
    sizes = {p.name: p.stat().st_size for p in VISUAL.iterdir() if p.is_file()}
    for name, size in sizes.items():
        assert size <= 2000000, '%s is %d bytes' % (name, size)
    assert sum(sizes.values()) <= 8000000, sum(sizes.values())


def test_provenance_sha256_matches_files(provenance, robot):
    outputs = provenance['outputs']
    assert outputs
    files = set()
    for out in outputs.values():
        path = PKG / out['file']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == out['sha256'], out['file']
        files.add(path.name)
    assert files == {p.name for p in VISUAL.iterdir() if p.is_file()}


def test_urdf_materials_match_provenance(robot, provenance):
    colours = {m.get('name'): [float(v) for v in m.find('color').get('rgba').split()]
               for m in robot.findall('material') if m.find('color') is not None}
    for out in provenance['outputs'].values():
        np.testing.assert_allclose(colours[out['material']], out['rgba'], atol=1e-3,
                                   err_msg=out['material'])


def test_glb_nodes_have_id_class_and_source(provenance):
    """The docs GLB has one node per part with id, class and source (PROVENANCE.yaml glb.nodes)."""
    glb = provenance['glb']
    nodes = glb['nodes']
    assert nodes and glb['node_count'] == len(nodes)
    assert glb['bytes'] <= 1600000, glb['bytes']
    ids = [n.get('id') for n in nodes]
    assert all(ids) and len(ids) == len(set(ids)), ids
    for node in nodes:
        assert node.get('class') in ('chassis', 'component', 'envelope'), node
        assert node.get('source', '').strip(), node
    by_id = {n['id']: n for n in nodes}
    for cid, comp in provenance['components'].items():
        if comp['status'] != 'modelled':
            continue
        for nid in comp['node_ids']:
            assert nid in by_id, '%s: no GLB node %s' % (cid, nid)
            assert by_id[nid]['class'] == comp['class'], nid
            assert by_id[nid].get('provisional', False) == comp.get('provisional', False), nid
    assert by_id['battery']['class'] == 'envelope' and by_id['battery']['provisional'] is True
