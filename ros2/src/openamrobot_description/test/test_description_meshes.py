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

# The 2.0 base components of BOM Issue 7.14 that the model must account for.
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


def test_base_link_visual_bounding_box(robot, provenance):
    pts = link_points_in_base_link(robot, 'base_link')
    lo, hi = pts.min(0), pts.max(0)
    # only the front-panel buttons (P-03 rev20.0 item 24) may stand proud of the panel faces
    front = max([0.393] + [c['panel_hole']['parts_x_mm'][1] / 1000.0 + 1e-4
                           for c in provenance['components'].values() if 'panel_hole' in c])
    assert front < 0.393 + 0.040, front
    assert lo[0] >= -0.393 and hi[0] <= front, (lo, hi)
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


def test_battery_cell_group_envelope_recorded(provenance):
    """Record the cell-group envelope, position, clearances and free space."""
    rec = provenance['components']['battery']['rear_deck_check']
    env = rec['envelope_mm']
    assert (env['x'], env['y'], env['z']) == (140.0, 314.0, 210.0), env
    assert rec['envelope'].startswith('cell-group envelope, provisional')
    box = rec['box_mm']
    assert box['x'] == [rec['rear_face_x_mm'], rec['front_face_x_mm']]
    assert abs(rec['front_face_x_mm'] - rec['rear_face_x_mm'] - 140.0) < 1e-6
    assert abs(rec['reference_rear_face_x_mm'] - rec['front_face_x_mm'] - 2.0) < 0.011
    assert box['y'] == [-157.0, 157.0] and box['z_floor'] == [54.0, 264.0], box
    subjects = {r['subject'] for r in rec['clearances']}
    for s in (
        'centre bracket', 'top-cover support left',
        'top-cover support right', 'rear sloped cover flange',
        'cover rear edge', 'cover underside (above the envelope)',
        'base plate (above the deck top)'
    ):
        assert s in subjects, s
    smallest = min(r['clearance_mm'] for r in rec['clearances_by_part'])
    assert rec['smallest_clearance']['clearance_mm'] == smallest
    assert rec['fits_with_clearance'] == (smallest >= 2.0 and all(
        r['clearance_mm'] is None or r['clearance_mm'] >= 2.0 for r in rec['clearances']))
    assert [f['name'].split(' (')[0] for f in rec['free_space']] == [
        'behind the envelope', 'left of the envelope', 'right of the envelope']
    assert 'BMS and power distribution zone (not modelled)' in rec['free_space'][0]['name']
    assert provenance['components']['battery'].get('provisional') is True


def test_battery_drawn_as_eight_cells_inside_the_envelope(provenance):
    """Check eight LF105 cell boxes fit inside the envelope without touching."""
    rec = provenance['components']['battery']['rear_deck_check']
    box, cells = rec['box_mm'], rec['cells_mm']['boxes']
    assert len(cells) == 8, cells
    for c in cells:
        size = [round(c[k][1] - c[k][0], 2) for k in ('x', 'y', 'z')]
        assert size == [130.0, 38.0, 200.0], size
        assert box['x'][0] <= c['x'][0] and c['x'][1] <= box['x'][1], c
        assert box['y'][0] <= c['y'][0] and c['y'][1] <= box['y'][1], c
        assert box['z_base_link'][0] <= c['z'][0] and c['z'][1] <= box['z_base_link'][1], c
    ys = sorted(cells, key=lambda c: c['y'][0])
    assert all(b['y'][0] - a['y'][1] > 0.0 for a, b in zip(ys[:-1], ys[1:])), 'cells touch'
    feats = provenance['components']['battery']['features']
    assert (
        feats['cell'] == 8
        and feats['terminal stud'] == 16
        and feats['envelope shell'] == 1
    ), feats


def test_estop_and_reset_in_release_001_button_holes(provenance):
    """Check E-stop and RESET hole axes and side checks for release 001."""
    nodes = {n['id']: n for n in provenance['glb']['nodes']}
    for cid, side, button in (('estop', 1, 'B200MK'), ('reset_button', -1, 'B100FY')):
        comp = provenance['components'][cid]
        assert comp['status'] == 'modelled' and comp['placeholder'] is True, cid
        assert 'P-03 Decision Addendum rev20.0 item 24' in comp['position_source'], cid
        hole = comp['panel_hole']
        assert button in hole['release001_instance'], hole
        axis = hole['hole_axis_mm']
        assert abs(axis['y'] - side * 130.0) < 0.5 and abs(axis['z'] - 44.5) < 0.5, axis
        assert abs(hole['old_hole_diameter_mm'] - 22.5) < 0.05, hole
        assert hole['required_mounting_hole_mm'] == 22.4
        assert hole['side_ok'] is True and 'finding' not in hole, hole
        assert hole['parts_x_mm'][0] < axis['x_panel_outer_face'] < hole['parts_x_mm'][1], hole
        assert hole['expected_side'].startswith('+y' if side > 0 else '-y')
        assert any(nodes[n]['class'] == 'component' for n in comp['node_ids']), cid
        assert all(nodes[n].get('placeholder') is True for n in comp['node_ids']), cid


@pytest.mark.parametrize('cid, feature, count', [
    ('compute', 'heatsink fin', 12),
    ('compute', 'Orin NX module', 1),
    ('compute', 'J401 carrier board', 1),
    ('base_controller', 'ST morpho header row', 2), ('motor_driver', 'heatsink fin', 10),
    ('lidar', 'window band', 1), ('base_camera', 'lens', 3), ('ultrasonic', 'hex lock nut', 1),
    ('tof', 'lens', 2)])
def test_component_envelope_features(provenance, cid, feature, count):
    """Check recognisable base features from base_mesh_map.yaml."""
    assert provenance['components'][cid].get('features', {}).get(feature) == count, cid


# Files that cite the BOM issue in force (P-03 Decision Addendum rev20.2 item 7).
BOM_CITING = [PKG / 'README.md', FRAMES, PROVENANCE, URDF, PKG / 'urdf' / 'sensors.xacro',
              Path(__file__), PKG.parents[2] / 'tools' / 'cad' / 'README.md',
              PKG.parents[2] / 'tools' / 'cad' / 'base_mesh_map.yaml',
              PKG.parents[2] / 'tools' / 'cad' / 'step_to_description_meshes.py',
              PKG.parents[2] / 'NOTICE.md']


def test_bom_issue_and_component_references(provenance):
    """Check BOM Issue 7.14, absence of click shield, and the specified TDK IMU board."""
    import re
    stale = re.compile(r'BOM (Issue )?7\.(7|10|11)\b')
    for path in BOM_CITING:
        for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            assert not stale.search(line), (
                '%s:%d cites a superseded BOM issue' % (path.name, number)
            )
    with open(BOM_CITING[7], encoding='utf-8') as stream:
        documents = yaml.safe_load(stream)['sources']['documents']
    assert any(d.startswith('BOM Issue 7.14 ') for d in documents), documents
    shield = [
        g for g in provenance['groups']
        if g['group'].startswith('click shield for Nucleo-64')
    ]
    assert len(shield) == 1 and shield[0]['class'] == 'release_001', shield
    assert 'not part of 2.0' in shield[0]['reason'], shield[0]['reason']
    assert 'CTL-002 removed 1 Nov 2026' in shield[0]['reason'], shield[0]['reason']
    assert 'P-03 rev20.2 item 26' in shield[0]['reason'], shield[0]['reason']
    imu = provenance['components']['imu']
    assert imu['bom'].startswith('TDK ICM-42688-P on the TDK EV_ICM-42688-P '), imu['bom']
    assert 'on 6DOF IMU 14 Click' not in imu['bom'], imu['bom']


# Top of the 2.0 base top cover above the floor (base_covers.stl tops out at 0.2951 m).
TOP_COVER_Z = 0.296
# Links allowed above the top cover: the lift interface only. The base model in this
# repository has no lift interface link; the upperbody model adds it on top of base_link.
LIFT_INTERFACE_LINKS = set()


def geometry_points(geometry):
    """Points bounding a URDF visual or collision geometry, in the geometry's own frame."""
    shape = geometry[0]
    if shape.tag == 'mesh':
        scale = np.array([float(v) for v in shape.get('scale', '1 1 1').split()])
        return read_stl(mesh_file(shape)) * scale
    if shape.tag == 'box':
        half = np.array([float(v) for v in shape.get('size').split()]) / 2.0
        return np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half
    if shape.tag == 'sphere':
        r = float(shape.get('radius'))
        return np.array([[s * r if i == k else 0.0 for i in range(3)]
                         for k in range(3) for s in (-1, 1)])
    if shape.tag == 'cylinder':
        r, half = float(shape.get('radius')), float(shape.get('length')) / 2.0
        ring = [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 73)]
        return np.array([[x, y, z] for x, y in ring for z in (-half, half)])
    raise AssertionError('unknown geometry %s' % shape.tag)


def test_nothing_above_the_top_cover_except_the_lift_interface(robot):
    """2.0 configuration: no base visual or collision rises above the top cover (z 0.296 m)."""
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    above = []
    for link in robot.findall('link'):
        if link.get('name') in LIFT_INTERFACE_LINKS:
            continue
        pose, name = np.eye(4), link.get('name')
        while name != 'base_footprint':
            joint = joints[name]
            pose = origin(joint) @ pose
            name = joint.find('parent').get('link')
        for kind in ('visual', 'collision'):
            for element in link.findall(kind):
                m = pose @ origin(element)
                pts = geometry_points(element.find('geometry')) @ m[:3, :3].T + m[:3, 3]
                top = pts[:, 2].max()
                if top > TOP_COVER_Z:
                    above.append('%s %s top %.4f m' % (link.get('name'), kind, top))
    assert not above, 'above the top cover: %s' % above
