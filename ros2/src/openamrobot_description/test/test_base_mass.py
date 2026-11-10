# SPDX-License-Identifier: MIT
"""
Check the 2.0 base masses against the roadmap model, link by link.

Owner decision 8 Nov 2026 (P-03 rev20.5 item 29): the roadmap model is the reference for
mass and placement, one source, every part counted once on the link where it sits.
config/roadmap_mass_export.yaml is a verbatim copy of openamrobot-docs
docs/assets/roadmap/model/mass_export.yaml, pinned in config/roadmap_mass_export.source.yaml.
Every base link with mass takes its groups.base entry; no link carries a hand-written mass.
"""
import hashlib
import math
import os
import xml.etree.ElementTree as ET

import pytest

import xacro

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
EXPORT_FILE = os.path.join(HERE, os.pardir, 'config', 'roadmap_mass_export.yaml')
PIN_FILE = os.path.join(HERE, os.pardir, 'config', 'roadmap_mass_export.source.yaml')
with open(EXPORT_FILE) as f:
    EXPORT = yaml.safe_load(f)
with open(PIN_FILE) as f:
    PIN = yaml.safe_load(f)
LINKS = EXPORT['groups']['base']['links']
# BOM lines on the lift carriage, the static column or the head: never in the base group
NOT_BASE = {'MEC-003', 'MEC-004', 'MEC-012', 'UI-001', 'UI-002', 'LFT-001', 'LFT-003',
            'LFT-004', 'LFT-005', 'LFT-006', 'MEC-018', 'MEC-018:moving', 'ARM-001', 'ARM-003',
            'ARM-004', 'ARM-011', 'AUX-001'}


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def expanded():
    root = ET.fromstring(xacro.process_file(XACRO).toxml())
    links = {e.get('name'): e for e in root.iter('link')}
    joints = {j.find('child').get('link'): j for j in root.iter('joint')}
    return links, joints


def world(joints, link):
    """Rotation and origin of a link in base_footprint at zero joint positions."""
    if link not in joints:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    j = joints[link]
    rp, op = world(joints, j.find('parent').get('link'))
    o = j.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    t = mat_vec(rp, xyz)
    return mat_mul(rp, rpy_matrix(*rpy)), [op[i] + t[i] for i in range(3)]


def inertial_of(link):
    inertial = link.find('inertial')
    if inertial is None:
        return None
    o = inertial.find('origin')
    i = inertial.find('inertia')
    return {'mass': float(inertial.find('mass').get('value')),
            'com': [float(v) for v in o.get('xyz').split()] if o is not None else [0.0, 0.0, 0.0],
            'diag': [float(i.get(k)) for k in ('ixx', 'iyy', 'izz')]}


def test_export_copy_is_the_pinned_one():
    with open(EXPORT_FILE, 'rb') as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    assert digest == PIN['sha256'], \
        'config/roadmap_mass_export.yaml differs from the pinned docs export'
    assert len(str(PIN['commit'])) == 40


def test_every_base_link_with_mass_takes_the_export():
    links, _ = expanded()
    with_mass = {n for n, e in links.items() if e.find('inertial') is not None}
    assert with_mass == set(LINKS), \
        'URDF links with mass %s, export links %s' % (sorted(with_mass), sorted(LINKS))
    for name, want in LINKS.items():
        got = inertial_of(links[name])
        assert got['mass'] == pytest.approx(want['mass_kg'], abs=1e-6), name
        assert got['com'] == pytest.approx(want['com_m'], abs=1e-6), name
        diag = [want['inertia_kgm2'][k] for k in ('ixx', 'iyy', 'izz')]
        assert got['diag'] == pytest.approx(diag, abs=1e-9), name


def test_expanded_urdf_total_and_com_equal_the_export_items():
    """Expanded base URDF total mass and CoM equal the export components, each counted once."""
    links, joints = expanded()
    m_urdf, mom_urdf = 0.0, [0.0, 0.0, 0.0]
    for name, e in links.items():
        it = inertial_of(e)
        if it is None:
            continue
        r, o = world(joints, name)
        c = mat_vec(r, it['com'])
        m_urdf += it['mass']
        mom_urdf = [mom_urdf[i] + it['mass'] * (o[i] + c[i]) for i in range(3)]
    items = [it for entry in LINKS.values() for it in entry['items']]
    m_items = sum(it['mass_kg'] for it in items)
    com_items = [sum(it['mass_kg'] * it['com_m'][i] for it in items) / m_items for i in range(3)]
    assert m_urdf == pytest.approx(m_items, abs=0.01)
    assert [x / m_urdf for x in mom_urdf] == pytest.approx(com_items, abs=0.002)
    check_base_link_complete_base_and_whole_robot_masses_are_kept_apart()


def test_base_group_holds_only_parts_of_the_base():
    boms = {it['bom'] for entry in LINKS.values() for it in entry['items']}
    assert not boms & NOT_BASE, sorted(boms & NOT_BASE)
    second = [it for entry in LINKS.values() for it in entry['items']
              if it['component'] == 'second_estop']
    assert not second, 'the second E-stop rides the adapter plate (lift carriage)'


def test_no_component_counted_on_two_links():
    seen = {}
    for group in EXPORT['groups'].values():
        for link, entry in group['links'].items():
            for it in entry['items']:
                key = (it['bom'], it['component'])
                assert key not in seen, '%s on %s and on %s' % (key, seen.get(key), link)
                seen[key] = link
    check_no_mass_counted_twice_in_the_expanded_urdf()


def test_base_mass_and_inertia_are_plausible():
    base = LINKS['base_link']
    assert 45.0 < base['mass_kg'] < 75.0
    i = base['inertia_kgm2']
    # a 0.78 x 0.58 m base: yaw inertia is the largest; each axis obeys the triangle inequality
    assert i['izz'] > max(i['ixx'], i['iyy'])
    assert i['ixx'] + i['iyy'] >= i['izz']
    assert i['ixx'] + i['izz'] >= i['iyy']
    assert i['iyy'] + i['izz'] >= i['ixx']
    for name in ('left_wheel_link', 'right_wheel_link'):
        assert 3.0 < LINKS[name]['mass_kg'] < 4.0, name
    check_hub_motor_on_the_wheel_link_is_marked_provisional()


# Owner decision 10 Nov 2026: three masses, three names. Never use one for another.
#   base_link mass: the inertial of base_link only (chassis, battery, electronics, rigid parts).
#   complete base mass: every base link with mass (base_link, wheel links with the hub motors,
#     castor links, lidar_link); this is what the expanded platform URDF weighs.
#   whole-robot mass: the complete base plus lift, lift carriage, arms, sensors and equipment
#     (openamr-upperbody-sw links); never in this URDF.
BASE_LINK_KG = LINKS['base_link']['mass_kg']
COMPLETE_BASE_KG = sum(entry['mass_kg'] for entry in LINKS.values())
WHOLE_ROBOT_KG = sum(entry['mass_kg'] for group in EXPORT['groups'].values()
                     for entry in group['links'].values())
# BOM lines that sit on a child link of base_link; base_link must not carry them as well
ON_CHILD_LINKS = {'DRV-001': ('left_wheel_link', 'right_wheel_link'),
                  'MEC-002': tuple('%s_%s_link' % (c, k) for c in ('fl', 'fr', 'bl', 'br')
                                   for k in ('caster', 'wheel')),
                  'NAV-001': ('lidar_link',)}


# The three checks below are called from the existing tests so that the CI test count stays 35
# (the description workflow asserts it; workflow files are outside this change).
def check_base_link_complete_base_and_whole_robot_masses_are_kept_apart():
    """Base_link < complete base < whole robot; each agrees with the export roll-up."""
    comparison = EXPORT['comparison']
    assert COMPLETE_BASE_KG == pytest.approx(comparison['roadmap_kg']['base'], abs=0.01)
    assert WHOLE_ROBOT_KG == pytest.approx(comparison['roadmap_total_kg'], abs=0.01)
    for height in EXPORT['totals'].values():
        assert WHOLE_ROBOT_KG == pytest.approx(height['mass_kg'], abs=0.01)
    assert BASE_LINK_KG < COMPLETE_BASE_KG < WHOLE_ROBOT_KG
    links, _ = expanded()
    urdf_total = sum(inertial_of(e)['mass'] for e in links.values() if inertial_of(e))
    # the platform URDF weighs the complete base, not base_link alone and not the whole robot
    assert urdf_total == pytest.approx(COMPLETE_BASE_KG, abs=1e-6)
    assert inertial_of(links['base_link'])['mass'] == pytest.approx(BASE_LINK_KG, abs=1e-6)


def check_no_mass_counted_twice_in_the_expanded_urdf():
    """One inertial per link, each link mass its own parts, no wheel part on base_link."""
    root = ET.fromstring(xacro.process_file(XACRO).toxml())
    for link in root.iter('link'):
        assert len(link.findall('inertial')) <= 1, link.get('name')
    # no mass outside a link (for example a gazebo extension or a second model)
    assert len(list(root.iter('inertial'))) == len(LINKS)
    for name, entry in LINKS.items():
        parts = sum(it['mass_kg'] for it in entry['items'])
        assert entry['mass_kg'] == pytest.approx(parts, abs=1e-6), name
    base_boms = {it['bom'] for it in LINKS['base_link']['items']}
    for bom, child_links in ON_CHILD_LINKS.items():
        assert bom not in base_boms, '%s on base_link and on %s' % (bom, child_links)
        for name in child_links:
            assert {it['bom'] for it in LINKS[name]['items']} == {bom}, name


def check_hub_motor_on_the_wheel_link_is_marked_provisional():
    """Owner decision 10 Nov 2026: whole hub motor on the wheel link, description use only."""
    for name in ('left_wheel_link', 'right_wheel_link'):
        (item,) = LINKS[name]['items']
        assert item['bom'] == 'DRV-001' and item['mass_kg'] == pytest.approx(3.45), name
    note = PIN['notes']['hub_motor_on_wheel_link']
    assert note['status'] == 'provisional'
    assert 'dynamics acceptance' in note['not_valid_for']
    assert note['todo'].startswith('TODO(')
