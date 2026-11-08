# SPDX-License-Identifier: MIT
"""
Check the 2.0 base_link mass against the roadmap model.

Owner decision 8 Nov 2026 (P-03 rev20.5 item 29): the roadmap model is the reference for
mass and placement, one source. config/roadmap_mass_export.yaml is a verbatim copy of
openamrobot-docs docs/assets/roadmap/model/mass_export.yaml; base_link takes its
groups.base entry, and only the parts fixed to the base are in it.
"""
import os
import xml.etree.ElementTree as ET

import pytest

import xacro

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
EXPORT_FILE = os.path.join(HERE, os.pardir, 'config', 'roadmap_mass_export.yaml')
with open(EXPORT_FILE) as f:
    EXPORT = yaml.safe_load(f)
BASE = EXPORT['groups']['base']['links']['base_link']
# BOM lines on the lift carriage, the static column or the head: never on base_link
NOT_BASE = {'MEC-003', 'MEC-004', 'MEC-012', 'UI-001', 'UI-002', 'LFT-001', 'LFT-003',
            'LFT-004', 'LFT-005', 'LFT-006', 'MEC-018', 'ARM-001', 'ARM-003', 'ARM-004',
            'ARM-011', 'AUX-001'}


def expanded_base_inertial():
    root = ET.fromstring(xacro.process_file(XACRO).toxml())
    link = [e for e in root.iter('link') if e.get('name') == 'base_link'][0]
    return link.find('inertial')


def test_base_link_takes_the_roadmap_base_group():
    inertial = expanded_base_inertial()
    mass = float(inertial.find('mass').get('value'))
    assert mass == pytest.approx(BASE['mass_kg'], abs=1e-6)
    xyz = [float(v) for v in inertial.find('origin').get('xyz').split()]
    assert xyz == pytest.approx(BASE['com_m'], abs=1e-6)
    inertia = inertial.find('inertia')
    got = [float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz')]
    want = [BASE['inertia_kgm2'][k] for k in ('ixx', 'iyy', 'izz')]
    assert got == pytest.approx(want, abs=1e-9)


def test_base_group_holds_only_parts_fixed_to_the_base():
    boms = {it['bom'] for it in BASE['items']}
    assert not boms & NOT_BASE, sorted(boms & NOT_BASE)
    second = [it for it in BASE['items'] if it['component'] == 'second_estop']
    assert not second, 'the second E-stop rides the adapter plate (lift carriage)'


def test_no_component_counted_on_two_links():
    seen = {}
    for group in EXPORT['groups'].values():
        for link, entry in group['links'].items():
            for it in entry['items']:
                key = (it['bom'], it['component'])
                assert key not in seen, '%s on %s and on %s' % (key, seen.get(key), link)
                seen[key] = link


def test_base_mass_and_inertia_are_plausible():
    assert 60.0 < BASE['mass_kg'] < 75.0
    i = BASE['inertia_kgm2']
    # a 0.78 x 0.58 m base: yaw inertia is the largest; each axis obeys the triangle inequality
    assert i['izz'] > max(i['ixx'], i['iyy'])
    assert i['ixx'] + i['iyy'] >= i['izz']
    assert i['ixx'] + i['izz'] >= i['iyy']
    assert i['iyy'] + i['izz'] >= i['ixx']
