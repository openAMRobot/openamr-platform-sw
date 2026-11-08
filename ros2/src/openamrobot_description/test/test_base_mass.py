# SPDX-License-Identifier: MIT
"""The 2.0 base_link carries the BOM Issue 7.14 base mass, not the chassis shell alone (P-03 rev20.4 item 28)."""
import os
import xml.etree.ElementTree as ET

import pytest

XACRO = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "urdf", "robo_urdf.urdf.xacro")


def base_inertial():
    root = ET.parse(XACRO).getroot()
    link = [e for e in root.iter("link") if e.get("name") == "base_link"][0]
    return link.find("inertial")


def test_base_link_mass_is_the_bom_base_group():
    inertial = base_inertial()
    assert float(inertial.find("mass").get("value")) == pytest.approx(69.737, abs=0.01)


def test_base_link_com_and_inertia_from_the_roadmap_mass_export():
    inertial = base_inertial()
    xyz = [float(v) for v in inertial.find("origin").get("xyz").split()]
    assert xyz == pytest.approx([-0.055, -0.0038, 0.111], abs=1e-4)
    inertia = inertial.find("inertia")
    assert [float(inertia.get(k)) for k in ("ixx", "iyy", "izz")] == pytest.approx([2.18018, 3.46861, 1.61334], rel=1e-4)


def test_total_base_mass_is_within_the_gross_mass_limit():
    root = ET.parse(XACRO).getroot()
    total = sum(float(e.find("inertial/mass").get("value")) for e in root.iter("link") if e.find("inertial/mass") is not None)
    assert 69.0 < total < 120.0
