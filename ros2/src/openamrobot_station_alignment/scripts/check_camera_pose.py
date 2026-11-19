#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check where `gz sdf -p` puts the test-only base camera sensor: base_footprint + (0.387, 0, 0.144), pitch -0.1745.

Usage: check_camera_pose.py <expanded.urdf>. Exit 1 when the converted pose differs by more than 1 mm or 1 mrad, or
when the sensor is missing. This checks the URDF to SDF conversion only, not the rendered image.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET

EXPECTED = (0.387, 0.0, 0.098 + 0.046, 0.0, -0.1745, 0.0)

sdf = subprocess.run(["gz", "sdf", "-p", sys.argv[1]], check=True, capture_output=True, text=True).stdout
root = ET.fromstring(sdf)
for link in root.iter("link"):
    for sensor in link.findall("sensor"):
        if sensor.get("name") != "station_alignment_base_camera":
            continue
        pose = sensor.find("pose")
        values = tuple(float(v) for v in (pose.text or "").split()) if pose is not None else ()
        frame = pose.get("relative_to") if pose is not None else None
        print(f"sensor on link {link.get('name')}, pose {values} relative_to {frame}")
        if link.get("name") != "base_footprint" or frame not in (None, "base_footprint") or len(values) != 6:
            sys.exit(f"unexpected link or frame: {link.get('name')} {frame}")
        if any(abs(a - b) > 1e-3 for a, b in zip(values, EXPECTED)):
            sys.exit(f"pose {values} differs from {EXPECTED}")
        print("camera pose OK")
        sys.exit(0)
sys.exit("station_alignment_base_camera not found in the converted SDF")
