#!/usr/bin/env python3
"""
Offline map-identity hash, for declaring site_config.yaml's map_version.

interfaces#27 / platform-sw#36's OPEN ITEM: navigation_status_node hashes the
live /map topic's OccupancyGrid (NavigationStatusNode._map_identity_hash) to
decide whether the robot is actually running the map an operator declared in
site_config.yaml. That only works once something can compute the *same* hash
offline, from the map.yaml + image pair an operator actually has in hand,
before the robot ever loads it. This is that tool.

It replicates nav2_map_server's own image-to-OccupancyGrid conversion
(mode/negate/occupied_thresh/free_thresh), then calls the exact same
map_identity_hash() function the live node uses, so the two can never
silently compute different things for the same map.

Usage:
    python3 tools/compute_map_identity.py path/to/map.yaml
    python3 tools/compute_map_identity.py path/to/map.yaml \\
        --write path/to/site_config.yaml

Without --write, it only prints the hash. With --write, it replaces the
map_version: "..." line in the given site_config.yaml in place - a plain
text substitution of that one line, not a full YAML rewrite, so none of the
file's comments or formatting are disturbed.
"""

import argparse
import math
import os
import sys

import yaml

try:
    from PIL import Image
except ImportError:  # pragma: no cover - environment-dependent
    print(
        "error: this tool needs Pillow (pip install --break-system-packages Pillow)",
        file=sys.stderr)
    raise

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'openamrobot_nav2'))
from map_identity import map_identity_hash  # noqa: E402


def _yaw_to_quaternion_zw(yaw):
    """Return (z, w) for a pure yaw rotation, same convention tf2 uses."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_map_yaml(path):
    with open(path, 'r') as f:
        doc = yaml.safe_load(f)
    image_path = doc['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(path), image_path)
    return {
        'image_path': image_path,
        'resolution': float(doc['resolution']),
        'origin': [float(v) for v in doc['origin']],
        'negate': bool(doc.get('negate', 0)),
        'occupied_thresh': float(doc.get('occupied_thresh', 0.65)),
        'free_thresh': float(doc.get('free_thresh', 0.196)),
        'mode': doc.get('mode', 'trinary'),
    }


def image_to_occupancy_data(image_path, negate, occupied_thresh, free_thresh, mode):
    """
    Return (width, height, data) the way nav2_map_server's trinary/scale/raw
    modes would, for the same inputs:
      - the image is read top-to-bottom, then flipped, since OccupancyGrid
        row 0 is the map's bottom row and image row 0 is the image's top;
      - a pixel's occupancy probability is its averaged, normalized shade,
        inverted when negate is set (dark = occupied by default);
      - trinary collapses that probability to -1/0/100 at the two
        thresholds; scale keeps 0-100 directly; raw passes the pixel value
        through unchanged (rarely used, included for completeness).
    """
    img = Image.open(image_path).convert('L')
    width, height = img.size
    pixels = list(img.getdata())  # row-major, top row first

    data = [0] * (width * height)
    for row in range(height):
        src_row = height - 1 - row  # flip: OccupancyGrid row 0 = image bottom row
        for col in range(width):
            shade = pixels[src_row * width + col] / 255.0
            occ = (1.0 - shade) if not negate else shade
            if mode == 'raw':
                cell = int(round(occ * 100))
            elif occ > occupied_thresh:
                cell = 100
            elif occ < free_thresh:
                cell = 0
            else:
                cell = -1 if mode == 'trinary' else int(round(occ * 100))
            data[row * width + col] = cell
    return width, height, data


def compute(map_yaml_path):
    cfg = load_map_yaml(map_yaml_path)
    width, height, data = image_to_occupancy_data(
        cfg['image_path'], cfg['negate'], cfg['occupied_thresh'],
        cfg['free_thresh'], cfg['mode'])
    origin_x, origin_y, origin_yaw = cfg['origin']
    yaw_z, yaw_w = _yaw_to_quaternion_zw(origin_yaw)
    return map_identity_hash(
        resolution=cfg['resolution'], width=width, height=height,
        origin_x=origin_x, origin_y=origin_y,
        origin_yaw_z=yaw_z, origin_yaw_w=yaw_w,
        cell_data=data)


def write_map_version(site_config_path, map_hash):
    with open(site_config_path, 'r') as f:
        lines = f.readlines()
    found = False
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith('map_version:'):
            indent = line[:len(line) - len(stripped)]
            lines[i] = f'{indent}map_version: "{map_hash}"\n'
            found = True
            break
    if not found:
        raise SystemExit(
            f"error: no 'map_version:' line found in {site_config_path}, "
            "refusing to guess where to add one")
    with open(site_config_path, 'w') as f:
        f.writelines(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml', help='path to the map.yaml to hash')
    parser.add_argument(
        '--write', metavar='SITE_CONFIG_YAML',
        help='also write the computed hash into this site_config.yaml\'s map_version field')
    args = parser.parse_args()

    map_hash = compute(args.map_yaml)
    print(map_hash)
    if args.write:
        write_map_version(args.write, map_hash)
        print(f'wrote map_version into {args.write}', file=sys.stderr)


if __name__ == '__main__':
    main()
