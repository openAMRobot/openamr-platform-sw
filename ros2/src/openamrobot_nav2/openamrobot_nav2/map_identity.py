"""
Shared, dependency-free map-identity hashing (interfaces#27 / platform-sw#36).

One function, used by two callers that must never be allowed to drift apart:
  - NavigationStatusNode._map_identity_hash, hashing the live /map topic's
    OccupancyGrid.
  - tools/compute_map_identity.py, hashing a map.yaml + image pair the way
    nav2_map_server would have converted it, offline, to produce the
    site_config.yaml map_version an operator declares.

Neither caller does its own hashing; both call this. If they computed the
hash differently, a real match could show up as a false MISMATCH (or worse,
a false match) with no way to tell the two computations had quietly
diverged.
"""

import hashlib
import json


def map_identity_hash(*, resolution, width, height,
                       origin_x, origin_y, origin_yaw_z, origin_yaw_w,
                       cell_data) -> str:
    """
    SHA-256 over a map's resolution/origin/size and cell data, canonicalized
    so float formatting/whitespace can never change the hash but any real
    value or cell-data change always does.

    cell_data is any iterable of int8-range cell values (-1 unknown, 0-100
    occupied probability), in row-major order, matching
    nav_msgs/OccupancyGrid.data. Negative values are folded into a byte the
    same way the wire representation does, so this agrees with the live
    node's hash regardless of whether cell_data came in as an
    array.array('b', ...) (what a real OccupancyGrid.data is) or a plain list
    of Python ints (what the offline tool builds from an image).
    """
    canonical = json.dumps({
        'resolution': round(float(resolution), 6),
        'width': int(width),
        'height': int(height),
        'origin': {
            'x': round(float(origin_x), 6),
            'y': round(float(origin_y), 6),
            'yaw_z': round(float(origin_yaw_z), 6),
            'yaw_w': round(float(origin_yaw_w), 6),
        },
    }, sort_keys=True, separators=(',', ':')).encode('utf-8')
    digest = hashlib.sha256()
    digest.update(canonical)
    digest.update(bytes(bytearray(int(v) & 0xFF for v in cell_data)))
    return digest.hexdigest()
