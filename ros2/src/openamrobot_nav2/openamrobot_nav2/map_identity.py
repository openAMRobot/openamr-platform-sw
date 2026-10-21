"""
Shared, dependency-free map-identity hashing (interfaces#27/#28, platform-sw#36).

One function, used by two callers that must never be allowed to drift apart:
  - NavigationStatusNode._compute_map_hash, reading the map.yaml + image file
    this node is configured to run against.
  - tools/compute_map_identity.py, reading the same kind of map.yaml + image
    pair offline to produce the site_config.yaml map_version an operator
    declares.

Neither caller does its own hashing; both call this, and both read the map
file the same way through load_map_identity_source. If they computed the
hash differently, a real match could show up as a false MISMATCH (or worse,
a false match) with no way to tell the two computations had quietly
diverged.

interfaces#28's SiteConfigStatus.map_version is defined as "SHA-256 over
canonicalized map YAML fields + raw image bytes" - deliberately the raw
bytes of the image file, not any decoded/thresholded interpretation of them.
An earlier version of this module hashed a trinary-converted OccupancyGrid
instead, which meant reproducing nav2_map_server's own pixel-conversion
semantics (mode/negate/occupied_thresh/free_thresh) to predict what the live
node would compute - and that reproduction disagreed with the real
nav2_map_server on several boundary cases (Mohamed Sayed's platform-sw#73
review: scale-mode normalization, raw-mode threshold direction, alpha
handling, equality-boundary handling). Hashing the declared YAML fields and
the image file's raw bytes directly needs no pixel decoding at all, so there
is nothing left to disagree with Nav2 about: both this node and the offline
tool read the identical bytes off disk and hash them the identical way.
"""

import hashlib
import json
import os

import yaml


def map_identity_hash(*, resolution, origin_x, origin_y, origin_yaw,
                       negate, occupied_thresh, free_thresh, mode,
                       image_bytes) -> str:
    """
    SHA-256 over a map's canonicalized YAML fields plus its image file's raw
    bytes, per interfaces#28's SiteConfigStatus.map_version definition.

    Canonicalizing the YAML fields (rounding floats, normalizing types)
    means float formatting/whitespace in the YAML file can never change the
    hash, but any real value change always does. image_bytes is hashed
    exactly as read from disk - never decoded - so this does not need to
    know, or agree with Nav2 about, how to interpret pixels as occupancy.
    """
    canonical = json.dumps({
        'resolution': round(float(resolution), 6),
        'origin': {
            'x': round(float(origin_x), 6),
            'y': round(float(origin_y), 6),
            'yaw': round(float(origin_yaw), 6),
        },
        'negate': bool(negate),
        'occupied_thresh': round(float(occupied_thresh), 6),
        'free_thresh': round(float(free_thresh), 6),
        'mode': str(mode),
    }, sort_keys=True, separators=(',', ':')).encode('utf-8')
    digest = hashlib.sha256()
    digest.update(canonical)
    digest.update(bytes(image_bytes))
    return digest.hexdigest()


def load_map_identity_source(map_yaml_path: str) -> dict:
    """
    Read a map.yaml + its referenced image file and return the fields
    map_identity_hash needs, as **kwargs-ready dict.

    Reads exactly the inputs nav2_map_server itself reads for this file -
    the declared YAML fields, and the image file's bytes, untouched. Raises
    (OSError, yaml.YAMLError, KeyError, ValueError) on anything wrong with
    the file; callers decide how to handle that (the live node treats it the
    same as a missing/blank declared map_version - a correctly-reported
    mismatch, not a crash).
    """
    with open(map_yaml_path, 'r') as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f'{map_yaml_path} is not a mapping')
    image_rel = doc['image']
    image_path = image_rel if os.path.isabs(image_rel) else os.path.join(
        os.path.dirname(map_yaml_path), image_rel)
    origin = doc['origin']
    with open(image_path, 'rb') as f:
        image_bytes = f.read()
    return {
        'resolution': float(doc['resolution']),
        'origin_x': float(origin[0]),
        'origin_y': float(origin[1]),
        'origin_yaw': float(origin[2]),
        'negate': bool(doc.get('negate', 0)),
        'occupied_thresh': float(doc.get('occupied_thresh', 0.65)),
        'free_thresh': float(doc.get('free_thresh', 0.196)),
        'mode': str(doc.get('mode', 'trinary')),
        'image_bytes': image_bytes,
    }
