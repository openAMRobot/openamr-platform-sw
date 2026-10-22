#!/usr/bin/env python3
"""
Offline map-identity hash, for declaring site_config.yaml's map_version.

interfaces#28's SiteConfigStatus.map_version is "SHA-256 over canonicalized
map YAML fields + raw image bytes" - NavigationStatusNode._compute_map_hash
reads a map.yaml + image file off disk and hashes exactly that, to decide
whether the robot is actually running the map an operator declared in
site_config.yaml. This is the same computation, offline, from the map.yaml +
image pair an operator has in hand before the robot ever loads it.

It calls load_map_identity_source() to read the file the same way the live
node does, then map_identity_hash() - the exact same two functions the live
node uses - so the two can never silently compute different things for the
same map. Earlier, this tool re-decoded the image into occupancy values
itself (replicating nav2_map_server's mode/negate/occupied_thresh/
free_thresh conversion) to predict what the live node would compute; that
replication disagreed with the real nav2_map_server on several boundary
cases (Mohamed Sayed's platform-sw#73 review). Hashing the raw bytes
directly - what the contract actually specifies - removes the need to
replicate that conversion at all, and with it, the entire class of
disagreement: both this tool and the live node read the identical bytes off
disk through the identical function.

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
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'openamrobot_nav2'))
from map_identity import load_map_identity_source, map_identity_hash  # noqa: E402


def compute(map_yaml_path):
    return map_identity_hash(**load_map_identity_source(map_yaml_path))


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
