#!/usr/bin/env python3
"""
Offline config-identity hash, for declaring site_config.yaml's config_version.

interfaces#28's SiteConfigStatus.config_version is "SHA-256 over canonicalized
site-config file content" - NavigationStatusNode._site_config_status computes
this from the loaded site_config.yaml's own declared identity/zone fields
(site_id, map_id, config_id, required_zones, optional_zones - see
config_identity.py's docstring for why those fields and not the
map_version/config_version ones) and compares it against the declared
config_version. This tool computes the same hash offline, from a
site_config.yaml an operator has in hand, the same way
tools/compute_map_identity.py does for map_version.

It calls config_identity_hash() - the exact same function the live node
uses - so the two can never silently compute different things for the same
file.

Usage:
    python3 tools/compute_config_identity.py path/to/site_config.yaml
    python3 tools/compute_config_identity.py path/to/site_config.yaml \\
        --write path/to/site_config.yaml

Without --write, it only prints the hash. With --write, it replaces the
config_version: "..." line in the given file in place - a plain text
substitution of that one line, not a full YAML rewrite, so none of the
file's comments or formatting are disturbed. --write's target does not have
to be the same file the hash was computed from, the same way
compute_map_identity.py's doesn't, but in practice it always is.
"""

import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'openamrobot_nav2'))
from config_identity import config_identity_hash, parse_zone_list  # noqa: E402


def compute(site_config_path):
    with open(site_config_path, 'r') as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise SystemExit(f'error: {site_config_path} is not a mapping')

    required_ok, required_zones = parse_zone_list(doc.get('required_zones'))
    optional_ok, optional_zones = parse_zone_list(doc.get('optional_zones'))
    if not (required_ok and optional_ok):
        raise SystemExit(
            f'error: {site_config_path} has a missing, null or malformed '
            "required_zones/optional_zones - fix those before a config_version "
            'computed here could mean anything (an empty list must be '
            'written explicitly as []).')

    return config_identity_hash(
        site_id=doc.get('site_id', ''), map_id=doc.get('map_id', ''),
        config_id=doc.get('config_id', ''),
        required_zones=required_zones, optional_zones=optional_zones)


def write_config_version(site_config_path, config_hash):
    with open(site_config_path, 'r') as f:
        lines = f.readlines()
    found = False
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith('config_version:'):
            indent = line[:len(line) - len(stripped)]
            lines[i] = f'{indent}config_version: "{config_hash}"\n'
            found = True
            break
    if not found:
        raise SystemExit(
            f"error: no 'config_version:' line found in {site_config_path}, "
            "refusing to guess where to add one")
    with open(site_config_path, 'w') as f:
        f.writelines(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('site_config_yaml', help='path to the site_config.yaml to hash')
    parser.add_argument(
        '--write', metavar='SITE_CONFIG_YAML',
        help='also write the computed hash into this file\'s config_version field')
    args = parser.parse_args()

    config_hash = compute(args.site_config_yaml)
    print(config_hash)
    if args.write:
        write_config_version(args.write, config_hash)
        print(f'wrote config_version into {args.write}', file=sys.stderr)


if __name__ == '__main__':
    main()
