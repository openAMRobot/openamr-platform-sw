"""
Shared, dependency-free site-config-identity hashing (interfaces#27/#28,
platform-sw#36).

Same pattern as map_identity.py: one function, used by both
NavigationStatusNode._site_config_status (hashing the loaded site_config.yaml's
declared identity/zone content to check it against the declared
config_version) and tools/compute_config_identity.py (which lets an operator
regenerate the matching config_version for a site_config.yaml by hand,
offline, the same way tools/compute_map_identity.py does for map_version).
Neither caller hashes this on its own.

interfaces#28's SiteConfigStatus.config_version is defined as "SHA-256 over
canonicalized site-config file content". The only site-config content that
exists in this codebase so far is the identity and zone fields in
site_config.yaml itself (platform-sw#36's OPEN ITEM: there is no separate
fleet/approval-manifest source yet - see navigation_status_node.py's
_site_config_status docstring) - so that is what this hashes: everything the
file declares about what this deployment should be running, except the
map_version/config_version fields themselves (hashing those in would make
the check compare the file to its own declared answer).
"""

import hashlib
import json


def parse_zone_list(value):
    """
    Validate a required_zones/optional_zones field loaded from
    site_config.yaml.

    Returns (ok, zones). ok is False when the field is absent, null, not a
    list, or contains anything but plain strings - the caller must then
    treat the whole site config as schema-invalid rather than guessing an
    empty or partial zone set from it (Mohamed Sayed's platform-sw#73
    review: a missing declaration must not be read the same as an
    explicitly confirmed empty one). zones is None in that case.

    An explicit empty list is valid and returns (True, frozenset()) - "this
    site declares no zones of this kind" is a real, confirmed answer, not a
    missing one.

    Never raises: a malformed entry (e.g. a list of dicts instead of
    strings) is reported through ok=False, not a crash - the old
    `set(value)` construction raised TypeError: unhashable type: 'dict' on
    exactly this input, which this function exists to prevent.

    Pure and dependency-free (no ROS imports), deliberately: both the live
    node (which has rclpy available) and tools/compute_config_identity.py
    (which must run standalone, without ROS installed or running) need this
    exact same validation, and must never be allowed to disagree about it.
    """
    if not isinstance(value, list):
        return False, None
    if not all(isinstance(zone, str) for zone in value):
        return False, None
    return True, frozenset(value)


def parse_identity_fields(site_id, map_id, config_id):
    """
    Validate the three identity fields declared in site_config.yaml before
    they are hashed or assigned to SiteConfigStatus's output fields.

    Returns (ok, site_id, map_id, config_id). ok is False when any of the
    three is absent, None, empty, or not a string - previously an absent
    field defaulted to '' at the call site and hashed cleanly, so a profile
    missing site_id/map_id/config_id entirely could still compute a real
    hash (of three empty strings) and read as MATCHED/FULL against a
    declared config_version of that same hash (Mohamed Sayed's
    platform-sw#73 review). A numeric or null id was silently stringified
    for the hash and then assigned unchanged to the message's string
    output fields, carrying the same unvalidated value through either way.
    ok is True only once all three are confirmed non-empty strings; the
    caller must then treat an invalid result the same as an invalid zone
    schema - SITE_CONFIG_UNKNOWN, never a hash comparison against
    unvalidated input.

    Never raises, same reason as parse_zone_list: a malformed profile must
    report invalid, not crash the status producer. Pure and
    dependency-free, for the same reason parse_zone_list is - the live
    node and tools/compute_config_identity.py must never be allowed to
    disagree about what counts as a valid identity field.
    """
    values = (site_id, map_id, config_id)
    if any(not isinstance(value, str) or not value for value in values):
        return False, None, None, None
    return True, site_id, map_id, config_id


def config_identity_hash(*, site_id, map_id, config_id,
                          required_zones, optional_zones) -> str:
    """
    SHA-256 over a site config's declared identity and zone content.

    required_zones/optional_zones may be any iterable of strings, in any
    order - sorted here so declaration order in the YAML file never changes
    the hash, the same way map_identity_hash rounds floats so formatting
    never changes its hash.
    """
    canonical = json.dumps({
        'site_id': str(site_id),
        'map_id': str(map_id),
        'config_id': str(config_id),
        'required_zones': sorted(str(z) for z in required_zones),
        'optional_zones': sorted(str(z) for z in optional_zones),
    }, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(canonical).hexdigest()
