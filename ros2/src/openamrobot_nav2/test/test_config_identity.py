"""
Pure tests for config_identity_hash.

Both the live node (NavigationStatusNode._site_config_status) and the
offline tool (tools/compute_config_identity.py) call this same function, so
a real match or mismatch can never come from the two computing it
differently - see config_identity.py's docstring.
"""

import unittest

from openamrobot_nav2.config_identity import (
    config_identity_hash, parse_identity_fields, parse_zone_list)

BASE_KWARGS = dict(
    site_id='sim', map_id='sim-floor-1', config_id='sim-floor-1-config',
    required_zones=['charging_dock'], optional_zones=['loading_bay'],
)


class TestConfigIdentityHash(unittest.TestCase):

    def test_identical_inputs_hash_identically(self):
        self.assertEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**BASE_KWARGS))

    def test_site_id_change_changes_the_hash(self):
        changed = dict(BASE_KWARGS, site_id='other-site')
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_map_id_change_changes_the_hash(self):
        changed = dict(BASE_KWARGS, map_id='other-map')
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_config_id_change_changes_the_hash(self):
        changed = dict(BASE_KWARGS, config_id='other-config')
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_adding_a_required_zone_changes_the_hash(self):
        changed = dict(
            BASE_KWARGS, required_zones=['charging_dock', 'second_dock'])
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_adding_an_optional_zone_changes_the_hash(self):
        changed = dict(BASE_KWARGS, optional_zones=['loading_bay', 'second_bay'])
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_removing_a_zone_changes_the_hash(self):
        changed = dict(BASE_KWARGS, required_zones=[])
        self.assertNotEqual(
            config_identity_hash(**BASE_KWARGS), config_identity_hash(**changed))

    def test_zone_declaration_order_does_not_change_the_hash(self):
        a = dict(BASE_KWARGS, required_zones=['a', 'b'], optional_zones=[])
        b = dict(BASE_KWARGS, required_zones=['b', 'a'], optional_zones=[])
        self.assertEqual(config_identity_hash(**a), config_identity_hash(**b))

    def test_zones_accept_any_string_iterable_not_just_lists(self):
        as_list = dict(BASE_KWARGS, required_zones=['a', 'b'])
        as_frozenset = dict(BASE_KWARGS, required_zones=frozenset(['a', 'b']))
        as_tuple = dict(BASE_KWARGS, required_zones=('b', 'a'))
        self.assertEqual(
            config_identity_hash(**as_list), config_identity_hash(**as_frozenset))
        self.assertEqual(
            config_identity_hash(**as_list), config_identity_hash(**as_tuple))

    def test_empty_zone_lists_hash_differently_from_nonempty_ones(self):
        empty = dict(BASE_KWARGS, required_zones=[], optional_zones=[])
        self.assertNotEqual(
            config_identity_hash(**empty), config_identity_hash(**BASE_KWARGS))

    def test_two_sites_with_identical_declared_content_hash_the_same(self):
        # Proves this is a real content hash, not an opaque per-call token:
        # two independently-built kwargs dicts with the same values agree.
        a = dict(
            site_id='sim', map_id='sim-floor-1', config_id='sim-floor-1-config',
            required_zones=['charging_dock'], optional_zones=['loading_bay'])
        b = {
            'site_id': 'sim', 'map_id': 'sim-floor-1', 'config_id': 'sim-floor-1-config',
            'required_zones': ['charging_dock'], 'optional_zones': ['loading_bay'],
        }
        self.assertEqual(config_identity_hash(**a), config_identity_hash(**b))


class TestParseIdentityFields(unittest.TestCase):
    """
    Mohamed Sayed's platform-sw#73 review (second pass): an absent, null,
    empty or wrong-type site_id/map_id/config_id must not reach
    config_identity_hash at all - that is the identity-field counterpart of
    TestParseZoneList above, which already covers required_zones/
    optional_zones.
    """

    def test_real_string_identities_are_valid_and_passed_through(self):
        ok, site_id, map_id, config_id = parse_identity_fields(
            'sim', 'sim-floor-1', 'sim-floor-1-config')
        self.assertTrue(ok)
        expected = ('sim', 'sim-floor-1', 'sim-floor-1-config')
        self.assertEqual((site_id, map_id, config_id), expected)

    def test_all_three_absent_is_invalid(self):
        ok, site_id, map_id, config_id = parse_identity_fields(None, None, None)
        self.assertFalse(ok)
        self.assertIsNone(site_id)
        self.assertIsNone(map_id)
        self.assertIsNone(config_id)

    def test_one_absent_field_is_enough_to_invalidate_all_three(self):
        ok, *_ = parse_identity_fields('sim', None, 'sim-floor-1-config')
        self.assertFalse(ok)

    def test_empty_string_is_invalid_not_a_real_identity(self):
        # a profile that declares site_id: "" must not be treated as a
        # confirmed, if blank, identity - empty is absent in spirit too.
        ok, *_ = parse_identity_fields('', 'sim-floor-1', 'sim-floor-1-config')
        self.assertFalse(ok)

    def test_an_integer_identity_is_invalid_not_silently_stringified(self):
        # previously config_identity_hash's str(site_id) would have hashed
        # this cleanly as "7" - the point of this gate is that it never
        # reaches that call at all.
        ok, *_ = parse_identity_fields(7, 'sim-floor-1', 'sim-floor-1-config')
        self.assertFalse(ok)

    def test_a_none_identity_is_invalid_not_silently_stringified(self):
        # str(None) == 'None' would also have hashed cleanly before this
        # gate existed.
        ok, *_ = parse_identity_fields('sim', None, 'sim-floor-1-config')
        self.assertFalse(ok)

    def test_a_boolean_identity_is_invalid(self):
        # bool is a str subtype-free int subtype in Python, not a string -
        # worth its own case since isinstance(True, int) is True but
        # isinstance(True, str) is False, which is exactly what this checks.
        ok, *_ = parse_identity_fields('sim', True, 'sim-floor-1-config')
        self.assertFalse(ok)


class TestParseZoneList(unittest.TestCase):
    """
    Mohamed Sayed's platform-sw#73 review: a missing zone declaration must
    not be read the same as an explicitly confirmed empty one, and a
    malformed entry must not crash (the old `set(value)` construction raised
    TypeError: unhashable type: 'dict' on a list of dicts).
    """

    def test_explicit_empty_list_is_valid_and_empty(self):
        ok, zones = parse_zone_list([])
        self.assertTrue(ok)
        self.assertEqual(zones, frozenset())

    def test_a_real_zone_list_is_valid(self):
        ok, zones = parse_zone_list(['charging_dock', 'loading_bay'])
        self.assertTrue(ok)
        self.assertEqual(zones, frozenset(['charging_dock', 'loading_bay']))

    def test_duplicate_entries_collapse_but_stay_valid(self):
        ok, zones = parse_zone_list(['a', 'a'])
        self.assertTrue(ok)
        self.assertEqual(zones, frozenset(['a']))

    def test_absent_field_is_invalid_not_empty(self):
        ok, zones = parse_zone_list(None)
        self.assertFalse(ok)
        self.assertIsNone(zones)

    def test_explicit_null_is_invalid_the_same_as_absent(self):
        # YAML's `required_zones:` with nothing after the colon parses to
        # None, same as the key being absent entirely - both must be
        # treated as "not declared", not "declared empty".
        ok, zones = parse_zone_list(None)
        self.assertFalse(ok)

    def test_a_string_instead_of_a_list_is_invalid(self):
        ok, zones = parse_zone_list('charging_dock')
        self.assertFalse(ok)
        self.assertIsNone(zones)

    def test_a_list_of_non_strings_is_invalid_not_a_crash(self):
        # this is exactly the input that used to raise
        # TypeError: unhashable type: 'dict' before this function existed
        ok, zones = parse_zone_list([{'id': 'charging_dock'}])
        self.assertFalse(ok)
        self.assertIsNone(zones)

    def test_a_mixed_list_is_invalid(self):
        ok, zones = parse_zone_list(['charging_dock', 42])
        self.assertFalse(ok)
        self.assertIsNone(zones)

    def test_an_integer_is_invalid(self):
        ok, zones = parse_zone_list(0)
        self.assertFalse(ok)
        self.assertIsNone(zones)


if __name__ == '__main__':
    unittest.main(verbosity=2)
