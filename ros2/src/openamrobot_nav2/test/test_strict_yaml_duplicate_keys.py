"""Strict YAML duplicate-key validation (owner request, 23 Nov 2026 platform convergence).

The duplicate default_nav_to_pose_bt_xml/default_nav_through_poses_bt_xml keys that showed up
twice in nav2_params.yaml during this convergence (once on the #164 rebase, once again on the
#164/integration-baseline1 merge) were only ever caught by a human reading the file: PyYAML's
own safe_load() silently keeps the LAST value of a repeated key and raises nothing, so the file
kept loading throughout, just with whichever side's value happened to be written last. This file
proves strict_yaml.load_strict() actually rejects a duplicate key instead of silently resolving
it, and that the shipped nav2_params.yaml has none.
"""
import os

from openamrobot_nav2.strict_yaml import DuplicateKeyError, load_strict, load_strict_file

import pytest

import yaml

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def test_a_clean_mapping_loads_the_same_as_plain_yaml():
    text = 'a: 1\nb:\n  c: 2\n  d: 3\n'
    assert load_strict(text) == yaml.safe_load(text) == {'a': 1, 'b': {'c': 2, 'd': 3}}


def test_an_intentionally_duplicated_top_level_key_is_rejected():
    # The exact shape of the real incident: the same top-level key written twice under the
    # same parent mapping, with two different values - plain yaml.safe_load() would silently
    # keep only "second" below and report nothing wrong.
    text = (
        'bt_navigator:\n  ros__parameters:\n'
        '    default_nav_to_pose_bt_xml: "first"\n'
        '    default_nav_to_pose_bt_xml: "second"\n')
    # Confirm the premise: PyYAML's own loader does NOT catch this - it is exactly the
    # silent-failure mode load_strict() exists to close.
    assert yaml.safe_load(text)['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == 'second'
    with pytest.raises(DuplicateKeyError, match='default_nav_to_pose_bt_xml'):
        load_strict(text)


def test_a_duplicated_nested_key_is_rejected_even_when_the_outer_keys_are_unique():
    text = (
        'controller_server:\n  ros__parameters:\n    StationApproach:\n'
        '      allow_backward: true\n      allow_backward: false\n')
    with pytest.raises(DuplicateKeyError, match='allow_backward'):
        load_strict(text)


def test_the_same_key_name_at_two_different_nesting_levels_is_not_a_duplicate():
    # "plugin" legitimately repeats once per controller block; only a repeat WITHIN the same
    # mapping is an error.
    text = 'controller_server:\n  ros__parameters:\n    A:\n      plugin: "x"\n    B:\n      plugin: "y"\n'
    loaded = load_strict(text)
    assert loaded['controller_server']['ros__parameters']['A']['plugin'] == 'x'
    assert loaded['controller_server']['ros__parameters']['B']['plugin'] == 'y'


def test_nav2_params_yaml_has_no_duplicate_keys():
    # The regression guard for today's actual incident: this must keep passing on every future
    # edit to the shipped file, including the next merge/rebase.
    loaded = load_strict_file(os.path.join(_CONFIG_DIR, 'nav2_params.yaml'))
    assert loaded['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == \
        'behavior_trees/navigate_to_pose.xml'


def test_every_shipped_config_yaml_has_no_duplicate_keys():
    for name in sorted(os.listdir(_CONFIG_DIR)):
        if name.endswith(('.yaml', '.yml')):
            load_strict_file(os.path.join(_CONFIG_DIR, name))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
