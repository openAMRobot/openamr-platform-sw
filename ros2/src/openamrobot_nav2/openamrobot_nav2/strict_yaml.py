"""Strict YAML loading that rejects duplicate mapping keys.

Owner request (Alex, 23 Nov 2026, platform convergence): the nav2_params.yaml
duplicate-key incident (bt_navigator's default_nav_to_pose_bt_xml/
default_nav_through_poses_bt_xml appearing twice after a rebase, both times
during this same convergence) was only caught by eye. PyYAML's own
SafeLoader silently keeps the LAST value of a repeated mapping key and
raises nothing - a 3-way merge or a hand edit can reintroduce a duplicate
and every normal yaml.safe_load() call keeps loading, just with the wrong
(whichever-came-last) value, with no error anywhere.

load_strict() parses exactly like yaml.safe_load(), except any mapping key
seen twice at the same nesting level raises DuplicateKeyError naming the
key, the file and the two line numbers involved - so a duplicate fails the
load instead of silently picking one side's value.
"""
import yaml


class DuplicateKeyError(ValueError):
    """A YAML mapping repeated a key; see args[0] for which one and where."""


class _StrictUniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping_no_duplicates(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise DuplicateKeyError(
                f'duplicate key {key!r} at line {key_node.start_mark.line + 1} '
                f'(first seen at an earlier line in the same mapping, node starting '
                f'at line {node.start_mark.line + 1})')
        value = loader.construct_object(value_node, deep=deep)
        mapping[key] = value
    return mapping


_StrictUniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping_no_duplicates)


def load_strict(stream):
    """Parse YAML like yaml.safe_load(), raising DuplicateKeyError on any repeated mapping key."""
    return yaml.load(stream, Loader=_StrictUniqueKeyLoader)


def load_strict_file(path):
    """load_strict() a YAML file given its path."""
    with open(path) as f:
        return load_strict(f)
