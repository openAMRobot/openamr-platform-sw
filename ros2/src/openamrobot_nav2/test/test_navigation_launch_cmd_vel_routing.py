#!/usr/bin/env python3
"""
Regression test for #35 (GATE-A): behavior_server must stay in the chain.

Statically inspects launch/navigation_launch.py's source via ast, instead of
executing the launch file - no ROS graph needed, same style as this
package's test_flake8/test_pep257 checks. Fails if either the plain Node()
or the ComposableNode() definition for behavior_server loses the
('cmd_vel', 'cmd_vel_nav') remap: without it, spin/backup/drive_on_heading/
assisted_teleop publish straight to the final /cmd_vel again, bypassing
velocity_smoother and collision_monitor entirely.
"""
import ast
import os

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'navigation_launch.py')

REQUIRED_REMAP = ('cmd_vel', 'cmd_vel_nav')
TARGET_NODE_NAME = 'behavior_server'


def _string_value(node):
    """Return the literal string value of an ast node, or None."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _remap_pairs(node):
    """Yield every (old, new) string pair out of a remappings=... AST node."""
    if not isinstance(node, (ast.List, ast.Tuple)):
        return
    for element in node.elts:
        if isinstance(element, ast.Tuple) and len(element.elts) == 2:
            old = _string_value(element.elts[0])
            new = _string_value(element.elts[1])
            if old is not None and new is not None:
                yield (old, new)


def _target_node_calls():
    """Yield each Node()/ComposableNode() call naming TARGET_NODE_NAME."""
    with open(LAUNCH_FILE) as f:
        tree = ast.parse(f.read(), filename=LAUNCH_FILE)
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func_name = getattr(call.func, 'id', None) or getattr(call.func, 'attr', None)
        if func_name not in ('Node', 'ComposableNode'):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        if _string_value(kwargs.get('name')) == TARGET_NODE_NAME:
            yield call, kwargs


def test_behavior_server_definitions_exist():
    """Sanity check: the launch file still defines behavior_server at all."""
    calls = list(_target_node_calls())
    assert len(calls) >= 2, (
        f'Expected at least 2 Node()/ComposableNode() definitions named '
        f'"{TARGET_NODE_NAME}" (the non-composed Node and the ComposableNode '
        f'variant) in navigation_launch.py, found {len(calls)}. If one was '
        f'intentionally removed, update this test to match.')


def test_behavior_server_remaps_cmd_vel_to_cmd_vel_nav():
    """
    Every behavior_server definition must remap cmd_vel to cmd_vel_nav.

    #35: without this remap, spin/backup/drive_on_heading/assisted_teleop
    publish straight to the final /cmd_vel, bypassing velocity_smoother and
    collision_monitor entirely.
    """
    calls = list(_target_node_calls())
    assert len(calls) >= 1, 'No behavior_server definition found to check.'

    for call, kwargs in calls:
        remappings_node = kwargs.get('remappings')
        assert remappings_node is not None, (
            f'behavior_server at line {call.lineno} has no remappings= at all.')

        # remappings is built as `remappings + [(...)]` - walk every list/
        # tuple literal anywhere inside that expression, not just the top
        # level, so this survives reasonable reformatting.
        pairs = set()
        for sub in ast.walk(remappings_node):
            pairs.update(_remap_pairs(sub))

        assert REQUIRED_REMAP in pairs, (
            f'behavior_server at line {call.lineno} is missing the '
            f'{REQUIRED_REMAP} remap - it will publish straight to the '
            f'final /cmd_vel again, bypassing velocity_smoother and '
            f'collision_monitor (#35).')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
