#!/usr/bin/env python3
"""
Regression test for #35/#90 (GATE-A): velocity_smoother must read cmd_vel_gated.

Statically inspects launch/navigation_launch.py's source via ast, instead of
executing the launch file - no ROS graph needed, same style as this
package's test_navigation_launch_cmd_vel_routing.py (behavior_server) and
test_flake8/test_pep257 checks.

Platform-sw#90, Alex's review: a nonzero Twist from some OTHER publisher on
cmd_vel_stop would clear twist_mux's own arbitration at equal priority and
reach velocity_smoother unmodified if it read cmd_vel_mux directly - the fix
is safety_stop_gate_node.py's post-mux gate, published on cmd_vel_gated
(see that node's own docstring). This test fails if either the plain Node()
or the ComposableNode() definition for velocity_smoother ever points back at
cmd_vel_mux (or anything else) instead of cmd_vel_gated, which would silently
re-open the gap this review closed without anything else here catching it -
the gate node existing and working correctly says nothing about what
velocity_smoother is actually wired to read.
"""
import ast
import os

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'navigation_launch.py')

REQUIRED_REMAP = ('cmd_vel', 'cmd_vel_gated')
FORBIDDEN_REMAP = ('cmd_vel', 'cmd_vel_mux')
TARGET_NODE_NAME = 'velocity_smoother'


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


def test_velocity_smoother_definitions_exist():
    """Sanity check: the launch file still defines velocity_smoother at all."""
    calls = list(_target_node_calls())
    assert len(calls) >= 2, (
        f'Expected at least 2 Node()/ComposableNode() definitions named '
        f'"{TARGET_NODE_NAME}" (the non-composed Node and the ComposableNode '
        f'variant) in navigation_launch.py, found {len(calls)}. If one was '
        f'intentionally removed, update this test to match.')


def test_velocity_smoother_remaps_cmd_vel_to_cmd_vel_gated_not_cmd_vel_mux():
    """
    Every velocity_smoother definition must remap cmd_vel to cmd_vel_gated.

    #90: reading cmd_vel_mux directly bypasses safety_stop_gate_node's
    post-mux lock check, re-opening the equal-priority nonzero-stop gap
    Alex's review closed.
    """
    calls = list(_target_node_calls())
    assert len(calls) >= 1, 'No velocity_smoother definition found to check.'

    for call, kwargs in calls:
        remappings_node = kwargs.get('remappings')
        assert remappings_node is not None, (
            f'velocity_smoother at line {call.lineno} has no remappings= at all.')

        # remappings is built as `remappings + [(...)]` - walk every list/
        # tuple literal anywhere inside that expression, not just the top
        # level, so this survives reasonable reformatting.
        pairs = set()
        for sub in ast.walk(remappings_node):
            pairs.update(_remap_pairs(sub))

        assert REQUIRED_REMAP in pairs, (
            f'velocity_smoother at line {call.lineno} is missing the '
            f'{REQUIRED_REMAP} remap - it will read straight from twist_mux '
            f'again, bypassing safety_stop_gate_node\'s post-mux lock check (#90).')
        assert FORBIDDEN_REMAP not in pairs, (
            f'velocity_smoother at line {call.lineno} still remaps '
            f'{FORBIDDEN_REMAP} - this re-opens the equal-priority nonzero-stop '
            f'gap Alex\'s review (#90) closed; it must read cmd_vel_gated only.')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
