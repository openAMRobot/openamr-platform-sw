#!/usr/bin/env python3
"""
Regression test: navigation_launch.py must never start waypoint_follower.

Alex's finding on #90 at 2e0b8f3: follow_waypoints treats a stop-gate-
cancelled child NavigateToPose as a failed waypoint, and with
stop_on_failure: false (nav2_params.yaml) it just continues to the next
waypoint on its own - a new child goal with no fresh operator request,
breaking D-14 ("Resume clears the inhibit only, never restarts motion").
Decision (Alex, target 13 Nov): disable waypoint_follower in the supported
profile entirely, so no /follow_waypoints server ever exists and an
external client cannot submit that goal in the first place.

Statically inspects launch/navigation_launch.py's source via ast, same
style as test_navigation_launch_cmd_vel_routing.py - no ROS graph needed.
See test_navigation_launch_waypoint_follower_server_absent.py for the live
proof that no /follow_waypoints action server exists in the real launch.
"""
import ast
import os

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'navigation_launch.py')


def _string_value(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _tree():
    with open(LAUNCH_FILE) as f:
        return ast.parse(f.read(), filename=LAUNCH_FILE)


def test_no_node_or_composable_node_is_named_waypoint_follower():
    tree = _tree()
    matches = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func_name = getattr(call.func, 'id', None) or getattr(call.func, 'attr', None)
        if func_name not in ('Node', 'ComposableNode'):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        if _string_value(kwargs.get('name')) == 'waypoint_follower':
            matches.append(call.lineno)
    assert matches == [], (
        f'navigation_launch.py must not start waypoint_follower - found '
        f'Node()/ComposableNode() definition(s) at line(s) {matches}. '
        "Alex's D-14 finding on #90: follow_waypoints auto-continues to "
        'the next waypoint after a gate-cancelled child goal, with no '
        'fresh operator request.')


def test_lifecycle_nodes_list_excludes_waypoint_follower():
    tree = _tree()
    for assign in ast.walk(tree):
        if not isinstance(assign, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == 'lifecycle_nodes' for t in assign.targets):
            continue
        if not isinstance(assign.value, ast.List):
            continue
        names = [_string_value(e) for e in assign.value.elts]
        assert 'waypoint_follower' not in names, (
            f'lifecycle_nodes still lists waypoint_follower: {names}')
        return
    raise AssertionError('no lifecycle_nodes = [...] assignment found to check')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
