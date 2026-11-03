"""
Regression test: map_saver_launch.py must start an active map_saver.

Statically inspects the launch file's source via ast, the same style as
test_navigation_launch_cmd_vel_routing.py - no ROS graph needed. nav2's
map_saver_server only advertises /map_saver/save_map once a lifecycle_manager
has brought it to the active state (confirmed live against nav2_map_server on
ROS 2 Jazzy: the service is absent while the node is unconfigured). Without
both the map_saver_server node and a lifecycle_manager naming it in
node_names, /map_saver/save_map never appears.
"""
import ast
import os

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'map_saver_launch.py')


def _string_value(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _tree():
    with open(LAUNCH_FILE) as f:
        return ast.parse(f.read(), filename=LAUNCH_FILE)


def _node_calls(tree):
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func_name = getattr(call.func, 'id', None) or getattr(call.func, 'attr', None)
        if func_name != 'Node':
            continue
        yield {kw.arg: kw.value for kw in call.keywords if kw.arg}


def _resolve_name(tree, node):
    """If node is a bare variable reference, resolve it to its assigned value."""
    if not isinstance(node, ast.Name):
        return node
    for assign in ast.walk(tree):
        if isinstance(assign, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == node.id for t in assign.targets):
            return assign.value
    return node


def _list_of_strings(node):
    if not isinstance(node, ast.List):
        return None
    return [_string_value(e) for e in node.elts]


def test_map_saver_server_node_is_defined():
    nodes = list(_node_calls(_tree()))
    matches = [
        kwargs for kwargs in nodes
        if _string_value(kwargs.get('package')) == 'nav2_map_server'
        and _string_value(kwargs.get('executable')) == 'map_saver_server'
        and _string_value(kwargs.get('name')) == 'map_saver'
    ]
    assert len(matches) == 1, (
        'Expected exactly one Node(package="nav2_map_server", '
        'executable="map_saver_server", name="map_saver") in '
        f'map_saver_launch.py, found {len(matches)}.')


def test_lifecycle_manager_activates_map_saver():
    tree = _tree()
    nodes = list(_node_calls(tree))
    matches = [
        kwargs for kwargs in nodes
        if _string_value(kwargs.get('package')) == 'nav2_lifecycle_manager'
        and _string_value(kwargs.get('executable')) == 'lifecycle_manager'
    ]
    assert len(matches) == 1, (
        'Expected exactly one nav2_lifecycle_manager Node in '
        f'map_saver_launch.py, found {len(matches)}.')

    parameters_node = matches[0].get('parameters')
    assert isinstance(parameters_node, ast.List), (
        'lifecycle_manager Node has no parameters=[...] list to check.')

    node_names = None
    for dict_node in parameters_node.elts:
        if not isinstance(dict_node, ast.Dict):
            continue
        for key, value in zip(dict_node.keys, dict_node.values):
            if _string_value(key) == 'node_names':
                node_names = _list_of_strings(_resolve_name(tree, value))

    assert node_names == ['map_saver'], (
        "lifecycle_manager's node_names must be exactly ['map_saver'] so it "
        f'brings map_saver to the active state; found {node_names}. Without '
        'this, /map_saver/save_map is never advertised.')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
