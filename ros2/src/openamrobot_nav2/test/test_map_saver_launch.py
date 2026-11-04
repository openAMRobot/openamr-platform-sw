"""
Regression test: map_saver_launch.py must start an active, correctly wired map_saver.

Statically inspects the launch file's source via ast, the same style as
test_navigation_launch_cmd_vel_routing.py - no ROS graph needed. nav2's
map_saver_server only advertises /map_saver/save_map once a lifecycle_manager
has brought it to the active state (confirmed live against nav2_map_server on
ROS 2 Jazzy: the service is absent while the node is unconfigured). Without
both the map_saver_server node and a lifecycle_manager naming it in
node_names, /map_saver/save_map never appears.

Also covers two review findings (platform-sw#115, Mohamed, head f903003):
use_sim_time never reached the map_saver_server node's own parameters (only
the lifecycle_manager had it, so the saver silently stayed on wall time under
sim:=true), and the namespace argument was declared but never applied to
either node (no namespace= and no PushROSNamespace), so namespace:=robot1
never actually namespaced the nodes or /map_saver/save_map. These were
confirmed live - `ros2 param get /map_saver use_sim_time` and
`ros2 node list`/`ros2 service list` under namespace:=robot1 - not just by
this static check; see the PR for the exact commands and output.
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


def _is_launch_configuration_of(tree, node, name):
    """Return True if node is (or resolves through a Name to) LaunchConfiguration(name)."""
    node = _resolve_name(tree, node)
    if not isinstance(node, ast.Call):
        return False
    func_name = getattr(node.func, 'id', None) or getattr(node.func, 'attr', None)
    if func_name != 'LaunchConfiguration':
        return False
    args = [_string_value(a) for a in node.args]
    return args[:1] == [name]


def _group_actions(tree):
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func_name = getattr(call.func, 'id', None) or getattr(call.func, 'attr', None)
        if func_name == 'GroupAction':
            yield call


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


def test_map_saver_server_receives_use_sim_time():
    """
    Check map_saver_server's own parameters, not just the lifecycle_manager's.

    platform-sw#115 P2 (Mohamed): the map_saver: yaml block has no
    use_sim_time key and RewrittenYaml(param_rewrites={}) adds none, so
    use_sim_time must be passed to the Node's own parameters directly, or the
    saver silently stays on wall time while the rest of a sim stack runs on
    Gazebo time.
    """
    tree = _tree()
    nodes = list(_node_calls(tree))
    matches = [
        kwargs for kwargs in nodes
        if _string_value(kwargs.get('package')) == 'nav2_map_server'
        and _string_value(kwargs.get('executable')) == 'map_saver_server'
    ]
    assert len(matches) == 1, f'Expected one map_saver_server Node, found {len(matches)}.'

    parameters_node = matches[0].get('parameters')
    assert isinstance(parameters_node, ast.List), (
        'map_saver_server Node has no parameters=[...] list to check.')

    found = False
    for element in parameters_node.elts:
        if not isinstance(element, ast.Dict):
            continue
        for key, value in zip(element.keys, element.values):
            if _string_value(key) == 'use_sim_time' and _is_launch_configuration_of(
                    tree, value, 'use_sim_time'):
                found = True

    assert found, (
        "map_saver_server's parameters must include "
        "{'use_sim_time': LaunchConfiguration('use_sim_time')} directly - the "
        'yaml params file alone does not carry it, so without this the saver '
        'silently stays on wall time under sim:=true.')


def test_both_nodes_are_namespaced():
    """
    Check that both nodes are wrapped in PushROSNamespace(namespace).

    platform-sw#115 P2 (Mohamed): the namespace argument was declared and
    fed into the params-file rewriter, but never applied to either node (no
    namespace= and no PushROSNamespace), so namespace:=robot1 never actually
    namespaced the nodes or /map_saver/save_map.
    """
    tree = _tree()
    groups = list(_group_actions(tree))
    assert len(groups) == 1, (
        f'Expected the map_saver_server and lifecycle_manager Nodes to be '
        f'wrapped in exactly one GroupAction so PushROSNamespace applies to '
        f'both; found {len(groups)} GroupAction call(s).')

    group_call = groups[0]
    assert group_call.args, 'GroupAction([...]) must take its actions as a list.'
    actions_list = group_call.args[0]
    assert isinstance(actions_list, ast.List), (
        "GroupAction's first argument must be a list literal.")

    push_ns_calls = []
    for element in actions_list.elts:
        if isinstance(element, ast.Call):
            func_name = getattr(element.func, 'id', None) or getattr(element.func, 'attr', None)
            if func_name == 'PushROSNamespace':
                push_ns_calls.append(element)
    assert len(push_ns_calls) == 1, (
        f'Expected exactly one PushROSNamespace(...) inside the GroupAction, '
        f'found {len(push_ns_calls)}.')

    push_ns_args = push_ns_calls[0].args + [
        kw.value for kw in push_ns_calls[0].keywords if kw.arg == 'namespace']
    assert push_ns_args, 'PushROSNamespace(...) must be given the namespace argument.'
    assert _is_launch_configuration_of(tree, push_ns_args[0], 'namespace'), (
        "PushROSNamespace must be given LaunchConfiguration('namespace') so "
        'the declared namespace argument actually applies to the nodes, not '
        'just the rewritten params file.')

    node_calls_in_group = [
        c for c in actions_list.elts
        if isinstance(c, ast.Call)
        and (getattr(c.func, 'id', None) or getattr(c.func, 'attr', None)) == 'Node'
    ]
    assert len(node_calls_in_group) == 2, (
        'Expected both the map_saver_server and lifecycle_manager Node() '
        f'calls inside the same namespaced GroupAction, found '
        f'{len(node_calls_in_group)}.')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
