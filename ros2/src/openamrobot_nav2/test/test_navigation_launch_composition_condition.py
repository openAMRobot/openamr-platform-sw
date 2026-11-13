#!/usr/bin/env python3
"""Regression check for lowercase use_composition launch arguments (#48)."""
import ast
import os


LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'navigation_launch.py')


def test_non_composed_nodes_use_boolean_launch_condition():
    """The non-composed group must parse lowercase true/false as booleans."""
    with open(LAUNCH_FILE) as launch_file:
        tree = ast.parse(launch_file.read(), filename=LAUNCH_FILE)

    load_nodes = next(
        node.value for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'load_nodes'
                for target in node.targets))
    condition = next(
        keyword.value for keyword in load_nodes.keywords
        if keyword.arg == 'condition')

    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'UnlessCondition'
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == 'use_composition'


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
