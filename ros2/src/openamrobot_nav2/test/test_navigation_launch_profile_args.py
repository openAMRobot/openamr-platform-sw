#!/usr/bin/env python3
"""
navigation_launch.py profile arguments: use_waypoint_follower and status_profile_path.

The legacy_v001 profile (openamrobot_bringup legacy_v001.launch.py) turns the waypoint
follower off for Build 1, because follow_waypoints is not cancelled by the stop path, and
gives navigation_status_node its own sensor profile. These tests check that:
  * with the defaults, the launch is unchanged: waypoint_follower runs and is managed, and
    navigation_status_node reads the package profile it always read;
  * with use_waypoint_follower:=false, waypoint_follower is neither started nor in the
    lifecycle manager's node_names, in both the composed and the non-composed paths.

Pure introspection: nothing is started, no Nav2 package needs to be installed (only
nav2_common for RewrittenYaml).
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters
import pytest

pytest.importorskip('nav2_common.launch')

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'navigation_launch.py')
FAKE_SHARE = '/fake-share/openamrobot_nav2'


def _description():
    spec = importlib.util.spec_from_file_location('navigation_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda pkg: FAKE_SHARE
    return module.generate_launch_description()


def _context(description, **overrides):
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value)
    context.launch_configurations.update(overrides)
    return context


def _text(context, value):
    if value is None:
        return None
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _groups(description):
    return [e for e in description.entities if isinstance(e, GroupAction)]


def _plain_nodes(description):
    for group in _groups(description):
        for action in group._GroupAction__actions:
            if isinstance(action, Node):
                yield action


def _composable_nodes(description):
    for group in _groups(description):
        for action in group._GroupAction__actions:
            if isinstance(action, LoadComposableNodes):
                yield from action._LoadComposableNodes__composable_node_descriptions


def _flat_params(context, parameters):
    merged = {}
    for entry in evaluate_parameters(context, parameters):
        if isinstance(entry, dict):
            merged.update(entry)
    return merged


def _started(context, condition):
    # Node.condition is a property; ComposableNode.condition is a method.
    if callable(condition) and not hasattr(condition, 'evaluate'):
        condition = condition()
    return condition is None or condition.evaluate(context)


def _plain_started_names(description, context):
    # Some plain Nodes (controller_server on main) set no name=; the executable is the name.
    return [_text(context, n._Node__node_name) or _text(context, n.node_executable)
            for n in _plain_nodes(description) if _started(context, n.condition)]


def _composable_started_names(description, context):
    return [_text(context, n.node_name) for n in _composable_nodes(description)
            if _started(context, n.condition)]


def _managed_names(description, context):
    plain = [n for n in _plain_nodes(description)
             if _text(context, n._Node__node_name) == 'lifecycle_manager_navigation']
    composed = [n for n in _composable_nodes(description)
                if _text(context, n.node_name) == 'lifecycle_manager_navigation']
    assert len(plain) == 1 and len(composed) == 1
    return (list(_flat_params(context, plain[0]._Node__parameters)['node_names']),
            list(_flat_params(context, composed[0].parameters)['node_names']))


def _status_node(description):
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    return nodes[0]


def test_defaults_keep_the_waypoint_follower():
    description = _description()
    context = _context(description)
    assert context.launch_configurations['use_waypoint_follower'] == 'true'
    assert 'waypoint_follower' in _plain_started_names(description, context)
    assert 'waypoint_follower' in _composable_started_names(description, context)
    for names in _managed_names(description, context):
        assert names == [
            'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
            'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']


def test_follower_off_removes_it_from_start_and_lifecycle():
    description = _description()
    context = _context(description, use_waypoint_follower='false')
    assert 'waypoint_follower' not in _plain_started_names(description, context)
    assert 'waypoint_follower' not in _composable_started_names(description, context)
    for names in _managed_names(description, context):
        assert 'waypoint_follower' not in names
        assert names == [
            'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
            'bt_navigator', 'velocity_smoother', 'collision_monitor']
    # Everything else still starts.
    started = _plain_started_names(description, context)
    for name in ('controller_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor'):
        assert name in started


def test_status_profile_default_is_the_package_profile():
    description = _description()
    context = _context(description)
    params = _flat_params(context, _status_node(description)._Node__parameters)
    assert params['profile_path'] == os.path.join(
        FAKE_SHARE, 'config', 'navigation_status_profile.yaml')


def test_status_profile_can_be_overridden():
    description = _description()
    context = _context(description, status_profile_path='/x/legacy.yaml')
    params = _flat_params(context, _status_node(description)._Node__parameters)
    assert params['profile_path'] == '/x/legacy.yaml'
