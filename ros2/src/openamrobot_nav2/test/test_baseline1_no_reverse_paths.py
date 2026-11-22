# SPDX-License-Identifier: MIT
"""No reverse path is enabled in the shipped Nav2 configuration (D-19; #90's behaviour set on Baseline 1, Proposed).

Pure Python over config/nav2_params.yaml and behavior_trees/: no ROS, no Nav2 start. It proves what the files say,
not how the robot moves. reverse_paths() lists every way the configuration lets Nav2 drive the base backwards:

1. behavior_server registers a reverse-capable behaviour (BackUp, DriveOnHeading, AssistedTeleop plugins, by plugin
   class, whatever name they are registered under).
2. A DWB controller (FollowPath's primary controller) has a negative min_vel_x.
3. velocity_smoother lets a negative linear x through (min_velocity[0] < 0).
4. A graceful controller sets allow_backward true. nav2_graceful_controller turns it off itself when
   initial_rotation is on (it logs "setting allow backward to false"), but the file must say what runs, so any
   allow_backward true is listed, with or without initial_rotation (Kunal's review of #164 and #178).
5. A Regulated Pure Pursuit controller sets allow_reversing true (PROPOSED with StationRetreat, upperbody #117). RPP
   only rejects a later dynamic change to true while use_rotate_to_heading is on, so any allow_reversing true is listed.
6. A behaviour tree calls BackUp, DriveOnHeading or AssistedTeleop (none of them is registered, see 1).

The Baseline 1 retreat of openamr-upperbody-sw uses behavior_server's backup action today; with this configuration it
has no server. Its alternatives (a declared station departure, or a forward retreat) are prepared in that repository.
The docking undock reverse of openamrobot_docking (dock_trigger.yaml) is not Nav2 configuration and is not checked here.
"""
import glob
import os
import xml.etree.ElementTree as ET

import yaml

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
PARAMS = os.path.join(PACKAGE, 'config', 'nav2_params.yaml')
TREES = os.path.join(PACKAGE, 'behavior_trees')

REVERSE_BEHAVIOURS = ('nav2_behaviors::BackUp', 'nav2_behaviors::DriveOnHeading', 'nav2_behaviors::AssistedTeleop')
REVERSE_TREE_NODES = ('BackUp', 'DriveOnHeading', 'AssistedTeleop')


def _controllers(controller_server):
    """(name, parameters) of every controller and its primary controller parameters."""
    for name in controller_server.get('controller_plugins', []):
        yield name, controller_server.get(name, {})


def reverse_paths(params, trees):
    """Every reverse path the configuration enables. params: the parsed nav2_params.yaml; trees: {file: xml text}."""
    found = []
    behaviors = params.get('behavior_server', {}).get('ros__parameters', {})
    for name in behaviors.get('behavior_plugins', []):
        plugin = (behaviors.get(name) or {}).get('plugin')
        if plugin in REVERSE_BEHAVIOURS or plugin is None:
            found.append(f'behavior_server registers {name} ({plugin or "no plugin entry"})')
    controller = params.get('controller_server', {}).get('ros__parameters', {})
    for name, values in _controllers(controller):
        plugin = values.get('plugin', '')
        primary = values.get('primary_controller', '')
        if 'DWB' in plugin or 'dwb' in primary or 'dwb' in plugin:
            if values.get('min_vel_x', 0.0) < 0.0:
                found.append(f'{name} min_vel_x {values["min_vel_x"]}')
        if 'graceful' in plugin.lower():
            if values.get('allow_backward', False):
                found.append(f'{name} allow_backward true')
        if 'regulated_pure_pursuit' in plugin.lower():
            if values.get('allow_reversing', False):
                found.append(f'{name} allow_reversing true')
    smoother = params.get('velocity_smoother', {}).get('ros__parameters', {})
    if smoother.get('min_velocity', [0.0])[0] < 0.0:
        found.append(f'velocity_smoother min_velocity {smoother["min_velocity"]}')
    for file, text in sorted(trees.items()):
        root = ET.fromstring(text)
        for tag in REVERSE_TREE_NODES:
            if any(True for _ in root.iter(tag)):
                found.append(f'{file} calls {tag}')
    return found


def shipped():
    with open(PARAMS, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    trees = {}
    for path in sorted(glob.glob(os.path.join(TREES, '*.xml'))):
        with open(path, encoding='utf-8') as f:
            trees[os.path.basename(path)] = f.read()
    return params, trees


def test_the_shipped_configuration_has_no_reverse_path():
    params, trees = shipped()
    assert trees, 'no behaviour trees found'
    assert reverse_paths(params, trees) == []


def test_spin_and_wait_stay_registered():
    params, _trees = shipped()
    assert params['behavior_server']['ros__parameters']['behavior_plugins'] == ['spin', 'wait']


def test_the_check_finds_each_reverse_path():
    params = {
        'behavior_server': {'ros__parameters': {
            'behavior_plugins': ['spin', 'retreat', 'drive', 'teleop'],
            'spin': {'plugin': 'nav2_behaviors::Spin'},
            'retreat': {'plugin': 'nav2_behaviors::BackUp'},
            'drive': {'plugin': 'nav2_behaviors::DriveOnHeading'},
            'teleop': {'plugin': 'nav2_behaviors::AssistedTeleop'}}},
        'controller_server': {'ros__parameters': {
            'controller_plugins': ['FollowPath', 'Approach', 'Retreat'],
            'FollowPath': {'plugin': 'nav2_rotation_shim_controller::RotationShimController',
                           'primary_controller': 'dwb_core::DWBLocalPlanner', 'min_vel_x': -0.05},
            'Approach': {'plugin': 'nav2_graceful_controller::GracefulController', 'allow_backward': True,
                         'initial_rotation': False},
            'Retreat': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                        'allow_reversing': True, 'use_rotate_to_heading': False}}},
        'velocity_smoother': {'ros__parameters': {'min_velocity': [-0.2, 0.0, -0.5]}},
    }
    trees = {'t.xml': '<root><BehaviorTree ID="T"><BackUp backup_dist="0.3"/></BehaviorTree></root>'}
    found = reverse_paths(params, trees)
    assert len(found) == 8, found


def test_graceful_allow_backward_true_is_a_reverse_path():
    def graceful(**values):
        return {'controller_server': {'ros__parameters': {
            'controller_plugins': ['A'], 'A': dict(plugin='nav2_graceful_controller::GracefulController', **values)}}}
    assert reverse_paths(graceful(allow_backward=True, initial_rotation=True), {}) != []
    assert reverse_paths(graceful(allow_backward=False, initial_rotation=True), {}) == []
    assert reverse_paths(graceful(allow_backward=True, initial_rotation=False), {}) != []
    params, _trees = shipped()
    station = params['controller_server']['ros__parameters'].get('StationApproach')
    if station is not None:
        assert station['allow_backward'] is False, 'StationApproach is forward only (D-19)'
        assert station['initial_rotation'] is True


def test_rpp_allow_reversing_true_is_a_reverse_path():
    def rpp(**values):
        return {'controller_server': {'ros__parameters': {
            'controller_plugins': ['R'],
            'R': dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController', **values)}}}
    assert reverse_paths(rpp(allow_reversing=True, use_rotate_to_heading=False), {}) == ['R allow_reversing true']
    assert reverse_paths(rpp(allow_reversing=True, use_rotate_to_heading=True), {}) == ['R allow_reversing true']
    assert reverse_paths(rpp(allow_reversing=False, use_rotate_to_heading=True), {}) == []
    params, _trees = shipped()
    retreat = params['controller_server']['ros__parameters'].get('StationRetreat')
    if retreat is not None:
        assert retreat['allow_reversing'] is False, 'StationRetreat is forward only (D-19)'
        assert retreat['use_rotate_to_heading'] is True


def test_the_station_retreat_tree_uses_station_retreat_and_the_station_checker():
    """PROPOSED (upperbody #117): the forward-retreat tree is the station stage only, on StationRetreat."""
    params, trees = shipped()
    controller = params['controller_server']['ros__parameters']
    assert 'StationRetreat' in controller['controller_plugins']
    assert controller['StationRetreat']['plugin'] == \
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert 'station_goal_checker' in controller['goal_checker_plugins']
    root = ET.fromstring(trees['navigate_to_station_retreat.xml'])
    follows = [(f.get('controller_id'), f.get('goal_checker_id')) for f in root.iter('FollowPath')]
    assert follows == [('StationRetreat', 'station_goal_checker')]
    assert [p.get('planner_id') for p in root.iter('ComputePathToPose')] == ['StationPlanner']
    station = ET.fromstring(trees['navigate_to_station.xml'])
    station_controllers = {f.get('controller_id') for f in station.iter('FollowPath')}
    assert station_controllers == {'{selected_controller}', 'StationApproach'}, 'the shelf keeps StationApproach'
