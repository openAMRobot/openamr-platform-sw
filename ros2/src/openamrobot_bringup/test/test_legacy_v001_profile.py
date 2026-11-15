# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Introspect legacy_v001.launch.py without hardware and without installed packages.

The launch file is loaded from the source tree, share directories point at the source
packages, and launch_setup() is evaluated in a LaunchContext. Nothing is started.
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.dirname(PKG_DIR)
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'legacy_v001.launch.py')
NAV2_PARAMS = os.path.join(SRC_DIR, 'openamrobot_nav2', 'config', 'nav2_params.yaml')
OVERRIDES = os.path.join(PKG_DIR, 'config', 'legacy_v001', 'nav2_overrides.yaml')
STATUS_PROFILE = os.path.join(PKG_DIR, 'config', 'legacy_v001',
                              'navigation_status_profile.yaml')

# P-03 item 30 A5: the base speed ceiling. The profile must stay at or below it.
PRODUCT_CEILING_MPS = 0.5
# Commissioning limit for the first floor run on the v0.0.1 platform (roadmap rev 2, L-G5).
COMMISSIONING_MPS = 0.20


def _module():
    spec = importlib.util.spec_from_file_location('legacy_v001_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda pkg: os.path.join(SRC_DIR, pkg)
    return module


def _actions(tmp_path):
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n')
    context = LaunchContext()
    context.launch_configurations['map'] = str(map_file)
    return _module().launch_setup(context), context, str(map_file)


def _location(include):
    source = include.launch_description_source
    return perform_substitutions(LaunchContext(), source._LaunchDescriptionSource__location)


def _includes(actions):
    return {os.path.relpath(_location(a), SRC_DIR): a
            for a in actions if isinstance(a, IncludeLaunchDescription)}


def _include_args(include, context):
    return {
        perform_substitutions(context, normalize_to_list_of_substitutions(k)):
        perform_substitutions(context, normalize_to_list_of_substitutions(v))
        for k, v in include.launch_arguments}


def _nodes(actions):
    return [a for a in actions if isinstance(a, Node)]


def _node_name(node, context):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(node._Node__node_name))


def _static_tf_pairs(actions):
    pairs = []
    for node in _nodes(actions):
        flat = [perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(a))
                for a in node._Node__arguments or []]
        if '--frame-id' in flat:
            pairs.append((flat[flat.index('--frame-id') + 1],
                          flat[flat.index('--child-frame-id') + 1]))
    return pairs


def test_map_is_required():
    context = LaunchContext()
    context.launch_configurations['map'] = ''
    with pytest.raises(RuntimeError, match='requires map'):
        _module().launch_setup(context)


def test_missing_map_file_is_rejected(tmp_path):
    context = LaunchContext()
    context.launch_configurations['map'] = str(tmp_path / 'absent.yaml')
    with pytest.raises(RuntimeError, match='not found'):
        _module().launch_setup(context)


def test_includes_exactly_the_nav_only_stack(tmp_path):
    actions, _, _ = _actions(tmp_path)
    assert sorted(_includes(actions)) == sorted([
        'openamrobot_drivers/launch/drivers.launch.py',
        'openamrobot_perception/launch/scan_body_filter.launch.py',
        'openamrobot_bringup/launch/goal_relay.launch.py',
        'openamrobot_nav2/launch/localization_launch.py',
        'openamrobot_nav2/launch/navigation_launch.py',
    ])


def test_no_docking_camera_or_second_goal_forwarder(tmp_path):
    actions, _, _ = _actions(tmp_path)
    locations = ' '.join(_includes(actions))
    assert 'docking' not in locations
    assert 'camera' not in locations
    assert 'real_bringup' not in locations
    assert list(_includes(actions)).count('openamrobot_bringup/launch/goal_relay.launch.py') == 1


def test_drivers_get_the_a1_lidar(tmp_path):
    actions, context, _ = _actions(tmp_path)
    include = _includes(actions)['openamrobot_drivers/launch/drivers.launch.py']
    assert _include_args(include, context) == {'lidar_model': 'a1'}


def test_navigation_args_disable_follower_and_set_profile(tmp_path):
    actions, context, _ = _actions(tmp_path)
    include = _includes(actions)['openamrobot_nav2/launch/navigation_launch.py']
    args = _include_args(include, context)
    assert args['use_sim_time'] == 'false'
    assert args['use_waypoint_follower'] == 'false'
    assert args['status_profile_path'].endswith(
        os.path.join('legacy_v001', 'navigation_status_profile.yaml'))
    producers = args['enabled_producers'].split(',')
    assert producers == ['navigate_to_pose', 'navigate_through_poses']
    loc = _include_args(
        _includes(actions)['openamrobot_nav2/launch/localization_launch.py'], context)
    assert loc['params_file'] == args['params_file']
    assert loc['map'] == ''


def test_temp_params_have_map_and_limits(tmp_path):
    actions, context, map_file = _actions(tmp_path)
    include = _includes(actions)['openamrobot_nav2/launch/navigation_launch.py']
    with open(_include_args(include, context)['params_file']) as f:
        params = yaml.safe_load(f)
    assert params['map_server']['ros__parameters']['yaml_filename'] == map_file
    assert params['amcl']['ros__parameters']['set_initial_pose'] is False
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_vel_x'] == COMMISSIONING_MPS
    assert follow['max_speed_xy'] == COMMISSIONING_MPS
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == COMMISSIONING_MPS
    assert smoother['min_velocity'][0] == -COMMISSIONING_MPS
    # The rest of nav2_params.yaml survives the merge untouched.
    with open(NAV2_PARAMS) as f:
        base = yaml.safe_load(f)
    assert params['collision_monitor'] == base['collision_monitor']
    assert params['bt_navigator'] == base['bt_navigator']


def test_speed_limits_never_exceed_the_ceiling():
    with open(OVERRIDES) as f:
        overrides = yaml.safe_load(f)
    follow = overrides['controller_server']['ros__parameters']['FollowPath']
    smoother = overrides['velocity_smoother']['ros__parameters']
    linear = [follow['max_vel_x'], follow['max_speed_xy'],
              smoother['max_velocity'][0], -smoother['min_velocity'][0]]
    assert all(0.0 < v <= PRODUCT_CEILING_MPS for v in linear)
    # Smoother and controller agree, so neither silently clips the other.
    assert smoother['max_velocity'][0] == follow['max_vel_x']
    assert smoother['max_velocity'][2] == follow['max_vel_theta']


def test_nav2_params_file_is_not_modified_by_the_profile(tmp_path):
    with open(NAV2_PARAMS) as f:
        before = f.read()
    _actions(tmp_path)
    with open(NAV2_PARAMS) as f:
        assert f.read() == before


def test_ekf_reads_the_teensy_imu_topic_and_owns_odom(tmp_path):
    actions, context, _ = _actions(tmp_path)
    ekf = [n for n in _nodes(actions) if _node_name(n, context) == 'ekf_filter_node']
    assert len(ekf) == 1
    params = {}
    for entry in evaluate_parameters(context, ekf[0]._Node__parameters):
        if isinstance(entry, dict):
            params.update(entry)
    assert params['imu0'] == '/imu/data'
    remaps = [(perform_substitutions(context, normalize_to_list_of_substitutions(a)),
               perform_substitutions(context, normalize_to_list_of_substitutions(b)))
              for a, b in ekf[0]._Node__remappings]
    assert remaps == [('odometry/filtered', '/odom')]


def test_ekf_config_matches_the_teensy_odometry():
    with open(os.path.join(PKG_DIR, 'config', 'ekf.yaml')) as f:
        ekf = yaml.safe_load(f)['ekf_filter_node']['ros__parameters']
    assert ekf['odom0'] == '/odom/unfiltered'
    assert ekf['publish_tf'] is True
    assert ekf['odom_frame'] == 'odom'
    assert ekf['base_link_frame'] == 'base_link'


def test_one_tf_tree_and_no_second_odom_publisher(tmp_path):
    actions, _, _ = _actions(tmp_path)
    pairs = _static_tf_pairs(actions)
    assert sorted(pairs) == sorted([
        ('base_link', 'lidar_link'),
        ('base_link', 'imu_link'),
        ('base_link', 'base_footprint'),
    ])
    children = [child for _, child in pairs]
    assert len(children) == len(set(children))  # every frame has exactly one parent
    assert 'odom' not in [p for p, _ in pairs]  # only the EKF publishes odom -> base_link
    assert 'base_link' not in children


def test_status_profile_is_valid_and_never_optimistic():
    with open(STATUS_PROFILE) as f:
        profile = yaml.safe_load(f)
    assert profile['profile_id'] == 'legacy_v001'
    keys = ('id', 'kind', 'topic', 'frame_id', 'required', 'nominal_period_s')
    for sensor in profile['sensors']:
        assert all(k in sensor for k in keys), sensor
    required = {s['id'] for s in profile['sensors'] if s['required']}
    assert required == {'lidar_nav', 'odom'}
    topics = {s['id']: s['topic'] for s in profile['sensors']}
    assert topics['imu'] == '/imu/data'
    assert topics['lidar_nav'] == '/scan_filtered'
    # No motion source claims full protection and the rear is not claimed as covered.
    assert all(m['coverage'] != 'PROTECTED' for m in profile['motion_sources'])
    assert profile['rear_coverage'] != 'AVAILABLE'


def test_existing_launch_defaults_untouched():
    real = os.path.join(PKG_DIR, 'launch', 'real_bringup.launch.py')
    with open(real) as f:
        text = f.read()
    # The legacy profile keeps its own copy of the TFs; real_bringup is not edited by it.
    assert "lidar_model', default_value='a1'" in text
    assert "'use_camera', default_value='true'" in text
