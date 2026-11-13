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

"""H-day legacy profile: nav-only, no docking, no replay, no legacy UI move_base, 0.20 m/s."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PKG, 'launch', 'hday_legacy.launch.py')
LIMITS_FILE = os.path.join(PKG, 'config', 'hday_commissioning_limits.yaml')
NAV2_PARAMS = os.path.join(
    os.path.dirname(PKG), 'openamrobot_nav2', 'config', 'nav2_params.yaml')

PRODUCT_CEILING = 0.5


def _module():
    spec = importlib.util.spec_from_file_location('hday_legacy_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fake_share(pkg):
    if pkg == 'openamrobot_bringup':
        return PKG
    if pkg == 'openamrobot_nav2':
        return os.path.dirname(NAV2_PARAMS).rsplit(os.sep, 1)[0]
    return os.path.join('/fake-share', pkg)


def _expand(tmp_path, lidar_model_file=True):
    module = _module()
    module.get_package_share_directory = _fake_share
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: m.pgm\n')
    context = LaunchContext()
    context.launch_configurations['map'] = str(map_file)
    return module, module.launch_setup(context), map_file


def _include_info(include):
    source = include.launch_description_source
    location = perform_substitutions(
        LaunchContext(), source._LaunchDescriptionSource__location)
    return location, {
        (perform_substitutions(LaunchContext(), k) if not isinstance(k, str) else k):
        (v if isinstance(v, str) else perform_substitutions(LaunchContext(), v))
        for k, v in include.launch_arguments}


def test_declares_map_and_nothing_to_enable_motion():
    description = _module().generate_launch_description()
    names = [e.name for e in description.entities if hasattr(e, 'default_value')]
    assert names == ['map']
    assert any(isinstance(e, OpaqueFunction) for e in description.entities)


def test_map_is_required(tmp_path):
    module = _module()
    module.get_package_share_directory = _fake_share
    context = LaunchContext()
    context.launch_configurations['map'] = ''
    with pytest.raises(RuntimeError, match='requires map'):
        module.launch_setup(context)
    context.launch_configurations['map'] = str(tmp_path / 'missing.yaml')
    with pytest.raises(RuntimeError, match='not found'):
        module.launch_setup(context)


def test_includes_exactly_the_nav_only_stack(tmp_path):
    _, actions, _ = _expand(tmp_path)
    assert all(isinstance(a, IncludeLaunchDescription) for a in actions)
    files = [os.path.basename(_include_info(a)[0]) for a in actions]
    assert files == ['real_bringup.launch.py', 'goal_relay.launch.py',
                     'localization_launch.py', 'navigation_launch.py']


def test_nothing_forbidden_is_included(tmp_path):
    _, actions, _ = _expand(tmp_path)
    for action in actions:
        location = _include_info(action)[0].lower()
        for word in ('docking', 'dock_trigger', 'apriltag', 'replay', 'move_base',
                     'physnode', 'openamr_ui', 'gazebo', 'gz_'):
            assert word not in location, location
    with open(LAUNCH_FILE, encoding='utf-8') as f:
        code = [line for line in f if not line.lstrip().startswith('#')]
    body = ''.join(code).split('"""', 2)[2]
    for word in ('docking_real', 'openamrobot_docking', 'bag', 'move_base_launch'):
        assert word not in body


def test_legacy_rig_arguments(tmp_path):
    _, actions, _ = _expand(tmp_path)
    location, args = _include_info(actions[0])
    assert location.endswith('real_bringup.launch.py')
    assert args['lidar_model'] == 'a1'        # release 001 rig: RPLIDAR A1
    assert args['use_camera'] == 'false'      # no IMX708 on H-day
    # sim time off for both Nav2 includes
    for action in actions[2:]:
        assert _include_info(action)[1]['use_sim_time'] == 'false'


def test_teensy_is_the_default_base_controller_in_drivers():
    drivers = os.path.join(os.path.dirname(PKG), 'openamrobot_drivers', 'launch',
                           'drivers.launch.py')
    with open(drivers, encoding='utf-8') as f:
        text = f.read()
    assert "TEENSY_DEFAULT = '/dev/serial/by-id/usb-Teensyduino" in text


def test_commissioning_limits_below_product_ceiling():
    with open(LIMITS_FILE, encoding='utf-8') as f:
        limits = yaml.safe_load(f)
    follow = limits['controller_server']['ros__parameters']['FollowPath']
    smoother = limits['velocity_smoother']['ros__parameters']
    assert follow['max_vel_x'] == 0.20 and follow['max_speed_xy'] == 0.20
    assert follow['max_vel_theta'] == 1.0
    assert smoother['max_velocity'] == [0.20, 0.0, 1.0]
    assert smoother['min_velocity'] == [-0.20, 0.0, -1.0]
    assert follow['max_vel_x'] <= PRODUCT_CEILING
    assert abs(smoother['max_velocity'][0]) <= PRODUCT_CEILING


def test_merged_params_apply_limits_and_keep_everything_else(tmp_path):
    module, actions, map_file = _expand(tmp_path)
    params_path = _include_info(actions[2])[1]['params_file']
    with open(params_path, encoding='utf-8') as f:
        merged = yaml.safe_load(f)
    with open(NAV2_PARAMS, encoding='utf-8') as f:
        original = yaml.safe_load(f)
    follow = merged['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_vel_x'] == 0.20 and follow['max_vel_theta'] == 1.0
    assert merged['velocity_smoother']['ros__parameters']['max_velocity'] == [0.20, 0.0, 1.0]
    # the rest of the DWB block is untouched
    o_follow = original['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == o_follow['plugin']
    assert follow['acc_lim_x'] == o_follow['acc_lim_x']
    assert merged['collision_monitor'] == original['collision_monitor']
    assert merged['map_server']['ros__parameters']['yaml_filename'] == str(map_file)
    assert merged['amcl']['ros__parameters']['set_initial_pose'] is False
    # nav2_params.yaml on disk keeps the product values
    assert original['velocity_smoother']['ros__parameters']['max_velocity'][2] == 0.5


def test_nav2_and_localization_share_the_merged_file(tmp_path):
    _, actions, _ = _expand(tmp_path)
    loc = _include_info(actions[2])[1]
    nav = _include_info(actions[3])[1]
    assert loc['params_file'] == nav['params_file']
    assert loc['map'] == ''


def test_imu_topic_is_gate_a_when_real_bringup_declares_it(tmp_path):
    # Before #129: real_bringup has no imu_topic and none is passed. After #129 it must be
    # /imu/data, the Gate A Teensy topic.
    _, actions, _ = _expand(tmp_path)
    _, args = _include_info(actions[0])
    with open(os.path.join(PKG, 'launch', 'real_bringup.launch.py'), encoding='utf-8') as f:
        declared = "'imu_topic'" in f.read()
    if declared:
        assert args['imu_topic'] == '/imu/data'
    else:
        assert 'imu_topic' not in args
