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

"""Introspect real_bringup.launch.py: lidar_model is declared and passed to the drivers."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'real_bringup.launch.py')


def _description():
    spec = importlib.util.spec_from_file_location('real_bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Resolve share directories to fake paths: no installed packages needed, nothing started.
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    return module.generate_launch_description()


def _location(include):
    # The public .location is only a repr string until the include is expanded.
    source = include.launch_description_source
    return perform_substitutions(LaunchContext(), source._LaunchDescriptionSource__location)


def _drivers_include(description):
    includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)
                and _location(e) == os.path.join(
                    '/fake-share', 'openamrobot_drivers', 'launch', 'drivers.launch.py')]
    assert len(includes) == 1
    return includes[0]


def test_lidar_model_declared_a1_default():
    args = [e for e in _description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name == 'lidar_model']
    assert len(args) == 1
    assert args[0].default_value[0].text == 'a1'
    assert sorted(args[0].choices) == ['a1', 's3']


def test_lidar_model_passed_to_drivers():
    include = _drivers_include(_description())
    context = LaunchContext()
    for value in ('a1', 's3'):
        context.launch_configurations['lidar_model'] = value
        passed = {
            perform_substitutions(context, normalize_to_list_of_substitutions(k)):
            perform_substitutions(context, normalize_to_list_of_substitutions(v))
            for k, v in include.launch_arguments}
        assert passed == {'lidar_model': value}


def _ekf_node(description):
    from launch_ros.actions import Node
    nodes = [e for e in description.entities if isinstance(e, Node)
             and e.node_executable == 'ekf_node']
    assert len(nodes) == 1
    return nodes[0]


def test_imu_topic_declared_raw_default():
    # D-10: the 2.0 base firmware topic is the default; Gate A is an explicit choice.
    args = [e for e in _description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name == 'imu_topic']
    assert len(args) == 1
    assert args[0].default_value[0].text == '/imu/data_raw'
    assert sorted(args[0].choices) == ['/imu/data', '/imu/data_raw']


def test_ekf_imu_input_follows_imu_topic():
    # Both producer/consumer pairs: 2.0 firmware /imu/data_raw and Gate A Teensy /imu/data.
    node = _ekf_node(_description())
    overrides = [p for p in node._Node__parameters if isinstance(p, dict)]
    assert len(overrides) == 1
    context = LaunchContext()
    for value in ('/imu/data_raw', '/imu/data'):
        context.launch_configurations['imu_topic'] = value
        resolved = {
            perform_substitutions(context, normalize_to_list_of_substitutions(k)):
            perform_substitutions(context, normalize_to_list_of_substitutions(v))
            for k, v in overrides[0].items()}
        assert resolved == {'imu0': value}
