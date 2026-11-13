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

"""Introspect bringup_composed.launch.py: imu_topic is declared and forwarded to bringup."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'bringup_composed.launch.py')


def _description():
    spec = importlib.util.spec_from_file_location('bringup_composed_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Resolve share directories to fake paths: no installed packages needed, nothing started.
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    return module.generate_launch_description()


def _bringup_include(description):
    # The source path is a substitution until launch time, so find the bringup include by
    # its arguments: it is the only include that passes use_camera.
    context = LaunchContext()
    includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)]

    def names(include):
        return {perform_substitutions(context, normalize_to_list_of_substitutions(k))
                for k, _ in include.launch_arguments}
    found = [i for i in includes if 'use_camera' in names(i)]
    assert len(found) == 1
    return found[0]


def test_imu_topic_declared_raw_default():
    args = [e for e in _description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name == 'imu_topic']
    assert len(args) == 1
    assert args[0].default_value[0].text == '/imu/data_raw'
    assert sorted(args[0].choices) == ['/imu/data', '/imu/data_raw']


def test_imu_topic_forwarded_to_bringup():
    # The 16 Nov legacy session runs this entry point on the Gate A Teensy (/imu/data).
    include = _bringup_include(_description())
    context = LaunchContext()
    for value in ('/imu/data_raw', '/imu/data'):
        context.launch_configurations['imu_topic'] = value
        context.launch_configurations['map'] = '/tmp/m.yaml'
        passed = {
            perform_substitutions(context, normalize_to_list_of_substitutions(k)):
            perform_substitutions(context, normalize_to_list_of_substitutions(v))
            for k, v in include.launch_arguments}
        assert passed['imu_topic'] == value
        assert passed['use_camera'] == 'false' and passed['use_docking'] == 'false'
