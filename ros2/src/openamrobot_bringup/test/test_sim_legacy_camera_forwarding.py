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

"""Check legacy camera forwarding without launching Gazebo or ROS nodes."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


BRINGUP_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'bringup.launch.py')
DOCKING_SIM_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'openamrobot_docking', 'launch', 'bringup_sim.launch.py')


def _load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _argument_default(description, name):
    arguments = [entity for entity in description.entities
                 if isinstance(entity, DeclareLaunchArgument) and entity.name == name]
    assert len(arguments) == 1
    value = arguments[0].default_value
    return perform_substitutions(
        LaunchContext(), normalize_to_list_of_substitutions(value))


def _resolved_arguments(include, context):
    return {
        perform_substitutions(context, normalize_to_list_of_substitutions(key)):
        perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for key, value in include.launch_arguments
    }


def _unified_gazebo_arguments(use_docking=None):
    module = _load_launch(BRINGUP_LAUNCH, 'bringup_sim_camera_test')
    module.get_package_share_directory = lambda package: '/fake/' + package
    module._params_with_map = lambda *args: '/tmp/fake-nav2-params.yaml'

    description = module.generate_launch_description()
    if use_docking is None:
        use_docking = _argument_default(description, 'use_docking')
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': 'true',
        'use_rviz': 'false',
        'gui': 'false',
        'use_camera': 'true',
        'use_docking': use_docking,
        'map': '/tmp',
    })
    actions = module.launch_setup(context)
    gazebo_include = next(
        action for action in actions if isinstance(action, IncludeLaunchDescription))
    return description, _resolved_arguments(gazebo_include, context)


def test_unified_sim_docking_default_enables_legacy_camera():
    description, arguments = _unified_gazebo_arguments()

    assert _argument_default(description, 'use_docking') == 'true'
    assert arguments['legacy_camera'] == 'true'


def test_unified_sim_navigation_only_disables_legacy_camera():
    _, arguments = _unified_gazebo_arguments('false')

    assert arguments['legacy_camera'] == 'false'


@pytest.mark.parametrize('override', ['true', 'false'])
def test_docking_sim_forwards_legacy_camera_override(override):
    module = _load_launch(DOCKING_SIM_LAUNCH, 'docking_sim_camera_test')
    description = module.generate_launch_description()
    assert _argument_default(description, 'legacy_camera') == 'true'

    gazebo_include = next(
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
        and any(key == 'legacy_camera' for key, _ in action.launch_arguments))
    context = LaunchContext()
    context.launch_configurations.update({
        'legacy_camera': override,
        'gazebo_gui': 'false',
        'base_tilt': 'bt10',
    })

    assert _resolved_arguments(gazebo_include, context)['legacy_camera'] == override
