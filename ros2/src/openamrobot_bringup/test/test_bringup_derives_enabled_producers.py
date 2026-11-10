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
bringup.launch.py's navigation_launch.py include must pass enabled_producers.

Derived from THIS profile's own use_docking (platform-sw#124, D-13;
panthera-momagdii's #90 review, P1).

Before this fix, the include only passed use_sim_time, so
navigation_launch.py's own default (all four producers enabled) applied
unconditionally - with use_docking:=false there is no docking server or
heartbeat at all, so safety_stop_gate_node.py's _release_blockers() read
dock_robot as never-received and undock_robot as UNKNOWN forever, and
Resume never succeeded in the supported nav-only profile.
"""
import importlib.util
import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'bringup.launch.py')


def _module(tmp_path):
    spec = importlib.util.spec_from_file_location('bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    nav2_share = tmp_path / 'openamrobot_nav2'
    (nav2_share / 'config').mkdir(parents=True)
    (nav2_share / 'config' / 'nav2_params.yaml').write_text(
        'map_server:\n  ros__parameters: {}\namcl:\n  ros__parameters: {}\n')
    (nav2_share / 'maps').mkdir()
    (nav2_share / 'maps' / 'my_map.yaml').write_text('')

    bringup_share = tmp_path / 'openamrobot_bringup'
    bringup_share.mkdir()

    shares = {'openamrobot_nav2': str(nav2_share), 'openamrobot_bringup': str(bringup_share)}
    module.get_package_share_directory = lambda pkg: shares.get(pkg, str(tmp_path / pkg))
    return module


def _location(include):
    source = include.launch_description_source
    return perform_substitutions(LaunchContext(), source._LaunchDescriptionSource__location)


def _actions_for(module, sim, use_docking, map_file=''):
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': sim, 'use_rviz': 'false', 'gui': 'false',
        'use_camera': 'true', 'use_docking': use_docking, 'map': map_file,
    })
    return module.launch_setup(context)


def _navigation_launch_include(actions):
    matches = [
        a for a in actions
        if isinstance(a, IncludeLaunchDescription)
        and _location(a).endswith(os.path.join('launch', 'navigation_launch.py'))
    ]
    assert len(matches) == 1, (
        f'expected exactly one navigation_launch.py include, found {len(matches)}')
    return matches[0]


def _enabled_producers_arg(include):
    for key, value in include.launch_arguments:
        if key == 'enabled_producers':
            return value
    return None


def test_sim_nav_only_excludes_both_dock_producers_by_name(tmp_path):
    include = _navigation_launch_include(_actions_for(_module(tmp_path), 'true', 'false'))
    enabled = _enabled_producers_arg(include)
    assert enabled == 'navigate_to_pose,navigate_through_poses', (
        f'use_docking:=false must exclude dock_robot and undock_robot by name, got {enabled!r}')


def test_sim_with_docking_enables_all_four_producers(tmp_path):
    include = _navigation_launch_include(_actions_for(_module(tmp_path), 'true', 'true'))
    enabled = _enabled_producers_arg(include)
    assert enabled == 'navigate_to_pose,navigate_through_poses,dock_robot,undock_robot', (
        f'use_docking:=true must enable all four producers, got {enabled!r}')


def test_real_nav_only_excludes_both_dock_producers_by_name(tmp_path):
    real_map = tmp_path / 'real_map.yaml'
    real_map.write_text('')
    include = _navigation_launch_include(
        _actions_for(_module(tmp_path), 'false', 'false', str(real_map)))
    enabled = _enabled_producers_arg(include)
    assert enabled == 'navigate_to_pose,navigate_through_poses', (
        f'use_docking:=false must exclude dock_robot and undock_robot by name, got {enabled!r}')
