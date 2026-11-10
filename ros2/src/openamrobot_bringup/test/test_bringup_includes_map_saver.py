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

"""bringup.launch.py must include map_saver_launch.py so /map_saver/save_map is up."""

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

    # launch_setup() does real file I/O on nav2_params.yaml (map injection) - give it a
    # minimal but valid fake share tree instead of mocking that logic away.
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


def _actions_for(module, sim, map_file=''):
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': sim, 'use_rviz': 'false', 'gui': 'false',
        'use_camera': 'true', 'use_docking': 'false', 'map': map_file,
    })
    return module.launch_setup(context)


def _includes_map_saver(actions):
    return [
        a for a in actions
        if isinstance(a, IncludeLaunchDescription)
        and _location(a).endswith(os.path.join('launch', 'map_saver_launch.py'))
    ]


def test_sim_profile_includes_map_saver(tmp_path):
    module = _module(tmp_path)
    matches = _includes_map_saver(_actions_for(module, 'true'))
    assert len(matches) == 1, (
        'sim:=true must include map_saver_launch.py so /map_saver/save_map is '
        f'available; found {len(matches)} matching includes.')


def test_real_profile_includes_map_saver(tmp_path):
    module = _module(tmp_path)
    real_map = tmp_path / 'real_map.yaml'
    real_map.write_text('')
    matches = _includes_map_saver(_actions_for(module, 'false', str(real_map)))
    assert len(matches) == 1, (
        'sim:=false must include map_saver_launch.py so /map_saver/save_map is '
        f'available; found {len(matches)} matching includes.')
