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
Regression guard for issue #74 / PR #97 (the OTHER sim entry point).

PR #97 switched openamrobot_nav2/sim_bringup_launch.py's scan body filter from the
laser_filters truncation chain to openamrobot_perception/scan_body_filter, and deleted the
laser_filters config it used (openamrobot_nav2/config/scan_body_filter.yaml). It missed that
openamrobot_bringup/bringup.launch.py's sim branch builds its OWN scan_body_filter Node
in-line rather than including sim_bringup_launch.py, and that Node was still pointed at
laser_filters + the now-deleted config - a silent regression: launching
`bringup.launch.py sim:=true` (this package's own README calls it "the single entry
point"/"ONE command for the whole simulation") would start a body-filter node with a missing
parameters file, almost certainly failing it at startup and leaving sim with NO body
filtering - reproducing issue #74's original symptom (or worse) through a path the #97
testing never exercised, since that testing went through sim_bringup_launch.py directly.

This introspects bringup.launch.py the same way test_real_bringup_launch.py and
test_drivers_launch.py already do in this package: no ROS processes started, no Gazebo, just
the launch description the sim branch builds.
"""
import importlib.util
import os

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'bringup.launch.py')
# openamrobot_nav2's real source dir (sibling package) - launch_setup's map-injection step
# (_params_with_map) opens nav2_params.yaml and checks maps/my_map.yaml really exist, so this
# needs to resolve to files that are actually on disk, not a fake path.
REAL_NAV2_SHARE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'openamrobot_nav2')


def _module():
    spec = importlib.util.spec_from_file_location('bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def fake_share(pkg):
        # Only openamrobot_nav2's files are actually opened by launch_setup (see above); every
        # other package here is just used to build an IncludeLaunchDescription source, which
        # this test never expands, so a fake path is fine for those.
        return REAL_NAV2_SHARE if pkg == 'openamrobot_nav2' else os.path.join('/fake-share', pkg)
    module.get_package_share_directory = fake_share
    return module


def _sim_context():
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': 'true', 'use_rviz': 'false', 'gui': 'false',
        'use_camera': 'true', 'use_docking': 'true', 'map': '',
    })
    return context


def _sim_scan_filter_node():
    # Node.name isn't a plain string to compare against (it's resolved through substitutions),
    # so - same as this package's own test_drivers_launch.py - identify the node by its
    # node_package/node_executable properties instead, which stay plain strings as constructed.
    module = _module()
    actions = module.launch_setup(_sim_context())
    nodes = [a for a in actions if isinstance(a, Node)]
    matches = [n for n in nodes if getattr(n, 'node_package', None) == 'openamrobot_perception']
    assert nodes, (
        f'expected at least one Node action in the sim branch, found none among: '
        f'{[type(a).__name__ for a in actions]}')
    assert len(matches) == 1, (
        f'expected exactly one openamrobot_perception Node (the scan body filter), found '
        f'{len(matches)} among node packages: {[getattr(n, "node_package", None) for n in nodes]}')
    return matches[0]


def test_sim_scan_body_filter_is_the_shared_perception_node_not_laser_filters():
    node = _sim_scan_filter_node()
    assert node.node_package == 'openamrobot_perception'
    assert node.node_executable == 'scan_body_filter'


def test_sim_scan_body_filter_config_file_exists_on_disk():
    node = _sim_scan_filter_node()
    context = _sim_context()
    evaluated = evaluate_parameters(context, node._Node__parameters)
    # A params-file entry comes back as a PosixPath (not a plain str) in this launch_ros
    # version; a dict entry (e.g. {'use_sim_time': True}) is excluded by also rejecting dicts.
    path_entries = [e for e in evaluated if isinstance(e, os.PathLike) and not isinstance(e, dict)]
    assert len(path_entries) == 1, (
        f'expected exactly one params-file entry, got: {evaluated!r}')
    path_str = os.fspath(path_entries[0])
    assert os.path.basename(path_str) == 'scan_body_filter_sim.yaml', (
        'bringup.launch.py sim branch is not pointed at the sim config '
        '(regression guard for issue #74 / PR #97)')
    assert os.path.exists(path_str), (
        'bringup.launch.py sim branch points at a scan_body_filter config that does not '
        'exist - this is exactly the #97 regression this test guards against')
