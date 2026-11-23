# SPDX-License-Identifier: MIT
"""No behaviour tree of this package reverses the base (Alex, 19 Nov 2026: no general reversing recovery; D-19).

Pure Python over behavior_trees/ and nav2_params.yaml: no ROS, no Nav2 start. It proves what the tree files say, not
how the robot moves; the velocity clamps (min_vel_x, velocity_smoother.min_velocity) are a separate check.

1. No tree has a BackUp node. BackUp only drives backwards.
2. No tree has a DriveOnHeading that can drive backwards: a negative speed or distance, or a value read from the
   blackboard that this file cannot prove forward.
3. bt_navigator's default trees are files of this package, so Nav2's stock trees (which contain BackUp) are not loaded.
4. Spin and Wait stay in each recovery round robin: only the reversing action was removed. Every tree has one,
   except the trees in NO_ROUND_ROBIN, each with its reason (PROPOSED, upperbody #117).
5. No nav2_graceful_controller plugin in controller_server has allow_backward true (Kunal's
   review of #164: this PR's own StationApproach shipped allow_backward true, contradicting
   point 1-3's own D-19 reasoning, with nothing here checking controller_server at all -
   test_baseline1_no_reverse_paths.py on platform-sw#178 covers the full reverse-path surface
   including behavior_server/DWB/velocity_smoother, which is #178's own removal scope, not
   #164's; this is the narrow part of that same check that belongs here).
"""
import os
import xml.etree.ElementTree as ET

import yaml

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
TREES = os.path.join(PACKAGE, 'behavior_trees')
PARAMS = os.path.join(PACKAGE, 'config', 'nav2_params.yaml')
# Tree fold of #90 and #164 (platform owner, 23 Nov 2026): a relative placeholder, resolved to
# an absolute path at launch time by navigation_launch.py's own RewrittenYaml param_substitutions
# (nav2_params.yaml's own comment on this key explains why - a $(find-pkg-share ...) substitution
# here was never run against a real launch).
SHARE_PREFIX = 'behavior_trees/'
# PROPOSED (upperbody #117): the forward-retreat tree is the station stage only (Wait between retries); it never had a
# stock recovery round robin, so nothing was removed from it. Any other tree without one still fails.
NO_ROUND_ROBIN = {'navigate_to_station_retreat.xml': 'station stage only, Wait 1.0 between two retries'}


def tree_files():
    return sorted(f for f in os.listdir(TREES) if f.endswith('.xml'))


def reversing_nodes(root):
    """Return a description of every node under root that can drive the base backwards."""
    found = []
    for node in root.iter('BackUp'):
        found.append(f'BackUp {dict(node.attrib)}')
    for node in root.iter('DriveOnHeading'):
        for key in ('speed', 'dist_to_travel'):
            value = node.get(key)
            if value is None:
                continue
            try:
                negative = float(value) < 0.0
            except ValueError:
                negative = True  # blackboard value: not provably forward
            if negative:
                found.append(f'DriveOnHeading {key}={value}')
    return found


def test_the_check_finds_backup_and_reversing_drive_on_heading():
    root = ET.fromstring(
        '<root><BehaviorTree ID="T"><RoundRobin>'
        '<BackUp backup_dist="0.30" backup_speed="0.15"/>'
        '<DriveOnHeading dist_to_travel="0.2" speed="-0.1"/>'
        '<DriveOnHeading dist_to_travel="{d}" speed="0.1"/>'
        '<DriveOnHeading dist_to_travel="0.2" speed="0.1"/>'
        '</RoundRobin></BehaviorTree></root>')
    assert len(reversing_nodes(root)) == 3


def test_no_tree_reverses_the_base():
    files = tree_files()
    assert files, 'no behaviour tree files found'
    for name in files:
        found = reversing_nodes(ET.parse(os.path.join(TREES, name)).getroot())
        assert not found, f'{name} reverses the base: {found} (no general reversing recovery, 19 Nov 2026)'


def test_default_trees_are_this_packages_files():
    with open(PARAMS) as f:
        bt = yaml.safe_load(f)['bt_navigator']['ros__parameters']
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        value = bt.get(key, '')
        assert value.startswith(SHARE_PREFIX), f'{key} is {value!r}: Nav2 stock trees contain BackUp'
        assert os.path.isfile(os.path.join(TREES, value[len(SHARE_PREFIX):])), value


def test_spin_and_wait_stay_in_each_recovery_round_robin():
    for name in tree_files():
        robins = list(ET.parse(os.path.join(TREES, name)).getroot().iter('RoundRobin'))
        if name in NO_ROUND_ROBIN:
            assert not robins, f'{name}: has a round robin, remove it from NO_ROUND_ROBIN'
            continue
        assert robins, f'{name}: no recovery round robin'
        for robin in robins:
            assert robin.find('Spin') is not None, f'{name}: Spin removed'
            assert robin.find('Wait') is not None, f'{name}: Wait removed'


def graceful_controllers_with_backward_allowed(controller_server):
    """Name of every nav2_graceful_controller plugin whose allow_backward is true."""
    found = []
    for name in controller_server.get('controller_plugins', []):
        values = controller_server.get(name, {}) or {}
        if 'graceful' in values.get('plugin', '').lower() and values.get('allow_backward', False):
            found.append(name)
    return found


def test_the_check_finds_a_graceful_controller_with_backward_allowed():
    controller_server = {
        'controller_plugins': ['A'],
        'A': {'plugin': 'nav2_graceful_controller::GracefulController', 'allow_backward': True},
    }
    assert graceful_controllers_with_backward_allowed(controller_server) == ['A']


def test_no_graceful_controller_allows_backward():
    with open(PARAMS) as f:
        controller_server = yaml.safe_load(f)['controller_server']['ros__parameters']
    found = graceful_controllers_with_backward_allowed(controller_server)
    assert not found, f'{found} allow(s) backward (no general reversing, D-19)'
