# SPDX-License-Identifier: MIT
"""Static checks of the station approach (Proposed, 17 Nov 2026) in nav2_params.yaml and behavior_trees/.

Pure Python: no ROS, no Gazebo. It proves the configuration is consistent, not that the robot stops where it should;
that evidence is the whole-robot Baseline 1 run in openamr-upperbody-sw.

1. controller_server has two goal checkers, so every FollowPath of every tree bt_navigator can load must name one
   (an empty goal_checker_id is refused when there is more than one).
2. The default trees are the files of this package and keep the general checker; the global tolerances stay 0.35.
3. The station tree runs the default navigation first, then one StationPlanner path and the StationApproach
   controller with station_goal_checker.
4. The station tolerances stay inside the arm-reach budget minus the localization allowance, and the station
   controller stays inside the commissioning speed limits.
"""
import os
import xml.etree.ElementTree as ET

import yaml

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
PARAMS = os.path.join(PACKAGE, 'config', 'nav2_params.yaml')
TREES = os.path.join(PACKAGE, 'behavior_trees')
SHARE_PREFIX = '$(find-pkg-share openamrobot_nav2)/behavior_trees/'

# openamr-upperbody-sw docs/integration/station_approach.md: every base pose error up to 0.04 m and 0.08 rad keeps the
# Build 1 approach, grasp and lift poses reachable; 0.015 m and 0.03 rad are left to localization.
REACH_XY_M, REACH_YAW_RAD = 0.04, 0.08
LOCALIZATION_XY_M, LOCALIZATION_YAW_RAD = 0.015, 0.03
COMMISSIONING_V, COMMISSIONING_W = 0.20, 0.5


def params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def controller():
    return params()['controller_server']['ros__parameters']


def tree(name):
    return ET.parse(os.path.join(TREES, name)).getroot()


def tree_files():
    return sorted(f for f in os.listdir(TREES) if f.endswith('.xml'))


def test_every_follow_path_names_a_configured_goal_checker():
    checkers = controller()['goal_checker_plugins']
    assert len(checkers) > 1, 'this test exists because there are two goal checkers'
    for name in tree_files():
        nodes = tree(name).iter('FollowPath')
        found = 0
        for node in nodes:
            found += 1
            assert node.get('goal_checker_id') in checkers, f'{name}: FollowPath without a configured goal_checker_id'
        assert found, f'{name}: no FollowPath'


def test_the_default_trees_are_this_packages_files_and_use_the_general_checker():
    bt = params()['bt_navigator']['ros__parameters']
    for key, expected in (('default_nav_to_pose_bt_xml', 'navigate_to_pose.xml'),
                          ('default_nav_through_poses_bt_xml', 'navigate_through_poses.xml')):
        assert bt[key] == SHARE_PREFIX + expected
        ids = {n.get('goal_checker_id') for n in tree(expected).iter('FollowPath')}
        assert ids == {'general_goal_checker'}, f'{expected}: {ids}'
        controllers = {n.get('controller_id') for n in tree(expected).iter('FollowPath')}
        assert controllers == {'{selected_controller}'}, f'{expected}: the stock controller selection is kept'


def test_the_global_tolerances_are_unchanged():
    general = controller()['general_goal_checker']
    assert (general['xy_goal_tolerance'], general['yaw_goal_tolerance']) == (0.35, 0.35)
    assert controller()['FollowPath']['xy_goal_tolerance'] == 0.35


def test_the_station_tree_navigates_first_then_approaches_with_its_own_planner_controller_and_checker():
    root = tree('navigate_to_station.xml')
    sequence = root.find('BehaviorTree/Sequence')
    stages = [child.get('name') for child in sequence]
    assert stages == ['NavigateRecovery', 'StationApproach']
    coarse = {n.get('goal_checker_id') for n in sequence[0].iter('FollowPath')}
    assert coarse == {'general_goal_checker'}
    plans = list(sequence[1].iter('ComputePathToPose'))
    follows = list(sequence[1].iter('FollowPath'))
    assert [p.get('planner_id') for p in plans] == ['StationPlanner']
    assert [(f.get('controller_id'), f.get('goal_checker_id')) for f in follows] == [('StationApproach', 'station_goal_checker')]
    assert plans[0].get('path') == follows[0].get('path') == '{station_path}'


def test_the_station_plugins_exist_and_are_the_intended_ones():
    c = controller()
    assert 'StationApproach' in c['controller_plugins']
    assert c['StationApproach']['plugin'] == 'nav2_graceful_controller::GracefulController'
    planner = params()['planner_server']['ros__parameters']
    assert 'StationPlanner' in planner['planner_plugins']
    assert planner['StationPlanner']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert planner['StationPlanner']['tolerance'] == 0.0, 'the path must end at the goal, not near it'


def test_the_station_tolerances_stay_inside_the_reach_budget():
    station = controller()['station_goal_checker']
    assert station['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert 0.0 < station['xy_goal_tolerance'] <= REACH_XY_M - LOCALIZATION_XY_M
    assert 0.0 < station['yaw_goal_tolerance'] <= REACH_YAW_RAD - LOCALIZATION_YAW_RAD


def test_the_station_controller_stays_inside_the_commissioning_limits():
    approach = controller()['StationApproach']
    assert 0.0 < approach['v_linear_min'] < approach['v_linear_max'] <= COMMISSIONING_V
    assert approach['v_angular_max'] <= COMMISSIONING_W
    assert approach['v_angular_min_in_place'] <= approach['v_angular_max']


def test_amcl_updates_within_the_station_approach():
    """AMCL corrects the map pose during the last part of the station approach: it updates at least every 0.05 m and
    0.05 rad of odometry, well inside the 0.35 m and 0.35 rad left after the coarse navigation."""
    amcl = params()['amcl']['ros__parameters']
    assert amcl['update_min_d'] <= 0.05
    assert amcl['update_min_a'] <= 0.05
