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
5. controller_server has two progress checkers (Proposed, 23 Nov 2026, platform-sw #192), so every FollowPath names
   one. Only the station stages use the one that counts rotation; the general one and its values are unchanged.
"""
import os
import xml.etree.ElementTree as ET

import yaml

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
PARAMS = os.path.join(PACKAGE, 'config', 'nav2_params.yaml')
TREES = os.path.join(PACKAGE, 'behavior_trees')
# Tree fold of #90 and #164 (platform owner, 23 Nov 2026): a relative placeholder, resolved to
# an absolute path at launch time by navigation_launch.py's own RewrittenYaml param_substitutions
# (nav2_params.yaml's own comment on this key explains why - a $(find-pkg-share ...) substitution
# here was never run against a real launch).
SHARE_PREFIX = 'behavior_trees/'

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



# navigation2 1.3.14 controller_server.cpp findProgressCheckerId: with more than one progress checker an empty
# progress_checker_id ends the goal with "Failed to find progress checker name".
STATION_PROGRESS = 'station_progress_checker'
GENERAL_PROGRESS = 'progress_checker'


def progress_ids(root):
    return [(f.get('controller_id'), f.get('progress_checker_id')) for f in root.iter('FollowPath')]


def test_every_follow_path_names_a_configured_progress_checker():
    checkers = controller()['progress_checker_plugins']
    assert checkers == [GENERAL_PROGRESS, STATION_PROGRESS]
    for name in tree_files():
        ids = progress_ids(tree(name))
        assert ids, f'{name}: no FollowPath'
        for controller_id, checker in ids:
            assert checker in checkers, f'{name}: FollowPath {controller_id} without a configured progress_checker_id'


def test_only_the_station_stages_use_the_station_progress_checker():
    for name in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):
        assert {c for _, c in progress_ids(tree(name))} == {GENERAL_PROGRESS}, name
    sequence = tree('navigate_to_station.xml').find('BehaviorTree/Sequence')
    assert {c for _, c in progress_ids(sequence[0])} == {GENERAL_PROGRESS}, 'stage 1 is ordinary navigation'
    assert progress_ids(sequence[1]) == [('StationApproach', STATION_PROGRESS)]
    assert progress_ids(tree('navigate_to_station_retreat.xml')) == [('StationRetreat', STATION_PROGRESS)]


def test_the_general_progress_checker_is_unchanged():
    general = controller()[GENERAL_PROGRESS]
    assert general == {'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.05,
                       'movement_time_allowance': 12.0}


def test_the_station_progress_checker_adds_rotation_and_relaxes_nothing():
    general, station = controller()[GENERAL_PROGRESS], controller()[STATION_PROGRESS]
    assert station['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert station['required_movement_radius'] == general['required_movement_radius']
    assert station['movement_time_allowance'] == general['movement_time_allowance'], 'no longer stall window'
    angle = station['required_movement_angle']
    # Standing inside the station yaw tolerance must not count as progress.
    assert angle > controller()['station_goal_checker']['yaw_goal_tolerance']
    # The slowest in-place rotation the station controller commands must be able to show progress well inside the
    # allowance, or a healthy rotation is aborted like before.
    slowest = controller()['StationApproach']['v_angular_min_in_place']
    assert angle / slowest <= station['movement_time_allowance'] / 4.0


def test_a_short_station_correction_can_show_progress_only_by_rotation():
    """The case of platform-sw #192: the station goal checker accepts within 0.025 m, so a correction can be shorter
    than the general progress radius. Such a goal cannot satisfy a translation-only checker."""
    c = controller()
    assert c['station_goal_checker']['xy_goal_tolerance'] < c[GENERAL_PROGRESS]['required_movement_radius']
    assert 'required_movement_angle' not in c[GENERAL_PROGRESS]
    assert c[STATION_PROGRESS]['required_movement_angle'] > 0.0
