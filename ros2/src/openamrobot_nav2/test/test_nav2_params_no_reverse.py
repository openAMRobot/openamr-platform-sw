"""
No linear.x reverse is offered anywhere in the shipped nav2_params.yaml.

D-19 (platform-sw#124, Alex's 8 Nov ruling, option a): "reverse and
automatic undock are not offered in Build 1... The catch-all does not
make reverse fail-closed because rear sectors are masked, so reverse is
not offered." panthera-momagdii's #90 review made explicit this is an
implementation gap, not an open question, and named three more places it
was still offered:

1. backup/drive_on_heading/assisted_teleop are each their own directly-
   callable behavior_server action, independent of whatever the BT does
   (the BT-recovery-only fix is navigate_to_pose_no_backup.xml) -
   drive_on_heading accepts a negative target/speed, assisted_teleop
   forwards whatever linear.x a teleop operator sends. Removed from
   behavior_plugins entirely.
2. DWB's own min_vel_x was -0.05 ("small reverse only when stuck,
   collision_monitor guards the rear") - the rear sectors are masked
   (scan_body_filter), so that claim does not hold. Clamped to 0.0.
3. velocity_smoother's min_velocity linear.x was -0.20, a final backstop
   that still let a reverse command from any upstream source through.
   Clamped to 0.0.

Statically parses the shipped YAML, no ROS graph needed. See
test_navigation_launch_behavior_server_direct_actions_absent.py for the
live proof that backup/drive_on_heading/assisted_teleop never advertise
as action servers.
"""
import os

import yaml

_PARAMS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'nav2_params.yaml')

_REMOVED_BEHAVIOR_PLUGINS = ('backup', 'drive_on_heading', 'assisted_teleop')


def _params():
    with open(_PARAMS_FILE) as f:
        return yaml.safe_load(f)


def test_behavior_plugins_excludes_every_reverse_capable_interface():
    plugins = _params()['behavior_server']['ros__parameters']['behavior_plugins']
    for name in _REMOVED_BEHAVIOR_PLUGINS:
        assert name not in plugins, (
            f'behavior_plugins still lists {name!r} ({plugins}) - D-19 '
            '(platform-sw#124): this is a directly-callable action, reverse-'
            'capable while registered, independent of the BT.')
    for name in ('spin', 'wait'):
        assert name in plugins, (
            f'behavior_plugins lost {name!r} ({plugins}) - spin/wait do not '
            'reverse the chassis and should stay registered.')


def test_dwb_followpath_min_vel_x_is_not_negative():
    params = _params()['controller_server']['ros__parameters']['FollowPath']
    min_vel_x = params['min_vel_x']
    # Exactly 0.0, not merely >= 0.0 (panthera-momagdii's #90 review, P2):
    # a small positive value would pass a looser check too, but there is no
    # legitimate reason for min_vel_x to be anything other than exactly
    # clamped - forward motion is max_vel_x's job.
    assert min_vel_x == 0.0, (
        f"FollowPath's min_vel_x is {min_vel_x}, expected exactly 0.0 - D-19 "
        '(platform-sw#124): any nonzero-negative value lets DWB itself command '
        "a blind reverse during ordinary path following, and collision_monitor's "
        "rear sectors are masked (panthera-momagdii's #90 review), so it cannot "
        'see behind the robot to guard it.')


def test_velocity_smoother_min_velocity_linear_x_is_not_negative():
    min_velocity = _params()['velocity_smoother']['ros__parameters']['min_velocity']
    # Exactly 0.0, same reasoning as min_vel_x above (panthera-momagdii's #90
    # review, P2).
    assert min_velocity[0] == 0.0, (
        f'velocity_smoother min_velocity linear.x is {min_velocity[0]}, expected '
        'exactly 0.0 - D-19 (platform-sw#124): this is the final backstop '
        'before the real /cmd_vel, and it must not let a reverse command '
        'through from any upstream source either.')


def test_bt_xml_paths_are_top_level_bt_navigator_parameters():
    # panthera-momagdii's #90 review, P1: Jazzy's NavigateToPoseNavigator/
    # NavigateThroughPosesNavigator only ever read default_nav_to_pose_bt_xml/
    # default_nav_through_poses_bt_xml at the TOP LEVEL of
    # bt_navigator.ros__parameters (nav2_bt_navigator/src/navigators/
    # navigate_to_pose.cpp, navigate_through_poses.cpp) - nested inside the
    # navigate_to_pose:/navigate_through_poses: sub-blocks, Jazzy's
    # automatically_declare_parameters_from_overrides(true) still lets them
    # exist as declared parameters (so a naive parameter-read check can pass
    # even when the real navigator never looks at them), but the navigators
    # themselves silently fall back to the stock trees, which contain BackUp.
    params = _params()['bt_navigator']['ros__parameters']
    assert 'default_nav_to_pose_bt_xml' in params, (
        'default_nav_to_pose_bt_xml must be a TOP-LEVEL bt_navigator.ros__'
        'parameters key, not nested inside navigate_to_pose: - Jazzy only '
        'reads it there')
    assert 'default_nav_through_poses_bt_xml' in params, (
        'default_nav_through_poses_bt_xml must be a TOP-LEVEL bt_navigator.'
        'ros__parameters key, not nested inside navigate_through_poses: - '
        'Jazzy only reads it there')
    assert 'default_nav_to_pose_bt_xml' not in params.get('navigate_to_pose', {}), (
        'default_nav_to_pose_bt_xml is still ALSO nested inside navigate_to_pose: '
        '- remove the nested copy entirely, it is declared-but-unused there and '
        'invites exactly this confusion again')
    assert 'default_nav_through_poses_bt_xml' not in params.get(
        'navigate_through_poses', {}), (
        'default_nav_through_poses_bt_xml is still ALSO nested inside '
        'navigate_through_poses: - remove the nested copy entirely')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
