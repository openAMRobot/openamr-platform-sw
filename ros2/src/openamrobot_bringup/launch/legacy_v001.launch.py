"""
legacy_v001 profile: the OpenAMRobot 2.0 base software on the v0.0.1 platform (release 001 rig).

One command:

  ros2 launch openamrobot_bringup legacy_v001.launch.py map:=/path/to/real_map.yaml

The v0.0.1 platform keeps its Teensy 4.0 firmware (linorobot2 overlay, platform-fw #33): it
reads the external encoders, drives the existing motors and drivers, subscribes to /cmd_vel
(geometry_msgs/Twist) and publishes /odom/unfiltered and /imu/data over the micro-ROS serial
agent. This profile adapts the host side to that firmware with launch and config only. It
reuses the same Nav2 launch (openamrobot_nav2 navigation_launch.py and localization_launch.py)
as every other profile and changes no default of any existing launch file.

What it starts:
  * openamrobot_drivers drivers.launch.py with lidar_model:=a1 (micro-ROS agent, RPLIDAR A1).
  * openamrobot_perception scan_body_filter.launch.py (/scan -> /scan_filtered).
  * robot_localization EKF from bringup config/ekf.yaml with imu0 set to IMU_TOPIC, the topic
    the deployed Teensy build publishes. The EKF is the only odom -> base_link publisher.
  * The measured static transforms of the v0.0.1 unit (LEGACY_STATIC_TFS). This profile
    owns them so that the 2.0 TF cleanup of real_bringup.launch.py does not move this rig.
  * The plain goal relay /goal_pose -> /goal_pose_nav (the only goal forwarder; no docking).
  * Nav2 localization and navigation, use_sim_time:=false, with a temporary parameter file:
    nav2_params.yaml merged with config/legacy_v001/nav2_overrides.yaml and the map.
  * navigation_launch.py with use_waypoint_follower:=false (Build 1: follow_waypoints is not
    cancelled by the stop path) and the legacy_v001 status profile.

What it never starts: camera, docking (no docking server, dock_trigger or AprilTag), the
waypoint follower, robot_state_publisher, bag replay, any legacy UI move_base launch. It
publishes no velocity command itself.

When platform-sw #90 lands, navigation_launch.py declares enabled_producers. This profile
already passes ENABLED_PRODUCERS (no docking producers); until then the argument is unused.
See docs/legacy-16nov/compatibility.md for the full interface table.
"""
import atexit
import copy
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PROFILE = 'legacy_v001'

# Deployed Teensy build: no magnetometer is configured, so the linorobot2 base defines
# USE_FAKE_MAG and the firmware publishes sensor_msgs/Imu on imu/data (not imu/data_raw).
IMU_TOPIC = '/imu/data'

LIDAR_MODEL = 'a1'

# Producers the stop gate of platform-sw #90 must see idle before Resume (P-03 item 30 A1).
# Docking is not started in this profile, so dock_robot and undock_robot are excluded by name.
# follow_waypoints is not a producer here because the waypoint follower is not started.
ENABLED_PRODUCERS = ('navigate_to_pose', 'navigate_through_poses')

# Measured static transforms of the v0.0.1 unit (Gate A), the same values real_bringup.launch.py
# carries today: (name, parent, child, x, y, z, roll, pitch, yaw), metres and radians.
# The firmware odometry child frame is base_footprint and the EKF publishes odom -> base_link,
# so base_footprint hangs off base_link here (identity) and only one odom -> base publisher runs.
LEGACY_STATIC_TFS = (
    ('base_to_lidar', 'base_link', 'lidar_link', 0.335, 0.0, 0.18, 0.0, 0.0, 3.14159),
    ('base_to_imu', 'base_link', 'imu_link', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ('base_to_footprint', 'base_link', 'base_footprint', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)


def deep_merge(base, override):
    """Return base with override merged in recursively (override wins)."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def legacy_nav2_params(nav2_params_path, overrides_path, map_file):
    """Return nav2_params.yaml merged with the legacy_v001 overrides and the map file."""
    with open(nav2_params_path) as f:
        params = yaml.safe_load(f)
    with open(overrides_path) as f:
        overrides = yaml.safe_load(f)
    params = deep_merge(params, overrides)
    params.setdefault('map_server', {}).setdefault('ros__parameters', {})[
        'yaml_filename'] = map_file
    return params


def _write_temp_params(params):
    fd, path = tempfile.mkstemp(prefix='nav2_params_legacy_v001_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    atexit.register(lambda p=path: os.path.exists(p) and os.remove(p))
    return path


def _static_tf(name, parent, child, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', parent, '--child-frame-id', child],
        output='screen')


def _src(share, rel):
    return PythonLaunchDescriptionSource(os.path.join(share, 'launch', rel))


def launch_setup(context, *args, **kwargs):
    nav2 = get_package_share_directory('openamrobot_nav2')
    bringup = get_package_share_directory('openamrobot_bringup')
    drivers = get_package_share_directory('openamrobot_drivers')
    perception = get_package_share_directory('openamrobot_perception')

    map_file = LaunchConfiguration('map').perform(context)
    if not map_file:
        raise RuntimeError('legacy_v001 requires map:=/path/to/your_real_map.yaml '
                           '(the bundled sim map is never used on the real robot).')
    if not os.path.exists(map_file):
        raise RuntimeError('map file not found: ' + map_file)

    params = legacy_nav2_params(
        os.path.join(nav2, 'config', 'nav2_params.yaml'),
        os.path.join(bringup, 'config', PROFILE, 'nav2_overrides.yaml'),
        map_file)
    params_file = _write_temp_params(params)
    status_profile = os.path.join(bringup, 'config', PROFILE, 'navigation_status_profile.yaml')

    actions = [
        IncludeLaunchDescription(
            _src(drivers, 'drivers.launch.py'),
            launch_arguments={'lidar_model': LIDAR_MODEL}.items()),
        IncludeLaunchDescription(_src(perception, 'scan_body_filter.launch.py')),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[os.path.join(bringup, 'config', 'ekf.yaml'), {'imu0': IMU_TOPIC}],
            remappings=[('odometry/filtered', '/odom')],
            output='screen'),
    ]
    actions += [_static_tf(*tf) for tf in LEGACY_STATIC_TFS]
    actions += [
        IncludeLaunchDescription(_src(bringup, 'goal_relay.launch.py')),
        IncludeLaunchDescription(
            _src(nav2, 'localization_launch.py'),
            launch_arguments={'map': '', 'params_file': params_file,
                              'use_sim_time': 'false'}.items()),
        IncludeLaunchDescription(
            _src(nav2, 'navigation_launch.py'),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',
                'use_waypoint_follower': 'false',
                'status_profile_path': status_profile,
                'enabled_producers': ','.join(ENABLED_PRODUCERS),
            }.items()),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML for AMCL. Required: there is no default map on the real robot.'),
        OpaqueFunction(function=launch_setup),
    ])
