"""
H-day legacy profile: reproduces the release 001 rig, nav-only, with commissioning limits.

One command for 16 Nov 2026:

  ros2 launch openamrobot_bringup hday_legacy.launch.py map:=/path/to/real_map.yaml

What it starts (and nothing else):
  * real_bringup.launch.py with lidar_model:=a1 and use_camera:=false: Teensy micro-ROS agent,
    RPLIDAR A1, scan body filter, EKF, and the Gate A static transforms (the legacy
    description: base_link parent, lidar rotated 180 deg, identity IMU).
  * the plain goal relay /goal_pose -> /goal_pose_nav (exactly one forwarder).
  * Nav2 localization and navigation with use_sim_time:=false and a temporary params file:
    nav2_params.yaml + config/hday_commissioning_limits.yaml (0.20 m/s, 1.0 rad/s).

What it never starts: docking (no docking server, no dock_trigger, no AprilTag), bag replay,
the legacy UI move_base launch (openamr_ui_package move_base_launch.py, also reachable through
physnode_launch.py, mapping_launch.py and navigation_launch.py of the UI package), the 2.0
sensors, the micro-ROS UDP transport, robot_state_publisher. It publishes no /cmd_vel itself.

The 0.20 m/s and 1.0 rad/s limits are a commissioning limit for this rig. The product speed
ceiling stays 0.5 m/s; nav2_params.yaml is not modified.

Expected merge points once the open PRs land (this file keeps its own copy until then):
  * #90 / #129: bringup.launch.py and real_bringup.launch.py gain imu_topic (EKF input). The
    Gate A Teensy publishes /imu/data, so after #129 this profile must pass
    imu_topic:=/imu/data (see IMU_TOPIC below; it is only passed if real_bringup declares it).
  * #90: navigation_launch.py adds the twist_mux chain and the stop gate; the nav2 include
    below needs no change, but check its launch arguments.
  * Once bringup.launch.py accepts a params overrides argument, replace _params_for_hday()
    with that argument and make this file a thin wrapper.
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
import yaml

# The Gate A (release 001) Teensy publishes /imu/data. Passed to real_bringup only when it
# declares imu_topic (#129).
IMU_TOPIC = '/imu/data'

# Product ceiling (P-03 rev20.6). The commissioning limits must stay below it.
PRODUCT_CEILING_MPS = 0.5
COMMISSIONING_LINEAR_MPS = 0.20
COMMISSIONING_ANGULAR_RADPS = 1.0


def _deep_merge(base, override):
    """Return base with override merged in recursively (override wins)."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _params_for_hday(nav2_share, bringup_share, map_file):
    """Write nav2_params.yaml + commissioning limits + map to a temp file; return its path."""
    with open(os.path.join(nav2_share, 'config', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(bringup_share, 'config', 'hday_commissioning_limits.yaml')) as f:
        limits = yaml.safe_load(f)
    params = _deep_merge(params, limits)
    map_server = params.setdefault('map_server', {}).setdefault('ros__parameters', {})
    map_server['yaml_filename'] = map_file
    # The real robot is not at the map origin at start: the operator sets the pose.
    params.setdefault('amcl', {}).setdefault('ros__parameters', {})['set_initial_pose'] = False
    fd, path = tempfile.mkstemp(prefix='nav2_params_hday_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    atexit.register(lambda p=path: os.path.exists(p) and os.remove(p))
    return path


def _src(share, rel):
    return PythonLaunchDescriptionSource(os.path.join(share, 'launch', rel))


def launch_setup(context, *args, **kwargs):
    nav2 = get_package_share_directory('openamrobot_nav2')
    bringup = get_package_share_directory('openamrobot_bringup')

    map_file = LaunchConfiguration('map').perform(context)
    if not map_file:
        raise RuntimeError('hday_legacy requires map:=/path/to/your_real_map.yaml '
                           '(the bundled sim map is never used on the real robot).')
    if not os.path.exists(map_file):
        raise RuntimeError('map file not found: ' + map_file)

    params = _params_for_hday(nav2, bringup, map_file)

    real_args = {'lidar_model': 'a1', 'use_camera': 'false'}
    real_bringup_file = os.path.join(bringup, 'launch', 'real_bringup.launch.py')
    if os.path.exists(real_bringup_file):
        with open(real_bringup_file) as f:
            if "'imu_topic'" in f.read():
                real_args['imu_topic'] = IMU_TOPIC

    return [
        IncludeLaunchDescription(
            _src(bringup, 'real_bringup.launch.py'), launch_arguments=real_args.items()),
        # Nav-only: exactly one goal forwarder, the plain relay. No docking.
        IncludeLaunchDescription(_src(bringup, 'goal_relay.launch.py')),
        IncludeLaunchDescription(
            _src(nav2, 'localization_launch.py'),
            launch_arguments={'map': '', 'params_file': params,
                              'use_sim_time': 'false'}.items()),
        IncludeLaunchDescription(
            _src(nav2, 'navigation_launch.py'),
            launch_arguments={'params_file': params, 'use_sim_time': 'false'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML for AMCL. Required: there is no default map on the real robot.'),
        OpaqueFunction(function=launch_setup),
    ])
