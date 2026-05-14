# SPDX-License-Identifier: Apache-2.0
"""Launch Gazebo Harmonic with the OpenAMRobot platform.

Accepted launch arguments:
  world         — full path to a .sdf world (default: walled_world.sdf
                  shipped by this package).
  spawn_x/y/yaw — robot spawn pose in the world frame (default: 0, 0, 0).
  use_sim_time  — drive ROS time from /clock (default: True).

Downstream packages compose this launch and override `world` to drop the
robot into a scenario-specific environment. See
`openamrobot_docking/launch/docking_sim.launch.py` for an example.
"""
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_nodes(context):
    description_dir = get_package_share_directory('openamrobot_description')
    gazebo_dir = get_package_share_directory('openamrobot_gazebo')

    world = LaunchConfiguration('world').perform(context)
    if not world:
        world = os.path.join(gazebo_dir, 'worlds', 'walled_world.sdf')

    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    xacro_file = os.path.join(description_dir, 'urdf', 'robo_urdf.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(gazebo_dir, 'worlds'),
            str(Path(description_dir).parent.resolve()),
        ]),
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items(),
    )

    # The URDF root (base_footprint) is at ground level. base_joint lifts
    # base_link by 0.053467 m so wheel centres sit at z = wheel_radius = 0.10 m.
    # Spawn z must therefore be 0.0 — non-zero z makes the drive wheels float.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'openamrobot',
            '-topic', '/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.0',
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    return [
        gz_resource_path,
        gz_sim,
        bridge,
        spawn_entity,
        start_robot_state_publisher_cmd,
        joint_state_publisher,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='',
            description=(
                'Full path to a .sdf world file. If empty, defaults to '
                'walled_world.sdf shipped by openamrobot_gazebo.'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Robot spawn X in the world frame (metres).',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Robot spawn Y in the world frame (metres).',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Robot spawn yaw in the world frame (radians).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true.',
        ),
        OpaqueFunction(function=_build_nodes),
    ])
