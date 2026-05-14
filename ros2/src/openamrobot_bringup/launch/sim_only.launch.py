# SPDX-License-Identifier: MIT
"""Sim-only bringup: Gazebo + URDF + bridge. No navigation, no docking.

Thin wrapper around openamrobot_gazebo/gz_simulator.launch.py — exists in
this package for discoverability ("the bringup launches live under
openamrobot_bringup") and so users have a single launch namespace to
remember regardless of how the lower-level packages get reorganised later.

Use this when you want to:
  - Drive the robot manually with teleop (`teleop_twist_keyboard` -> /cmd_vel)
  - Validate the Gazebo plugins, friction, mesh collisions
  - Iterate on the URDF or sensors without paying for Nav2 startup

Launch arguments are passed through to gz_simulator unchanged:
  world, spawn_x, spawn_y, spawn_yaw, use_sim_time.

Default world: walled_world.sdf (empty 10x10 m arena).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('openamrobot_gazebo'),
        'launch', 'gz_simulator.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='',
                              description='Path to a .sdf world; empty = walled_world.sdf.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
