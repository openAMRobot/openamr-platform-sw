# SPDX-License-Identifier: MIT
"""Sim + navigation bringup: Gazebo + URDF + bridge + Nav2 + SLAM + RViz.

NO docking, NO AprilTag pipeline. Use this when you want a navigating
robot in simulation but do not need the dock pipeline — e.g. for tuning
the controller, exploring with `2D Goal Pose` from RViz, or testing
SLAM mapping.

Composition (each piece comes from its own sibling package):

  openamrobot_gazebo/gz_simulator.launch.py  -> Gazebo Harmonic + URDF + ROS<->gz bridge + robot_state_publisher
  openamrobot_nav2/nav2_bringup.launch.py    -> Nav2 + SLAM Toolbox (4 s delay so /scan + /tf exist)
  rviz2                                      -> nav2_default.rviz layout from openamrobot_nav2

Launch arguments:
  world         path to a .sdf (default: walled_world.sdf, no dock)
  spawn_x/y/yaw spawn pose in the world frame (default 0,0,0)
  use_sim_time  drive ROS time from /clock (default True)
  use_rviz      open RViz with the Nav2 layout (default True)

For the full docking simulation (Gazebo + Nav2 + AprilTag + 4-phase
sequencer), launch `openamrobot_docking docking_sim.launch.py` instead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('openamrobot_gazebo')
    pkg_nav2 = get_package_share_directory('openamrobot_nav2')

    gazebo_launch = os.path.join(pkg_gazebo, 'launch', 'gz_simulator.launch.py')
    nav2_launch = os.path.join(pkg_nav2, 'launch', 'nav2_bringup.launch.py')
    rviz_config = os.path.join(pkg_nav2, 'rviz', 'nav2_default.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Path to a .sdf world file. Empty = walled_world.sdf shipped by openamrobot_gazebo.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Robot spawn X in the world frame (m).'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Robot spawn Y in the world frame (m).'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Robot spawn yaw in the world frame (rad).'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use the /clock topic published by Gazebo.'),
        DeclareLaunchArgument('use_rviz', default_value='True',
                              description='Open RViz with the Nav2 layout.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
