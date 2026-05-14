# SPDX-License-Identifier: MIT
"""Launch the Nav2 stack + SLAM Toolbox for the OpenAMRobot platform.

This is the navigation-only bringup: it does NOT spawn Gazebo or the robot.
Compose it with `openamrobot_gazebo/launch/gz_simulator.launch.py` (or your
real-robot driver launch) to get a full stack.

SLAM Toolbox is started after a 4 s delay so the `/scan` topic and odom TF
are already flowing — slam_toolbox crashes if /tf is missing when it starts.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_nav2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the /clock topic published by Gazebo.',
        ),

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
                    ),
                    launch_arguments={
                        'slam_params_file': slam_params,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py'),
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_localization': 'False',
                'map': '',
                'params_file': nav2_params,
            }.items(),
        ),
    ])
