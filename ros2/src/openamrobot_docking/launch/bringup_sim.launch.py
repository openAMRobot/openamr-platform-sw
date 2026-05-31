# SPDX-License-Identifier: MIT
"""
One-shot bringup for the whole docking simulation.

Starts the three layers that are otherwise run in three separate terminals,
in the right order and with a delay between each so every layer is up before
the next one needs it:

    1. openamrobot_gazebo  gz_simulator.launch.py   (Gazebo + robot + ros<->gz bridge)
    2. openamrobot_nav2    sim_bringup_launch.py    (Nav2 + AMCL + map + RViz)
    3. openamrobot_docking openamrobot_docking.launch.py  (apriltag + docking sequencer)

Usage:

    ros2 launch openamrobot_docking bringup_sim.launch.py

Headless simulation without Gazebo GUI or RViz:

    ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false

Then trigger docking from any sourced terminal:

    ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once

The three individual launches still work on their own if you want to start /
restart one layer at a time (e.g. while tuning):

    ros2 launch openamrobot_gazebo  gz_simulator.launch.py
    ros2 launch openamrobot_nav2    sim_bringup_launch.py
    ros2 launch openamrobot_docking openamrobot_docking.launch.py

Startup timing is configurable if your machine needs more (or less) warm-up:

    ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=20
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_delay = LaunchConfiguration('nav2_delay')
    docking_delay = LaunchConfiguration('docking_delay')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('openamrobot_gazebo'),
            'launch', 'gz_simulator.launch.py',
        ])),
        launch_arguments={'gui': gazebo_gui}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('openamrobot_nav2'),
            'launch', 'sim_bringup_launch.py',
        ])),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('openamrobot_docking'),
            'launch', 'openamrobot_docking.launch.py',
        ])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_delay', default_value='8.0',
            description='Seconds to wait after Gazebo before starting Nav2.',
        ),
        DeclareLaunchArgument(
            'docking_delay', default_value='16.0',
            description='Seconds to wait after Gazebo before starting the docking layer '
                        '(must be > nav2_delay so AMCL has localized).',
        ),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Start Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz.',
        ),
        # Gazebo first (at t=0): it owns /clock, so everything else can run on
        # sim time once it is up.
        gazebo,
        # Nav2 + AMCL + RViz once Gazebo + the bridge are alive.
        TimerAction(period=nav2_delay, actions=[nav2]),
        # Docking layer last, after Nav2 has had time to localize on the map.
        TimerAction(period=docking_delay, actions=[docking]),
    ])
