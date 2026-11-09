"""
OpenAMRobot 2.0 real-robot bring-up (base controller NUCLEO-H723ZG), next to the legacy profile.

``real_bringup.launch.py`` (legacy Teensy, existing robot) is unchanged. This profile adds:

* openamrobot_drivers ``drivers_20.launch.py``: micro-ROS agent (serial or USB CDC), RPLIDAR S3
  and the base status monitor;
* the scan body filter and the 2.0 robot description (robot_state_publisher);
* robot_localization EKF on /odom/unfiltered and /imu/data_raw (decision D-10);
* the command chain  twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel,
  with the speed ceiling 0.5 m/s and travel deceleration 1.5 m/s2 (P-03 item 30 A5, A6).
  collision_monitor and velocity_smoother have their own lifecycle manager with the bond off
  (item 30 A2). The twist_mux lock /base/command_lock comes from the base status monitor and
  stays engaged until the base is RUN and fresh and an operator calls /base/resume.

Nav2 itself is not started here. Its controller publishes cmd_vel_nav and docking publishes
cmd_vel_dock, which are the mux inputs; wiring those is the command-arbitration work of
platform-sw #90 and #125. The functional chain is not a safety function: the hardwired
E-stop is.

  ros2 launch openamrobot_bringup real_bringup_20.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    drivers = get_package_share_directory('openamrobot_drivers')
    perception = get_package_share_directory('openamrobot_perception')
    description = get_package_share_directory('openamrobot_description')
    bringup = get_package_share_directory('openamrobot_bringup')
    chain_params = os.path.join(bringup, 'config', 'cmd_chain_20.yaml')
    ekf_params = os.path.join(bringup, 'config', 'ekf_20.yaml')
    xacro_file = os.path.join(description, 'urdf', 'robo_urdf.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_port', default_value='/dev/openamr/nucleo_stlink_vcp',
            description='Serial or USB CDC device of the base controller.'),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Launches the base camera (off until the 2.0 camera launch exists).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drivers, 'launch', 'drivers_20.launch.py')),
            launch_arguments={'base_port': LaunchConfiguration('base_port')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception, 'launch', 'scan_body_filter.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception, 'launch', 'camera.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_camera'))),
        # 2.0 description: owns every static transform (base_footprint, base_link, lidar_link,
        # imu_link). No hand-written static transforms here.
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str)}]),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', '/odom')], output='screen'),
        # --- command chain ---
        Node(
            package='twist_mux', executable='twist_mux', name='twist_mux',
            parameters=[chain_params],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')], output='screen'),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', parameters=[chain_params],
            remappings=[('cmd_vel', 'cmd_vel_mux')], output='screen'),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', parameters=[chain_params], output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_cmd_chain', parameters=[chain_params], output='screen'),
    ])
