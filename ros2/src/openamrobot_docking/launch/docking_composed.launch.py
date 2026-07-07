"""
Docking stack for the COMPOSED vision pipeline.

Same as docking_real.launch.py BUT without its own AprilTag: the composed vision container
(apriltag_composed.launch.py) already provides camera + apriltag intra-process, so launching
apriltag.launch.yml here would double-start the detector and fight libcamera. `use_apriltag_gate`
is therefore False — the composed apriltag is always-on and there is no SetBool image gate to flip.

Starts:
  - detected_dock_pose_publisher  — map -> charging_dock_tag_1 TF -> /detected_dock_pose.
  - dock_trigger (Python)         — docking sequence + /goal_pose -> /goal_pose_nav undock gate.

Usually included by openamrobot_bringup/bringup_composed.launch.py, not run alone.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_docking')
    trigger_params = os.path.join(pkg, 'config', 'dock_trigger.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real robot uses the real clock (false).'),

        # TF (map -> dock tag) -> /detected_dock_pose at 10 Hz.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'detected_dock_pose_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # dock_trigger: docking sequence + the /goal_pose -> /goal_pose_nav undock gate.
        # use_apriltag_gate=False: the composed apriltag runs always-on (no SetBool gate).
        Node(
            package='openamrobot_docking', executable='dock_trigger.py',
            name='dock_trigger',
            parameters=[trigger_params, {'use_sim_time': use_sim_time,
                                         'obstacle_scan_forward_angle': 3.14159,
                                         'use_apriltag_gate': False}],
            output='screen'),
    ])
