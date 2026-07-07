"""
Real-robot docking pipeline (no simulation).

Composes the hardware-agnostic docking stack onto the REAL camera, WITHOUT any
Gazebo glue (no GZ_SIM_RESOURCE_PATH, no ros_gz_bridge camera_info bridge, no
camera_info_sync — the real camera already publishes a calibrated
/camera/camera_info, started by openamrobot_bringup/real_bringup.launch.py):

  - apriltag.launch.yml          — AprilTagNode on /camera/image_raw +
                                    /camera/camera_info -> TF to the dock tags.
  - detected_dock_pose_publisher — map -> charging_dock_tag_1 TF -> /detected_dock_pose.
  - dock_trigger (Python)        — listens on /dock_trigger (dock) and /undock_robot,
                                    AND owns /goal_pose: a navigation goal that arrives
                                    while docked undocks first, then is republished on
                                    goal_pose_forward_topic (= /goal_pose_nav). When NOT
                                    docked the goal is forwarded immediately. This is the
                                    real replacement for the placeholder
                                    `topic_tools relay /goal_pose /goal_pose_nav`.

  ros2 launch openamrobot_docking docking_real.launch.py

Prerequisites (hardware): a physical dock with the 3-tag 36h11 bundle (IDs 0/1/2),
the real tag size set in config/tags_36h11.yaml, the camera calibrated, and the dock
pose set in config/dock_trigger.yaml for the real map. See the porting plan / runbook.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
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

        # AprilTag detection on the real camera (raw image + calibrated camera_info).
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'apriltag.launch.yml'))),

        # TF (map -> dock tag) -> /detected_dock_pose at 10 Hz.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'detected_dock_pose_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # dock_trigger: docking sequence + the /goal_pose -> /goal_pose_nav undock gate
        # (replaces the placeholder relay).
        Node(
            package='openamrobot_docking', executable='dock_trigger.py',
            name='dock_trigger',
            # obstacle_scan_forward_angle=π: this unit's RPLIDAR is mounted rotated 180°, so the
            # forward obstacle cone must be offset by π (scan angle 0 points backward). Sim keeps 0.
            # use_apriltag_gate: real robot only. The dock sequence flips the
            # on-demand AprilTag image gate (apriltag.launch.yml) so apriltag
            # burns CPU only during the approach, not during navigation.
            parameters=[trigger_params, {'use_sim_time': use_sim_time,
                                         'obstacle_scan_forward_angle': 3.14159,
                                         'use_apriltag_gate': True}],
            output='screen'),
    ])
