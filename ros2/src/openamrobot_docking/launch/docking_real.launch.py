"""
Real-robot docking pipeline (no simulation).

Composes the hardware-agnostic docking stack onto the REAL camera, WITHOUT any
Gazebo glue (no GZ_SIM_RESOURCE_PATH, no ros_gz_bridge camera_info bridge, no
camera_info_sync — the real camera already publishes a calibrated
/camera/camera_info, started by openamrobot_bringup/real_bringup.launch.py):

  - apriltag.launch.yml          — AprilTagNode on /camera/image_raw +
                                    /camera/camera_info -> TF to the dock tags.
  - detected_dock_pose_publisher — map -> charging_dock_tag_1 TF -> /detected_dock_pose.
  - docking_server (opennav_docking) — the action server /dock_robot and
                                    /undock_robot execute on, same as the sim path
                                    (platform-sw#123/#125). Its velocity output is
                                    remapped to cmd_vel_dock so docking commands
                                    enter the twist_mux arbitration instead of
                                    publishing on the final /cmd_vel.
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
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_docking')
    trigger_params = os.path.join(pkg, 'config', 'dock_trigger.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_docking_server = LaunchConfiguration('launch_docking_server')
    # The docking server's parameters live with the rest of the Nav2 config; a
    # real deployment can point this at its own file.
    docking_server_params = LaunchConfiguration('docking_server_params')
    nav2_params_default = PathJoinSubstitution(
        [FindPackageShare('openamrobot_nav2'), 'config', 'nav2_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real robot uses the real clock (false).'),
        DeclareLaunchArgument(
            'launch_docking_server', default_value='false',
            description=(
                'Start the opennav_docking action server (/dock_robot, '
                '/undock_robot). OFF by default on the real robot: D-19 (Alex, '
                '11 Nov) keeps reverse and automatic undock disabled in Build 1, '
                'and this server exposes a direct /undock_robot that the '
                'dock_trigger constant cannot gate. Enable it only once (1) '
                'release no longer needs a dock_robot server, (2) a direct '
                '/undock_robot produces no motion, and (3) the real-profile '
                'runtime test runs. Simulation keeps its own default (on).'
            )),
        DeclareLaunchArgument(
            'docking_server_params', default_value=nav2_params_default,
            description='Parameter file holding the docking_server block.'),

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
            # obstacle_scan_forward_angle=π: this unit's RPLIDAR A1, legacy (existing robot), is
            # mounted rotated 180°, so the forward obstacle cone must be offset by π (scan angle 0
            # points backward). To re-check on the RPLIDAR S3. Sim keeps 0.
            # use_apriltag_gate: real robot only. The dock sequence flips the
            # on-demand AprilTag image gate (apriltag.launch.yml) so apriltag
            # burns CPU only during the approach, not during navigation.
            parameters=[trigger_params, {'use_sim_time': use_sim_time,
                                         'obstacle_scan_forward_angle': 3.14159,
                                         'use_apriltag_gate': True}],
            output='screen'),

        # ── Docking action server (platform-sw#123/D-13) ───────────────────
        # Same node and the same remap as the sim path: DockingServer publishes
        # on the relative topic "cmd_vel" (docking_server.cpp:64), so launched at
        # the root namespace it would publish on the final /cmd_vel and bypass
        # collision_monitor. twist_mux takes docking at priority 20 on
        # cmd_vel_dock, which is where it belongs.
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[docking_server_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_dock')],
            condition=IfCondition(launch_docking_server),
        ),
        # opennav_docking is a lifecycle node; without a manager it stays
        # unconfigured and its action servers never become available.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['docking_server'],
            }],
            condition=IfCondition(launch_docking_server),
        ),
    ])
