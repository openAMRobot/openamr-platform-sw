# SPDX-License-Identifier: MIT
"""
Docking pipeline bringup for the OpenAMRobot platform.

This is the docking layer ONLY — it does NOT start Gazebo, Nav2 or the
robot state publisher. The intended usage is to compose it on top of
the platform's running simulation:

    Terminal 1 :  ros2 launch openamrobot_gazebo gz_simulator.launch.py
    Terminal 2 :  ros2 launch openamrobot_nav2   sim_bringup_launch.py
    Terminal 3 :  ros2 launch openamrobot_docking openamrobot_docking.launch.py

What this launch starts:
  - apriltag_ros (sim variant)      — detects the 3-tag bundle (IDs 0/1/2)
                                       in `/rgb_image` and publishes one TF
                                       per tag:
                                         camera_optical_frame ->
                                         charging_dock_tag_{0,1,2}
  - detected_dock_pose_publisher    — reads map -> charging_dock_tag_1
                                       (the centre tag, docking target)
                                       and publishes /detected_dock_pose
                                       as PoseStamped at 10 Hz
  - dock_trigger (Python, bundle    — listens on /dock_trigger; on True,
    sequencer)                       runs the bundle docking sequence:
                                         1) NavigateToPose to staging
                                         2) camera-frame centring scan on
                                            the bundle midpoint
                                         3) estimate the dock surface
                                            normal from the outer tags'
                                            wide baseline (90 cm)
                                         4) pure-pursuit the normal axis
                                            in the camera/tag frame, with
                                            a re-verification step
                                         5) two-regime final approach:
                                            EMA-averaged axis far,
                                            axis-frozen visual servo on
                                            the centre tag near
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_trigger = LaunchConfiguration('launch_trigger')
    trigger_params = LaunchConfiguration('trigger_params')

    # Make `model://apriltag_dock` resolvable by Gazebo.
    # The dock model lives in this package's share/models/ directory; we
    # add its parent (the package's share/) to GZ_SIM_RESOURCE_PATH so that
    # the world's <include><uri>model://apriltag_dock</uri></include> can
    # find it. This is also why the PBR <albedo_map> uses model:// — it
    # needs the model to be registered through GZ_SIM_RESOURCE_PATH or the
    # texture won't render in the camera sensor (Gazebo Harmonic quirk).
    docking_models_dir = os.path.join(
        get_package_share_directory('openamrobot_docking'),
        'models',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the /clock topic published by Gazebo.'
        ),
        DeclareLaunchArgument(
            'launch_trigger',
            default_value='true',
            description='Launch the 4-phase dock_trigger node.'
        ),
        DeclareLaunchArgument(
            'trigger_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openamrobot_docking'),
                 'config', 'dock_trigger.yaml']
            ),
            description='Path to dock_trigger params YAML.'
        ),

        # Extend GZ_SIM_RESOURCE_PATH so Gazebo can find `model://apriltag_dock`
        # (referenced by walled_world.sdf via <include><uri>model://apriltag_dock</uri></include>).
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', docking_models_dir
        ),

        # ── Camera pipeline (how the image becomes tag TFs) ────────────────
        # 1. Gazebo renders the camera image (sensor declared in the robot URDF,
        #    openamrobot_description) and publishes it + a camera_info with the
        #    intrinsics (fx, fy, cx, cy, distortion). The image is RENDERED by
        #    Gazebo's 3D engine — not OpenCV.
        # 2. ros_gz_bridge turns the gz topics into ROS topics. The image is
        #    /rgb_image. camera_info needs the bridge below.
        # 3. apriltag_ros (started further down) consumes /rgb_image +
        #    /camera_info. Internally it uses OpenCV (cv_bridge to grayscale,
        #    cv::solvePnP for the pose) and publishes one TF per tag.
        # On a real robot only step 1 changes (a real camera driver replaces
        # Gazebo); steps 2-3 are identical.
        #
        # Bridge gz /camera_info -> ROS /camera_info.
        # apriltag_ros uses image_transport::CameraSubscriber, which derives
        # the camera_info topic from the image topic name (sibling at the
        # same namespace level). With the image remapped to /rgb_image at
        # the root, image_transport subscribes to /camera_info at the root.
        # Gazebo natively publishes a /camera_info gz topic — we just need
        # to bridge it. (Raj's bridge in openamrobot_gazebo only bridges
        # /camera/camera_info which isn't where image_transport looks.)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # apriltag_ros (simulation variant) — subscribes to /rgb_image
        # and (via the relay above) /camera_info; publishes one TF per tag:
        # camera_optical_frame -> charging_dock_tag_{0,1,2}.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('openamrobot_docking'),
                     'launch', 'apriltag_sim.launch.yml']
                )
            ),
        ),

        # TF -> /detected_dock_pose bridge at 10 Hz.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('openamrobot_docking'),
                     'launch', 'detected_dock_pose_publisher.launch.py']
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 4-phase docking sequencer. Listens on /dock_trigger.
        Node(
            package='openamrobot_docking',
            executable='dock_trigger.py',
            name='dock_trigger',
            output='screen',
            parameters=[trigger_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_trigger),
        ),
    ])
