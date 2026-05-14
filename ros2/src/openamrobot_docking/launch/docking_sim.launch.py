# SPDX-License-Identifier: Apache-2.0
"""Full docking-simulation bringup for the OpenAMRobot platform.

Composition (each piece comes from a sibling package — this launch only adds
the docking-specific glue):

  openamrobot_gazebo/gz_simulator    : Gazebo Harmonic + URDF + bridge
  openamrobot_nav2/nav2_bringup      : Nav2 stack + SLAM Toolbox
  apriltag_sim                       : AprilTag detection on the gz camera
  detected_dock_pose_publisher       : TF -> /detected_dock_pose 10 Hz
  dock_trigger                       : the 4-phase docking sequencer

Coordinate convention (default spawn x=-4, y=-4, yaw=0):

  AprilTag dock in the world frame : (0.0, 4.9, 0.3) yaw=-pi/2
  Map origin = robot spawn pose (SLAM Toolbox convention),
  so dock in the map frame becomes  (4.0, 8.9) yaw=+pi/2.

Pass `spawn_x:= spawn_y:= spawn_yaw:=` to spawn the robot elsewhere — the
dock's map-frame pose is automatically re-projected through the rotation
implied by `spawn_yaw`.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORLD_DOCK_X = 0.0
WORLD_DOCK_Y = 4.9
WORLD_APPROACH_YAW = math.pi / 2.0  # heading the robot ends up with at the dock


def _build_nodes(context):
    pkg = get_package_share_directory('openamrobot_docking')
    pkg_gazebo = get_package_share_directory('openamrobot_gazebo')
    pkg_nav2 = get_package_share_directory('openamrobot_nav2')

    # Sources of compositional truth — these are the same paths anyone else
    # would use when composing the same set of packages.
    gazebo_launch = os.path.join(pkg_gazebo, 'launch', 'gz_simulator.launch.py')
    nav2_launch = os.path.join(pkg_nav2, 'launch', 'nav2_bringup.launch.py')
    rviz_config = os.path.join(pkg_nav2, 'rviz', 'nav2_default.rviz')
    scenario_world = os.path.join(pkg, 'worlds', 'docking_scenario.sdf')

    dock_trigger_config = os.path.join(pkg, 'config', 'dock_trigger.yaml')

    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))

    # SLAM Toolbox places the map origin at the robot spawn pose, so the dock
    # has to be re-projected. dock_in_map = R(-spawn_yaw) * (dock_in_world - spawn_in_world).
    cos_s = math.cos(spawn_yaw)
    sin_s = math.sin(spawn_yaw)
    dx = WORLD_DOCK_X - spawn_x
    dy = WORLD_DOCK_Y - spawn_y
    map_dock_x = cos_s * dx + sin_s * dy
    map_dock_y = -sin_s * dx + cos_s * dy
    map_dock_yaw = WORLD_APPROACH_YAW - spawn_yaw

    print(
        f'[docking_sim] spawn=({spawn_x:.2f}, {spawn_y:.2f}, '
        f'yaw={spawn_yaw:.3f}) -> dock in map=({map_dock_x:.2f}, '
        f'{map_dock_y:.2f}, yaw={map_dock_yaw:.3f})'
    )

    return [
        # Tell gz-sim where to find model://apriltag_dock. The gz_simulator
        # launch only adds the description package's parent dir to the
        # resource path; we add the docking package's models dir on top.
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')
        ),

        # Gazebo + URDF + bridge, with the docking scenario world.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': scenario_world,
                'spawn_x': f'{spawn_x:.6f}',
                'spawn_y': f'{spawn_y:.6f}',
                'spawn_yaw': f'{spawn_yaw:.6f}',
                'use_sim_time': 'True',
            }.items(),
        ),

        # Nav2 + SLAM.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # AprilTag detection (gz camera variant — no camera_ros, no rectification).
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('openamrobot_docking'),
                    'launch', 'apriltag_sim.launch.yml',
                ])
            ),
        ),

        # TF -> PoseStamped bridge at 10 Hz.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('openamrobot_docking'),
                    'launch', 'detected_dock_pose_publisher.launch.py',
                ])
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # The 4-phase docking sequencer. The YAML provides defaults; the
        # parameter override below wins so non-default spawn poses still aim
        # at the right (auto-reprojected) map-frame dock pose.
        Node(
            package='openamrobot_docking',
            executable='dock_trigger.py',
            name='dock_trigger',
            output='screen',
            parameters=[
                dock_trigger_config,
                {
                    'use_sim_time': True,
                    'dock_pose_x': map_dock_x,
                    'dock_pose_y': map_dock_y,
                    'dock_pose_yaw': map_dock_yaw,
                },
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_x',
            default_value='-4.0',
            description='Robot spawn X in the WORLD frame (metres).',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='-4.0',
            description='Robot spawn Y in the WORLD frame (metres).',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description=(
                'Robot spawn yaw in the WORLD frame (radians). The SLAM map '
                'origin coincides with the spawn pose, so the dock pose is '
                'automatically re-projected from the (fixed) world dock pose '
                'into the resulting map frame.'
            ),
        ),
        OpaqueFunction(function=_build_nodes),
    ])
