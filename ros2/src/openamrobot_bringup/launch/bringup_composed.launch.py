"""
Full real-robot bring-up with the OPTIMIZED (composed) vision pipeline — one command.

Brings up everything to see the 2026-07-03 optimizations (B1 + A1) end-to-end:
  1. bringup.launch.py  with use_camera:=false, use_docking:=false
        -> nav (Nav2 localization + navigation) + sensors + micro-ROS + EKF + TF, NO camera.
  2. apriltag_composed.launch.py
        -> camera + apriltag in ONE component_container_mt, intra-process (zero-copy), 15 fps.
           Replaces the old 3-process camera -> apriltag_gate.py (Python) -> apriltag path.
  3. docking_composed.launch.py
        -> dock_trigger + detected_dock_pose (no apriltag here; use_apriltag_gate:=false).

Why this exists: the old path serialized every 2.7 MB frame twice across 3 processes and a
Python GIL gate, starving the detector on a Pi that was actually 50% idle. Composition drops
vision CPU ~124% -> ~55% and feeds the detector at the full 15 Hz. See
docs/AUDIT-2026-07-03-cpu-pipeline-optimization.md.

  ros2 launch openamrobot_bringup bringup_composed.launch.py map:=/home/botshare/maps/piece_actuelle.yaml

Then: 2D Pose Estimate in RViz; dock with `ros2 topic pub --once /dock_trigger std_msgs/msg/Bool "{data: true}"`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('openamrobot_bringup')
    docking = get_package_share_directory('openamrobot_docking')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/botshare/maps/piece_actuelle.yaml',
            description='Map yaml for AMCL localization.'),

        # nav + sensors, camera OFF (the composed container owns libcamera) and docking OFF
        # (started below, without a second apriltag).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'bringup.launch.py')),
            launch_arguments={'map': map_yaml,
                              'use_camera': 'false',
                              'use_docking': 'false'}.items()),

        # camera + apriltag composed intra-process (B1 15 fps + A1 zero-copy).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(docking, 'launch', 'apriltag_composed.launch.py'))),

        # dock_trigger + detected_dock_pose (no apriltag; gate off).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(docking, 'launch', 'docking_composed.launch.py'))),
    ])
