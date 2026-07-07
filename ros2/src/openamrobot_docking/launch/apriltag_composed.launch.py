"""
Composed vision pipeline: camera_ros + apriltag_ros in ONE multithreaded container
with intra-process communication (zero-copy).

WHY (see docs/AUDIT-2026-07-03-cpu-pipeline-optimization.md): the old path ran the image
through 3 processes and 2 DDS hops — camera_node -> apriltag_gate.py (Python/GIL, full-res
republish) -> apriltag_node — serializing a 2.7 MB frame twice per cycle. On a Pi 5 that is
NOT compute-bound (50% idle) but drowns in ~35k context-switches/s, which starves the
detector. Putting both composable nodes in one `component_container_mt` with
`use_intra_process_comms=True` passes the image BY POINTER: no DDS, no copy, no Python gate.

Replaces (for the real robot) camera.launch.py + apriltag.launch.yml. The camera therefore
lives here, not in the bring-up — start bring-up WITHOUT its camera when using this.

  ros2 launch openamrobot_docking apriltag_composed.launch.py

Publishes:
  /camera/image_raw, /camera/camera_info
  /apriltag/detections, /tf (camera_optical_frame -> charging_dock_tag_{0,1,2})
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception = get_package_share_directory('openamrobot_perception')
    docking = get_package_share_directory('openamrobot_docking')
    default_info = 'file://' + os.path.join(perception, 'config', 'camera_info.yaml')
    tags_cfg = os.path.join(docking, 'config', 'tags_36h11.yaml')

    decls = [
        DeclareLaunchArgument('width', default_value='1280',
                              description='Image width (px). Must match the calibration.'),
        DeclareLaunchArgument('height', default_value='720',
                              description='Image height (px). Must match the calibration.'),
        DeclareLaunchArgument('format', default_value='RGB888', description='Pixel format.'),
        DeclareLaunchArgument('camera_info_url', default_value=default_info,
                              description="Calibration URL (this unit's intrinsics)."),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame',
                              description='Camera optical frame id.'),
    ]

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',   # multithreaded: camera + detector on separate threads
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros', plugin='camera::CameraNode', name='camera',
                parameters=[{
                    'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                    'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                    'format': LaunchConfiguration('format'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'camera_info_url': LaunchConfiguration('camera_info_url'),
                    'FrameDurationLimits': [66667, 66667],   # ~15 fps cap (precision-neutral)
                }],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='apriltag_ros', plugin='AprilTagNode', name='apriltag',
                namespace='apriltag',
                parameters=[tags_cfg],
                remappings=[
                    ('/apriltag/image_rect', '/camera/image_raw'),
                    ('/camera_info', '/camera/camera_info'),
                    ('/apriltag/camera_info', '/camera/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='screen')

    return LaunchDescription(decls + [container])
