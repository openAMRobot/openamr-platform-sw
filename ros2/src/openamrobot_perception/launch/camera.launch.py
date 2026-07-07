"""
Camera bring-up for the real OpenAMRobot (Sony IMX708 via camera_ros).

Resolution / format / frame / calibration are all parameters (Raj review PR1): defaults match THIS
unit (1280x720, calibrated), but everything is overridable for another camera. The node name is the
`camera_name` so it stays aligned with the device/calibration.

  ros2 launch openamrobot_perception camera.launch.py width:=640 height:=480 \
       camera_info_url:=file:///path/to/other_camera_info.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_perception')
    default_info = 'file://' + os.path.join(pkg, 'config', 'camera_info.yaml')

    decls = [
        DeclareLaunchArgument('camera_info_url', default_value=default_info,
                              description="Calibration URL (this unit's intrinsics)."),
        DeclareLaunchArgument('width', default_value='1280',
                              description='Image width (px). Must match the calibration.'),
        DeclareLaunchArgument('height', default_value='720',
                              description='Image height (px). Must match the calibration.'),
        DeclareLaunchArgument('format', default_value='RGB888', description='Pixel format.'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame',
                              description='Camera optical frame id.'),
        DeclareLaunchArgument('camera_name', default_value='camera',
                              description='Node name / camera name (kept aligned with the device).'),
    ]

    node = Node(
        package='camera_ros', executable='camera_node',
        name=LaunchConfiguration('camera_name'),
        parameters=[{
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'format': LaunchConfiguration('format'),
            'frame_id': LaunchConfiguration('frame_id'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            # Cap the sensor to ~15 fps (µs frame-duration min/max). The AprilTag detector
            # never needs more than ~15 fps for docking; running the sensor+software-ISP at
            # 30 fps just doubled camera_node CPU and message churn for frames the gate drops.
            # This is precision-NEUTRAL (same resolution) — verified 2026-07-03: camera_node
            # work halved, context-switches down, CPU idle 55%->67%. See
            # docs/AUDIT-2026-07-03-cpu-pipeline-optimization.md (B1).
            'FrameDurationLimits': [66667, 66667],
        }],
        respawn=True, respawn_delay=3.0,
        output='screen')

    return LaunchDescription(decls + [node])
