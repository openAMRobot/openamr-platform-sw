import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_perception')
    default_params = os.path.join(pkg, 'config', 'scan_body_filter_real.yaml')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=default_params,
            description='YAML of scan_body_filter parameters (default = real-robot calibration).',
        ),
        Node(
            package='openamrobot_perception',
            executable='scan_body_filter',
            name='scan_body_filter',
            output='screen',
            parameters=[params],
        ),
    ])
