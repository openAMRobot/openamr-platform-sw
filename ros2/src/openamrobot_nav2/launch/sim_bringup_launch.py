import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_nav2')
    map_file = os.path.join(pkg, 'maps', 'my_map.yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    # Scan filter for this (legacy / per-terminal) sim flow, now that navigation_launch no longer
    # runs it (filtering is a data-source responsibility — Raj review PR1). One /scan_filtered.
    #
    # issue #74: the previous laser_filters/scan_to_scan_filter_chain config
    # (scan_body_filter.yaml) is a truncation filter - it can only keep one
    # contiguous angular window and cannot mask an interior sector while
    # keeping both wraparound ends. That shape fit the body footprint's old,
    # rear-centered location, but not where it actually is after the 12 Oct
    # 2026 LiDAR mount rotation (now ~0 deg, dead ahead - see
    # scan_body_filter_sim.yaml for the full writeup and re-measured bounds).
    # Switched to the same per-sector masking node already used and tested on
    # the real robot (openamrobot_perception/scan_body_filter.py), which can
    # express this topology, with sim-specific mask bounds.
    scan_filter_config = os.path.join(pkg, 'config', 'scan_body_filter_sim.yaml')
    scan_filter = Node(
        package='openamrobot_perception', executable='scan_body_filter', name='scan_body_filter',
        parameters=[scan_filter_config, {'use_sim_time': True}],
    )

    rviz_config = os.path.join(pkg, 'rviz', 'nav2_view.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
            description='Start RViz. Default false reduces simulation lag.'
        ),
        localization,
        navigation,
        scan_filter,
        rviz,
    ])
