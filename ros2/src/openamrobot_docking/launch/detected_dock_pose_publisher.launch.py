from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tf_topic = LaunchConfiguration('tf_topic')
    tf_static_topic = LaunchConfiguration('tf_static_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openamrobot_docking'), 'config', 'docking_pose_publisher.yaml']
            ),
            description='Path to the parameters file.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            'tf_topic',
            default_value='/tf',
            description='TF topic to listen on.'
        ),
        DeclareLaunchArgument(
            'tf_static_topic',
            default_value='/tf_static',
            description='TF static topic to listen on.'
        ),
        Node(
            package='openamrobot_docking',
            executable='detected_dock_pose_publisher',
            name='detected_dock_pose_publisher',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', tf_topic),
                ('/tf_static', tf_static_topic),
            ],
            output='screen',
        ),
    ])
