from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    docking_params = LaunchConfiguration('docking_params')
    trigger_params = LaunchConfiguration('trigger_params')
    launch_trigger = LaunchConfiguration('launch_trigger')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'docking_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openamrobot_docking'), 'config', 'openamrobot_docking.yaml']
            ),
            description='Path to docking server params YAML.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            'launch_trigger',
            default_value='true',
            description='Launch the dock trigger node.'
        ),
        DeclareLaunchArgument(
            'trigger_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openamrobot_docking'), 'config', 'dock_trigger.yaml']
            ),
            description='Path to dock trigger params YAML.'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('openamrobot_docking'), 'launch', 'detected_dock_pose_publisher.launch.py']
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('apriltag_ros'), 'launch', 'camera_36h11.launch.yml']
                )
            ),
        ),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[docking_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['docking_server']},
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='openamrobot_docking',
            executable='dock_trigger.py',
            name='dock_trigger',
            output='screen',
            parameters=[trigger_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_trigger),
        ),
    ])
