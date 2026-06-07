import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('openamrobot_description')
    gazebo_dir = get_package_share_directory('openamrobot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    xacro_file = os.path.join(description_dir, 'urdf', 'robo_urdf.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # walled_world.sdf <include>s `model://apriltag_dock` which is shipped
    # by the openamrobot_docking package. We add its models/ directory to
    # GZ_SIM_RESOURCE_PATH so Gazebo can resolve the include at world load.
    # If openamrobot_docking is not installed, skip silently — gz_simulator
    # still works for non-docking workflows.
    resource_paths = [
        os.path.join(gazebo_dir, 'worlds'),
        str(Path(description_dir).parent.resolve()),
    ]
    try:
        docking_dir = get_package_share_directory('openamrobot_docking')
        resource_paths.append(os.path.join(docking_dir, 'models'))
    except Exception:
        pass

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths),
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        condition=IfCondition(use_robot_state_pub),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ])

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -v 2 ', world]}.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -s -v 2 ', world]}.items(),
        condition=UnlessCondition(gui),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'openamrobot',
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'use_robot_state_pub', default_value='True',
            description='Whether to start robot_state_publisher'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_dir, 'worlds', 'walled_world.sdf'),
            description='Full path to the Gazebo world file (.sdf) to load'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start Gazebo GUI. Default false keeps simulation timing stable.'),
        gz_resource_path,
        gz_sim_gui,
        gz_sim_headless,
        bridge,
        spawn_entity,
        start_robot_state_publisher_cmd,
        joint_state_publisher,
    ])
