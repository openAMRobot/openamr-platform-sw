# SPDX-License-Identifier: MIT
"""Station alignment simulation: Gazebo world with the marker plate, the platform robot, AMCL, Nav2 and alignment.

  ros2 launch openamrobot_station_alignment station_alignment_sim.launch.py [headless:=true]

The world (worlds/station_alignment.sdf) is a separate file with the Baseline 1 work shelf layout and the marker
plate; the Baseline 1 world is not changed. Nav2 runs openamrobot_nav2's own nav2_params.yaml (platform-sw #164 head)
unchanged; only the map is this world's. The robot is spawned at the map origin, which is nav2_params.yaml's AMCL
initial pose. Simulation only.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    share = get_package_share_directory("openamrobot_station_alignment")
    headless = context.launch_configurations["headless"].lower() == "true"
    world = os.path.join(share, "worlds", "station_alignment.sdf")
    gz_args = f"-r {'-s --headless-rendering ' if headless else ''}-v 2 {world}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items())]


def generate_launch_description():
    share = get_package_share_directory("openamrobot_station_alignment")
    description = get_package_share_directory("openamrobot_description")
    nav2 = get_package_share_directory("openamrobot_nav2")
    robot = ParameterValue(Command(["xacro ", os.path.join(share, "urdf", "station_alignment_test_robot.urdf.xacro")]),
                           value_type=str)
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join([
            os.path.join(share, "models"), os.path.dirname(description)])),
        OpaqueFunction(function=_gazebo),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             output="screen", parameters=[sim_time, {"robot_description": robot}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
             output="screen", parameters=[sim_time]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-name", "openamrobot", "-topic", "/robot_description", "-x", "0", "-y", "0", "-z", "0.02"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="station_alignment_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge_station.yaml"),
                          "qos_overrides./tf_static.publisher.durability": "transient_local"}]),
        Node(package="openamrobot_perception", executable="scan_body_filter", name="scan_body_filter", output="screen",
             parameters=[os.path.join(nav2, "config", "scan_body_filter_sim.yaml"), sim_time]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "localization_launch.py")),
            launch_arguments={"map": os.path.join(share, "maps", "station_alignment_map.yaml"),
                              "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()),
        # Gazebo's CameraInfo restamped with each image (reused from openamrobot_docking).
        Node(package="openamrobot_docking", executable="camera_info_sync.py", name="camera_info_sync", output="screen",
             parameters=[sim_time, {"image_topic": "/base_camera/color/image_raw",
                                    "camera_info_topic": "/base_camera/color/camera_info",
                                    "synced_camera_info_topic": "/base_camera/color/camera_info_synced"}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "station_alignment.launch.py")),
            launch_arguments={"use_sim_time": "true", "image_topic": "/base_camera/color/image_raw",
                              "camera_info_topic": "/base_camera/color/camera_info_synced"}.items()),
    ])
