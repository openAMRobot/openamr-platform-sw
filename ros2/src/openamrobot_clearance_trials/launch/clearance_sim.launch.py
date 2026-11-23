# SPDX-License-Identifier: MIT
"""Clearance trial simulation: the generated arena, the platform robot, AMCL and Nav2 from openamrobot_nav2.

  ros2 launch openamrobot_clearance_trials clearance_sim.launch.py scene_dir:=/tmp/clearance_scene \
      [params_file:=<nav2 parameters>] [headless:=true]

scene_dir holds clearance.sdf and clearance_map.yaml (python3 -m openamrobot_clearance_trials.scene <dir>).
params_file defaults to openamrobot_nav2's nav2_params.yaml unchanged. Simulation only.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _stack(context):
    share = get_package_share_directory("openamrobot_clearance_trials")
    nav2 = get_package_share_directory("openamrobot_nav2")
    scene_dir = context.launch_configurations["scene_dir"]
    params = context.launch_configurations["params_file"] or os.path.join(nav2, "config", "nav2_params.yaml")
    headless = context.launch_configurations["headless"].lower() == "true"
    world = os.path.join(scene_dir, "clearance.sdf")
    gz_args = f"-r {'-s --headless-rendering ' if headless else ''}-v 2 {world}"
    sim_time = {"use_sim_time": True}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clearance_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge_clearance.yaml"),
                          "qos_overrides./tf_static.publisher.durability": "transient_local"}]),
        Node(package="openamrobot_perception", executable="scan_body_filter", name="scan_body_filter", output="screen",
             parameters=[os.path.join(nav2, "config", "scan_body_filter_sim.yaml"), sim_time]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "localization_launch.py")),
            launch_arguments={"map": os.path.join(scene_dir, "clearance_map.yaml"), "use_sim_time": "true",
                              "params_file": params}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": "true", "params_file": params,
                              "enabled_producers": "navigate_to_pose,navigate_through_poses"}.items()),
    ]


def generate_launch_description():
    share = get_package_share_directory("openamrobot_clearance_trials")
    description = get_package_share_directory("openamrobot_description")
    robot = ParameterValue(Command(["xacro ", os.path.join(share, "urdf", "clearance_test_robot.urdf.xacro")]),
                           value_type=str)
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("scene_dir", default_value="/tmp/clearance_scene"),
        DeclareLaunchArgument("params_file", default_value=""),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(description)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             output="screen", parameters=[sim_time, {"robot_description": robot}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
             output="screen", parameters=[sim_time]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-name", "openamrobot", "-topic", "/robot_description", "-x", "0", "-y", "0", "-z", "0.02"]),
        OpaqueFunction(function=_stack),
    ])
