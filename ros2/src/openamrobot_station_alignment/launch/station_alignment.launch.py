# SPDX-License-Identifier: MIT
"""Station-relative final alignment (Proposed): marker detection, station pose estimate, goal refinement.

  ros2 launch openamrobot_station_alignment station_alignment.launch.py \
      image_topic:=/base_camera/color/image_raw camera_info_topic:=/base_camera/color/camera_info

Starts apriltag_ros (namespace apriltag), station_marker_detector and station_alignment. Needs Nav2 with the station
tree of openamrobot_nav2 (navigate_to_station.xml) running. Starts no controller, no navigator and no velocity
publisher. Launch argument names are Proposed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _apriltag(context):
    """apriltag_ros subscribes image_rect with an image_transport camera subscriber: its CameraInfo topic is the
    sibling camera_info of the resolved image topic, so that sibling is remapped too."""
    image = context.launch_configurations["image_topic"]
    info = context.launch_configurations["camera_info_topic"]
    sibling = image.rsplit("/", 1)[0] + "/camera_info"
    remaps = [("/apriltag/image_rect", image), ("/apriltag/camera_info", info)]
    if sibling != info:
        remaps.append((sibling, info))
    return [Node(package="apriltag_ros", executable="apriltag_node", name="apriltag", namespace="apriltag",
                 output="screen", remappings=remaps,
                 parameters=[context.launch_configurations["tags_file"],
                             {"use_sim_time": context.launch_configurations["use_sim_time"].lower() == "true"}])]


def generate_launch_description():
    share = get_package_share_directory("openamrobot_station_alignment")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    sim_time = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("image_topic", default_value="/base_camera/color/image_raw",
                              description="rectified or raw colour image of the base camera"),
        DeclareLaunchArgument("camera_info_topic", default_value="/base_camera/color/camera_info",
                              description="calibrated CameraInfo, stamped like the image"),
        DeclareLaunchArgument("tags_file", default_value=os.path.join(share, "config", "tags_station_sim.yaml"),
                              description="apriltag_ros parameters: family, black-square size, ids, frames"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "station_alignment.yaml")),
        OpaqueFunction(function=_apriltag),
        # station_marker_detector and station_alignment in one process: in-process hand-over of the station pose
        # (stack.py). No name= here: it would rename both nodes of the process.
        Node(package="openamrobot_station_alignment", executable="station_alignment_stack", output="screen",
             parameters=[params, sim_time]),
    ])
