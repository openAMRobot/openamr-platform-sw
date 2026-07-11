"""
Goal forwarder for NAV-ONLY operation (no docking).

Nav2's ``bt_navigator`` listens on ``/goal_pose_nav`` (``navigation_launch`` remaps it there so the
docking node can gate ``/goal_pose`` — undock-before-navigate). So when you run the stack WITHOUT
docking, nothing forwards the RViz "2D Goal Pose" (``/goal_pose``) to Nav2 and goals are silently
dropped. This launch is that missing forwarder: a plain ``topic_tools relay /goal_pose ->
/goal_pose_nav``.

  ros2 launch openamrobot_bringup goal_relay.launch.py

USE EXACTLY ONE FORWARDER — never two on /goal_pose_nav at once (double goals):
  * NAV-ONLY  -> run THIS relay.
  * DOCKING   -> run the docking layer instead (its ``dock_trigger`` IS the forwarder); do NOT run
                 this relay. As soon as docking is up, it owns the goal routing.

The one-shot ``bringup.launch.py`` makes this choice automatically via ``use_docking``; this standalone
launch is for the per-terminal / "do what you want" workflow where you compose layers by hand.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools', executable='relay', name='goal_pose_relay',
            arguments=['/goal_pose', '/goal_pose_nav'], output='screen'),
    ])
