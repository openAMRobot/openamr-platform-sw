"""
Goal forwarder for NAV-ONLY operation (no docking) - platform-sw#90/#124, Alex's 14 Nov
owner-decision review on head 162a19a.

navigation_launch.py remaps bt_navigator's internal goal_pose topic to a name nothing
publishes to (goal_pose_bt_navigator_unused - see that file's own comment), so the RViz
"2D Goal Pose" (/goal_pose) needs a real forwarder onto the PUBLIC navigate_to_pose action,
not a raw topic relay into bt_navigator's internal client. This launch starts
goal_pose_forwarder_node (openamrobot_nav2), which does that: one NavigateToPose goal per
/goal_pose message, through navigate_to_pose_goal_proxy_node.py like any other client,
checking /safety/stop_state before ever sending - see that node's own module docstring for
the full contract.

Formerly a plain `topic_tools relay /goal_pose -> /goal_pose_nav` straight into bt_navigator's
internal self_client_ - that bypassed the public action entirely, which is the exact defect
Nikolai's 14 Nov Gazebo reproduction confirmed (UI status/cancel on the public name saw
nothing for a goal sent this way, and the robot kept driving after a cancel that reached no
goal). Replaced, not just rewired - the relay could never have been fixed to close this gap,
since it forwarded onto a topic, not an action.

  ros2 launch openamrobot_bringup goal_relay.launch.py

USE EXACTLY ONE FORWARDER — never two on the public navigate_to_pose action at once (double
goals):
  * NAV-ONLY  -> run THIS forwarder.
  * DOCKING   -> run the docking layer instead (its ``dock_trigger`` IS the forwarder); do NOT run
                 this one. As soon as docking is up, it owns the goal routing. (The docking
                 helper's own /goal_pose path still targets goal_pose_nav today, which
                 bt_navigator no longer listens on - #124 gives docking its own proxy-based
                 path; not this PR's scope.)

The one-shot ``bringup.launch.py`` makes this choice automatically via ``use_docking``; this
standalone launch is for the per-terminal / "do what you want" workflow where you compose
layers by hand.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openamrobot_nav2', executable='goal_pose_forwarder_node',
            name='goal_pose_forwarder', output='screen'),
    ])
