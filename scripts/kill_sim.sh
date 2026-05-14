#!/bin/bash
# Kill every process that docking_sim.launch.py (or any other openamr-platform-sw
# simulation launch) spawns. Run this before relaunching if a previous sim was
# interrupted with Ctrl+C and zombies remain (gz-sim, parameter_bridge,
# slam_toolbox, etc.).
#
# Use: ./kill_sim.sh   (or  bash scripts/kill_sim.sh)

# -9 = SIGKILL : ne laisse pas le choix au process. Pas de SIGTERM gracieux,
# c'est volontaire — on veut nettoyer un état foiré.
PATTERNS=(
    # Gazebo
    "gz sim"
    "gz-sim"
    "gz-tools"
    "ruby.*gz"
    # ROS 2 / Nav2
    "ros2 launch"
    "ros2 run"
    "component_container"
    "component_container_isolated"
    "lifecycle_manager"
    "controller_server"
    "planner_server"
    "behavior_server"
    "bt_navigator"
    "smoother_server"
    "velocity_smoother"
    "collision_monitor"
    "waypoint_follower"
    "docking_server"
    "amcl"
    "map_server"
    # SLAM
    "async_slam_toolbox_node"
    "slam_toolbox"
    # AprilTag / docking
    "apriltag_node"
    "dock_trigger"
    "detected_dock_pose_publisher"
    # Bridge / RViz / TF
    "parameter_bridge"
    "ros_gz_bridge"
    "ros_gz_image"
    "robot_state_publisher"
    "joint_state_publisher"
    "rviz2"
)

killed_any=0
for p in "${PATTERNS[@]}"; do
    if pgrep -f "$p" > /dev/null 2>&1; then
        pkill -9 -f "$p" 2>/dev/null
        killed_any=1
    fi
done

# Petit délai pour que l'OS récolte les processus zombies
sleep 1

if [ $killed_any -eq 1 ]; then
    echo "✓ Sim processes killed."
else
    echo "(no sim processes were running)"
fi

# Vérification résiduelle
remaining=$(pgrep -f "gz sim|component_container|slam_toolbox|apriltag|dock_trigger" 2>/dev/null | wc -l)
if [ "$remaining" -gt 0 ]; then
    echo "⚠ $remaining process(es) still alive — try again:"
    pgrep -af "gz sim|component_container|slam_toolbox|apriltag|dock_trigger"
fi
