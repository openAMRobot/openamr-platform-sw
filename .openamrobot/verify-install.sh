#!/usr/bin/env bash
# Bootstrap the official ROS 2 Jazzy container for the shared PR verifier.
# This runs inside an ephemeral, root-owned CI container; no robot hardware.
set -euo pipefail

apt-get update
apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest \
  python3-yaml

# The clean verification environment uses its own HOME/rosdep cache.
# The system-wide source list may already exist in a reused image.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update --rosdistro jazzy

# These two hardware-driver runtime packages are explicitly tracked gaps:
# micro_ros_agent: platform-sw #50 (no Noble rosdep mapping)
# sllidar_ros2: source-pinned via openamrobot-manifest #18, not installed here.
# Skipping only their rosdep *installation* does not skip colcon tests or lint.
rosdep install --from-paths ros2/src --ignore-src --rosdistro jazzy \
  --skip-keys 'micro_ros_agent sllidar_ros2' -y

command -v colcon
