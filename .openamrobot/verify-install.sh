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

# openamrobot_nav2 depends on openamr_nav_msgs from openamrobot-interfaces.
# Match the existing pinned revision from nav-status-tests.yml, rather than
# building against whichever interfaces revision is currently on main.
# The temporary checkout and copied ROS package exist only in this CI run.
interfaces_ref=77950470a87f1f5815346b2f000a662ee8c56f94
interfaces_checkout=$(mktemp -d)
trap 'rm -rf "$interfaces_checkout"' EXIT
git clone --quiet https://github.com/openAMRobot/openamrobot-interfaces.git "$interfaces_checkout"
git -C "$interfaces_checkout" checkout --quiet --detach "$interfaces_ref"
test "$(git -C "$interfaces_checkout" rev-parse HEAD)" = "$interfaces_ref"
test -s "$interfaces_checkout/ros2/openamr_nav_msgs/package.xml"
if [ -e ros2/src/openamr_nav_msgs ]; then
  echo "ERROR: refusing to overwrite existing openamr_nav_msgs" >&2
  exit 1
fi
cp -a "$interfaces_checkout/ros2/openamr_nav_msgs" ros2/src/openamr_nav_msgs

# These two hardware-driver runtime packages are explicitly tracked gaps:
# micro_ros_agent: platform-sw #50 (no Noble rosdep mapping)
# sllidar_ros2: source-pinned via openamrobot-manifest #18, not installed here.
# Skipping only their rosdep *installation* does not skip colcon tests or lint.
rosdep install --from-paths ros2/src --ignore-src --rosdistro jazzy \
  --skip-keys 'micro_ros_agent sllidar_ros2' -y

command -v colcon
