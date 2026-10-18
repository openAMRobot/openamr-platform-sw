#!/bin/bash
# Issue #35 review: SIM-ONLY fixture corrections applied to an INSTALL tree (never to src).
# They make the current main simulation usable for the arbitration tests; none is a
# proposed robot value. usage: apply_sim_fixtures.sh <ws>/install [--s3-range]
set -euo pipefail
I=$1
F1=$I/openamrobot_nav2/share/openamrobot_nav2/config/scan_body_filter.yaml
cat > "$F1" <<'YAML'
# SIM FIXTURE F1: lidar_link has yaw pi since #55, so the robot body is at scan
# angle 0 +/- 46 deg (measured). Remove [-52, +52] deg in place.
scan_body_filter:
  ros__parameters:
    filter1:
      name: body_filter
      type: laser_filters/LaserScanAngularBoundsFilterInPlace
      params:
        lower_angle: -0.9076
        upper_angle:  0.9076
YAML
Y=$I/openamrobot_docking/share/openamrobot_docking/config/dock_trigger.yaml
# F2: the checked-in dock pose is the real map's; the sim world dock is at (4.899, 0).
sed -i 's/^    dock_pose_x: 1.362/    dock_pose_x: 4.899/; s/^    dock_pose_y: 0.222/    dock_pose_y: 0.0/' "$Y"
# F3: forward cone offset pi, as the real profile, because the sim lidar is also yaw pi now.
grep -q '^    obstacle_scan_forward_angle' "$Y" || printf '    # SIM FIXTURE F3\n    obstacle_scan_forward_angle: 3.14159\n' >> "$Y"
if [ "${2:-}" = "--s3-range" ]; then
  # F4 (REJECTED, kept for the record): sim lidar range_min 0.40 -> 0.05. The sim
  # lidar_link sits inside the simplified chassis collision box, so every ray then
  # returns ~0.058 m. Do not use; near-field coverage cannot be validated in this sim.
  X=$I/openamrobot_description/share/openamrobot_description/urdf/gazebo_control.xacro
  sed -i 's|<min>0.40</min>|<min>0.05</min>|' "$X"
fi
echo "fixtures applied to $I"
