# SPDX-License-Identifier: MIT
"""Constants of the trial runner that the static tests check without importing ROS."""
# nav2_params.yaml footprint (local and global costmap). test_clearance_static.py checks it against the file.
FOOTPRINT = ((0.415, 0.19), (0.415, -0.19), (0.315, -0.29), (-0.290, -0.29), (-0.390, -0.19), (-0.390, 0.19),
             (-0.290, 0.29), (0.315, 0.29))
