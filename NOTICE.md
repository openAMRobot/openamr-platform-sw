# Notice

OpenAMRobot and OpenAMR are project names maintained by the OpenAMRobot organization.

This repository contains the ROS 2 software stack for the OpenAMRobot mobile platform: robot description, Gazebo Harmonic simulation, Nav2 navigation, and AprilTag-based autodocking.

## Project Ownership

The OpenAMRobot organization maintains this repository and its project direction.

Contributors retain visibility and attribution through GitHub commits, Pull Requests, and release notes where applicable.

Contributions are accepted under the terms described in `CONTRIBUTING.md` and the repository `LICENSE`.

## Third-Party Material

This repository depends on third-party open-source packages from the ROS 2 ecosystem and related robotics tools. Third-party dependencies remain under their respective licenses.

Do not add third-party code, meshes, models, textures, datasets, marker images, or other assets unless their license is clearly documented.

### Bundled third-party assets

#### AprilTag panel texture

- **File:** `ros2/src/openamrobot_docking/models/apriltag_dock/materials/textures/tag0_big.png`
- **Source:** AprilRobotics `apriltag-imgs` repository ([github.com/AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs))
- **Family:** tag36h11, ID 0
- **License:** BSD-2-Clause (as published by AprilRobotics)
- **Modification status:** unmodified — `tag0_big.png` is the same pattern as `tag0.png` upscaled for use as a Gazebo material texture and as a printable calibration target.
- **Reason for inclusion:** required to render the AprilTag panel in the docking simulation. Installed by `CMakeLists.txt` into `share/openamrobot_docking/models/apriltag_dock/materials/textures/`.

#### OpenAMRobot URDF / xacro / STL meshes

- **Files:** `ros2/src/openamrobot_description/urdf/robo_urdf.urdf.xacro`, `ros2/src/openamrobot_description/urdf/gazebo_control.xacro`, `ros2/src/openamrobot_description/urdf/robot.sdf`, and the full mesh set under `ros2/src/openamrobot_description/meshes/{collision,visual}/`.
- **Source:** the URDF/xacro and STL mesh set originate from a SolidWorks-exported URDF (sw_urdf_exporter, **Stephen Brawner**) of the OpenAMRobot mobile platform by **Niraj Dhakal**. First packaged for ROS 2 by **Raj Indulkar** in [`rajindulkar22/openamrobot-simulation`](https://github.com/rajindulkar22/openamrobot-simulation); imported into the present `openamrobot_description` package by the OpenAMRobot organization.
- **License:** the underlying CAD and meshes are OpenAMRobot mobile-platform assets; presumed to be the OpenAMRobot organization's intellectual property. To be confirmed with a formal license declaration in `package.xml`.
- **Modifications applied in this repository (see `AUTHORS.md` for credit):**
  - added a `base_footprint` link plus `base_joint` for Nav2-conventional TF
  - symmetrised the left/right wheel inertials (off-diagonals zeroed and the two wheels made identical) to fix the "robot curves when commanded straight" failure caused by tiny (1e-8-level) asymmetries in the SolidWorks export
  - reconciled an inconsistent `wheel_radius` (the DiffDrive plugin had `0.11` while the wheel cylinder collisions had `0.10`) to a single value of `0.10 m`
  - added a `camera_link` plus `camera_rgb_optical_frame` with the standard −π/2, 0, −π/2 optical rotation required by `apriltag_ros::solvePnP`
  - adjusted the lidar range from 0.40–10 m to 0.15–12 m to span the 10×10 m docking scenario
  - added a Gazebo RGB camera plugin (640×480 @ 15 Hz, horizontal_fov 1.2 rad)
  - relative topic names in `gazebo_control.xacro` (`cmd_vel`, `odom`, `scan`, `camera/image_raw`, `joint_states`) so the gz↔ROS bridge resolves consistently when the robot is spawned inside a namespace
- **Reason for inclusion:** provides the canonical OpenAMRobot mobile-platform description (URDF + meshes), consumed by `openamrobot_gazebo`, `openamrobot_nav2`, and `openamrobot_docking`.

## Trademarks and Project Names

The names OpenAMRobot and OpenAMR are used to identify the project maintained by the OpenAMRobot organization.

Use of these names should not imply official endorsement, certification, or partnership unless explicitly approved by the maintainers.
