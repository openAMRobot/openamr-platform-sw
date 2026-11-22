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
- **Original creator and CAD author:** **Alex (Oleksii) Reznichenko**, OpenAMRobot project, 12 January 2021.
- **Rights history:** the relevant original CAD and project rights were assigned to **Botshare LTD** on 12 January 2025. Botshare LTD maintains the supporting chain-of-title records and can provide appropriate confirmation where reasonably required.
- **URDF contributors:** **Niraj Dhakal** contributed to creation of the URDF/xacro and STL mesh set from the original OpenAMRobot CAD files using the SolidWorks URDF Exporter (`sw_urdf_exporter`, Stephen Brawner). **Raj Indulkar** contributed to ROS 2 packaging based on those CAD-derived files, initially through [`rajindulkar22/openamrobot-simulation`](https://github.com/rajindulkar22/openamrobot-simulation). Contributor authorship and attribution are preserved; the files and transferable economic rights are owned by or validly assigned to **OpenAMRobot (Botshare LTD)**.
- **Current outbound licence for the CAD-derived URDF/xacro and mesh source:** **CERN-OHL-P-2.0**, subject to authentic third-party notices and rights. Software surrounding and consuming those assets remains MIT; documentation remains CC BY 4.0. See [`LICENSING.md`](LICENSING.md).
- **Modifications applied in this repository (see `AUTHORS.md` for credit):**
  - added a `base_footprint` link plus `base_joint` for Nav2-conventional TF
  - symmetrised the left/right wheel inertials (off-diagonals zeroed and the two wheels made identical) to fix the "robot curves when commanded straight" failure caused by tiny (1e-8-level) asymmetries in the SolidWorks export
  - set the DiffDrive plugin `wheel_radius` to `0.10 m` (kinematic radius used for `ω = v / r`) while keeping the wheel cylinder **collision** radius at `0.11 m`. The 1 cm difference is intentional: `base_footprint` is at ground level, `base_joint` lifts `base_link` 0.053467 m so the wheel centres sit at z = 0.10 m; a 0.10 m collision cylinder only touches the ground (zero penetration → zero normal force → zero friction), so the 0.11 m collision is what gives ODE the contact depth required for traction
  - added a `camera_link` plus `camera_rgb_optical_frame` with the standard −π/2, 0, −π/2 optical rotation required by `apriltag_ros::solvePnP`
  - adjusted the lidar range from 0.40–10 m to 0.15–12 m to span the 10×10 m docking scenario
  - added a Gazebo RGB camera plugin (640×480 @ 15 Hz, horizontal_fov 1.2 rad)
  - relative topic names in `gazebo_control.xacro` (`cmd_vel`, `odom`, `scan`, `camera/image_raw`, `joint_states`) so the gz↔ROS bridge resolves consistently when the robot is spawned inside a namespace
- **Reason for inclusion:** provides the canonical OpenAMRobot mobile-platform description (URDF + meshes), consumed by `openamrobot_gazebo`, `openamrobot_nav2`, and `openamrobot_docking`.
- **Superseded visual meshes:** the SolidWorks visual STLs in `meshes/visual/` are replaced by the 2.0 base meshes below. `urdf/robot.sdf` now points at the byte-identical copies in `meshes/collision/`, which are unchanged.

#### OpenAMRobot 2.0 base visual meshes (CAD v0.2)

- **Files:** `ros2/src/openamrobot_description/meshes/visual/*.stl` and `ros2/src/openamrobot_description/meshes/PROVENANCE.yaml`, generated by `tools/cad/step_to_description_meshes.py` with `tools/cad/base_mesh_map.yaml`.
- **Source:** [`openAMRobot/openamr-platform-hw`](https://github.com/openAMRobot/openamr-platform-hw), `mechanical/cad/Full_assembly_STEP/MMP.00.00.00.000_full_assembly.STEP.zip` (v0.2 full assembly, Rev B front and back panels, 2.0 sensor mounts) and `mechanical/cad/production_files/MMP.01.00.00.003 Camera mount.STEP`. The repository, commit and sha256 of every input are recorded in `meshes/PROVENANCE.yaml`.
- **Licence:** the chassis meshes (`base_chassis.stl`, `base_covers.stl`) are tessellated, decimated exports of OpenAMRobot parts (MMP.*) and are licensed **CERN-OHL-P-2.0**, as the hardware source.
- **Vendor components are envelopes:** the hub motor wheels, castors, motor driver, sensors, compute module, base controller and battery are simplified boxes, cylinders and revolved profiles generated from dimensions in `base_mesh_map.yaml` (datasheets, BOM Issue 7.14 (27 Oct 2026, with 1 Nov changes) and the CAD instances, each listed in `PROVENANCE.yaml`). No vendor CAD geometry is redistributed. The envelopes are generated by MIT-licensed code.
- **Modifications:** release 001 parts in the STEP are excluded; the empty camera mount instance is replaced by the production part file, and the displaced base plate is placed at its design position (both recorded under `corrections` in `PROVENANCE.yaml`).

#### CAN software-in-the-loop emulators from openamr-platform-fw

- **Files:** `ros2/src/openamrobot_sil/openamrobot_sil/vendor/can_sil/`: `canopen_common.py`, `zlac8015d_emulator.py`, `ctl017_emulator.py`, `ctl017_od_v1.py`, `tests/conftest.py`, `tests/test_zlac8015d.py`, `tests/test_ctl017.py`, `LICENSE`.
- **Source:** [openAMRobot/openamr-platform-fw](https://github.com/openAMRobot/openamr-platform-fw), directory `tools/can_sil`, commit `ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d` (head of draft PR #53, not merged).
- **Licence:** MIT, copyright OpenAMRobot (Botshare LTD). The licence file of that repository at the same commit is kept next to the files.
- **Modification status:** unmodified, byte for byte. `SHA256SUMS` records the hashes and `test/test_vendor_integrity.py` checks them.
- **Reason for inclusion:** the ZLAC8015D and CTL-017 CANopen emulators are reused by `openamrobot_sil` instead of being rewritten. Details: `vendor/can_sil/PROVENANCE.md`.

#### Nav2 bringup launch files

- **Files:** `ros2/src/openamrobot_nav2/launch/navigation_launch.py`, `ros2/src/openamrobot_nav2/launch/localization_launch.py`.
- **Source:** Nav2 ([github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)), package `nav2_bringup`, files `nav2_bringup/launch/navigation_launch.py` and `nav2_bringup/launch/localization_launch.py`, branch `jazzy`. The upstream commit the copies were taken from is not recorded; they were compared with `jazzy` at `c7046550547d382a5186d106b0895eda02485d7c` (22 Nov 2026). Upstream has no NOTICE file.
- **Copyright:** Copyright (c) 2018 Intel Corporation (as stated in the header of each file).
- **Licence:** Apache-2.0 ([apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)). The upstream licence header is kept in each file. These two files are not covered by the repository's MIT licence.
- **Modification status:** modified by OpenAMRobot. Both files read their share directory from `openamrobot_nav2` and their default parameters from `config/nav2_params.yaml`. `localization_launch.py` passes the map file to the map server through a rewritten parameter file. `navigation_launch.py` drops `route_server` and `docking_server`, reorders the lifecycle nodes, remaps the behavior server and BT navigator topics (`cmd_vel`, `goal_pose`), sets `bond_timeout`, and adds `navigation_status_node`.
- **Reason for inclusion:** Nav2 bringup adapted to the OpenAMRobot topic chain and docking flow. `openamrobot_nav2/package.xml` lists Apache-2.0 next to MIT for these files.

## Trademarks and Project Names

The names OpenAMRobot and OpenAMR are used to identify the project maintained by the OpenAMRobot organization.

Use of these names should not imply official endorsement, certification, or partnership unless explicitly approved by the maintainers.

## Botshare LTD and project ownership

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD**, Company ID HE479056, Chrysanthou Mylona 1, Panayides Building, Office 1, 3030 Limassol, Cyprus (info@botshare.ai; https://botshare.ai).

Copyright and transferable economic rights in original OpenAMRobot material are owned by Botshare LTD or used by it under applicable agreements. Accepted external contributions are governed by the OpenAMRobot Contributor Agreement. Botshare LTD does not claim ownership of third-party material; all authentic upstream ownership, licence, patent, attribution, and modification notices remain effective.
