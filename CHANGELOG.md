# Changelog

All notable changes to this repository should be documented in this file.

This project follows a simple changelog structure:

- `Added` for new features
- `Changed` for changes in existing functionality
- `Fixed` for bug fixes
- `Documentation` for documentation updates
- `Simulation` for simulation-related changes
- `Maintenance` for repository structure, CI, metadata, and contribution process

## Unreleased

### Added

- **`openamrobot_docking/docking_sim.launch.py`** — full docking-simulation bringup that composes `openamrobot_gazebo` (Gazebo + bridge), `openamrobot_nav2` (Nav2 + SLAM), and the docking-specific pieces (AprilTag detection, dock_trigger, RViz). Accepts `spawn_x` / `spawn_y` / `spawn_yaw` and auto-projects the world dock pose into the resulting map frame.
- **4-phase docking sequencer** (`ros2/src/openamrobot_docking/scripts/dock_trigger.py`):
  - Phase 1: `NavigateToPose` to a staging zone.
  - Phase 2: open-loop scan until the AprilTag is detected, then closed-loop yaw P-control on the camera-frame angle `atan2(X_optical, Z_optical)` until the tag is centred for N consecutive frames; running-average filter collects M samples.
  - Phase 3: in-place spin to the running-average `perpendicular_yaw`.
  - Phase 4a: pure-pursuit line-tracking — `desired_yaw = perp_yaw − atan2(lateral, line_lookahead_distance)`, `omega = line_yaw_kp · (desired_yaw − robot_yaw)`.
  - Phase 4b: at `distance < visual_servo_distance`, one-shot in-place align spin then straight-line approach to `docking_distance`.
  - Replaces both the discrete reverse-and-realign safety loop and the exponential low-pass auto-calibration with a continuous controller stable around the dock axis.
- **`TagRunningAverage` class** — incremental running mean for the tag position and componentwise sign-aligned quaternion mean. Updates throughout phases 2 and 4.
- **Direct TF helper** `lookup_tag_in_camera_optical` (queries `camera_rgb_optical_frame → charging_dock_apriltag`) so the centring scan is robust to map-frame solvePnP bias.
- **Docking scenario world** `ros2/src/openamrobot_docking/worlds/docking_scenario.sdf` — 10×10 m walled room with the AprilTag dock against the north wall. Robot is spawned at runtime via `ros_gz_sim create`.
- **AprilTag dock model** `ros2/src/openamrobot_docking/models/apriltag_dock/` — 0.40 × 0.40 × 0.01 m static panel textured with `tag0_big.png` (tag36h11 ID 0).
- **`apriltag_sim.launch.yml`** + **`tags_36h11_sim.yaml`** for AprilTag detection from the simulated Gazebo camera (no `camera_ros`, no rectification).
- **`scripts/kill_sim.sh`** — SIGKILLs zombie simulation processes between runs.

### Changed

- **`openamrobot_description/urdf/robo_urdf.urdf.xacro`** modifications vs the upstream SolidWorks-exported URDF:
  - added `base_footprint` root + `base_joint` for Nav2-conventional TF
  - symmetrised left/right wheel inertials (off-diagonals zeroed; the two wheels made identical) — fixes the "robot curves when commanded straight" failure caused by SolidWorks-exported asymmetries
  - cylinder collisions on drive wheels at radius 0.10 m (reconciled with the DiffDrive plugin)
  - added `camera_link` and `camera_rgb_optical_frame` with the −π/2, 0, −π/2 optical rotation required by `apriltag_ros::solvePnP`
- **`openamrobot_description/urdf/gazebo_control.xacro`**:
  - `wheel_radius` reconciled to 0.10 m (previously inconsistent 0.11 in plugin vs 0.10 in collisions)
  - `child_frame_id` changed from `base_link` to `base_footprint`
  - lidar range adjusted from 0.40-10 m to 0.15-12 m
  - added RGB camera sensor (640×480 @ 15 Hz, horizontal_fov 1.2 rad), `gz_frame_id=camera_rgb_optical_frame`
  - relative topic names so the bridge works under namespacing
- **`openamrobot_gazebo/config/gz_bridge.yaml`** extended with `/camera/image_raw`, `/camera/camera_info`, `/joint_states`.
- **`openamrobot_gazebo/launch/gz_simulator.launch.py`** now accepts `world`, `spawn_x`, `spawn_y`, `spawn_yaw` launch arguments. Spawn z fixed to `0.0` (the URDF root `base_footprint` is at ground level — non-zero z made the drive wheels float).
- **`openamrobot_nav2/config/nav2_params.yaml`** — Nav2 stack tuned for the OpenAMRobot platform:
  - NavfnPlanner with `use_astar: true`, `tolerance: 1.0`
  - RegulatedPurePursuitController, `desired_linear_vel: 0.55 m/s`
  - costmap `cost_scaling_factor: 8.0`, `inflation_radius: 0.45`
  - `velocity_smoother.max_decel = -0.5` softened from `-1.2` for the caster meshes
  - `collision_monitor.FootprintApproach.enabled = false` to avoid phantom near-obstacle stops during fast rotations
- **`openamrobot_nav2/config/slam_toolbox_params.yaml`** — SLAM Toolbox in mapping mode, `max_laser_range: 10.0`, `base_frame: base_footprint`.
- **`openamrobot_docking/config/dock_trigger.yaml`** — defaults for the 4-phase pipeline (`staging_distance: 2.5`, `docking_distance: 0.9`, `drive_speed: 0.05`, `filter_num_samples: 40`, `spin_kp: 1.5`, `spin_max_omega: 0.3`, `line_yaw_kp: 2.5`, `line_lookahead_distance: 0.3`, `visual_servo_distance: 1.4`, plus the scan parameters `scan_rotation_speed`, `scan_consecutive_target`, `scan_centring_tolerance`, `scan_centring_kp`). Removed: `realign_*` and `auto_cal_*`.
- **`openamrobot_description/launch/launch.py`** — static `odom→base_link` TF replaced with `odom→base_footprint` to align with the new TF root.

### Removed

- The 4-state reverse-and-realign safety loop in `dock_trigger.py` (caused convergence loops; replaced by the continuous pure-pursuit controller).
- The exponential low-pass auto-calibration on perpendicular yaw in `dock_trigger.py` (replaced by the incremental running-mean filter that is stable without an arbitrary blend coefficient).
- All `realign_*` and `auto_cal_*` parameters from `config/dock_trigger.yaml`.

### Documentation

- **Top-level docs** organised under `docs/`:
  - `getting_started/00_workspace_setup.md`, `getting_started/01_quickstart_docking_sim.md`
  - `architecture/01_repo_layout.md` — repo + dependency graph + what-goes-where rule
  - per-domain pointers: `docs/docking/`, `docs/simulation/`, `docs/navigation/`, `docs/safety/`
- **Package docs** under `ros2/src/openamrobot_docking/docs/`:
  - 13 in-depth engineering documents (00 → 12), including `08_sequencer_4phase.md` (4-phase walkthrough), `09_troubleshooting.md` (symptom → cause → fix matrix), `12_lessons_learned.md` (24-lesson pedagogical write-up)
  - `legacy/` subfolder preserving the 9 original `controlled_approach`-era docs verbatim
- **AUTHORS.md** — full attribution chain (Brawner → Dhakal → Indulkar → this revision, plus Alex for the platform monorepo scaffolding).
- **NOTICE.md** — third-party assets and the OpenAMRobot URDF/mesh provenance documented.

### Maintenance

- Migrated from the standalone `openamrobot-docking` repository into this monorepo (`openamr-platform-sw`). Code locations:
  - `omr_description/` → `ros2/src/openamrobot_description/`
  - `openamrobot_docking/` → `ros2/src/openamrobot_docking/`
  - `openamrobot_docking/simulation/config/nav2_sim_full.yaml` split into `openamrobot_nav2/config/nav2_params.yaml` (no docking_server) and `openamrobot_docking/config/dock_trigger.yaml` (the docking sequencer's own params)
  - `openamrobot_docking/simulation/config/ros_gz_bridge.yaml` merged into `openamrobot_gazebo/config/gz_bridge.yaml`
  - `openamrobot_docking/simulation/worlds/docking_world.sdf` → `openamrobot_docking/worlds/docking_scenario.sdf`
- `openamrobot_docking/package.xml` extended with platform-internal dependencies (`openamrobot_description`, `openamrobot_gazebo`, `openamrobot_nav2`) plus runtime deps (`apriltag_ros`, `apriltag_msgs`, `slam_toolbox`, `ros_gz_*`, `xacro`).
- `openamrobot_docking/CMakeLists.txt` updated to install `worlds/` and `models/` and to drop the AprilTag panel texture into the model's `materials/textures/` subdir.
