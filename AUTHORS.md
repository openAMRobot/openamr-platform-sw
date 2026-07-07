# Authors and Contributors

This repository is maintained by the OpenAMRobot organization.

## Maintainer

- OpenAMRobot organization
- Contact: botshare.ai@gmail.com

## Contributors

Recognition is given to contributors whose work has materially shaped this repository. Contributions are grouped by area of focus rather than by chronology. Listing here does not replace GitHub history — it complements it by making non-trivial contributions easy to find for new readers, students, and downstream users.

### Repository architecture

- **Alex** ([OpenAMRobot maintainer](mailto:botshare.ai@gmail.com))
  - Top-level `openamr-platform-sw` monorepo structure (`ros2/src/`, `simulation/`, `config/`, `docs/`, `scripts/`, `tools/`)
  - Initial package scaffolding for `openamrobot_description`, `openamrobot_gazebo`, `openamrobot_nav2`, `openamrobot_docking`
  - Placeholder packages reserved for future work: `openamrobot_bringup`, `openamrobot_control`, `openamrobot_drivers`, `openamrobot_perception`
  - Repository governance scaffolding (`LICENSE`, `CONTRIBUTING.md`, `SECURITY.md`, `NOTICE.md`, `AUTHORS.md`, `CHANGELOG.md`)
  - Initial `gz_simulator.launch.py` and `walled_world.sdf` test world in `openamrobot_gazebo`
  - PR review, scope guidance, repository conventions

### Simulation, navigation and robot description

- **Raj Indulkar** — [@rajindulkar22](https://github.com/rajindulkar22)
  - **Populated `openamrobot_nav2`** from the empty scaffold to a complete Nav2 stack:
    - `config/nav2_params.yaml` — RegulatedPurePursuitController, NavfnPlanner, AMCL, behavior tree, costmaps (global + local with `obstacle_layer + voxel_layer`), velocity smoother, collision monitor.
    - `config/slam.yaml` — SLAM Toolbox configuration for online mapping.
    - `config/scan_body_filter.yaml` — `laser_filters` configuration to mask the robot's own body from the lidar scan.
    - `launch/navigation_launch.py` — Nav2 lifecycle bringup.
    - `launch/localization_launch.py` — AMCL on a saved map.
    - `launch/online_async_launch.py` — SLAM Toolbox online launch.
    - `launch/sim_bringup_launch.py` — composed simulation bringup (localization + navigation + RViz).
    - `maps/my_map.yaml` + `my_map.pgm` — a saved map of the walled world used by AMCL.
    - `rviz/nav2_view.rviz` — the pre-configured RViz layout for the navigation stack.
  - **Extended `openamrobot_description`** to add the front-facing RGB camera:
    - `camera_link` (`base_link + (0.35, 0, 0.20)` m, 4 × 8 × 4 cm box, 0.05 kg).
    - `camera_optical_frame` with the `rpy = (−π/2, 0, −π/2)` ROS optical convention required by `apriltag_ros::solvePnP`.
    - Gazebo `camera` sensor in `gazebo_control.xacro` (640×480 @ 30 Hz, FOV 1.047 rad, topic `/rgb_image`, `gz_frame_id: camera_optical_frame`).
  - **Built the docking scenario inside `openamrobot_gazebo/worlds/walled_world.sdf`**:
    - Inserted the AprilTag dock (`apriltag_dock` model with PBR `<albedo_map>`) on the inner face of the `+x` wall at world `(4.899, 0, 0.5)` yaw=π.
    - Shipped the `apriltag_36h11_id0.png` texture inside `worlds/`.
  - **Extended `openamrobot_gazebo/config/gz_bridge.yaml`** with the camera bridge entries (`/rgb_image`, `/camera/camera_info`).

### Docking pipeline and platform integration

- **Matthieu Vinet** — [@SHuttooo](https://github.com/SHuttooo)
  - **End-to-end integration of the docking pipeline into the platform-software repository.** The simulation, navigation, and robot-description packages were developed independently of the docking package; making them compose into a working end-to-end stack required substantial reconciliation work:
    - Migrated the original 4-phase pipeline from the standalone `openamrobot-docking-main` repository into the integrated `openamr-platform-sw` structure, then **redesigned it as a multi-phase 3-tag bundle pipeline** (see below).
    - Reconciled frame and topic naming conventions between the docking package and the description/simulation packages (`camera_optical_frame` vs `camera_rgb_optical_frame`, `/rgb_image` vs `/camera/image_raw`, `base_link` vs `base_footprint`).
    - Adapted the sequencer parameters to the simulation's world coordinates (centre tag at map `(4.899, 0, 0.5)`, approach yaw `0`, staging distance `2.0 m`).
    - Replaced the inline `apriltag_dock` model in `walled_world.sdf` (which had a hardcoded absolute texture path that did not resolve on any other machine) with a proper Gazebo model directory + `<include>` for portability across machines.
    - Bridged the gz `/camera_info` topic into ROS, then added a **`camera_info_sync.py`** node to stamp camera_info with the image time so `apriltag_ros`'s exact-sync sees pairs (Gazebo publishes image and info at different rates).
    - Restored the wheel-collision-cylinder vs DiffDrive-kinematic-radius mismatch required for ODE traction (without the 1 cm penetration the robot could not move under torque).
    - Extended `GZ_SIM_RESOURCE_PATH` in `openamrobot_docking.launch.py` so `model://apriltag_dock` resolves at world-load time.
    - Diagnosed and fixed the CycloneDDS / FastDDS issue that crashes `dock_trigger.py` silently on ROS 2 Jazzy.
    - **Enabled AMCL kidnap-recovery** (`recovery_alpha_fast: 0.1`, `recovery_alpha_slow: 0.001`) so the robot relocalises after Gazebo drag-and-drop / wheel slip / bumps. Was disabled (`0.0`) upstream.
  - **Bundle docking sequencer** (`ros2/src/openamrobot_docking/scripts/dock_trigger.py`, ~1900 lines), iterated from the earlier 4-phase single-tag design into a multi-phase, **camera-centric 3-tag bundle pipeline**:
    - Phase 1 — Nav2 `NavigateToPose` to the staging zone.
    - Phase 2 — camera-frame closed-loop centring scan on the **midpoint of the outer tags** (id 0 and id 2) + running-average filter on the centre tag (id 1).
    - Phase 3 — **estimate the dock surface normal from the outer tags' wide baseline (0.90 m)**. Back off if arrived too close (`too_close_distance`).
    - Phase 4 — drive to a point on the normal (P1 at `predock_distance`), **re-verify the normal from there**, iterate to P2 (`refined_predock_distance`) if `|N − N'| > normal_tolerance_deg`.
    - Phase 5 — two-regime final approach: FAR — average the 3-tag axis (EMA, depth-weighted) and pure-pursuit it in the camera/tag frame. NEAR (≤ `freeze_axis_distance`) — freeze the axis and finish on the image-frame visual servo on the centre tag, then a blind straight advance (odometry-measured) to `docking_distance` ≈ 0.15 m camera→tag depth.
    - Bypasses `opennav_docking::SimpleChargingDock::controlled_approach` (curved trajectory) for a head-on, predictable approach.
    - Bypasses `nav2_behaviors::Spin` to avoid the costmap-collision false-positive triggered by the lidar glimpsing the robot's own body during fast rotation.
  - **Obstacle guard during drive phases** — LIDAR-cone collision check inside the forward-drive and reverse phases. Pre-check + per-iteration check; wait up to `obstacle_wait_timeout` for the path to clear, then abort. Skipped during Phase 5 (the dock itself is the target).
  - **3-tag dock model** — extended `models/apriltag_dock/model.sdf` to three coplanar 0.20 m panels (outer tags at `y = ±0.45 m`, centre at `y = 0`), with PBR `<albedo_map>` per tag. Updated `config/tags_36h11_sim.yaml` to detect IDs `[0, 1, 2]` with frames `charging_dock_tag_{0, 1, 2}`, `size: 0.16` (= 0.20 m panel × 8/10 black-square edge).
  - **TF → PoseStamped bridge** (`ros2/src/openamrobot_docking/src/detected_dock_pose_publisher.cpp`) republishing the chained `map → charging_dock_tag_1` TF (the **centre tag**, the docking target) as a 10 Hz `/detected_dock_pose`. The outer tags are consumed directly by `dock_trigger.py` for the dock-normal estimate.
  - **Debug markers** — green LINE_STRIP (perpendicular line) and red SPHERE (running-average centre) published as `MarkerArray` for RViz, mirrored inside the Gazebo GUI via the `gz` CLI marker service.
  - **Docking-layer launch** (`ros2/src/openamrobot_docking/launch/openamrobot_docking.launch.py`) composing `apriltag_sim.launch.yml` + `camera_info_sync.py` + `detected_dock_pose_publisher` + `dock_trigger.py` on top of an already-running Gazebo + Nav2 stack.
  - **AprilTag detection assets** — three textures (`apriltag_36h11_id{0,1,2}.png`), refreshed model directory.
  - **CycloneDDS / FastDDS diagnostic** and workaround for the Python action-client crash bug on ROS 2 Jazzy.
  - **15 in-depth engineering documents** under `ros2/src/openamrobot_docking/docs/` (00 → 14): overview, quickstart, architecture, TF frames, AprilTag, parameters, camera calibration, reproduction checklist, sequencer walkthroughs (legacy 4-phase + current bundle), troubleshooting, diagrams, changes from upstream, lessons-learned diary, perception + perpendicular line + RViz/Gazebo markers, and vendor-agnostic precision-docking research (validation matrix, failure modes, calibration, multi-dock).

### Real-robot bring-up, navigation tuning, and hardware integration

- **Matthieu Vinet** — [@SHuttooo](https://github.com/SHuttooo)
  - **Real-robot bring-up** (`openamrobot_bringup`, `openamrobot_drivers`, `openamrobot_perception`): micro-ROS agent + RPLIDAR + EKF + measured static TFs, a top-level `bringup.launch.py` (sim/real selector with `use_camera` / `use_docking`), and a `/goal_pose → /goal_pose_nav` relay for nav-only operation. Took the placeholder packages to a working real-robot stack.
  - **Real-robot Nav2 tuning**: the hardware-tested `nav2_params.yaml` (RotationShimController, costmap/critic tuning) and the **acceleration fix that clears the motor stiction floor** (`acc_lim_theta` 0.5→3.0) which had deadlocked in-place rotation, plus a planner speedup (SmacPlanner2D costmap downsampling, capped planning time).
  - **Real-robot perception**: the LiDAR self-view **scan body filter** tuned to the chassis, camera bring-up, and a 15 fps sensor cap to cut Pi load.
  - **On-demand AprilTag gate** — the detector idles during navigation and is enabled only for the dock approach (via a `SetBool` service), freeing the Pi.
  - **Robot-frame (base_link) dock-normal** final approach — robust to one outer tag dropping out — with **sigma-delta PWM** so sub-stiction yaw corrections still execute, and measured velocity floors.
  - **NEAR-field corrector rewrite** (hardware-validated): real elapsed-time derivative (was a fixed period → jerk on tag reacquisition), fixed-lookahead depth compensation, and stability-weighted axis averaging; plus **continuous camera autofocus** for the close-range image.
  - **Intra-process vision composition** launches (camera + AprilTag in one `component_container_mt`) removing the multi-process / DDS-hop pipeline that starved the detector.
  - **Hardware diagnostics** scripts (`tools/diagnostics/`) and a troubleshooting log.
  - **Real-robot engineering doc series** under `docs/`: `navigation/`, `safety/`, and `real_robot/` (bring-up, networking/DDS, vision pipeline & CPU, compute/thermal, calibration, operator UI, troubleshooting).

### Robot description — upstream geometry and meshes

- **Stephen Brawner** — original author of the SolidWorks-to-URDF Exporter ([sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)) used to generate the OpenAMRobot URDF and STL mesh set.
- **Niraj Dhakal** — original SolidWorks URDF export of the OpenAMRobot mobile base.
- **Raj Indulkar** ([@rajindulkar22](https://github.com/rajindulkar22)) — upstream packaging in [`openamrobot-simulation`](https://github.com/rajindulkar22/openamrobot-simulation) and the first ROS 2 description package skeleton.

Modifications applied in this revision (see *Simulation, navigation and robot description* and *Docking pipeline* above for credits):
- Camera link + optical frame added (Raj).
- DiffDrive plugin kinematic `wheel_radius` set to `0.10 m` while keeping the wheel collision cylinder at `0.11 m` so ODE has the 1 cm contact penetration required for traction.
- Lidar range tuned to the 10 × 10 m walled world (`0.15 m` near limit, `12 m` far limit).

---

## How to be listed here

If you submit a Pull Request that adds a substantive contribution (a new feature, a documented bug fix, a simulation asset, a significant doc rewrite), you may add yourself to the relevant section in the same PR. Trivial changes (typos, formatting) are recognized through GitHub commit history rather than in this file.

When adding yourself, follow the existing format:

```
- **Your Name** — [@your-handle](https://github.com/your-handle)
  - One-line summary of your contribution
  - Bullet points for specific files, features, or design decisions
```

Maintainers may reorganize, condense, or move entries to keep the file readable.

## Attribution policy

Contributors retain attribution for their work through GitHub history and through this file.

By contributing to this repository, contributors agree that their contributions may be used, modified, distributed, sublicensed, and commercialized under the repository license and contribution policy.

See:

- [`LICENSE`](LICENSE)
- [`CONTRIBUTING.md`](CONTRIBUTING.md)
- [`NOTICE.md`](NOTICE.md)
