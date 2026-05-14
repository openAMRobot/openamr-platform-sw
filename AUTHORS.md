# Authors and Contributors

This repository is maintained by the OpenAMRobot organization.

## Maintainer

- OpenAMRobot organization
- Contact: botshare.ai@gmail.com

## Contributors

Recognition is given to contributors whose work has materially shaped this repository. Contributions are grouped by area of focus rather than by chronology. Listing here does not replace GitHub history — it complements it by making non-trivial contributions easy to find for new readers, students, and downstream users.

### Repository Architecture

- **Alex** ([OpenAMRobot maintainer](mailto:botshare.ai@gmail.com))
  - Top-level `openamr-platform-sw` monorepo structure (`ros2/src/`, `simulation/`, `config/`, `docs/`, `scripts/`, `tools/`)
  - Initial package scaffolding for `openamrobot_description`, `openamrobot_gazebo`, `openamrobot_nav2`, `openamrobot_docking`
  - Repository governance scaffolding (`LICENSE`, `CONTRIBUTING.md`, `SECURITY.md`, `NOTICE.md`, `AUTHORS.md`, `CHANGELOG.md`)
  - Initial `gz_simulator.launch.py` and `walled_world.sdf` test world in `openamrobot_gazebo`

### Docking pipeline and simulation

- **Matthieu Vinet** — [@SHuttooo](https://github.com/SHuttooo)
  - **4-phase docking sequencer** (`ros2/src/openamrobot_docking/scripts/dock_trigger.py`), iterated from an initial 7-phase design (filtered detection → parallel spot → rotate-drive → align → advance with auto-calibration) to a final 4-phase pipeline:
    - Nav2 staging → camera-frame centring scan + running-average filter → align spin → pure-pursuit line-tracking + final-align + straight-line approach.
    - Replaces both the discrete reverse-and-realign safety loop and the exponential low-pass auto-calibration with a continuous controller stable around the dock axis.
    - Bypasses `opennav_docking::SimpleChargingDock::controlled_approach` for a head-on, predictable approach.
  - **Docking simulation bringup** (`ros2/src/openamrobot_docking/launch/docking_sim.launch.py`):
    - Composes `openamrobot_gazebo`, `openamrobot_nav2` and the docking-specific pieces shipped in this package.
    - Configurable `spawn_x` / `spawn_y` / `spawn_yaw` arguments with auto-projection of the dock pose into the resulting map frame.
  - **Docking world and dock model** (`ros2/src/openamrobot_docking/worlds/docking_scenario.sdf`, `ros2/src/openamrobot_docking/models/apriltag_dock/`): 10×10 m walled room with a 0.40 m × 0.40 m AprilTag 36h11 panel against the north wall.
  - **AprilTag detection pipeline for the simulation** (`launch/apriltag_sim.launch.yml`, `config/tags_36h11_sim.yaml`).
  - **TF chain fixes** added to `openamrobot_description`: `base_footprint` root, `camera_link`, `camera_rgb_optical_frame` with the −π/2, 0, −π/2 rotation `apriltag_ros::solvePnP` requires. Wheel inertials symmetrised to fix the "robot curves when commanded straight" failure caused by SolidWorks export asymmetries.
  - **Extended `gz_bridge.yaml`** in `openamrobot_gazebo` to include `/camera/image_raw`, `/camera/camera_info`, `/joint_states`.
  - **Nav2 + SLAM tuning** in `openamrobot_nav2/config/nav2_params.yaml` and `slam_toolbox_params.yaml`:
    - NavfnPlanner A* with `tolerance: 1.0`, RegulatedPurePursuitController, costmap `cost_scaling_factor: 8.0`, `velocity_smoother` softened decel for the caster meshes, `collision_monitor.FootprintApproach` disabled.
  - **CycloneDDS / FastDDS diagnostic** and workaround for the Python action-client crash bug on ROS 2 Jazzy.
  - **13 in-depth engineering documents** under `ros2/src/openamrobot_docking/docs/` (00 → 12), including the 24-lesson `12_lessons_learned.md` pedagogical write-up.
  - **Top-level documentation** under `docs/architecture/`, `docs/getting_started/`, and the per-domain READMEs in `docs/docking/`, `docs/simulation/`, `docs/navigation/`.

### Robot description — geometry and meshes

- **Stephen Brawner** — original author of the SolidWorks-to-URDF Exporter ([sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)) used to generate the OpenAMRobot URDF and STL mesh set.
- **Niraj Dhakal** — original SolidWorks URDF export of the OpenAMRobot mobile base.
- **Raj Indulkar** ([@rajindulkar22](https://github.com/rajindulkar22)) — upstream packaging in [`openamrobot-simulation`](https://github.com/rajindulkar22/openamrobot-simulation) and the first ROS 2 description package skeleton.

Modifications applied in this revision (Matthieu Vinet, see *Docking pipeline and simulation* above):
- added `base_footprint` root + `base_joint` for Nav2 TF convention
- symmetrised left/right wheel inertials (off-diagonals zeroed, wheels made identical)
- reconciled `wheel_radius` between DiffDrive plugin and cylinder collisions to 0.10 m
- added `camera_link` + `camera_rgb_optical_frame` with the optical rotation `apriltag_ros::solvePnP` requires
- lidar range adjusted from 0.40-10 m to 0.15-12 m to span the 10×10 m docking scenario
- added a Gazebo RGB camera plugin (640×480 @ 15 Hz)

## How to be listed here

If you submit a Pull Request that adds a substantive contribution (a new feature, a documented bug fix, a simulation asset, a significant doc rewrite), you may add yourself to the relevant section in the same PR. Trivial changes (typos, formatting) are recognized through GitHub commit history rather than in this file.

When adding yourself, follow the existing format:

```
- **Your Name** — [@your-handle](https://github.com/your-handle)
  - One-line summary of your contribution
  - Bullet points for specific files, features, or design decisions
```

Maintainers may reorganize, condense, or move entries to keep the file readable.

## Attribution Policy

Contributors retain attribution for their work through GitHub history and through this file.

By contributing to this repository, contributors agree that their contributions may be used, modified, distributed, sublicensed, and commercialized under the repository license and contribution policy.

See:

- [`LICENSE`](LICENSE)
- [`CONTRIBUTING.md`](CONTRIBUTING.md)
- [`NOTICE.md`](NOTICE.md)
