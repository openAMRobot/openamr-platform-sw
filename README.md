# OpenAMR Platform Software

ROS 2 Jazzy software stack for the **OpenAMRobot** mobile robot platform: robot description, Gazebo Harmonic simulation, Nav2 navigation, and AprilTag-based autodocking (dock + undock).

📖 **[README](README.md)** &nbsp;·&nbsp;
🤝 **[Contributing](CONTRIBUTING.md)** &nbsp;·&nbsp;
🏗️ **[Architecture](docs/architecture/ARCHITECTURE_OVERVIEW.md)** &nbsp;·&nbsp;
🛠️ **[Developer setup](docs/getting_started/DEVELOPER_SETUP.md)** &nbsp;·&nbsp;
🧰 **[Troubleshooting](docs/getting_started/TROUBLESHOOTING.md)** &nbsp;·&nbsp;
✅ **[Testing](docs/getting_started/TESTING_GUIDE.md)** &nbsp;·&nbsp;
🌱 **[Git guide](docs/getting_started/GIT_GUIDE.md)** &nbsp;·&nbsp;
📜 **[License](LICENSE)** &nbsp;·&nbsp;
🔒 **[Security](SECURITY.md)** &nbsp;·&nbsp;
👥 **[Authors](AUTHORS.md)** &nbsp;·&nbsp;
📝 **[Changelog](CHANGELOG.md)** &nbsp;·&nbsp;
ℹ️ **[Notice](NOTICE.md)**

> 📦 **Status: experimental.** Tuned end-to-end in the docking simulation. Real-robot bringup (drivers, control, hardware integration) is in progress and will land under the placeholder packages described below.

---

## Quickstart — simulation, navigation & docking

> This walks you from a fresh install through the full stack: the Gazebo simulation, Nav2 navigation, and the AprilTag dock/undock pipeline. It is what runs end-to-end today — the repo is broader than this (see [Repository layout](#repository-layout)), but everything below works now.

### 1. Prerequisites

- **Ubuntu 24.04 (Noble)**, native install (Gazebo Harmonic needs a Linux display server).
- **ROS 2 Jazzy** installed system-wide.
- **Gazebo Harmonic** (`gz-sim 8.x`), provided by `ros-jazzy-ros-gz-sim`.

One-time package install:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-amcl ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-slam-toolbox ros-jazzy-laser-filters \
  ros-jazzy-apriltag-ros ros-jazzy-image-proc \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-topic-tools ros-jazzy-rviz2 \
  python3-colcon-common-extensions

# CycloneDDS is required — see "Why CycloneDDS" below.
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone + build

The colcon workspace is the **`ros2/`** sub-directory (it is the folder that contains `src/`), so build from there:

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw/ros2          # the colcon workspace root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install        # creates build/ install/ log/ inside ros2/
source install/setup.bash
```

Sourcing does **not** carry over between terminals, so in **every** new terminal run (from `ros2/`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 3a. Launch everything with one command

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

This brings up the whole stack in the right order with a delay between each layer (Gazebo, then Nav2 at +8 s, then docking at +16 s). On a slower machine, widen the gaps:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=22
```

For headless simulation without Gazebo GUI or RViz:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

### 3b. …or run the three layers separately

Useful while tuning or restarting one layer without bringing the others down. **The order matters** — each layer depends on the one before it, so start them top to bottom (one sourced terminal each):

```bash
# 1. Simulation — Gazebo + robot + ros<->gz bridge.
#    Must be first: it owns /clock, spawns the robot, and bridges /scan,
#    /odom, /rgb_image, /cmd_vel. Nothing else has data until this is up.
ros2 launch openamrobot_gazebo gz_simulator.launch.py

# 2. Navigation + RViz — Nav2 + AMCL on the saved map + the RViz view.
#    Needs the simulator's /scan, /odom and /clock to localize the robot
#    on the map; RViz lets you watch it and send "2D Goal Pose" goals.
ros2 launch openamrobot_nav2 sim_bringup_launch.py

# 3. Docking — AprilTag detection + the dock/undock sequencer.
#    Needs the camera (/rgb_image) from layer 1 and the navigate_to_pose
#    action + TF tree from layer 2.
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

> If a launch fails with `package 'openamrobot_...' not found`, you forgot `source install/setup.bash` in that terminal.

### 4. Drive the robot

Wait ~10 s for Nav2 to localize, then from any sourced terminal:

```bash
ros2 topic pub /dock_trigger  std_msgs/msg/Bool "{data: true}" --once   # dock
ros2 topic pub /undock_robot  std_msgs/msg/Bool "{data: true}" --once   # undock: reverse 1.5 m + spin 180°
```

You can also send a navigation goal (RViz **"2D Goal Pose"**, or a `PoseStamped` on `/goal_pose`): if the robot is docked it **undocks first**, then drives to the goal. The robot navigates to a staging zone, scans for the AprilTag, aligns perpendicular to it, then drives onto the dock — ending ~90 cm from the tag, perpendicular.

For a step-by-step walkthrough with diagnostics: [`ros2/src/openamrobot_docking/docs/01_quickstart.md`](ros2/src/openamrobot_docking/docs/01_quickstart.md).

---

## Why CycloneDDS

The default Jazzy RMW (FastDDS) has a Python-side crash bug that makes the docking sequencer (`dock_trigger.py`) exit silently when sending Nav2 action goals. Always export:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Put it in your `~/.bashrc` once and forget about it.

---

## Repository layout

```
openamr-platform-sw/
├── ros2/                                 ROS 2 colcon workspace (build from here)
│   └── src/
│       ├── openamrobot_description/      URDF + meshes + Gazebo sensor plugin tags
│       ├── openamrobot_gazebo/           Gazebo Harmonic bringup + ros↔gz bridge + worlds
│       ├── openamrobot_nav2/             Nav2 stack + AMCL + map + RViz layout
│       ├── openamrobot_docking/          AprilTag + dock/undock sequencer + dock model
│       ├── openamrobot_bringup/          (placeholder) top-level launch compositions
│       ├── openamrobot_control/          (placeholder) ros2_control + low-level control
│       ├── openamrobot_drivers/          (placeholder) hardware drivers (lidar, camera, IMU…)
│       └── openamrobot_perception/       (placeholder) perception beyond docking
│
├── config/                               (reserved) product-level config: robot/ nav2/ docking/ simulation/
├── simulation/                           (reserved) cross-package assets: models/ worlds/ scenarios/
├── docs/                                 (reserved) platform docs: getting_started/ architecture/ navigation/ docking/ simulation/ safety/
├── scripts/                              (reserved) operator utilities
├── tools/                                (reserved) developer tools
│
├── README.md                             you are here
├── CONTRIBUTING.md · SECURITY.md · NOTICE.md · AUTHORS.md · CHANGELOG.md
└── LICENSE                               MIT
```

The four `(placeholder)` packages are folder + `README.md` markers; they don't build yet. They reserve the architectural slot for upcoming real-robot work. The reserved root directories currently hold only `.gitkeep` markers — engineering docs and configs live next to their code under `ros2/src/`.

---

## Package responsibilities

Strict separation of concerns:

| Package | Owns | Does NOT own |
|---|---|---|
| `openamrobot_description` | Robot URDF, meshes, mass/inertia, Gazebo sensor plugin tags | Worlds, navigation, docking |
| `openamrobot_gazebo` | Simulator bringup, ros↔gz bridge, generic + scenario worlds | Robot model, navigation, docking |
| `openamrobot_nav2` | Nav2 params, AMCL on a saved map, RViz layout | Gazebo, docking |
| `openamrobot_docking` | AprilTag detection, dock model, dock/undock sequencer, the one-command sim bringup | Robot, simulator, navigation stack |

Each package may **reference** sibling packages at launch composition time (`FindPackageShare` + `IncludeLaunchDescription`), but **must not duplicate** their files.

---

## Architecture — the `/cmd_vel` flow

In the docking simulation, the chain from a velocity command to the wheels is:

```
dock_trigger.py / Nav2 controller       ──>  /cmd_vel
                                              │
                                              ▼
                          ros_gz_bridge  ──>  gz /cmd_vel
                                              │
                                              ▼
                          DiffDrive plugin    (Gazebo, applies torques to joints)
                                              │
                                              ▼
                          ODE contact solver  (friction, the robot moves)
                                              │
                                              ▼
                          gz odom + tf  ──>   /odom, /tf  (via bridge)
                                              │
                                              ▼
                          robot_state_publisher fills in base_link → camera_optical_frame → …
```

If any link breaks, the robot stops moving. The most common failure mode is the bridge not forwarding `/cmd_vel` (verify with `ros2 topic info /cmd_vel`). The full TF chain on top of this is in [`ros2/src/openamrobot_docking/docs/03_tf_frames.md`](ros2/src/openamrobot_docking/docs/03_tf_frames.md).

---

## Per-package documentation

Each package ships its own README. The deep engineering docs live under [`ros2/src/openamrobot_docking/docs/`](ros2/src/openamrobot_docking/docs/) (numbered files: overview, quickstart, architecture, TF chain, AprilTag setup, parameters, troubleshooting, lessons learned).

| Package | README |
|---|---|
| `openamrobot_description` | [`ros2/src/openamrobot_description/README.md`](ros2/src/openamrobot_description/README.md) |
| `openamrobot_gazebo` | [`ros2/src/openamrobot_gazebo/README.md`](ros2/src/openamrobot_gazebo/README.md) |
| `openamrobot_nav2` | [`ros2/src/openamrobot_nav2/README.md`](ros2/src/openamrobot_nav2/README.md) |
| `openamrobot_docking` | [`ros2/src/openamrobot_docking/README.md`](ros2/src/openamrobot_docking/README.md) |

For the docking pipeline specifically, start with [`docs/01_quickstart.md`](ros2/src/openamrobot_docking/docs/01_quickstart.md).

---

## Roadmap / TODO

- [ ] **Obstacle-aware dock & undock.** Today the dock and undock phases bypass Nav2 — `dock_trigger.py` publishes straight to `/cmd_vel`, so the lidar, costmaps and collision monitor are not in the loop during those maneuvers. Bring obstacle awareness back into the dock/undock phases (lidar-based collision checking and/or routing the maneuver through Nav2) so the robot stops or replans if something enters its path while approaching or leaving the dock.
- [ ] **Higher-precision docking (target ~99.99% reliability).** The current 4-phase approach lands within a few centimetres laterally and ~1° in yaw, which is good but not yet production-grade. Investigate a more reliable approach (e.g. tighter visual-servo final stage, multi-tag or larger-tag geometry, better camera calibration, sensor fusion) so docking succeeds essentially every time across lighting and pose variations.

---

## Related repositories (organisation-level)

This repo is the **software** side of the OpenAMRobot platform. Sister repositories cover the rest of the stack:

| Repo | Contents |
|---|---|
| `openamr-platform-hw` | Mechanical CAD, BOM, wiring, electrical |
| `openamr-platform-fw` | Embedded firmware (STM32, Teensy, ESP32) |
| `openamrobot-docs` | Organisation-wide user-facing documentation |
| `openamrobot-interfaces` | Shared ROS 2 messages, services, actions |
| `openamrobot-comm` | Shared communication contracts |
| `openamrobot-ui` | Operator interface (publishes `/dock_trigger` on docking) |

---

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

Key rule: a contribution should modify **only the package or directory related to the task**. For example, an autodocking change should normally touch only `ros2/src/openamrobot_docking/`. If it *requires* a touch in a sibling package (e.g. a remap that the docking gate depends on), describe the necessity in the PR.

Architecture, roadmap, and collaboration: <https://github.com/orgs/openAMRobot/discussions>

---

## Safety notice

This repository may affect real-robot behaviour. Users are responsible for validating robot safety (E-stop, watchdog, fault handling), navigation, docking, motor control, sensor integration, deployment suitability, and regulatory compliance.

This software is provided for research, education, and development.

---

## License

MIT. See [`LICENSE`](LICENSE). Attribution for bundled third-party assets (AprilTag panel texture, etc.) is in [`NOTICE.md`](NOTICE.md).
