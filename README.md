# OpenAMR Platform Software

ROS 2 Jazzy software stack for the **OpenAMRobot** mobile robot platform: robot description, Gazebo Harmonic simulation, Nav2 navigation, and AprilTag-bundle-based autodocking (dock + undock) — three AprilTags (family 36h11, IDs 0/1/2) on the dock give a wide-baseline normal for stable yaw alignment.

📖 **[README](README.md)** &nbsp;·&nbsp;
🤝 **[Contributing](CONTRIBUTING.md)** &nbsp;·&nbsp;
🏗️ **[Architecture](docs/architecture/ARCHITECTURE_OVERVIEW.md)** &nbsp;·&nbsp;
🎓 **[ROS 2 course](docs/ROS2_COMPLETE_COURSE.md)** &nbsp;·&nbsp;
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

> [!NOTE]
> This repository is part of the **OpenAMRobot vX.X.X** release.
>
> Download the complete product release (Hardware + Software + Firmware + UI + Documentation) here:
>
> **https://github.com/openAMRobot/openamrobot-release/releases/latest**

---

## Learning ROS 2

New to ROS 2, Gazebo, RViz, launch files, TF, URDF/Xacro, actions, lifecycle nodes, or components? Start with the hands-on [Complete ROS 2 Course for Beginners](docs/ROS2_COMPLETE_COURSE.md). It walks from a fresh ROS 2 Jazzy setup through building nodes, interfaces, simulation, visualization, and a complete differential-drive robot project.

---

## Quickstart — simulation, navigation & docking

> This walks you from a fresh install through the full stack: the Gazebo simulation, Nav2 navigation, and the AprilTag dock/undock pipeline. It is what runs end-to-end today — the repo is broader than this (see [Repository layout](#repository-layout)), but everything below works now.

Choose the setup path that suits you:

| | [Option A — Docker](#option-a--docker-recommended) | [Option B — Manual install](#option-b--manual-install-ubuntu-2404) |
|---|---|---|
| **Best for** | New contributors, quick onboarding | Native development, hardware work |
| **Requires** | Docker + Docker Compose | Ubuntu 24.04 + ROS 2 Jazzy |
| **Effort** | ~5 min | ~30–60 min |
| **GUI (Gazebo/RViz)** | Yes (X11 passthrough) | Yes |

---

## Option A — Docker (recommended)

### What is Docker?

Docker is a tool that packages software and all its dependencies into a self-contained **container** — like a lightweight virtual machine that shares your kernel. You don't install ROS 2, Gazebo, or Nav2 on your host; Docker does it inside the container automatically. Every contributor runs the exact same environment, so _"it works on my machine"_ stops being a problem.

**You only need two things installed on your host:** Docker and Docker Compose.

### A.1 Install Docker

Follow the official install guide for your OS: <https://docs.docker.com/engine/install/>

Then install the Compose plugin (usually bundled with Docker Desktop; on Linux run):

```bash
sudo apt install docker-compose-plugin
```

Verify everything works:

```bash
docker --version          # Docker version 24.x or later
docker compose version    # Docker Compose version v2.x or later
docker info               # confirms your user can talk to the Docker daemon
```

> **Permission tip:** add yourself to the `docker` group so you don't need `sudo` on every command:
> ```bash
> sudo usermod -aG docker $USER
> newgrp docker
> ```
> If `docker info` still says `permission denied`, fully log out and log back in
> (or reboot) so the new group membership is applied.

### A.2 Clone the repo

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
```

### A.3 Allow Docker to open GUI windows

Gazebo and RViz need access to your host display:

```bash
xhost +local:docker
```

Run this once per host session (or add it to your `~/.bashrc`).

### A.4 Build the image

Run Compose commands from the repository root, where `docker-compose.yml` lives:

```bash
cd openamr-platform-sw
docker compose build
```

If you see `no configuration file provided: not found`, you are in the wrong
directory; `cd` back to the repository root and rerun the command.

This downloads the ROS 2 Jazzy base image and installs every dependency. First build takes **5–10 minutes**; subsequent builds reuse the cached layers and are nearly instant.

### A.5 Launch the full simulation stack

```bash
docker compose run --rm openamr \
  ros2 launch openamrobot_docking bringup_sim.launch.py
```

Gazebo and RViz windows will appear on your host screen. Jump to [Drive the robot](#drive-the-robot) once the stack is up.

### A.6 Open an interactive shell (for development)

```bash
docker compose run --rm openamr bash
```

The `ros2/src/` directory is **bind-mounted** into the container — edits you make on the host are immediately visible inside, and vice versa. After changing code, rebuild only the affected package:

```bash
# inside the container
colcon build --symlink-install --packages-select openamrobot_docking
source install/setup.bash
```

No need to rebuild the Docker image unless you add a new apt dependency to the `Dockerfile`.

### A.7 Run layers separately (optional)

Same as the manual path below — just prefix each command with `docker compose run --rm openamr`:

```bash
docker compose run --rm openamr ros2 launch openamrobot_gazebo gz_simulator.launch.py
docker compose run --rm openamr ros2 launch openamrobot_nav2 sim_bringup_launch.py
docker compose run --rm openamr ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

All containers use `network_mode: host`, so DDS discovery works between them automatically.

> **CycloneDDS:** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is already set in [`docker-compose.yml`](docker-compose.yml) — you don't need to do anything extra. See [Why CycloneDDS](#why-cyclonedds) for the reason.

> **NVIDIA GPU:** If you have an NVIDIA GPU and want hardware-accelerated Gazebo rendering, install [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) and uncomment the `openamr-gpu` service in [`docker-compose.yml`](docker-compose.yml).

---

## Option B — Manual install (Ubuntu 24.04)

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

---

## 4. Drive the robot

> Both paths (Docker and manual) end up here once the stack is running.

Wait ~10 s for Nav2 to localize, then from any sourced terminal (or `docker compose run --rm openamr bash` if using Docker):

```bash
ros2 topic pub /dock_trigger  std_msgs/msg/Bool "{data: true}" --once   # dock
ros2 topic pub /undock_robot  std_msgs/msg/Bool "{data: true}" --once   # undock: reverse 0.7 m + spin 180°
```

You can also send a navigation goal (RViz **"2D Goal Pose"**, or a `PoseStamped` on `/goal_pose`): if the robot is docked it **undocks first**, then drives to the goal. The robot navigates to a staging zone, finds the 3-tag bundle, estimates the dock normal from the outer tags' wide baseline, follows the normal axis on a pure-pursuit, then finishes with an axis-frozen visual servo on the centre tag — ending perpendicular to the dock, aligned for charging.

For a step-by-step walkthrough with diagnostics: [`ros2/src/openamrobot_docking/docs/01_quickstart.md`](ros2/src/openamrobot_docking/docs/01_quickstart.md).

---

## Why CycloneDDS

The default Jazzy RMW (FastDDS) has a Python-side crash bug that makes the docking sequencer (`dock_trigger.py`) exit silently when sending Nav2 action goals. CycloneDDS must be used instead.

**Docker (Option A):** already handled — `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set in [`docker-compose.yml`](docker-compose.yml). Nothing to do.

**Manual install (Option B):** add it to your shell profile once:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

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
├── docker/                               Docker support files
│   └── entrypoint.sh                     Sources ROS 2 + workspace on container start
├── config/                               (reserved) product-level config: robot/ nav2/ docking/ simulation/
├── simulation/                           (reserved) cross-package assets: models/ worlds/ scenarios/
├── docs/                                 (reserved) platform docs: getting_started/ architecture/ navigation/ docking/ simulation/ safety/
├── scripts/                              (reserved) operator utilities
├── tools/                                (reserved) developer tools
│
├── Dockerfile                            Builds the containerised ROS 2 + Gazebo + Nav2 environment
├── docker-compose.yml                    Runs the container with GUI, GPU, and volume config
├── .dockerignore                         Keeps build context lean (excludes build/ install/ log/)
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

## Engineering deep dives

Beyond per-package READMEs, three doc series record the *why* behind the real-robot work —
tuned values, what was tried and rejected, and the gaps that remain:

| Series | Covers |
|---|---|
| [`docs/navigation/`](docs/navigation/README.md) | The Nav2 stack on real hardware: architecture, costmaps, planner/controller tuning, goal routing, troubleshooting |
| [`docs/safety/`](docs/safety/README.md) | The motion-safety envelope: collision monitor, speed/accel limits and watchdogs, and known gaps (no hardware E-stop yet) |
| [`docs/real_robot/`](docs/real_robot/README.md) | Taking the stack from Gazebo onto real hardware: bring-up, networking/DDS, vision pipeline & CPU budget, thermal limits, calibration, operator UI |

---

## Roadmap / TODO

- [x] **Forward obstacle guard during dock approach.** ✅ Done — `dock_trigger.py` runs a LIDAR-cone check on `/scan_filtered` during the dock drive phases (pre-check + per-iteration). The dock and undock phases still publish straight to `/cmd_vel` (bypassing Nav2's `collision_monitor`), so this guard is the safety net. See [`docs/05_parameters.md`](ros2/src/openamrobot_docking/docs/05_parameters.md) "Obstacle guard during drive phases" for the parameters and the empirical calibration of the body filter.
- [ ] **Rear obstacle awareness during undock.** Not done. The LIDAR sits on top of the chassis and `scan_body_filter` chops the rear ±40° angular sector (where the body would otherwise reflect every ray), so a rear cone check on `/scan_filtered` reads +inf and would be a no-op. Prerequisite: add a rear sensor (bumper, rear camera, sonar). Then re-introduce a backward `_wait_for_path_clear` in `_reverse_distance` and `run_undock_sequence`.
- [ ] **Higher-precision docking (target ±10 mm / ±2°, ~99.99 % reliability).** The current 3-tag bundle delivers ~1–2 cm laterally and ~1° in yaw in simulation, which is solid but not yet production-grade across lighting and pose variations. The full research — vendor-agnostic target derivation, sensing-method catalogue, validation protocol, failure modes, calibration & commissioning, multi-dock handling — is in [`ros2/src/openamrobot_docking/docs/14_docking_research.md`](ros2/src/openamrobot_docking/docs/14_docking_research.md). Next concrete step: execute the §9 bench validation matrix once hardware lands.

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

## 💜 Support OpenAMRobot

Support open-source robotics, ROS 2 development, AI robotics education, and dual-arm mobile robot research.

### ⚡ Back the build — one-time, no strings

| Tier | What it says about you | Link |
|---|---|---|
| ⚡ **First Mover - €5** | You got here first, and you didn't overthink it. Your name goes on the backers wall - permanently - as one of the people who moved before it was obvious. Five euros, one good instinct. | <a href="https://buy.stripe.com/eVqcN5b99eeAaSd7WPgUM06" target="_blank" rel="noopener noreferrer">💳&nbsp;Back&nbsp;it&nbsp;→</a> |
| 🎯 **Sharpshooter - €25** | You spotted it early and called it. Name on the wall + a shareable "OpenAMRobot Backer" badge - proof you saw it coming while everyone else was still scrolling. | <a href="https://buy.stripe.com/4gMdR9ell1rO2lH90TgUM07" target="_blank" rel="noopener noreferrer">💳&nbsp;Back&nbsp;it&nbsp;→</a> |
| 🕶️ **Insider - €50** | You want in behind the curtain. Everything above + the backer-only build log and early files - every breakthrough, every faceplant, unfiltered. You see it before the internet does. | <a href="https://buy.stripe.com/eVq14nfpp4E0gcx2CvgUM08" target="_blank" rel="noopener noreferrer">💳&nbsp;Back&nbsp;it&nbsp;→</a> |
| 🔩 **Immortal - €100** | Your name goes on the actual robot. Physically. Forever. A machine will roll around carrying your name long after any of us remember why - and you'll have the photo to prove you were there. | <a href="https://buy.stripe.com/00w00jdhhb2o4tPfphgUM09" target="_blank" rel="noopener noreferrer">💳&nbsp;Back&nbsp;it&nbsp;→</a> |
| 🏆 **Founding Backer - €250** | Not a supporter - a co-author. Everything above + a personal thank-you in a build video. When this becomes something, you were one of the people who decided it would. | <a href="https://buy.stripe.com/28EeVdcdddawaSdeldgUM0a" target="_blank" rel="noopener noreferrer">💳&nbsp;Back&nbsp;it&nbsp;→</a> |

### 🔁 Monthly subscriptions — build it with us, every month

| Tier | What you get | Link |
|---|---|---|
| 😇 **Benefactor - €5/mo** | This month, officially not wasted. €5 to help build an open robot for everyone - cheaper than the coffee you'll forget you bought. Your name goes on the wall. History will remember you - well, me for sure. 🤖 | <a href="https://buy.stripe.com/9B6cN5dhh9Yk7G1cd5gUM05" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |
| ❤️ **Community - €19/mo** | You're in. Community access, project & roadmap updates, basic documentation, and community Q&A. (Private consultation not included.) | <a href="https://buy.stripe.com/6oUcN55OPc6s3pL4KDgUM00" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |
| 🔧 **Builder - €79/mo** | For the ones who actually build. Everything in Community + builder docs, monthly group Q&A, selected tutorials, early design updates, and discounts on digital packs. (Private consultation not included.) | <a href="https://buy.stripe.com/14A28r0uvdaw9O9eldgUM01" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |
| 🚀 **Pro Support - €299/mo** | Expert support for advanced builders, early founders, and small labs. Includes 1 private consulting call per month and up to 3 hours/month of technical guidance. | <a href="https://buy.stripe.com/dRm4gz4KLdaw6BX4KDgUM02" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |
| 🏢 **Startup Support - €750/mo** | For robotics startups and teams heading toward a prototype. Includes 2 private consulting calls per month, roadmap support, GitHub/documentation review, supplier review, and up to 6 hours/month. | <a href="https://buy.stripe.com/7sY8wPfpp8Ugf8t90TgUM03" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |
| 🔬 **Lab Support - €1,500/mo** | For universities, corporate labs, and training centers. Includes 4 private sessions per month, lab implementation support, architecture reviews, training-roadmap support, and up to 10 hours/month. | <a href="https://buy.stripe.com/eVq14ndhh2vSaSda4XgUM04" target="_blank" rel="noopener noreferrer">💳&nbsp;Subscribe&nbsp;→</a> |

**❤️ GitHub Sponsors:** <a href="https://github.com/sponsors/openAMRobot" target="_blank" rel="noopener noreferrer"> 🐙 &nbsp;github.com/sponsors/openAMRobot&nbsp;→</a>

*Every contribution — €5 or €1,500 — literally builds this robot. No billion-dollar lab required. **You're not donating. You're building it.** 🤖*

---

## License

MIT. See [`LICENSE`](LICENSE). Attribution for bundled third-party assets (AprilTag panel texture, etc.) is in [`NOTICE.md`](NOTICE.md).

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement governing assignment of transferable economic rights to Botshare LTD. Contributor attribution and legally non-waivable authorship or moral rights remain recognized.

See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
