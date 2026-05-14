# Quickstart — From zero to docking

This document walks you from a fresh Ubuntu install to a working docking sequence. Follow it linearly the first time. Each step is independent and copy-pastable.

For background on what this package does and how it works, see [`00_overview.md`](00_overview.md) first.

---

## Prerequisites

### System requirements

The simulation needs:
- **Ubuntu 24.04 (Noble)** — native install. WSL2 and macOS/Windows are not supported (Gazebo Harmonic requires Linux with a display server).
- **ROS 2 Jazzy** installed system-wide (see [docs.ros.org/en/jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for the official install).
- **Gazebo Harmonic** (`gz-sim 8.x`) — comes with `ros-jazzy-ros-gz-sim`.
- **A working display server** (X11 or Wayland). Headless environments (Codespaces, CI runners) cannot run Gazebo's GUI.

The real-robot pipeline needs:
- Same Ubuntu 24.04 + ROS 2 Jazzy.
- A USB camera (UVC-compatible).
- A printed AprilTag 36h11 ID 0 of measured size.
- A robot with `/cmd_vel` accepting `geometry_msgs/Twist` and publishing `/odom`, `/tf` (`map → odom → base_footprint`), and `/scan`.

> **About the devcontainer:** the bundled `.devcontainer/` is intended for **editing code and `colcon build`** only — not for running the simulation. Gazebo and RViz are GUI applications and are not configured for the devcontainer. Run everything from a host Linux system with display access.

---

## 1. Install system dependencies

Open a terminal and paste:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-opennav-docking \
  ros-jazzy-opennav-docking-msgs \
  ros-jazzy-apriltag-ros \
  ros-jazzy-image-proc \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions
```

For the **simulation only**, also install:

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2
```

For the **real robot only**, also install:

```bash
sudo apt install -y \
  ros-jazzy-camera-ros \
  ros-jazzy-camera-calibration
```

---

## 2. Set up CycloneDDS (required)

The default FastDDS on Jazzy has a Python-side crash bug that silently kills `dock_trigger.py` when it sends action goals. Use CycloneDDS instead.

In every terminal that will run ROS commands:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

To make this automatic for every shell, add the line to your `~/.bashrc`:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $RMW_IMPLEMENTATION
# Expected output: rmw_cyclonedds_cpp
```

If this prints anything else (empty, or `rmw_fastrtps_cpp`), the export didn't take effect. See [`09_troubleshooting.md`](09_troubleshooting.md) "FastDDS Python crash".

---

## 3. Clone and build the package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/openAMRobot/openamrobot-docking.git
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash
colcon build --packages-select openamrobot_docking
source install/setup.bash
```

Verify the build:

```bash
ros2 pkg list | grep openamrobot
# Expected output: openamrobot_docking
```

> **Note:** in every fresh terminal, you need to re-source the environment:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/ros2_ws/install/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```
> Add these three lines to your `~/.bashrc` to skip this step.

---

## 4a. First run — Simulation (recommended)

This is the fastest way to see the docking pipeline working.

### Launch the full stack

```bash
ros2 launch openamrobot_docking simulation.launch.py
```

This starts Gazebo (server + GUI), `robot_state_publisher`, `ros_gz_bridge`, `slam_toolbox` (4 s delay), Nav2, `apriltag_node`, `detected_dock_pose_publisher`, `dock_trigger.py`, and RViz.

Wait ~15 seconds. You should see in the logs:
- `slam_toolbox` reaching "active"
- All Nav2 lifecycle nodes reaching "active"

If RViz opens with the robot model visible and a map starting to populate, you're good.

### Verify the stack before triggering

In a second terminal (with the same environment sourced and `RMW_IMPLEMENTATION` exported):

```bash
ros2 topic hz /camera/image_raw          # should be ~15 Hz
ros2 topic echo /detected_dock_pose --once   # should show a pose with x≈4.0, y≈8.9
ros2 lifecycle nodes                     # every Nav2 node should be 'active'
```

If `/detected_dock_pose` returns nothing, the AprilTag isn't being detected. Check that the robot has line of sight to the tag in the Gazebo viewport.

### Trigger the docking sequence

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Watch the `dock_trigger` logs in the first terminal. You should see the 4 phases scroll by:

```
── Phase 1/4: NavigateToPose → staging zone
── Phase 2/4: tag search + initial filter (40 samples)
   scanning to centre tag in camera (tolerance ±2.0°, need 5 consecutive frames)
   tag centred in camera (image_angle=-1.5°, consecutive=5)
   tag at (4.000, 8.900) after 40 samples
── Phase 3/4: ALIGN (spin to perpendicular yaw 1.571)
   spin done: yaw=1.582 target=1.571 err=-0.011
── Phase 4/4: line-tracking advance to 0.90m
   start d_to_tag=2.500m, forward to travel ≈ 1.600m
   d=1.40m < 1.40m — final align then straight-line approach
   reached: d_to_tag=0.900m, lateral=+0.4cm (340 samples averaged)
Docking sequence complete ✓
```

End state: the robot is stopped ~90 cm in front of the AprilTag, perpendicular to the tag plane.

### Clean restart between runs

If a previous launch left zombie processes and you can't relaunch cleanly:

```bash
~/ros2_ws/src/openamrobot-docking/openamrobot_docking/scripts/kill_sim.sh
```

This SIGKILLs every process spawned by the simulation launch file.

---

## 4b. First run — Real robot

### 4b.1 — Calibrate the camera

Follow [`06_camera_calibration.md`](06_camera_calibration.md) end-to-end. Output: a calibration file at `~/.ros/camera_info/<camera_name>.yaml`. Skip if your camera is already calibrated.

### 4b.2 — Measure and configure the AprilTag

Print a 36h11 tag of any reasonable size (5–10 cm works for most setups). Measure the outer black-square edge in metres and update [`config/tags_36h11.yaml`](../config/tags_36h11.yaml):

```yaml
size: 0.0555    # ← replace with YOUR measured value
```

### 4b.3 — Verify camera + AprilTag detection

```bash
ros2 launch openamrobot_docking apriltag.launch.yml
```

In a second terminal:

```bash
ros2 topic echo /apriltag/detections --once   # should detect tag ID 0
ros2 run tf2_ros tf2_echo camera_rgb_optical_frame charging_dock_apriltag
```

If detection works, stop this launch (Ctrl-C) before the next step.

### 4b.4 — Place the AprilTag and measure its pose in your map

You need a pre-built map (typically from SLAM Toolbox or `slam_gmapping`) and AMCL running for localization. With the robot localized, drive it to the dock and read the TF:

```bash
ros2 run tf2_ros tf2_echo map charging_dock_apriltag
```

Use the printed `(x, y, yaw)` to update [`config/openamrobot_docking.yaml`](../config/openamrobot_docking.yaml):

```yaml
home_dock:
  frame: map
  pose: [X, Y, YAW]    # ← from tf2_echo
```

See [`04_apriltag.md`](04_apriltag.md) for the full dock-placement procedure.

### 4b.5 — Launch the full real-robot stack

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

Wait for all lifecycle nodes to reach "active", then trigger:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The real-robot pipeline uses `opennav_docking::SimpleChargingDock::controlled_approach`, not the 4-phase sequencer. See [`02_architecture.md`](02_architecture.md) for the difference.

**Important safety note:** during the first attempts, stay near the robot. Collision detection is disabled during the final approach (the dock itself would otherwise trigger it).

---

## What to read next

| Topic | Read |
| :---- | :---- |
| Deeper understanding of the pipeline | [`02_architecture.md`](02_architecture.md) |
| All available parameters and how to tune them | [`05_parameters.md`](05_parameters.md) |
| The custom 4-phase sequencer in detail | [`08_sequencer_4phase.md`](08_sequencer_4phase.md) |
| Something broke — diagnostic table | [`09_troubleshooting.md`](09_troubleshooting.md) |
| Visual reference (diagrams) | [`10_diagrams.md`](10_diagrams.md) |
| Why we made certain design choices | [`12_lessons_learned.md`](12_lessons_learned.md) |

For the full documentation index, see [`README.md`](README.md).
