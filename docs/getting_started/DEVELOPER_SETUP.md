# Developer Setup Guide

This guide explains how to set up a local development environment for `openamr-platform-sw`.

The README gives the fastest path to run the simulation. This document gives more detail for contributors who need to build, rebuild, debug, and work on the repository regularly.

---

## 1. Supported System

Recommended development environment:

| Item | Version |
|---|---|
| Operating system | Ubuntu 24.04 Noble |
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Build tool | colcon |
| DDS/RMW | CycloneDDS |

Native Ubuntu is recommended. Gazebo and RViz need graphical support, so a full Linux desktop environment is easier than WSL or a headless machine.

---

## 2. Repository Structure

The Git repository root is:

```text
openamr-platform-sw/
```

The ROS 2 workspace is inside:

```text
openamr-platform-sw/ros2/
```

This matters because `colcon build` should be run from `ros2/`, not from the repository root.

Correct:

```bash
cd openamr-platform-sw/ros2
colcon build --symlink-install
```

Incorrect:

```bash
cd openamr-platform-sw
colcon build --symlink-install
```

---

## 3. Install Required Packages

Update package lists:

```bash
sudo apt update
```

Install the ROS 2, Gazebo, navigation, docking, and development packages used by the current simulation stack:

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-amcl ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-slam-toolbox ros-jazzy-laser-filters \
  ros-jazzy-apriltag-ros ros-jazzy-image-proc \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-topic-tools ros-jazzy-rviz2 \
  python3-colcon-common-extensions
```

This assumes ROS 2 Jazzy is already installed and available at `/opt/ros/jazzy/`.

Check that the ROS 2 environment is available:

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
ros2 pkg list
```

The first command should print `jazzy`.

---

## 4. Configure CycloneDDS

This project uses CycloneDDS for the current docking simulation workflow.

Set it in your current terminal:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

To make it available in every new terminal:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

Check it:

```bash
echo $RMW_IMPLEMENTATION
```

Expected output:

```text
rmw_cyclonedds_cpp
```

---

## 5. Clone the Repository

For read-only use:

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
```

For contribution work, fork the repository first, then clone your fork:

```bash
git clone https://github.com/YOUR_USERNAME/openamr-platform-sw.git
cd openamr-platform-sw
git remote add upstream https://github.com/openAMRobot/openamr-platform-sw.git
```

For the full Git and fork workflow, see:

```text
docs/getting_started/GIT_GUIDE.md
```

---

## 6. Build the Workspace

Go to the ROS 2 workspace:

```bash
cd ros2
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Build:

```bash
colcon build --symlink-install
```

Source the local workspace:

```bash
source install/setup.bash
```

Why `--symlink-install` is useful:

- Python files and launch files update more easily during development.
- You do not need a full rebuild after every small script or launch-file edit.
- It is the normal development mode for ROS 2 workspaces.

---

## 7. Source the Environment in Every New Terminal

Every new terminal needs the environment again.

From `openamr-platform-sw/ros2`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If you forget this, ROS 2 may not find the packages:

```text
package 'openamrobot_...' not found
```

---

## 8. Verify the Build

Check that packages are visible:

```bash
ros2 pkg list | grep openamrobot
```

Expected packages include:

```text
openamrobot_description
openamrobot_docking
openamrobot_gazebo
openamrobot_nav2
```

Launch the full simulation stack:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

If your machine is slow, use larger startup delays:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=22
```

For a lighter headless run:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

---

## 9. Run the Main Layers Separately

Running layers separately is useful during debugging.

Open three sourced terminals from `ros2/`.

Terminal 1, simulation:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Terminal 2, navigation:

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py
```

Terminal 3, docking:

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

The order matters. Start simulation first, then navigation, then docking.

---

## 10. Test Dock and Undock

From a sourced terminal:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

To undock:

```bash
ros2 topic pub /undock_robot std_msgs/msg/Bool "{data: true}" --once
```

Useful checks:

```bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /odom --once
ros2 action list
```

---

## 11. Rebuild After Changes

For many Python or launch-file changes, rebuild may not be required when using `--symlink-install`.

For package metadata, dependencies, generated files, or installed resource changes, rebuild:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To build one package and its dependencies:

```bash
colcon build --symlink-install --packages-up-to openamrobot_docking
```

To build one package only:

```bash
colcon build --symlink-install --packages-select openamrobot_docking
```

---

## 12. Clean Build Outputs

Generated build folders live inside `ros2/`:

```text
ros2/build/
ros2/install/
ros2/log/
```

They should not be committed.

If a build gets into a strange state, you can remove those folders and rebuild:

```bash
cd openamr-platform-sw/ros2
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Use this only for local generated build outputs.

---

## 13. Before Opening a Pull Request

Check Git status from the repository root:

```bash
cd openamr-platform-sw
git status
```

Make sure generated folders are not staged:

```bash
git status --ignored
```

Build from `ros2/`:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

For docking or simulation changes, run at least one launch smoke test:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

Commit with DCO sign-off:

```bash
git commit -s -m "Describe the change"
```

---

## 14. Common Setup Problems

### Package Not Found

Example:

```text
package 'openamrobot_docking' not found
```

Usually the workspace was not sourced.

Fix:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Wrong Build Directory

If `colcon build` was run from the repository root, build folders may appear in the wrong place.

Correct build location:

```bash
openamr-platform-sw/ros2/
```

### Docking Node Exits or Nav2 Action Fails

Check CycloneDDS:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_cyclonedds_cpp
```

Set it if missing:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Gazebo or RViz Does Not Open

Check that you are on a machine with graphical support.

For headless testing:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

### Build Fails After Switching Branches

Clean generated outputs and rebuild:

```bash
cd openamr-platform-sw/ros2
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 15. Quick Command Summary

Fresh setup:

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch openamrobot_docking bringup_sim.launch.py
```

Every new terminal:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Dock test:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```
