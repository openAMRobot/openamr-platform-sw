# Quickstart — docking simulation

End-to-end walkthrough: clone the workspace, build, and run the autodocking pipeline in Gazebo Harmonic.

## 0. Prerequisites

- Ubuntu 24.04 (Noble), native install (not WSL, not a devcontainer with no display).
- ROS 2 **Jazzy**.
- Gazebo **Harmonic** (`gz-sim 8.x`).
- A working display server (X11 or Wayland).

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-opennav-docking \
  ros-jazzy-opennav-docking-msgs \
  ros-jazzy-apriltag-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rmw-cyclonedds-cpp
```

## 1. CycloneDDS (required)

The default FastDDS on Jazzy has a Python-side crash bug that silently breaks `dock_trigger.py` when it sends action goals. Use CycloneDDS instead:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

## 2. Clone and build

```bash
cd ~
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to openamrobot_docking
source install/setup.bash
```

`--packages-up-to openamrobot_docking` builds the four packages in dependency order:

1. `openamrobot_description`
2. `openamrobot_gazebo`
3. `openamrobot_nav2`
4. `openamrobot_docking`

## 3. Launch

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Wait ~10–15 s for SLAM and Nav2 to all reach `active` in the logs.

## 4. Trigger the docking

In a second terminal (same `RMW_IMPLEMENTATION` export, `install/setup.bash` sourced):

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The 4-phase sequencer logs each phase. End state: the robot stops ~0.90 m in front of the AprilTag, perpendicular to the tag plane. Typical residual error: a few cm laterally and ~1° in yaw.

## 5. Cleanup between runs

If a previous launch left zombie processes:

```bash
bash openamr-platform-sw/scripts/kill_sim.sh
```

## 6. Custom spawn pose

```bash
ros2 launch openamrobot_docking docking_sim.launch.py \
    spawn_x:=2.0 spawn_y:=-3.0 spawn_yaw:=1.57
```

The world dock pose `(0, 4.9, -π/2)` is auto-projected through `spawn_yaw` so the docking pipeline still targets the same physical dock.

## See also

- Detailed sequencer walkthrough: `ros2/src/openamrobot_docking/docs/08_sequencer_4phase.md`
- Parameter reference: `ros2/src/openamrobot_docking/docs/05_parameters.md`
- Troubleshooting matrix: `ros2/src/openamrobot_docking/docs/09_troubleshooting.md`
- Repo-level architecture: `docs/architecture/01_repo_layout.md`
