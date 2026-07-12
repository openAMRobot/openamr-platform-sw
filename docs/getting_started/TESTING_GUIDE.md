# Testing Guide

This guide explains how to test changes in `openamr-platform-sw` before opening a pull request.

The project is a ROS 2 robotics stack, so testing has several levels:

- documentation checks
- package builds
- package tests
- launch smoke tests
- simulation behavior checks
- docking-specific checks

Use the smallest test set that matches your change, but always record what you ran in the pull request.

## Automated Docker Check

The pull-request workflow builds the ROS 2 Jazzy image, builds the complete
workspace, runs the focused docking and perception package tests, and enforces
at least 70% line coverage for `openamrobot_perception.scan_body_filter`.

Run the same fast gate locally from the repository root:

```bash
docker build --tag openamr-platform:test .
docker run --rm openamr-platform:test bash -lc '
  cd /ros2_ws &&
  colcon build --symlink-install &&
  colcon test --packages-select openamrobot_docking openamrobot_perception \
    --event-handlers console_direct+ &&
  colcon test-result --verbose &&
  python3 -m pytest src/openamrobot_perception/test/test_scan_body_filter.py \
    --cov=openamrobot_perception.scan_body_filter \
    --cov-report=term-missing \
    --cov-fail-under=70
'
```

The coverage threshold deliberately applies only to the perception filter in
this first test foundation. Docking geometry helpers and package resources are
tested, but the full stateful docking node is not represented as having 70%
coverage.

Passing automation reduces routine hardware work; it does not replace the
simulation and hardware checks later in this guide for changes affecting robot
motion, sensors, calibration, timing, or safety.

---

## 1. Before You Start

Use Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.

From the ROS 2 workspace:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If the workspace has already been built:

```bash
source install/setup.bash
```

If `install/setup.bash` does not exist, build first:

```bash
colcon build --symlink-install
source install/setup.bash
```

Check that the project packages are visible:

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

---

## 2. Choose the Right Test Level

| Change type | Minimum recommended checks |
|---|---|
| Documentation only | Read the changed docs, check links/commands, run `git diff --check` |
| Git/contributing docs | Check examples use the right repo, branch, DCO, and PR target |
| Package README/docs | Build the related package if commands or package names changed |
| Launch files | Build, source, run the affected launch smoke test |
| URDF/xacro or meshes | Build, launch Gazebo, confirm robot spawns and TF exists |
| Gazebo bridge/worlds | Build, launch Gazebo, check `/clock`, `/scan`, `/odom`, `/rgb_image`, `/cmd_vel` |
| Nav2 config/maps | Build, launch Gazebo + Nav2, send or inspect a navigation goal |
| Docking code/config | Build, launch full stack, trigger dock and undock |
| C++ node changes | Build package, run package tests, run launch that uses the node |
| Python node changes | Build package, run package tests if available, run launch that uses the script |

---

## 3. Documentation Checks

For documentation-only changes, run from the repository root:

```bash
git status
git diff --check
```

Review the changed files:

```bash
git diff
```

Check:

- links point to real files
- commands use the correct workspace path: `openamr-platform-sw/ros2`
- commands source ROS 2 before ROS commands
- commands source `install/setup.bash` after building
- generated folders such as `build/`, `install/`, and `log/` are not committed

For docs that mention topics or launch files, verify names against the repo:

```bash
find ros2/src -path "*launch*" -type f
find ros2/src -name package.xml
```

---

## 4. Build Checks

Build the full current simulation stack:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build only one package:

```bash
colcon build --symlink-install --packages-select openamrobot_nav2
```

Build a package and everything needed up to it:

```bash
colcon build --symlink-install --packages-up-to openamrobot_docking
```

Use `--packages-up-to openamrobot_docking` for most docking, simulation, and launch-composition changes.

---

## 5. Package Tests

Run all available package tests:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
colcon test
colcon test-result --verbose
```

Run tests for one package:

```bash
colcon test --packages-select openamrobot_nav2
colcon test-result --verbose
```

Run tests for the packages that currently include Python lint/test files:

```bash
colcon test --packages-select openamrobot_description openamrobot_gazebo openamrobot_nav2
colcon test-result --verbose
```

Current test coverage:

| Package | Current test type |
|---|---|
| `openamrobot_description` | Python package lint tests: copyright, flake8, pep257 |
| `openamrobot_gazebo` | Python package lint tests: copyright, flake8, pep257 |
| `openamrobot_nav2` | Python package lint tests: copyright, flake8, pep257 |
| `openamrobot_docking` | Build coverage for C++ node and installed Python scripts; no dedicated test files yet |

If `colcon test` fails, inspect:

```bash
colcon test-result --verbose
```

Then fix the failure before opening the PR, or explain clearly why the failure is unrelated.

---

## 6. Launch Smoke Tests

Launch smoke tests prove that the main runtime layers still start.

Use a sourced terminal:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb            # Wayland fix — required on Ubuntu 24.04 default
```

> ⚠️ **GUIs are OFF by default** in both `gz_simulator.launch.py` (`gui:=true` to enable) and `sim_bringup_launch.py` (`use_rviz:=true` to enable). Without these flags the simulation still runs (you can verify with `ros2 topic hz /clock`) but no window appears. Use the flags below for the first smoke test so you can see what's happening.

### One-Command Simulation Smoke Test

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

On slower machines:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=22
```

Headless mode:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

Expected:

- Gazebo starts or headless server starts.
- Robot spawns.
- `/clock`, `/scan`, `/odom`, `/tf`, and `/rgb_image` appear.
- Nav2 starts after the configured delay.
- Docking layer starts after Nav2.
- No package lookup errors appear.

### Three-Layer Smoke Test

Use three sourced terminals.

Terminal 1:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py gui:=true
```

Terminal 2:

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py use_rviz:=true
```

Terminal 3:

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

Expected:

- Gazebo owns `/clock` and sensor topics.
- Nav2 localizes on the map.
- Docking starts AprilTag detection and the dock trigger node.

---

## 7. Runtime Topic Checks

After launching the simulation, check core topics:

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

Check robot model and static transforms:

```bash
ros2 topic echo /robot_description --once --qos-durability transient_local
ros2 topic echo /tf_static --once --qos-durability transient_local
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

Check Nav2 action servers:

```bash
ros2 action list
```

Expected action:

```text
/navigate_to_pose
```

---

## 8. Gazebo and Bridge Checks

Use these checks after Gazebo is running.

Check sensor topics:

```bash
ros2 topic echo /scan --once
ros2 topic hz /rgb_image
ros2 topic echo /odom --once
```

Check the velocity command bridge:

```bash
ros2 topic info /cmd_vel
```

The Gazebo bridge should be connected to `/cmd_vel`.

Manual movement test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
```

Expected:

- Robot moves forward briefly in Gazebo.
- `/odom` changes.

Use a small velocity and stop the test if the robot behaves unexpectedly.

---

## 9. Navigation Checks

After Gazebo and Nav2 are running:

```bash
ros2 action list
ros2 topic echo /map --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

In RViz:

- fixed frame should be `map`
- robot should appear on the map
- lidar scan should line up with the map
- a simple 2D goal should produce a plan

If testing without RViz, check that `/navigate_to_pose` exists and Nav2 logs show active lifecycle nodes.

---

## 10. Docking Checks

After the full stack is running, trigger docking:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Expected:

- `dock_trigger.py` receives the trigger.
- Phase 1 sends a Nav2 goal to the staging pose.
- AprilTag detections are available.
- The robot aligns and approaches the dock.

Check AprilTag topics:

```bash
ros2 topic hz /rgb_image
ros2 topic hz /camera_info
ros2 topic hz /camera_info_synced
ros2 topic echo /apriltag/detections --once
```

Check dock pose (the dock has a 3-tag bundle — tag 1 is the centre, the docking target;
`/detected_dock_pose` is the centre tag's pose republished in `map`):

```bash
ros2 topic echo /detected_dock_pose --once
ros2 run tf2_ros tf2_echo map charging_dock_tag_1
```

Test undock:

```bash
ros2 topic pub /undock_robot std_msgs/msg/Bool "{data: true}" --once
```

Expected:

- Robot reverses away from the dock.
- Robot rotates to face away from the dock.

For deeper docking diagnostics, see:

```text
ros2/src/openamrobot_docking/docs/09_troubleshooting.md
```

---

## 11. What to Test by Change Area

### Documentation Change

Run:

```bash
git diff --check
```

Also manually review changed commands, links, and package names.

### Robot Description Change

Run:

```bash
colcon build --symlink-install --packages-select openamrobot_description
colcon test --packages-select openamrobot_description
colcon test-result --verbose
```

Then launch Gazebo and confirm the robot spawns.

### Gazebo Change

Run:

```bash
colcon build --symlink-install --packages-up-to openamrobot_gazebo
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Check:

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

### Nav2 Change

Run:

```bash
colcon build --symlink-install --packages-up-to openamrobot_nav2
colcon test --packages-select openamrobot_nav2
colcon test-result --verbose
ros2 launch openamrobot_nav2 sim_bringup_launch.py
```

Check `/navigate_to_pose`, `/map`, and `map -> base_link`.

### Docking Change

Run:

```bash
colcon build --symlink-install --packages-up-to openamrobot_docking
ros2 launch openamrobot_docking bringup_sim.launch.py
```

Then trigger:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /undock_robot std_msgs/msg/Bool "{data: true}" --once
```

---

## 12. Pull Request Test Plan

Every pull request should include a test plan.

Good examples:

```text
## Test plan

- Ran `git diff --check`
- Built from `ros2/` with `colcon build --symlink-install --packages-up-to openamrobot_docking`
- Ran `colcon test --packages-select openamrobot_description openamrobot_gazebo openamrobot_nav2`
- Launched `ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false`
- Triggered docking with `/dock_trigger`; robot reached staging and started AprilTag alignment
```

For documentation-only changes:

```text
## Test plan

- Ran `git diff --check`
- Manually reviewed links and commands in the changed docs
```

If you did not run a test, say so and explain why:

```text
## Test plan

- Not run: simulation smoke test requires Gazebo display access
```

---

## 13. Common Testing Mistakes

Avoid these:

- running `colcon build` from the repository root instead of `ros2/`
- forgetting `source /opt/ros/jazzy/setup.bash`
- forgetting `source install/setup.bash` after building
- running ROS commands without `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- testing only the package build but not the launch file that uses it
- committing generated folders such as `ros2/build/`, `ros2/install/`, or `ros2/log/`
- opening a PR without a test plan
