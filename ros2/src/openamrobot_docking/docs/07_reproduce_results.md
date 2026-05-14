# Reproducing Results

A step-by-step recipe for getting docking to work, both in simulation and on the real robot.

## Simulation (recommended path for first-time setup)

### 1. Install dependencies

```bash
sudo apt update
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
  ros-jazzy-rmw-cyclonedds-cpp     # CycloneDDS — required (see below)
```

### 1b. Use CycloneDDS instead of FastDDS

The default Jazzy FastDDS has a Python-side crash bug that makes `dock_trigger.py` exit silently when sending action goals. Export CycloneDDS in every terminal that runs ROS commands:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

You can type this in each new terminal, or add the line to your `~/.bashrc` for automatic setup. Either way, verify before launching:

```bash
echo $RMW_IMPLEMENTATION   # should print: rmw_cyclonedds_cpp
```

`docs/09_troubleshooting.md` has the full diagnostic story.

### 2. Build

```bash
cd ~/Downloads/openamrobot-docking-main
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openamrobot_docking
source install/setup.bash
```

### 3. Launch the simulation

```bash
ros2 launch openamrobot_docking simulation.launch.py
```

This brings up: Gazebo server + GUI, robot_state_publisher, ros_gz_bridge, slam_toolbox (delayed 4 s), Nav2 stack, apriltag_node, detected_dock_pose_publisher, dock_trigger.py, RViz.

Wait ~10–15 s for SLAM and Nav2 lifecycle to all show "active" in the logs.

### 4. Verify the stack before triggering

In a second terminal:

```bash
source ~/Downloads/openamrobot-docking-main/install/setup.bash

# (a) Camera and tag detection
ros2 topic hz /camera/image_raw           # ~15 Hz
ros2 topic echo /detected_dock_pose --once  # should give a pose with x≈4.0, y≈8.9 in map

# (b) TF chain end-to-end
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo map charging_dock_apriltag

# (c) Nav2 lifecycle
ros2 lifecycle nodes
# (every node should be 'active')
```

### 5. Trigger docking

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Watch the `dock_trigger` logs. You should see 4 phases in order:

```
── Phase 1/4: NavigateToPose → staging zone
── Phase 2/4: tag search + initial filter (40 samples)
   scanning to centre tag in camera (tolerance ±2.0°, need 5 consecutive frames)
   tag centred in camera (image_angle=-1.5°, consecutive=5)
   tag at (4.000, 8.900) after 40 samples
── Phase 3/4: ALIGN (spin to perpendicular yaw 1.571)
── Phase 4/4: line-tracking advance to 0.90m
   d=1.40m < 1.40m — final align then straight-line approach
   reached: d_to_tag=0.900m, lateral=+0.4cm (340 samples averaged)
Docking sequence complete ✓
```

If anything fails partway, the logs will say which phase and what error.

### 6. Document and rerun

If you change parameters, document them in commit messages or a per-experiment log:

```text
date: 2026-05-08
git commit: abc1234
config/dock_trigger.yaml:
  staging_distance: 2.5 → 3.0   (testing larger zone)
result: docks correctly, ~5 cm lateral offset, 1.2° yaw error at end
```

To rerun cleanly between experiments:

```bash
~/Downloads/openamrobot-docking-main/openamrobot_docking/scripts/kill_sim.sh
ros2 launch openamrobot_docking simulation.launch.py
```

## Real robot

### 1. Install dependencies

Same `apt install` line as above, minus the `ros-jazzy-ros-gz-*` and `ros-jazzy-slam-toolbox` if you don't need them. Add `ros-jazzy-camera-ros`, `ros-jazzy-camera-calibration`, `ros-jazzy-image-proc`.

### 2. Calibrate the camera

Follow `06_camera_calibration.md` step by step. Output: a calibration YAML at `~/.ros/camera_info/<camera_name>.yaml`.

### 3. Measure the printed tag

```bash
# Print a 36h11 tag of any size (e.g., from https://github.com/AprilRobotics/apriltag-imgs)
# Measure the printed tag's outer black-square edge in meters
# Update config/tags_36h11.yaml with that size
```

### 4. Verify camera + tag detection in isolation

```bash
ros2 launch openamrobot_docking apriltag.launch.yml
# In another terminal:
ros2 topic echo /apriltag/detections --once     # should detect tag ID 0
ros2 run tf2_ros tf2_echo camera_optical_frame charging_dock_apriltag
```

### 5. Place the tag in the map and configure `home_dock.pose`

See `04_apriltag.md`.

### 6. Launch the full stack

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

This launches camera + apriltag + detected_dock_pose_publisher + opennav_docking + nav2 + dock_trigger. Note that with the current `dock_trigger.py`, this runs the **4-phase sequencer** on the real robot (not `opennav_docking`'s built-in approach) — so it expects `dock_pose_x/y/yaw`, `staging_distance`, `docking_distance`, `visual_servo_distance` and the other parameters in `config/dock_trigger.yaml` to match your real-world dock.

If you'd rather use `opennav_docking::SimpleChargingDock`'s `controlled_approach`, see `02_architecture.md` for how to swap the trigger.

### 7. Trigger and document

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Record the result, including:

- Tag size and the path of the printed tag
- Camera model, calibration YAML hash
- `home_dock.pose` and any other tuned params
- Distance and angle error at the final docking position (measure with TF or a tape measure)

This makes results reproducible for the next iteration.
