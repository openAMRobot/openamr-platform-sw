# Troubleshooting Guide

This guide helps diagnose common problems while setting up, building, running, and contributing to `openamr-platform-sw`.

For deep docking-specific diagnostics, also see:

```text
ros2/src/openamrobot_docking/docs/09_troubleshooting.md
```

Start with the quick checks below, then use the section that matches your symptom.

---

## 1. Quick Checks

Run these first from the repository root:

```bash
pwd
git status
```

Then check the ROS 2 workspace:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
ros2 pkg list | grep openamrobot
```

Expected:

```text
jazzy
rmw_cyclonedds_cpp
```

Expected packages include:

```text
openamrobot_description
openamrobot_docking
openamrobot_gazebo
openamrobot_nav2
```

If these checks fail, fix the setup before debugging higher-level launch or docking behavior.

---

## 2. Quick Symptom Table

| Symptom | Likely cause | First fix |
|---|---|---|
| `package 'openamrobot_...' not found` | Workspace not sourced | `source install/setup.bash` from `ros2/` |
| `colcon: command not found` | colcon extension missing | Install `python3-colcon-common-extensions` |
| Build creates `build/ install/ log/` in repo root | Built from wrong folder | Build from `openamr-platform-sw/ros2` |
| `dock_trigger.py` exits silently | FastDDS issue on Jazzy | Use CycloneDDS |
| Gazebo/RViz does not open | No graphical display or headless machine | Use headless launch arguments |
| Robot does not move | `/cmd_vel` not reaching Gazebo | Check `/cmd_vel` and bridge |
| Nav2 does not plan | Costmap/localization not ready | Wait, check `/scan`, `/map`, `/tf` |
| AprilTag is never detected | Camera/tag/config issue | Check image, camera info, tag family/size |
| TF lookup errors | Missing transform or time mismatch | Check TF chain and `use_sim_time` |
| PR DCO check fails | Commit missing sign-off | Amend commit with `--signoff` |
| Push is rejected | Branch behind remote or rewritten commit | Pull or use `--force-with-lease` after amend |

---

## 3. Setup Problems

### `colcon: command not found`

Install colcon:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Then source ROS 2 and try again:

```bash
source /opt/ros/jazzy/setup.bash
colcon --help
```

### ROS 2 Commands Are Not Found

Symptom:

```text
ros2: command not found
```

Fix:

```bash
source /opt/ros/jazzy/setup.bash
```

Check:

```bash
echo $ROS_DISTRO
```

Expected:

```text
jazzy
```

If `/opt/ros/jazzy/setup.bash` does not exist, ROS 2 Jazzy is not installed correctly.

### Wrong Build Directory

The ROS 2 workspace is:

```text
openamr-platform-sw/ros2/
```

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

If you accidentally built from the repository root, remove the generated root-level build outputs only after confirming they are not tracked:

```bash
git status
```

Generated folders should not be committed.

### Missing ROS Dependencies

If a build fails with a message like:

```text
Could not find a package configuration file provided by ...
```

Install the required packages from the README or run dependency installation from `ros2/`:

```bash
cd openamr-platform-sw/ros2
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` is not available:

```bash
sudo apt update
sudo apt install python3-rosdep
```

---

## 4. Build Problems

### Build Fails After Switching Branches

Old generated files can conflict with the new branch.

From the ROS 2 workspace:

```bash
cd openamr-platform-sw/ros2
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Only remove `build`, `install`, and `log` when they are local generated build outputs.

### Package Builds but Launch Cannot Find It

Symptom:

```text
package 'openamrobot_docking' not found
```

Fix:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | grep openamrobot
```

If the package still does not appear, rebuild:

```bash
colcon build --symlink-install
source install/setup.bash
```

### Build One Package While Debugging

Build one package and its dependencies:

```bash
colcon build --symlink-install --packages-up-to openamrobot_docking
```

Build only one package:

```bash
colcon build --symlink-install --packages-select openamrobot_docking
```

Use `--packages-up-to` when you are unsure.

---

## 5. Sourcing Problems

Every new terminal needs the environment again.

From `openamr-platform-sw/ros2`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If you forget this, common symptoms are:

- packages not found
- launch files not found
- topics missing
- different terminals seeing different ROS graphs

Check the current terminal:

```bash
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
ros2 pkg list | grep openamrobot
```

---

## 6. CycloneDDS Problems

This project uses CycloneDDS for the current Jazzy docking workflow.

Check:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_cyclonedds_cpp
```

If it is empty or different:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Install CycloneDDS if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

Make it permanent:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

Important: set this in every terminal that runs ROS commands.

---

## 7. Launch Problems

### Full Bringup Does Not Start Cleanly

Run the full stack:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch openamrobot_docking bringup_sim.launch.py
```

On slower machines, widen startup delays:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=22
```

For headless testing:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

### Layered Launch Debugging

If the full launch is confusing, run layers separately.

Terminal 1:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Terminal 2:

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py
```

Terminal 3:

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

Start them in that order.

### Old Processes After Ctrl-C

If a relaunch behaves strangely, check for old processes:

```bash
ps aux | grep -E "gz|ros2|rviz|parameter_bridge"
```

Close old terminals first. If stale simulation processes remain, stop them before relaunching.

---

## 8. Gazebo and RViz Problems

### Gazebo or RViz Does Not Open

Make sure you are on a machine with graphical display support.

For headless mode:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

### RViz Opens but Shows Nothing

Wait around 10 seconds after launch. Some nodes need time to start.

Check basic topics:

```bash
ros2 topic list
ros2 topic echo /robot_description --once
ros2 topic echo /tf_static --once
```

In RViz, set the fixed frame to:

```text
map
```

### Simulation Is Slow

Use headless mode:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

Close extra terminals that are echoing high-rate topics such as images or TF.

---

## 9. Robot Movement Problems

### Robot Does Not Move

Check whether `/cmd_vel` exists:

```bash
ros2 topic info /cmd_vel
```

Check whether velocity commands are being published:

```bash
ros2 topic echo /cmd_vel
```

Send a small manual command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
```

If `/cmd_vel` has messages but the robot does not move in Gazebo, check the Gazebo bridge and diff-drive plugin.

### Nav2 Moves in RViz but Robot Does Not Move in Gazebo

This usually means the ROS-to-Gazebo bridge is not forwarding `/cmd_vel`.

Check:

```bash
ros2 topic info /cmd_vel
ros2 node list | grep bridge
```

The bridge should be subscribed to `/cmd_vel`.

### Robot Moves but Drifts or Slides

In simulation, this can come from contact, wheel, or spawn-height issues.

Check the detailed docking troubleshooting and lessons learned:

```text
ros2/src/openamrobot_docking/docs/09_troubleshooting.md
ros2/src/openamrobot_docking/docs/12_lessons_learned.md
```

---

## 10. Navigation Problems

### Nav2 Does Not Plan

Check localization and sensor data:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

If `/scan` is missing, the simulator or bridge layer is not providing lidar data.

If `/map` is missing, localization or mapping is not ready.

If `map -> base_link` is missing, Nav2 does not know where the robot is.

### Goal Fails Immediately

Wait for Nav2 lifecycle nodes to activate. Then check action servers:

```bash
ros2 action list
```

Expected action includes:

```text
/navigate_to_pose
```

If the action server is missing, restart the Nav2 layer and watch the terminal logs.

---

## 11. AprilTag and Docking Problems

### Dock Trigger Does Nothing

Check CycloneDDS first:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_cyclonedds_cpp
```

Check that the trigger topic exists:

```bash
ros2 topic list | grep dock
```

Trigger docking:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

### AprilTag Is Not Detected

Check camera topics:

```bash
ros2 topic list | grep camera
ros2 topic hz /rgb_image
ros2 topic hz /camera_info
ros2 topic hz /camera_info_synced
```

Check detections:

```bash
ros2 topic list | grep apriltag
ros2 topic echo /apriltag/detections --once
```

Common causes:

- tag is not visible to the camera
- wrong tag family
- wrong tag size
- camera image is not being bridged
- `camera_info` is missing

### Tag Is Detected but Dock Pose Is Missing

Check TF (the dock has a 3-tag bundle — tag 1 is the centre, the docking target):

```bash
ros2 run tf2_ros tf2_echo map charging_dock_tag_1
```

If this fails, check the full TF chain:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

For full details, use:

```text
ros2/src/openamrobot_docking/docs/03_tf_frames.md
ros2/src/openamrobot_docking/docs/09_troubleshooting.md
```

---

## 12. TF and Time Problems

### TF Lookup or Extrapolation Errors

Common causes:

- `use_sim_time` mismatch between nodes
- missing transform
- node started before `/clock`
- stale process from an old launch

Check `/clock`:

```bash
ros2 topic echo /clock --once
```

Check node time settings:

```bash
ros2 node list
```

Then inspect important transforms:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

Generate a TF diagram:

```bash
ros2 run tf2_tools view_frames
```

This creates a `frames.pdf` in the current directory.

---

## 13. Git and Pull Request Problems

### DCO Check Failed

Each commit needs a sign-off line:

```text
Signed-off-by: Your Name <your.email@example.com>
```

For the latest commit:

```bash
git commit --amend --signoff --no-edit
git push --force-with-lease origin your-branch-name
```

For new commits, use:

```bash
git commit -s -m "Describe the change"
```

### Review Required

This is normal on protected branches.

You cannot approve your own PR unless the repository rules allow it. A maintainer or reviewer with write access must approve it.

### Push Updates Were Rejected

Check your branch:

```bash
git status
git branch
```

If your branch is behind the remote:

```bash
git pull
git push
```

If you amended a commit, push with:

```bash
git push --force-with-lease origin your-branch-name
```

Use `--force-with-lease` only when you intentionally rewrote commits on your own branch.

### Pushed to Fork Main but Upstream Did Not Change

Pushing to your fork updates:

```text
https://github.com/YOUR_USERNAME/openamr-platform-sw
```

It does not directly update:

```text
https://github.com/openAMRobot/openamr-platform-sw
```

To update the upstream repository, open a pull request into:

```text
openAMRobot/openamr-platform-sw:main
```

---

## 14. What to Include When Asking for Help

Include the exact command you ran, the error message, and these outputs:

```bash
git status
git log --oneline -5
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
ros2 pkg list | grep openamrobot
```

For runtime issues, also include:

```bash
ros2 topic list
ros2 node list
ros2 action list
```

For TF issues:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

For docking-specific issues, include the launch command and the relevant logs from:

```text
ros2/src/openamrobot_docking/docs/09_troubleshooting.md
```
