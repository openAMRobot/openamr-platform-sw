# Quickstart — From zero to docking

This document walks you from a fresh Ubuntu install to a working docking sequence in simulation. Follow it linearly the first time.

For background on **what** this package does, see [`00_overview.md`](00_overview.md) first.

---

## Prerequisites

### System

- **Ubuntu 24.04 (Noble)** — native install. WSL2 / macOS / Windows are not supported (Gazebo Harmonic needs a Linux display server).
- **ROS 2 Jazzy** installed system-wide ([install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
- **Gazebo Harmonic** (`gz-sim 8.x`) — comes with `ros-jazzy-ros-gz-sim`.
- A working **X11 or Wayland** display.

### ROS 2 packages (one-time)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-amcl \
  ros-jazzy-apriltag-ros \
  ros-jazzy-image-proc \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-laser-filters \
  ros-jazzy-rviz2 \
  ros-jazzy-topic-tools \
  python3-colcon-common-extensions
```

> ⚠️ **CycloneDDS is required.** The default Jazzy RMW (FastDDS) has a Python crash bug that makes `dock_trigger.py` exit silently when sending Nav2 action goals. Always export:
> ```bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```
> Put this in `~/.bashrc` once and you're done.

---

## 2. Clone and build

```bash
cd ~/Downloads
git clone <fork-or-org-url>/openamr-platform-sw.git
cd openamr-platform-sw

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`colcon build` should finish in <10 s with all packages succeeded. If a package is missing dependencies, re-run the `apt install` above.

---

## 3. The 3-terminal launch sequence

The docking pipeline is **layered**. Each layer runs in its own terminal so you can restart any one without bringing the others down. In every terminal you open, first source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Downloads/openamr-platform-sw/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

(Or put these in a shell alias: `alias src-amr='source ...'`.)

> ⚠️ **GUIs are OFF by default.** Both Gazebo and RViz launch headless unless you opt in:
> - Gazebo : `ros2 launch openamrobot_gazebo gz_simulator.launch.py gui:=true`
> - Nav2 / RViz : `ros2 launch openamrobot_nav2 sim_bringup_launch.py use_rviz:=true`
>
> The default keeps the sim timing stable on slow machines; turn them on the first time you bring the stack up so you can see what's happening.
>
> ⚠️ **Wayland users (Ubuntu 24.04 default).** Qt-based apps (Gazebo GUI, RViz) need the X11 backend or the window silently never appears:
> ```bash
> export QT_QPA_PLATFORM=xcb   # in every terminal that launches a GUI
> ```
> Symptom: `wmctrl -l | grep -i gazebo` returns nothing but the simulation is running (you can see `ros2 topic hz /clock` at ~50–1000 Hz). The fix is `xcb` or putting the export in `~/.bashrc`.

### Terminal 1 — Gazebo + robot + ros↔gz bridge

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py gui:=true
```

This brings up Gazebo Harmonic with `walled_world.sdf` (a 10×10 m walled arena containing the AprilTag dock on the +x wall), spawns the robot at world `(0, 0, 0)`, and runs the ros↔gz bridge for `/clock`, `/odom`, `/tf`, `/cmd_vel`, `/scan`, `/rgb_image`, `/camera/camera_info`, `/imu`.

Wait for the Gazebo GUI window to open and the robot to be visible (~3 s).

### Terminal 2 — Nav2 + localization + RViz

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py use_rviz:=true
```

This brings up Nav2's planner / controller / behavior server, AMCL on a saved map (`maps/my_map.yaml`), and the RViz layout. AMCL is initialised at map `(0, 0, 0)`, so **map ≡ world**.

Wait for RViz to show the robot localized on the map (you'll see the lidar scan overlaying the map walls, ~10 s).

### Terminal 3 — Docking layer (this package)

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

This adds three nodes on top:

- `apriltag_ros::apriltag_node` (in the `/apriltag` namespace) subscribing to `/rgb_image` + `/camera_info_synced` — detects the **3-tag bundle** (IDs 0/1/2) and publishes one TF per tag
- `detected_dock_pose_publisher` (C++) publishing `/detected_dock_pose` at 10 Hz — tracks the **centre tag** (id 1)
- `dock_trigger.py` (Python, the **bundle sequencer**) waiting on `/dock_trigger`
- A small `camera_info_bridge` `ros_gz_bridge` instance bridging `gz /camera_info → ROS /camera_info`, plus a `camera_info_sync` node that stamps `/camera_info` with the image timestamp so apriltag_ros's exact-sync sees pairs (Gazebo publishes image and camera_info at different rates)

Wait for the `[dock_trigger.py-N] [INFO] Dock trigger ready on 'dock_trigger'` log line.

---

## 4. Trigger the docking

In any sourced terminal (or from the UI):

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Watch the **Terminal 3** logs. The bundle pipeline runs roughly:

```
── Phase 1: NavigateToPose → staging zone
   → staging (2.90, 0.00, yaw=0.00)
   ... (Nav2 controller messages)
   Goal succeeded
── Phase 2: bundle search + centring scan on midpoint of outer tags
   scanning to centre bundle in camera (tolerance ±2.0°, need 5 consecutive frames)
   bundle centred (image_angle=±X.X°, consecutive=5)
── Phase 3: estimate dock normal N from outer tags (90 cm baseline)
   normal yaw ≈ X.XX rad, depth ≈ Y.YY m → point P1 on the normal
── Phase 4: drive to P1 (pure-pursuit) → re-verify normal N' from P1
   N vs N' agreement: Δ = X.X° (≤ tol)  → proceed
── Phase 5: final approach (two-regime)
   FAR:  averaging 3-tag axis (EMA), pure-pursuit
   NEAR: axis frozen at depth ≤ 0.70 m, image-frame visual servo on centre tag
   blind final advance (tag out of FOV) → stop at depth ≈ 0.15 m
   Phase 5 done.
```

End state: the robot is **stopped ~0.15 m camera→tag depth in front of the centre tag, perpendicular to the dock face**. The exact log lines may differ from the snapshot above as the pipeline evolves — the phase order is what matters.

---

## 5. Diagnostics if something doesn't behave

| Symptom | Quick check |
|---|---|
| Robot stays at the staging point | `ros2 topic hz /apriltag/detections` — is it >0 Hz? Look for `id: 0, 1, 2` in `ros2 topic echo /apriltag/detections` (all three bundle tags should be detected at staging) |
| `bundle never detected during scan` | `ros2 run rqt_image_view rqt_image_view /rgb_image` — are the three AprilTag patterns visible (black/white) or uniformly grey? |
| `apriltag_node` shows `Synchronized pairs: 0` | The `camera_info_sync` node isn't running, or `/camera_info_synced` isn't being produced. `ros2 topic hz /camera_info_synced` should match `/rgb_image` rate. |
| Robot moves in RViz but not in Gazebo | `ros2 topic info /cmd_vel` — does the bridge subscribe? |
| Final approach stops 2 m short | Tag size mismatch — check `config/tags_36h11_sim.yaml` `size: 0.16` (= the 0.20 m panel × 0.8 black-square edge) matches `models/apriltag_dock/model.sdf` |
| Robot stops mid-approach with "obstacle blocking" | The new LIDAR obstacle guard fired. Check `/scan` for spurious returns, or widen `obstacle_arc_half_width_deg` |

Full troubleshooting matrix in [`09_troubleshooting.md`](09_troubleshooting.md).

---

## 6. Where to go next

- [`05_parameters.md`](05_parameters.md) — every YAML knob explained
- [`13_perception_and_line.md`](13_perception_and_line.md) — perception + how the line is built + RViz/Gazebo markers
- [`14_docking_research.md`](14_docking_research.md) — design rationale, validation plan, failure modes
- [`07_reproduce_results.md`](07_reproduce_results.md) — exact reproduction checklist
- [`10_diagrams.md`](10_diagrams.md) — block + state diagrams
- [`08_legacy_sequencer.md`](08_legacy_sequencer.md) — historical context (legacy single-tag pipeline)

---

## Real-robot port (high level)

To deploy the same pipeline on hardware, the changes from this quickstart are:

1. Stop using `openamrobot_gazebo` / `sim_bringup_launch.py`. Instead, launch:
   - your camera driver (e.g. `camera_ros`) publishing `/camera/image_raw` + `/camera/camera_info`
   - `image_proc` for rectification, publishing `/camera/image_rect`
   - your lidar driver publishing `/scan`
   - your motor controller driver publishing `/odom` + `/tf (odom → base_link)` and consuming `/cmd_vel`
   - `robot_state_publisher` with your robot URDF
   - Nav2 with AMCL on a pre-built map of the real environment
2. Use `apriltag.launch.yml` (real-robot variant) instead of `apriltag_sim.launch.yml`. It expects rectified images.
3. Update `config/dock_trigger.yaml`:
   - `dock_pose_x`, `dock_pose_y`, `dock_pose_yaw` = the measured real-world pose of the **centre tag** (id 1) in the map frame
   - All other parameters can stay (the bundle logic is hardware-agnostic)
4. Print the **three AprilTags 36h11** (IDs 0, 1, 2) of the measured panel size; mount them coplanar with the outer two at the same lateral offset from the centre tag (the simulation uses ±0.45 m, but the normal estimator uses the *observed* positions, not a hard-coded spacing). Update `tags_36h11.yaml` `size:` to the **black-square edge** of your printed tag (= panel side × 8/10 for 36h11).
5. Calibrate the camera and ship the intrinsics — see [`06_camera_calibration.md`](06_camera_calibration.md).

The Python sequencer is unchanged.
