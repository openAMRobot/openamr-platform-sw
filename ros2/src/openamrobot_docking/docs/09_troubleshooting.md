# Troubleshooting

Quick-reference table of common failures, plus detailed diagnostics. Failures specific to the simulation are flagged **[sim]**; those specific to the real robot **[real]**; others apply to both.

For the *why* behind each fix (and lessons we learned the hard way), see [`12_lessons_learned.md`](12_lessons_learned.md).

---

## 1. Quick-reference table

| Symptom | Most likely cause | First thing to try | Section |
| :---- | :---- | :---- | :---- |
| `dock_trigger.py` exits silently after triggering | FastDDS Python crash bug on Jazzy | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | [§2.1](#21-cyclonedds--rmw) |
| `symbol lookup error: ... fastcdr` | Same as above | Same as above | [§2.1](#21-cyclonedds--rmw) |
| `Rectified topic '/camera/image_rect' requested but camera publishing '...' is uncalibrated` **[real]** | No calibration file | Calibrate camera (see [`06_camera_calibration.md`](06_camera_calibration.md)) | [§2.2](#22-camera--apriltag) |
| `ros2 topic echo /apriltag/detections --once` returns no tags | Tag not visible / wrong family or size | Verify `family`, `size`, lighting | [§2.2](#22-camera--apriltag) |
| `/detected_dock_pose` is silent even though tag is detected | TF chain broken | Check `map → odom → base_link → camera → tag` | [§2.3](#23-tf--localization) |
| **[sim]** `Image messages received: 0` while `CameraInfo: ~15` | gz↔ROS bridge can't keep up at HD | Reduce camera resolution to 640×480 | [§2.6](#26-simulation-specific) |
| **[sim]** "Failed to create plan with tolerance of: 0.500000" | Goal in inflation zone or costmap not ready | Use `tolerance: 1.0` + A* planner | [§2.4](#24-navigation) |
| **[sim]** Robot moves a little, then "Collision Ahead - Exiting Spin" | Robot tipped → lidar scans its own body | Reduce velocity_smoother accels | [§2.6](#26-simulation-specific) |
| **[sim]** Robot reaches dock position but is offset laterally vs the real tag | AprilTag map-frame detection bias, not controller | Bigger tag, more samples in phase 2 filter | [§2.5](#25-docking-flow) |
| **[sim]** Final yaw off by a few degrees | Phase 4 final-align tolerance, or skipped because `visual_servo_distance ≤ docking_distance` | Tighten `spin_yaw_tolerance` or bump `visual_servo_distance` | [§2.5](#25-docking-flow) |
| **[sim]** Robot wobbles in the last metre of approach | Closed-loop yaw correction near the tag over-reacts to tiny lateral motion | Make sure `visual_servo_distance > docking_distance` so the straight-line phase engages | [§2.5](#25-docking-flow) |
| **[sim]** Robot slides on the floor, drive wheels don't grip | URDF spawn z wrong — `base_footprint` not at z=0, drive wheels float | Confirm spawn `-z 0.0` in `simulation.launch.py` | [§2.6](#26-simulation-specific) |
| **[sim]** Phase 2 scan rotates forever, never centres | Tag never enters the camera frustum / detection rate too low | Check `/apriltag/detections`; bump `scan_rotation_speed` or extend timeout | [§2.5](#25-docking-flow) |
| **[real]** `opennav_docking` "drives behind the dock" | Sign of `external_detection_translation_x` wrong | Flip to `+0.18` | [§2.5](#25-docking-flow) |
| **[sim]** `Address already in use` on relaunch | Zombie processes from previous Ctrl-C | Run `scripts/kill_sim.sh` | [§2.7](#27-process--launch) |
| `Could not find a package configuration file provided by ...` | Missing apt deps | `rosdep install --from-paths . -y` | [§2.7](#27-process--launch) |

---

## 2. Detailed diagnostics

### 2.1 CycloneDDS / RMW

#### `dock_trigger.py` crashes silently when triggering / never sees `/dock_trigger`

**Symptom:** You publish to `/dock_trigger` with `ros2 topic pub`, and nothing happens. The `dock_trigger` node might log nothing, or you might see:

```
symbol lookup error: undefined symbol: _ZN8eprosima7fastcdr3Cdr...
```

**Cause:** This repository requires **CycloneDDS** (`rmw_cyclonedds_cpp`). The default FastDDS on Jazzy has a Python-side bug that crashes the `rclpy.action.ActionClient` used by `dock_trigger.py`.

**Verify your RMW:**

```bash
echo $RMW_IMPLEMENTATION
# Should print: rmw_cyclonedds_cpp
```

**Fix:** If it's empty or shows `rmw_fastrtps_cpp`:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The setting must apply to **every terminal** that runs ROS commands (launch, topic pub, action call). Either type the export in each new terminal, or add it to `~/.bashrc` for automatic setup.

See [`12_lessons_learned.md`](12_lessons_learned.md) "FastDDS Python crash" for how we found this bug.

---

### 2.2 Camera & AprilTag

#### Camera is uncalibrated **[real]**

**Symptom:**

```
[ERROR] Rectified topic '/camera/image_rect' requested but camera publishing
'/.../camera_info' is uncalibrated
```

**Cause:** No calibration file exists at `~/.ros/camera_info/<camera_name>.yaml`, so `camera_ros` publishes a `camera_info` with a zero matrix and `image_proc::RectifyNode` rejects it.

**Fix:** Follow [`06_camera_calibration.md`](06_camera_calibration.md) to calibrate and install the calibration file.

#### No tag detected

- Confirm `family` and `size` in the AprilTag config match your tag.
  - **[sim]** `simulation/config/tags_36h11_sim.yaml`, `size: 0.40` (matches the panel in `apriltag_dock/model.sdf`).
  - **[real]** `config/tags_36h11.yaml`, `size: 0.0555` (or whatever you measured).
- Verify the camera image is reaching `apriltag_ros`:

  ```bash
  ros2 topic hz /camera/image_raw          # should be ~15 Hz [sim] or ~30 Hz [real]
  ros2 topic hz /camera/camera_info
  ```

- Verify image and camera_info timestamps are synchronized (`apriltag_ros` logs `Synchronized pairs: 0` if not).
- For real cameras, confirm lighting and focus are adequate.

#### Tag detected but no pose published in `map`

- Check the TF chain from `map` to `charging_dock_apriltag`:

  ```bash
  ros2 run tf2_ros tf2_echo map charging_dock_apriltag
  ```

- Verify `parent_frame` and `child_frame` in [`../config/docking_pose_publisher.yaml`](../config/docking_pose_publisher.yaml).
- Confirm localization is publishing `map → odom`:
  - **[sim]** `slam_toolbox` runs in mapping mode with a 4-second startup delay (in `simulation.launch.py`).
  - **[real]** AMCL or your localizer must be active.

#### Tag pose has large noise

- Increase the temporal filtering in `dock_trigger.py`: raise `filter_num_samples` to 40–60 in [`../config/dock_trigger.yaml`](../config/dock_trigger.yaml). Make sure `filter_max_collect_time` is correspondingly long enough (≥ N/10 s plus margin).
- The detector's `decimate` controls accuracy vs speed. Lower `decimate` (1.0 or 0.5) → more accurate but slower.
- A larger physical tag improves accuracy more than higher camera resolution. The sim uses `size: 0.40` for this reason.

---

### 2.3 TF / Localization

#### `/detected_dock_pose` is silent even though `/apriltag/detections` works

The TF chain `map → odom → base_footprint → base_link → camera_link → camera_rgb_optical_frame → charging_dock_apriltag` must exist end-to-end. If any link is missing, `detected_dock_pose_publisher` silently produces nothing.

Diagnose:

```bash
ros2 run tf2_ros tf2_echo map charging_dock_apriltag       # full chain
ros2 run tf2_ros tf2_echo map base_footprint               # localization OK?
ros2 run tf2_ros tf2_echo base_link camera_rgb_optical_frame   # static TF OK?
ros2 run tf2_tools view_frames                              # → frames.pdf
```

See [`03_tf_frames.md`](03_tf_frames.md) for the full required chain.

#### **[sim]** Map frame appears rotated relative to world

SLAM Toolbox initialises the map at the robot's spawn pose with the robot's spawn yaw → 0 in map. If you spawned at world yaw=π/2, then `map_x = world_y` and `map_y = -world_x`.

`simulation.launch.py` accepts a `spawn_yaw` argument and automatically re-projects the dock pose into the new map frame so the sequencer keeps working. But any externally hard-coded map-frame coordinate (e.g. an RViz goal pose) will not auto-rotate — use `spawn_yaw:=0` if you want map and world axes aligned.

#### Camera optical frame missing or wrong

- The TF chain must include `camera_link → camera_rgb_optical_frame` (one fixed joint in `omr_description/urdf/omr_robot.urdf.xacro`).
- The optical frame carries the (`−π/2`, `0`, `−π/2`) rpy rotation that REP-103 specifies. Don't omit it.
- **[sim]** Check that the camera sensor's `<gz_frame_id>` in `gazebo_control.xacro` is `camera_rgb_optical_frame` (not `camera_link`).

---

### 2.4 Navigation

#### **[sim]** "Failed to create plan with tolerance of: 0.500000"

The planner couldn't find a path. Common causes:

- **Goal in inflation zone** → use `tolerance: 1.0` in the planner config (already set in `nav2_sim_full.yaml`).
- **Costmap not yet populated at startup** (`static_layer` waiting for `/map`) — we use `obstacle_layer` only to avoid this.
- **Robot is in a "lethal" cell** (very rare; usually means the lidar misregistered the robot's body).

Visualize the costmap in RViz to confirm:
- Add a "Costmap" display
- Topic: `/global_costmap/costmap`
- If grey areas surround the robot, that's the problem.

#### `/cmd_vel` is silent during docking but `/cmd_vel_nav` has data

The Nav2 chain is `cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel`. If any node fails or any plugin clamps to zero, the chain breaks at that point.

Check each topic's rate:

```bash
ros2 topic hz /cmd_vel_nav /cmd_vel_smoothed /cmd_vel
```

Common culprit: `collision_monitor`'s polygon checker. We disabled `FootprintApproach` in the sim (`FootprintApproach.enabled: false` in `nav2_sim_full.yaml`).

#### Robot drifts left/right when commanded straight forward

- **[sim]** Check that drive wheel inertials are identical (`ixy=ixz=iyz=0`, same mass, same diagonal). See [`12_lessons_learned.md`](12_lessons_learned.md).
- **[real]** Calibrate `wheel_separation` and `wheel_radius` if odometry diverges from physical motion.

---

### 2.5 Docking flow

#### **[sim]** Robot reaches the docking position but is offset laterally vs the real tag

The 4-phase sequencer drives onto the **detected** dock's perpendicular axis. If `solvePnP` reports the tag a few tens of centimetres away from its true world position (a known sim issue with Gazebo's auto-derived `camera_info`), the robot ends offset relative to the actual tag by the same amount even though `lateral` reads zero.

The phase-2 camera-frame scan and the running-average filter keep this bias as small as possible, but they cannot remove it completely.

**Mitigation:**
- Bigger physical tag (already at 0.40 m in sim).
- More samples in the phase-2 filter (`filter_num_samples: 40` is the default; raise to 60 if your machine can keep up with the longer wait at staging).
- Lower `scan_centring_tolerance` (default 2° → try 1°) so the scan locks the camera closer to centred before the filter starts. solvePnP is less biased when the tag sits near the image centre.
- Use `decimate: 1.0` in the AprilTag detector (already the default in sim).

#### **[sim]** Final yaw is off by a few degrees

Phase 4 performs a one-shot in-place spin to the running-average perpendicular yaw at `visual_servo_distance`, then drives straight (`omega = 0`) until `docking_distance`. The residual yaw error is bounded by `spin_yaw_tolerance` (default `0.02 rad ≈ 1.1°`).

**If the angle is still too large:**
- Confirm `visual_servo_distance > docking_distance`; otherwise the straight-line phase is never entered and the line-tracking controller hands off its (possibly imperfect) heading directly to the stop condition.
- Tighten `spin_yaw_tolerance` to `0.01` (≈ 0.6°), at the cost of a slightly longer spin.
- Verify the camera-frame scan locked at < 2° in the logs (`tag centred in camera (image_angle=…°)`); a poor lock at phase 2 propagates downstream.

#### **[sim]** Robot wobbles in the last metre of the approach

The line-tracking controller is sensitive near the tag because a centimetre of lateral motion produces a large image-angle change. The straight-line final-approach mode exists precisely to avoid this.

**Check:**
- `visual_servo_distance` is **larger** than `docking_distance`. With defaults, that's `1.4 > 0.9`, so the straight-line phase covers the last 0.5 m. If you bumped `docking_distance` past `visual_servo_distance`, the line-tracking stays active all the way and wobbles.
- The launch log should show `d=1.40m < 1.40m — final align then straight-line approach`.

#### **[sim]** Robot hits the wall during the advance phase

The sequencer stops when distance to the running-average tag ≤ `docking_distance`. With the default `docking_distance: 0.9` and the tag at world `(0, 4.9)` with the panel facing south, the robot stops with its front about 75 cm from the wall. Plenty of margin.

If you reduce `docking_distance` close to 0.4 m or below, double-check the robot's footprint length and the wall position before relaunching.

#### **[sim]** Robot slides instead of driving cleanly

Check the spawn `z`. The URDF root `base_footprint` must be at world z=0, with `base_joint` lifting `base_link` by `0.053 m` so the wheel centres sit at z=`wheel_radius=0.1 m`. If you spawn `base_footprint` at z=0.053 (an earlier wrong setting), the wheel centres end up at 0.153 m and the drive wheels float 5.3 cm above the ground — only the casters touch, and the robot slides.

`simulation.launch.py` already uses `-z 0.0`. Verify in the launch file if you've edited it.

#### **[real]** `opennav_docking` fails: "drives behind the dock"

See [`11_changes_from_upstream.md`](11_changes_from_upstream.md). The sign of `external_detection_translation_x` matters: it should be **+0.18** (shift toward robot), not −0.18.

This depends on your `apriltag_ros` version's solvePnP convention. Verify by echoing `/detected_dock_pose` at staging — it should be in front of the tag (between the robot and the wall), not behind.

#### Docking server does not start at all

- Confirm `opennav_docking` is in the `lifecycle_manager_navigation` autostart list.
- Check action servers exist:

  ```bash
  ros2 action list | grep -E 'dock_robot|undock_robot|navigate_to_pose'
  ```

- Confirm `dock_trigger.py` is running:

  ```bash
  ros2 node list | grep dock_trigger
  ```

---

### 2.6 Simulation-specific

#### **[sim]** Robot moves a little, then BT recoveries fail with "Collision Ahead"

Check whether the robot has tipped over (look at the Gazebo viewport). Tipping causes the lidar to scan horizontally near the ground → phantom obstacles → recovery behaviours abort.

**If it tipped:**
- Lower `desired_linear_vel` in RPP and `max_accel` in `velocity_smoother`.
- See [`12_lessons_learned.md`](12_lessons_learned.md) for the caster radius / drive wheel traction tradeoff.

#### **[sim]** Camera dropping frames: `Image: 0.2 Hz, CameraInfo: 15 Hz`

The gz plugin or `ros_gz_bridge` can't keep up at high camera resolutions.

**Fix:** Keep the camera at **640×480 @ 15 Hz** in
`omr_description/urdf/gazebo_control.xacro` (the camera sensor block).
HD (1280×720) consistently stalls the bridge. Compensate for resolution
with a larger physical tag (0.40 m vs 0.25 m).

See [`12_lessons_learned.md`](12_lessons_learned.md) for the full diagnostic.

#### **[sim]** RViz shows nothing despite the launch succeeding

Wait ~10 s for SLAM and Nav2 lifecycle to activate. The map first appears once SLAM has processed at least one scan.

Check `/tf_static` and `/robot_description` are published:

```bash
ros2 topic echo /robot_description --once   # should print URDF
ros2 topic echo /tf_static --once           # should include the camera optical frame
```

---

### 2.7 Process / launch

#### **[sim]** `Address already in use` or weird ROS errors after a restart

Previous `gz-sim` or ROS component containers didn't fully terminate. Run:

```bash
~/Downloads/openamrobot-docking-main/openamrobot_docking/scripts/kill_sim.sh
```

(Adjust path to your workspace.) Then relaunch.

#### Build fails: `Could not find a package configuration file provided by ...`

Missing dependencies. Install via:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

Or apt-install missing `ros-jazzy-<package>` directly (see [`01_quickstart.md`](01_quickstart.md) for the full list).

---

## 3. FAQ — common questions

### Do I need the physical AprilTag to test?

**It depends on what you want to test.**

- **With a real camera and real robot:** yes, you need the printed tag.
- **In simulation (Gazebo):** no. The tag is a 3D model in the world; the simulated camera detects it virtually. No printing required.
- **Testing just node startup and TF wiring:** no tag needed. You can publish a fake `camera → charging_dock_apriltag` TF with a static transform publisher to simulate a detected tag.

### Is camera calibration strictly required (real robot)?

**Yes**, for the `image_proc::RectifyNode` to produce `image_rect`. Without it, `camera_info` contains a zero matrix and the rectify node rejects every frame.

In simulation, the simulated camera publishes a valid `camera_info` automatically — no calibration needed.

### Can I skip rectification?

Yes, with reduced accuracy. Remove the `RectifyNode` from the component container in `launch/apriltag.launch.yml` and remap the apriltag node directly to `image_raw` instead of `image_rect`. Tag detections will be slightly less accurate, especially near image edges where lens distortion is highest.

### How do I visualize what's happening?

RViz config is bundled — the simulation launch opens it automatically with the right layout. For manual use:

```bash
rviz2
```

Useful displays:

| Display | Topic / Frame | Shows |
| :---- | :---- | :---- |
| TF | — | Full TF tree including camera and tag frames |
| PoseStamped | `/detected_dock_pose` | Estimated dock pose in the map |
| Image | `/camera/image_raw` (sim) / `/camera/image_rect` (real) | Camera feed |
| Map | `/map` | Navigation map |
| Path | `/plan` | Planned navigation path |
| RobotModel | — | The OMR robot URDF |
| LaserScan | `/scan` | Lidar returns |

Set **Fixed Frame** to `map`.

### Why does the package require CycloneDDS specifically?

Because the default FastDDS on Jazzy has a Python action-client crash bug that silently kills `dock_trigger.py`. The bug surfaces as a `symbol lookup error: ... fastcdr` and is reproducible on every Jazzy install we tested. CycloneDDS is the standard workaround. See [`12_lessons_learned.md`](12_lessons_learned.md) for the diagnostic story.

---

## 4. Reporting a new issue

If you hit a failure not covered here:

1. Capture the exact error message and the command that triggered it.
2. Run `ros2 doctor` and copy the output.
3. Note the build state: `git rev-parse HEAD` and `colcon list --packages-select openamrobot_docking`.
4. For runtime failures, attach the relevant launch logs.
5. Open an issue on the upstream repository following [`../../CONTRIBUTING.md`](../../CONTRIBUTING.md).
