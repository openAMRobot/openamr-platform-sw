# Changes from the Original Repository

This document lists every file that was added or modified compared to the original `main` branch on GitHub (`openAMRobot/openamrobot-docking`). It explains what changed and why. Read this before modifying any config or launch file so you understand the intent behind each decision.

---

## Summary

| File | Status | Reason |
|---|---|---|
| `.gitignore` | **Created** | Exclude colcon build artifacts from version control |
| `package.xml` | **Modified** | Add 5 missing runtime dependencies |
| `config/tags_36h11.yaml` | **Created** | AprilTag config adapted for this robot (tag ID, size, frame name) |
| `config/openamrobot_docking.yaml` | **Modified** | Fix two misleading comments |
| `launch/apriltag.launch.yml` | **Created** | Local copy of the system launch pointing to local config |
| `launch/openamrobot_docking.launch.py` | **Modified** | Point to local apriltag launch + forward `use_sim_time` |
| `docs/06_camera_calibration.md` | **Created** | Step-by-step camera calibration and AprilTag detection guide |
| `docs/11_changes_from_upstream.md` | **Created** | This file |
| `docs/09_troubleshooting.md` | **Created** | Answers to setup and reproduction questions |
| `docs/12_lessons_learned.md` | **Created** | Full code audit with findings and severity |

---

## 1. `.gitignore` — Created

### What it is

A new file at the repository root.

### Content

```
build/
install/
log/
__pycache__/
*.py[cod]
*.egg-info/
*.pyc
```

### Why

`colcon build` generates three directories (`build/`, `install/`, `log/`) that must not be committed. Without a `.gitignore`, these would appear as untracked files in `git status` and could accidentally be staged and pushed.

---

## 2. `package.xml` — Modified

### What changed

Five `exec_depend` entries were added.

**Before:**
```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nav2_lifecycle_manager</exec_depend>
<exec_depend>opennav_docking</exec_depend>
<exec_depend>opennav_docking_msgs</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

**After:**
```xml
<exec_depend>apriltag_msgs</exec_depend>
<exec_depend>apriltag_ros</exec_depend>
<exec_depend>camera_ros</exec_depend>
<exec_depend>image_proc</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nav2_lifecycle_manager</exec_depend>
<exec_depend>opennav_docking</exec_depend>
<exec_depend>opennav_docking_msgs</exec_depend>
<exec_depend>rclcpp_components</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### Why

The five added packages are all required to run the stack but were not declared:

| Package | Where it is used |
|---|---|
| `apriltag_ros` | `launch/apriltag.launch.yml` — plugin `AprilTagNode` |
| `apriltag_msgs` | Published by `AprilTagNode` on `/apriltag/detections` |
| `camera_ros` | `launch/apriltag.launch.yml` — plugin `camera::CameraNode` |
| `image_proc` | `launch/apriltag.launch.yml` — plugin `image_proc::RectifyNode` |
| `rclcpp_components` | `launch/apriltag.launch.yml` — exec `component_container` |

Without these declarations, `rosdep install` would not install them and a new user would get a broken install with no clear error message.

---

## 3. `config/tags_36h11.yaml` — Created

### What it is

A new file. The original repository had no tag config — it relied entirely on the system file at `/opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml`, which cannot be modified (read-only system path).

### Content

```yaml
/**:
    ros__parameters:
        image_transport: raw
        family: 36h11
        size: 0.0555            # tag edge size in meters
        max_hamming: 0

        detector:
            threads: 1
            decimate: 2.0
            blur: 0.0
            refine: True
            sharpening: 0.25
            debug: False

        pose_estimation_method: "pnp"

        tag:
            ids: [0]
            frames: [charging_dock_apriltag]
```

### What changed compared to the system defaults and why

| Parameter | System default | Our value | Why |
|---|---|---|---|
| `size` | `0.173` | `0.0555` | Measured on the physical printed tag. Wrong size = wrong pose estimation. |
| `tag.ids` | `[9, 14]` | `[0]` | The tag printed for the dock is tag ID 0. |
| `tag.frames` | `[base, object]` | `[charging_dock_apriltag]` | Must match `child_frame` in `config/docking_pose_publisher.yaml`. If these differ, the pose publisher cannot find the transform. |
| `tag.sizes` | `[0.162, 0.162]` | *(removed)* | Not needed when all tags share the same global `size`. |

### What you must adapt

- **`size`**: measure your printed tag and update this value. Even a few millimeters of error shifts the estimated dock pose.
- **`tag.ids`**: update if you print a different tag ID.
- **`tag.frames`**: must stay in sync with `child_frame` in `config/docking_pose_publisher.yaml`.

---

## 4. `config/openamrobot_docking.yaml` — Modified

### What changed

Two comments were corrected.

**Line 35 — before:**
```yaml
# CHANGED: enable stall detection so it stops if it physically can't move
use_stall_detection: false
```

**Line 35 — after:**
```yaml
# CHANGED: disabled — enable if robot physically stalls against the dock
use_stall_detection: false
```

**Line 62 — before:**
```yaml
# CHANGED: enable collision detection so it stops before hitting/pushing into wall
use_collision_detection: false
```

**Line 62 — after:**
```yaml
# CHANGED: disabled — enable only if obstacles are expected near the dock
use_collision_detection: false
```

### Why

The original comments said "enable" but the values were `false`, which is contradictory. The comments implied the features were being turned on, when they were actually turned off. The new comments accurately describe the state of the parameter and when to change it.

---

## 5. `launch/apriltag.launch.yml` — Created

### What it is

A local copy of `/opt/ros/jazzy/share/apriltag_ros/launch/camera_36h11.launch.yml`.

### What changed compared to the system file

One line changed: the `param: from:` line that loads the tag configuration.

**System file:**
```yaml
param:
- from: $(find-pkg-share apriltag_ros)/cfg/tags_36h11.yaml
```

**Our file:**
```yaml
param:
- from: $(find-pkg-share openamrobot_docking)/config/tags_36h11.yaml
```

### Why

The system tag config had wrong tag IDs, wrong size, and wrong frame names for this robot. Since `/opt/ros/jazzy/` is read-only, the only option is a local copy of the launch file that loads our own config. Everything else (node structure, remaps, intra-process comms) is identical to the original.

> **If you upgrade `apriltag_ros`**, check whether the system launch file changed and update `launch/apriltag.launch.yml` accordingly.

---

## 6. `launch/openamrobot_docking.launch.py` — Modified

### What changed

Two changes in the `IncludeLaunchDescription` block for the AprilTag stack.

**Before:**
```python
IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('apriltag_ros'), 'launch', 'camera_36h11.launch.yml']
        )
    ),
),
```

**After:**
```python
IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('openamrobot_docking'), 'launch', 'apriltag.launch.yml']
        )
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
),
```

### Why — change 1: local launch file

The original pointed to `apriltag_ros/launch/camera_36h11.launch.yml`, a system file that hardcodes the system tag config path. Since the system config has the wrong tag parameters and cannot be modified, we now point to the local `apriltag.launch.yml` which uses our local `config/tags_36h11.yaml`.

### Why — change 2: `use_sim_time` forwarding

The original block had no `launch_arguments`, so `camera_ros`, `image_proc::RectifyNode`, and `AprilTagNode` always used wall clock time regardless of the `use_sim_time` launch argument. This causes TF timestamp mismatches in simulation because all other nodes use the Gazebo sim clock. The fix forwards `use_sim_time` to the apriltag container, matching the pattern already used for the `detected_dock_pose_publisher` include above it.

---

## 7. Documentation files — Created

Four documentation files were added to `docs/`:

| File | Contents |
|---|---|
| `docs/06_camera_calibration.md` | Step-by-step guide: install dependencies, build, print calibration targets, calibrate the camera, verify AprilTag detection. Includes all problems encountered and their fixes. |
| `docs/11_changes_from_upstream.md` | This file. |
| `docs/09_troubleshooting.md` | Answers to questions raised during initial setup: pipeline explanation, whether a physical tag is required, calibration bypass options, missing dependencies, Gazebo world status, FastDDS crash fix, exact command lines, RViz setup. |
| `docs/12_lessons_learned.md` | Full audit of the repository: undeclared dependencies, complete ROS 2 interface per node, platform-specific vs site-specific parameters, TF chain consistency check, and all inconsistencies found with exact file and line references. |

---

## What was NOT changed

| File | Why untouched |
|---|---|
| `launch/detected_dock_pose_publisher.launch.py` | Works correctly as-is |
| `config/docking_pose_publisher.yaml` | Already configured for `charging_dock_apriltag` and `map` frame |
| `config/dock_trigger.yaml` | Parameters are correct |
| `config/openamrobot_docking_backup.yaml` | Left in place — flagged in `docs/12_lessons_learned.md` as a confusing artifact |
| `src/detected_dock_pose_publisher.cpp` | No changes needed |
| `scripts/dock_trigger.py` | No changes needed |
| `CMakeLists.txt` | No changes needed |

---

## Files created outside the repository

One file must be created on each machine but is not part of the repository:

| File | What it is |
|---|---|
| `~/.ros/camera_info/<camera_name>.yaml` | Camera calibration file generated by `camera_calibration` |

This file is machine-specific (encodes the physical camera's intrinsic parameters). Every person setting up the system must generate their own by following `docs/06_camera_calibration.md`. The `camera_name` field inside the file must be corrected from the default `narrow_stereo` to the actual camera identifier — see section 4.6 of that guide.

---

## Subsequent additions (Gazebo simulation + custom 4-phase docking)

After the initial fixes documented above, the repository was extended with a **complete Gazebo Harmonic simulation** and a **custom 4-phase docking sequencer** (centring scan → align → line-tracking pure-pursuit → final-align + straight-line). These are additive — they don't replace the real-robot pipeline.

### New files / directories

| Path | Purpose |
|---|---|
| `omr_description/` | New ROS 2 package providing the canonical OMR robot description (URDF/xacro, STL meshes). Spawned at runtime by the simulation launch via `ros_gz_sim create` |
| `simulation/` | Whole Gazebo Harmonic simulation: world, configs, launch (robot description now provided by `omr_description`) |
| `simulation/launch/simulation.launch.py` | Top-level sim bringup with `spawn_x` / `spawn_y` / `spawn_yaw` launch arguments and auto-derived dock pose |
| `simulation/worlds/docking_world.sdf` | 10×10 m room, AprilTag dock against north wall (robot is spawned from the URDF at launch time, not declared inside the world) |
| `simulation/models/apriltag_dock/` | 0.40 × 0.40 m AprilTag panel, textured with `tag0_big.png` |
| `simulation/config/nav2_sim_full.yaml` | Full Nav2 + opennav_docking + slam_toolbox params for the sim (softer decel for the omr_description casters) |
| `simulation/config/tags_36h11_sim.yaml` | AprilTag config (`size: 0.40`) |
| `simulation/config/ros_gz_bridge.yaml` | gz↔ROS topic bridge |
| `simulation/config/slam_toolbox_params.yaml` | SLAM in mapping mode |
| `simulation/config/simulation.rviz` | RViz layout with `ThirdPersonFollower` camera view of `base_footprint` |
| `launch/apriltag_sim.launch.yml` | apriltag_ros launch for sim (no `camera_ros`) |
| `scripts/kill_sim.sh` | SIGKILL cleanup helper for zombie sim processes |
| `docs/12_lessons_learned.md` | Story of every sim issue encountered + fix |
| `docs/08_sequencer_4phase.md` | Description of the 4-phase sequencer |
| `tag0_big.png` | 200×200 px tag36h11 image, mapped onto the AprilTag panel |

### Changed files

| File | What changed |
|---|---|
| `scripts/dock_trigger.py` | **Completely rewritten** as a 4-phase sequencer (Nav2 to staging → camera-frame centring scan + running-average filter → align spin → line-tracking pure-pursuit then final-align + straight-line). Earlier iterations included a 7-phase pipeline with auto-calibration low-pass and a reverse-and-realign safety loop; both were retired in favour of the continuous pure-pursuit controller. See `08_sequencer_4phase.md`. |
| `config/dock_trigger.yaml` | Now declares: dock pose, staging distance, scan parameters (`scan_rotation_speed`, `scan_consecutive_target`, `scan_centring_tolerance`, `scan_centring_kp`), filter parameters (`filter_num_samples`, `filter_max_collect_time`), spin parameters (`spin_kp`, `spin_max_omega`, `spin_yaw_tolerance`), line-tracking parameters (`drive_speed`, `line_yaw_kp`, `line_lookahead_distance`, `drive_yaw_max_omega`, `visual_servo_distance`, `docking_distance`). |
| `CMakeLists.txt` | Installs `simulation/` directory and `dock_trigger.py`/`kill_sim.sh` scripts |
| `package.xml` | Added `slam_toolbox`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `nav2_msgs`, `tf2_ros`, `omr_description`, `xacro` dependencies |

### Key configuration choices for the simulation

These differ from typical Nav2 defaults and are documented in the simulation YAMLs:

| Param | Value | Reason |
|---|---|---|
| `RegulatedPurePursuitController` | (used) | MPPI's stochastic sampling produced visible oscillations on simple paths |
| `desired_linear_vel` | `0.55` m/s | Compromise between speed and stability on a balanced-on-2-wheels robot |
| `max_angular_accel` | `0.6` rad/s² | Low to limit lateral inertia → less tipping risk |
| `inflation_radius` | `0.45` m | A bit above `robot_radius=0.35` |
| `cost_scaling_factor` | `8.0` | Sharp falloff → less plan wobble |
| `tolerance` (planner) | `1.0` m | Accept any free cell within 1 m if exact goal cell is in transient inflation |
| `use_astar: true` | (planner) | Avoids Dijkstra "potential found, plan failed" rare bug |
| `velocity_smoother.max_accel` | `[1.2, 0, 1.5]` | Conservative — anti-tipping |
| AprilTag panel size | `0.40` m | Bigger tag = better solvePnP accuracy at our 640×480 camera |
| Camera resolution | `640×480 @ 15 Hz` | HD stalled the gz plugin / `ros_gz_bridge` |

For the full reasoning see `12_lessons_learned.md`.
