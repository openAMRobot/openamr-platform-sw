# Architecture Overview

This document explains the high-level architecture of `openamr-platform-sw`.

It is meant as a system map for contributors. The README gives a short quickstart; this file explains how the main ROS 2 packages fit together, which package owns which responsibility, and how data moves through the simulation, navigation, and docking stack.

For detailed docking internals, see:

```text
ros2/src/openamrobot_docking/docs/02_architecture.md
ros2/src/openamrobot_docking/docs/03_tf_frames.md
ros2/src/openamrobot_docking/docs/08_legacy_sequencer.md
```

---

## 1. System Scope

`openamr-platform-sw` is the ROS 2 software stack for the OpenAMRobot mobile robot platform.

The current working path is the simulation stack:

```text
Gazebo Harmonic -> ROS 2 bridge -> Nav2 -> AprilTag docking sequencer
```

The active, buildable packages are:

| Package | Responsibility |
|---|---|
| `openamrobot_description` | Robot URDF/xacro, meshes, sensor frames, Gazebo plugin tags |
| `openamrobot_gazebo` | Gazebo world launch, robot spawn, `ros_gz_bridge` configuration |
| `openamrobot_nav2` | Map, AMCL localization, Nav2 stack, RViz navigation layout |
| `openamrobot_docking` | AprilTag detection launch, dock pose publishing, dock/undock sequencer |

The reserved placeholder areas are:

| Area | Planned responsibility |
|---|---|
| `openamrobot_bringup` | Future top-level real-robot launch compositions |
| `openamrobot_control` | Future low-level control and `ros2_control` integration |
| `openamrobot_drivers` | Future hardware drivers for lidar, camera, motor controller, IMU |
| `openamrobot_perception` | Future perception beyond docking |

The placeholders reserve architectural space but are not the working simulation stack today.

---

## 2. Workspace Layout

The repository root is:

```text
openamr-platform-sw/
```

The ROS 2 workspace root is:

```text
openamr-platform-sw/ros2/
```

Build from `ros2/`:

```bash
cd openamr-platform-sw/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The package layout is:

```text
ros2/src/
├── openamrobot_description/
├── openamrobot_gazebo/
├── openamrobot_nav2/
├── openamrobot_docking/
├── openamrobot_bringup/       reserved placeholder
├── openamrobot_control/       reserved placeholder
├── openamrobot_drivers/       reserved placeholder
└── openamrobot_perception/    reserved placeholder
```

---

## 3. Package Boundaries

The project keeps package ownership strict so files are easy to find and changes do not leak across layers.

| Package | Owns | Should not own |
|---|---|---|
| `openamrobot_description` | URDF/xacro, meshes, robot links, joints, sensor frames, Gazebo plugin tags attached to the robot | Worlds, maps, Nav2 parameters, docking logic |
| `openamrobot_gazebo` | Gazebo world launch, robot spawn, ROS/Gazebo bridge configuration, world files | Robot URDF source, Nav2 config, docking sequence logic |
| `openamrobot_nav2` | AMCL, Nav2 launch files, maps, RViz navigation config, navigation parameters | Gazebo launch, robot meshes, AprilTag/docking control |
| `openamrobot_docking` | AprilTag launch, camera info sync, dock pose publisher, dock model, docking sequencer, one-command sim bringup wrapper | Robot model, Gazebo bridge base config, Nav2 internals |

Packages may reference each other through launch composition using `FindPackageShare` and `IncludeLaunchDescription`.

Packages should not duplicate another package's files.

---

## 4. One-Command Simulation Bringup

The main simulation entry point is:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

This launch file composes three layers:

| Startup order | Package | Launch file | Purpose |
|---|---|---|---|
| 1 | `openamrobot_gazebo` | `gz_simulator.launch.py` | Start Gazebo, spawn robot, publish robot description, bridge Gazebo topics |
| 2 | `openamrobot_nav2` | `sim_bringup_launch.py` | Start AMCL, Nav2, map server, optional RViz |
| 3 | `openamrobot_docking` | `openamrobot_docking.launch.py` | Start AprilTag detection, dock pose publisher, dock trigger node |

`bringup_sim.launch.py` starts Gazebo immediately, Nav2 after `nav2_delay`, and docking after `docking_delay`.

Default timing:

```text
Gazebo:  t + 0 s
Nav2:    t + 8 s
Docking: t + 16 s
```

On slower machines:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py nav2_delay:=10 docking_delay:=22
```

Headless mode:

```bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

---

## 5. Layer Responsibilities

### Gazebo Layer

Started by:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Main responsibilities:

- Load the Gazebo world.
- Expand the robot xacro into `/robot_description`.
- Spawn the robot into Gazebo.
- Start `robot_state_publisher`.
- Start `joint_state_publisher`.
- Start `ros_gz_bridge` using `openamrobot_gazebo/config/gz_bridge.yaml`.

Important topics bridged by this layer:

| Topic | Direction | Purpose |
|---|---|---|
| `/clock` | Gazebo to ROS | Simulation time |
| `/tf` | Gazebo to ROS | Dynamic transforms from simulation |
| `/odom` | Gazebo to ROS | Odometry |
| `/scan` | Gazebo to ROS | 2D lidar |
| `/rgb_image` | Gazebo to ROS | Simulated RGB camera image |
| `/cmd_vel` | ROS to Gazebo | Velocity command to the diff-drive plugin |

Gazebo also publishes camera info. The docking layer bridges the root-level `/camera_info` topic needed by the AprilTag pipeline.

### Navigation Layer

Started by:

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py
```

Main responsibilities:

- Start localization on `maps/my_map.yaml`.
- Start the Nav2 planner, controller, behavior server, BT navigator, velocity smoother, and collision monitor.
- Start RViz if `use_rviz:=true`.
- Provide the `/navigate_to_pose` action used by docking Phase 1.

Important inputs:

| Input | Why it matters |
|---|---|
| `/scan` | Costmaps and localization |
| `/odom` | Robot motion estimate |
| `/tf` | Frame relationships |
| `/map` | Static map for localization and planning |

Important navigation interfaces:

| Interface | Purpose |
|---|---|
| `/navigate_to_pose` | Action server for navigation goals |
| `/goal_pose_nav` | Goal topic consumed by Nav2 after the docking gate remap |
| `/cmd_vel` | Final velocity command after Nav2 smoothing/collision monitoring |

### Docking Layer

Started by:

```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

Main responsibilities:

- Bridge Gazebo camera info to ROS `/camera_info`.
- Stamp camera info to match `/rgb_image` as `/camera_info_synced`.
- Start AprilTag detection on `/rgb_image` and `/camera_info_synced`.
- Publish the detected dock pose in the `map` frame.
- Run the Python dock/undock trigger node.

Important nodes:

| Node | Purpose |
|---|---|
| `camera_info_bridge` | Bridges Gazebo camera info to root-level ROS `/camera_info` |
| `camera_info_sync` | Produces `/camera_info_synced` stamped with image time |
| `apriltag_ros` node | Detects the 3-tag AprilTag bundle (IDs 0/1/2) on the dock |
| `detected_dock_pose_publisher` | Publishes the centre tag's pose as `/detected_dock_pose` |
| `dock_trigger.py` | Runs the bundle docking and undocking sequence (camera-centric, normal estimation from the outer tags' wide baseline) |

Important topics:

| Topic | Purpose |
|---|---|
| `/dock_trigger` | Publish `std_msgs/Bool true` to start docking |
| `/undock_robot` | Publish `std_msgs/Bool true` to undock |
| `/rgb_image` | Camera image used by AprilTag detection |
| `/camera_info` | Raw bridged camera info |
| `/camera_info_synced` | Camera info stamped to match image frames |
| `/apriltag/detections` | Raw AprilTag detections |
| `/detected_dock_pose` | Dock pose as `PoseStamped` in `map` |
| `/cmd_vel` | Direct velocity output during docking phases 2, 3, and 4 |

---

## 6. Main Data Flow

The current simulation data flow is:

```text
Gazebo world
  -> sensors and robot physics
  -> ros_gz_bridge
  -> ROS topics: /clock, /scan, /odom, /tf, /rgb_image
  -> Nav2 localization and planning
  -> docking AprilTag detection
  -> dock_trigger.py
  -> /cmd_vel
  -> ros_gz_bridge
  -> Gazebo diff-drive plugin
  -> robot motion
```

The shortest useful mental model:

```text
Sensors -> localization -> planning/docking -> /cmd_vel -> Gazebo wheels
```

If the robot does not move, debug from right to left:

1. Is `/cmd_vel` being published?
2. Is `ros_gz_bridge` subscribed to `/cmd_vel`?
3. Is Gazebo running?
4. Is the diff-drive plugin loaded?
5. Are `/odom` and `/tf` changing?

---

## 7. Velocity Command Flow

All motion eventually reaches Gazebo through ROS `/cmd_vel`.

Navigation flow:

```text
Nav2 controller
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel
  -> ros_gz_bridge
  -> Gazebo /cmd_vel
  -> DiffDrive plugin
```

Docking flow:

```text
dock_trigger.py
  -> /navigate_to_pose action during Phase 1
  -> direct /cmd_vel during Phases 2, 3, and 4
  -> ros_gz_bridge
  -> Gazebo /cmd_vel
  -> DiffDrive plugin
```

This split is intentional. Phase 1 uses Nav2 to reach the staging zone. The final camera-guided scan, align, and approach phases publish directly to `/cmd_vel` for precise, simple control.

---

## 8. Perception and Dock Pose Flow

The docking perception chain is:

```text
Gazebo camera
  -> /rgb_image
  -> /camera_info
  -> camera_info_sync
  -> /camera_info_synced
  -> apriltag_ros
  -> TF: camera_optical_frame -> charging_dock_tag_{0,1,2}
  -> detected_dock_pose_publisher (consumes the centre tag, id 1)
  -> /detected_dock_pose
  -> dock_trigger.py
```

`apriltag_ros` publishes one TF per detected tag in the 3-tag bundle (outer tags at `y = ±0.45 m`, centre tag at `y = 0`). `detected_dock_pose_publisher` looks up the **centre tag** `charging_dock_tag_1` in the `map` frame and republishes it as a `PoseStamped`. The outer tags `charging_dock_tag_0` and `charging_dock_tag_2` are consumed directly by the sequencer to estimate the dock surface normal from a wide baseline.

This lets the docking sequencer reason about the dock in the same global frame used by Nav2.

---

## 9. TF Architecture

The key frame chain is:

```text
map
  -> odom
  -> base_link
  -> camera_link
  -> camera_optical_frame
  -> charging_dock_tag_{0,1,2}        (one TF per tag in the bundle)
```

Frame ownership:

| Transform | Publisher |
|---|---|
| `map -> odom` | AMCL/localization |
| `odom -> base_link` | Gazebo diff-drive odometry through the bridge |
| `base_link -> camera_link` | `robot_state_publisher` from URDF |
| `camera_link -> camera_optical_frame` | `robot_state_publisher` from URDF |
| `camera_optical_frame -> charging_dock_tag_{0,1,2}` | `apriltag_ros` (one TF per detected bundle tag) |

Useful checks:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo map charging_dock_tag_1      # centre tag = docking target
ros2 run tf2_ros tf2_echo map charging_dock_tag_0      # left outer tag
ros2 run tf2_ros tf2_echo map charging_dock_tag_2      # right outer tag
```

Generate a TF diagram:

```bash
ros2 run tf2_tools view_frames
```

For deeper TF details:

```text
ros2/src/openamrobot_docking/docs/03_tf_frames.md
```

---

## 10. Docking Sequence

The current simulation docking path is a custom multi-phase, camera-centric bundle sequencer in:

```text
ros2/src/openamrobot_docking/scripts/dock_trigger.py
```

It estimates the dock surface normal from the outer tags' wide baseline (90 cm),
follows that normal with a pure-pursuit controller in the camera/tag frame, and
finishes with an axis-frozen visual servo on the centre tag. See
`ros2/src/openamrobot_docking/docs/14_docking_research.md` for the full design rationale.

Trigger:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

Phases:

| Phase | Behavior | Main dependency |
|---|---|---|
| 1 | Navigate to staging pose near the dock | Nav2 `/navigate_to_pose` |
| 2 | Rotate/scan until the AprilTag is centered, then collect filtered samples | Camera, AprilTag, TF |
| 3 | Spin in place to align perpendicular to the dock | TF and `/cmd_vel` |
| 4 | Drive forward along the filtered dock line until final distance | TF and `/cmd_vel` |

Undock:

```bash
ros2 topic pub /undock_robot std_msgs/msg/Bool "{data: true}" --once
```

For phase-level detail:

```text
ros2/src/openamrobot_docking/docs/08_legacy_sequencer.md
```

---

## 11. Simulation Time

The simulation uses Gazebo `/clock`.

Every simulation node should use:

```text
use_sim_time: true
```

Why this matters:

- TF timestamps must agree.
- Nav2 costmaps depend on consistent time.
- AprilTag transforms must line up with camera frames.
- Mixed wall time and sim time can cause TF extrapolation errors.

Quick check:

```bash
ros2 topic echo /clock --once
```

If TF errors mention extrapolation into the future or past, check `use_sim_time` first.

---

## 12. Design Rules for Contributors

Use these rules when changing the architecture:

1. Keep robot model files in `openamrobot_description`.
2. Keep Gazebo worlds and bridge config in `openamrobot_gazebo`.
3. Keep navigation maps, Nav2 params, and RViz navigation layouts in `openamrobot_nav2`.
4. Keep AprilTag, dock pose, and docking sequence logic in `openamrobot_docking`.
5. Do not duplicate files across packages.
6. Prefer launch composition over copying package assets.
7. Document topic, TF, parameter, or launch behavior changes in the owning package README.
8. For simulation changes, test the one-command bringup and the three-layer manual bringup.

---

## 13. Where to Read Next

| Topic | Document |
|---|---|
| Beginner setup | `docs/getting_started/DEVELOPER_SETUP.md` |
| General troubleshooting | `docs/getting_started/TROUBLESHOOTING.md` |
| Git and fork workflow | `docs/getting_started/GIT_GUIDE.md` |
| Docking architecture | `ros2/src/openamrobot_docking/docs/02_architecture.md` |
| TF frames | `ros2/src/openamrobot_docking/docs/03_tf_frames.md` |
| AprilTag setup | `ros2/src/openamrobot_docking/docs/04_apriltag.md` |
| Docking parameters | `ros2/src/openamrobot_docking/docs/05_parameters.md` |
| Perception + perpendicular line + RViz/Gazebo markers | `ros2/src/openamrobot_docking/docs/13_perception_and_line.md` |
| Vendor-agnostic precision-docking research | `ros2/src/openamrobot_docking/docs/14_docking_research.md` |
| Legacy 4-phase sequencer notes (superseded by the bundle pipeline) | `ros2/src/openamrobot_docking/docs/08_legacy_sequencer.md` |
| Docking troubleshooting | `ros2/src/openamrobot_docking/docs/09_troubleshooting.md` |
