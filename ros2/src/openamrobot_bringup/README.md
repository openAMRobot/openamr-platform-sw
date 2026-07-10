# openamrobot_bringup

Top-level launch composition for OpenAMRobot. Wires subsystems together; contains no
low-level nodes, models, or Nav2 parameters of its own (those live in their packages).

## `bringup.launch.py` — pick simulation or real

The single entry point. The **same** Nav2 stack and the **same** `nav2_params.yaml` run in
both cases; only the **data source** and the **clock** change.

```bash
ros2 launch openamrobot_bringup bringup.launch.py                       # real hardware (default)
ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_rviz:=true   # Gazebo
ros2 launch openamrobot_bringup bringup.launch.py map:=/path/to/real_map.yaml
```

| `sim:=` | Data source | Window(s) | `use_sim_time` | Hardware |
|---|---|---|---|---|
| **false** (default) | `real_bringup.launch.py` (drivers + perception + EKF + TFs) | RViz only, if `use_rviz:=true` | `false` (real clock) | **required** + a real map |
| **true** | `openamrobot_gazebo` (Gazebo + gz_bridge + robot_state_publisher) | **Gazebo** + optional RViz | `true` (Gazebo clock) | none |

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `false` | `true` = Gazebo simulation, `false` = real hardware |
| `use_rviz` | `false` | open RViz with the Nav2 view |
| `map` | `…/openamrobot_nav2/maps/my_map.yaml` | map for AMCL (pass your real map when `sim:=false`) |

### Sending a navigation goal (RViz "2D Goal Pose")

`navigation_launch.py` remaps `bt_navigator`'s goal input `goal_pose → goal_pose_nav` so the
**docking** node can gate `/goal_pose` (undock-before-navigate). So a goal published on
`/goal_pose` only reaches Nav2 if something forwards it to `/goal_pose_nav`. On the **real robot**
this is controlled by `use_docking`:

- **`use_docking:=true`** (default) — launches `openamrobot_docking/docking_real.launch.py` (AprilTag
  on the real camera → `/detected_dock_pose`, + `dock_trigger`). `dock_trigger` **owns** `/goal_pose`:
  it forwards a goal to `/goal_pose_nav` immediately when not docked, or undocks first when docked, and
  also performs AprilTag docking on `/dock_trigger`. This is the real port of the docking pipeline.
- **`use_docking:=false`** — nav-only debug: a plain `topic_tools relay /goal_pose → /goal_pose_nav`
  (no docking). Use when you don't have the physical dock or just want to test navigation.
- **sim** — same `use_docking` switch: `use_docking:=true` (default) folds the Gazebo docking layer
  (`openamrobot_docking.launch.py` — apriltag_sim + `dock_trigger`) into `sim:=true`, so one command
  runs Gazebo + Nav2 + docking. This replaces the legacy `openamrobot_docking/bringup_sim.launch.py`
  (kept working for backwards compatibility). `use_docking:=false` falls back to the plain relay.

The standard RViz "2D Goal Pose" tool (publishes `/goal_pose`) works in all cases.

> **Real-dock prerequisites** (for `use_docking:=true` to actually dock): a physical dock with the
> 3-tag 36h11 bundle (IDs 0/1/2); the printed tag size set in
> `openamrobot_docking/config/tags_36h11.yaml`; the camera calibrated; and the dock pose set in
> `openamrobot_docking/config/dock_trigger.yaml` for your real map. Without the dock, navigation still
> works (goals are forwarded) — only docking/undocking is inactive.

## Individual commands (compose the stack by hand)

`bringup.launch.py` is the one-shot command. But you can also launch each **layer in its own
terminal** to debug or "do what you want" — and pick the forwarder yourself. The layers are the same
ones `bringup.launch.py` includes; the only rule to respect is the forwarder.

> **THE ONE RULE — exactly one goal forwarder on `/goal_pose_nav`:**
> nav-only → launch `goal_relay.launch.py`; **OR** docking → launch the docking layer (its
> `dock_trigger` is the forwarder). **Never both.** As soon as you launch docking, it owns the goal
> routing, so do **not** also run the relay. `bringup.launch.py` makes this choice for you via
> `use_docking`; here you make it by hand.

Every terminal first needs the same sourcing + env (`/opt/ros/jazzy`, the workspace `install`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=0`).

### Simulation — per terminal

```bash
# T1 — Gazebo (data source + clock)
ros2 launch openamrobot_gazebo gz_simulator.launch.py gui:=true

# T2 — Nav2 (localization + navigation + RViz), use_sim_time hard-set to true
ros2 launch openamrobot_nav2 sim_bringup_launch.py use_rviz:=true

# T3 — forwarder: pick EXACTLY ONE
#   (a) nav-only  — plain relay /goal_pose -> /goal_pose_nav
ros2 launch openamrobot_bringup goal_relay.launch.py
#   (b) docking   — apriltag_sim + dock_trigger (dock_trigger IS the forwarder); do NOT run (a)
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

This is exactly what `bringup.launch.py sim:=true` runs in one process (with `use_docking` choosing
T3a vs T3b). The legacy `openamrobot_docking/bringup_sim.launch.py` chains the same three with delays.

### Real robot — per terminal

```bash
# T1 — data source (drivers + perception + camera + EKF + static TFs)
ros2 launch openamrobot_bringup real_bringup.launch.py

# T2 — Nav2 with your real map (AMCL needs a non-empty map)
ros2 launch openamrobot_nav2 localization_launch.py map:=/path/to/real_map.yaml use_sim_time:=false
ros2 launch openamrobot_nav2 navigation_launch.py use_sim_time:=false use_scan_filter:=false

# T3 — forwarder: pick EXACTLY ONE
#   (a) nav-only
ros2 launch openamrobot_bringup goal_relay.launch.py
#   (b) docking on the real camera — dock_trigger IS the forwarder; do NOT run (a)
ros2 launch openamrobot_docking docking_real.launch.py
```

The detailed real-robot per-terminal procedure (SSH, startup order, all the gotchas) is in
`docs/procedures/real-robot-runbook.md`.

## `real_bringup.launch.py`

Brings up the **real-robot data sources** so the same Nav2 / SLAM / docking stack runs on
hardware as in simulation. Only the data source differs:

| Layer | Simulation | Real (this launch) |
|---|---|---|
| odom / scan / imu / camera | `openamrobot_gazebo` (Gazebo + `gz_bridge`) | `openamrobot_drivers` + `openamrobot_perception` + EKF |

It composes:
- `openamrobot_drivers/drivers.launch.py` — micro-ROS agent (Teensy) + RPLIDAR → `/scan`;
- `openamrobot_perception/scan_body_filter.launch.py` — `/scan` → `/scan_filtered`;
- `openamrobot_perception/camera.launch.py` — `camera_ros` (IMX708);
- `robot_localization` **EKF** (`config/ekf.yaml`) — wheels + IMU gyro-Z → `/odom` + TF
  `odom→base_link`;
- **measured static TFs** for this unit (`base_link→{lidar_link, imu_link, base_footprint,
  camera_link→camera_optical_frame}`).

```bash
ros2 launch openamrobot_bringup real_bringup.launch.py
```

### EKF (`config/ekf.yaml`)

Fuses `/odom/unfiltered` (wheel vx, vyaw) + `/imu/data` (**only** `angular_velocity.z`).
The MPU-6500 has no valid orientation quaternion and a tilted accelerometer, so the EKF runs
`two_d_mode: true` with gyro-Z only. `transform_time_offset: 0.2` dates the published TF
forward because the LiDAR scan arrives ~0.14 s ahead of the TF (micro-ROS latency),
preventing "extrapolation into the future" on the SLAM side.

### Measured static TFs (unit-specific)

The LiDAR is mounted **rotated 180°** (`yaw=π`), 0.335 m ahead of the axle, 0.18 m up. The
camera is 0.415 m ahead, 0.12 m up. Re-measure if the sensor mounts change. These describe
**this** robot; on another unit, edit the values in `real_bringup.launch.py`.

> `real_bringup.launch.py` is the **data source only**. For the full stack (data source +
> Nav2), use `bringup.launch.py` above — it adds localization + navigation with the shared
> `openamrobot_nav2/config/nav2_params.yaml` and the right `use_sim_time`.
