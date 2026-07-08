# Overview

What the navigation stack is, how it is packaged, and the contract it expects from the
platform. For installation and per-terminal bring-up see the package
[`README`](../../ros2/src/openamrobot_bringup/README.md); for the deeper *why* of each tuned
value continue into [`03_planner_controller.md`](03_planner_controller.md) and
[`04_real_robot_tuning.md`](04_real_robot_tuning.md).

---

## What this stack does

`openamrobot_nav2` runs a standard **Nav2 (Jazzy)** navigation stack on a differential-drive
AMR: it localizes against a pre-built map with **AMCL**, plans a global path with
**SmacPlanner2D**, follows it with a **RotationShimController → DWB** controller, keeps two
costmaps for obstacle avoidance, and passes every velocity command through a
**velocity_smoother** and a **collision_monitor** before it reaches the base.

The key architectural decision is **one stack, two profiles**. The exact same Nav2 nodes and
the exact same
[`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml) run in both
simulation and on the real robot. Only two things change:

| | Simulation (`sim:=true`) | Real robot (`sim:=false`) |
|---|---|---|
| Data source | `openamrobot_gazebo` (Gazebo + `ros_gz_bridge`) | `openamrobot_drivers` + `openamrobot_perception` + EKF + static TFs |
| Clock | `use_sim_time:=true` (Gazebo `/clock`) | `use_sim_time:=false` (wall time) |
| Scan filter | `laser_filters` angular chain (sim profile) | `scan_body_filter` node (real profile) |
| Initial pose | `set_initial_pose: true` at `(0,0)` (robot spawns at origin) | disabled → operator sets it with RViz **2D Pose Estimate** |

Both profiles publish the *same* topics, so the navigation layer above them is identical.
This is what makes the simulation a faithful rehearsal for the hardware.

## What it does **not** do

- **Build maps** — it localizes on a pre-built map. Mapping is a separate `slam_toolbox`
  pass (see the working-repo navigation notes / `slam.yaml`).
- **Dock** — precision docking is a separate camera-guided pipeline in `openamrobot_docking`
  that layers *on top* of Nav2 (Nav2 drives to a staging pose, then the dock sequencer takes
  over). See [`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md).
- **Provide hard functional safety** — the collision_monitor + velocity limits + firmware
  watchdog are the current safety envelope; there is **no hardware E-stop / safety_io** yet.
  See [`../safety/`](../safety/README.md).

---

## Package layout

`openamrobot_nav2` owns the navigation configuration and the launch files; the top-level
bring-up that composes a data source under it lives in `openamrobot_bringup`.

```
ros2/src/
├── openamrobot_nav2/
│   ├── config/
│   │   ├── nav2_params.yaml          ← the whole tuned stack (single source of truth)
│   │   ├── scan_body_filter.yaml     ← SIM profile laser_filters angular chain
│   │   └── slam.yaml                 ← mapping (separate from navigation)
│   ├── launch/
│   │   ├── localization_launch.py    ← map_server + amcl + lifecycle_manager_localization
│   │   └── navigation_launch.py      ← planner/controller/bt/behaviors/smoother/
│   │                                    velocity_smoother/collision_monitor + its lifecycle mgr
│   ├── maps/                         ← sim map (real maps live on the robot)
│   └── rviz/nav2_view.rviz
│
├── openamrobot_bringup/
│   └── launch/
│       ├── bringup.launch.py         ← top-level selector (sim:= / map:= / use_docking:=)
│       ├── real_bringup.launch.py    ← real data source (drivers + perception + EKF + TFs)
│       └── goal_relay.launch.py      ← /goal_pose → /goal_pose_nav forwarder (nav-only)
│
└── openamrobot_perception/
    └── openamrobot_perception/scan_body_filter.py   ← REAL profile scan filter
```

---

## The contract Nav2 expects from the platform

Whatever the data source (Gazebo or real hardware), it must satisfy the same interface, read
straight from `nav2_params.yaml`:

| Nav2 needs | Provided by (real) | Notes |
|---|---|---|
| TF `map → odom` | AMCL | after a 2D Pose Estimate on the real robot |
| TF `odom → base_link` | `robot_localization` EKF (wheels + IMU gyro-Z) | Gazebo bridge in sim |
| TF `base_link → {lidar_link, camera_link, …}` | static TFs in `real_bringup.launch.py` | measured for *this* unit |
| `/scan_filtered` (`LaserScan`) | `scan_body_filter` (real) / `laser_filters` (sim) | body reflections removed |
| `/odom` (`Odometry`) | EKF (real) / bridge (sim) | |
| `/cmd_vel` (`Twist`) → base | consumed by the drivers (real) / diff-drive plugin (sim) | final output of the safety chain |

Exactly **one** publisher of `/scan_filtered` must exist per profile — the duplicate-filter
trap is described in [`04_real_robot_tuning.md`](04_real_robot_tuning.md).

---

## One-command bring-up

```bash
# Real robot, everything (drivers + localization + Nav2 + docking), your map:
ros2 launch openamrobot_bringup bringup.launch.py map:=~/maps/coin2.yaml

# Simulation, everything + RViz:
ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_rviz:=true

# Nav-only debug (no docking → a plain relay forwards goals):
ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_docking:=false use_rviz:=true
```

`sim:=false` **requires** an explicit `map:=` — the bundled sim map is never silently used on
the real robot (a deliberate guard in `bringup.launch.py`). On the real robot, after launch:
set the pose with RViz **2D Pose Estimate**, then send goals with **2D Goal Pose** (not the
"Nav2 Goal" tool — see [`05_goal_routing.md`](05_goal_routing.md)).

> **Before any real-robot navigation test**, confirm the battery is **≥ 25 V at rest**. A soft
> 24 V bus produces weak torque and the robot fails to follow the plan — you then debug Nav2
> for a hardware problem. See [`../safety/02_limits_and_watchdog.md`](../safety/02_limits_and_watchdog.md).

---

## Where to go next

| If you want to… | Read |
|---|---|
| See the node graph, lifecycle, and data flow | [`01_architecture.md`](01_architecture.md) |
| Understand the costmaps and the footprint | [`02_costmaps.md`](02_costmaps.md) |
| Tune the planner / controller | [`03_planner_controller.md`](03_planner_controller.md) |
| Avoid the real-robot traps | [`04_real_robot_tuning.md`](04_real_robot_tuning.md) |
| Understand goal routing | [`05_goal_routing.md`](05_goal_routing.md) |
| Diagnose a navigation failure | [`06_troubleshooting.md`](06_troubleshooting.md) |
