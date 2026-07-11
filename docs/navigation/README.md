# Navigation — engineering documentation

This folder is the engineering record for the **Nav2 navigation stack** as it runs on the
real OpenAMRobot platform (and, unchanged, in simulation). It is the *why* and *how it's
built* — the reasoning behind the tuned values in
[`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml), the node
graph, and the real-robot gotchas that cost us time.

For a system-wide map of the packages, read
[`../architecture/ARCHITECTURE_OVERVIEW.md`](../architecture/ARCHITECTURE_OVERVIEW.md)
first. For docking (a separate pipeline layered on top of Nav2), read
[`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md).
For the safety layer (collision monitor, velocity limits, watchdog, the current gaps), read
[`../safety/`](../safety/README.md).

---

## Layout

```
docs/navigation/
├── README.md                    ← this file (index)
│
├── 00_overview.md               what the stack is, the profiles, the contract
├── 01_architecture.md           node graph, lifecycle, TF and topic/velocity flow
├── 02_costmaps.md               global/local layers, footprint, the empty-costmap gotcha
├── 03_planner_controller.md     SmacPlanner2D + RotationShim→DWB tuning (real values, why)
├── 04_real_robot_tuning.md      velocity floors, sub-stiction yaw, scan gotchas, teleop
├── 05_goal_routing.md           2D Goal Pose → relay/dock_trigger → bt_navigator
└── 06_troubleshooting.md        nav-specific symptom → cause → fix matrix
```

---

## How to read

| Goal | Read |
|---|---|
| **New to the stack** | `00_overview.md` → `01_architecture.md` |
| **Understand the node graph / lifecycle** | `01_architecture.md` |
| **Tune the planner or controller** | `03_planner_controller.md` |
| **Bring it up on the real robot without hitting known traps** | `04_real_robot_tuning.md` + `06_troubleshooting.md` |
| **"I sent a goal and nothing happens"** | `05_goal_routing.md` then `06_troubleshooting.md` |
| **"Costmaps are empty / robot drives blind"** | `02_costmaps.md` + `06_troubleshooting.md` |
| **Safety behaviour (stopping, limits, watchdog)** | [`../safety/`](../safety/README.md) |

---

## Scope

These docs cover **navigation only**: localization, planning, control, costmaps, goal
routing, and their tuning. They deliberately do **not** cover CPU/thermal/power/network
troubleshooting on the Pi 5, nor the full real-hardware bring-up sequence — those belong to
the real-robot series ([`../real_robot/`](../real_robot/README.md)) and are cross-linked
where relevant. Safety behaviour (collision monitor, velocity clamping, firmware watchdog)
lives in [`../safety/`](../safety/README.md).

## Conventions used throughout

- **Frames**: `map → odom → base_link → {lidar_link, camera_link → camera_optical_frame}`.
  `map → odom` is published by AMCL; `odom → base_link` by the EKF (real) or the Gazebo
  bridge (sim); the sensor frames are static TFs (see
  [`01_architecture.md`](01_architecture.md)).
- **Velocity chain**: `controller_server → /cmd_vel_nav → velocity_smoother →
  /cmd_vel_smoothed → collision_monitor → /cmd_vel → base`.
- **Profiles**: the **same** Nav2 stack and the **same** `nav2_params.yaml` run in both
  simulation (`sim:=true`, `use_sim_time:=true`) and on the real robot (`sim:=false`,
  `use_sim_time:=false`). Only the data source and the clock differ.
- **The single source of truth for values** is
  [`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml). Where a doc
  quotes a number, it is quoted from that file; where a value is a documented option that is
  *not* currently in the file, that is stated explicitly.
- **Environment prelude (run this once per shell first).** Every bare `ros2 …` command in
  this series assumes a sourced ROS 2 + workspace overlay and the project's DDS settings. On
  the robot (or a networked PC on the same graph), run the block below *before* any snippet —
  it is not repeated in each doc.

  ```bash
  # ROS 2 + the OpenAMRobot workspace overlays
  source /opt/ros/jazzy/setup.bash
  source ~/linorobot2_ws/install/setup.bash
  source ~/openamr-platform-sw/ros2/install/setup.bash
  # Project DDS settings — MUST match every node on the graph (robot uses CycloneDDS, domain 0)
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=0
  ```

  A mismatched `RMW_IMPLEMENTATION` or `ROS_DOMAIN_ID` is the classic "everything is `active`
  but topics are empty" trap — see [`../real_robot/`](../real_robot/README.md) for the full
  networking/DDS notes.
