# Architecture — node graph, lifecycle, data flow

This document maps the Nav2 node graph as launched by
[`localization_launch.py`](../../ros2/src/openamrobot_nav2/launch/localization_launch.py) and
[`navigation_launch.py`](../../ros2/src/openamrobot_nav2/launch/navigation_launch.py), the
two lifecycle managers that bring it up, and how TF / topics / velocity commands flow through
it. For the parameter *values*, see [`03_planner_controller.md`](03_planner_controller.md);
for costmaps see [`02_costmaps.md`](02_costmaps.md).

---

## 1. The two launch layers = two lifecycle managers

Nav2 is split into two launch files, each managed by its **own** `lifecycle_manager`. This is
deliberate: localization must come up (and publish `map → odom`) **before** the navigation
costmaps can initialize (see [`02_costmaps.md`](02_costmaps.md), the empty-costmap gotcha).

| Launch file | Lifecycle manager | Managed nodes (activation order) |
|---|---|---|
| `localization_launch.py` | `lifecycle_manager_localization` | `map_server`, `amcl` |
| `navigation_launch.py` | `lifecycle_manager_navigation` | `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `collision_monitor` |

Both managers run with `autostart:=true` (they configure→activate their nodes automatically).
The navigation manager sets `bond_timeout: 60.0` — a longer bond so a node that is slow to
activate on the loaded Pi 5 is not declared dead and torn down (a real cause of "the stack
half-starts then dies" on hardware).

The full graph:

```
                       ┌──────────────────────────── localization_launch.py ────────────────────┐
   map YAML ─────────► │  map_server ──(/map, latched)──►                                        │
                       │  amcl  ◄── /scan_filtered, /odom, TF odom→base_link                     │
                       │        └── publishes TF: map → odom                                     │
                       │  lifecycle_manager_localization  [map_server, amcl]                     │
                       └────────────────────────────────────────────────────────────────────────┘
                                             │ map→odom
                                             ▼
   ┌──────────────────────────────── navigation_launch.py ──────────────────────────────────────┐
   │                                                                                              │
   │   bt_navigator ◄── /goal_pose_nav (goal)   ── serves NavigateToPose / NavigateThroughPoses  │
   │      │  ticks the behaviour tree                                                             │
   │      ├──► planner_server (SmacPlanner2D)  ── uses global_costmap ──► global path             │
   │      ├──► controller_server (RotationShim→DWB) ── uses local_costmap ──► /cmd_vel_nav        │
   │      ├──► smoother_server (path smoothing, optional in BT)                                   │
   │      └──► behavior_server (spin, backup, drive_on_heading, wait, assisted_teleop) recoveries │
   │                                                                                              │
   │   /cmd_vel_nav ──► velocity_smoother ──► /cmd_vel_smoothed ──► collision_monitor ──► /cmd_vel│
   │                                                                                              │
   │   lifecycle_manager_navigation  [controller, planner, smoother, behavior, bt,               │
   │                                  waypoint_follower, velocity_smoother, collision_monitor]    │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
```

`map_server` ships with `yaml_filename: ""` in `nav2_params.yaml`; the actual map is injected
at launch (see §5).

---

## 2. Node responsibilities

| Node | Plugin(s) | Role |
|---|---|---|
| `map_server` | — | Serves the static occupancy grid on `/map` (latched / transient-local). |
| `amcl` | `DifferentialMotionModel`, `likelihood_field` | Particle-filter localization; publishes TF `map → odom`. Kidnap recovery enabled (`recovery_alpha_fast/slow`). |
| `planner_server` | `SmacPlanner2D` (`GridBased`) | Global A* path on the **global** costmap. |
| `controller_server` | `RotationShimController` → `DWBLocalPlanner` (`FollowPath`) | Local trajectory following on the **local** costmap; emits `/cmd_vel_nav`. |
| `smoother_server` | `SimpleSmoother` | Optional path smoothing invoked from the BT. |
| `behavior_server` | `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `AssistedTeleop` | Recovery behaviours the BT falls back to. |
| `bt_navigator` | `NavigateToPose`, `NavigateThroughPoses` | Runs the behaviour tree; the action entry point for goals. |
| `waypoint_follower` | `WaitAtWaypoint` | Multi-goal sequencing (pause at each waypoint). |
| `velocity_smoother` | — | Clamps accel/decel and velocity to the approved limits (open-loop). |
| `collision_monitor` | `FootprintApproach` polygon | Last-resort reactive guard: slows/stops before the footprint would collide. |

The controller is a **RotationShim wrapping DWB**: when the heading error to the path exceeds
`angular_dist_threshold` (0.785 rad ≈ 45°) it rotates in place toward the path *first*, then
hands off to DWB. DWB alone struggles to turn in place on a diff-drive base, which is why the
shim exists. Details and gains in [`03_planner_controller.md`](03_planner_controller.md).

---

## 3. TF tree

```
map                                   ← amcl (localization)
 └─ odom                              ← EKF (real) / gz bridge (sim)
     └─ base_link                     ← the control/kinematic frame
         ├─ base_footprint            ← static (firmware odom child_frame_id)
         ├─ lidar_link                ← static: x=0.335, z=0.18, yaw=π (mounted rotated 180°)
         ├─ imu_link                  ← static: identity (only gyro-Z used)
         └─ camera_link               ← static: x=0.415, z=0.12
             └─ camera_optical_frame  ← static: roll=−π/2, yaw=−π/2 (ROS optical convention)
```

On the **real robot** these static TFs are published by `real_bringup.launch.py` (measured
for this specific unit — note the lidar is physically mounted **rotated 180°**, which is why
the scan filter's angle conventions look "backwards"; see
[`04_real_robot_tuning.md`](04_real_robot_tuning.md)). In **simulation** the same tree comes
from `robot_state_publisher` + the URDF via the Gazebo bridge.

`base_frame_id: base_link`, `odom_frame_id: odom`, `global_frame_id: map` are set on AMCL and
the costmaps consistently. `transform_tolerance` is **1.0 s** on AMCL and **0.6 s** on the
costmaps — generous, to survive TF jitter under load on the Pi 5.

---

## 4. Velocity command chain (reactive-safety pipeline)

Every motion command passes through three stages before it reaches the wheels. The chain is
wired by remaps in `navigation_launch.py` and topic names in `nav2_params.yaml`:

```
controller_server
   │  (remap cmd_vel → cmd_vel_nav)
   ▼
/cmd_vel_nav ──► velocity_smoother  (input remapped to cmd_vel_nav; clamps accel/vel)
   │
   ▼
/cmd_vel_smoothed ──► collision_monitor  (cmd_vel_in_topic=cmd_vel_smoothed)
   │
   ▼
/cmd_vel ──► base (real: drivers/Teensy · sim: diff-drive plugin)
```

- The **velocity_smoother** limits are pinned to the DWB limits (`max_velocity: [0.20, 0.0,
  0.5]`, `max_accel: [0.5, 0.0, 0.5]`, `max_decel: [-2.5, 0.0, -2.0]`) so the smoother never
  fights the controller.
- The **collision_monitor** runs a `FootprintApproach` polygon with `action_type: "approach"`
  — it scales the command down to avoid a projected collision rather than hard-stopping.
  Full behaviour and the docking self-view caveat are in
  [`../safety/01_collision_monitor.md`](../safety/01_collision_monitor.md).

Because `/cmd_vel` is the *final* topic, any stray publisher on it (a lingering teleop) will
fight Nav2 — see the teleop gotcha in [`04_real_robot_tuning.md`](04_real_robot_tuning.md).

---

## 5. How the map gets in

`nav2_params.yaml` intentionally ships `map_server.yaml_filename: ""`. Two mechanisms fill it:

1. **`localization_launch.py`** accepts a `map:=` argument and, when non-empty, rewrites
   `yaml_filename` via `RewrittenYaml` into a separate `map_server` node instance.
2. **`bringup.launch.py`** takes a different route: it writes a *temporary copy* of
   `nav2_params.yaml` with `map_server.yaml_filename` already set, then launches localization
   with `map:=''` (the simple, no-condition path). This is deterministic across the include
   boundary — the stock `RewrittenYaml` + `map==''` condition mechanism does **not** propagate
   reliably through an `IncludeLaunchDescription`, which would leave `map_server` with an empty
   filename and stall the localization lifecycle. The same temp-file step also disables
   `amcl.set_initial_pose` on the real profile (the real robot is not at the map origin at
   startup).

---

## 6. Composition

`navigation_launch.py` and `localization_launch.py` both support a **composed** variant
(`use_composition:=True`) that loads the nodes into a single `nav2_container` via
`LoadComposableNodes` — lower IPC overhead, which matters on the Pi 5. The non-composed
(separate-process) variant is the default and is what the working real-robot recipe uses; the
composed path exists and mirrors the same remaps and parameters.

---

## Cross-links

- Costmaps, layers, footprint, empty-costmap gotcha → [`02_costmaps.md`](02_costmaps.md)
- Planner / controller values → [`03_planner_controller.md`](03_planner_controller.md)
- Goal routing into `bt_navigator` → [`05_goal_routing.md`](05_goal_routing.md)
- Safety chain (smoother, collision_monitor, watchdog) → [`../safety/`](../safety/README.md)
- System-wide package map → [`../architecture/ARCHITECTURE_OVERVIEW.md`](../architecture/ARCHITECTURE_OVERVIEW.md)
