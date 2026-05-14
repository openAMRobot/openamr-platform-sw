# openamrobot_nav2

Nav2 navigation stack bringup for **OpenAMRobot**: parameters, SLAM Toolbox configuration, RViz layout, and the launch that ties them together.

This package is **simulator-agnostic** and **docking-agnostic**: it provides the navigation primitives (path planning, controller, costmaps, localization or SLAM) and nothing else. Compose it with [`openamrobot_gazebo`](../openamrobot_gazebo/) (for simulation) or a real-robot driver launch (for hardware). [`openamrobot_docking`](../openamrobot_docking/) builds on top.

---

## Contents

```
openamrobot_nav2/
├── config/
│   ├── nav2_params.yaml            full Nav2 stack params (RPP, NavfnPlanner, costmaps, smoother, collision_monitor, ...)
│   └── slam_toolbox_params.yaml    SLAM Toolbox in mapping mode, 0.05 m grid, 10 m lidar range
├── launch/
│   └── nav2_bringup.launch.py      Nav2 + SLAM bringup (no Gazebo, no robot description)
├── rviz/
│   └── nav2_default.rviz           RobotModel + LaserScan + Map + GlobalCostmap + TF + Plan + LocalPlan
├── maps/                            user-saved maps (empty by default)
├── behavior_trees/                  custom BTs (empty by default)
├── package.xml
└── setup.py
```

---

## Usage

### Composed with the simulator (typical case)

```bash
ros2 launch openamrobot_bringup sim_nav.launch.py
```

The bringup composes Gazebo, this package, and RViz.

### On its own (e.g. once the real-robot driver is up)

```bash
ros2 launch openamrobot_nav2 nav2_bringup.launch.py
```

This package does not start the robot — it expects:

- `/scan` to be flowing (from Gazebo lidar plugin or real lidar driver),
- `/odom` and `/tf` to be flowing (DiffDrive plugin or wheel-odometry node),
- `robot_state_publisher` already running (so the `base_footprint -> base_link -> ...` TF chain is live).

If any of these are missing when Nav2 starts, the lifecycle nodes will refuse to activate and the bringup hangs in "Configuring..." state.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | drive ROS time from Gazebo's `/clock`. Set `false` on real hardware. |

The launch only declares one argument because all real tuning happens in the YAML files in `config/`. To swap parameter sets, edit the YAML or fork the launch to point at a different file.

---

## What `nav2_bringup.launch.py` does

1. **TimerAction(4 s)** → includes SLAM Toolbox's `online_async_launch.py` with `slam_params_file = slam_toolbox_params.yaml`. The 4-second delay is mandatory: SLAM Toolbox crashes if `/tf` (odom → base_footprint) is missing when it boots. Gazebo's DiffDrive plugin starts publishing TF after the entity is spawned (which the gazebo launch delays by 2 s); the extra 2 s slack ensures the chain is live.
2. **Immediately** includes upstream `nav2_bringup/bringup_launch.py` with:
   - `autostart=true` — lifecycle manager activates everything automatically
   - `use_localization=False` — we use SLAM Toolbox instead of AMCL
   - `map=''` — no static map, SLAM builds it online
   - `params_file=config/nav2_params.yaml` — our tuned stack
3. The upstream bringup starts: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `smoother_server`, `waypoint_follower`, `velocity_smoother`, `collision_monitor`, `docking_server`, plus the lifecycle manager.

> **About `docking_server`.** Nav2 Jazzy's `lifecycle_manager` hard-codes `docking_server` in its node list, so a valid `docking_server` config must exist or the bringup aborts. The full docking is done by `openamrobot_docking/dock_trigger.py` (4-phase pipeline, bypasses `docking_server`). The `docking_server` config in `nav2_params.yaml` exists only to keep the lifecycle happy.

---

## The `/cmd_vel` chain (this package's slice)

After `nav2_bringup` is up, three of its components produce velocity commands and feed each other:

```
controller_server  ─publish on→  /cmd_vel_nav         (raw Nav2 output)
behavior_server    ─publish on→  /cmd_vel_nav         (recovery behaviours: spin, backup, ...)
                                       │
                                       ▼
velocity_smoother  ─sub /cmd_vel_nav, pub→  /cmd_vel_smoothed
                                       │
                                       ▼
collision_monitor  ─sub /cmd_vel_smoothed, pub→  /cmd_vel
```

The two important re-wirings:

- **`navigation_launch.py` (inside nav2_bringup) remaps** `cmd_vel → cmd_vel_nav` for `controller_server`, `behavior_server`, and `velocity_smoother`. This is so the smoother and the collision_monitor sit between the raw command and the actuator.
- **`collision_monitor.cmd_vel_in_topic = cmd_vel_smoothed`** and **`cmd_vel_out_topic = cmd_vel`** in our YAML. Get these wrong and the chain breaks silently — there's no error, just no motion.

Downstream of `/cmd_vel`, the chain leaves this package: the bridge (in `openamrobot_gazebo`) sends `/cmd_vel` to Gazebo's DiffDrive in simulation, or the hardware driver consumes it on the real robot.

> **`dock_trigger.py` publishes directly on `/cmd_vel_nav`** during phases 2/3/4 of docking, so its commands pass through the same smoother + collision_monitor pipeline as Nav2's own output. This is intentional: we want phase-2 spinning to obey the same accel limits and collision checks as Nav2 navigation.

---

## Key parameter highlights — `nav2_params.yaml`

Section by section, the choices that aren't defaults:

### Controller — `RegulatedPurePursuitController` (RPP)

```yaml
desired_linear_vel: 0.55
rotate_to_heading_angular_vel: 0.5
max_angular_accel: 0.6
```

Conservative for a small indoor robot. RPP gives smooth path tracking; the chosen `desired_linear_vel` is fast enough for the 10×10 m world but not so fast that overshoot drops the controller into recovery.

### Planner — `NavfnPlanner`, A* mode

```yaml
use_astar: true
tolerance: 1.0
```

A* avoids the rare "Failed to plan from potential" Dijkstra bug. `tolerance: 1.0` lets the planner accept a free cell within 1 m of the goal if the exact goal cell is in a transient inflation zone — this prevents the dock approach from failing every time SLAM's costmap briefly inflates around the dock model.

### Costmaps

- **Global**: rolling 20×20 m, `obstacle_layer + inflation_layer`, `inflation_radius: 0.45 m`, `cost_scaling_factor: 8.0`.
- **Local**: rolling 3×3 m, `voxel_layer + inflation_layer`.

The `inflation_radius` matches the robot footprint diagonal plus a small margin (the robot is ~50 cm across).

### velocity_smoother

```yaml
max_velocity:   [0.7, 0.0, 1.2]   # linear-x, linear-y, angular-z
max_accel:      [1.2, 0.0, 1.5]
max_decel:      [-0.5, 0.0, -1.5] # softened decel
```

The softened linear deceleration (`-0.5` vs the upstream `-1.2`) avoids a visible overshoot at the end of phase 1 of docking: with the caster meshes (more drag than the simpler sphere-collision model upstream), a harder brake makes the chassis pitch forward briefly and the staging zone overshoots by ~10 cm.

### collision_monitor

```yaml
FootprintApproach:
  enabled: false
```

Disabled because the lidar occasionally glimpses the robot's own body during fast rotation and triggers a phantom near-obstacle clamp. The downstream `dock_trigger.py` publishes directly on `/cmd_vel_nav` so it still goes through the rest of the safety chain — we just don't want this specific polygon checking.

For real-robot deployment, **re-enable `FootprintApproach`** (and tune the polygon to the actual footprint) — the phantom-detection problem is sim-specific (the simulated lidar sees the URDF body more easily than a real lidar at correct mounting height).

---

## SLAM Toolbox — mapping mode

`config/slam_toolbox_params.yaml` runs SLAM Toolbox in **mapping** mode (`mode: mapping`). It subscribes to `/scan` and publishes `/map` + `map -> odom`. Highlights:

- `map_update_interval: 1.0` s — fast enough that the robot sees the map evolve during a long traversal
- `max_laser_range: 10.0` m — matches the lidar far limit minus 2 m safety
- `resolution: 0.05` m grid — standard for indoor Nav2
- `scan_topic: /scan`, `base_frame: base_footprint`, `odom_frame: odom`, `map_frame: map`

For localization in a pre-built map, switch to `mode: localization` and provide a starting pose via `/initialpose`. We don't use AMCL because SLAM Toolbox's localizer handles the same map type and is already in the bringup.

---

## RViz layout — `rviz/nav2_default.rviz`

Pre-configured displays:

- `RobotModel` on `/robot_description`
- `LaserScan` on `/scan`
- `Map` on `/map`
- `GlobalCostmap` and `LocalCostmap`
- `TF` (filtered to keep the tree readable)
- `Path` for the global plan, `Path` for the local plan
- `Goal Pose` tool wired to `/goal_pose` (Nav2's default)

View mode: `ThirdPersonFollower` of `base_footprint` so the camera follows the robot.

Open from the command line:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix openamrobot_nav2)/share/openamrobot_nav2/rviz/nav2_default.rviz
```

The `sim_nav` bringup opens this for you.

---

## What this package does NOT contain

| Concern | Lives in |
|---|---|
| Robot URDF, meshes, Gazebo plugins | `openamrobot_description` |
| Gazebo launch, ros↔gz bridge, worlds | `openamrobot_gazebo` |
| Docking: AprilTag, dock_trigger.py, dock world | `openamrobot_docking` |
| Top-level launch compositions | `openamrobot_bringup` |

---

## Adapting to real hardware

Most of `nav2_params.yaml` carries over to the real robot. Changes to expect:

- `use_sim_time: true` → `false` everywhere
- `collision_monitor.FootprintApproach.enabled: true` (re-enable, tune polygon)
- Costmap inflation radius and `cost_scaling_factor` may need re-tuning for the real environment
- `velocity_smoother.max_decel` may be tightened back if the real robot doesn't pitch
- The SLAM map can be built once with `slam_toolbox` then saved; subsequent runs switch to localization mode against the saved map (or AMCL)

---

## Status

Experimental. Parameters are tuned end-to-end in the docking simulation. Expect to revisit costmap radii, accel limits, and the planner tolerance once the real environment is known.
