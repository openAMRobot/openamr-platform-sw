# Planner & controller tuning

The reasoning behind the tuned planner and controller values in
[`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml). This is a
**heavy, slow diff-drive robot on a Raspberry Pi 5** — most of the tuning is about (a) not
overshooting with a heavy base, and (b) not stalling the CPU-bound planner. Every number below
is quoted from the file.

For the real-robot velocity floors and the sub-stiction limitation, read this together with
[`04_real_robot_tuning.md`](04_real_robot_tuning.md).

---

## 1. Planner — `SmacPlanner2D`

```yaml
planner_server:
  expected_planner_frequency: 20.0
  GridBased:
    plugin: "nav2_smac_planner::SmacPlanner2D"
    tolerance: 0.5
    downsample_costmap: true
    downsampling_factor: 2          # plan on a 0.10 m grid (costmap is 0.05 m)
    use_astar: true
    max_iterations: 1000000
    max_on_approach_iterations: 1000
    max_planning_time: 1.0          # hard cap
    cost_travel_multiplier: 2.0
    use_final_approach_orientation: false
```

**Why these values:**

- **`downsample_costmap: true` + `downsampling_factor: 2`** — the single biggest planner win.
  SmacPlanner2D at the full 0.05 m resolution was the main cause of the "takes too long to
  think" pause. Planning on a 2×-downsampled 0.10 m grid means **~4× fewer A\* nodes**, so the
  global plan computes far faster. The path is slightly coarser but adequate; the 0.05 m local
  costmap still handles fine obstacle avoidance.
- **`max_planning_time: 1.0`** — a hard cap so a difficult query cannot stall the robot for
  seconds. Combined with the downsampling, planning stays responsive on the Pi 5.
- **`use_astar: true`** — A\* (vs Dijkstra) for a directed, faster search on the downsampled grid.
- SmacPlanner2D is used despite a known caveat: it warns it can be slow with a **non-circular
  footprint + small inflation**. If planning is ever sluggish, `NavFn`
  (`nav2_navfn_planner::NavfnPlanner`) is the faster fallback — but NavFn treats the robot as a
  **point**, so it needs inflation ≈ the inscribed radius (~0.29 m) to plan feasible paths for
  this large robot. This is a documented escape hatch, not the current config.

---

## 2. Controller — `RotationShimController` → `DWB`

The controller is a **RotationShim wrapping DWB**. DWB alone struggles to turn a diff-drive
base in place to set off in a new direction; the shim rotates toward the path first, then hands
control to DWB.

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "dwb_core::DWBLocalPlanner"
  angular_dist_threshold: 0.785        # heading error > 45° → turn in place first
  rotate_to_heading_angular_vel: 0.5   # ≤ max_vel_theta
  max_angular_accel: 0.5               # gentle in-place pivot
  rotate_to_goal_heading: true
```

### DWB kinematic limits (heavy-robot tuning)

```yaml
  max_vel_x: 0.20        max_speed_xy: 0.20     min_vel_x: -0.05
  max_vel_theta: 0.5
  acc_lim_x: 0.5         acc_lim_theta: 0.5
  decel_lim_x: -2.5      decel_lim_theta: -2.0
```

**Why:**

- **`max_vel_x: 0.20`** (raised 0.12 → 0.20) with **`max_speed_xy: 0.20`** aligned to it — if
  `max_speed_xy` is left at the default 0.5 the robot "runs away" past `max_vel_x`.
- **`max_vel_theta: 0.5`** — 2.0 was far too fast for this base and caused overshoot in turns.
- **Asymmetric braking `decel_lim_x: -2.5` / `decel_lim_theta: -2.0`** (much stronger than the
  `acc_lim_*: 0.5` ramps). The robot is heavy and carries momentum; strong deceleration lets it
  slow *early* and stop on target instead of overshooting. Gentle acceleration (0.5) keeps
  starts smooth; hard braking (−2.5) absorbs the inertia.
- **`min_vel_x: -0.05`** — a small reverse, only used to unstick; the collision_monitor guards
  the rear (~0.365 m from `base_link`).

### DWB critics

```yaml
  critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "ObstacleFootprint",
            "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
  BaseObstacle.scale: 5.0
  ObstacleFootprint.scale: 10.0
  PathAlign.scale: 16.0     PathAlign.forward_point_distance: 0.30
  GoalAlign.scale: 12.0     GoalAlign.forward_point_distance: 0.30
  PathDist.scale: 32.0      GoalDist.scale: 24.0
  RotateToGoal.scale: 32.0  RotateToGoal.slowing_factor: 5.0
  vx_samples: 20  vy_samples: 1  vtheta_samples: 30  sim_time: 1.7
```

**Why:**

- **`ObstacleFootprint` is the critical addition.** The default `BaseObstacle` critic tests only
  the robot **centre** — inadequate for a robot whose front overhangs `base_link` by ~0.42 m (0.415): the
  front would hit before the centre came near. `ObstacleFootprint` (scale 10.0) scores the
  **whole footprint** against the local costmap and rejects any trajectory where the shape would
  touch. This, plus the enlarged footprint option in [`02_costmaps.md`](02_costmaps.md), is what
  actually keeps the robot off obstacles. *(Critics load only at controller (re)start — a plugin
  change requires relaunching `navigation_launch.py`, not a live `param set`.)*
- **`PathAlign` lowered 32 → 16** and **`forward_point_distance` raised 0.1 → 0.30** — the heavy
  robot was "snaking" (over-aggressive heading-to-path pull); a weaker pull looking further ahead
  is smoother. `GoalAlign` softened the same way near the goal.
- **`vy_samples: 1`** — differential base, `max_vel_y = 0`, so there is nothing to sample
  laterally. **`vtheta_samples: 30`** (lowered 40 → 30) trims a little angular hesitation.
- **`trans_stopped_velocity: 0.02`** — the measured odometry noise floor (the default 0.25 was
  *above* `max_vel_x`, so the robot was considered "moving" when stopped).

---

## 3. Progress checker & goal checker

```yaml
progress_checker:
  plugin: "nav2_controller::SimpleProgressChecker"
  required_movement_radius: 0.05
  movement_time_allowance: 12.0        # tightened from 30.0

general_goal_checker:
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.35
  yaw_goal_tolerance: 0.35
```

- **`movement_time_allowance: 12.0`** (from 30.0) — declare "no progress" and run recoveries
  **sooner** instead of sitting stuck for 30 s. A large part of the perceived "thinking" time
  was this timeout, especially if a wheel momentarily stalls (a motor cable working loose, or
  start-up stiction — reseat the cable, or restart the stack; not tied to a specific wheel).
- **`xy_goal_tolerance` / `yaw_goal_tolerance: 0.35`** — these are **coarse navigation**
  tolerances. Docking precision is a **separate**, much tighter loop
  (`docking_server.docking_threshold: 0.05`, plus tolerances in `dock_trigger.yaml`) and does
  **not** use this nav tolerance, so a loose nav goal does not compromise the dock approach. See
  [`../../ros2/src/openamrobot_docking/docs/05_parameters.md`](../../ros2/src/openamrobot_docking/docs/05_parameters.md).

---

## 4. AMCL (localization) tuning notes

```yaml
amcl:
  laser_max_range: 12.0        # RPLIDAR A1's real reach (not 100 → phantom beams)
  laser_min_range: -1.0
  max_beams: 60   min_particles: 500   max_particles: 2000
  recovery_alpha_fast: 0.1     recovery_alpha_slow: 0.001
  robot_model_type: "nav2_amcl::DifferentialMotionModel"
  scan_topic: /scan_filtered
  transform_tolerance: 1.0
```

- **`recovery_alpha_fast/slow`** enable **kidnap recovery**: when the short-term average
  particle weight drops well below the long-term average (teleport, wheel slip, bump), AMCL
  injects uniform particles and relocalizes in 1–2 s. Previously both were 0.0 (recovery
  disabled) and the robot silently kept believing a stale pose.
- **`transform_tolerance: 1.0 s`** is generous on purpose — TF timing jitters under CPU load on
  the Pi 5.

---

## 5. Live-tuning cheat-sheet

Most of these are settable at runtime (no relaunch) — useful for tuning on the robot:

```bash
# DWB velocity limits do NOT apply via param set at runtime (the getter lies) — edit nav2_params.yaml + relaunch:
# ros2 param set /controller_server FollowPath.max_vel_x 0.18
# ros2 param set /controller_server FollowPath.max_vel_theta 0.4
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.15
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

**Exceptions that require a relaunch of `navigation_launch.py`:** the controller **plugin**
(RotationShim/DWB) and its **critic list** load only at (re)start; likewise the DWB velocity/accel limits (`max_vel_x`, `max_speed_xy`, `max_vel_theta`, `sim_time`) do **not** apply via `param set` — the getter lies, so edit the yaml and relaunch. Cursors: too fast in a
straight line → lower `max_vel_x`; overshoots in turns → lower `max_vel_theta` / strengthen
`decel_lim_theta`; snakes → raise `PathAlign`/`PathDist`, lower `vtheta_samples`; planner slow →
NavFn (§1).

---

## Cross-links

- Costmaps, footprint, `ObstacleFootprint` → [`02_costmaps.md`](02_costmaps.md)
- Velocity floors and the sub-stiction yaw limitation → [`04_real_robot_tuning.md`](04_real_robot_tuning.md)
- Velocity smoother / collision monitor → [`../safety/`](../safety/README.md)
