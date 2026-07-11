# Real-robot tuning gotchas

Things that are invisible in simulation but decide whether the robot moves cleanly on the
ground: the measured **velocity floors**, the **sub-stiction yaw** limitation, the scan-filter
pitfalls (`obstacle_min_range: 0.0`, the 2D-lidar blind spot, the duplicate `/scan_filtered`
publisher, QoS), and the teleop conflict.

CPU / power / thermal / network problems on the Pi 5 are **not** here — they are in the
real-robot series ([`../real_robot/`](../real_robot/README.md)). This file is navigation
behaviour only.

---

## 1. Measured velocity floors

The real robot has a **minimum speed below which the wheels judder or stall** — a stick-slip +
coarse Hall-commutation effect at low RPM, *not* a torque shortfall (the motor is well-sized;
above the floors the commanded/actual ratio is ~1.0). Measured on the ground, under load,
closed-loop (`scripts/min_velocity_sweep.py`, 2026-07-02):

| Axis | Stalls | Judder | Reliable floor | Clean | Notes |
|---|---|---|---|---|---|
| **Linear** | ≤ 0.02 m/s | 0.03 m/s | **0.04 m/s** | **0.05 m/s** | 0.06–0.10 perfect |
| **Angular** | ≤ 0.08 rad/s | 0.10–0.12 rad/s | **0.15 rad/s** | 0.20+ | |

**Consequence for navigation:** keep commanded velocities **above** these floors. The DWB
limits already respect them for cruising (`max_vel_x: 0.20`, `max_vel_theta: 0.5`), but the
problem is at the **low end** — see the sub-stiction yaw issue next.

---

## 2. Sub-stiction yaw — the controller can command a turn too small to execute

**The problem.** DWB's angular config has **no lower floor** (`min_speed_theta: 0.0`), and its
`angular_granularity` is 0.025 rad. Near a goal, or when making a fine heading correction, DWB
can therefore emit a yaw command **well below the 0.15 rad/s angular floor** — on the order of a
few hundredths of a rad/s. Such a command is **below the robot's rotational stiction**: the
wheels don't break away, the robot doesn't turn, no progress is made, and the robot can appear
to stall while "trying" to satisfy a fine yaw goal.

**Where it is solved today.** The **docking** sequencer hit this first and added an explicit
floor: `min_turn_omega: 0.15` (snap any non-zero yaw correction up to the 0.15 rad/s floor so it
actually executes) plus `turn_deadband: 0.09` (zero out corrections small enough to ignore). See
[`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md)
and the velocity-floor note in [`../safety/02_limits_and_watchdog.md`](../safety/02_limits_and_watchdog.md).

**Where it is *not* solved.** The **Nav2 controller has no equivalent floor** — DWB/RotationShim
do not expose a "snap small omega up to a floor" parameter. In practice the coarse
`yaw_goal_tolerance: 0.35` usually keeps the controller from chasing a sub-stiction correction
(the goal is "reached" long before the yaw command gets that small), so it rarely bites during
normal navigation. It is a **known limitation** to be aware of when tightening the yaw tolerance:
do not lower `yaw_goal_tolerance` far without confirming the robot can actually execute the
resulting fine corrections.

---

## 3. Scan-filter pitfalls

The real robot's own chassis reflects the lidar. `scan_body_filter`
([`openamrobot_perception/scan_body_filter.py`](../../ros2/src/openamrobot_perception/openamrobot_perception/scan_body_filter.py))
blanks those self-returns and republishes `/scan` → `/scan_filtered`. Several traps live here.

### a. `obstacle_min_range: 0.0` — do not raise it

Because the body filter already removes the robot's own body, the costmaps set
`obstacle_min_range: 0.0` and see obstacles right up to the robot. A previous `0.35` **blinded
close obstacles** — they dropped out of the costmap as the robot approached and it drove into
them. Keep it 0.0. (Covered in [`02_costmaps.md`](02_costmaps.md); repeated here because it is
easy to "fix" wrongly.)

### b. QoS must match, or the costmap is empty

Nav2 Jazzy does **not** universally require RELIABLE scans (SensorData sources default to
BEST_EFFORT). What matters is **endpoint compatibility**: our costmap observation source is
configured **RELIABLE**, so a BEST_EFFORT publisher is **silently dropped** and the obstacle
layer stays empty (→ blind robot). `scan_body_filter` therefore publishes `/scan_filtered`
**RELIABLE** (`reliable_qos: true`) to match. Verify with:

```bash
ros2 topic info /scan_filtered --verbose      # publisher & subscriber reliability must be compatible
```

### c. Exactly one `/scan_filtered` publisher per profile

Scan filtering is a **data-source** responsibility, owned by exactly one place per profile:

| Profile | Owner of `/scan_filtered` |
|---|---|
| Real | `openamrobot_perception` (`scan_body_filter` node) via `real_bringup.launch.py` |
| Sim | `laser_filters` chain (`scan_body_filter.yaml`) started in `bringup.launch.py` |

`navigation_launch.py` **only consumes** `/scan_filtered` — it does **not** run a filter. This
was a real bug: the launch used to start its own `laser_filters` chain that conflicted with the
body filter → **two publishers** on `/scan_filtered` → corrupted obstacle data. If you ever see
two publishers again: `pkill -9 -f scan_to_scan_filter_chain`.

### d. The lidar is mounted rotated 180°

On this unit the RPLIDAR A1 is physically **rotated 180°** (static TF `base_link → lidar_link`,
`yaw = π`). That is why the real filter's angle conventions look inverted (0° = robot rear,
±180° = front). The masked sectors are calibrated for **this** mount — re-measure (watch `/scan`
in RViz) if the mount, chassis, or URDF changes.

### e. 2D-lidar vertical blind spot (no software fix)

The lidar sits at ~18 cm and sees only a horizontal slice. An obstacle **shorter than ~18 cm**
(a low box, a table foot, a cable, a threshold) is **invisible to the costmap** — neither
inflation nor an enlarged footprint can avoid something that was never marked. A head-on hit on a
low object is this, not a tuning problem. There is **no software fix**; it needs hardware (a
lower sensor, a down-tilted depth camera, or a physical bumper). Test avoidance with obstacles
**taller than ~20 cm**.

---

## 4. Teleop conflict on `/cmd_vel`

`/cmd_vel` is the *final* topic to the base (after the safety chain). A running
`teleop_twist_keyboard` with a `repeat_rate` **floods `/cmd_vel` with its last command (often 0)
at ~10 Hz**, which **overrides Nav2** — the robot won't move. **Kill any teleop before starting
navigation.** More generally, only one authority should own `/cmd_vel` at a time.

---

## 5. Process hygiene

Repeated relaunches leave duplicate agents / lidar drivers / EKFs fighting over serial/USB/TF →
everything becomes flaky. Always **clean-kill + single launch**. Note `ros2 run` adds a wrapper
process, so `pgrep` over-counts by one — confirm with `ps -ef`.

---

## Cross-links

- Costmaps and `obstacle_min_range` → [`02_costmaps.md`](02_costmaps.md)
- Controller limits (the cruising velocities) → [`03_planner_controller.md`](03_planner_controller.md)
- Velocity floors as a safety limit + battery rule → [`../safety/02_limits_and_watchdog.md`](../safety/02_limits_and_watchdog.md)
- CPU / power / thermal / network on the Pi 5 → [`../real_robot/`](../real_robot/README.md)
- Troubleshooting matrix → [`06_troubleshooting.md`](06_troubleshooting.md)
