# Navigation troubleshooting

Symptom → cause → fix, **navigation-specific only**. Network, power-brownout, CPU/thermal, and
full hardware bring-up problems on the Pi 5 live in the real-robot series
([`../real_robot/`](../real_robot/README.md)) — cross-linked where they overlap. Safety-layer
behaviour (collision monitor, watchdog, velocity clamp) is in
[`../safety/`](../safety/README.md).

Work the velocity chain **right-to-left** when the robot won't move:
`controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor →
/cmd_vel → base`.

---

## 1. Robot navigates but hits obstacles ("blind")

**Most likely: empty costmaps.** Verify the costmaps actually contain occupied cells:

```bash
ros2 topic echo /global_costmap/costmap --field data --once \
  | tr ',' '\n' | grep -vE '^0$|^-1$|^$' | wc -l   # >0 (map loaded)
ros2 topic echo /local_costmap/costmap  --field data --once \
  | tr ',' '\n' | grep -vE '^0$|^-1$|^$' | wc -l   # >0 (obstacles in view)
```

If **global == 0**: navigation was started before AMCL published `map → odom`, and/or the nodes
were hand-activated. **Fix:** set the 2D Pose Estimate first, then relaunch navigation and let
the lifecycle manager activate the nodes itself. Never hand-activate.
If **local == 0** but global > 0: the scan isn't reaching the obstacle layer — check QoS and the
duplicate-filter trap below. Full detail: [`02_costmaps.md`](02_costmaps.md) §4.

---

## 2. No obstacles in the local costmap (scan not delivered)

- **QoS mismatch:** `scan_body_filter` must publish `/scan_filtered` **RELIABLE** to match the
  RELIABLE costmap subscriber. `ros2 topic info /scan_filtered --verbose` — endpoints must be
  compatible; a BEST_EFFORT publisher is silently dropped.
- **Duplicate publisher:** `ros2 topic info /scan_filtered --verbose` → publisher count must be
  **1**. If 2, a stray `laser_filters` chain is running: `pkill -9 -f scan_to_scan_filter_chain`.
- **Low obstacle:** if the object is shorter than ~18 cm it is under the lidar plane and will
  never be marked — hardware limitation, not tuning ([`04_real_robot_tuning.md`](04_real_robot_tuning.md) §3e).

---

## 3. Goal sent, robot does nothing

Almost always **goal routing**. Check there is exactly one forwarder on `/goal_pose_nav`:

```bash
ros2 topic info /goal_pose_nav --verbose      # publisher count == 1
ros2 topic echo /goal_pose_nav                # your goal should appear on "2D Goal Pose"
```

- Nothing on `/goal_pose_nav` → no forwarder (start `goal_relay.launch.py` for nav-only, or bring
  up the docking layer for docking — **not both**).
- Using the "Nav2 Goal" tool → switch to **2D Goal Pose**.

Full detail: [`05_goal_routing.md`](05_goal_routing.md).

---

## 4. Goal accepted, but the robot still won't move

Not a routing problem — walk the chain:

| Check | Command | Meaning |
|---|---|---|
| Is the controller emitting? | `ros2 topic hz /cmd_vel_nav` | 0 Hz → controller stuck (no plan / progress-checker abort) |
| Does the smoother pass it? | `ros2 topic hz /cmd_vel_smoothed` | 0 while `/cmd_vel_nav` > 0 → smoother clamped it to zero |
| Does collision_monitor pass it? | `ros2 topic hz /cmd_vel` | 0 while smoothed > 0 → collision guard is stopping it ([`../safety/01_collision_monitor.md`](../safety/01_collision_monitor.md)) |
| Is a teleop fighting? | `ros2 topic info /cmd_vel --verbose` | >1 publisher → kill teleop ([`04_real_robot_tuning.md`](04_real_robot_tuning.md) §4) |
| **Is the command sub-stiction?** | echo `/cmd_vel` | tiny non-zero yaw (< ~0.15 rad/s) that doesn't turn the robot = sub-stiction yaw ([`04_real_robot_tuning.md`](04_real_robot_tuning.md) §2) |
| **Is the battery low?** | multimeter, `/debug/openloop` rpm test | **< 25 V at rest → recharge before debugging Nav2** ([`../safety/02_limits_and_watchdog.md`](../safety/02_limits_and_watchdog.md)) |

> "Robot won't move" was, on this robot, most often a **24 V power** problem, not Nav2. Prove the
> motors respond with a direct `/debug/openloop` rpm test **before** debugging the nav stack.

---

## 5. "Failed to make progress"

The controller isn't translating and the progress checker fired (now after
`movement_time_allowance: 12.0` s, so it surfaces sooner). Causes:

- **Infeasible path** for the big robot (inflation too low → planned path scrapes walls the
  footprint can't follow). Raise global inflation or reduce the footprint pad.
- **RotationShim/DWB stuck rotating** near an obstacle whose costmap cells block the pivot. Lower
  local inflation (0.15 → 0.10) or reduce the footprint so the pivot has room.
- Confirm `/cmd_vel` actually goes non-zero (a wheel that momentarily stalls — a loose motor
  cable, or start-up stiction — also looks like "no progress": reseat any loose cable, and a
  full stack restart clears a start-up stall).

---

## 6. Planner is slow / long "thinking" pause

- SmacPlanner2D warns it is slow with a **non-circular footprint + small inflation**. It is
  already mitigated by `downsample_costmap: true`, `downsampling_factor: 2`, and
  `max_planning_time: 1.0`. If still sluggish, switch to **NavFn** — but NavFn treats the robot
  as a point, so raise inflation to ≈ the inscribed radius (~0.29 m). See
  [`03_planner_controller.md`](03_planner_controller.md) §1.
- A general "everything is slow" (planner + TF timeouts + laggy costmaps) is usually **CPU
  saturation** on the Pi 5, not a Nav2 parameter — see [`../real_robot/`](../real_robot/README.md).

---

## 7. RViz shows "No map received" on a costmap display

Not an empty costmap — a **delivery** problem. Ensure `always_send_full_costmap: True` (set in
`nav2_params.yaml` for both costmaps) so the full grid is republished every cycle instead of a
single latched send that a late/WiFi subscriber misses. On the RViz side, set the costmap display
to **Durability: Transient Local, Reliability: Reliable**. Detail: [`02_costmaps.md`](02_costmaps.md) §1.

---

## 8. Localization is wrong / robot "teleports" on the map

- Set the **2D Pose Estimate** to seed AMCL — nothing localizes until you do.
- After a bump/slip, AMCL's kidnap recovery (`recovery_alpha_fast/slow`) should relocalize in
  1–2 s; if it never recovers, confirm those alphas are non-zero and the scan is reaching AMCL.
- Persistent TF extrapolation errors → check `use_sim_time` matches the profile (true in sim,
  false on the real robot) across **all** nodes.

---

## Health check (one-shot)

```bash
ros2 lifecycle get /amcl                 # active
ros2 lifecycle get /controller_server    # active
ros2 run tf2_ros tf2_echo map odom       # AMCL publishes map→odom (after 2D Pose Estimate)
ros2 topic info /scan_filtered --verbose # exactly 1 publisher, compatible QoS
ros2 topic hz /local_costmap/costmap     # updating
ros2 topic info /goal_pose_nav --verbose # exactly 1 forwarder
```

---

## Cross-links

- Costmaps → [`02_costmaps.md`](02_costmaps.md) · Controller/planner → [`03_planner_controller.md`](03_planner_controller.md)
- Real-robot behaviour traps → [`04_real_robot_tuning.md`](04_real_robot_tuning.md) · Goal routing → [`05_goal_routing.md`](05_goal_routing.md)
- Safety layer → [`../safety/`](../safety/README.md) · Pi 5 CPU/power/thermal/network → [`../real_robot/`](../real_robot/README.md)
