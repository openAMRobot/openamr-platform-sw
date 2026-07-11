# Costmaps — layers, footprint, and the empty-costmap gotcha

The two costmaps are where "the robot can see obstacles" actually lives. This document covers
the global vs local split, the layers on each, the footprint (and the enlarged-footprint
hard-clearance mechanism), and the single most expensive real-robot trap we hit: **empty
costmaps → the robot drives blind**.

All values are quoted from
[`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml).

---

## 1. Global vs local — two costmaps, tuned differently

This is the key idea (and it is native Nav2): the global and local costmaps serve different
jobs, so they are inflated **differently**.

| | `global_costmap` | `local_costmap` |
|---|---|---|
| Frame | `map` | `odom` |
| Extent | whole map (static) | `3 × 3 m` rolling window (`rolling_window: true`) |
| Update / publish | 5 Hz / 2 Hz | 10 Hz / 5 Hz |
| Resolution | 0.05 m | 0.05 m |
| Layers | `static_layer`, `obstacle_layer`, `inflation_layer` | `voxel_layer`, `inflation_layer` |
| Used by | `planner_server` (global path) | `controller_server` (DWB trajectory scoring) + `collision_monitor` |
| Inflation radius | **0.35 m** | **0.15 m** |
| `cost_scaling_factor` | **3.5** | 3.0 |

**Why the asymmetry:**

- The **global** costmap drives the *plan*. SmacPlanner2D treats the robot as a point, so a
  larger inflation (0.35 m) pads the path away from walls — the global plan keeps its distance.
  `cost_scaling_factor` was raised 2.5 → **3.5** so the cost falls off faster past the radius:
  a thinner hard halo means the robot is less likely to "set off in near-collision" and
  trigger a recovery.
- The **local** costmap drives *avoidance*. A small inflation (0.15 m) keeps it crisp, and the
  actual "don't touch" guarantee comes from DWB checking the **footprint** against occupied
  cells every cycle (the `ObstacleFootprint` critic — see
  [`03_planner_controller.md`](03_planner_controller.md)), not from inflation.

> **Concept — inflation ≠ avoidance.** Inflation is a *soft planning cost*; the robot may
> traverse it. "Never touch" comes from the controller testing the real footprint against the
> local costmap in real time, and from the collision_monitor as a last resort.

Both costmaps set `always_send_full_costmap: True`. This republishes the full grid every cycle
instead of a single latched send + deltas, which fixes the RViz **"No map received"** symptom
for a late-joining viewer over WiFi (the transient-local sample is not reliably delivered to a
late subscriber). Network cost is negligible at 2–5 Hz.

---

## 2. The obstacle sources

Both costmaps ingest the **same** filtered scan. Observation source `scan` on `/scan_filtered`:

```yaml
scan:
  topic: /scan_filtered
  data_type: "LaserScan"
  clearing: True
  marking: True
  raytrace_max_range: 3.0
  raytrace_min_range: 0.10
  obstacle_max_range: 2.5
  obstacle_min_range: 0.0     # ← must stay 0.0, see below
  max_obstacle_height: 2.0
```

- **`obstacle_min_range: 0.0`** is deliberate and load-bearing. A previous value of `0.35`
  *blinded close obstacles*: returns within 0.35 m of the lidar were dropped, so an obstacle
  **vanished from the costmap as the robot approached it → the robot drove into it**. It is
  safe to keep 0.0 because `scan_body_filter` already removes the robot's own body from the
  scan (see [`04_real_robot_tuning.md`](04_real_robot_tuning.md)). **Do not re-introduce a min
  range.**
- The global costmap uses a plain `ObstacleLayer`; the local costmap uses a `VoxelLayer` (3D
  marking, `z_voxels: 16`, `max_obstacle_height: 2.0`) — same 2D scan source, but the voxel
  layer gives cleaner clearing of transient returns in the rolling window.
- AMCL uses `laser_max_range: 12.0` (the RPLIDAR A1's real reach; 100 m would inject phantom
  beams) while the costmaps mark only to 2.5 m — appropriate for a 3 × 3 m local window.

---

## 3. Footprint

`base_link` (the wheel axle / rotation centre) is **not** the geometric centre of this robot.
The chassis is **0.78 × 0.58 m** with the front 0.415 m ahead of `base_link` and the rear
0.365 m behind it. Using a circle (`robot_radius`) was wrong — a 0.22 m circle let the
overhanging front clip obstacles. The committed footprint is the **true octagonal hull**, set
identically on both costmaps:

```yaml
footprint: "[[0.415, 0.19], [0.415, -0.19], [0.315, -0.29], [-0.265, -0.29],
             [-0.365, -0.19], [-0.365, 0.19], [-0.265, 0.29], [0.315, 0.29]]"
```

### Enlarged footprint (+0.12 m) — the hard-clearance mechanism

To enforce a **hard** "never get within ~12 cm of any obstacle the lidar sees" rule, the
footprint can be **padded by +0.12 m** on both costmaps. Because DWB tests the footprint
against the local costmap every cycle, a padded footprint is a *hard* rule (far stronger than
inflation, which is only a soft planning cost). The padded hull is:

```yaml
# +0.12 m padded hull (hard ~12 cm clearance) — set on BOTH costmaps
footprint: "[[0.535, 0.31], [0.535, -0.31], [0.435, -0.41], [-0.385, -0.41],
             [-0.485, -0.31], [-0.485, 0.31], [-0.385, 0.41], [0.435, 0.41]]"
```

> **Accuracy note.** The file currently committed in `nav2_params.yaml` ships the **true
> hull** (the first array above), not the padded one. The +0.12 m padded footprint is the
> documented, validated mechanism for the operator's "keep 10–20 cm clearance" requirement and
> is swapped in when that margin is wanted. Trade-off: a padded footprint makes the robot
> "bigger", so in a **very tight space** Nav2 may fail to find a path — reduce the pad (e.g.
> +0.10) or clear the space. Both footprint arrays are runtime-settable
> (`ros2 param set /local_costmap/local_costmap footprint "…"`, same for global).

The footprint dimensions must be re-confirmed with a tape measure if the axle-to-front /
axle-to-rear offsets differ on another unit — an offset axle means an offset footprint that
touches on one side.

---

## 4. The empty-costmap gotcha (the expensive one)

**Symptom:** the robot navigates but **hits everything** — it behaves as if the costmaps
contain no obstacles at all.

**Root cause:** the navigation costmaps need TF `map → base_link` to initialize. On the real
robot that requires AMCL to be publishing `map → odom`, which only happens **after a 2D Pose
Estimate**. If `navigation_launch.py` is started *before* `map → odom` exists, the costmaps'
lifecycle **aborts** ("transform from base_link to map did not become available"). If you then
**hand-activate** the nodes (`ros2 lifecycle set … activate`), they come up **mis-initialized**:
the static layer never loads the map and the obstacle layer never subscribes to the scan → both
costmaps report **0 occupied cells** → nothing to avoid → the robot drives straight into
obstacles.

**Fix / procedure:**

1. Bring up localization first, then set the **2D Pose Estimate** in RViz (or wait for AMCL to
   publish `map → odom`).
2. **Only then** start navigation, and let `lifecycle_manager_navigation` activate the nodes
   **by itself**. Never hand-activate lifecycle nodes — if they aborted, **relaunch**.

**Verify the costmaps actually contain obstacles** (not just that the nodes are `active`):

```bash
ros2 topic echo /global_costmap/costmap --field data --once \
  | tr ',' '\n' | grep -vE '^0$|^-1$|^$' | wc -l   # >0  (map loaded)
ros2 topic echo /local_costmap/costmap  --field data --once \
  | tr ',' '\n' | grep -vE '^0$|^-1$|^$' | wc -l   # >0  (obstacles in view)
```

Once healthy we measured ~36041 non-empty global cells / ~3602 local. A count of 0 means the
robot is blind — do not drive it.

> A **second**, unrelated cause of "sees the obstacle but hits it" is a QoS mismatch on
> `/scan_filtered` (a BEST_EFFORT publisher against the RELIABLE costmap subscriber → silently
> dropped → empty obstacle layer). That is fixed at the source (`scan_body_filter` publishes
> RELIABLE); see [`04_real_robot_tuning.md`](04_real_robot_tuning.md).

---

## Cross-links

- Footprint checking by DWB (`ObstacleFootprint` critic) → [`03_planner_controller.md`](03_planner_controller.md)
- The scan filter and its QoS → [`04_real_robot_tuning.md`](04_real_robot_tuning.md)
- Collision monitor (last-resort reactive guard) → [`../safety/01_collision_monitor.md`](../safety/01_collision_monitor.md)
- Troubleshooting matrix → [`06_troubleshooting.md`](06_troubleshooting.md)
