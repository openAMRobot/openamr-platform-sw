# Parameters reference

Every YAML knob that `openamrobot_docking` reads, grouped by config file. Defaults are the values shipped in this package and tuned for the simulation.

The YAML files this doc covers:

```
config/
├── dock_trigger.yaml              ← bundle sequencer (the bulk of this doc)
├── tags_36h11_sim.yaml            ← AprilTag detector (3-tag bundle, simulation)
└── docking_pose_publisher.yaml    ← TF → PoseStamped publisher (centre tag)
```

> **Note** — the package previously shipped a 4-phase single-tag pipeline; that legacy is preserved in [`08_legacy_sequencer.md`](08_legacy_sequencer.md). The values below describe the **current** bundle (3-tag) pipeline. The newer architectural framing lives in [`13_perception_and_line.md`](13_perception_and_line.md) and [`14_docking_research.md`](14_docking_research.md).

---

## `config/dock_trigger.yaml` — the bundle sequencer

Read by `scripts/dock_trigger.py`. Every parameter is also commented in-place in the YAML.

### Trigger plumbing

| Parameter | Default | Meaning |
|---|---|---|
| `trigger_topic` | `dock_trigger` | Bool topic that fires the docking sequence. `true` = dock. |
| `undock_on_false` | `false` | If `true`, `Bool false` triggers undocking. Off by default — undock has its own topic. |
| `undock_trigger_topic` | `undock_robot` | Bool topic that fires the undock sequence. |
| `goal_pose_topic` | `goal_pose` | Nav2 goal-pose intercept (undock-before-navigate). |
| `goal_pose_forward_topic` | `goal_pose_nav` | Where `dock_trigger.py` forwards the goal once it's safe (`bt_navigator` is remapped to this). |
| `undock_reverse_distance` | `1.5` | m — straight-line reverse before the 180° turn. |
| `undock_reverse_speed` | `0.10` | m/s — reverse speed magnitude. |

### Dock pose in the `map` frame

| Parameter | Default | Meaning |
|---|---|---|
| `dock_pose_x` | `4.899` | Dock x in the **map** frame (centre tag position). |
| `dock_pose_y` | `0.0` | Dock y in the **map** frame. |
| `dock_pose_yaw` | `0.0` | **Approach yaw** — the heading the robot has when docked (facing the bundle, perpendicular to the panel). |

> The simulation places the bundle centre tag at world `(4.899, 0, 0.5)` with the tag plane normals pointing `-x`. AMCL is initialised at map `(0, 0, 0)` = world `(0, 0, 0)`, so map ≡ world for this scenario. The robot approaches from `-x` heading `+x` → approach yaw = `0`.

### Phase 1 — Nav2 staging

| Parameter | Default | Meaning |
|---|---|---|
| `staging_distance` | `2.0` | Distance (m) in front of the dock at which Nav2 stops. Far enough that the camera sees all three tags of the bundle (90 cm baseline) cleanly. |
| `staging_hold_seconds` | `1.0` | Quiet time at staging before the tag scan begins (velocity decay + image stabilisation). |

### Phase 2 — bundle search + centring scan

| Parameter | Default | Meaning |
|---|---|---|
| `scan_rotation_speed` | `0.3` | Open-loop scan rotation rate (rad/s) and clamp on the centring P-loop. |
| `scan_consecutive_target` | `5` | Centred-frame count required in a row before the scan exits. |
| `scan_centring_tolerance` | `0.035` | Tag must be within this many rad (~2°) of image centre to count as centred. |
| `scan_centring_kp` | `1.0` | P-gain on the camera-frame image angle during the closed-loop centring on the bundle midpoint. |
| `detection_topic` | `/detected_dock_pose` | PoseStamped topic from `detected_dock_pose_publisher` (= the centre tag, id 1). |
| `detection_max_age` | `1.5` | Drop detections older than this (s) — staleness guard. |
| `filter_num_samples` | `40` | Number of fresh detections folded into the running-average tag pose during phase 2 (legacy filter, used by the seed). |
| `filter_max_collect_time` | `6.0` | Max seconds spent collecting the seed samples. |

### Phase 3 — spin to perpendicular yaw

| Parameter | Default | Meaning |
|---|---|---|
| `spin_kp` | `1.5` | Angular P gain for the in-place spin. |
| `spin_max_omega` | `0.5` | Clamp on the spin angular velocity (rad/s). Was `0.3` originally but the 180° undock spin then timed out — `0.5` keeps precision because the P-loop already slows the robot near the target. |
| `spin_yaw_tolerance` | `0.02` | Exit when `\|yaw_err\| < this` (~1.1°). |

### Phase 4 — line-tracking advance (pure-pursuit on the perpendicular line)

| Parameter | Default | Meaning |
|---|---|---|
| `docking_distance` | `0.15` | Final **camera→tag depth** (m) at which Phase 5 stops. The tag leaves the FOV before this, so the last stretch is a blind straight advance (odometry-measured). |
| `drive_speed` | `0.05` | Forward speed (m/s), linearly tapered inside `2 × docking_distance`. |
| `line_yaw_kp` | `2.5` | Yaw P gain: `omega = line_yaw_kp · (desired_yaw − robot_yaw)`. |
| `line_lookahead_distance` | `0.3` | Pure-pursuit lookahead. Smaller = more aggressive lateral convergence. |
| `drive_yaw_max_omega` | `0.3` | Clamp on omega during Phase 4 (rad/s). |
| `drive_rate_hz` | `20.0` | Control loop rate for phases 2/3/4/5. |
| `cmd_vel_topic` | `/cmd_vel` | Topic where dock_trigger publishes `Twist`. Direct to `/cmd_vel` because Raj's Nav2 doesn't run a `velocity_smoother` on `/cmd_vel_nav`. |

### Image-frame visual servo (used by Phase 5 NEAR regime)

Once the dock axis is frozen (Phase 5 NEAR), the heading controller closes the loop on the **image-frame angle** to the centre tag. Map-frame solvePnP is biased in the near field, but the image-frame angle is self-consistent — keeping the tag centred in the image = aiming straight at the dock.

| Parameter | Default | Meaning |
|---|---|---|
| `visual_servo_kp` | `0.6` | P gain on the image-frame angle: `omega = −visual_servo_kp · atan2(X_optical, Z_optical)`. |
| `visual_servo_filter_alpha` | `0.2` | Low-pass smoothing on the image-frame angle. `0.0 < alpha ≤ 1.0`. Lower = more smoothing. |

### Camera-centric approach (bundle path)

The current path estimates the dock surface **normal** from the outer tags' wide baseline (0/2, 90 cm apart), goes to a point on that normal, re-verifies, then runs a two-regime final approach (axis-averaged FAR → axis-frozen visual-servo NEAR on the centre tag). See [`13_perception_and_line.md`](13_perception_and_line.md) §6 and [`14_docking_research.md`](14_docking_research.md) §6.

| Parameter | Default | Meaning |
|---|---|---|
| `too_close_distance` | `1.0` | m — if the robot arrives closer than this, back off first to re-establish a clean far-field 3-tag view. |
| `predock_distance` | `2.0` | m on the normal (P1) — far enough to see all three tags. |
| `refined_predock_distance` | `1.50` | m on the normal (P2, if the re-verified normal N' disagrees with N). |
| `normal_tolerance_deg` | `5.0` | deg — agreement threshold between the first and second normal estimates. |
| `obs_lateral` | `0.5` | m — side offset for the second observation viewpoint. |
| `obs_distance` | `2.0` | m from the tag for the second observation. |
| `freeze_axis_distance` | `0.70` | m camera→centre tag. Phase 5 hand-over: FAR (> this) → average the 3-tag axis + pure-pursuit it; NEAR (≤ this) → freeze the axis, finish on the centre-tag visual corrector. Freezing kills the close-range zig-zag from noisy near estimates. |
| `axis_filter_alpha` | `0.40` | EMA weight for the live axis estimate (FAR regime). Weight grows as the robot gets closer (weight ∝ predock_distance/depth, capped) so nearer/more-accurate samples dominate the early far ones. Higher = more reactive/noisier. |

### Debug visualisation

| Parameter | Default | Meaning |
|---|---|---|
| `publish_debug_markers` | `true` | Publish the perpendicular line (green LINE_STRIP) and the running-average centre (red SPHERE) as an RViz `MarkerArray`. Add a `MarkerArray` display on `debug_marker_topic` to see it. |
| `debug_marker_topic` | `/docking/debug_markers` | The MarkerArray topic. |
| `publish_gz_marker` | `true` | Mirror the same line + centre **inside the Gazebo GUI**. Gazebo does not consume ROS markers, so this pushes a `gz.msgs.Marker_V` via the `gz` CLI (throttled, off-thread). Set to `false` if you only use RViz. |
| `gz_marker_service` | `/marker_array` | The gz marker service name (may differ on some Gazebo versions). |
| `gz_marker_period` | `0.4` | s — min interval between gz CLI calls. |

### Obstacle guard during drive phases

The sequencer publishes `cmd_vel` straight to the robot, bypassing Nav2's `collision_monitor`. A simple LIDAR-cone guard re-adds collision awareness inside the docking forward-drive phase (pure-pursuit onto the dock normal — `_goto_point_on_normal` / `_drive_to_xy`) and the undock reverse (`run_undock_sequence` / `_reverse_distance`).

| Parameter | Default | Meaning |
|---|---|---|
| `obstacle_check_enabled` | `true` | Master switch. |
| `obstacle_scan_topic` | `/scan_filtered` | LIDAR `LaserScan` topic. Default is the angle-filtered scan from Nav2's `scan_body_filter` chain — it chops the angular sector where the LIDAR sees the robot's enclosure, so close returns inside the kept sector are guaranteed to be real obstacles, not self-reflections. Falling back to raw `/scan` is risky: a distance-based floor (`obstacle_min_range` > 0) would mask real near obstacles. |
| `obstacle_forward_distance` | `0.6` | m — stop if an obstacle is within this distance ahead. |
| `obstacle_arc_half_width_deg` | `30.0` | deg — half-width of the detection cone (so a full 60° arc). |
| `obstacle_wait_timeout` | `10.0` | s — max wait for the path to clear before aborting the phase. |
| `obstacle_check_period` | `0.2` | s — poll period while waiting. |
| `obstacle_min_range` | `0.0` | m — **range floor**. Default disabled (`0.0`): self-reflections from the robot's body are removed UPSTREAM by Nav2's `scan_body_filter` (angle-based filter, the correct primitive), and the sequencer subscribes to `/scan_filtered`. A `>0` floor would silently mask real obstacles closer than the floor inside the kept angular sector — unsafe. Set `>0` ONLY as a fallback when `/scan_filtered` isn't available, and then pick a value clearly below the closest legitimate obstacle distance. |

Behaviour:

- **BEFORE** each guarded phase a "garde-fou" pre-check verifies the path is clear. If blocked, the robot stops and waits up to `obstacle_wait_timeout`; if still blocked, the phase aborts.
- **DURING** the phase the same check runs every control-loop iteration with the same wait-or-abort behaviour.
- **Phase 5** (IBVS final approach onto the dock) deliberately skips this — the dock itself is "an obstacle" we are approaching on purpose. The centring scan / spin-in-place phases also skip it (no translation).

### Tuning intuitions

- **Robot oscillates near the line** → increase `line_lookahead_distance` (smoother heading) or decrease `line_yaw_kp`.
- **Robot converges too slowly to the line** → decrease `line_lookahead_distance` or increase `line_yaw_kp` (watch `drive_yaw_max_omega` saturation).
- **Visual servo wobbles near the dock** → lower `visual_servo_kp` (e.g. 0.4) or lower `visual_servo_filter_alpha` (e.g. 0.1).
- **Robot zig-zags in the last 70 cm** → lower `axis_filter_alpha` (more smoothing) or raise `freeze_axis_distance` so the axis-frozen regime kicks in sooner.
- **Normal estimation disagrees too often (re-verify loop runs)** → check the bundle textures aren't blurry, raise `predock_distance` (cleaner samples), or relax `normal_tolerance_deg`.
- **Phase 2 timeouts** → either the bundle isn't in the camera, or the detector isn't getting synced `/camera_info_synced` (see [`09_troubleshooting.md`](09_troubleshooting.md)).
- **Robot stops during dock approach with "obstacle blocking"** → check `/scan` for spurious returns; widen `obstacle_arc_half_width_deg` if the cone is too narrow, or raise `obstacle_forward_distance` for a safer stand-off.

---

## `config/tags_36h11_sim.yaml` — AprilTag detector (simulation, 3-tag bundle)

| Parameter | Default | Meaning |
|---|---|---|
| `family` | `36h11` | Tag family. |
| `size` | `0.16` | **Side of the BLACK border square** in metres, not the whole texture. The 36h11 image is 10 modules wide with a 1-module white quiet zone each side, so the black square is 8/10 of the panel. All three tags ship on 0.20 m panels → 0.16 m black edge. |
| `max_hamming` | `0` | Bit-error tolerance (0 = strict). |
| `image_transport` | `raw` | Sim camera publishes raw uncompressed, no rectification needed. |
| `detector.threads` | `2` | CPU threads for detection. |
| `detector.decimate` | `1.0` | Image down-sampling factor. 1.0 = no decimation. |
| `detector.blur` | `0.0` | Pre-detection Gaussian blur σ. |
| `detector.refine` | `True` | Sub-pixel corner refinement. |
| `detector.sharpening` | `0.25` | Pre-detection sharpening. |
| `pose_estimation_method` | `pnp` | Use solvePnP for the tag → camera transform. |
| `tag.ids` | `[0, 1, 2]` | The three bundle IDs: outer-left, centre, outer-right. |
| `tag.frames` | `[charging_dock_tag_0, charging_dock_tag_1, charging_dock_tag_2]` | TF child frame names — must match the lookups in `dock_trigger.py` and the `child_frame` in `docking_pose_publisher.yaml` (which tracks the centre tag `_tag_1`). |

---

## `config/docking_pose_publisher.yaml` — TF → PoseStamped bridge

| Parameter | Default | Meaning |
|---|---|---|
| `parent_frame` | `map` | The frame in which the dock pose is published. |
| `child_frame` | `charging_dock_tag_1` | The **centre tag** of the 3-tag bundle — the docking target the robot drives onto. The outer tags (id 0, id 2) are consumed directly by `dock_trigger.py` to estimate the dock normal; they are not republished here. |
| `output_topic` | `detected_dock_pose` | PoseStamped output. Must match `detection_topic` in `dock_trigger.yaml`. |
| `publish_rate` | `10.0` | Hz. |

---

## Where these parameters interact

```
config/dock_trigger.yaml
  dock_pose_x/y/yaw  ──────  ground truth: the bundle centre tag pose in walled_world.sdf

  detection_topic  ─────────  Must equal  ────────  docking_pose_publisher.yaml output_topic

  scan_centring_tolerance  ─  Tighter than  ──────  Phase 4 lateral tolerance, else Phase 4
                                                     wobbles around the line

config/tags_36h11_sim.yaml
  size  ────────────────────  Must equal  ────────  models/apriltag_dock/model.sdf
                                                     (0.20 m panel × 0.8 = 0.16 m black edge)

  tag.frames[1] (centre)  ──  Must equal  ────────  docking_pose_publisher.yaml child_frame
                                                     ('charging_dock_tag_1')

  tag.frames[0], [2]  ──────  Must equal  ────────  dock_trigger.py outer-tag lookups
                                                     ('charging_dock_tag_0'/'_tag_2')

URDF (openamrobot_description)
  camera_optical_frame  ────  Used by  ──────────  apriltag_ros (per-tag TF parent)
                                 and by  ─────────  detected_dock_pose_publisher (centre tag lookup)
                                 and by  ─────────  dock_trigger.py (camera-frame centring + normal)

  base_link             ────  Used by  ──────────  dock_trigger.py (map → base_link lookup
                                                     for the robot's current pose)
```

When you change one value, update the matching consumer.

---

## Real-robot deployment

To port to a real robot:

1. Use `tags_36h11.yaml` instead of `tags_36h11_sim.yaml`. Differences:
   - `size` set to **your measured printed tag's black-square edge** (typically 8/10 of the printed panel side, e.g. a 0.20 m panel → 0.16 m).
   - `decimate` may need to be `2.0` or higher to keep the detector real-time on the robot's CPU.
   - `image_transport: compressed` (typical) — `apriltag.launch.yml` includes `image_proc` rectification.
2. Set `dock_pose_x/y/yaw` to the **measured map-frame pose of the centre tag** of your physical bundle.
3. The outer-tag spacing (0.45 m each side) is hard-wired by the dock geometry — if you build a wider/narrower bundle, the normal estimator still works (it uses the observed positions, not the configured spacing) but the FAR / NEAR thresholds may want re-tuning.
4. All other parameters are hardware-agnostic.
