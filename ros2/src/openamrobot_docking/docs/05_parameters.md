# Parameters Reference

This file summarises the parameters most commonly tuned in this
repository, both for the **real-robot config** and the **simulation**.
It is not exhaustive — see the YAML files for the full list.

## AprilTag detection

### Real robot — `config/tags_36h11.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `family` | `36h11` | Tag family |
| `size` | `0.0555` | **Tag edge size in metres — measure your printed tag** |
| `max_hamming` | `0` | Bit-error tolerance (0 = strict) |
| `detector.threads` | `1` | CPU threads for detection |
| `detector.decimate` | `2.0` | Image down-sampling factor (lower = more accurate, slower) |
| `detector.blur` | `0.0` | Gaussian blur σ before detection |
| `detector.refine` | `True` | Sub-pixel corner refinement |
| `tag.ids` | `[0]` | Tag IDs to detect |
| `tag.frames` | `[charging_dock_apriltag]` | TF frame names (must match `child_frame` in `docking_pose_publisher.yaml`) |

### Simulation — `simulation/config/tags_36h11_sim.yaml`

Same fields, with these differences:

| Parameter | Sim value | Why |
|---|---|---|
| `size` | `0.40` | Matches the panel size in `simulation/models/apriltag_dock/model.sdf` |
| `image_transport` | `raw` | Sim camera publishes raw, no compression |
| `decimate` | `1.0` | Camera is already low-resolution, no need to downsample |

## Detected dock pose publisher — `config/docking_pose_publisher.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `parent_frame` | `map` | Frame in which to publish the pose |
| `child_frame` | `charging_dock_apriltag` | Tag's TF frame (must match `tag.frames` in the AprilTag config) |
| `output_topic` | `detected_dock_pose` | PoseStamped output |
| `publish_rate` | `10.0` | Hz |

## Dock trigger — `config/dock_trigger.yaml`

This file is consumed by `scripts/dock_trigger.py`'s 4-phase sequencer.
All parameters affect the simulation's docking flow.

### Trigger / action params

| Parameter | Default | Meaning |
|---|---|---|
| `trigger_topic` | `dock_trigger` | Bool topic that triggers the sequence |
| `undock_on_false` | `false` | If true, `Bool false` triggers `UndockRobot` |
| `dock_type` | `openamrobot_dock` | Passed to `UndockRobot` action |

### Dock pose (must match `nav2_sim_full.yaml`'s `home_dock.pose`)

| Parameter | Default | Meaning |
|---|---|---|
| `dock_pose_x` | `4.0` | Dock x in **map** frame |
| `dock_pose_y` | `8.9` | Dock y in **map** frame |
| `dock_pose_yaw` | `1.5707` | Dock approach yaw (= robot heading when docked) |

> **Note.** `simulation.launch.py` overrides these three at launch time
> from the `spawn_x`, `spawn_y`, `spawn_yaw` arguments so the dock keeps
> pointing at the same physical tag regardless of where the robot
> spawns. The yaml defaults match the default spawn pose `(-4, -4, 0)`.

### Staging / docking distances

| Parameter | Default | Meaning |
|---|---|---|
| `staging_distance` | `2.5` | Distance (m) from the dock at which the robot first stops (phase 1) |
| `staging_hold_seconds` | `1.0` | Hold time at staging before starting tag scan |
| `docking_distance` | `0.9` | Final distance (m) from the running-average tag at which the advance phase stops. Bumped from 0.6 → 0.9 to keep the robot out of the near-field regime where tiny lateral motion produces large image-angle changes |

### Phase 2 — tag search and centring scan

| Parameter | Default | Meaning |
|---|---|---|
| `scan_rotation_speed` | `0.3` | Open-loop scan rotation rate (rad/s) and clamp on the centring P-loop |
| `scan_consecutive_target` | `5` | Centred frames required in a row before the scan exits |
| `scan_centring_tolerance` | `0.035` | Tag must be within this angle (rad ≈ 2°) of image centre to count as "centred" |
| `scan_centring_kp` | `1.0` | P-gain on the camera-frame image angle during centring |

### Phase 2 — initial filter

| Parameter | Default | Meaning |
|---|---|---|
| `detection_topic` | `/detected_dock_pose` | PoseStamped from `detected_dock_pose_publisher` |
| `detection_max_age` | `1.5` | Drop tag detections older than this (s) |
| `filter_num_samples` | `40` | Detections folded into the running-average tag pose during phase 2 |
| `filter_max_collect_time` | `6.0` | Max seconds spent collecting samples — 40 samples at 10 Hz needs ≥ 4 s with margin for drops |

### Phase 3 — spin to perpendicular

| Parameter | Default | Meaning |
|---|---|---|
| `spin_kp` | `1.5` | Angular P gain for the in-place spin |
| `spin_max_omega` | `0.3` | Clamp on spin angular velocity (rad/s) |
| `spin_yaw_tolerance` | `0.02` | Spin completion tolerance (~1.1°) |

### Phase 4 — line-tracking advance + straight-line final

| Parameter | Default | Meaning |
|---|---|---|
| `drive_speed` | `0.05` | Forward speed (m/s) during phase 4. Lowered to reduce slip with the omr_description casters |
| `line_yaw_kp` | `2.5` | Yaw P gain in the pure-pursuit phase: `omega = line_yaw_kp · (desired_yaw − robot_yaw)` |
| `line_lookahead_distance` | `0.3` | Pure-pursuit lookahead. `desired_yaw = perp_yaw − atan2(lateral, lookahead)`. Smaller = more aggressive lateral convergence |
| `drive_yaw_max_omega` | `0.3` | Clamp on omega during phase 4 |
| `visual_servo_distance` | `1.4` | At distance < this value, the controller does a one-shot final-align spin, then drives forward with `omega = 0` until `docking_distance`. **Must be > `docking_distance`** for the straight-line phase to engage |
| `drive_rate_hz` | `20.0` | Control loop rate for phases 2/3/4 |
| `cmd_vel_topic` | `/cmd_vel_nav` | Topic for direct cmd_vel (goes through smoother + collision monitor) |

The legacy parameter `drive_yaw_kp` is still declared for backwards
compatibility with older code paths but is **not used by the
line-tracking advance**.

### Removed parameters

These were used by earlier iterations (auto-calibration low-pass,
reverse-and-realign safety loop) and are no longer relevant:

`auto_cal_enabled`, `auto_cal_alpha`, `realign_enabled`,
`realign_lateral_threshold`, `realign_reverse_speed`,
`realign_reverse_distance`, `realign_reach_tolerance`,
`realign_min_distance`, `realign_max_retries`, `realign_omega`.

## Real-robot docking server — `config/openamrobot_docking.yaml`

### Top-level

| Parameter | Default | Meaning |
|---|---|---|
| `controller_frequency` | `50.0` | Control loop frequency (Hz) |
| `initial_perception_timeout` | `25.0` | Time to wait for tag detection before failing |
| `dock_approach_timeout` | `30.0` | Time to approach dock before failing |
| `max_retries` | `3` | Max docking attempts |
| `base_frame` | `base_footprint` | Robot base frame |
| `fixed_frame` | `odom` | Stable frame for short-term motion |

### `home_dock_plugin` (`opennav_docking::SimpleChargingDock`)

| Parameter | Default | Meaning |
|---|---|---|
| `docking_threshold` | `0.25` | Robot stops when within this distance of the (corrected) dock pose |
| `staging_x_offset` | `-0.7` | Staging position offset along dock_yaw (negative = behind dock) |
| `use_external_detection_pose` | `true` | Use `/detected_dock_pose` instead of static dock pose |
| `external_detection_translation_x` | `0.18` | Shift the dock pose in dock-frame X (toward robot) — see `12_lessons_learned.md` |
| `external_detection_translation_y` | `0.0` | Lateral shift |
| `external_detection_rotation_pitch` | `-1.5707` | Tag-to-dock frame rotation (pitch) |
| `external_detection_rotation_roll` | `-1.5707` | Tag-to-dock frame rotation (roll) |
| `external_detection_rotation_yaw` | `0.0` | Tag-to-dock frame rotation (yaw) |
| `filter_coef` | `0.5` | Low-pass filter on detected pose (higher = more responsive, noisier) |
| `use_stall_detection` | `false` | Stop if robot physically stalled |

### Controller (used by `opennav_docking`'s `controlled_approach`)

| Parameter | Default | Meaning |
|---|---|---|
| `dock_collision_threshold` | `0.3` | Stop distance for the controller's internal collision check |
| `use_collision_detection` | `false` | Enable controller collision check |
| `k_phi` | (varies) | Heading correction gain (graceful_controller) |
| `k_delta` | (varies) | Cross-track correction gain |
| `v_linear_min` / `v_linear_max` | (varies) | Speed limits for the approach |

### `home_dock` (the dock database entry)

| Parameter | Meaning |
|---|---|
| `frame: map` | Frame the pose is expressed in |
| `pose: [x, y, yaw]` | Map-frame coords. **Set this from your tag's measured map position.** |

## Simulation Nav2 + costmaps — `simulation/config/nav2_sim_full.yaml`

### Controller (RegulatedPurePursuitController)

| Parameter | Value | Meaning |
|---|---|---|
| `desired_linear_vel` | `0.55` | Target forward speed (m/s) during nav |
| `lookahead_dist` | `0.5` | Pure-pursuit carrot distance (m) |
| `rotate_to_heading_angular_vel` | `0.5` | Angular velocity during initial rotate-to-heading |
| `max_angular_accel` | `0.6` | Conservative — limits lateral inertial torque on the 2-wheel-balanced robot |
| `use_rotate_to_heading` | `true` | Rotate first if heading-to-path > `rotate_to_heading_min_angle` |
| `rotate_to_heading_min_angle` | `0.785` | 45° threshold |

### Planner (NavfnPlanner)

| Parameter | Value | Meaning |
|---|---|---|
| `tolerance` | `1.0` | Accept any free cell within this distance of goal if exact goal cell is blocked |
| `use_astar` | `true` | A* (more robust than Dijkstra here) |
| `allow_unknown` | `true` | Plan through unknown cells (combined with `track_unknown_space: false`) |

### Costmap inflation

| Parameter | Value | Meaning |
|---|---|---|
| `inflation_radius` | `0.45` | Inflated radius around obstacles (m) |
| `cost_scaling_factor` | `8.0` | Sharp falloff → less plan wobble; cells past ~25 cm from obstacle have ~zero cost |

### Velocity smoother

| Parameter | Value (linear, lateral, angular) | Notes |
|---|---|---|
| `max_accel` | `[1.2, 0.0, 1.5]` | |
| `max_decel` | `[-0.5, 0.0, -1.0]` | Softer linear decel (-0.5 instead of -1.2) since the omr_description full-mesh casters drag more — keeps the staging-zone approach from overshooting |
| `max_velocity` | `[0.7, 0.0, 1.2]` | |
| `min_velocity` | `[-0.7, 0.0, -1.2]` | |

### `home_dock` for the sim

```yaml
home_dock:
  frame: map
  pose: [4.0, 8.9, 1.5707]    # Must match dock_pose_* in dock_trigger.yaml
```

## Where the parameters interact

```
config/dock_trigger.yaml
  dock_pose_*  ───────────────  Must equal  ────────  nav2_sim_full.yaml home_dock.pose
                                                       (but simulation.launch.py overrides
                                                        from spawn pose automatically)
  detection_topic  ───────────  Must equal  ────────  docking_pose_publisher.yaml output_topic
  visual_servo_distance  ─────  Must be >   ────────  docking_distance (so the straight-line
                                                       phase actually engages before stop)

simulation/models/apriltag_dock/model.sdf
  box size  ──────────────────  Must equal  ────────  tags_36h11_sim.yaml size

omr_description/urdf/omr_robot.urdf.xacro
  camera_rgb_optical_frame  ───  Used by  ──────────  apriltag_ros (publishes pose in this frame)
                                 and by  ───────────  detected_dock_pose_publisher (looks up TF
                                                       map → charging_dock_apriltag)
                                 and by  ───────────  dock_trigger.py (camera-frame visual
                                                       centring during the scan)
```

When you change one of these, update the matching parameter in the
corresponding file.
