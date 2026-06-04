# 4-phase docking sequencer (legacy — superseded)

> ⚠️ **This document describes the original single-tag 4-phase pipeline.** It is **kept for historical context** but is no longer the production design.
>
> The current docking stack uses a **3-tag AprilTag bundle** (IDs 0/1/2, outer tags at `y = ±0.45 m`) and a **camera-centric multi-stage sequencer** that estimates the dock surface normal from the outer tags' wide baseline rather than from a single tag's yaw. See:
> - [`13_perception_and_line.md`](13_perception_and_line.md) — how the bundle pose produces the perpendicular line, with the proximity-weighted EMA normal estimate
> - [`14_docking_research.md`](14_docking_research.md) §6 — the current architecture, stage-by-stage with numerical derivations
> - [`00_overview.md`](00_overview.md) — short summary of the bundle pipeline
>
> Some terminology in this document (`TagRunningAverage`, "the 4-phase pipeline", `charging_dock_apriltag` single TF) refers to the earlier design and does not match the live code. The principles below (centring scan, perpendicular-line tracking, visual-servo handover) carry over to the bundle pipeline; the implementation around them has been replaced.

---

Phase-by-phase walkthrough of the (legacy) docking pipeline implemented in earlier revisions of [`scripts/dock_trigger.py`](../scripts/dock_trigger.py).

For the YAML knobs that tune each phase, see [`05_parameters.md`](05_parameters.md).

---

## Why a custom sequencer

Nav2's `opennav_docking::SimpleChargingDock::controlled_approach` (the upstream solution) follows a **curved** path that does not always arrive head-on at the dock. For an AprilTag dock that requires perpendicular contact for the charger contacts to meet, that curve is fragile — lateral offset accumulates along it.

The 4-phase sequencer keeps Nav2 (planner, controller, costmaps) for the long-range navigation and replaces the **final approach** with an explicit closed-loop controller that drives the robot directly onto the dock's perpendicular axis.

---

## High-level flow

```
Phase 1 ─ NavigateToPose → staging zone   (Nav2 NavigateToPose action)
   │
Phase 2 ─ scan + initial filter           (cmd_vel on /cmd_vel)
   │  • open-loop slow rotation until tag detected
   │  • closed-loop yaw P-control to centre the tag in the image
   │  • collect filter_num_samples (40) into a TagRunningAverage
   │
Phase 3 ─ ALIGN — in-place spin           (cmd_vel on /cmd_vel)
   │  • spin to the running-average perpendicular_yaw
   │  • tolerance ≈ 1.1°
   │
Phase 4 ─ line-tracking advance           (cmd_vel on /cmd_vel)
   │  • pure-pursuit on the perpendicular line through the
   │    running-average tag centre
   │  • running average keeps refining the line on every fresh detection
   │  • if the tag is lost (out of FOV in near field), the line is
   │    frozen and the robot continues along it open-loop
   │  • stop at distance ≤ docking_distance (0.9 m)
```

---

## Phase 1 — Nav2 to staging

Sends a `NavigateToPose` goal to the **staging pose**:

```
staging_x = dock_pose_x − staging_distance · cos(dock_pose_yaw)
staging_y = dock_pose_y − staging_distance · sin(dock_pose_yaw)
staging_yaw = dock_pose_yaw
```

With Raj's setup (dock at map `(4.899, 0)`, approach yaw `0`, `staging_distance = 1.5`), the robot navigates to map `(3.40, 0)` facing `+x`.

Nav2 plans around walls, the RegulatedPurePursuitController tracks the path, and the goal checker accepts arrival when `\|xy_err\| < xy_goal_tolerance` and `\|yaw_err\| < yaw_goal_tolerance`.

---

## Phase 2 — scan and initial filter

After Nav2 finishes, the robot may have a small yaw offset versus the ideal staging pose. Phase 2 fixes this and seeds the tag-pose running average.

### 2a / 2b — centring scan

Two regimes:

- **Open-loop** rotation at `scan_rotation_speed` (0.3 rad/s) until the tag is in the camera frustum.
- **Closed-loop** yaw P-control once a detection is available:

```python
omega = −scan_centring_kp · atan2(X_cam, Z_cam)        # rad/s
omega = clamp(omega, ±scan_rotation_speed)
```

`(X_cam, Y_cam, Z_cam)` is the tag's translation in `camera_optical_frame` (looked up via TF — **not** via the noisier map-frame `/detected_dock_pose`). In the optical frame, `+X` is image-right and `+Z` is forward, so `atan2(X, Z)` is the horizontal angle from the image centre to the tag centre.

The scan exits when `\|atan2(X, Z)\| < scan_centring_tolerance` (~2°) for `scan_consecutive_target` (5) frames in a row.

> **Why camera-frame and not map-frame.** Single-shot solvePnP carries a systematic bias on the *map-frame* tag pose (typically a few tens of centimetres), but the camera-frame angle `atan2(X, Z)` is self-consistent with whatever the apriltag node detects. Centring on it actually centres the tag in the image regardless of map-frame bias.

### 2c — initial filter (seed the running average)

Once centred, the robot is stationary and starts folding fresh detections into a `TagRunningAverage`:

- **Position**: true incremental running mean (each sample averages with `count` so far).
- **Quaternion**: sign-aligned to the existing mean before componentwise averaging, then renormalised. This is the "naive quaternion mean" and is valid here because all samples lie close to each other on the sphere (the tag is static and the camera is stationary).

Phase 2 ends after `filter_num_samples` (40) detections.

---

## Phase 3 — ALIGN

Spin in place to `perpendicular_yaw` — the yaw the robot needs to be perpendicular to the tag plane. Computed from the running-average tag quaternion:

```python
n = quat_rotate_z(avg.quat)            # tag's +Z axis (its normal) in map frame
n = n / ||n||
if n · (robot − tag) < 0: n = −n       # disambiguate so n points toward the robot
perpendicular_yaw = atan2(−n.y, −n.x)  # yaw to face the tag
```

The spin uses `spin_kp` / `spin_max_omega` and exits at `\|yaw_err\| < spin_yaw_tolerance` (~1.1°).

> Phase 3 bypasses `nav2_behaviors::Spin` because the latter's costmap-based collision check spuriously fires when the lidar glimpses the robot's own body during a quick rotation. We publish `Twist` on `/cmd_vel` directly.

---

## Phase 4 — line-tracking advance + visual-servo handover

The core of the new design. The robot drives forward in two regimes:

```
 ┌──────────────────────────────────────────┐    Far regime (line not yet stabilised)
 │  LINE-TRACKING                            │    • Map-frame pure-pursuit
 │  (pure-pursuit on the perpendicular line │    • Running-average filter is refined
 │   through the running-average tag)        │      on every accepted detection
 └──────────────────────────────────────────┘    • Heading = ±atan2(lateral, lookahead)
                  │
                  │   Triggers (first-of):
                  │     • N refinement samples accepted
                  │     • OR distance ≤ visual_servo_distance
                  ▼
 ┌──────────────────────────────────────────┐    Near regime (line stable, near-field
 │  VISUAL SERVO                             │    detections are noisy)
 │  (camera-frame closed-loop on the         │    • Running-average is FROZEN
 │   image-frame angle to the tag)           │    • omega = −kp · low_pass(atan2(X_cam, Z_cam))
 └──────────────────────────────────────────┘    • The live image direction drives heading
                  │
                  ▼  distance ≤ docking_distance
                STOP
```

### Phase 4 — far regime: line-tracking pure-pursuit

Every iteration:

```python
# Robot pose
rx, ry, ryaw = lookup map → base_link

# Distance to the running-average tag (drives the handover)
distance_to_avg = ‖(avg.x, avg.y) − (rx, ry)‖

# Fold the latest detection into the running average if:
#   (1) it's fresh,
#   (2) it's not an outlier (sample_offset ≤ refinement_outlier_threshold),
#   (3) we're still in the far regime.
if fresh and not outlier and not in_visual_servo:
    weight = clamp(distance_to_avg / refinement_weight_full_distance,
                   refinement_weight_min, 1.0)
    avg.update(pos, quat, weight=weight)         # DISTANCE-WEIGHTED MEAN

# Targets from the (refined) running average
perp_yaw = avg.perpendicular_yaw(rx, ry)
lateral  = avg.signed_lateral_offset(rx, ry, perp_yaw)

# Pure-pursuit on the perpendicular line
desired_yaw = perp_yaw − atan2(lateral, line_lookahead_distance)
yaw_err     = normalize_angle(desired_yaw − ryaw)
omega       = clamp(line_yaw_kp · yaw_err, ±drive_yaw_max_omega)

# Forward speed with linear taper inside 2 × docking_distance
v = drive_speed_with_taper(distance_to_avg, docking_distance)

publish Twist(linear.x = v, angular.z = omega) on /cmd_vel
```

#### Robustness layers on the running-average update

These three layers stop noisy near-field detections from corrupting the line:

1. **Outlier rejection** — samples > `refinement_outlier_threshold` (default 0.30 m) from the current mean are skipped (single-frame solvePnP glitches).
2. **Distance-weighted mean** — `weight = clamp(distance / weight_full_distance, weight_min, 1.0)`. Far samples (cleaner) dominate the mean over near samples (noisier).
3. **Stabilisation count** — after `line_stabilization_samples` accepted refinements (default 25), the line is declared "stable" and Phase 4 hands the heading over to the visual servo.

#### Why this works

- `atan2(lateral, lookahead)` maps the signed lateral offset to a **bounded** heading deviation (±π/2). When far from the line, the robot heads toward it at a steep angle; as `lateral → 0`, the desired yaw collapses to `perpendicular_yaw` — no standing yaw bias.
- The weighted mean keeps **all** detections in the running average but lets the cleanest ones dominate.
- The forward speed taper inside `2 × docking_distance` slows the robot near the goal so the convergence has more time to act.

### Phase 4 — handover criterion

The transition to visual servoing is **first-of**, two triggers in parallel:

```python
line_stabilised_by_count    = samples_in_phase4 >= line_stabilization_samples
line_stabilised_by_distance = distance_to_avg   <= visual_servo_distance
in_visual_servo = line_stabilised_by_count or line_stabilised_by_distance
```

The count trigger is the primary one — it fires while detections are still far-field and clean. The distance trigger is the failsafe: if outlier rejection kills most samples (e.g. lighting issues), we still hand over before entering the noisy near-field.

A single log line marks the transition: `line stabilised (count: 25 samples, d=1.10m) — switching to camera-frame visual servo`.

### Phase 4 — near regime: camera-frame visual servo

Once handed over, the controller forgets the running-average and steers directly from the live camera-frame angle to the tag:

```python
# Live tag pose in camera_optical_frame (looked up via TF)
tx_cam, _, tz_cam = lookup camera_optical_frame → charging_dock_apriltag

# Image-frame angle to the tag (atan2 because +X is image-right, +Z is forward)
raw_angle = atan2(tx_cam, tz_cam)

# Low-pass against single-frame solvePnP spikes
filtered_image_angle = alpha · raw_angle + (1 − alpha) · filtered_image_angle_prev

omega = clamp(−visual_servo_kp · filtered_image_angle, ±drive_yaw_max_omega)

v = drive_speed_with_taper(distance_to_avg, docking_distance)

publish Twist(linear.x = v, angular.z = omega) on /cmd_vel
```

#### Why visual servo for the last leg

`atan2(X_cam, Z_cam)` is **self-consistent** with what the camera actually sees: it directly describes where the tag is in the image, not where solvePnP thinks it is in the map. In the near field, solvePnP's *map-frame* output drifts (the tag's corners hug the bottom of the FOV → noisy corner detection → noisy pose), but the *image-frame angle* remains stable as a steering signal. Keeping the tag centred in the image is geometrically equivalent to aiming straight at it.

If the TF is briefly unavailable (between detection frames), `omega` falls back to 0 — the robot drives straight rather than blind-steer.

### Stop condition

Phase 4 stops when `distance(robot, avg_tag) ≤ docking_distance` (0.9 m, default). A **travel-safety bound** aborts the phase if the robot has travelled more than `initial_distance + 0.5 m` without reaching the goal — guards against runaway loops.

---

## Coordinate conventions

- `lateral > 0` → robot is to the **left** of the dock's perpendicular axis (looking along the approach direction).
- `omega > 0` → robot rotates CCW.
- `perpendicular_yaw` is the yaw the robot has when **facing the tag** perpendicular to its plane. For Raj's setup with the dock tag normal pointing `−x` (yaw=π in SDF), the robot's approach yaw = `0` (facing `+x`).

See [`03_tf_frames.md`](03_tf_frames.md) for the full TF chain.

---

## What was retired from earlier sequencer designs

| Earlier design element | Why removed |
|---|---|
| 7-phase pipeline with separate Nav2 hops | Replaced by phase 4 line-tracking, which converges in one continuous controller |
| Exponential low-pass auto-calibration on `perpendicular_yaw` | Replaced by the `TagRunningAverage` (true incremental mean), stable without an arbitrary blend coefficient |
| Reverse-and-realign safety loop (4-state machine) | Replaced by pure-pursuit's continuous convergence + the travel-safety bound; no "reverse then retry" |
| Phase 4b "visual_servo" as a one-shot align + straight-line | Replaced by a **closed-loop** camera-frame servo with low-pass smoothing. Triggered by line stabilisation (count) rather than just distance. |

The result is fewer moving parts, fewer parameters, and a controller whose convergence can be reasoned about with a simple two-equation linearised model.
