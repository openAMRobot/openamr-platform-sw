# 4-Phase Docking Sequencer

Detailed walkthrough of the docking pipeline implemented in
`scripts/dock_trigger.py`. Replaces the earlier 7-phase sequencer.

## Why a custom sequencer at all

`opennav_docking::SimpleChargingDock::controlled_approach` (the upstream
docking pipeline) follows a curved path that does not always arrive
head-on at the dock. For an AprilTag dock that requires perpendicular
contact, this is fragile. The custom sequencer keeps the upstream Nav2
stack (planner, controller, costmaps) but replaces the final approach
with an explicit closed-loop controller that drives the robot directly
onto the dock's perpendicular axis.

## High-level flow

```
Phase 1: NavigateToPose → staging zone
   │
Phase 2: scan + initial filter
   │   2a. open-loop slow rotation until tag detected
   │   2b. closed-loop yaw P-control to centre the tag in the image
   │   2c. collect filter_num_samples into a running-average tag pose
   │
Phase 3: ALIGN — spin in place to the running-average perpendicular yaw
   │
Phase 4: line-tracking advance
   │   4a. pure-pursuit on the perpendicular line through the
   │       running-average tag (lateral convergence)
   │   4b. at distance < visual_servo_distance:
   │         - one-shot final-align spin to perpendicular yaw
   │         - then straight-line forward (omega = 0)
   │   4c. stop at distance ≤ docking_distance
```

## Phase 1 — NavigateToPose to staging

Sends a Nav2 NavigateToPose goal to `(dock_x − staging_distance ·
cos(dock_yaw), dock_y − staging_distance · sin(dock_yaw))`. The dock
pose comes from `dock_trigger.yaml`, or is overridden by the simulation
launch file based on the world spawn pose so the same physical dock
remains reachable from any starting position (see `02_architecture.md`).

Nav2's planner (A*) plans around walls, the controller (Regulated Pure
Pursuit) tracks the path, and the goal checker accepts the arrival when
the robot is within `xy_goal_tolerance` (0.15 m) and `yaw_goal_tolerance`
(0.05 rad) of the staging pose.

The `velocity_smoother` deceleration is intentionally soft (`max_decel:
-0.5` m/s²) so the robot doesn't overshoot the staging zone — important
because the new `omr_description` casters drag a bit more than the old
hand-tuned spheres did.

## Phase 2 — scan and initial filter

After Nav2 finishes, the robot may have a small yaw offset versus the
ideal staging pose. Even if the staging pose is exact, lidar/odom drift
during the Nav2 traversal can leave the camera slightly mis-aimed.
Phase 2 fixes this in two sub-steps.

### 2a / 2b — centring scan

The scan operates in two regimes:

- **Open-loop** rotation at `scan_rotation_speed` (0.3 rad/s) when no
  fresh detection is available.
- **Closed-loop** yaw P-control once the tag is in the camera frustum:

      omega = −scan_centring_kp · atan2(X_cam, Z_cam)
      saturated by scan_rotation_speed

  where `(X_cam, Y_cam, Z_cam)` is the tag's translation in
  `camera_rgb_optical_frame` (looked up via TF — not via the noisy
  map-frame `/detected_dock_pose`). In the optical frame, +X is
  image-right and +Z is forward, so `atan2(X, Z)` is the horizontal
  angle from the image centre to the tag centre.

The scan exits when `|atan2(X, Z)| < scan_centring_tolerance` (≈ 2°)
for `scan_consecutive_target` consecutive frames. Hard-limited by a
one-full-rotation timeout.

**Why camera-frame and not map-frame.** The map-frame solvePnP pose
carries a systematic bias (typically a few tens of centimetres) caused
by the simulated camera's intrinsics drift. The camera-frame
`atan2(X, Z)` is self-consistent with whatever the apriltag node
detects, so centring on it actually centres the tag in the image
regardless of the map-frame bias.

### 2c — initial filter

Once centred, the robot is stationary and starts folding fresh
detections into a `TagRunningAverage`:

- Positions average via incremental running mean.
- Quaternions are sign-aligned to the existing mean before being
  averaged componentwise, then renormalised. This is the well-known
  "naive quaternion mean" and is valid here because all samples lie
  close to each other on the sphere (the tag is static).

Phase 2 ends after `filter_num_samples` (40) detections have been
accumulated. The mean is much less noisy than any single detection.

## Phase 3 — ALIGN

Spin in place to `perpendicular_yaw`, the yaw the robot needs to be
perpendicular to the tag plane. This is computed from the running
average:

```
n = quat_rotate_z(avg_quat)             # tag's +Z axis (its normal) in map
n ← n / |n|
if n · (robot − tag) < 0: n ← −n        # disambiguate to "normal toward robot"
perpendicular_yaw = atan2(−ny, −nx)     # yaw to face the tag
```

The spin uses `spin_kp` / `spin_max_omega` and exits at
`spin_yaw_tolerance` (≈ 1.1°). This bypasses `nav2_behaviors::Spin`,
whose costmap-based collision check spuriously fires when the lidar
glimpses the robot's own body during a quick rotation.

## Phase 4 — line-tracking advance

This is the heart of the new design. Drive forward toward the
running-average tag while steering to stay on its perpendicular line.

### 4a — pure-pursuit on the perpendicular line

For `distance > visual_servo_distance` (1.4 m), the control law is:

```
lateral      = signed_lateral_offset(robot, avg_tag, perpendicular_yaw)
desired_yaw  = perpendicular_yaw − atan2(lateral, line_lookahead_distance)
yaw_err      = desired_yaw − robot_yaw
omega        = line_yaw_kp · yaw_err
omega        = saturated to ±drive_yaw_max_omega
v            = drive_speed (with a linear taper inside 2 · docking_distance)
```

The `atan2(lateral, lookahead)` maps lateral offset to a bounded heading
deviation: when off-axis, the robot points partly toward the line; as
`lateral → 0`, `desired_yaw → perpendicular_yaw` and the robot ends
perpendicular. No standing yaw bias.

The running average keeps updating throughout, so the perpendicular
direction is refined as more samples arrive.

### 4b — final align + straight-line approach

When the robot crosses `visual_servo_distance` (1.4 m), one-shot:

1. Stop forward motion.
2. Do an in-place spin to the latest `perpendicular_yaw` using
   `spin_kp` / `spin_max_omega` (re-uses phase-3 spin code).
3. Below that distance, hold `omega = 0` and drive forward at
   `drive_speed` with the taper.

This eliminates near-field wobble: at < 1 m from the tag, small lateral
motion produces large image-angle changes that any closed-loop yaw
corrector would over-react to. Holding the heading constant after a
clean final-align gives a much steadier final approach.

### 4c — stop condition

The phase stops when `distance(robot, avg_tag) ≤ docking_distance`
(0.9 m). A travel-safety bound aborts the phase if the robot has
travelled `forward_to_travel + 0.5 m` without reaching the goal.

## Coordinate convention recap

- `lateral > 0` → robot is to the **left** of the dock's perpendicular
  axis, looking along the approach direction.
- `omega > 0` → robot rotates CCW (left).
- `perpendicular_yaw` is the yaw the robot has when **facing** the tag
  perpendicular to its plane. For `dock_yaw = π/2` (north-facing
  approach), `perpendicular_yaw ≈ π/2`.

See `03_tf_frames.md` for the full TF chain and `05_parameters.md`
for the tunable parameters.

## What was retired from the earlier 7-phase design

| Old phase | Why removed |
| :---- | :---- |
| Phase 4 "parallel spot" + phase 5 Nav2 hop to that spot | Replaced by phase 4 line-tracking, which converges onto the perpendicular line in one continuous controller instead of two discrete Nav2 hops |
| Phase 7 auto-calibration via exponential low-pass on perpendicular yaw | Replaced by the `TagRunningAverage` (true incremental mean), which is stable without an arbitrary blend coefficient |
| Reverse-and-realign safety loop (4-state machine) | Replaced by the pure-pursuit converging onto the line continuously; failed convergence is now handled by the travel-safety bound, not by reversing |

The result is fewer moving parts, fewer parameters, and a controller
whose convergence behaviour can be reasoned about with a simple
two-equation linearised model — see lesson 18 in
`12_lessons_learned.md`.
