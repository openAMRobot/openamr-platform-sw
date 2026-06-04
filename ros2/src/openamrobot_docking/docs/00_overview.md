# Overview

What this package does, why it exists, and how to navigate the rest of the documentation.

For installation and a first run, see [`01_quickstart.md`](01_quickstart.md).

---

## What this package does

`openamrobot_docking` is a ROS 2 (Jazzy) package that makes a differential-drive robot autonomously dock onto a charging station marked with a **3-tag AprilTag bundle** (family 36h11, IDs **0, 1, 2** — outer tags at ±0.45 m from the centre on a horizontal line, all on 0.20 m × 0.20 m panels with a 0.16 m black-square edge).

In one sentence: **publish a `Bool true` on `/dock_trigger`, the robot drives to a staging zone near the dock, estimates the dock's surface normal from the wide-baseline bundle, refines its position with a camera-centric approach, and engages perpendicular to the tag at a configurable distance — aligned and ready to charge.**

This package handles:

- AprilTag bundle detection from a simulated camera (`apriltag_ros` configured for tags 0/1/2 → frames `charging_dock_tag_0`, `charging_dock_tag_1`, `charging_dock_tag_2`)
- TF transformation of the centre tag's pose into the `map` frame (`detected_dock_pose_publisher` publishes `/detected_dock_pose` from `charging_dock_tag_1` at 10 Hz)
- Navigation to a *staging* pose in front of the dock (via Nav2's `NavigateToPose` action)
- A **multi-phase docking sequencer** that takes over after staging:
  - in-camera centring scan,
  - dock-normal estimation from the **outer tags' wide baseline** (gives a stable perpendicular axis independent of single-tag yaw jitter),
  - back-off if the robot arrived too close, then a camera-frame goto-point-on-normal,
  - a re-verification of the normal against the agreed tolerance,
  - a final approach that pure-pursuits the dock normal, then **freezes the axis** at close range and switches to a **visual corrector on the centre tag** to absorb close-range pose noise.
- An **undock** maneuver (reverse a configurable distance, then 180° in-place spin) plus an undock-before-navigate gate on `/goal_pose`.

This package does **not** handle:

- Battery monitoring or charge-state detection — needs separate code
- Map building — assume Nav2 + AMCL is running on a pre-built map (or SLAM in mapping mode)
- The decision of *when* to dock — must be triggered externally (UI, scheduler, behaviour tree, etc.)

---

## Why this design

A single AprilTag's pose estimate has a well-known **yaw ambiguity** ("flip"): the small planar square gives two near-equal `solvePnP` solutions and a jittery surface-normal estimate, especially when viewed off-axis. A robot steered on a single noisy tag wobbles at the dock.

The current pipeline addresses this with **three** complementary mechanisms:

1. **3-tag bundle for a wide-baseline normal** — the outer tags (`charging_dock_tag_0` at `y = −0.45 m`, `charging_dock_tag_2` at `y = +0.45 m`) span 90 cm horizontally. The vector between them defines the dock surface direction, and its perpendicular is the dock normal. A 90 cm baseline locks the normal far more tightly than any single-tag PnP — the geometry is what makes the orientation stable, not the filtering.
2. **Camera-centric closed loop** — the sequencer expresses control in the camera/robot frame (image-frame angle to the centre tag, depth from the bundle pose) rather than in `map`. This makes the result independent of `map → odom` drift: if the wheels slip mid-approach, the next bundle observation corrects in the camera frame immediately, without an odometry-induced steady-state error.
3. **Two-regime final approach** — at close range (camera→centre-tag depth ≤ `freeze_axis_distance`), the dock normal is **frozen** at its averaged value and the controller switches to a **visual corrector** on the centre tag (`omega = −visual_servo_kp · atan2(X, Z)` in the camera optical frame). Freezing kills the close-range zig-zag caused by noisy near-field PnP, while the visual corrector keeps the centre tag aligned with the image centre — which is geometrically equivalent to driving straight onto the dock.

In simulation this delivers ~1–2 cm of lateral error and ~1° of yaw error at the dock. The longer research arc — vendor-agnostic precision target, sensing-method catalogue, validation protocol, failure modes, calibration — is in [`14_docking_research.md`](14_docking_research.md).

---

## High-level architecture

```
            ┌──────────────────────────────────────┐
            │  Gazebo camera plugin (gz sensor)    │
            └──────────────────┬───────────────────┘
                               │ gz /rgb_image
                               │ gz /camera_info
                               ▼
            ┌──────────────────────────────────────┐
            │  ros_gz_bridge (this package adds    │
            │  a /camera_info bridge instance)     │
            └──────────────────┬───────────────────┘
                               │ /rgb_image
                               │ /camera_info
                               ▼
            ┌──────────────────────────────────────┐
            │  apriltag_ros::apriltag_node         │
            │  (family 36h11, IDs 0/1/2,           │
            │   size 0.16 m, bundle layout)        │
            └──────────────────┬───────────────────┘
                               │ TF: camera_optical_frame →
                               │     charging_dock_tag_{0,1,2}
                               ▼
            ┌──────────────────────────────────────┐
            │  detected_dock_pose_publisher (C++)  │
            │  TF map→charging_dock_tag_1 → Pose   │
            └──────────────────┬───────────────────┘
                               │ /detected_dock_pose @ 10 Hz
                               ▼
            ┌──────────────────────────────────────┐
            │  dock_trigger.py (multi-phase seq.)  │
            │  • Phase 1:   NavigateToPose (Nav2)  │
            │  • Phase 2:   centring scan          │
            │  • Estimate dock normal from outer   │
            │    tags 0 & 2 (wide baseline)        │
            │  • Phase 1.5: back-off if too close  │
            │  • Phase 3:   goto point on normal   │
            │  • Phase 4:   re-verify normal       │
            │  • Phase 5:   axis-frozen visual     │
            │              servo on centre tag     │
            └──────────────────┬───────────────────┘
                               │ /cmd_vel
                               ▼
            ┌──────────────────────────────────────┐
            │  DiffDrive plugin (Gazebo)           │
            └──────────────────────────────────────┘
```

The full TF chain is in [`03_tf_frames.md`](03_tf_frames.md), the node graph + topic table in [`02_architecture.md`](02_architecture.md).

---

## Prerequisites at a glance

| What you need | Why |
|---|---|
| Ubuntu 24.04 + ROS 2 Jazzy | The whole stack targets Jazzy specifically |
| CycloneDDS as RMW (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) | FastDDS has a Python crash bug on Jazzy that breaks `dock_trigger.py` silently when sending action goals |
| `openamrobot_gazebo` + `openamrobot_nav2` + `openamrobot_description` installed | This package composes them at runtime |
| `apriltag_ros` (`sudo apt install ros-jazzy-apriltag-ros`) | The detector |
| `topic_tools` (only if you choose the relay variant) and `ros_gz_bridge` | Bridge / forwarding utilities |

For exact install commands and the launch sequence, see [`01_quickstart.md`](01_quickstart.md).

---

## Where to go next

| If you want to… | Start with |
|---|---|
| Install everything and run the simulation | [`01_quickstart.md`](01_quickstart.md) |
| Understand how the pipeline works | [`02_architecture.md`](02_architecture.md) |
| Read about the TF chain | [`03_tf_frames.md`](03_tf_frames.md) |
| Set up AprilTag detection (sim or real) | [`04_apriltag.md`](04_apriltag.md) |
| Tune parameters | [`05_parameters.md`](05_parameters.md) |
| Calibrate a real camera | [`06_camera_calibration.md`](06_camera_calibration.md) |
| Reproduce results | [`07_reproduce_results.md`](07_reproduce_results.md) |
| Deep-dive the 4-phase sequencer | [`08_sequencer_4phase.md`](08_sequencer_4phase.md) |
| Diagnose a failure | [`09_troubleshooting.md`](09_troubleshooting.md) |
| See block diagrams | [`10_diagrams.md`](10_diagrams.md) |
| Understand what we changed from upstream | [`11_changes_from_upstream.md`](11_changes_from_upstream.md) |
| Learn from past mistakes | [`12_lessons_learned.md`](12_lessons_learned.md) |
| Understand perception + how the line is built (and see it in RViz) | [`13_perception_and_line.md`](13_perception_and_line.md) |
| Read the docking research: chargers, sensing methods, validation, failure modes, calibration | [`14_docking_research.md`](14_docking_research.md) |
