# Overview

What this package does, why it exists, and how to navigate the rest of the documentation.

For installation and a first run, see [`01_quickstart.md`](01_quickstart.md).

---

## What this package does

`openamrobot_docking` is a ROS 2 (Jazzy) package that makes a differential-drive robot autonomously dock onto a charging station marked with an AprilTag.

In one sentence: **trigger a Bool topic, the robot drives to a pre-known dock location, refines its position using a camera-detected AprilTag, and stops at a configurable distance from the tag — aligned and ready to charge.**

The package handles:
- AprilTag detection (`apriltag_ros`)
- TF transformation of the tag pose into the map frame
- Navigation to a "staging" pose near the dock (`nav2_bringup`)
- Final alignment and approach using either:
  - **`opennav_docking::SimpleChargingDock`** (real-robot default — curved trajectory)
  - **Custom 4-phase sequencer** (simulation default, portable to real hardware — line-tracking pure-pursuit then straight-line final approach)

It does **not** handle:
- Battery monitoring or charge-state detection (must be implemented separately)
- Map building (use `slam_toolbox` or `slam_gmapping` upstream)
- The decision of *when* to dock (must be triggered externally)

---

## Why two pipelines?

The upstream `openamrobot_docking` package uses Nav2's `opennav_docking::SimpleChargingDock` for both navigation to staging and the final approach. This works, but in practice we found two limitations:

1. **The graceful controller produces curved trajectories.** The robot approaches the dock from the side, not head-on. Lateral offset accumulates along the curve.
2. **One-shot AprilTag detection is noisy.** Single-frame pose estimates have ~3–13 cm error depending on tag size, lighting, and camera resolution.

To address both, we built a **custom 4-phase docking sequencer** (`scripts/dock_trigger.py`) that:
- Centres the tag in the camera with a camera-frame closed-loop scan (phase 2)
- Folds 40 detections into a running-average tag pose that keeps updating throughout the approach (phase 2 + phase 4)
- Approaches via a pure-pursuit controller on the perpendicular line through the averaged tag (phase 4a), converging onto the dock axis
- Switches to a straight-line approach with no closed-loop yaw correction in the last metre to avoid near-field wobble (phase 4b)

Both pipelines coexist in this package. The real-robot launch uses Pipeline A by default; the simulation launch uses Pipeline B. Either can be ported to either platform — see [`02_architecture.md`](02_architecture.md) for the migration steps.

---

## High-level architecture

```
                            ┌─────────────────────────────────┐
                            │  Camera (real: camera_ros       │
                            │          sim:  gz camera plugin)│
                            └────────────┬────────────────────┘
                                         │ /camera/image_raw
                                         ▼
                            ┌─────────────────────────────────┐
                            │  apriltag_ros                   │
                            │  (real: + image_proc rectify)   │
                            └────────────┬────────────────────┘
                                         │ TF: camera → charging_dock_apriltag
                                         ▼
                            ┌─────────────────────────────────┐
                            │  detected_dock_pose_publisher   │
                            │  (C++, TF → PoseStamped)        │
                            └────────────┬────────────────────┘
                                         │ /detected_dock_pose (map frame, 10 Hz)
                                         ▼
                  ┌──────────────────────┴──────────────────────┐
                  │                                             │
        Pipeline A: opennav_docking                  Pipeline B: dock_trigger.py
        (controlled_approach,                        (4-phase sequencer,
         real-robot default)                          sim default + portable)
                  │                                             │
                  └──────────────────────┬──────────────────────┘
                                         ▼
                            ┌─────────────────────────────────┐
                            │  /cmd_vel → DiffDrive plugin    │
                            │             (or real-robot     │
                            │              base controller)   │
                            └─────────────────────────────────┘
```

For the full TF chain and a deeper architectural view, see [`03_tf_frames.md`](03_tf_frames.md) and [`02_architecture.md`](02_architecture.md).

---

## Prerequisites at a glance

| What you need | Why |
| :---- | :---- |
| **Ubuntu 24.04 + ROS 2 Jazzy** | The package targets Jazzy specifically; other distros are not tested. |
| **CycloneDDS as RMW** (`rmw_cyclonedds_cpp`) | FastDDS has a Python crash bug on Jazzy that breaks `dock_trigger.py` silently. |
| **A pre-built map + localization** (real) | Nav2 needs to know where the robot is. AMCL on a pre-built map for real; SLAM Toolbox in mapping mode for sim. |
| **A calibrated camera** (real) | AprilTag pose estimation needs intrinsic camera parameters. The sim camera provides them automatically. |
| **A printed AprilTag 36h11 ID 0** (real) | The dock is identified by this specific tag. Family and ID are configurable but must match across files. |
| **The dock's pose in the map frame** | Measured once, stored in config. Both pipelines need it. |

For exact `apt install` commands, see [`01_quickstart.md`](01_quickstart.md).

---

## Where to go next

| If you want to… | Start with |
| :---- | :---- |
| Install everything and run the simulation | [`01_quickstart.md`](01_quickstart.md) |
| Understand how the pipeline works | [`02_architecture.md`](02_architecture.md) |
| Read about the TF chain | [`03_tf_frames.md`](03_tf_frames.md) |
| Set up AprilTag detection | [`04_apriltag.md`](04_apriltag.md) |
| Tune parameters | [`05_parameters.md`](05_parameters.md) |
| Calibrate a real camera | [`06_camera_calibration.md`](06_camera_calibration.md) |
| Reproduce results | [`07_reproduce_results.md`](07_reproduce_results.md) |
| Deep-dive the 4-phase sequencer | [`08_sequencer_4phase.md`](08_sequencer_4phase.md) |
| Diagnose a failure | [`09_troubleshooting.md`](09_troubleshooting.md) |
| See block diagrams | [`10_diagrams.md`](10_diagrams.md) |
| Understand what we changed from the upstream repo | [`11_changes_from_upstream.md`](11_changes_from_upstream.md) |
| Learn from our mistakes (pedagogical) | [`12_lessons_learned.md`](12_lessons_learned.md) |

For the full index, see [`README.md`](README.md).
