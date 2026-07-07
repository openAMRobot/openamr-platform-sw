# openamrobot_docking — engineering documentation

This folder contains the engineering documentation for the autodocking pipeline shipped by [`openamrobot_docking`](../).

The package-level [README.md](../README.md) is the practical entry point ("how to run it today"). These docs are the **engineering record** — the *why* and *how it's built*.

---

## Layout

```
docs/
├── README.md                       ← this file (index)
│
├── 00_overview.md                  what the package is and why this design
├── 01_quickstart.md                run the docking simulation end-to-end
├── 02_architecture.md              node graph, lifecycle, topic flow
├── 03_tf_frames.md                 the TF chain (robot, camera, dock)
├── 04_apriltag.md                  AprilTag setup (sim + real robot)
├── 05_parameters.md                every dock_trigger.yaml parameter
├── 06_camera_calibration.md        intrinsics + extrinsics (real robot)
├── 07_reproduce_results.md         end-to-end reproduction checklist
├── 08_legacy_sequencer.md          legacy: notes on the original 4-phase pipeline
│                                   (superseded by docs 13 + 14 for the bundle stack)
├── 09_troubleshooting.md           symptom → cause → fix matrix
├── 10_diagrams.md                  block / TF / state diagrams (text)
├── 11_changes_from_upstream.md     what this revision changes vs prior pipelines
├── 12_lessons_learned.md           decisions diary with rationale
├── 13_perception_and_line.md       what AprilTag gives us + how the line is built + RViz markers
├── 14_docking_research.md          vendor-agnostic precision-docking research, validation,
│                                   failure modes, calibration, multi-dock
└── 15_legacy_near_approach.md      legacy: the pre-2026-07-07 NEAR-field visual servo
                                    (superseded by the dt-fix + depth-compensated version)
```

---

## How to read

| Goal | Read |
|---|---|
| **New to the pipeline** | `00_overview.md` → `01_quickstart.md` → `13_perception_and_line.md` |
| **Run it on my machine** | `01_quickstart.md` (3-terminal flow) |
| **Tune the controller** | `05_parameters.md` + `13_perception_and_line.md` |
| **Understand the current bundle architecture** | `13_perception_and_line.md` + `14_docking_research.md` |
| **Something doesn't work** | `09_troubleshooting.md` then `12_lessons_learned.md` for the deeper why |
| **Onboarding a teammate** | `10_diagrams.md` + `13_perception_and_line.md` |
| **Port to real hardware** | `04_apriltag.md` + `06_camera_calibration.md` + `03_tf_frames.md` + `14_docking_research.md` §8 |
| **Audit / understand a design choice** | `11_changes_from_upstream.md` + `12_lessons_learned.md` |
| **Vendor / sensing research, validation plan, failure modes** | `14_docking_research.md` |
| **Historical context (legacy single-tag pipeline)** | `08_legacy_sequencer.md` |
| **Roll back the NEAR-field corrector to before 2026-07-07** | `15_legacy_near_approach.md` (+ git tag `docking-legacy-pre-2026-07-07-near-approach`) |

---

## Conventions used throughout these docs

- **Frames**: `map → odom → base_link → camera_link → camera_optical_frame → charging_dock_tag_{0,1,2}` — three tag frames published by `apriltag_ros` (one per detected tag). The centre tag `charging_dock_tag_1` is the docking target; the outer tags `charging_dock_tag_0` / `charging_dock_tag_2` provide the wide baseline used to estimate the dock normal.
- **Topics**:
  - Image: `/rgb_image` (gz bridge)
  - Camera intrinsics: `/camera_info` (bridged in this package's launch)
  - Tag detections: `/apriltag/detections`
  - Dock pose in map: `/detected_dock_pose` (PoseStamped, 10 Hz — from the centre tag)
  - Drive commands: `/cmd_vel` (the sequencer publishes directly; Phase 1 goes through Nav2's action server)
  - Trigger: `/dock_trigger` (Bool), `/undock_robot` (Bool)
- **World coordinates** (Raj's `walled_world.sdf`):
  - Robot spawn: world `(0, 0, 0)` yaw=0
  - AprilTag dock bundle: world `(4.899, 0, 0.5)` yaw=π (panel mounted on the +x wall, tag normals facing −x); outer tags at `y = ±0.45 m` from the centre
  - Map ≡ world (AMCL initialised at map origin = robot spawn position)
- **Sequencer**: the multi-phase pipeline in [`scripts/dock_trigger.py`](../scripts/dock_trigger.py).
