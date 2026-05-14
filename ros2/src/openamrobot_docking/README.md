# openamrobot_docking

ROS 2 autodocking for **OpenAMRobot** — AprilTag-based, fully simulated. Provides:

- a **4-phase docking sequencer** (`scripts/dock_trigger.py`): Nav2 staging → centring scan + initial filter → align spin → line-tracking pure-pursuit + straight-line final approach
- a TF → PoseStamped bridge for the detected dock (`src/detected_dock_pose_publisher.cpp`)
- AprilTag launches for both Gazebo simulation (`apriltag_sim.launch.yml`, uses the gz camera bridge) and real-robot (`apriltag.launch.yml`, uses `camera_ros`)
- the **full docking simulation** (`launch/docking_sim.launch.py`) composing `openamrobot_gazebo` + `openamrobot_nav2` + the docking-specific pieces shipped here
- the docking world (10×10 m room + AprilTag dock against the north wall) and the dock model

This is the **only package in the repo that knows about docking specifically**. Everything below `openamrobot_docking` (description, gazebo, nav2) is docking-agnostic and reusable.

---

## Quickstart

```bash
# In a CycloneDDS terminal with the workspace sourced:
ros2 launch openamrobot_docking docking_sim.launch.py

# Wait ~10 s for SLAM + Nav2 lifecycle to activate, then:
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

End state: the robot is stopped ~0.90 m in front of the AprilTag, perpendicular to the tag plane. Typical residual error: a few centimetres laterally and ~1° in yaw (dominated by AprilTag detection noise).

For the full why-and-how, [`docs/`](docs/) has the engineering documentation (overview, parameters, sequencer phase-by-phase, troubleshooting, lessons-learned diary).

---

## Contents

```
openamrobot_docking/
├── scripts/
│   └── dock_trigger.py             the 4-phase docking sequencer (~ 970 lines)
├── src/
│   └── detected_dock_pose_publisher.cpp   TF → /detected_dock_pose 10 Hz (C++)
├── launch/
│   ├── docking_sim.launch.py             full sim bringup (recommended)
│   ├── openamrobot_docking.launch.py     legacy controlled_approach bringup (kept for reference)
│   ├── apriltag.launch.yml               real-robot apriltag (with camera_ros)
│   ├── apriltag_sim.launch.yml           sim apriltag (gz camera, no rectification)
│   └── detected_dock_pose_publisher.launch.py
├── config/
│   ├── dock_trigger.yaml                 4-phase params (every knob is commented in-place)
│   ├── tags_36h11_sim.yaml               sim AprilTag detector (size: 0.40 m, family: 36h11, id: 0)
│   ├── tags_36h11.yaml                   real-robot AprilTag detector
│   ├── openamrobot_docking.yaml          legacy docking_server params
│   ├── openamrobot_docking_backup.yaml
│   └── docking_pose_publisher.yaml
├── worlds/
│   └── docking_scenario.sdf              10×10 m room with AprilTag dock against the north wall
├── models/
│   └── apriltag_dock/                    0.40 × 0.40 × 0.01 m textured panel
├── docs/                                 engineering documentation, see docs/README.md
├── CMakeLists.txt
├── package.xml
└── setup.py / setup.cfg
```

`CMakeLists.txt` builds the C++ `detected_dock_pose_publisher` and installs the launch/config/world/model trees; the Python `dock_trigger.py` is shipped as a script via the same CMake install.

---

## Two launch entry points

### `docking_sim.launch.py` — recommended

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Composes the full platform stack:

| Sibling package           | What it provides                                          |
| ------------------------- | --------------------------------------------------------- |
| `openamrobot_description` | URDF + STL meshes + Gazebo plugin tags                    |
| `openamrobot_gazebo`      | Gazebo Harmonic + ros↔gz bridge + spawn                   |
| `openamrobot_nav2`        | Nav2 stack + SLAM Toolbox                                 |
| (this package)            | AprilTag pipeline, dock model + world, 4-phase sequencer  |

This launch also passes `world:=docking_scenario.sdf` to `openamrobot_gazebo`, extends `GZ_SIM_RESOURCE_PATH` so gz-sim can find `model://apriltag_dock`, and starts the apriltag pipeline + `detected_dock_pose_publisher` + `dock_trigger.py` + RViz.

**Launch arguments**:

| Argument | Default | Description |
|---|---|---|
| `spawn_x` | `-4.0` | robot spawn X in the **world** frame (m) |
| `spawn_y` | `-4.0` | robot spawn Y in the **world** frame (m) |
| `spawn_yaw` | `0.0` | robot spawn yaw in the **world** frame (rad) |

> The dock's **world-frame** pose is fixed by `docking_scenario.sdf` at `(0, 4.9, π/2)`. The launch re-projects that pose through the spawn-pose inverse rotation to compute the dock's **map-frame** pose (which is what `dock_trigger.py` consumes via `dock_pose_{x,y,yaw}`). This is correct because SLAM Toolbox places the **map origin** at the robot's spawn pose, so every change of spawn moves the map frame, but the *physical* dock is unchanged in the world. The dock_trigger parameters are overridden at launch time with the auto-projected map-frame values — you don't need to edit YAML to change spawn.

### `openamrobot_docking.launch.py` — legacy

Brings up `opennav_docking::SimpleChargingDock` for the original Nav2 `controlled_approach` flow. The 4-phase sequencer in `docking_sim.launch.py` is what we test end-to-end in this revision; `openamrobot_docking.launch.py` is kept for reference and as a fallback path on the real robot.

### `apriltag_sim.launch.yml` / `apriltag.launch.yml`

Both start `apriltag_ros::apriltag_node`, namespaced as `/apriltag`, with the topic remaps:

```
/apriltag/image_rect       →  /camera/image_raw
/camera/camera_info        →  /camera/camera_info
```

Sim variant: passes `config/tags_36h11_sim.yaml`, expects images straight from the Gazebo camera bridge (no rectification needed in sim).

Real variant: passes `config/tags_36h11.yaml`, depends on `camera_ros` running upstream to rectify the image.

---

## Triggering a dock

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The trigger topic is configured in `config/dock_trigger.yaml` (parameter `trigger_topic: dock_trigger`). Publishing `false` triggers an undock if `undock_on_false: true` (off by default).

---

## The 4-phase pipeline

`dock_trigger.py` runs a custom 4-phase sequence. It deliberately bypasses `opennav_docking::SimpleChargingDock` because the latter (in our scenario) produced curved trajectories that didn't arrive perpendicular to the dock.

| Phase | Action | What's running |
|---|---|---|
| **1** | `NavigateToPose` action to the staging pose, `dock − staging_distance × approach_dir` (default 2.5 m in front of the tag) | Nav2 controller (RPP), Nav2 planner |
| **2** | **Scan**: rotate until the tag is visible (open-loop), then closed-loop yaw P-control on the camera-frame image angle `atan2(X_optical, Z_optical)` until centred (±2°) for 5 consecutive frames. **Filter**: collect 40 detections into a `TagRunningAverage` (true incremental mean for position; sign-aligned componentwise mean for the quaternion). | `dock_trigger.py` publishes on `/cmd_vel_nav` |
| **3** | **ALIGN**: spin in place to the running-average `perpendicular_yaw` (heading needed to face the tag plane perpendicular). Tolerance ~1.1°. | same |
| **4a** | **Line-tracking pure-pursuit** along the perpendicular line through the averaged tag: `desired_yaw = perp_yaw − atan2(lateral, lookahead)`, `omega = line_yaw_kp × (desired_yaw − robot_yaw)` clamped by `drive_yaw_max_omega`. `v = drive_speed`, tapered inside `2 × docking_distance`. The running average keeps updating. | same |
| **4b** | At `distance ≤ visual_servo_distance` (1.4 m): one-shot in-place spin to `perp_yaw`, then straight-line forward (`omega = 0`) until `distance ≤ docking_distance` (0.9 m). Opens the loop in the near field where small lateral motion produces large image-angle changes. | same |

Every parameter is commented in [`config/dock_trigger.yaml`](config/dock_trigger.yaml). The tuning intuition for each gain is recorded there inline.

Phase 1 uses the Nav2 `NavigateToPose` action server. Phases 2/3/4 publish directly on `/cmd_vel_nav`, so they share the smoother + collision_monitor pipeline configured in `openamrobot_nav2`. They deliberately avoid `nav2_behaviors`' costmap-based collision check, which falsely fires when the simulated lidar glimpses the robot's own body during a quick rotation.

---

## TF chain — how detection becomes a pose

```
camera_link
└── camera_rgb_optical_frame                          ← URDF, static
    └── (apriltag_ros publishes:)
        └── charging_dock_apriltag                    ← AprilTag detection (per-frame)

map  ← SLAM Toolbox publishes
└── odom  ← gz DiffDrive publishes
    └── base_footprint
        └── base_link
            └── camera_link
                └── camera_rgb_optical_frame
                    └── charging_dock_apriltag        ← composed: map → tag
```

`detected_dock_pose_publisher` (C++ node) reads the composed TF `map → charging_dock_apriltag` and publishes `/detected_dock_pose` as `PoseStamped` at 10 Hz. `dock_trigger.py` consumes this for the running-average filter.

For the camera-frame centring in phase 2, `dock_trigger.py` looks up `camera_rgb_optical_frame → charging_dock_apriltag` **directly** (not via map) — this makes the scan invariant to solvePnP's small biases that affect the *position* in the map frame more than the *direction* in the camera frame.

---

## AprilTag setup

| Property | Value | Where |
|---|---|---|
| Family | `36h11` | `config/tags_36h11_sim.yaml` |
| Tag ID | `0` | same |
| Tag size | `0.40 m` (outer black square) | same |
| Frame published | `charging_dock_apriltag` | same |
| Physical panel | 0.40 × 0.40 × 0.01 m | `models/apriltag_dock/model.sdf` |
| Texture | `tag0_big.png` (200×200 px) mapped on the face | same |
| World position | `(0, 4.9, 0.3)` yaw=−π/2 | `worlds/docking_scenario.sdf` |
| Map position | `(4.0, 8.9, π/2)` (default spawn) | computed by the launch |

**Why a 40 cm tag in sim, not 25?** At 1.5 m distance with our 640×480 camera, a 0.40 m tag occupies ~125 px of side, giving ~12 px per code cell — solvePnP is sub-pixel accurate. A 25 cm tag at the same distance gives ~78 px → ~8 px per cell → the corner detection starts noticing aliasing. We could downscale on the real robot if needed; in sim, the 0.40 m is just easy.

The `tags_36h11_sim.yaml` configures the detector with `family: 36h11`, `ids: [0]`, `size: 0.40`, and `frames: [charging_dock_apriltag]`. **Keep these in sync with `model.sdf`** — if you change the tag size on the panel, change it in the YAML or solvePnP returns wrong distances.

---

## Coordinate conventions

Default spawn (`-4, -4, 0`):

| Frame | Position | Notes |
|---|---|---|
| Robot spawn (world) | `(-4, -4, 0)` yaw=0 | base_footprint at world z=0 |
| Map origin | coincides with spawn | SLAM Toolbox convention |
| AprilTag (world) | `(0, 4.9, 0.3)` yaw=−π/2 | tag normal points −y (south) |
| AprilTag (map, default spawn) | `(4.0, 8.9, π/2)` | approach yaw = +π/2 (robot ends up facing +y) |
| Staging zone (map) | `(4.0, 6.4, π/2)` | dock − `staging_distance × (cos yaw, sin yaw)` = dock − 2.5 m south |
| Final dock distance | 0.9 m | `docking_distance` in `dock_trigger.yaml` |

The dock pose in the map frame depends on the spawn pose; the launch re-projects automatically. Don't hardcode `dock_pose_{x,y,yaw}` in the YAML unless you're sure the launch override is also off — the override always wins.

---

## Troubleshooting

| Symptom | First thing to check |
|---|---|
| "tag never detected during scan" | `config/tags_36h11_sim.yaml` is being loaded — check the apriltag launch's path matches the installed file location |
| Sequencer dies silently after the goal succeeds | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set in the terminal that publishes the trigger |
| Phase 4 robot circles back and forth | `line_yaw_kp` too high or `line_lookahead_distance` too small — increase lookahead, decrease `kp` |
| Phase 4 lateral overshoot | `drive_speed` too high for the current `line_yaw_kp` — lower speed first |
| Phase 1 stops short of staging | `velocity_smoother.max_decel` too soft — but a hard brake makes the chassis pitch; better to accept the short stop and let phase 2 align |
| Robot in Gazebo doesn't move at all | Not a docking bug — check `openamrobot_description/README.md#wheel-traction` and `openamrobot_gazebo/README.md#do-not-add-a-standalone-joint_state_publisher` |

The full troubleshooting matrix is in [`docs/09_troubleshooting.md`](docs/09_troubleshooting.md).

---

## What this package does NOT contain

| Concern | Lives in |
|---|---|
| Robot URDF, meshes, Gazebo plugin tags | `openamrobot_description` |
| Gazebo launch, ros↔gz bridge, generic worlds | `openamrobot_gazebo` |
| Nav2 + SLAM stack and parameters | `openamrobot_nav2` |
| Top-level launch compositions (sim_only, sim_nav) | `openamrobot_bringup` |

---

## Status

The 4-phase pipeline is tested end-to-end in `docking_scenario.sdf`. Migration to real hardware is the next step — the controllers and filter are hardware-agnostic; only the camera intrinsics and the dock pose need to be re-grounded in the real environment. See [`docs/`](docs/) for the migration notes.
