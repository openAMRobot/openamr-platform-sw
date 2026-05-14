# openamrobot_docking

ROS 2 autodocking package for **OpenAMRobot**. Provides:

- a **4-phase docking sequencer** (`scripts/dock_trigger.py`): Nav2 staging → centring scan + initial AprilTag filter → align spin → line-tracking pure-pursuit + straight-line final approach
- a TF → PoseStamped bridge for the detected dock (`src/detected_dock_pose_publisher.cpp`)
- AprilTag launches for both real-robot (`apriltag.launch.yml`) and Gazebo simulation (`apriltag_sim.launch.yml`)
- a **full docking simulation** (`launch/docking_sim.launch.py`) that composes `openamrobot_gazebo`, `openamrobot_nav2`, and the docking-specific pieces shipped here

## Two launch entry points

### `docking_sim.launch.py` — recommended, end-to-end docking simulation

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Composes the OpenAMRobot platform stack:

| Sibling package          | What it provides                                  |
| ------------------------ | ------------------------------------------------- |
| `openamrobot_description`| URDF/xacro + STL meshes                           |
| `openamrobot_gazebo`     | Gazebo Harmonic + ros_gz_bridge                   |
| `openamrobot_nav2`       | Nav2 stack + SLAM Toolbox                         |
| (this package)           | AprilTag pipeline, dock model, 4-phase sequencer  |

The launch accepts `spawn_x:=`, `spawn_y:=`, `spawn_yaw:=` to start the robot anywhere in the world — the dock's map-frame pose is auto-projected.

### `openamrobot_docking.launch.py` — legacy, controlled_approach pipeline

Brings up `opennav_docking::SimpleChargingDock` for the original Nav2 controlled_approach flow. Kept for reference; the 4-phase sequencer in `docking_sim.launch.py` is the path tested end-to-end in this revision.

## What's in this package

```
openamrobot_docking/
├── scripts/
│   └── dock_trigger.py         ← the 4-phase docking sequencer
├── src/
│   └── detected_dock_pose_publisher.cpp  ← TF → /detected_dock_pose 10 Hz
├── launch/
│   ├── docking_sim.launch.py             ← full sim bringup (recommended)
│   ├── openamrobot_docking.launch.py     ← legacy controlled_approach bringup
│   ├── apriltag.launch.yml               ← real-robot apriltag + camera_ros
│   ├── apriltag_sim.launch.yml           ← sim apriltag (gz camera, no rectification)
│   └── detected_dock_pose_publisher.launch.py
├── config/
│   ├── dock_trigger.yaml                 ← 4-phase params
│   ├── tags_36h11.yaml                   ← real-robot AprilTag detector params
│   ├── tags_36h11_sim.yaml               ← sim AprilTag (size: 0.40 m, family: 36h11, id: 0)
│   ├── openamrobot_docking.yaml          ← legacy docking_server params
│   ├── openamrobot_docking_backup.yaml
│   └── docking_pose_publisher.yaml
├── worlds/
│   └── docking_scenario.sdf              ← 10×10 m room + AprilTag dock against north wall
├── models/
│   └── apriltag_dock/                    ← 0.40 × 0.40 × 0.01 m textured panel
└── docs/                                 ← see docs/README.md
```

## Triggering a dock

In a terminal with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` exported:

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The sequencer logs each phase to stdout. End state: the robot stops ~0.90 m in front of the AprilTag, perpendicular to the tag plane. Typical residual error: a few centimetres laterally and ~1° in yaw, dominated by AprilTag detection noise.

See `docs/` (in this package and at the top of the repository) for parameter reference, the troubleshooting matrix, and the lessons-learned diary that documents every design decision.
