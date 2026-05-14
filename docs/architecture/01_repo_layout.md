# Repo layout — openamr-platform-sw

This is the software side of the **OpenAMRobot platform**. It hosts the canonical ROS 2 packages for the differential-drive base: description, simulation, navigation, and docking.

## Directory layout

```
openamr-platform-sw/
├── ros2/
│   └── src/
│       ├── openamrobot_description/   ← URDF/xacro + STL meshes
│       ├── openamrobot_gazebo/        ← Gazebo Harmonic + ros_gz_bridge
│       ├── openamrobot_nav2/          ← Nav2 stack + SLAM Toolbox
│       └── openamrobot_docking/       ← AprilTag docking + 4-phase sequencer
│
├── simulation/
│   ├── worlds/        ← reusable generic worlds (placeholder)
│   ├── models/        ← reusable generic models (placeholder)
│   └── scenarios/     ← reusable generic scenarios (placeholder)
│
├── config/            ← top-level config overlays
│   ├── robot/         (placeholder)
│   ├── nav2/          (placeholder)
│   ├── docking/       (placeholder)
│   └── simulation/    (placeholder)
│
├── docs/
│   ├── architecture/    ← this folder; repo-level architectural notes
│   ├── getting_started/ ← onboarding, quickstart, first run
│   ├── docking/         ← top-level pointer; deep dive lives in ros2/src/openamrobot_docking/docs/
│   ├── navigation/      ← top-level pointer for openamrobot_nav2
│   ├── simulation/      ← top-level pointer for openamrobot_gazebo
│   └── safety/          ← (placeholder)
│
├── scripts/    ← repo-wide helper scripts (kill_sim.sh, etc.)
└── tools/      ← (placeholder)
```

## Package dependency graph

```
openamrobot_description
        ↑
        │
        ├──────── openamrobot_gazebo
        │              ↑
        │              │
        └──────── openamrobot_nav2
                       ↑
                       │
                 openamrobot_docking
```

- `openamrobot_description` has **no** OpenAMRobot deps — it ships the canonical URDF/xacro/meshes and is consumable by anyone (Gazebo, RViz, real-robot driver).
- `openamrobot_gazebo` depends on `openamrobot_description` (it loads the URDF and spawns it).
- `openamrobot_nav2` is independent of the simulator — it works against any URDF that exposes the standard TF + odom + scan topics.
- `openamrobot_docking` composes all three to provide the docking simulation; on real hardware it only needs `openamrobot_description` plus the camera/lidar drivers.

## What goes where — rule of thumb

| If it's about…                                                       | …it belongs in            |
|----------------------------------------------------------------------|---------------------------|
| Geometry of the robot, joint origins, meshes                         | `openamrobot_description` |
| Gazebo plugins, bridge config, generic test worlds                   | `openamrobot_gazebo`      |
| Nav2 controllers/planners/costmaps, SLAM, AMCL, RViz layout for nav  | `openamrobot_nav2`        |
| AprilTag detection, dock pose pipeline, docking sequencer, dock model| `openamrobot_docking`     |
| Cross-cutting scripts, top-level READMEs, license, governance        | the repo root             |

When a piece of code crosses two packages, prefer adding it to the higher-level consumer (e.g. a docking-related Nav2 parameter override goes in `openamrobot_docking/config`, not in `openamrobot_nav2/config`).

## Build all four packages

```bash
cd ~/openamr-platform-sw
colcon build --packages-up-to openamrobot_docking
source install/setup.bash
```

## Launch entry points

| What you want                                | How                                                                |
|---------------------------------------------|--------------------------------------------------------------------|
| View the robot in RViz                       | `ros2 launch openamrobot_description launch.py`                    |
| Gazebo Harmonic + the robot, no nav, no dock | `ros2 launch openamrobot_gazebo gz_simulator.launch.py`            |
| Nav2 + SLAM only (compose with a sim)        | `ros2 launch openamrobot_nav2 nav2_bringup.launch.py`              |
| Full docking simulation                      | `ros2 launch openamrobot_docking docking_sim.launch.py`            |
