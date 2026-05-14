# openamrobot_bringup

Top-level launch compositions for the **OpenAMRobot** stack. This package contains **only launches** — it owns no robot description, no Gazebo configuration, no Nav2 parameters. Its single responsibility is to wire the sibling packages together in useful ways.

Think of it as the "front door" of the workspace: users learn one package name (`openamrobot_bringup`) and pick the level that matches what they need.

---

## The three launch levels

The stack is layered. Each level adds one capability on top of the previous one. **Pick the lowest level that gives you what you need** — every extra layer costs startup time and CPU.

```
                                     ┌─────────────────────────────────────────────────────┐
Level 3 — full docking sim           │  + AprilTag pipeline + dock model + 4-phase trigger │
(in openamrobot_docking)             └────────────────────────────▲────────────────────────┘
                                                                  │
                                     ┌────────────────────────────┴────────────────────────┐
Level 2 — sim + navigation           │  + Nav2 + SLAM Toolbox + RViz (nav layout)          │
sim_nav.launch.py                    └────────────────────────────▲────────────────────────┘
                                                                  │
                                     ┌────────────────────────────┴────────────────────────┐
Level 1 — sim only                   │  Gazebo + URDF + ROS↔gz bridge + robot_state_pub    │
sim_only.launch.py                   └─────────────────────────────────────────────────────┘
```

### Level 1 — `sim_only.launch.py`

```bash
ros2 launch openamrobot_bringup sim_only.launch.py
```

Thin wrapper around `openamrobot_gazebo/gz_simulator.launch.py`. Just Gazebo + the robot. No Nav2. No docking.

**Use it when**: you want to drive manually with teleop, validate the URDF/physics/friction, test sensor data, or iterate on Gazebo plugins without paying for Nav2 startup.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# moves the robot via /cmd_vel
```

### Level 2 — `sim_nav.launch.py`

```bash
ros2 launch openamrobot_bringup sim_nav.launch.py
```

Adds Nav2 + SLAM Toolbox + RViz on top. The robot can plan paths and navigate; you send goals with `2D Goal Pose` in RViz.

**Use it when**: you want to tune the navigation controller, validate costmap behaviour, build maps, or check Nav2 integration on the simulated platform. **No AprilTag, no dock, no docking sequencer.**

### Level 3 — `docking_sim.launch.py` (lives in `openamrobot_docking`)

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Adds the AprilTag detector, the dock model in the world, and the `dock_trigger.py` 4-phase sequencer.

**Use it when**: you want to test the docking pipeline end-to-end.

Level 3 lives in `openamrobot_docking` (not here) because the launch *is* part of the docking package's surface area — it ships the dock world, the dock model, and the parameter override that re-projects the dock pose. Keeping it next to its data makes the relationship obvious.

---

## Common launch arguments

All three launches share the spawn-pose convention:

| Argument | Default (sim_only / sim_nav) | Default (docking_sim) | Description |
|---|---|---|---|
| `spawn_x` | `0.0` | `-4.0` | robot spawn X in the world frame (m) |
| `spawn_y` | `0.0` | `-4.0` | robot spawn Y in the world frame (m) |
| `spawn_yaw` | `0.0` | `0.0` | robot spawn yaw in the world frame (rad) |
| `world` | `''` (= `walled_world.sdf`) | (overridden to `docking_scenario.sdf`) | full path to a `.sdf` world |
| `use_sim_time` | `True` | `True` | drive ROS time from Gazebo's `/clock` |
| `use_rviz` | (sim_only: no RViz; sim_nav: `True`) | `True` (docking RViz layout) | open RViz |

Example — start with the robot at (2, 1) facing north and the Nav2 RViz layout:

```bash
ros2 launch openamrobot_bringup sim_nav.launch.py \
    spawn_x:=2.0 spawn_y:=1.0 spawn_yaw:=1.5707
```

---

## Composition philosophy

Why so many launch files? Because each package owns one concern and its launch file reflects that. The `_bringup` package is just glue:

```
sim_only.launch.py
└── IncludeLaunchDescription(openamrobot_gazebo/gz_simulator.launch.py)
        └── Gazebo + URDF + bridge + robot_state_publisher

sim_nav.launch.py
├── IncludeLaunchDescription(openamrobot_gazebo/gz_simulator.launch.py)
│       └── (same as sim_only)
├── IncludeLaunchDescription(openamrobot_nav2/nav2_bringup.launch.py)
│       ├── TimerAction(4 s) → slam_toolbox/online_async_launch.py
│       └── nav2_bringup/bringup_launch.py
│           └── (controller, planner, behaviour, smoother, collision_monitor, ...)
└── Node(rviz2 with nav2_default.rviz)

openamrobot_docking/launch/docking_sim.launch.py
├── AppendEnvironmentVariable(GZ_SIM_RESOURCE_PATH, ./models)
├── IncludeLaunchDescription(openamrobot_gazebo/gz_simulator.launch.py, world=docking_scenario.sdf)
├── IncludeLaunchDescription(openamrobot_nav2/nav2_bringup.launch.py)
├── IncludeLaunchDescription(apriltag_sim.launch.yml)
├── IncludeLaunchDescription(detected_dock_pose_publisher.launch.py)
├── Node(dock_trigger.py, with reprojected dock pose)
└── Node(rviz2)
```

This lets a downstream user swap any one piece (e.g. real-robot driver instead of Gazebo) without rewriting the others.

---

## What this package does NOT contain

| Concern | Lives in |
|---|---|
| Robot URDF and meshes | `openamrobot_description` |
| Gazebo bringup, ros↔gz bridge, worlds | `openamrobot_gazebo` |
| Nav2 stack and SLAM Toolbox parameters | `openamrobot_nav2` |
| AprilTag, dock model, 4-phase sequencer, docking_sim launch | `openamrobot_docking` |

If you want to compose a new bringup (e.g. real-robot driver + Nav2 without docking), add a launch here and `IncludeLaunchDescription` the right sibling launches.

---

## Future launches

Slots that are likely to land here as the platform matures:

- `real.launch.py` — real-robot bringup: driver + Nav2 + (optional) docking, no Gazebo.
- `sim_dock.launch.py` — thin wrapper for `openamrobot_docking/docking_sim.launch.py` for consistency with `sim_only` / `sim_nav`. (Currently not added to avoid two paths to the same launch.)

Anything that composes existing packages into a new bringup belongs here, not in the leaf packages.
