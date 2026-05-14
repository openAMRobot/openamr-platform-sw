# openamrobot_control

> **🚧 STUB package — not yet implemented.** This README captures the intended scope so that future work has a clear landing spot. The directory currently builds cleanly with `colcon build` but ships no nodes, launches, or configs.

## Intended scope

Low-level control integration for **OpenAMRobot**. The clean boundary with sibling packages:

| Layer | Owned by | Topic / interface |
|---|---|---|
| Navigation goals | `openamrobot_nav2` | publishes `/cmd_vel_nav` |
| Velocity smoothing + collision check | `openamrobot_nav2` (`velocity_smoother`, `collision_monitor`) | publishes `/cmd_vel` |
| **`/cmd_vel` → joint commands (real robot)** | **`openamrobot_control` (this package)** | publishes wheel velocity / effort commands |
| Hardware drivers | `openamrobot_drivers` | talks to motor controllers, encoders |

In simulation, the Gazebo `gz-sim-diff-drive-system` plugin (loaded from `openamrobot_description/urdf/gazebo_control.xacro`) replaces this entire package: it consumes `/cmd_vel` directly and applies torques. So `openamrobot_control` is **real-robot only**.

## Expected contents (when populated)

```
openamrobot_control/
├── config/
│   ├── controller_manager.yaml      ros2_control manager config
│   ├── diff_drive_controller.yaml   wheel separation, kinematic radius, limits
│   └── joint_state_broadcaster.yaml
├── launch/
│   └── control_bringup.launch.py    starts controller_manager + spawners
├── urdf/                            ros2_control <ros2_control> tag fragments
│   └── ros2_control.xacro
└── README.md
```

## Likely dependencies

To be added to `package.xml` when implementation starts:

- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `diff_drive_controller`
- `joint_state_broadcaster`
- a hardware-interface plugin matching the motor controller (TBD by `openamrobot_drivers`)

## Why a stub now

The top-level [`README.md`](../../../README.md) advertises this package as part of the platform. A scaffolded stub:

- makes the package show up in `ros2 pkg list`,
- gives CI a place to verify it builds (catch the day someone adds a broken `package.xml`),
- makes the future scope discoverable to contributors before any code lands.

Until real content is added, **do not depend on this package from other launches**.
