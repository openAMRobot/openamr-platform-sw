# docs/safety

Placeholder for the platform-wide safety story. To be filled in as the real-robot bringup matures (E-stop integration, watchdog behaviour, fault-handling matrix, deployment-readiness checklist).

## Current safety-relevant configuration

Until the dedicated safety doc lands, the safety-relevant configuration lives in:

| Concern | Location |
|---|---|
| Nav2 collision monitor | [`ros2/src/openamrobot_nav2/config/nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml) (`collision_monitor` section) — currently has `FootprintApproach.enabled: false` because the simulated lidar self-intersects with the URDF body. **Re-enable on the real robot.** |
| Velocity smoother accel/decel limits | same file (`velocity_smoother` section) |
| Docking near-field decisions | [`ros2/src/openamrobot_docking/config/dock_trigger.yaml`](../../ros2/src/openamrobot_docking/config/dock_trigger.yaml) — `visual_servo_distance`, `docking_distance` |

## Where safety is currently discussed

- **Operational failures + recoveries**: [`ros2/src/openamrobot_docking/docs/09_troubleshooting.md`](../../ros2/src/openamrobot_docking/docs/09_troubleshooting.md)
- **Tip-over and braking dynamics**: [`ros2/src/openamrobot_docking/docs/12_lessons_learned.md`](../../ros2/src/openamrobot_docking/docs/12_lessons_learned.md) (lessons 7, 22, 23)
- **Wheel traction and contact**: [`ros2/src/openamrobot_description/README.md#wheel-traction--why-collision-radius--kinematic-radius`](../../ros2/src/openamrobot_description/README.md#wheel-traction--why-collision-radius--kinematic-radius) — relevant because a robot that can't get traction is also a robot whose `cmd_vel` chain is silently broken, which fails any safety analysis based on commanded vs measured velocity.

## What this doc will cover (eventually)

- E-stop architecture (hardware + software paths)
- Watchdog timeouts on `/cmd_vel`, `/scan`, `/joint_states`
- Lifecycle behaviour on partial subsystem failure
- Safe stop on lidar saturation / camera dropout
- Speed limits per zone (warehouse vs near-dock)
- Real-robot deployment safety checklist

If you have a safety concern that doesn't have an obvious home in the package READMEs, add it to this file rather than scattering it.
