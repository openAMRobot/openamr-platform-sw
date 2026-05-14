# docs/safety

Placeholder. The safety story for the OpenAMRobot platform (E-stop integration, collision-monitor configuration, watchdog behaviour, fault-handling matrix) is a deliberate platform-wide concern that should be written as the project matures.

Until then, the docking-specific safety considerations are recorded in:

- `ros2/src/openamrobot_docking/docs/09_troubleshooting.md` (operational failures + recoveries)
- `ros2/src/openamrobot_docking/docs/12_lessons_learned.md` (Lessons 7, 22, 23 on tip-over and braking dynamics)

Nav2's `collision_monitor` is configured in `ros2/src/openamrobot_nav2/config/nav2_params.yaml`. The `FootprintApproach` polygon is disabled by default — re-enable it in environments with real near-obstacle risk.
