# docs/navigation

Top-level pointer for Nav2 documentation.

The Nav2 bringup, parameter tuning, SLAM Toolbox configuration, and RViz nav layout live in:

**[`ros2/src/openamrobot_nav2/`](../../ros2/src/openamrobot_nav2/)** — see its [README](../../ros2/src/openamrobot_nav2/README.md) for:

- the parameter highlights (controller, planner, costmaps, smoother, collision_monitor) with the rationale for every non-default choice,
- the `/cmd_vel` chain (controller → velocity_smoother → collision_monitor → /cmd_vel) and how to verify each link,
- SLAM Toolbox configuration (mapping mode, the 4-second delay rationale),
- the recommended composition pattern (`sim_nav.launch.py` in `openamrobot_bringup` composes Gazebo + this package).

## Running Nav2

You typically don't launch `openamrobot_nav2` directly. The composed entry points:

```bash
# Gazebo + Nav2 + SLAM (no docking):
ros2 launch openamrobot_bringup sim_nav.launch.py

# Gazebo + Nav2 + SLAM + AprilTag + dock_trigger:
ros2 launch openamrobot_docking docking_sim.launch.py
```

Once Nav2 is active (~10 s after launch), send a goal from RViz with **2D Goal Pose** in the top toolbar.

## Tuning notes

Scenario-specific tuning notes (why `velocity_smoother.max_decel` is softened, why `collision_monitor.FootprintApproach` is disabled in sim) are recorded in:

- [`openamrobot_nav2/README.md`](../../ros2/src/openamrobot_nav2/README.md) — the why-not-defaults for every parameter
- [`openamrobot_docking/docs/12_lessons_learned.md`](../../ros2/src/openamrobot_docking/docs/12_lessons_learned.md) — design-decision diary that walks through why each tuning lands where it does

For the real robot, expect to re-enable `FootprintApproach` and re-tune costmap inflation. See [`openamrobot_nav2/README.md#adapting-to-real-hardware`](../../ros2/src/openamrobot_nav2/README.md#adapting-to-real-hardware).
