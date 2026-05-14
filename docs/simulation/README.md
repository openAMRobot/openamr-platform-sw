# docs/simulation

Top-level pointer for Gazebo Harmonic simulation documentation.

The reusable Gazebo bringup, the ros↔gz bridge, and the generic walled / depot worlds live in:

**[`ros2/src/openamrobot_gazebo/`](../../ros2/src/openamrobot_gazebo/)** — see its [README](../../ros2/src/openamrobot_gazebo/README.md) for the runnable usage, the complete bridge topic table, what `gz_simulator.launch.py` does step by step, and the common pitfalls (the "do not add a standalone `joint_state_publisher`" trap, spawn `z = 0` requirement, etc).

## Where to launch from

You **rarely** launch `openamrobot_gazebo` directly. The three runnable entry points are in [`openamrobot_bringup`](../../ros2/src/openamrobot_bringup/) and [`openamrobot_docking`](../../ros2/src/openamrobot_docking/):

| Launch | What it gives you | Run with |
|---|---|---|
| `sim_only` | Gazebo + robot, no Nav2 | `ros2 launch openamrobot_bringup sim_only.launch.py` |
| `sim_nav` | Gazebo + Nav2 + SLAM + RViz | `ros2 launch openamrobot_bringup sim_nav.launch.py` |
| `docking_sim` | + AprilTag + dock world + 4-phase sequencer | `ros2 launch openamrobot_docking docking_sim.launch.py` |

See [`ros2/src/openamrobot_bringup/README.md`](../../ros2/src/openamrobot_bringup/README.md) for the layering rationale and which level to pick.

## Where scenario-specific assets live

Generic, reusable simulation assets are in `openamrobot_gazebo`. Scenario-specific assets (worlds, models for a specific use case) are owned by the package that owns the scenario:

| Asset | Package | Path |
|---|---|---|
| `walled_world.sdf` (default empty arena) | `openamrobot_gazebo` | `worlds/walled_world.sdf` |
| `depot.sdf` (warehouse) | `openamrobot_gazebo` | `worlds/depot.sdf` |
| `docking_scenario.sdf` (10×10 m room + AprilTag dock) | `openamrobot_docking` | `worlds/docking_scenario.sdf` |
| `apriltag_dock` model | `openamrobot_docking` | `models/apriltag_dock/` |

This split keeps `openamrobot_gazebo` scenario-agnostic — it knows how to *run* Gazebo, not which world you happen to need today.
