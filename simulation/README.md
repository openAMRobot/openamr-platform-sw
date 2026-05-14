# `simulation/` — product-level simulation assets

**Currently empty.** Reserved for simulation assets that are **product-wide**, not owned by a specific ROS 2 package.

Per-package simulation assets live next to their consumer:

| Asset | Lives in |
|---|---|
| `walled_world.sdf`, `depot.sdf` (generic worlds) | [`ros2/src/openamrobot_gazebo/worlds/`](../ros2/src/openamrobot_gazebo/worlds/) |
| Gazebo bringup launch + bridge config | [`ros2/src/openamrobot_gazebo/`](../ros2/src/openamrobot_gazebo/) |
| `docking_scenario.sdf` + AprilTag dock model | [`ros2/src/openamrobot_docking/worlds/`](../ros2/src/openamrobot_docking/worlds/) and [`models/`](../ros2/src/openamrobot_docking/models/) |
| RViz layouts | [`ros2/src/openamrobot_nav2/rviz/`](../ros2/src/openamrobot_nav2/rviz/) |

## What belongs here

Cross-package or fleet-level simulation content, e.g.:

- multi-robot scenarios that need worlds *and* configs *and* coordinated launches
- benchmark/regression worlds used by CI
- shared models (warehouse racks, conveyor belts, charging stations) used by multiple scenario worlds

If a world or model is consumed by exactly one package, it belongs inside that package's `worlds/` or `models/` directory.

## Why we split this way

`openamrobot_gazebo` is **scenario-agnostic** — it knows how to run the simulator, not which world you happen to need. `openamrobot_docking` ships its own dock + room because that scenario is part of the docking package's surface area. This `simulation/` directory is the third tier: product-level assets that no single package owns.
