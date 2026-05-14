# `config/` — product-level configuration

**Currently empty.** Reserved for configuration that is **product-wide**, not specific to a single ROS 2 package. Per-package YAML lives next to its code:

| Config concern | Lives in |
|---|---|
| Nav2 stack params | [`ros2/src/openamrobot_nav2/config/nav2_params.yaml`](../ros2/src/openamrobot_nav2/config/nav2_params.yaml) |
| SLAM Toolbox params | [`ros2/src/openamrobot_nav2/config/slam_toolbox_params.yaml`](../ros2/src/openamrobot_nav2/config/slam_toolbox_params.yaml) |
| Docking pipeline params | [`ros2/src/openamrobot_docking/config/dock_trigger.yaml`](../ros2/src/openamrobot_docking/config/dock_trigger.yaml) |
| AprilTag detector params | [`ros2/src/openamrobot_docking/config/tags_36h11_sim.yaml`](../ros2/src/openamrobot_docking/config/tags_36h11_sim.yaml) (sim) |
| ros↔gz bridge | [`ros2/src/openamrobot_gazebo/config/gz_bridge.yaml`](../ros2/src/openamrobot_gazebo/config/gz_bridge.yaml) |

## What belongs here

Things that don't naturally live inside one ROS 2 package, e.g.:

- deployment configs: site-specific calibration, network setup, fleet identity
- product-level safety policies (speed limits per zone, e-stop topology)
- multi-package overlays that override individual package YAMLs in a specific deployment

If a piece of config is consumed by exactly one package, it belongs **inside that package**, not here.
