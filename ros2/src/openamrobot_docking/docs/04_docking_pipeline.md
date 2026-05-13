# Docking Pipeline

This package uses AprilTag detections to compute the dock pose and feed it into `opennav_docking`.

**Relevant files:**
- `openamrobot_docking/launch/openamrobot_docking.launch.py`
- `openamrobot_docking/launch/detected_dock_pose_publisher.launch.py`
- `openamrobot_docking/src/detected_dock_pose_publisher.cpp`
- `openamrobot_docking/config/openamrobot_docking.yaml`
- `openamrobot_docking/config/docking_pose_publisher.yaml`
- `openamrobot_docking/scripts/dock_trigger.py`

## High-level flow
1. Camera + rectification + AprilTag detection runs.
2. AprilTag node publishes TF: `camera_optical_frame -> charging_dock_apriltag`.
3. A TF chain from `map` to `camera_optical_frame` (via localization) exists.
4. `detected_dock_pose_publisher` looks up TF `map -> charging_dock_apriltag` and publishes `PoseStamped` on `detected_dock_pose`.
5. `opennav_docking` consumes the pose and executes docking.

## Dock trigger
`openamrobot_docking/scripts/dock_trigger.py` listens to a Boolean topic. When `true`, it sends a `DockRobot` action goal; optionally it can undock on `false`.

## Key docking parameters (current defaults)
From `openamrobot_docking/config/openamrobot_docking.yaml`:
- `base_frame`: `base_link`
- `fixed_frame`: `odom`
- `dock_plugins`: `['home_dock_plugin']`
- `home_dock.frame`: `map`
- `home_dock.pose`: `[2.15, 0.0, 0.0]`

These must match your TF tree and your map coordinate system.

## Mermaid block diagram
```mermaid
graph TD
  CAM[Camera Node] --> RECT[Rectify Node]
  RECT --> TAG[AprilTag Node]
  TAG -->|TF: camera_optical_frame -> charging_dock_apriltag| TF2[TF Tree]
  LOC[Localization
map->odom->base_link] --> TF2
  TF2 --> DET[detected_dock_pose_publisher]
  DET --> POSE[detected_dock_pose (PoseStamped)]
  POSE --> DOCK[opennav_docking]
  TRIG[dock_trigger.py] --> DOCK
```
