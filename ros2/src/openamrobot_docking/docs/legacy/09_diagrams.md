# Diagrams

These diagrams summarize the system. Use them in reports or lectures.

## System block diagram
```mermaid
graph LR
  subgraph SENSORS[Perception]
    CAM[Camera] --> RECT[Rectify]
    RECT --> TAG[AprilTag Detector]
  end

  subgraph TF[TF Tree]
    LOC[Localization
map->odom->base_link]
    MOUNT[Static TF
base_link->camera_link->camera_optical_frame]
    TAGTF[Tag TF
camera_optical_frame->charging_dock_apriltag]
  end

  subgraph DOCK[Docking]
    DET[detected_dock_pose_publisher]
    SERVER[opennav_docking]
    TRIG[dock_trigger.py]
  end

  CAM --> RECT --> TAG
  TAG --> TAGTF
  LOC --> DET
  MOUNT --> DET
  TAGTF --> DET
  DET --> SERVER
  TRIG --> SERVER
```

## TF frame tree (simplified)
```mermaid
graph TD
  MAP[map]
  ODOM[odom]
  BASE[base_link]
  CAML[camera_link]
  CAMO[camera_optical_frame]
  TAG[charging_dock_apriltag]

  MAP --> ODOM --> BASE --> CAML --> CAMO --> TAG
```

## Docking state flow (conceptual)
```mermaid
stateDiagram-v2
  [*] --> Idle
  Idle --> Detecting : dock_trigger true
  Detecting --> Staging : pose stable
  Staging --> Approaching : at staging pose
  Approaching --> Docked : docking_threshold met
  Docked --> [*]
  Detecting --> Idle : timeout / cancel
  Staging --> Idle : timeout / cancel
  Approaching --> Idle : timeout / cancel
```

## Parameter dependency graph
```mermaid
graph TD
  TAGSIZE[Tag Size] --> DETACC[Detection Accuracy]
  CAMTF[Camera TF] --> POSE[Map->Tag Pose]
  LOCMAP[Localization] --> POSE
  POSE --> DOCKPOSE[Dock Pose]
  DOCKPOSE --> DOCKCTL[Dock Controller]
  SPEED[v_linear_max] --> DOCKCTL
  THRESH[docking_threshold] --> DOCKCTL
```
