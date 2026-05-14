# openamrobot_perception

> **🚧 STUB package — not yet implemented.** This README captures the intended scope so future work has a clear landing spot. The directory currently builds cleanly with `colcon build` but ships no nodes, launches, or configs.

## Intended scope

Perception modules **beyond** the AprilTag docking pipeline. The clean boundary with sibling packages:

| Concern | Owned by |
|---|---|
| AprilTag detection for docking | `openamrobot_docking` (uses `apriltag_ros`) |
| 2D lidar costmap layers | `openamrobot_nav2` (Nav2's `obstacle_layer`, `voxel_layer`) |
| **Higher-level perception (this package)** | `openamrobot_perception` |

## Candidate contents

The exact perception capabilities depend on the deployment. Likely candidates:

```
openamrobot_perception/
├── config/
│   ├── people_tracker.yaml
│   ├── obstacle_classifier.yaml
│   └── signage_recognition.yaml
├── launch/
│   ├── people_tracker.launch.py
│   ├── obstacle_classifier.launch.py
│   └── perception_bringup.launch.py     all of the above
├── openamrobot_perception/
│   ├── people_tracker_node.py           subs /scan + /camera/image_rect
│   ├── obstacle_classifier_node.py
│   └── signage_recognition_node.py
└── README.md
```

Examples of features that may land here:

- **Person tracking** via 2D lidar leg detection or via the camera (YOLO or similar). Output: `/perception/persons` (`vision_msgs/Detection3DArray`).
- **Semantic obstacle classification**: distinguishing static furniture from people from temporary objects, used by Nav2's costmap to set different inflation behaviours.
- **Signage / fiducial recognition** beyond AprilTag (QR codes, ArUco markers, custom signage) for non-docking tasks.
- **Depth / 3D processing** if a depth camera is added (point cloud filtering, ground-plane extraction).

## Topic-naming convention

To stay consistent with the rest of the stack, perception outputs should live under the `/perception/` namespace:

| Topic | Type | Producer |
|---|---|---|
| `/perception/persons` | `vision_msgs/Detection3DArray` | person tracker |
| `/perception/obstacles` | `vision_msgs/Detection3DArray` | classifier |
| `/perception/signage` | `vision_msgs/Detection2DArray` | sign recognition |

This keeps Nav2 / docking topics (`/scan`, `/cmd_vel`, `/detected_dock_pose`) clearly separated from perception products.

## Why a stub now

The top-level [`README.md`](../../../README.md) advertises this package as part of the platform. A scaffolded stub:

- makes the package show up in `ros2 pkg list`,
- gives CI a place to verify it builds,
- makes the future scope discoverable to contributors before any code lands.

Until real content is added, **do not depend on this package from other launches**.
