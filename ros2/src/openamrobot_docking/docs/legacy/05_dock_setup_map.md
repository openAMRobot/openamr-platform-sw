# Placing the AprilTag in the Map Frame

There are two common ways to place the dock/tag in the `map` frame:

1. **Fixed dock pose** (most repeatable): You measure the dock pose in the map and set it in `openamrobot_docking.yaml`.
2. **Live detection pose** (good for experimentation): You use TF from AprilTag detections to compute the tag pose in `map` and use that as the dock pose.

## Option 1: Fixed dock pose (recommended for reproducibility)
1. Run localization so `map -> odom -> base_link` exists.
2. Drive the robot to the dock and align it with the tag.
3. Use TF to read the tag pose in the `map` frame:

```bash
ros2 run tf2_ros tf2_echo map charging_dock_apriltag
```

4. Record the `translation (x, y, z)` and `rotation (quaternion)`.
5. Convert rotation to yaw (for 2D). The docking server uses `[x, y, yaw]` in `openamrobot_docking.yaml`.
6. Set `home_dock.pose` to the measured values.

Example (current config):
- `home_dock.pose: [2.15, 0.0, 0.0]`

## Option 2: Live detection pose
If `home_dock_plugin.use_external_detection_pose` is set to `true`, the docking server expects external detections. The provided pipeline publishes `detected_dock_pose` from TF.

When using live detection:
- Ensure `detected_dock_pose` is stable (avoid noisy detections).
- Ensure `parent_frame` in `docking_pose_publisher.yaml` matches the docking `fixed_frame` or map frame expected.

## Notes on placement
- Tag height matters. If the tag is too high/low relative to the camera, detection quality will degrade.
- Tag should be perpendicular to the camera view for best accuracy.
