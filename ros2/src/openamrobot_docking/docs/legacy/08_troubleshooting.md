# Troubleshooting

## No tag detected
- Confirm tag family and size in `apriltag_ros/cfg/tags_36h11.yaml`.
- Ensure correct topic remaps in `apriltag_ros/launch/camera_36h11.launch.yml`.
- Make sure lighting and camera focus are adequate.

## Tag detected but no docking pose
- Check TF chain from `map` to `charging_dock_apriltag`.
- Verify `parent_frame` and `child_frame` in `docking_pose_publisher.yaml`.
- Confirm localization is running (`map -> odom -> base_link`).

## Docking server does not move
- Ensure `opennav_docking` node is running and active.
- Check action servers `dock_robot` and `undock_robot` exist.
- Verify `use_external_detection_pose` and `home_dock.pose` are consistent.

## Docking stops too early / overshoots
- Tune `docking_threshold`, `dock_collision_threshold`, and `v_linear_max`.
- Confirm `external_detection_translation_*` offsets align with your tag placement.

## No response to dock trigger
- Confirm `dock_trigger.py` is running.
- Check `dock_trigger` topic name and message type.
- Confirm `use_dock_id` and `dock_id` match the docking server database.
