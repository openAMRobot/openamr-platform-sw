# Parameters Reference

This file summarizes the most important parameters in this repository. It is not exhaustive, but covers what students typically change.

## AprilTag parameters
File: `apriltag_ros/cfg/tags_36h11.yaml`
- `family`: Tag family (e.g., `36h11`)
- `size`: Global tag size in meters
- `max_hamming`: Error tolerance (0 = strict)
- `detector.threads`: Detection threads
- `detector.decimate`: Downsample ratio for speed
- `detector.refine`: Corner refinement
- `tag.ids`: List of tag IDs
- `tag.frames`: Frame name for each ID

## Dock pose publisher
File: `openamrobot_docking/config/docking_pose_publisher.yaml`
- `parent_frame`: Frame to publish pose in (default `map`)
- `child_frame`: Tag frame name (default `charging_dock_apriltag`)
- `output_topic`: Pose topic (default `detected_dock_pose`)
- `publish_rate`: Hz

## Dock trigger
File: `openamrobot_docking/config/dock_trigger.yaml`
- `trigger_topic`: Bool topic to start docking
- `use_dock_id`: Whether to use dock ID or dock type
- `dock_id`: Dock ID used for docking server database
- `navigate_to_staging_pose`: Ask docking server to navigate to staging pose
- `undock_on_false`: If true, Bool false triggers undock

## Docking server
File: `openamrobot_docking/config/openamrobot_docking.yaml`
General:
- `controller_frequency`: Control loop frequency
- `initial_perception_timeout`: Time to wait for detections
- `dock_approach_timeout`: Time to approach dock
- `max_retries`: Max docking attempts
- `base_frame`: Robot base frame (`base_link`)
- `fixed_frame`: Typically `odom` or `map`
- `dock_prestaging_tolerance`: Staging tolerance (meters)

Dock plugin (`home_dock_plugin`):
- `docking_threshold`: Distance tolerance at dock
- `staging_x_offset`: Backward offset from dock for staging
- `use_external_detection_pose`: Use `detected_dock_pose`
- `use_stall_detection`: Stop if stalled
- `external_detection_translation_*`: Transform offset between tag and dock frame
- `external_detection_rotation_*`: Orientation offsets

Dock database (`home_dock`):
- `frame`: Frame of the dock pose (usually `map`)
- `pose`: `[x, y, yaw]` in that frame

Controller:
- `dock_collision_threshold`: Stop distance for collision check
- `use_collision_detection`: Enable/disable collision detection
- `v_linear_min`, `v_linear_max`: Speed limits
