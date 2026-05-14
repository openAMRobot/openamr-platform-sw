# AprilTag ROS Configuration

This workspace uses `apriltag_ros` with a camera pipeline that rectifies images and detects tags from the `36h11` family.

**Relevant files:**
- `apriltag_ros/launch/camera_36h11.launch.yml`
- `apriltag_ros/cfg/tags_36h11.yaml`

## Launch pipeline
The launch file creates a component container with three composable nodes:
1. Camera driver: `camera_ros` (`camera::CameraNode`)
2. Rectification: `image_proc` (`image_proc::RectifyNode`)
3. Tag detection: `apriltag_ros` (`AprilTagNode`)

**Inputs/outputs (current remaps):**
- Rectify node subscribes to:
  - `image` -> `/rgb_camera/image_raw`
  - `camera_info` -> `/rgb_camera/camera_info`
- AprilTag node subscribes to:
  - `/apriltag/image_rect` -> `/camera/image_rect`
  - `/camera/camera_info` -> `/rgb_camera/camera_info`

If your camera topics differ, update the remaps in `apriltag_ros/launch/camera_36h11.launch.yml`.

## Tag configuration (`tags_36h11.yaml`)
Key parameters:
- `family`: Tag family. Here it is `36h11`.
- `size`: Edge length in meters. Default is `0.08`.
- `tag.ids`: List of tag IDs to detect.
- `tag.frames`: Frame name for each tag ID.

Example (current):
- Detect only tag ID `0`
- Publish TF frame name `charging_dock_apriltag`

If you print different tags or change size:
1. Update `tag.ids`
2. Update `tag.frames` to match new TF names
3. Update `size` or `tag.sizes` if a specific tag has a different size

## What the node publishes
The AprilTag node publishes:
- Tag detection messages (pose and ID)
- TF transforms between the camera optical frame and the tag frame

These TFs are used downstream by the docking pipeline to compute the tag pose in `map` or `odom`.
