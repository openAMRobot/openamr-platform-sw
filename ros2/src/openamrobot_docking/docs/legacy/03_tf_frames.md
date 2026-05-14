# TF Frames and Static Transforms

This system requires a consistent TF chain so the AprilTag pose can be expressed in the `map` frame:

`map -> odom -> base_link -> camera_link -> camera_optical_frame -> charging_dock_apriltag`

The AprilTag node provides `camera_optical_frame -> charging_dock_apriltag`. You must provide the camera static transforms yourself.

## Required camera frames
Common naming used by ROS cameras:
- `camera_link`: Physical camera body frame
- `camera_optical_frame`: Optical frame (X right, Y down, Z forward)

If your camera publishes only `camera_link`, define `camera_optical_frame` as a static transform.

## Example static transforms
### base_link -> camera_link
Replace values with your measured mount position and orientation.

```bash
ros2 run tf2_ros static_transform_publisher \
  0.10 0.00 0.20 0 0 0 \
  base_link camera_link
```

### camera_link -> camera_optical_frame
A typical optical frame rotation from camera_link (REP-103):

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5708 0 -1.5708 \
  camera_link camera_optical_frame
```

Notes:
- The roll/pitch/yaw order is `r p y` in radians.
- If your camera driver already publishes `camera_optical_frame`, do not duplicate it.

## Verifying TF
Use TF tools to verify the chain:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map charging_dock_apriltag
```

You should see a valid transform from `map` to `charging_dock_apriltag` once the tag is visible and localization is running.
