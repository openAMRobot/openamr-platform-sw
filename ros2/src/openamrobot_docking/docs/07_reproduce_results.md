# Reproducing Results

This is a step-by-step recipe for students to reproduce docking results in this workspace.

## 1. Verify camera and TF
- Camera driver is running and publishes images.
- `base_link -> camera_link` static transform is correct.
- `camera_link -> camera_optical_frame` exists.
- Localization is running so `map -> odom -> base_link` exists.

Quick checks:
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
ros2 run tf2_ros tf2_echo map base_link
```

## 2. Run AprilTag detection
Launch the camera + AprilTag pipeline:
```bash
ros2 launch apriltag_ros camera_36h11.launch.yml
```

Verify tag TF exists when the tag is visible:
```bash
ros2 run tf2_ros tf2_echo camera_optical_frame charging_dock_apriltag
```

## 3. Run docking nodes
Launch docking and pose publisher:
```bash
ros2 launch openamrobot_docking openamrobot_docking.launch.py
```

Check that `detected_dock_pose` is published:
```bash
ros2 topic echo /detected_dock_pose --once
```

## 4. Trigger docking
Publish a Bool `true` to start docking:
```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

If `undock_on_false` is enabled, publish `false` to undock.

## 5. Document results
Record:
- Tag ID and size
- Camera model and calibration
- Dock pose in map (`home_dock.pose`)
- Any parameter changes

This makes results reproducible for the next student.
