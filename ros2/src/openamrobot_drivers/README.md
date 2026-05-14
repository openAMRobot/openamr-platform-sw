# openamrobot_drivers

> **🚧 STUB package — not yet implemented.** This README captures the intended scope so future work has a clear landing spot. The directory currently builds cleanly with `colcon build` but ships no nodes, launches, or configs.

## Intended scope

Hardware drivers for the **real** OpenAMRobot. In simulation, sensors and actuators come from Gazebo plugins shipped in `openamrobot_description`/`gazebo_control.xacro`; this package is what replaces them on the physical robot.

## Expected contents (when populated)

```
openamrobot_drivers/
├── config/
│   ├── lidar.yaml                   serial port, baud, frame_id, scan rate
│   ├── camera.yaml                  camera_ros device, intrinsics path
│   ├── imu.yaml                     IMU port + frame
│   └── motor_controller.yaml        wheel controller bridge config
├── launch/
│   ├── lidar.launch.py
│   ├── camera.launch.py             starts camera_ros + image_proc rectification
│   ├── imu.launch.py
│   ├── motors.launch.py             starts the motor controller bridge
│   └── all_drivers.launch.py        bringup of every driver above
├── urdf/                            (if any sensor adds URDF fragments)
└── README.md
```

## Topics expected to be provided by this package

Matching the topic names already consumed by `openamrobot_nav2` and `openamrobot_docking` in simulation, so the rest of the stack ports verbatim:

| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | published by lidar driver |
| `/camera/image_raw` + `/camera/image_rect` | `sensor_msgs/Image` | published by `camera_ros` + `image_proc` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | published by `camera_ros` |
| `/imu` | `sensor_msgs/Imu` | published by IMU driver |
| `/odom` | `nav_msgs/Odometry` | published by motor controller bridge |
| `/tf` (`odom -> base_footprint`) | `tf2_msgs/TFMessage` | published by motor controller bridge |
| `/cmd_vel` | `geometry_msgs/Twist` | **consumed** — drives the motors |

If you keep these names identical to the simulation, the Nav2 + docking stack ports over with **zero topic remapping** in the launches.

## Likely dependencies

To be added when implementation starts:

- a lidar driver matching the hardware (rplidar_ros, sllidar_ros, ldlidar_node, …)
- `camera_ros` + `image_proc`
- an IMU driver (vendor-specific)
- a motor-controller bridge (often custom or vendor-supplied)
- `tf2_ros`

## Why a stub now

The top-level [`README.md`](../../../README.md) advertises this package as part of the platform. A scaffolded stub:

- makes the package show up in `ros2 pkg list`,
- gives CI a place to verify it builds,
- makes the future scope discoverable to contributors before any code lands.

Until real content is added, **do not depend on this package from other launches**.
