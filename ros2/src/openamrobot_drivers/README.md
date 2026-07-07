# openamrobot_drivers

Host-side hardware driver layer for the **real** OpenAMRobot. Thin on purpose: the
real control loop (per-wheel PID, encoder reading, odometry, IMU) runs in the **Teensy
firmware** (`openamr-platform-fw`); this package only bridges it to ROS and starts the
sensor drivers.

## `drivers.launch.py`

Starts:
- the **micro-ROS agent** over USB serial — bridges the Teensy topics `/cmd_vel`,
  `/odom/unfiltered`, `/imu/data` (needs `micro_ros_agent` installed);
- the **RPLIDAR** driver (`rplidar_ros`) → `/scan` in frame `lidar_link`.

```bash
ros2 launch openamrobot_drivers drivers.launch.py
# other robot / different ports:
ros2 launch openamrobot_drivers drivers.launch.py teensy_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
```

### Launch arguments (unit-specific defaults)

| Argument | Default | Meaning |
|---|---|---|
| `teensy_port` | `/dev/serial/by-id/usb-Teensyduino_USB_Serial_16778200-if00` | Teensy serial (this unit's by-id path) |
| `lidar_port` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._if00-port0` | RPLIDAR serial (this unit's by-id path) |

> The `by-id` paths are stable per device but **specific to this robot**; override them on
> another unit. micro-ROS link is 115200 baud (matches the firmware `BAUDRATE`).

This package is the **real** profile counterpart of `openamrobot_gazebo` (which, in
simulation, publishes the same `/odom`/`/scan`/`/imu` topics from Gazebo + `gz_bridge`).
`openamrobot_control` stays a placeholder: there is no host-side `ros2_control`, the
control lives in the firmware.
