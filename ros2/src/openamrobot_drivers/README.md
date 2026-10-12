# openamrobot_drivers

Host-side hardware driver layer for the **real** OpenAMRobot. Thin on purpose: the
real control loop (per-wheel PID, encoder reading, odometry, IMU) runs in the **Teensy
firmware** (`openamr-platform-fw`); this package only bridges it to ROS and starts the
sensor drivers.

## `drivers.launch.py`

Starts:
- the **micro-ROS agent** over USB serial — bridges the Teensy topics `/cmd_vel`,
  `/odom/unfiltered`, `/imu/data` (needs `micro_ros_agent` installed);
- the navigation **LiDAR** driver (Slamtec `sllidar_ros2`, node `sllidar_node`) → `/scan`
  in frame `lidar_link`. `lidar_model:=s3` is the RPLIDAR S3 (OpenAMRobot 2.0);
  `lidar_model:=a1` is the RPLIDAR A1, legacy (existing robot). Any other value is rejected.
  The default is `a1` until the S3 is physically mounted; the default flips to `s3` in the
  change that mounts the S3 (the `s3` settings are already prepared).
  `sllidar_ros2` has no Jazzy binary package; build it from source at the pinned commit
  (see [Pinned `sllidar_ros2` install](#pinned-sllidar_ros2-install)).

```bash
ros2 launch openamrobot_drivers drivers.launch.py
# other robot / different ports:
ros2 launch openamrobot_drivers drivers.launch.py teensy_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
# RPLIDAR S3 (OpenAMRobot 2.0), once mounted:
ros2 launch openamrobot_drivers drivers.launch.py lidar_model:=s3
```

### Pinned `sllidar_ros2` install

The LiDAR driver is Slamtec `sllidar_ros2` (BSD licence, not copied into this repository), pinned to
upstream commit `34300099fadfc772965962dec837bf436706188f` (main, 2024-06-17, "bugfix:set the minimum
distance to 5cm"). The `s3` and `a1` settings in `drivers.launch.py` were taken from this commit's
`launch/sllidar_s3_launch.py` and `launch/sllidar_a1_launch.py`. Build it once in its own workspace
and overlay it under this repository's workspace:

```bash
# 1. Check out the pinned commit
mkdir -p ~/sllidar_ws/src && cd ~/sllidar_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git -C sllidar_ros2 checkout 34300099fadfc772965962dec837bf436706188f

# 2. Build
cd ~/sllidar_ws
source /opt/ros/jazzy/setup.bash
rosdep install -y --rosdistro jazzy --ignore-src --from-paths src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Overlay: source it before building and running this repository's workspace
source ~/sllidar_ws/install/setup.bash
ros2 pkg executables sllidar_ros2   # expect: sllidar_ros2 sllidar_client, sllidar_ros2 sllidar_node
cd <this repository>/ros2
colcon build --symlink-install --packages-select openamrobot_drivers openamrobot_bringup
source install/setup.bash
```

The Docker quickstart (`Dockerfile`) copies only `ros2/src` and does not fetch `sllidar_ros2`, so the
image supports simulation only; `drivers.launch.py` and `real_bringup.launch.py` need the pinned install
above on the robot.

This repository has no `.repos` file. The workspace manifest entry for the same commit is proposed
in [openamrobot-manifest PR #11](https://github.com/openAMRobot/openamrobot-manifest/pull/11). CI job
`drivers-bringup-launch-tests` (`.github/workflows/drivers-bringup-tests.yml`) builds this pinned commit
on every change to the drivers or bringup packages.

### Launch arguments (unit-specific defaults)

| Argument | Default | Meaning |
|---|---|---|
| `teensy_port` | `/dev/serial/by-id/usb-Teensyduino_USB_Serial_16778200-if00` | Teensy serial (this unit's by-id path) |
| `lidar_port` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._if00-port0` | LiDAR USB to UART adapter (existing unit's CP2102 by-id path; to confirm on the S3 unit) |
| `lidar_model` | `a1` (flips to `s3` when the S3 is mounted) | `s3` = RPLIDAR S3 (1000000 baud, scan mode `DenseBoost`); `a1` = RPLIDAR A1, legacy (existing robot) (115200 baud, scan mode `Standard`) |

> The `by-id` paths are stable per device but **specific to this robot**; override them on
> another unit. micro-ROS link is 115200 baud (matches the firmware `BAUDRATE`).

This package is the **real** profile counterpart of `openamrobot_gazebo` (which, in
simulation, publishes the same `/odom`/`/scan`/`/imu` topics from Gazebo + `gz_bridge`).
`openamrobot_control` stays a placeholder: there is no host-side `ros2_control`, the
control lives in the firmware.
