# openamrobot_gazebo

ROS 2 Gazebo Harmonic simulation package for the **OpenAMRobot** mobile base.

Contains: world files, ROS–Gazebo bridge configuration, and simulation launch files.

> **Rule:** If a file must be used by `ros2 launch`, keep it inside `openamrobot_gazebo`.
> 
> **Rule:** If a file is a project-level scenario, design asset, experimental environment, or documentation/reference asset, keep it in `simulation/`.

## Contents

```
openamrobot_gazebo/
├── config/
│   └── gz_bridge.yaml          ← ROS–Gazebo topic bridge config
├── launch/
│   └── gz_simulator.launch.py  ← full Gazebo Harmonic simulation bringup
├── worlds/
│   └── walled_world.sdf        ← enclosed arena (default)
├── package.xml
└── setup.py
```

## Usage

Launch the full simulation stack with the default world:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Launch with any custom world file:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py world:=/path/to/your_world.sdf
```

This starts Gazebo Harmonic, spawns the robot, and brings up `robot_state_publisher`, `joint_state_publisher`, and the ROS–Gazebo bridge.

> **Note:** If your world references external meshes or resources, add their parent directory to `GZ_SIM_RESOURCE_PATH` before launching.

## ROS–Gazebo Bridge Topics

| ROS 2 Topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ → ROS |
| `/odom` | `nav_msgs/msg/Odometry` | GZ → ROS |
| `/tf` | `tf2_msgs/msg/TFMessage` | GZ → ROS |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → GZ |
| `/scan` | `sensor_msgs/msg/LaserScan` | GZ → ROS |
| `imu` | `sensor_msgs/msg/Imu` | GZ → ROS |
| `/rgb_image` | `sensor_msgs/msg/Image` | GZ → ROS |
| `camera/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ → ROS |

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `True` | All nodes use `/clock` from Gazebo |
| `use_robot_state_pub` | `True` | Start `robot_state_publisher` |
| `world` | `worlds/walled_world.sdf` | Full path to any Gazebo world file |
