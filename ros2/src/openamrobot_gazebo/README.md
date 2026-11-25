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
| `/sim/base_cmd_vel` → gz `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → GZ (from `sim_cmd_vel_timeout`, see below) |
| `/scan` | `sensor_msgs/msg/LaserScan` | GZ → ROS |
| `/imu` | `sensor_msgs/msg/Imu` | GZ → ROS |
| `/rgb_image` | `sensor_msgs/msg/Image` | GZ → ROS |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ → ROS |

## Command timeout of the simulated base (#79)

Gazebo's DiffDrive system keeps the last velocity command forever, so a dead `/cmd_vel` publisher
used to leave the simulated base driving. The real base controllers stop instead: the Teensy
firmware after 200 ms without a fresh `/cmd_vel`, the Build 1 STM32 by replacing a `/cmd_vel` older
than 200 ms with zero, non-latching (platform-fw `docs/safety/motion-safety.md`, platform-fw #17).

`gz_simulator.launch.py` therefore starts `sim_cmd_vel_timeout` between `/cmd_vel` and the bridge:

```
/cmd_vel -> sim_cmd_vel_timeout -> /sim/base_cmd_vel -> ros_gz_bridge -> gz /cmd_vel -> DiffDrive
```

- It forwards every command unchanged and, once no command has arrived for `timeout_s` (0.2 s,
  `config/sim_cmd_vel_timeout.yaml`), publishes zero at once and repeats it every
  `zero_repeat_period_s` (0.1 s) until the next command arrives. One zero already stops DiffDrive;
  the repeat is for a bridge that was not running when the timeout fired. A TRANSIENT_LOCAL
  publisher does not replace it: `ros_gz_bridge` 1.0.24 subscribes volatile, so a bridge started
  late does not receive the retained sample (measured on 22 Nov 2026).
- It runs on simulation time, so it behaves the same at any real-time factor. Before the first
  `/clock` message the node clock reads 0 and nothing times out; a command received then is held
  at zero once the clock starts. A clock that goes backwards (a simulation reset) also stops the
  base, and the log says so.
- Every timeout is logged ("holding the simulated base at zero"), so a trace shows whether the
  command chain sent its own zero or the timeout had to stop the base.
- `/sim/base_cmd_vel` is internal to this package: only this node publishes it, only the bridge
  reads it. It is not a platform interface; nothing else may use it.
- After the zero, DiffDrive slows down at its own acceleration limit (`gazebo_control.xacro`); it
  does not brake like the Teensy's full stop.

Simulation only. It is not the firmware's command watchdog and not safety evidence.

Tests (`test/`): `test_cmd_vel_timeout.py` (the decision logic, no ROS),
`test_sim_cmd_vel_timeout_node.py` (the node on a simulated clock, no Gazebo) and
`test_sim_cmd_vel_timeout_gazebo.py` (headless Gazebo: the base stops after its `/cmd_vel`
publisher is SIGKILLed, measured from Gazebo's world pose and from what reaches
`/sim/base_cmd_vel`; skips without `gz` unless
`OPENAMR_REQUIRE_GZ=1`).

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `True` | All nodes use `/clock` from Gazebo |
| `use_robot_state_pub` | `True` | Start `robot_state_publisher` |
| `world` | `worlds/walled_world.sdf` | Full path to any Gazebo world file |
| `gui` | `false` | Start Gazebo GUI; default `false` keeps simulation timing stable |
