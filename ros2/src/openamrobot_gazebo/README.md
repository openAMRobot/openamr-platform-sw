# openamrobot_gazebo

Gazebo Harmonic simulation bringup for **OpenAMRobot**: launches the simulator, spawns the robot using the URDF from [`openamrobot_description`](../openamrobot_description/), and wires the ROS↔Gazebo topic bridge.

This package is **scenario-agnostic**: it ships a generic walled arena and a depot world, but the docking-specific world + dock model live in [`openamrobot_docking`](../openamrobot_docking/). The launch is composable — higher-level launches (`openamrobot_bringup/sim_nav`, `openamrobot_docking/docking_sim`) include `gz_simulator.launch.py` from here and override the world.

---

## Contents

```
openamrobot_gazebo/
├── config/
│   └── gz_bridge.yaml              ROS<->Gazebo topic bridge config (parameter_bridge format)
├── launch/
│   └── gz_simulator.launch.py      Gazebo + URDF + bridge + robot_state_publisher
├── worlds/
│   ├── walled_world.sdf            empty 10x10 m walled arena (default world)
│   └── depot.sdf                   warehouse / depot environment
├── package.xml
└── setup.py
```

---

## Usage

### Standalone — just simulate the robot in an empty arena

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py
```

Equivalent to `openamrobot_bringup sim_only.launch.py` (which just wraps this).

Drive the robot manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The teleop publishes `/cmd_vel`, the bridge sends it to Gazebo, the DiffDrive plugin moves the robot.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `''` (= `walled_world.sdf`) | full path to a `.sdf` world file |
| `spawn_x` | `0.0` | robot spawn X in the world frame (m) |
| `spawn_y` | `0.0` | robot spawn Y in the world frame (m) |
| `spawn_yaw` | `0.0` | robot spawn yaw in the world frame (rad) |
| `use_sim_time` | `True` | drive ROS time from Gazebo's `/clock` |

Example with a custom world and a non-origin spawn:

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py \
    world:=/abs/path/to/my.sdf \
    spawn_x:=-2.0 spawn_y:=1.0 spawn_yaw:=1.5707
```

### Composed by other launches

Higher-level launches include this one and just pass a different `world`:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('openamrobot_gazebo'),
                     'launch', 'gz_simulator.launch.py'),
    ),
    launch_arguments={'world': scenario_world, 'spawn_x': '-4', ...}.items(),
)
```

`openamrobot_bringup/sim_nav.launch.py` does this with the default world, `openamrobot_docking/docking_sim.launch.py` does it with `docking_scenario.sdf`.

---

## What `gz_simulator.launch.py` does, step by step

1. **Resolves the world path.** Empty `world` argument → defaults to `walled_world.sdf` shipped in this package.
2. **Pre-computes `robot_description`** by running `xacro` on `openamrobot_description/urdf/robo_urdf.urdf.xacro`. The xacro pulls in `gazebo_control.xacro` (DiffDrive, JSP, lidar, camera plugins).
3. **Appends `GZ_SIM_RESOURCE_PATH`** with this package's `worlds/` directory and the parent directory of `openamrobot_description` — so SDF `<include>` directives can find `model://<...>` references (the docking launch adds the docking package's `models/` on top).
4. **Starts `robot_state_publisher`** with the xacro-expanded URDF on parameter `robot_description`. It publishes the static portion of the TF tree (`base_footprint -> base_link -> ...`) from URDF + joint_states.
5. **Starts the `ros_gz_bridge` parameter_bridge** with the YAML from `config/gz_bridge.yaml`. Includes `qos_overrides./tf_static.publisher.durability: transient_local` so RViz late subscribers receive `/tf_static`.
6. **Includes `ros_gz_sim/gz_sim.launch.py`** with `gz_args = -r -v 4 <world>` — that's the gz-sim launch upstream, started in run mode (`-r`) with verbose level 4.
7. **Schedules a `TimerAction` of 2 s** that spawns the robot via `ros_gz_sim create`, using `-topic /robot_description -x ... -y ... -z 0.0 -Y ...`. The 2 s delay is needed because `create` fires before the gz-sim server finishes loading the world, which silently drops the entity (or worse, spawns it disconnected from physics).

> **Spawn z = 0** is mandatory. `base_footprint` is the URDF root and sits at ground level by construction. Spawning at any other `z` lifts the wheels off the ground and the robot won't drive (no contact = no friction). See [openamrobot_description/README.md#wheel-traction](../openamrobot_description/README.md#wheel-traction--why-collision-radius--kinematic-radius) for the contact mechanics.

---

## ROS↔Gazebo bridge — full topic table

Configured in [`config/gz_bridge.yaml`](config/gz_bridge.yaml). Every topic name is **relative** (no leading slash) so the bridge behaves correctly if the robot is later spawned inside a ROS namespace; absolute paths are used only for simulator globals (`/clock`).

| ROS 2 Topic | Gazebo Topic | ROS Type | gz Type | Direction |
|---|---|---|---|---|
| `/clock` | `clock` | `rosgraph_msgs/Clock` | `gz.msgs.Clock` | GZ → ROS |
| `odom` | `odom` | `nav_msgs/Odometry` | `gz.msgs.Odometry` | GZ → ROS |
| `tf` | `tf` | `tf2_msgs/TFMessage` | `gz.msgs.Pose_V` | GZ → ROS |
| `cmd_vel` | `cmd_vel` | `geometry_msgs/Twist` | `gz.msgs.Twist` | **ROS → GZ** |
| `joint_states` | `joint_states` | `sensor_msgs/JointState` | `gz.msgs.Model` | GZ → ROS |
| `imu` | `imu` | `sensor_msgs/Imu` | `gz.msgs.IMU` | GZ → ROS |
| `scan` | `scan` | `sensor_msgs/LaserScan` | `gz.msgs.LaserScan` | GZ → ROS |
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/Image` | `gz.msgs.Image` | GZ → ROS |
| `/camera/camera_info` | `/camera/camera_info` | `sensor_msgs/CameraInfo` | `gz.msgs.CameraInfo` | GZ → ROS |

**`/cmd_vel` is the only ROS→GZ direction.** Everything else flows out of the simulator. The plugin advertising each gz topic comes either from the URDF (`gazebo_control.xacro` — DiffDrive, JSP, lidar, camera) or from the world (`gz-sim-physics-system` for `/clock`, IMU plugin if enabled).

### Verifying the bridge at runtime

If a topic seems missing on the ROS side:

```bash
ros2 topic list                 # should list everything in the table
gz topic -l | grep cmd_vel      # the gz-side topic must exist
gz topic -i -t /cmd_vel         # check it has a publisher (the bridge) and a subscriber (DiffDrive)
```

`parameter_bridge` only forwards messages between a single (ros, gz, type) tuple. If gz publishes on `/model/<modelname>/cmd_vel` (some plugin configs do that), the bridge listening on `/cmd_vel` will not see it. The DiffDrive plugin in this repo uses `<topic>cmd_vel</topic>` (a top-level gz topic), which is why the bridge config simply lists `cmd_vel`.

---

## Worlds

### `walled_world.sdf` (default)

10×10 m empty room with four walls at world ±5 m. Ground plane, sun, ambient light, IMU plugin attached to the world. Default robot spawn at the origin. Use for:

- Teleop validation
- Controller tuning
- Mapping experiments (open space)

### `depot.sdf`

Warehouse / depot environment. Larger, with structure. Use for navigation experiments in a more cluttered space.

### Docking-specific world

The 10×10 m room with the AprilTag dock against the north wall lives in `openamrobot_docking/worlds/docking_scenario.sdf`. The docking launch passes it via `world:=` rather than shipping it here, so this package stays scenario-agnostic.

---

## Common pitfalls

### Do not add a standalone `joint_state_publisher`

Gazebo's `gz-sim-joint-state-publisher-system` (loaded from `gazebo_control.xacro`) already publishes `/joint_states` with the real wheel positions, via the bridge. If you **also** start a standalone `joint_state_publisher` node, both publish to `/joint_states` — the standalone one publishes zeros for the continuous wheel joints, the Gazebo plugin publishes real angles, and `robot_state_publisher` alternates between the two depending on message order. Result: visible wheel-TF flicker in RViz and "the wheels keep snapping back to their start position" debugging frustration.

This launch deliberately does **not** start a standalone `joint_state_publisher`. If you copy this launch as a template, do not add one back.

(The standalone visualizer in `openamrobot_description/launch/launch.py` does start `joint_state_publisher_gui` — that's fine because no Gazebo plugin is publishing at the same time. The conflict is specifically about running both in the simulator launch.)

### Spawn z must be 0.0

`base_footprint` is at ground level by construction. Spawning at any non-zero `z` leaves the wheels floating above the ground plane and the robot will not drive — see [openamrobot_description/README.md#wheel-traction](../openamrobot_description/README.md#wheel-traction--why-collision-radius--kinematic-radius).

### After Ctrl-C, kill zombies before relaunching

`gz sim` does not always clean up on Ctrl-C. If a relaunch fails with `Address already in use`, `gz transport` errors, or a stuck RViz, run the script from the repo root:

```bash
bash scripts/kill_sim.sh
```

It SIGKILLs everything matching gz-sim, parameter_bridge, slam_toolbox, dock_trigger, rviz, etc.

---

## What this package does NOT contain

| Concern | Lives in |
|---|---|
| Robot URDF / meshes / Gazebo plugin definitions | `openamrobot_description` |
| Nav2 stack, SLAM Toolbox, RViz nav layout | `openamrobot_nav2` |
| Docking world (AprilTag dock), 4-phase sequencer | `openamrobot_docking` |
| Top-level launch compositions (sim_only, sim_nav) | `openamrobot_bringup` |
