# openamrobot_description

ROS 2 robot description for the **OpenAMRobot** mobile base: a differential-drive platform with four passive caster wheels, a 2D LiDAR, and an RGB camera.

This package defines **what the robot is** — its kinematics, masses, sensor placement, and the Gazebo plugins that turn the URDF into a physically simulated robot. It does **not** launch a simulator or a navigation stack; those live in [`openamrobot_gazebo`](../openamrobot_gazebo/) and [`openamrobot_nav2`](../openamrobot_nav2/).

---

## Contents

```
openamrobot_description/
├── launch/
│   └── launch.py                    standalone RViz visualizer (no simulator)
├── meshes/
│   ├── collision/                   simplified STLs for physics (cheap to collide)
│   └── visual/                      full-detail STLs for rendering
├── urdf/
│   ├── robo_urdf.urdf.xacro         main robot model — links, joints, inertias
│   ├── gazebo_control.xacro         Gazebo-side plugins: DiffDrive, JSP, lidar, camera, surface friction
│   └── robot.sdf                    legacy export, not used at runtime
├── package.xml
└── setup.py
```

---

## Usage

### Just visualize the URDF in RViz (no simulator)

```bash
ros2 launch openamrobot_description launch.py
```

Starts `robot_state_publisher`, `joint_state_publisher_gui` (sliders for the wheel joints), RViz, and two static TF publishers that anchor `map -> odom -> base_footprint` at the origin. Use this to inspect the URDF visually, check link transforms, or test changes to meshes without paying for Gazebo startup.

Available launch arguments:

| Argument | Default | Description |
|---|---|---|
| `use_robot_state_pub` | `True` | start `robot_state_publisher` |
| `use_joint_state_pub` | `True` | start `joint_state_publisher_gui` (the sliders) |
| `use_rviz` | `True` | open RViz with the description-package layout |
| `rviz_config_file` | `config/rviz.rviz` | full path to a custom RViz config |

> The two static TFs (`map -> odom` and `odom -> base_footprint`, both identity) are **only here for the standalone RViz visualizer**. They are **not** included by the simulator launches — there, Gazebo's DiffDrive plugin publishes `odom -> base_footprint` from physics, and SLAM Toolbox publishes `map -> odom`. Mixing these static publishers with a running simulator pins the robot to the origin and is a classic source of "the robot snaps back to (0,0)" bugs.

### Use the URDF from another launch (the normal case)

The URDF is consumed by `openamrobot_gazebo/launch/gz_simulator.launch.py` (which xacro-expands it and feeds it to `robot_state_publisher` and to `ros_gz_sim create`). You normally do not launch this package directly — you launch one of the bringup compositions and the URDF flows in automatically.

---

## Link tree

```
base_footprint                  [URDF root — ground projection of base_link]
└── base_link            (4.175 kg)        ← STL collision, main body
    ├── left_wheel_link  (1.061 kg)        ← cylinder collision r=0.11 m  (see below)
    ├── right_wheel_link (1.061 kg)        ← cylinder collision r=0.11 m
    ├── fl_caster_link / fr_caster_link / bl_caster_link / br_caster_link
    │       └── fl/fr/bl/br_wheel_link     ← sphere collision r=0.026 m, friction μ=0
    ├── lidar_link       (0.168 kg)        ← 2D GPU lidar, 0.15–12 m, 10 Hz, 360°
    └── camera_link                        ← visual frame
        └── camera_rgb_optical_frame       ← (R_x=-π/2, R_z=-π/2) for solvePnP convention
```

**Frame conventions** follow Nav2:

- `base_footprint` is the URDF root and sits at ground level. The `odom -> base_footprint` TF (published by Gazebo's DiffDrive plugin in simulation) is the robot's pose in the odometric frame.
- `base_joint` (fixed) lifts `base_link` by `0.053467 m`. With `wheel_radius = 0.10 m` and the wheel joint origin at `z = 0.046533 m` in `base_link`, the drive-wheel centres land at world `z = 0.10 m` exactly when `base_footprint` is at `z = 0`. **This is why simulator launches must spawn `base_footprint` at `z = 0`**, not at `z = 0.053467` — spawning higher leaves the wheels floating and the robot won't drive.
- `camera_rgb_optical_frame` carries the `rpy = (-π/2, 0, -π/2)` rotation relative to `camera_link` that the image-pinhole convention requires (X = right, Y = down, Z = into the scene). `apriltag_ros::solvePnP` outputs the tag pose in this frame, which is then chained with the rest of the TF tree to give the dock pose in `map`.

---

## Sensors

| Sensor | Position in `base_link` | Output topic (after bridge) | Plugin source |
|---|---|---|---|
| LiDAR (gpu_lidar) | `(0.35135, -0.001025, 0.1683)` | `/scan` (`sensor_msgs/LaserScan`) | `gazebo_control.xacro` |
| RGB camera | `(0.35, 0, 0.22)` | `/camera/image_raw` + `/camera/camera_info` | `gazebo_control.xacro` |
| IMU | `(0, 0, 0)` (configured per-world) | `/imu` (`sensor_msgs/Imu`) | world plugin |

**LiDAR:** 360°, range 0.15–12 m, 360 samples, 10 Hz, Gaussian noise σ = 0.001 m. The 0.15 m near limit is tight enough to see obstacles right next to the robot; the 12 m far limit spans the 10×10 m docking world without clipping the far wall. `gz_frame_id = lidar_link`.

**Camera:** 640×480 @ 15 Hz, `horizontal_fov = 1.2 rad`. Format `R8G8B8`. The image is bridged to `/camera/image_raw` with `frame_id = camera_rgb_optical_frame` so `apriltag_ros::solvePnP` already receives images in the canonical optical-frame orientation.

> **Why not HD?** 1280×720 @ 30 Hz stalls the ros↔gz image bridge down to ~0.2 Hz. The bridge serializes images on a single thread and HD frames exceed its budget. Keep 640×480 unless you also rewrite the image bridge in parallel mode.

---

## Wheel traction — why collision radius ≠ kinematic radius

The most non-obvious thing in this URDF: the wheel **collision cylinders** are `radius = 0.11 m`, while the `DiffDrive` plugin in [`gazebo_control.xacro`](urdf/gazebo_control.xacro) has `<wheel_radius>0.10</wheel_radius>`. **This 1 cm difference is intentional.**

Two unrelated parameters share the name "wheel radius":

1. **Kinematic wheel radius** (`DiffDrive plugin`, 0.10 m). The plugin computes wheel angular velocity from desired linear velocity: `ω = v / r`. This determines how fast the wheels spin for a commanded `cmd_vel`.
2. **Collision geometry radius** (`<cylinder radius="0.11">`). This is what ODE (the physics engine in Gazebo) uses to detect contact with the ground and compute friction forces.

`base_footprint` is at ground level (`z = 0`). With `base_joint` lifting `base_link` by 0.053467 m, the *centre* of each wheel ends up at `z = 0.10 m`. If the collision cylinder is also 0.10 m, the bottom of the wheel just **touches** the ground — ODE detects a zero-depth contact and computes a zero normal force. With zero normal force, friction is `μ × 0 = 0`. The wheel spins, but there's nothing transferring torque to translation. **The robot doesn't move.**

With the cylinder at 0.11 m, the bottom of the wheel is at `z = -0.01 m` (1 cm into the ground plane). ODE detects 1 cm of penetration, applies a contact spring force using the link's `<kp>` and `<kd>` (configured in `gazebo_control.xacro`), and the resulting normal force gives a non-zero friction `F = μ × N`. **The robot moves.**

This trick is used by most simulated diff-drive robots (e.g. TurtleBot3) — kinematic radius for control math, slightly larger collision radius for traction.

---

## Inertia normalisation

The two drive wheels were exported from SolidWorks with tiny `1e-8`-level asymmetries between left and right (`ixy`, `ixz`, `iyz` not exactly zero, and the diagonal terms slightly different between the two wheels). In Gazebo this asymmetry made the robot curve when commanded straight — invisible at the math level, very visible in 10 m of straight-line driving.

The current URDF zeroes off-diagonals and uses identical diagonal terms for the two wheels:

```xml
<inertia ixx="0.003398197" ixy="0" ixz="0"
         iyy="0.006426349" iyz="0"
         izz="0.003397389" />
```

If you re-import the URDF from a SolidWorks export, re-apply this symmetrisation.

---

## Gazebo plugins shipped with the description

Defined in [`urdf/gazebo_control.xacro`](urdf/gazebo_control.xacro), included from `robo_urdf.urdf.xacro`:

| Plugin | Purpose | Topic in gz | Bridged to ROS |
|---|---|---|---|
| `gz-sim-diff-drive-system` | reads `cmd_vel`, applies wheel torques, publishes odom + tf | `cmd_vel`, `odom`, `tf` | yes (see `openamrobot_gazebo/config/gz_bridge.yaml`) |
| `gz-sim-joint-state-publisher-system` | publishes wheel joint positions | `joint_states` | yes |
| `gpu_lidar` sensor on `lidar_link` | 2D laser scan | `scan` | yes (`/scan`) |
| `camera` sensor on `camera_link` | RGB image + camera_info | `camera/image_raw` | yes |

Surface friction is set per-link in the same file:

- Drive wheels: `μ = 1.5` in rolling direction, `μ2 = 0.5` lateral, low slip (`slip1 = slip2 = 0.001`), with `fdir1 = (1, 0, 0)` so the friction is anisotropic in the wheel-aligned frame.
- Caster wheels: `μ = μ2 = 0` — the casters carry weight but no traction, so the drive wheels do all the work.
- `base_link`: `μ = μ2 = 0.2`, used only if the body itself contacts something.

---

## Changes from the upstream SolidWorks export

Documented here so the next maintainer doesn't redo the same surgery:

1. Added `base_footprint` link + `base_joint` for the Nav2 TF convention.
2. Wheel inertials made symmetric (see [Inertia normalisation](#inertia-normalisation) above).
3. Wheel collision cylinder kept at `0.11 m` against the plugin's kinematic `0.10 m` for ODE traction (see [Wheel traction](#wheel-traction--why-collision-radius--kinematic-radius) above).
4. Added `camera_link` and `camera_rgb_optical_frame` for AprilTag detection.
5. LiDAR range tuned to `0.15–12 m` for the docking world.

Attribution chain (Brawner → Dhakal → Indulkar → this revision) is in the top-level [`NOTICE.md`](../../../NOTICE.md).
