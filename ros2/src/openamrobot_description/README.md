# openamrobot_description

ROS 2 robot description package for the **OpenAMRobot** mobile base — differential-drive platform with four passive caster wheels, a 2D LiDAR, and an RGB camera.

Contains: URDF/Xacro model, STL meshes, Gazebo plugins (in xacro), and RViz visualization launch.

## Contents

```
openamrobot_description/
├── launch/
│   └── launch.py               ← RViz + robot_state_publisher (no simulator)
├── meshes/
│   ├── collision/              ← simplified STL meshes for physics
│   └── visual/                 ← full-detail STL meshes for rendering
├── urdf/
│   ├── robo_urdf.urdf.xacro    ← main robot model (SolidWorks URDF export + corrections)
│   ├── gazebo_control.xacro    ← Gazebo plugins + surface friction properties
│   └── robot.sdf
├── package.xml
└── setup.py
```

## Usage

Visualize the robot in RViz with interactive joint sliders:

```bash
ros2 launch openamrobot_description launch.py
```

For Gazebo simulation, see the `openamrobot_gazebo` package.

## Robot Model

| Link | Collision Geometry |
|---|---|
| `base_footprint` | ground-projection of `base_link` (Nav2 TF convention) |
| `base_link` (4.175 kg) | STL mesh |
| `left_wheel_link` / `right_wheel_link` (1.061 kg) | Cylinder r=0.10 m |
| `fl/fr/bl/br_caster_link` (0.022 kg) | STL mesh |
| `fl/fr/bl/br_wheel_link` (0.024–0.027 kg) | Sphere r=0.026 m |
| `lidar_link` (0.168 kg) | STL mesh |
| `camera_link` | massless visual-only frame |
| `camera_rgb_optical_frame` | optical-image convention (X right, Y down, Z forward) |

**LiDAR:** 2D GPU lidar — 360°, 0.15–12 m range, 10 Hz, Gaussian noise σ=0.001 m.

**Camera:** RGB pinhole — 640×480 @ 15 Hz, `horizontal_fov=1.2 rad`. Image bridged to ROS on `/camera/image_raw` with `frame_id=camera_rgb_optical_frame` so `apriltag_ros::solvePnP` receives the canonical optical-frame pose.

## Changes from the upstream SolidWorks export

Applied so the same description works in both standalone RViz and the docking simulation:

- `base_footprint` root + `base_joint` lifts `base_link` by `0.053467 m`, so drive-wheel centres sit at `z = wheel_radius = 0.10 m` when `base_footprint` is at ground level.
- Left/right wheel inertials made symmetric (off-diagonals zeroed, components averaged). The SolidWorks export had 1e-8-level asymmetries that caused the robot to curve when commanded straight in Gazebo.
- `wheel_radius` reconciled to **0.10 m** between the DiffDrive plugin and the cylinder collisions (the upstream had `0.11` in the plugin and `0.10` in the collisions).
- `camera_link` and `camera_rgb_optical_frame` added — see `gazebo_control.xacro` for the matching sensor plugin.

See top-level `NOTICE.md` for the attribution chain (Brawner → Dhakal → Indulkar → this revision).
