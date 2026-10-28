# openamrobot_description

ROS 2 robot description package for the **OpenAMRobot** mobile base — differential-drive platform with four passive caster wheels and a 2D LiDAR sensor.

Contains: URDF/Xacro model, STL meshes, Gazebo plugins (in xacro), and RViz visualization launch.

## Contents

```
openamrobot_description/
├── launch/
│   └── launch.py               ← RViz + robot_state_publisher (no simulator)
├── meshes/
│   ├── collision/              ← legacy SolidWorks STLs (not used by the URDF; robot.sdf only)
│   ├── visual/                 ← 2.0 base visual meshes, generated (see below)
│   └── PROVENANCE.yaml         ← inputs, versions, parameters and classes of the visual meshes
├── urdf/
│   ├── robo_urdf.urdf.xacro    ← main robot model (includes camera_link + camera_optical_frame)
│   ├── gazebo_control.xacro    ← Gazebo plugins (diff-drive, LiDAR, RGB camera) + friction
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

## Visual meshes

The visual meshes are generated from the hardware CAD by `tools/cad/step_to_description_meshes.py`
(see `tools/cad/README.md`); do not edit them by hand. The chassis comes from the v0.2
full-assembly STEP of `openamr-platform-hw` (Rev B panels, 2.0 sensor mounts). Release 001 parts
in the STEP are excluded, and the 2.0 components of BOM Issue 7.11 are simplified envelopes:

| Mesh | Link | Content |
|---|---|---|
| `base_chassis.stl` | `base_link` | base plate, panels, supports, brackets, LiDAR support and spacer, sensor and camera mounts, motor driver mount, STM32 click mount plate |
| `base_covers.stl` | `base_link` | plastic cover and flanges |
| `base_equipment.stl` | `base_link` | ZLAC8015D, Jetson Orin NX on J401, NUCLEO-H723ZG (as in the CAD), battery cell-group envelope on the rear deck (314 x 140 x 210 mm, front face 2 mm behind the centre bracket, computed by the converter; provisional, dimensions not released), base E-stop and RESET in the release 001 button holes (placeholders) |
| `hub_motor_wheel.stl` | `left_wheel_link`, `right_wheel_link` (yaw pi) | ZLLG80ASM250-L-B hub motor wheel, brake side inboard |
| `caster.stl` | `fl/fr/bl/br_caster_link` | LPA-VSTH 35K castor with its wheel |
| `lidar.stl`, `base_camera.stl`, `ultrasonic.stl`, `tof.stl`, `imu.stl` | sensor links | RPLIDAR S3, Gemini 336L, MB7060, TF-Luna, ICM-42688-P board; the pose is the frame |

The base E-stop (XB4BS8445) and RESET button (XB4BW36B5) are in `base_equipment.stl`, centred on the
axes of the release 001 red and green button holes in the front panel (P-03 rev20.0 item 24); their
geometry is a placeholder from datasheet dimensions. Not modelled (no position in P-03 rev19.7 or
E-01): safety relay, contactors, main fuse and disconnect, wireless charging receiver. Collision geometry, inertials
and frames are unchanged. The converter also writes `build/base_visual.glb` for the docs roadmap
pages, one node per CAD part or component envelope with glTF extras (id, name, class, source,
BOM 7.11 row, provisional); `meshes/PROVENANCE.yaml` lists the nodes under `glb.nodes`. Test: `python3 -m pytest test/test_description_meshes.py`.

## Robot Model

| Link | Collision Geometry |
|---|---|
| `base_link` (4.175 kg) | STL mesh |
| `left_wheel_link` / `right_wheel_link` (1.061 kg) | Cylinder r=0.11 m |
| `fl/fr/bl/br_caster_link` (0.022 kg) | STL mesh |
| `fl/fr/bl/br_wheel_link` (0.024–0.027 kg) | Sphere r=0.026 m |
| `lidar_link` (0.168 kg) | STL mesh |
| `camera_link` (0.05 kg) | Box 0.04×0.08×0.04 m |
| `camera_optical_frame` | — (optical convention frame) |

**Sensor — LiDAR:** 2D GPU LiDAR — 360°, 10 m max range, 10 Hz, Gaussian noise σ=0.001 m.

**Sensor — Camera:** Forward-facing RGB camera mounted at `xyz="0.35 0 0.20"` on `base_link` — 640×480, 30 Hz, 60° HFOV, 0.05–15 m range, topic `/rgb_image`, Gaussian noise σ=0.005. Frame `camera_optical_frame` follows ROS optical convention.
