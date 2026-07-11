# Overview — the real robot

What the physical OpenAMRobot is, the software contract it presents, and how the real
stack differs from the simulation. This is the entry point for the **real-robot
engineering + operations** series; for a first bring-up jump to
[`01_bringup.md`](01_bringup.md).

---

## What the real robot is

A differential-drive AMR with **two brains** (the split is deliberate — see
[`../architecture/ARCHITECTURE_OVERVIEW.md`](../architecture/ARCHITECTURE_OVERVIEW.md)):

| Brain | Hardware | Runs | Talks |
|---|---|---|---|
| **Real-time** | Teensy 4.0 firmware @ 50 Hz | encoder read, IMU read, motor PID | **micro-ROS** over USB serial |
| **High-level** | Raspberry Pi 5 (4× Cortex-A76, 8 GB), ROS 2 Jazzy | perception, localization, Nav2, docking | **CycloneDDS** on the LAN |

Keeping motion control on the Teensy means the wheels are never scheduled against vision
or planning — the thing that repeatedly bites us on the Pi (see
[`04_compute_and_thermal.md`](04_compute_and_thermal.md)) never touches the control loop.

### Hardware inventory

| Part | Detail | Port |
|---|---|---|
| Compute | Raspberry Pi 5 Model B Rev 1.1, 8 GB, Ubuntu Server 24.04, ROS 2 Jazzy | — |
| MCU | Teensy 4.0, OpenAMRobot motor-control firmware (see `openamr-platform-fw`) | `/dev/ttyACM0` (by-id: `usb-Teensyduino_USB_Serial_<serial>-if00`) |
| LiDAR | RPLIDAR A1 (2D, ~5.5–10 Hz), mounted **rotated 180°** | `/dev/ttyUSB0` (CP2102) |
| Camera | Sony **IMX708** (Pi Camera Module 3 **NoIR**), CSI, `camera_ros` | CSI ribbon |
| Drive | 2× BLDC Z4BLD60-24GN-30S (60 W, 30:1), 2× ZBLD.C20-120L2R drivers | — |
| Power | 2× 12 V lead-acid in series (24 V bus), 5 V buck to the Pi | — |

Hardware detail lives in the hardware repo: `openamr-platform-hw` →
`electrical/computing/raspberry-pi.md`, `electrical/computing/teensy.md`; firmware in
`openamr-platform-fw`.

### How you reach it

The Pi is **headless** (Ubuntu Server). Everything is done over SSH from an Ubuntu 24.04
dev PC, and RViz runs on the PC.

```bash
ssh <user>@<robot>.local
```

> ⚠️ **Always use the mDNS name `<robot>.local`, never a hard-coded IP.** The Pi's
> address is DHCP and *changes*. On 2026-07-06 the Pi board was swapped (same SSD) → new
> MAC → the old DHCP lease went dead and a new one was assigned. The mDNS name follows
> the SSD. Find the current IP with `getent hosts <robot>.local`. Full SSH recipe:
> memory `amr-pi-ros-commands` / `pi-ssh-access`.

---

## The ROS "contract" the real robot publishes

The real data-source bring-up (`real_bringup.launch.py`) exists so the **same** Nav2 +
docking stack that runs in simulation runs unchanged on hardware. It produces exactly the
topics/TF that Gazebo produces in sim:

```
/cmd_vel  /odom  /imu/data  /scan  /scan_filtered  /camera/image_raw  /camera/camera_info
TF: (map →) odom → base_link → {lidar_link, imu_link, base_footprint,
                                 camera_link → camera_optical_frame}
```

| Node (real) | Package | Provides |
|---|---|---|
| micro-ROS agent | `openamrobot_drivers` | `/cmd_vel`, `/odom/unfiltered`, `/imu/data`, `/debug/*` |
| `rplidar_composition` | `openamrobot_drivers` | `/scan` (frame `lidar_link`) |
| `scan_body_filter` | `openamrobot_perception` | `/scan_filtered` (robot body masked) |
| `ekf_node` (`robot_localization`) | `openamrobot_bringup` cfg | `/odom` + TF `odom→base_link` (wheels + IMU gyro-Z) |
| `camera` (`camera_ros`) | `openamrobot_perception` | `/camera/image_raw`, `/camera/camera_info` |
| 5× `static_transform_publisher` | `openamrobot_bringup` | the measured mount TFs for **this** unit |

Odometry is **EKF-fused** (wheel `vx`+`vyaw` + IMU **gyro-Z only** — the IMU orientation
quaternion is invalid and its accel is tilted, so only the yaw rate is used). See
[`../../ros2/src/openamrobot_bringup/README.md`](../../ros2/src/openamrobot_bringup/README.md)
for the fusion detail and the `scan_body_filter` sector masks.

---

## Real vs simulation — what actually differs

The design principle is that **only the data source and the clock change** between sim and
real (see [`../architecture/ARCHITECTURE_OVERVIEW.md`](../architecture/ARCHITECTURE_OVERVIEW.md)).
Everything else — the Nav2 stack, the `nav2_params.yaml`, the docking sequencer — is identical.

| Concern | Simulation | Real robot |
|---|---|---|
| Data source | Gazebo + `ros_gz_bridge` | `real_bringup.launch.py` (Teensy + LiDAR + camera + EKF) |
| Clock | `use_sim_time:=true` | `use_sim_time:=false` |
| Initial pose | AMCL `set_initial_pose:=true` at (0,0) (robot spawns at origin) | **disabled** — operator sets it with RViz **2D Pose Estimate** |
| Map | bundled `my_map.yaml` (walled world) | **required** explicit `map:=…` (no silent fallback) |
| AprilTag input | Gazebo `/rgb_image` | IMX708 `/camera/image_raw`, detector composed intra-process |
| The dock | modelled in the world | **printed 3-tag bundle** (no physical dock) |
| Compute headroom | a desktop | a Pi 5 that saturates / throttles — the recurring wall |
| Transport | loopback | **CycloneDDS over Wi-Fi** — a fragile link (see [`02_networking_and_dds.md`](02_networking_and_dds.md)) |

The real-robot default for anything with camera/docking is the **composed** profile
(`bringup_composed.launch.py`, intra-process/zero-copy vision); the plain
`bringup.launch.py sim:=true|false` is for nav-only debug (`use_camera:=false`), since it
routes the camera through a Python gate that starves the detector. Either enforces the
real-robot guards above (explicit map, initial-pose off). See
[`01_bringup.md`](01_bringup.md).

### What is *harder* on real hardware (the whole reason this series exists)

Sim never had these; the real robot's real work was almost all here:

1. **The network is Wi-Fi and fragile.** DDS discovery, mDNS, and big reliable topics
   (camera, costmaps) collapse a degraded guest link — and it *looks* like a robot fault.
   → [`02_networking_and_dds.md`](02_networking_and_dds.md).
2. **Vision is CPU-bound.** AprilTag on the Pi CPU (no GPU) fights Nav2 and the camera for
   cores. The whole docking-latency story lives here.
   → [`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md).
3. **The Pi has no cooler and thermally throttles** navigation under the full stack.
   → [`04_compute_and_thermal.md`](04_compute_and_thermal.md).
4. **Everything needs calibration** — encoder ripple (per Teensy power-cycle), camera
   intrinsics + extrinsics, the dock pose. → [`05_calibration.md`](05_calibration.md).
5. **Power is marginal** — a low 24 V bus or a 5 V supply that can't hold 5 A causes
   brown-outs that masquerade as software bugs. → [`07_troubleshooting.md`](07_troubleshooting.md).

---

## Where to go next

| If you want to… | Read |
|---|---|
| Bring the robot up and drive it | [`01_bringup.md`](01_bringup.md) |
| Understand / fix the network + DDS | [`02_networking_and_dds.md`](02_networking_and_dds.md) |
| Understand the vision pipeline & why docking was slow | [`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md) |
| Budget the CPU / deal with thermal throttling | [`04_compute_and_thermal.md`](04_compute_and_thermal.md) |
| Calibrate encoders / camera / dock | [`05_calibration.md`](05_calibration.md) |
| Run the operator UI | [`06_operator_ui.md`](06_operator_ui.md) |
| Diagnose an infrastructure failure | [`07_troubleshooting.md`](07_troubleshooting.md) |
