# Calibration

The real robot needs four things calibrated that simulation gave for free: the **encoder
ripple**, the **camera intrinsics**, the **camera extrinsics** (mount TF), and the **dock
pose**. This doc is the operational index — it summarizes what and when, and points to the
authoritative firmware / docking docs rather than duplicating them.

Cross-refs: firmware — `openamr-platform-fw/docs/architecture/encoder-calibration.md`;
camera intrinsics — [`../../ros2/src/openamrobot_docking/docs/06_camera_calibration.md`](../../ros2/src/openamrobot_docking/docs/06_camera_calibration.md);
velocity floors / control — memory `amr-min-velocity-floors`, `amr-pid-tuning`.

---

## What needs calibrating, and how often

| Item | Cadence | Where it lives | Authority |
|---|---|---|---|
| **Encoder ripple table** | **every Teensy power-cycle** | Teensy **RAM** (volatile) | fw `encoder-calibration.md` |
| **Camera intrinsics** | once per camera (done) | `openamrobot_perception/config/camera_info.yaml` | docking `06_camera_calibration.md` |
| **Camera extrinsics (mount TF)** | once per build (measured) | `real_bringup.launch.py` static TFs | this doc + docking `03_tf_frames.md` |
| **Dock pose** | per printed bundle / per map | `openamrobot_docking/config/dock_trigger.yaml` | memory `amr-docking-bundle-setup` |

---

## Encoder ripple alignment (per Teensy power-cycle)

The left encoder is physically **off-centre**, producing a ~2/revolution velocity ripple
(~40 % at the worst phase) that shows up as a low-speed left-wheel "oscillation". The
firmware carries a **ripple-correction table** that cancels it — but the table has to be
placed **at the correct phase**, and because the encoder is **incremental it loses its phase
reference at every Teensy power-cycle**. So the table must be **re-aligned after every Teensy
power-cycle** — *not* per ROS launch.

Prerequisite: the micro-ROS agent (or a full bring-up) must be up so `/debug/*` telemetry
exists, or the aligner refuses (`REFUSED: no /debug telemetry`). **Wheels in the air, 24 V,
hand on the cut-off.**

```bash
# on the PC — the script talks to the firmware over Cyclone/domain 0
cd ~/Documents/openamr
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
python3 scripts/align_enc_cal.py --arm 250        # fast (~6-8 s)
# full variant: scripts/calibrate_and_apply.sh
```

Success prints `table placed -> /debug/enc_cal`. Reference shape:
`scripts/encoder_ref_table.json`.

### What the table does and does NOT fix (measured)

- The firmware applies the ripple table **before** publishing the debug rpm, so
  `/debug/left|right.y` is the **corrected** rpm (passthrough 1.0 until a table is loaded) —
  `encoder_calib.py` therefore measures the **residual** once a table is loaded.
- Residual error profile (3 speeds 120/180/250 overlap → speed-independent, geometric):
  **LEFT ±11 %**, **RIGHT ±6 %**. Left is the known off-centre encoder.
- **The profile is normalized to mean 1.000 over a full revolution → the error averages out
  per revolution.** So odometry / position **does not drift**; the only residual is
  **intra-revolution velocity ripple** (the low-speed "oscillation"). It will not stop a
  Station-4 nav.
- **The runtime table is a stop-gap, not the real fix.** Because the incremental encoder
  loses phase at each power-cycle, the accepted long-term fix is a **velocity filter** in
  firmware, not the RAM table.

> ⚠️ A **Teensy reset erases the ripple table** (and re-calibrates the IMU gyro bias). After
> any reset — including the one used to fix a large gyro bias — **re-run `align_enc_cal.py`**
> before trusting driving.

Related: the measured **velocity floors** the docking speeds are based on (linear reliable
0.04 / clean 0.05 m/s; angular reliable 0.15 rad/s) — memory `amr-min-velocity-floors`.

---

## Camera intrinsics (done — do not redo)

The IMX708 is calibrated at **1280×720**: checkerboard 9×12 squares (8×11 inner corners),
30 mm, 87 views → fx ≈ 1415.7, fy ≈ 1415.1, cx ≈ 629.3, cy ≈ 366.4, plumb-bob distortion
`[0.0038, 0.217, ~0, ~0, 0]`. Stored in `openamrobot_perception/config/camera_info.yaml`
(and `~/camera_info.yaml` on the Pi); the bring-up loads it via `camera_info_url`.

> ⚠️ **The resolution MUST match the calibration.** The camera is run at **1280×720** —
> the 16:9 mode *crops* the 4:3 sensor, so it is not a simple downscale of a 480p calibration.
> Do not change the resolution without re-calibrating.

The camera itself only works because the **RPi libcamera fork** + `camera_ros` are built in
`~/camera_ws` (upstream libcamera does not support the Camera Module 3 / IMX708 on the Pi 5).
Background and the recalibration recipe: memory `amr-camera-imx708-libcamera`, and the
authoritative intrinsics/extrinsics doc
[`../../ros2/src/openamrobot_docking/docs/06_camera_calibration.md`](../../ros2/src/openamrobot_docking/docs/06_camera_calibration.md).

---

## Camera extrinsics (mount TF)

The camera is mounted ~0.415 m ahead of the wheel axle and ~0.12 m up, and the physical
sensor is rotated (mounted on its side). The static TF chain in `real_bringup.launch.py`
encodes this:

```
base_link ──(x=0.415, z=0.12)──▶ camera_link ──(roll=-90°, yaw=-90°)──▶ camera_optical_frame
```

The `camera_link → camera_optical_frame` rotation puts the frame in the ROS optical
convention (z forward, x right, y down). If a new build changes the mount, update these
arguments; the docking normal estimate is sensitive to a camera-mount yaw bias (a constant
~+4° offset was observed and left as an open calibration item — see memory
`amr-docking-bundle-setup` Day 3). TF-chain detail:
[`../../ros2/src/openamrobot_docking/docs/03_tf_frames.md`](../../ros2/src/openamrobot_docking/docs/03_tf_frames.md).

---

## Dock pose (per bundle / per map)

Not a sensor calibration but a per-deployment measurement: `dock_pose_x/y/yaw` in
`dock_trigger.yaml` is the centre tag's pose in the **map** frame. Capture it by driving the
robot to the docked spot facing the centre tag and reading `/amcl_pose`; the tag black-square
`size` in `tags_36h11.yaml` must be the measured edge (current bundle: 0.131 m).
`dock_trigger` reads both **at start-up** → **re-launch** the docking stack after editing the
yaml (a hot `ros2 param set` is not enough). Full bundle geometry and the capture procedure:
memory `amr-docking-bundle-setup` and
[`../../ros2/src/openamrobot_docking/docs/README.md`](../../ros2/src/openamrobot_docking/docs/README.md).
