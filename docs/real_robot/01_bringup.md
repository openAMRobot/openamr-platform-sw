# Bring-up — the operational sequence

The exact start-up sequence for the real robot, with complete copy-pasteable command
blocks. This is the on-robot backbone; it consolidates the field runbook
(`openamr` instance repo → `docs/RUNBOOK-real-robot.md`) with the launch-file behaviour.

**Rule of the house:** every block below includes its `source` + `export` lines. Never run
a bare `ros2 launch` / `rviz2` — an SSH shell is non-interactive and does **not** source
ROS, and the PC defaults to the wrong DDS (see [`02_networking_and_dds.md`](02_networking_and_dds.md)).

---

## Sequence at a glance

```
0. DDS env          (every terminal, PC and Pi)
1. micro-ROS agent  (started by the bring-up; standalone only for bare calibration)
2. Encoder calib    (per Teensy power-cycle, BEFORE trusting driving)   → 05_calibration.md
3. Bring-up         ros2 launch openamrobot_bringup bringup.launch.py …
4. 2D Pose Estimate (RViz, on the PC)  ← MANDATORY, gives map→odom
5. Verify           node list · /scan_filtered alive · costmaps non-empty
6. (optional) UI    → 06_operator_ui.md   (dock/undock → docking docs)
```

---

## 0. DDS environment — every terminal, PC and Pi

The robot runs **CycloneDDS on domain 0**. The PC defaults to **FastDDS / domain 42** and
sees *nothing* until overridden. Prefix **every** terminal (and every non-interactive SSH
command) with:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

On the Pi, the bring-up additionally needs the workspace overlays sourced (agent + camera
fork + the platform install):

```bash
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash          # micro-ROS agent
source ~/camera_ws/install/setup.bash              # RPi libcamera fork + camera_ros
source ~/openamr-platform-sw/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

> An **interactive** SSH login sources jazzy + the micro-ROS agent workspace (`microros_ws`) via `~/.bashrc`; add the
> camera and platform overlays yourself. A **non-interactive** SSH command (`ssh host cmd`)
> sources nothing — always paste the full block.

Why this matters, and the Wi-Fi it rides on: [`02_networking_and_dds.md`](02_networking_and_dds.md).

---

## 1. micro-ROS agent (Teensy telemetry) — ON THE PI

The agent bridges the Teensy: `/cmd_vel`, `/odom/unfiltered`, `/imu/data`, `/debug/*`. The
full bring-up (§3) starts it automatically. Run it standalone **only** for a bare encoder
calibration with nothing else up:

```bash
# on the Pi
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyACM0
```

Leave it running. Harmless noise: `Failed to parse type hash … USER_DATA (null)`
(micro-ROS does not populate type hashes). Do **not** try to keep the agent alive in a
backgrounded SSH call — it dies with the session; use a detached script + logfile (recipe
in memory `amr-pi-ros-commands`).

---

## 2. Encoder calibration — per Teensy power-cycle, BEFORE driving

The ripple-correction table lives in **Teensy RAM** → re-run after **every Teensy
power-cycle** (not per ROS launch). Wheels in the air, 24 V, hand on the cut-off. Needs the
agent (§1) up so `/debug/*` exists. Summary here; full detail in
[`05_calibration.md`](05_calibration.md).

```bash
# on the PC — talks to the firmware over Cyclone/domain 0
cd ~/Documents/openamr
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
python3 scripts/align_enc_cal.py --arm 250        # ~6-8 s; success: "table placed -> /debug/enc_cal"
```

---

## 3. Full bring-up — ON THE PI

One command starts the whole stack: data source (agent + LiDAR + EKF + scan filter +
optionally camera) + Nav2 localization + navigation (+ docking with `use_docking:=true`).
`bringup.launch.py` is the single sim/real selector; `sim:=false` is the default.

### Full profile (nav + camera + docking)

```bash
# on the Pi — full block, copy-pasteable standalone
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash          # micro-ROS agent
source ~/camera_ws/install/setup.bash              # RPi libcamera fork + camera_ros
source ~/openamr-platform-sw/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 launch openamrobot_bringup bringup.launch.py \
  map:=/home/botshare/maps/piece_actuelle.yaml \
  use_docking:=true
```

### Light profile (nav only — RECOMMENDED default on Wi-Fi Guest)

Drop the camera **and** docking. This is the right default on the fragile guest network and
when you don't need vision: it removes the camera stream that saturates the link and frees
the ~74 % of a core that `camera` + `apriltag_gate` + `dock_trigger` burn (which starves the
planner near costmaps).

```bash
# on the Pi — full block, copy-pasteable standalone
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash          # micro-ROS agent
source ~/camera_ws/install/setup.bash              # RPi libcamera fork + camera_ros
source ~/openamr-platform-sw/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 launch openamrobot_bringup bringup.launch.py \
  map:=/home/botshare/maps/piece_actuelle.yaml \
  use_camera:=false use_docking:=false
```

### The launch arguments that matter

| Arg | Default | Effect |
|---|---|---|
| `sim` | `false` | `false` = real hardware; `true` = Gazebo. |
| `map` | *(empty)* | **REQUIRED on real** — `sim:=false` refuses to start without an explicit map (no silent fallback to the bundled sim map). |
| `use_camera` | `true` | Real only. `false` = no IMX708 → lighter load, no Wi-Fi camera flood, less brown-out risk. |
| `use_docking` | `true` | `true` = `dock_trigger` owns `/goal_pose` and forwards it to `/goal_pose_nav` (it **is** the goal forwarder + AprilTag docking). `false` = a plain `topic_tools` relay takes over the forwarding for nav-only debug. **Exactly one forwarder runs — never both.** |
| `use_rviz` | `false` | Open RViz with the Nav2 view (usually run RViz on the PC instead). |

> **Optimized vision variant.** For the composed (intra-process) camera+AprilTag pipeline
> there is a dedicated one-command launch,
> `openamrobot_bringup bringup_composed.launch.py map:=…` — it runs the light bring-up
> (`use_camera:=false use_docking:=false`) **plus** the composed vision container **plus**
> the docking nodes. Why it exists and what it fixes: [`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md).

### What `use_docking` does to goal routing (important)

`navigation_launch` remaps `bt_navigator`'s goal to `/goal_pose_nav`, so the RViz **2D Goal
Pose** (which publishes `/goal_pose`) must be **forwarded** or it never reaches Nav2:

```
use_docking:=true   RViz 2D Goal Pose → /goal_pose → dock_trigger (undock-if-docked, then forward) → /goal_pose_nav → Nav2
use_docking:=false  RViz 2D Goal Pose → /goal_pose → topic_tools relay → /goal_pose_nav → Nav2
```

If you ever see **two** forwarders (e.g. the composed bring-up's relay *and* `dock_trigger`),
`dock_trigger`'s double-forwarder guard fires — kill the stray relay:
`pkill -f "topic_tools/relay.*goal_pose"`.

---

## 4. 2D Pose Estimate — MANDATORY

On the real robot AMCL's `set_initial_pose` is **disabled** (the robot is almost never at
the map origin at boot). Until you give it a pose, there is **no `map→odom`** and the
costmaps stay **empty** → the robot is blind and Nav2 does nothing.

In RViz on the PC: use **2D Pose Estimate** and click/drag the robot's real pose on the map.
Nav2 auto-activates (the `bond_timeout 60 s` fix). Only then send a **2D Goal Pose** — **not**
"Nav2 Goal", which needs a panel we don't run and does nothing.

Run RViz on the PC with the working config (never bare `rviz2` — no map panel):

```bash
# on the PC
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
rviz2 -d ~/Documents/openamr/scripts/openamr_nav.rviz
```

---

## 5. Verify it's up

Bounded checks only — **never** `ros2 topic hz` over SSH/Wi-Fi (it blocks/timeouts). From
the PC:

```bash
# on the PC — full block, copy-pasteable standalone
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# nodes present
ros2 node list

# the body-filtered scan is alive (Best Effort!)
ros2 topic echo /scan_filtered --once --qos-reliability best_effort

# EKF is publishing odom→base_link
ros2 run tf2_ros tf2_echo odom base_link

# localized
ros2 topic echo /amcl_pose --once

# costmaps NOT empty (blind robot if 0) — count non-free/unknown cells
ros2 topic echo /global_costmap/costmap --field data --once \
  | tr ',' '\n' | grep -vE '^0$|^-1$|^$' | wc -l          # must be > 0

# nav lifecycle active
ros2 lifecycle get /controller_server                     # -> active
```

Expected: `/scan_filtered` ~6.8 Hz (normal for an A1), a valid `/amcl_pose`, thousands of
occupied costmap cells. If costmaps are empty **after** a 2D Pose Estimate, re-launch the
nav rather than hand-activating lifecycle nodes (hand-activation comes up mis-initialised —
see [`../navigation/`](../navigation/) and memory `amr-nav2-bringup` piège #8).

---

## 6. Docking (AprilTag bundle)

Docking is triggered by a `Bool` on `/dock_trigger` (or the UI dock button). The bring-up
must have been started with `use_docking:=true` (or use `bringup_composed.launch.py`). The
one-time bundle/pose config, the phase sequence, and the on-demand AprilTag gate are covered
in the docking docs and [`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md); the
operational trigger is:

```bash
# on the PC or Pi — full block, copy-pasteable standalone
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 topic pub --once /dock_trigger std_msgs/msg/Bool "{data: true}"    # dock
ros2 topic pub --once /undock_robot  std_msgs/msg/Bool "{data: true}"   # undock

# manual AprilTag toggle — gated non-composed path only (no-op on the composed pipeline)
ros2 service call /apriltag/set_enabled std_srvs/srv/SetBool "{data: true}"
```

Bundle geometry, dock-pose capture, and the tuned parameters:
[`../../ros2/src/openamrobot_docking/docs/README.md`](../../ros2/src/openamrobot_docking/docs/README.md)
and memory `amr-docking-bundle-setup`.

---

## Common bring-up faults (quick)

| Symptom | Likely cause | Action |
|---|---|---|
| PC sees no topics | wrong DDS (FastDDS/42) | re-export the §0 block; `ros2 daemon stop && ros2 daemon start` |
| `sim:=false requires an explicit map` | no `map:=` | pass `map:=/home/botshare/maps/piece_actuelle.yaml` |
| Costmaps empty, robot blind | no `map→odom` | do the **2D Pose Estimate** first |
| Duplicated agents/lidars/EKF, TF chaos | relaunched without clean-kill | clean-kill, then **one** launch |
| `/scan` silent, node alive | RPLIDAR stuck (`80008000`) | `pkill -f "[r]plidar_composition"`, or unplug/replug the LiDAR USB |
| Two goal forwarders warn | relay + dock_trigger both up | `pkill -f "topic_tools/relay.*goal_pose"` |
| Link collapses at bring-up | camera flood on Wi-Fi | use the **light profile** ([`02_networking_and_dds.md`](02_networking_and_dds.md)) |
| Pi freezes / drops off net at launch | 5 V brown-out / low 24 V | `ping` before blaming software ([`07_troubleshooting.md`](07_troubleshooting.md)) |

Full infrastructure matrix: [`07_troubleshooting.md`](07_troubleshooting.md). Nav-specific
tuning (footprint, inflation, DWB floors): [`../navigation/`](../navigation/).
