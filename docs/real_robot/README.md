# Real robot — engineering & operations

This folder is the **record of running the actual OpenAMRobot** — everything learned taking
the same Nav2 + docking stack from Gazebo onto real hardware: the networking, the vision /
CPU story, the thermal wall, calibration, and the bring-up sequence.

It complements the package-level docs: [`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md)
(per-node architecture), [`../navigation/`](../navigation/) (Nav2 tuning), [`../safety/`](../safety/),
and the docking engineering series
[`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md).
These docs are the **engineering + operations record** — the *why*, the *how it's built*,
and the *how you run it on the real robot*.

---

## Layout

```
docs/real_robot/
├── README.md                     ← this file (index)
│
├── 00_overview.md                the physical robot, the ROS contract, real-vs-sim
├── 01_bringup.md                 the exact start-up sequence (copy-pasteable), light vs full
├── 02_networking_and_dds.md      CycloneDDS/0, mDNS, the Wi-Fi Guest + RELIABLE-flood traps
├── 03_vision_pipeline_and_cpu.md AprilTag latency, intra-process composition, on-demand gate
├── 04_compute_and_thermal.md     compute budget, load-vs-idle insight, thermal throttling
├── 05_calibration.md             encoder ripple, camera intrinsics/extrinsics, dock pose
├── 06_operator_ui.md             the separate openamrobot-ui repo (DDS gotcha, run on the Pi)
└── 07_troubleshooting.md         infrastructure symptom → cause → fix matrix
```

---

## How to read

| Goal | Read |
|---|---|
| **New to the real robot** | `00_overview.md` → `01_bringup.md` |
| **Just bring it up and drive** | `01_bringup.md` (copy-pasteable blocks) |
| **"It was working yesterday" / topics gone** | `02_networking_and_dds.md` then `07_troubleshooting.md` |
| **Docking is slow / servo oscillates** | `03_vision_pipeline_and_cpu.md` |
| **Budget the CPU / the Pi is throttling** | `04_compute_and_thermal.md` |
| **Set up a fresh robot / after a Teensy power-cycle** | `05_calibration.md` |
| **Run the operator UI** | `06_operator_ui.md` |
| **Something's broken and it's not obviously nav** | `07_troubleshooting.md` |

---

## The five things that make real harder than sim

The design goal is that only the **data source** and the **clock** change between sim and
real (see [`../architecture/ARCHITECTURE_OVERVIEW.md`](../architecture/ARCHITECTURE_OVERVIEW.md)).
Everything hard about the real robot is in the gap that principle *doesn't* cover:

1. **The network is fragile Wi-Fi.** DDS discovery, mDNS, and big reliable topics collapse a
   degraded guest link — and it looks like a robot fault. → `02`
2. **Vision is CPU-bound.** AprilTag on the Pi CPU (no GPU) starves Nav2; the whole
   docking-latency saga is a pipeline-architecture problem, not a weak Pi. → `03`
3. **The Pi has no cooler** and thermally throttles navigation under load. → `04`
4. **Everything needs calibrating** — encoder ripple (per power-cycle), camera, dock pose. → `05`
5. **Power is marginal** — brown-outs and a low 24 V bus masquerade as software bugs. → `07`

---

## Conventions used throughout

- **Reach the robot** by mDNS: `ssh botshare@botshare.local` — never a hard-coded IP (DHCP
  moves; 2026-07-06 → `172.17.17.64`).
- **DDS**: CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), **domain 0**. Every terminal
  and every non-interactive SSH command must export it (an SSH shell sources nothing).
- **Command blocks are complete** — every one carries its `source` + `export` lines; never a
  bare `ros2 launch` / `rviz2`.
- **The default entry point** is `ros2 launch openamrobot_bringup bringup_composed.launch.py`
  (`sim:=false` default) — the **composed** profile is THE default for camera/docking. Plain
  `bringup.launch.py` routes the camera through a Python `apriltag_gate.py` (Pi5 load 8+) → use
  it only for nav-only debug with `use_camera:=false`.
- **2D Pose Estimate** in RViz is **mandatory** on the real robot before the robot can see or
  navigate (AMCL's auto-initial-pose is off).

---

## Companion material (other repos / docs)

- **Field runbook & audits** (the `openamr` instance repo): `docs/RUNBOOK-real-robot.md`,
  `docs/AUDIT-2026-07-02-vision-latency-and-compute.md`,
  `docs/AUDIT-2026-07-03-cpu-pipeline-optimization.md`, and the day logs.
- **Firmware** (`openamr-platform-fw`): `docs/architecture/encoder-calibration.md`.
- **Hardware** (`openamr-platform-hw`): `electrical/computing/raspberry-pi.md` (thermal +
  power), `electrical/computing/teensy.md`.
- **Docking engineering series**:
  [`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md).
