# Compute budget & thermal

What runs on the Pi 5 and what it costs, the "load vs utilization" insight that changed how
we read the numbers, which processes starve navigation, and the **thermal throttling** that
became the day-to-day blocker once the vision plumbing was fixed.

Companion to [`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md) (the vision
detail) and the two audits in the `openamr` instance repo.

---

## The platform

Raspberry Pi 5 Model B Rev 1.1 — **4× Cortex-A76 up to 2.4 GHz**, **8 GB** RAM, Ubuntu
Server 24.04, ROS 2 Jazzy. RAM is *not* a bottleneck (6.4 GB free under the full stack).
Two things are: **cores** (when the pipeline churns) and **temperature** (no cooler fitted).
Full hardware spec: `openamr-platform-hw` → `electrical/computing/raspberry-pi.md`.

---

## What runs, and what it costs

Snapshot during a **real dock**, full (non-composed) stack, load ≈ 8 on 4 cores:

| Process | %CPU | Needed by | Notes |
|---|---|---|---|
| `web_video_server` | 54 % | diagnostics only | **kill for real runs** |
| `camera_node` | 37 % | vision | IMX708 @ 1280×720, software ISP |
| `apriltag_gate.py` | 35 % | vision (old path) | Python full-res republish — removed by composition |
| `apriltag_node` | 20 % | docking only | the detector (~166 % when unthrottled) |
| `dock_trigger.py` | 18–36 % | docking only | the sequencer (burns ~30 % even *idle*) |
| Nav2 (controller/planner/bt/behavior/costmaps) | ~45 % | nav only | **idle during the visual approach** |

The composed pipeline (see doc 03) collapses the vision half of this: vision CPU ~124 % →
~55 %, and a real composed dock sits at **~45 % used / 55 % idle, 48 °C, no throttling**.

---

## The insight: "load 8" was not "out of CPU"

The instinctive reading of *load average 8 on 4 cores* is "the Pi is too weak". Measuring it
properly (`vmstat` + `mpstat`) proved otherwise:

- CPU **48–55 % idle**, iowait **0 %**, run-queue **0–2**, no swap, (that day) **50 °C**.
- But **35 000 context-switches/s** and **20 % system time**.

Half the cores were **free**. The high load number came from **scheduling churn** —
processes constantly waking to shuffle 2.7 MB frames across DDS hops — not from compute
demand. That reframed the whole problem from "buy a bigger computer" to "fix the pipeline
architecture" ([`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md)). **When the
Pi feels slow, measure `context-switches` and `%idle` before concluding it's out of cores.**

---

## Who starves navigation

Two structural wastes make nav slower than it needs to be:

1. **Docking/vision work that nav doesn't need.** With the full stack up but only
   *navigating*, `dock_trigger.py` (~36 %) + `camera_node` (~19 %) + `apriltag_gate.py`
   (~19 %) ≈ **74 % of a core** doing nothing nav uses → load 5.2 on 4 cores, planner and
   controller starved. `dock_trigger.py` in particular burns ~30 % **even while idle**.
2. **Nav2 running flat-out when it's idle.** During the Phase-5 visual approach the robot is
   driven on `/cmd_vel` directly; the controller/planner/costmaps (~45 %) are running but
   **unused**.

### Mitigations

- **Light bring-up** — `use_camera:=false use_docking:=false` — kills those three
  docking/vision processes (~74 % of a core freed), drops load well under 4, and lets the Pi
  cool. This is the right default on Wi-Fi Guest anyway (no camera flood — see
  [`02_networking_and_dds.md`](02_networking_and_dds.md)). Use it whenever you only need nav
  (e.g. the "go to Station 4" demo).
- **On-demand AprilTag (gated non-composed path only)** — on `bringup.launch.py
  use_docking:=true` the `apriltag_gate.py` gate keeps the detector idle (~0 %) until the
  approach. **The composed pipeline (`bringup_composed.launch.py`) has no such gate — its
  detector is always-on through nav**, so there the lever below (deactivate idle Nav2) is the
  only nav-CPU protection ([`03_vision_pipeline_and_cpu.md`](03_vision_pipeline_and_cpu.md)).
- **Deactivate idle Nav2 during the approach** — the highest-value software change left when
  docking. **Do it per-node, NOT with the lifecycle-manager `PAUSE`**: `PAUSE` would also
  kill `velocity_smoother` / `collision_monitor`, which carry `dock_trigger`'s
  `/cmd_vel_nav` → `/cmd_vel`. Deactivate only the truly idle set —
  `controller_server planner_server bt_navigator behavior_server smoother_server
  waypoint_follower` — at Phase-2/5 entry, reactivate at undock. Frees ~1.5 cores exactly
  when the detector needs them.

---

## Thermal — no cooler → the Pi throttles navigation (THE blocker, 2026-07-06)

The Pi 5 is fitted with **no fan / no active cooler**. Under the full robot stack it
overheats and thermally throttles — and unlike the vision-CPU story, this one is not a
software bug you can optimise away:

- **83.4 °C** core temperature (`vcgencmd measure_temp`), near the 85 °C hard limit.
- **`vcgencmd get_throttled` = `0x80008`** → **soft thermal limit ACTIVELY engaged** (bit 3)
  **and** has-occurred (bit 19). **No under-voltage** (power was fine that day).
- At 83 °C the Pi 5 caps its clock (~**2.1 GHz** vs the 2.4 GHz max) to stay under 85 °C →
  Nav2 planning / costmaps slow down. The CPU is being **clock-limited by heat**.

Note the contrast with 2026-07-03 (50 °C, `0x0`, no throttling): thermal state depends on
ambient, run time, and how loaded the stack is. **Always read *both* temp and throttle
flags before concluding.**

```bash
# on the Pi
vcgencmd measure_temp
vcgencmd get_throttled            # want 0x0; 0x80008 = throttling now; any bit-0 = under-voltage
watch -n1 "vcgencmd measure_temp; vcgencmd get_throttled"
```

### Mitigation

- **Real fix (hardware):** fit the official **Raspberry Pi 5 Active Cooler** (~€5–10 clip-on
  heatsink+fan) → ~50–60 °C under load, throttling gone. *On order as of 2026-07-06; this is
  why on-robot work paused in favour of documentation.*
- **Interim (software):** the **light bring-up** removes ~74 % of a core of unnecessary work,
  which both drops load and lets the SoC cool.

`get_throttled` bit reference and the full thermal note:
`openamr-platform-hw/electrical/computing/raspberry-pi.md` → "Thermal".

---

## Platform verdict

The Pi 5 is **not junk and not "too weak" in the abstract** — it runs Nav2 comfortably, or
camera + AprilTag comfortably. What it cannot do with headroom is **all of them at once at
full quality, un-cooled**. Two levers keep it working:

1. **Right the resource budget** — composed vision, on-demand AprilTag, light bring-up when
   nav-only, deactivate idle Nav2 during docking, never run viz during a real dock.
2. **Fit the cooler** — removes the thermal ceiling.

For comfortable production margins the structural answers are to **offload vision** (Jetson
`isaac_ros_apriltag` on GPU, or an OAK camera) or **change the docking modality** to
2D-LiDAR reflectors — but neither is needed to make the current Pi work. See the 2026-07-02
audit §3–5 and memory `amr-lidar-docking-alternatives`.
