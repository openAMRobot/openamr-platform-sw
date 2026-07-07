# Diagnostics & decisions (the "why")

*Last updated: 2026-06-18.*

A log of the main problems solved and **how** we reasoned, so newcomers understand why the robot is set
up the way it is. Method used throughout: **the LEFT wheel is the reference** (same firmware/gains, so any
asymmetry localizes the fault to the right channel), and **each hypothesis has a distinct data signature**
(we design a test whose result can only match one cause).

---

## 1. "Right wheel runs away / robot doesn't go straight"

### Initial (wrong) hypothesis: right encoder dead
The brief assumed the right encoder was dead (PID windup → runaway). **Refuted:**
- **Test A** (hand-spin each wheel, reconstruct per-wheel speed from `/odom/unfiltered`): both wheels
  produced a signal → both encoders count.
- **Sign test** (roll each wheel forward): both read **positive** → encoder direction correct.
- Later, raw counts (`/debug/*`) confirmed the right encoder reads real motion cleanly under power.

### Root cause #1: driver VAR pot at maximum
Powered test (small `/cmd_vel`, logging `/debug/*`): at nearly equal PWM, **LEFT did 9 rpm, RIGHT did 73 rpm**.
Comparing the only adjustable difference between channels: DIP switches identical, but the **VAR
potentiometer** (speed/gain) was at **10/10** on the right vs **3.5/10** on the left. Lowering the right
VAR to match → the gross runaway disappeared (causal proof: change the cause, change the effect).

### Root cause #2: driver ACC/DEC pot at 0
A residual remained: the right wheel still jerked sporadically (sudden ±50–130 rpm excursions) even after
the VAR fix and after adding PID anti-windup. A **50 Hz capture** around a jerk showed the right counts
moving **coherently** (real oscillation), not isolated spikes → not an encoder glitch; it was a **closed-
loop instability**. An **open-loop test** (`/debug/openloop`, same fixed PWM both wheels, PID bypassed)
showed **both wheels perfectly smooth** → the right *hardware* is healthy; the jerk only happens with the
PID active. Then we found the second pot: each driver has **two** pots, and the **ACC/DEC** (accel ramp)
was **0/10** on the right vs **4/10** on the left. With no ramp, every PID micro-correction hit the motor
abruptly → oscillation. Setting the right ACC/DEC to **4** (matching left) → **fixed**: an 8 s closed-loop
test ran with no jerk.

### Firmware side-fixes (made the loop more robust)
- Added **anti-windup** to the PID (the upstream `pid.cpp` had none — the integral was unbounded).
- Lowered `K_I` (0.4 → 0.15) to damp the loop. **(Later re-tuned to K_I 0.35 on 2026-06-18 — 0.15 was
  over-damped and left a −26 % steady-state error; see §6.)**

### Takeaway
The fault was **100 % motor-driver configuration** (two pots on the right driver), not the encoders,
wiring, or software. Always check the driver pots/DIP match between left and right.

---

## 2. "IMU doesn't work" → it was the wrong chip

Enabling `USE_MPU6050_IMU` made `setup()` hang (LED 3 blinks, no topics). We wrote a tiny **I²C scanner**
(`~/i2cscan`): a device answers at **0x68**, but `WHO_AM_I = 0x70`. A real MPU6050 returns 0x68; **0x70 is
an MPU6500**. The board labeled "MPU6050" is actually an **MPU6500** (common on clones). The MPU6050 driver
rejected it; the **MPU9250 driver accepts it** (it reads WHO_AM_I bits [6:1] = 0x38, which it allows, and
the MPU6500 is register-compatible). Switched config to **`USE_MPU9250_IMU`** → `/imu/data` now reads real
gravity (~9.74 m/s² on z). Hardware/wiring were fine all along. (Details: [../hardware/imu.md](../hardware/imu.md).)

---

## 3. Build/flash toolchain on the Pi (Ubuntu 24.04)

Setting up PlatformIO on a PEP 668 ("externally managed") Ubuntu was non-trivial:
- PlatformIO installed **inside its own venv** (`~/.platformio/penv`) so its internal `pip install`s
  (colcon, empy, lark…) aren't blocked by PEP 668. The penv was created with `virtualenv` (system
  `python3-venv` was missing).
- Flashing: `teensy_loader_cli` (`sudo apt install teensy-loader-cli`) + PJRC udev rules in
  `/etc/udev/rules.d/00-teensy.rules`. The reliable flash is `sudo teensy_loader_cli --mcu=TEENSY40 -s -w`
  (soft reboot); if the write races, retry `-w` once it's in HalfKay.
- `pkill -f micro_ros_agent` **self-matches** its own command line (kills the SSH shell, exit 255) → use
  `pkill micro_ros_agent` (by process name).

---

## 4. LiDAR issues
- **Stuck if killed mid-scan**: a hard SIGTERM leaves it `Cannot start scan: 80008000` / `operation
  timeout`. Recover by **restarting the node** (clean re-init); if that fails, **unplug/replug the USB**.
  Don't loop-respawn a stuck device.
- **Stops publishing on its own**: sometimes the node is alive but `/scan` goes silent. A single node
  restart brings it back. Cause not pinned (USB power / motor?). To watch — especially during SLAM.
- **Mounted rotated 180°**: found empirically (object in front shows at ±180° in the LiDAR frame). The TF
  `base_link→lidar_link` uses **yaw=180°**, x=0.335, z=0.18. The robot's own frame also produces close
  returns (≈ ±20–50°, ±80–90°) → handled with `min_laser_range`/scan filter.

## 5. Smaller gotchas
- **Debug topics QoS**: `/debug/*` are published **best-effort** → a subscriber must request best-effort
  too, else "incompatible QoS, no messages".
- **micro-ROS entities**: adding publishers/subscribers (we have 6 pub + 2 sub) stays under the default
  micro-ROS limit — if exceeded, the Teensy won't connect (no topics) and the LED blinks.

## 6. Session 2026-06-18 — EKF, scan filter, SLAM, camera, PID re-tune, HW diag

### Software brought up (the "inputs" layer for openamr-platform-sw)
- **EKF** (`robot_localization`, `~/ekf.yaml`) replaced `odom_tf_relay`: fuses wheel odom (vx, vyaw) +
  **IMU gyro Z only** → `/odom` + TF `odom→base_link`. IMU gyro Z verified: ~0 at rest (deadband), tracks
  real rotation in turns. See [../software/ros-architecture.md](../software/ros-architecture.md).
- **Scan body filter** (`~/scan_body_filter.py`) → `/scan_filtered`: rear shell masked at all distances,
  side posts masked only < 0.40 m (walls kept). Measured sectors (lidar frame).
- **SLAM** (`slam_toolbox` on `/scan_filtered`) builds & saves maps. Low-res lidar (~270 pts) → drive slow;
  fast moves / in-place spins lose tracking. See [../software/navigation.md](../software/navigation.md).
- **Camera fixed** (IMX708 NoIR): apt libcamera doesn't support Camera Module 3 on Pi 5 ("no cameras
  available" / "Unable to acquire a CFE instance"). Fix: build the **Raspberry Pi fork of libcamera** +
  `camera_ros` from source (`~/camera_ws`). ⚠️ Over WiFi use the **compressed** image only (raw 2.76 MB/frame
  lags everything). See [../hardware/camera.md](../hardware/camera.md).
- **Remote viz** from the Ubuntu desktop: domain 0 + CycloneDDS + same LAN subnet. See [../software/visualization.md](../software/visualization.md).

### Hardware diagnosis (method: per-wheel PWM vs rpm via `/debug/*`)
- **"Left wheel won't move"** = a **24 V faux-contact** (loose battery-side cable), NOT a driver/motor/PID
  fault: under command the firmware drove **PWM to max (saturated) but rpm stayed 0** → the motor wasn't
  powered. Intermittent (worked, then dead, then back after reconnecting). On **mains** both wheels always work.
- **Battery sags under load**: same speed needs **~60 % more PWM on battery than on mains** (read 24.4 V at
  rest). A sagging 24 V rail + the loose cable is what dropped the left wheel. Keep the pack charged; check
  voltage **under load**. See [../hardware/power.md](../hardware/power.md).
- **Startup veer** = right wheel is weaker/slower to break free (stiction asymmetry + right drive dynamics)
  → robot veers at the very start, then the PID catches up. Levers: balance the right driver pots, or
  firmware feedforward.

### PID re-tuned (step-response method)
The wheels didn't reach the commanded speed (−26 % steady-state, ~2.9 s rise). Root cause: **K_I far too
low (0.15)** — it had been over-lowered (0.4→0.15) to damp the right wheel. Step tests at 0.25 m/s (record
`/debug/left|right` target vs measured rpm): **K_P 0.6 / K_I 0.35 / K_D 0.15** → steady-state ±2 %, rise
~1 s. More K_D made it worse (amplifies the ~2.9 rpm measurement quantization). Demonstrated empirically
that the **PID, not "the author's gains," was the lever**. ⚠️ Two gotchas: (1) `pio run -e teensy40` uses
`config/lino_base_config.h`, **not** `custom/dev_config.h`; (2) flashing via `-s` is timing-flaky — the
**reliable method is the Teensy's physical button → HalfKay → `-w`**. See [../firmware/control-loop-pid.md](../firmware/control-loop-pid.md).

## 7. Session 2026-06-19 — wiring/component audit + validation

A full wiring + component audit (reading every label, cross-checking the firmware pins, fetching
datasheets) plus on-robot validation tests. Method: read the silkscreen/nameplates, confirm against
`lino_base_config.h`, then test motors/encoders via `/debug/openloop` (open loop, PID bypassed) and
`/cmd_vel` (closed loop). Comparing **encoder counts** (exact) rather than rpm (quantized ~2.9 rpm).

### Components identified (with datasheets → [../hardware/components-bom.md](../hardware/components-bom.md))
Teensy **4.0** (i.MX RT1062, confirmed by the big QFN chip + the `--mcu=TEENSY40` flash working), drivers
**ZBLD.C20-120L2R** (ZD), motors **ZD Z4BLD60-24GN-30S** (60 W, 24 V, 3.8 A, 3000 rpm, **P=5 pole pairs**,
~30:1 gearbox), encoders **AMS AS5040** (1024 cnt/rev, marking "AS5040 AB 2.2"), IMU **MPU-6500**, battery
**DM12-7S** (12 V 7 Ah). Driver signal terminals confirmed: only `VAR/AI2, FWD/DI1, REV/DI2, COM` wired,
mapping cleanly to the firmware pins (M1: 1/20/21, M2: 5/6/8).

### Two defects found and fixed
- 🔴→✅ **Encoder overvoltage**: AS5040 powered at 5 V → A/B outputs ~4 V into the 3.3 V (non-5 V-tolerant)
  Teensy pins. **Fixed**: moved the encoder supply to **3.3 V** (AS5040 supports it). Verified A/B now
  ~3.3 V; counting clean. See [../hardware/encoders.md](../hardware/encoders.md).
- 🔶→✅ **Pole-pair DIP wrong**: motor is P=5 but SW4/SW5 were OFF/OFF (=2). **Corrected to ON/ON (=5)**.
  Also switched the driver to **open loop (SW1 OFF)** so the Teensy PID is the sole controller (it reads
  the fine AS5040 at the wheel, vs the driver's coarse Hall at the motor shaft). See
  [../hardware/motors-drivers.md](../hardware/motors-drivers.md).

### Safety gaps found (to fix) — [../hardware/power.md](../hardware/power.md)
**No fuse** on the battery, **no battery-side disconnect/E-stop**, and battery+mains share the 24 V bus
in parallel (only one source at a time).

### Validation tests (3 repeated runs, driver open-loop + Teensy PID)
- ✅ **Left wheel stable**: no dropout across 3 sustained runs (counts strictly monotonic). The
  intermittent faux-contact did not reappear (keep monitoring — it's intermittent by nature).
- ✅ **Deadband ~PWM 100** (no motion below); roughly **linear** above: PWM 200→~17 rpm, 400→~40, 600→~65.
- ✅ **Direction**: reverse gives negative rpm and decrementing counts both sides.
- 🔶 **Residual L/R asymmetry ~6–9 %** in open loop (right faster) — reproducible. The **VAR pot is inert**
  to fix it (SW2=ON → speed from AI2, not the pot; confirmed no change after turning it).
- ✅ **Teensy PID equalizes** the wheels in closed loop: counts L vs R within **~0.2 %** at 0.15 m/s → good
  straight-line driving. Odometry scale consistent (0.15 m/s ↔ 14.3 rpm ↔ Ø0.2 wheel).
- ⚠️ **Odom instantaneous velocity is noisy at low speed** (±10 %: 0.138–0.169 m/s read for 0.15 cmd) due
  to the ~2.9 rpm rpm-quantization at 50 Hz. Counts are exact → no cumulative distance error; better at
  higher speed. Worth noting for Nav2 tuning.

## 8. Session 2026-06-19 (cont.) — right encoder, gearbox, SLAM map, Nav2 + AMCL

### Right wheel "drunk"/snaking → right ENCODER misaligned (root cause)
After the wiring audit, the robot still snaked. Method (per-wheel, ground vs air): open-loop fixed PWM
(`/debug/openloop`, PID + driver loop bypassed) **on the ground** showed the LEFT wheel smooth but the
**RIGHT wheel oscillating wildly (6–62 rpm at constant PWM)** — yet it was smooth in the air that morning →
a **load-dependent hardware fault**. Root cause found by the user: the **right AS5040 magnetic encoder was
misaligned** (magnet off-center) → erratic counts under vibration/load → the PID reacted → snaking.
Re-aligning it fixed the oscillation. *(Lesson: a constant-PWM open-loop test on the ground isolates a
hardware fault from any control issue.)*

### Odometry yaw + wheel separation
Manual-turn ground-truth: 1 full physical turn → odom read **−350°** (≈2.8 % under); 10 turns → −3449°
(≈4 % under, worse when spun fast = the gyro can't track fast hand-spins). At slow nav speeds the error is
small. The **wheel separation measured 0.46 m** (config was 0.45) → set **`LR_WHEELS_DISTANCE 0.46`** and
reflashed. Note: the wheel-odom yaw at 0.45 actually *over*-estimated by ~2 %; 0.46 corrects it. The small
residual under-count is the gyro scale (minor; scan-matching/AMCL corrects in SLAM/nav).

### SLAM map `coin2`
Built cleanly once the right encoder was fixed + driving in **arcs** (`u`/`o`, never `j`/`l` in-place) at
0.1 m/s. ⚠️ A featureless/symmetric cardboard enclosure makes the low-res scan-matcher drift — **add
distinctive objects** as landmarks. Saved `coin2.{pgm,yaml,posegraph,data}` (also in repo `maps/`).

### Lidar gotchas (recurring)
- **Stuck `80008000` / operation timeout** after a hard kill mid-scan → **unplug/replug the USB**.
- **Two `rplidar_composition` fighting one port** (from repeated launches) → timeout. Always **clean kill +
  single launch**. (pgrep over-counts due to the `ros2 run` wrapper — verify with `ps -ef`.)

### Nav2 + AMCL brought up and WORKING
Full procedure + all the traps are in [../software/navigation.md](../software/navigation.md). The big ones:
wrong **footprint** (circular 0.22 → real **0.78×0.58 rounded**, base_link offset front 0.415/rear −0.365);
**duplicate scan filter** from `navigation_launch.py` (kill `scan_to_scan_filter_chain`); **teleop flooding
`/cmd_vel`** (kill it); RViz **"Nav2 Goal" does nothing → use "2D Goal Pose"** + a **`goal_relay.py`**
(`/goal_pose`→action); **`obstacle_min_range 0.35` blinded close obstacles → 0.10**; **DWB weaves** (RPP got
stuck rotating → kept DWB); **inflation = soft planning cost**, not collision avoidance (that's the
controller's real-time footprint check; hard guarantee = Collision Monitor / footprint padding — next).

## 9. Still open / next
- **Hard collision avoidance**: ✅ done via **footprint padding +0.12 m** (see §10). Collision Monitor was
  tried but stopped the robot → abandoned.
- **DWB weave**: reduce the residual snaking (raise PathAlign/PathDist, lower vtheta_samples) without
  breaking what works; or revisit a properly-tuned RPP.
- **Nav2 launch hygiene**: edit `navigation_launch.py` to NOT start its duplicate scan filter; consider a
  one-shot bring-up script (clean kill → bring-up → localization → navigation → kill dup → relay).
- **Power safety**: add a **fuse** + **battery disconnect/E-stop** (see §7).
- **Wheel separation 0.46** flashed; **gyro scale** slightly under (minor) — optional firmware trim.
- **Docking**: camera ✅ calibrated; needs AprilTags + the docking pipeline.
- **robot_state_publisher + URDF** (optional; static TFs + footprint cover current needs).
- **Battery telemetry**: still no software voltage readout (lead-acid sags under load).
- **Completeness**: read the LiDAR model, DC-DC model, AC/DC converter, Pi RAM, gearbox ratio (`4GN__K`).

## 10. Session 2026-06-20 — obstacle avoidance: costmaps, footprint padding, battery

### Root cause: BOTH costmaps were EMPTY → robot navigated blind → hit everything
After a reboot, `navigation_launch` was started before AMCL had `map→odom`, so the costmap lifecycle
**aborted** ("transform from base_link to map did not become available"). The nodes were then **activated
by hand** (`ros2 lifecycle set ... activate`) — which brings them up **mis-initialized**: the static layer
never loads the map and the obstacle layer never subscribes to the scan. Result: `/global_costmap/costmap`
and `/local_costmap/costmap` both reported **0 occupied cells** → DWB had nothing to avoid → the robot drove
straight into obstacles ("de plein fouet"). **Fix: relaunch navigation only AFTER `map→odom` exists** (set
2D Pose Estimate first), let the `lifecycle_manager` activate cleanly — never hand-activate. After a clean
relaunch: **36041 global / 3602 local occupied cells** → obstacles seen again.

### Obstacle-avoidance margin (user: "avoid obstacles within 10–20 cm of the robot shape")
- **inflation = soft planning cost only** (the blue halo) — does NOT stop the robot touching things.
- The **hard** lever = **pad the footprint**. DWB checks the footprint against the local costmap every cycle,
  so a footprint padded by **+0.12 m** = a hard "never bring the hull within ~12 cm of a lidar-seen obstacle".
  Applied to both costmaps + persisted; inflation set to **0.20** (planner also routes ~20 cm away).
  `obstacle_min_range` set to **0.0** (scan_body_filter already removes the body; user forbade a min range).
- Padded footprint (base_link frame):
  `[[0.535,0.31],[0.535,-0.31],[0.435,-0.41],[-0.385,-0.41],[-0.485,-0.31],[-0.485,0.31],[-0.385,0.41],[0.435,0.41]]`
- ⚠️ Padded footprint makes the robot "bigger" in RViz (collision shape only, not physical) → in a very
  tight space Nav2 may fail to find a path; reduce the pad if boxed in.

### Two limits that no Nav2 tuning can fix
1. **2D-lidar height blind spot**: lidar at ~18 cm sees only a horizontal slice → obstacles **shorter than
   ~18 cm are invisible** to the costmap → can't be avoided (nothing to avoid). Needs hardware (low sensor /
   tilted depth cam / bumper). Test avoidance with obstacles **> 20 cm tall**.
2. **Execution ≠ plan**: the robot still hit objects that ARE visible on the map. If the costmap sees the
   obstacle and the robot hits it anyway, the **plan avoids it but the robot doesn't follow the plan** —
   prime suspects: **left-wheel faux-contact** (blocker #1 → robot veers) and/or **AMCL drift while moving**
   (bad odometry from the flaky wheel). NOT a costmap/inflation problem.

### Battery too low to draw conclusions
Battery read **23.4 V at rest** (24 V lead-acid pack) → ~30 %, **discharged**. Under motor load it sags
further → weak/erratic torque, the flaky left wheel drops out more → robot doesn't track the path → hits
obstacles. **Avoidance tests at 23.4 V are not conclusive.** Rule added to [../hardware/power.md](../hardware/power.md):
**charge to ≥ 25 V at rest before any navigation/avoidance test.** (Same trap as before: don't debug Nav2
while the 24 V rail is weak.)

### Saved
Map re-saved as `~/maps/coin_ok.{pgm,yaml}` (+ `coin2.*.bak` backups). Config snapshot
`config/nav2_params_real.yaml` updated (footprint, inflation 0.20, min_range 0). Live-trace tooling left on
the Pi: `/tmp/trace.sh` (cmd_vel vs odom vs wheel rpm during a goal — rerun after charging).

### Next session (resume here)
1. **Charge the battery** to ≥ 25 V at rest. Verify under load.
2. Bring up the stack (`/tmp/bringall.sh`), set 2D Pose Estimate, confirm costmaps non-empty (occupancy
   check in [../software/navigation.md](../software/navigation.md) "Verify it's healthy").
3. Run `/tmp/trace.sh` during a goal toward a **tall (>20 cm)** obstacle → check whether **cmd_vel matches
   odom** (else = wheel/execution = fix left-wheel cable, blocker #1) and **AMCL stays localized** (else =
   localization). This decides whether the remaining problem is nav tuning, hardware, or localization.
4. If execution is clean and it still hits a tall, mapped obstacle → revisit DWB tuning / footprint pad.
