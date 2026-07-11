# Troubleshooting — infrastructure

Symptom → cause → fix for the **infrastructure** failures on the real robot: network,
power, thermal, bring-up hygiene, DDS. These are the faults that repeatedly *masquerade* as
software or sensor bugs and cost hours if you debug the wrong layer.

**Scope:** this doc is infrastructure only. **Nav-specific** faults (empty costmaps, DWB
sub-stiction yaw, footprint/inflation, obstacle avoidance) live in
[`../navigation/`](../navigation/); **safety** behaviour in [`../safety/`](../safety/);
**docking** faults in
[`../../ros2/src/openamrobot_docking/docs/09_troubleshooting.md`](../../ros2/src/openamrobot_docking/docs/09_troubleshooting.md).

---

## The three golden reflexes

Most lost hours trace to skipping one of these:

1. **Network before robot.** Before blaming lidar/nav/DDS: `getent hosts botshare.local` +
   `ping`. If they fail, it's Wi-Fi — stop debugging ROS.
2. **Ping before software.** A frozen terminal / lost SSH at launch is very often a **power
   brown-out**, not a hang. Confirm the Pi is *alive* (`ping`) before touching the code.
3. **Voltage before Nav2.** "The robot won't move / drives into things" is very often a low
   **24 V** bus (soft torque), not a planner bug. Check the pack (≥ 25 V at rest) first.

---

## Symptom → cause → fix

### Network / DDS

| Symptom | Cause | Fix |
|---|---|---|
| PC `ros2 topic list` empty | PC on FastDDS / domain 42 | export CycloneDDS + domain 0; `ros2 daemon stop && start` ([`02`](02_networking_and_dds.md) Trap 1) |
| `botshare.local` won't resolve; 100 % ping loss; "lidar/robot dead" | Wi-Fi Guest degraded (or Pi rebooted — see power) | network reflex; verify sensors **on the Pi**; check PC didn't roam SSID |
| Can ping, no robot topics | different subnet / RMW / domain | same `172.17.x.x/16` subnet, `rmw_cyclonedds_cpp`, domain 0 |
| Link collapses when camera / "Start Camera" used | 1280×720 RELIABLE image floods a degraded link (retransmit storm) | light bring-up `use_camera:=false`; Ethernet for real camera use ([`02`](02_networking_and_dds.md) Trap 4a) |
| Nav goals stop reaching the Pi while UI is up | UI Docker **on the PC** pulling big RELIABLE topics | `docker compose down`; run UI **on the Pi** ([`02`](02_networking_and_dds.md) Trap 4b) |
| UI "connected" but panels empty | container inherited FastDDS/42 | launch with `RMW_IMPLEMENTATION=… ROS_DOMAIN_ID=0` prefix ([`06`](06_operator_ui.md)) |
| SSH agent dies / "no output" when backgrounded | agent killed with the SSH session | detached script + logfile, read in a separate call (memory `amr-pi-ros-commands`) |

### Power / brown-out

| Symptom | Cause | Fix |
|---|---|---|
| Terminal freezes at bring-up; SSH drops; `No route to host`; involuntary reboots | **5 V brown-out** — motors+lidar+camera current spike > the 5 A supply → the Pi freezes then loses the network | `ping` first; bring up **without camera** (`use_camera:=false`) to confirm; 5 V buck good for ≥ 5 A peak, short/thick cable; charge the 24 V ([memory `amr-pi5-power-brownout`]) |
| Boot banner "This power supply is not capable of supplying 5A…" | undersized 5 V rail | official 5 V/5 A supply for testing |
| Robot won't move / percusses obstacles nav was avoiding | **low 24 V** → drivers under-volt → soft torque (a wheel can drop out) | multimeter on the pack; **≥ 25 V at rest** before any nav test (memory `amr-battery-voltage-check`) |
| Wheels won't advance / robot blocks (occasional, at start-up) | a motor cable worked loose, or a wheel stalled from rest | **reseat any loose motor cable**; a **full stack restart** (clean-kill + relaunch bring-up) clears a start-up stall |
| Both motor drivers red-LED **code 10** (1 green / 5 red) | **busbar under-voltage** (ZBLD wants 24 V ±20 %) | recharge ≥ 25 V, power-cycle the 24 V (faults latch) |
| Driver **code 14** (2 green / 4 red) | **rotor blocked** (e.g. wheel jammed against a wall) | free the wheel, power-cycle 24 V |

> Testing tip: the robot runs directly off the **24 V mains brick** in parallel with the
> battery ("on a leash") — the stiffest supply for debugging, removes the "weak battery"
> variable. Drive slowly (the brick has no reserve for accel/regen spikes). Fault-code
> legend: `docs/hardware/motor-driver-fault-codes.md` in the instance repo.

### Thermal

| Symptom | Cause | Fix |
|---|---|---|
| Nav planning/costmaps slow after minutes under full stack | **thermal throttling** — no cooler, ~83 °C, `get_throttled=0x80008`, clock capped ~2.1 GHz | fit the **Pi 5 Active Cooler** (real fix); interim: **light bring-up** to shed ~74 % of a core ([`04`](04_compute_and_thermal.md)) |
| "Is the Pi out of CPU?" | load 8 but 48–55 % **idle** → it's **scheduling churn**, not cores | read `%idle` + **context-switches** (`vmstat 1`) + temp/throttle **together** before concluding ([`04`](04_compute_and_thermal.md)) |
| Docking servo oscillates / "tag 1 lost" | stale AprilTag detections from a churning/starved pipeline | composed vision + on-demand gate + kill viz ([`03`](03_vision_pipeline_and_cpu.md)) |

### Bring-up hygiene / processes

| Symptom | Cause | Fix |
|---|---|---|
| TF chaos, serial/USB conflicts, "everything unstable" | **duplicate** agents / lidars / EKF from relaunching without clean-kill | clean-kill, then **one** launch. Count real nodes with `ros2 node list`, **not** `grep` (a grep whose cmdline contains the pattern self-counts) |
| `pkill` kills your SSH session (exit 255) | `pkill -f pat` matches its own command line | bracket trick: `pkill -f "[m]icro_ros_agent"`, `pkill -f "[r]plidar_composition"` |
| `/scan` silent, node alive; or lidar hangs after "RPLIDAR running… SDK 1.12.0" (`80008000`) | RPLIDAR firmware stuck; `respawn` just hammers it | `pkill -f "[r]plidar_composition"`, or **unplug/replug the LiDAR USB**; a battery power-cycle also resets it |
| Two "double goal forwarder" warnings | a stray relay **and** `dock_trigger` both forwarding `/goal_pose` | `pkill -f "topic_tools/relay.*goal_pose"` (exactly one forwarder must run) |
| Costmaps empty after launch → robot blind | nav came up before `map→odom`, or lifecycle nodes hand-activated | do the **2D Pose Estimate** first; if still empty **re-launch** nav — never hand-activate ([`01`](01_bringup.md) §5, and [`../navigation/`](../navigation/)) |
| `sim:=false requires an explicit map` | no `map:=` passed | pass `map:=$HOME/maps/<your_map>.yaml` |
| Camera `no cameras available`; `/camera/image_raw` **0 publishers**; AprilTag silent; **docking aborts `tags not detected`** | forgot to source `~/camera_ws` → camera_ros binds the **system** libcamera, which can't enumerate the IMX708. **Not** the ribbon (kernel still shows `imx708_noir` / `/dev/video0` via `sudo dmesg \| grep imx708`) | re-launch with `~/camera_ws/install/setup.bash` in the block; verify `ros2 topic info /camera/image_raw` → **Publisher count: 1** ([`01`](01_bringup.md) §0) |
| Large IMU gyro bias at rest → yaw drifts → can't localize | gyro not zeroed at boot | reset the Teensy **while the robot is still**; then **re-run encoder calibration** ([`05`](05_calibration.md)) |

---

## The measurement toolkit (on the Pi)

Bounded, low/zero-overhead — never `ros2 topic hz` over SSH (it blocks):

```bash
uptime                                          # load average (want ≤ ~4)
vmstat 1 3                                       # %idle, context-switches (the churn KPI), swap
vcgencmd measure_temp; vcgencmd get_throttled    # thermal (want <60 °C, 0x0)
python3 ~/apriltag_latency.py                    # detector rate + latency (~0 CPU; target <120 ms)
getent hosts botshare.local; ping -c3 botshare.local   # network reachability
nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes'   # did the PC roam SSID?
```

---

## See also

- [`02_networking_and_dds.md`](02_networking_and_dds.md) — the full network failure matrix.
- [`04_compute_and_thermal.md`](04_compute_and_thermal.md) — CPU budget + thermal detail.
- [`../navigation/`](../navigation/) — nav-specific troubleshooting (costmaps, DWB, footprint).
- [`../safety/`](../safety/) — safety behaviour and cut-offs.
- `openamr-platform-hw/electrical/computing/raspberry-pi.md` — thermal + power hardware notes.
