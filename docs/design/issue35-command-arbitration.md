# Issue #35: command arbitration and docking protection

Status: **Proposed, review note**, pending owner review. Nothing here is approved,
deployed or validated on hardware.

- Issue: [openAMRobot/openamr-platform-sw#35](https://github.com/openAMRobot/openamr-platform-sw/issues/35), Gate A target 30 October 2026.
- Implementation owner: **Kunal Yadav (@skunal3318)**. The option (a) design reviewed here is
  Kunal's (Slack design note `issue35_arbitration_proposal.md`, 17 October 2026, and the #35
  command-chain and coverage tables of 9, 14, 17 and 18 October). This note reviews and extends
  it; it does not replace his ownership of the implementation.
- Reviewers named in #35: Amankhan Pathan, Evans Kwashie, Anushka Arun Muragod. Software lead:
  Mohamed Sayed (@panthera-momagdii). Firmware: Evans Kwashie.
- Basis: `main` at `9a87111578bff1d0d836f3c43c5d0e4022e270b6`, openamrobot-interfaces `main` at
  `87ab8a936c97ab55ee74b8988ca3d7659160ea6a`, openamr-platform-fw `main` at
  `f96014daae614da4e1bf0c00376f1d9e5d41ae02`. ROS 2 Jazzy, Nav2 1.3.13, twist_mux 4.5.0
  (Debian packages), Gazebo Harmonic. PR #54 (behavior_server remap) is approved but not merged.
- Boundary: the E-stop is hardwired and cuts actuator power (P-03 item 20). Every software stop
  in this note is a functional stop, not the safety function. Nav2 collision_monitor is not a
  certified safety device. Simulation results are development evidence only.

## 1. Current command chain on main, as built

Recorded live in Gazebo with `ros2 topic info -v`, lifecycle `transition_event` traces and a
command recorder (section 8). "Sim" is `bringup.launch.py sim:=true`. The real profile
(`sim:=false`) uses the same `navigation_launch.py` and `dock_trigger.yaml`; it was read from
source and launch descriptions, not run.

```text
controller_server --cmd_vel_nav (20 Hz)--> velocity_smoother --cmd_vel_smoothed (20 Hz)--> collision_monitor --+
behavior_server: spin, backup, drive_on_heading, assisted_teleop, wait (10 Hz) ----------------------------------+--> /cmd_vel --> base
dock_trigger (loop 20 Hz nominal, Phases 2-5 and undock) ----------------------------------------------------------+
teleop_twist_keyboard, if started by hand (docs/navigation/04_real_robot_tuning.md) ------------------------------+
   base, sim:  ros_gz_bridge -> gz DiffDrive (accel limit 0.5 m/s^2; NO command timeout)
   base, real: micro_ros_agent (serial 115200) -> Teensy, Twist, reliable; 200 ms watchdog -> fullStop()+brake
```

- All topics are `geometry_msgs/msg/Twist`, RELIABLE, VOLATILE.
- `/cmd_vel` publishers on main, live:
  - **sim, plain-node Nav2:** `dock_trigger`, `behavior_server` (5 endpoints) and `collision_monitor`.
  - **composable Nav2:** `behavior_server` (5) and `collision_monitor`. This was run by hand
    with a `nav2_container`; no repo launch starts that container or sets `use_composition`.
- The behavior_server bypass is live on main until #54 merges (`navigation_launch.py:174` and
  the composable equivalent).
- `assisted_teleop` subscribes to `/cmd_vel_teleop`; nothing publishes it in any launch.
- `collision_monitor`:
  - the only polygon is `FootprintApproach` (approach, 0.8 s) (`nav2_params.yaml:361-370`);
  - scan source `scan_filtered`, `source_timeout 1.0`, `stop_pub_timeout 2.0`;
  - there is no stop or slowdown polygon.
- `velocity_smoother`: 20 Hz, OPEN_LOOP, `max_accel [1.0, 0, 3.0]`, `max_decel [-2.5, 0, -3.0]`,
  `velocity_timeout 1.0`.

### 1.1 Live findings that change the picture

1. **Pausing collision_monitor resets the whole Nav2 stack.** `_pause_collision_monitor()`
   deactivates the monitor through its lifecycle service. That breaks its bond with
   `lifecycle_manager_navigation`, whose log then shows "CRITICAL FAILURE: SERVER
   collision_monitor IS DOWN ... Resetting managed nodes". It deactivates and cleans up all eight
   navigation nodes, then reconfigures and reactivates them.
   - The monitor is active again about 2.4 s (sim time) after the pause, while dock_trigger is
     still driving.
   - At the end of docking, `_resume_collision_monitor()` fails three times and publishes a
     **false** `collision_monitor_down`, while the monitor is in fact active.
   - So the "monitor OFF in Phases 2 to 5" rows in the current tables are not what runs.
   - Evidence: `issue35-evidence/lifecycle_manager_reset_on_pause.log` and the B0 trace.
2. **On main the sim robot cannot reach staging.** Since `lidar_link` moved to (0.330, 0, 0.124)
   with yaw pi (#55, #70), the robot body appears at scan angles 0 ± 46 deg (0.40 to 0.73 m).
   - `scan_body_filter.yaml:28-29` still keeps [-140, +140] deg for the old mount.
   - collision_monitor therefore sees the robot itself and zeroes every Nav2 command.
   - Docking aborts in Phase 1 (`NavigateToPose` result timeout after 120 s, robot static).
   - All docking runs below use sim-only fixtures F1 to F3 (section 8) to get past this.
3. **The sim lidar is blind within 0.40 m of `lidar_link`**, i.e. 0.31 m beyond the footprint
   front.
   - `range_min` is 0.40 in `gazebo_control.xacro:213`. The sensor sits inside the simplified
     chassis collision box, so with `range_min 0.05` every ray returns about 0.058 m.
   - Consequence: no near-field obstacle stop and no lidar dock-arrival stop can be validated in
     this simulation. The B0 and P1 final stops fired on the dead-reckoned lidar estimate
     (`lidar-dr`), not a measurement.
4. **Gazebo DiffDrive keeps executing the last command forever.** 0.10 m/s, then silence: the
   robot kept moving (0.5 m in about 5 s sim) until an explicit zero.
   - Any test that relies on "the source stops publishing" proves nothing in sim unless a zero
     is published downstream.
   - The real Teensy brakes after 200 ms (`firmware.ino:611`).
5. **dock_trigger timing is wall clock** (`time.time()`, `time.sleep()`).
   - In sim at a real-time factor of about 0.3, its loop runs about 3x faster against physics,
     and wall-clock deadlines expire early.
   - The undock reverse times out after 0.58 m of the 0.70 m (both B0 and P1).
6. **The sim dock pose is the real map's.** `dock_trigger.yaml:48-49` (1.362, 0.222) is the
   real map; the sim world dock is at (4.899, 0). The sim profile loads the real value.

### 1.2 Per-phase table, main, B0 run (main + fixtures F1 to F3), live trace

Docking run on 18 October, CSV `b0/cmd.csv`, sim times.

| Phase | Active publisher on /cmd_vel | collision_monitor lifecycle | Obstacle guard (effective) | Smoothing / rate limit | Timeout on its input | If that publisher dies |
|---|---|---|---|---|---|---|
| 1 NavigateToPose to staging | collision_monitor (controller -> smoother -> monitor) | active | FootprintApproach, 0.8 s, `scan_filtered` | velocity_smoother 1.0 / 2.5 m/s^2 | smoother `velocity_timeout 1.0`, monitor `source_timeout 1.0` | smoother ramps to 0 after 1.0 s; base stops |
| 1->2 handover (t=22.85) | dock_trigger publishes 0, then pauses monitor | active -> inactive; manager reset; **active again at t=25.24** | none for 2.4 s, then FootprintApproach with no input | none | none | sim: last command held forever; real: Teensy 200 ms brake |
| 2 tag search, centring scan (rotate) | dock_trigger direct | active (after reset), no input | none (swept body not checked) | none: steps up to 0.42 rad/s per message | none | same |
| 1.5 / 3 / 4 drive on the normal | dock_trigger direct | active, no input | cone guard is a no-op (`obstacle_check_enabled: false`, yaml:260) | none: steps up to 0.10 m/s per message | none | same |
| 5 final visual approach | dock_trigger direct | active, no input | none by design; stop at `stop_lidar_distance 0.13` from the **lidar** or camera depth 0.13 | taper to 0.05 m/s floor, unsmoothed | none | same |
| docked | none | active | n/a | n/a | n/a | n/a |
| undock reverse 0.70 m + spin | dock_trigger direct | active, untouched | none (rear sector removed by the filter) | none | none | same |

Measured in B0:
- dock command gaps while moving: median 0.054 s, p99 0.082 s, max 0.114 s (wall);
- travel after the final zero: 2.1 mm at 0.053 m/s;
- final front gap 0.049 m (front = base + 0.39 m);
- lidar to dock 0.109 m, while the dead-reckoned estimate that triggered the stop read 0.128 m.

## 2. Option (a) as proposed (Kunal, 17 October)

```text
collision_monitor -> cmd_vel_nav_out (priority 10, 0.5 s) -+
dock_trigger      -> cmd_vel_dock   (priority 20, 0.2 s) -+-> twist_mux -> /cmd_vel -> base
```

Checked against twist_mux 4.5.0 source and a standalone run (`option_a_checks.txt`):

| Check | Result |
|---|---|
| Output type with the package default | `use_stamped` defaults to **true** in 4.5.0: output is TwistStamped. The Gazebo bridge and the Teensy subscribe Twist, so the base would get nothing. Must set `use_stamped: false` (twist_mux.cpp L81). |
| All inputs time out | twist_mux publishes **nothing** (no zero). Measured: 0 messages in 1.5 s. Sim then keeps the last command forever; real base brakes after 200 ms (Teensy). |
| Lock 255 vs teleop 255 | teleop **not masked** (19 of 19 forwarded): the lock masks only priorities strictly lower (topic_handle.hpp L238). Mohamed's 18 Oct correction holds. |
| Lock 255 vs teleop 100 | masked (0 forwarded). |
| Bypass | behavior_server still writes `/cmd_vel` until #54 merges, and would race the mux output. Teleop is not in the table. |
| Dock path | still unsmoothed and unguarded while the monitor is "paused"; the pause still resets Nav2 (section 1.1). |
| Timeouts | dock 0.2 s equals the Teensy watchdog; measured dock gaps p99 0.082 s, max 0.114 s in sim, but the loop has a code-path worst case near 1.3 s (section 5). |
| Latency | twist_mux republishes in the input callback: no added period, no type change once `use_stamped: false`. |

Verdict: option (a) fixes the undock and handover race between two direct `/cmd_vel` writers,
but it is not correct as written (TwistStamped) and not enough for #35 (no rate limit or guard
on the dock path, no stop lock, no teleop, a monitor pause that resets Nav2).

## 3. Recommended design for Gate A: arbitration first, monitor last

This is the target Mohamed set on 9 October (arbitration -> limiting -> active protection ->
the sole final command path), using only existing components:

```text
cmd_vel_nav   (controller_server, behavior_server after #54)  prio 10  timeout 0.5 --+
cmd_vel_dock  (dock_trigger)                                  prio 20  timeout 0.3 --+
cmd_vel_joy   (human teleop with deadman)                     prio 100 timeout 0.25 -+--> twist_mux --> cmd_vel_mux
cmd_vel_stop  (zero Twist from the stop requester)            prio 255 timeout 0.5 --+    (use_stamped: false)
lock stop_lock (Bool, timeout 0 = manual)                     prio 255 masks all < 255
cmd_vel_mux --> velocity_smoother (limits every source) --> cmd_vel_smoothed --> collision_monitor --> /cmd_vel --> base
```

- **collision_monitor is never deactivated.**
  - For docking and undocking it switches polygons: `FootprintApproach` off, `DockStop` on
    (stop, footprint + 0.05 m, front margin below the 0.13 m dock gap).
  - The switch uses the per-polygon dynamic `enabled` parameters, read back and published
    latched on `/docking/guard_mode` (NAV, DOCK, FAILED).
  - In the prototype a separate node does this. In the implementation it belongs where
    dock_trigger now calls `_pause_collision_monitor()` and `_resume_collision_monitor()`.
- **Teleop topic name.** Do not name the human teleop topic `cmd_vel_teleop`: Nav2
  assisted_teleop already subscribes to it.
- **Stop lock.** It must be raised together with zeros on `cmd_vel_stop`. A lock alone makes
  twist_mux silent, and the smoother then holds the last command for `velocity_timeout`
  (measured 1.0 s).
- **Modes.** Modes are labels on the priority table (NAVIGATION 10, DOCKING 20, MANUAL 100,
  STOPPED 255), as Kunal proposed. They are reported by twist_mux diagnostics and
  `/docking/guard_mode`, and should surface through #37 reasons. A separate mode state machine
  is 2.0 work.

### 3.1 Timeouts and limits (Proposed values)

| Item | Value | Basis |
|---|---|---|
| nav input timeout | 0.5 s | controller 20 Hz, behaviors 10 Hz |
| dock input timeout | 0.3 s | B0 gaps p99 0.082 s, max 0.114 s; must stay above the real loop p99 measured on the Pi 5 |
| teleop timeout | 0.25 s | joystick at 20 Hz or more, deadman button |
| stop source timeout | 0.5 s | stop requester publishes at 10 Hz or more while engaged |
| velocity_smoother velocity_timeout | **0.3 s proposed** (now 1.0) | publisher loss: 1.0 s gives 0.111 to 0.113 m travel at 0.10 m/s; 0.3 s gives 0.041 m (sim, measured) |
| limits for every source | existing smoother limits (0.20 m/s, 0.5 rad/s, 1.0 / 2.5 m/s^2, 3.0 rad/s^2) | no new numbers |
| collision_monitor source_timeout | 1.0 s (unchanged) | stale scan forces stop while active, now including docking |

### 3.2 Guards per phase in the recommended design

| Phase | Final publisher | Guard | Rate limit | Loss of the source |
|---|---|---|---|---|
| 1 Nav2 to staging | collision_monitor | FootprintApproach | smoother | smoother zero after velocity_timeout |
| 2 rotate / 3, 4 drive / 5 final | collision_monitor | DockStop (stop at footprint + 0.05 m), stale-scan stop | smoother (now applied to docking) | same |
| docked / undock reverse + spin | collision_monitor | DockStop; rear limited by the lidar filter and still a gap | smoother | same |
| teleop | collision_monitor | FootprintApproach or DockStop per mode | smoother | same |
| software stop | collision_monitor | n/a | zero passes immediately | lock stays engaged (timeout 0) |

## 4. Results of the prototype in simulation (P1, P2)

Prototype on branch `review/issue35-arbitration` (config, launch, `dock_guard_mode.py`), with
sim fixtures F1 to F3. Docking runs on 18 October (P1 on the prototype before the last tool
edits; P4 and P5 on the committed code). C3 bounds per 20 Hz message: speed up at most 0.05 m/s,
down at most 0.125 m/s, turn rate change at most 0.15 rad/s (the existing smoother limits).

| Criterion (`check_run.py`) | main (B0) | prototype P1 | prototype P5 | prototype P4 |
|---|---|---|---|---|
| C1 single final publisher during docking | FAIL (`behavior_server`, `collision_monitor`, `dock_trigger`) | PASS | PASS | PASS |
| C2 collision_monitor stays active | FAIL (deactivated, manager reset) | PASS (0 transitions) | PASS | PASS |
| C3 every source rate limited before the protective stage | FAIL (0.424 rad/s step; 0.05 m/s over the bound) | PASS | PASS | PASS |
| C4 docked | PASS | PASS (front gap 0.049 m) | PASS | **FAIL**: Phase 1 aborted |

- P4 aborted in Phase 1 with "Timed out while waiting for action server to acknowledge goal
  request for follow_path" (`bt_navigator default_server_timeout: 20` ms on a loaded
  container). The robot was moving normally through the new chain (0.2 m/s on every stage).
- This was not root-caused. It is a Nav2 BT acknowledgement timeout, not seen in B0 or P1.
- Docking runs completed: prototype 2 of 3. A 10-run series (T9) is still required.
- No "Resetting managed nodes" in any prototype run; guard mode NAV -> DOCK at staging -> NAV
  after undock.

Scenarios P2 (fresh sim, `scenarios.py`, sim time; P3 repeated them on the committed code
with the same results within 0.02 s and 0.003 m, `scenarios_p3_committed_code.txt`):

| Scenario | Pass criterion | Result |
|---|---|---|
| priority and handover | dock preempts nav, no nav sample while dock live, nav back within dock timeout + 0.15 s | PASS: 0.019 s, 0 leaks, nav back 0.289 s |
| stop lock over teleop (100) | /cmd_vel zero, no teleop forwarded | PASS: zero 0.04 s after lock, 0 forwarded, 3.8 deg coast (gz 1.0 rad/s^2) |
| lock without zero source | documents the gap | mux silent; /cmd_vel held 0.4 rad/s for 1.00 s, zero at 1.04 s |
| teleop loss at 0.10 m/s | zero within velocity_timeout + 0.5 s | PASS: zero 1.0 s, travel 0.111 m |
| dock loss at 0.10 m/s | same | PASS: zero 1.02 s, travel 0.113 m |
| same two, velocity_timeout 0.3 (live param) | same | PASS: zero 0.30 to 0.31 s, travel 0.041 m |
| obstacle in the dock path (DockStop) | stop before contact | **BLOCKED in sim**: lidar blind within 0.40 m (finding 3) |

## 5. Stopping distance: bound, not estimate

Kunal's 18 October figure (1.5 cm) assumes 50 ms latency, 0.10 m/s and the Gazebo 0.5 m/s^2.
Mohamed is right that it is an estimate. The stop distance after the stop condition becomes true
in the Phase 5 loop (`dock_trigger.py:1297-1673`):

```text
d = v * (t_scan + t_pre + t_cyc + t_chain + t_link) + v^2 / (2 a)
```

| Term | Meaning | Value |
|---|---|---|
| v | speed at the threshold | 0.05 m/s (taper floor `final_push_speed`); 0.12 if raised as the yaml suggests |
| t_scan | scan age; `_min_range_in_arc` never checks it (2260-2294) | 0.1 s at 10 Hz; unbounded if the scan stops (falls back to dead reckoning) |
| t_pre | TF lookups before the check in the same iteration | 0 typical; up to 0.25 s (0.05 pose + 0.1 tag + 0.1 odom) |
| t_cyc | rest of the previous iteration + `sleep(period)` | measured p99 0.082 s (sim, wall); code-path worst case about 1.3 s when frames are missing (about ten lookups with 0.1 to 0.2 s timeouts) |
| t_chain | smoother period (prototype only) | up to 0.05 s |
| t_link | micro-ROS serial + Teensy 50 Hz loop | about 0.02 s, unmeasured |
| a | base deceleration | sim 0.5 m/s^2; **real unknown**: no firmware ramp, `brake()` behaviour unverified |

- **Typical** at 0.05 m/s: 0.25 s gives 1.3 cm, plus 0.3 cm, about **1.5 cm**.
- **Code-path worst case** at 0.05 m/s: 1.7 s gives 8.6 cm, plus 0.3 cm, about **8.9 cm**;
  at 0.12 m/s about 22 cm.

The margin is not 13 cm:
- `stop_lidar_distance` is measured from `lidar_link` (x 0.33 on 2.0; Gate A static TF 0.335),
  not from the front;
- the sim front gap at the stop was 0.049 m.

So the worst case exceeds the margin. The fix is to bound the loop (a timer at
`drive_rate_hz`, short or zero TF timeouts, and a scan age check), not to tune.

**What must be measured** (supervised bench, Evans and Kunal):
1. Base deceleration and distance from 0.05, 0.10 and 0.20 m/s, on a zero command and on
   watchdog expiry.
2. Teensy watchdog actual timeout.
3. dock_trigger loop period histogram on the Pi 5 in Phase 5.
4. Scan age at the stop decision.
5. End to end: scan stamp -> zero on /cmd_vel -> wheels stopped.
6. Final front gap over at least 10 dockings.
7. The A1 physical minimum range against the 0.13 m threshold. sllidar_ros2 at the pin
   publishes `range_min 0.05` (`sllidar_node.cpp:228`); the Slamtec A1 datasheet states 0.15 m
   (not verified on the unit).

## 6. Base side (firmware) and motion enable

| Item | Status | Evidence |
|---|---|---|
| Teensy /cmd_vel watchdog | implemented: 200 ms -> `fullStop()`: zero twist, `brake()`, PID reset; also on agent loss | platform-fw `firmware.ino:611, 583-592, 280-290` |
| Teensy watchdog test | missing; planned as H2 for Gate A 30 Oct | `tests/` has only `.gitkeep`; I8 WP rev19.1 section 16.3 |
| STM32H723 host-command watchdog | planned: stale /cmd_vel 200 ms, host heartbeat 500 ms, 3 missed driver heartbeats; no code | I8 WP sections 8, 9, 12; `boards/stm32/` empty |
| ZLAC8015D stop on loss of STM32 communication | planned, no timeout value or CANopen object | P-03 rev19.6 item 14, E-01 rev1.3 section 4, platform-fw#10 (B4 test) |
| motion-not-enabled reporting, Gate A | missing (no E-stop wired, no safety input) | `firmware.ino:368-413`, `docs/safety/motion-safety.md:6` |
| motion-not-enabled reporting, 2.0 | planned: `SAFETY_OK` (PNOZ Y32), `RESET_PRESSED`, failed-reset rule; `/base_controller/status` and `/estop`; message not proposed in interfaces | P-03 rev19.6 item 22, E-01 rev1.3 section 6, I8 WP sections 7.4, 8, 12, platform-fw#17 |
| host stack consumes it | missing: Nav2, docking and twist_mux keep publishing into a locked base; `navigation_status_node` always reports `BASE_LINK_LOST` | `navigation_status_node.py:284, 498-506` |

**Proposed for 2.0** (not Gate A):
- A base status adapter publishes a `base_motion_inhibited` twist_mux lock at 255 with a
  timeout greater than 0, so a dead adapter also engages it.
- It also publishes zeros on `cmd_vel_stop` and cancels navigation and docking goals.
- The firmware should require a zero command before it re-enables motion after RESET, so a held
  command cannot move the robot on re-enable (Evans to confirm).

## 7. Options compared

| Option | Effort | Risk | Fixes from #35 acceptance | Leaves open | Gate A (30 Oct) | 2.0 (rc.1 20 Nov) |
|---|---|---|---|---|---|---|
| (a) Kunal's mux after the monitor | 1 day | Medium: TwistStamped default; Nav2 reset on pause; sim runaway on silence | the dock/undock writer race; per-source timeout | bypassing chain, dock smoothing and guard, stop lock, teleop | not enough | no |
| (a+) (a) + stop lock and teleop at the top, dock rate limit, cone guard on | 3 to 4 days | Medium: teleop and dock still outside the monitor; second smoother or yaml work; the pause still resets Nav2 | the above + stop lock + teleop timeout | dock under an inactive monitor; "no source bypasses the chain" not met | partial | no |
| **(b) arbitration first, monitor never paused, DockStop polygon** (recommended) | 3 to 5 days: prototype exists; move the switch into dock_trigger; tests | Low to medium: dock geometry and near-field sensing must be checked on the bench | single final path, every source limited and monitored, stop lock, teleop timeout, observable guard mode | near-field and rear coverage need the S3 and bench work; swept-body rotation only as the polygon covers it | **yes** | base for 2.0 |
| (c) Nav2 opennav_docking | 2 to 3 weeks: perception plugin, staging, charging semantics | High before 30 Oct; stock docking_server is not remapped through the smoother or monitor either | standard collision check of the projected path near the dock | must still be routed through the mux; DOCKING-NOT-CHARGING decision | no | after rc.1, or a 2.x item |

**Decision:**
- **Gate A:** option (b) as prototyped, after #54 merges.
- **2.0:** the same chain plus the base status lock, measured stop margins on the STM32 base, and
  the dock loop bounded on a timer.
- opennav_docking is a later migration, not a blocker.

## 8. Test plan with pass criteria

Simulation, from `tools/issue35_arbitration/`. Fixtures (sim only, install tree, never `src`):
- F1: body filter with angular bounds in place, ±52 deg around 0;
- F2: dock pose (4.899, 0);
- F3: `obstacle_scan_forward_angle` pi;
- F4: rejected (finding 3).

| # | Test | Pass criterion |
|---|---|---|
| T1 | graph, plain-node and composable | `/cmd_vel` has exactly one publisher, collision_monitor; all cmd topics Twist; behavior_server publishes `cmd_vel_nav` |
| T2 | docking run, `check_run.py` | C1 to C4 PASS; no "Resetting managed nodes"; guard mode NAV -> DOCK -> NAV |
| T3 | priority and handover | dock preempts nav within 1 smoother period; nav back within dock timeout + 0.15 s; 0 leaked samples |
| T4 | stop lock over teleop | final zero within 0.1 s; 0 teleop samples forwarded |
| T5 | publisher loss, teleop, dock, nav | final zero within `velocity_timeout` + 0.05 s; ground-truth travel at most v * timeout + v^2/(2a) + 1 cm |
| T6 | stale or missing scan during docking | collision_monitor stop within `source_timeout` + 0.1 s |
| T7 | guard-mode failure (parameter set rejected) | FAILED published; docking must not start (implementation choice for Kunal) |
| T8 | obstacle in the dock path | stop before contact; needs a sim lidar that sees within 0.40 m, or the bench |
| T9 | no-obstacle docking, 10 runs | 10 of 10 docked, no premature stall, front gap recorded |
| T10 | deliberate fault | `check_run.py` on a main run fails C1 to C3 (done: B0) |
| T11 | Nav2 BT acknowledgement | no follow_path acknowledgement timeout in the T9 series, or a recorded owner decision on `default_server_timeout` |

Supervised bench (after sim, owners present, wheels clear of the floor first):
- T5 and T4 on the Teensy;
- the stop measurements in section 5;
- DockStop against a real obstacle near the dock;
- rear coverage stated as not covered.

## 9. Not verified

- **Real robot.** No real-robot, bench or hardware run. The real profile was read from source
  only.
- **Near-field and lidar stops.** No near-field obstacle stop, no rear coverage, and no lidar
  arrival stop on a fresh measurement were validated (sim lidar blind within 0.40 m).
- **Real base.** Real base deceleration, Teensy `brake()` behaviour, micro-ROS latency and the A1
  minimum range are unmeasured.
- **Undock completion.** Undock completion in sim is blocked by the wall-clock deadline.
- **Fixtures.** The fixture values are not proposed robot values.
- **#54.** It was assumed merged in the prototype, which repeats the same remap.
