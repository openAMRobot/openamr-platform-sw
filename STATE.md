# STATE

Read this file before any work in this repository and update it in the same PR as the work.
The PR evidence check fails when STATE.md exists and a PR neither updates it nor says
"no change" with a reason. Keep it under 60 lines; facts only, each with a date or SHA.

## Repository

- Type and lifecycle: software, active
- Readiness: not yet assessed (17 Oct 2026, rollout step a)
- Owner role: software-lead (maintainers.yaml)
- Verification: CI workflows `nav-status-tests`, `drivers-bringup-tests`,
  `description-tests` (added 24 Oct 2026; 29 tests from 1 Nov 2026; job `description-gazebo-tests`
  with pinned gz-sim 8.15.0 runs the friction test, #135, 14 Nov 2026), `sil-tests` (14 Nov 2026) and `repository-quality`; no
  `tools/verify.sh` yet. `repository-quality` PASS on main at
  dfd44a5 (16 Oct 2026).

## Current work

| Work package | Branch or PR | Status | Next step |
|---|---|---|---|
| Base mass from the roadmap model (P-03 rev20.5 item 29, one source: config/roadmap_mass_export.yaml, pinned at openamrobot-docs ff8ca57) | description/2026-11-08-base-mass | draft (10 Nov 2026) | base_link mass 49.948 kg, complete base mass 58.163 kg (hub motors 2 x 3.45 kg on the wheel links, provisional: not for dynamics acceptance; castors 4 x 0.3 kg; RPLIDAR S3 0.115 kg), whole-robot mass 87.11 kg; test_base_mass.py checks the expanded URDF and that nothing is counted twice; next: rebase on main, mass review, hub motor rotor/stator split, rerun Gazebo and Nav2, replace with weighed values |
| Base and lift SIL on virtual CAN: package `openamrobot_sil`, `docs/architecture/sil-topology.md`, workflow `sil-tests` | #139 merged 15 Nov 2026 (69847ab); follow-up claude/optimistic-planck-frfht7: vcan guard requires a device under /sys/devices/virtual/net/ and CAN link type, MTU 16 classic and 72 CAN FD set by `sil_vcan.sh` (15 Nov 2026) | follow-up in review | Renode attach (platform-fw #51), the combined model, re-vendor after platform-fw #53 merges |
| Integrated Digital Twin Baseline 1 integration branch: main 69847ab + #152, #146, #149, #151, #158 (no --ff, 17 Nov 2026); #141/#144/#145 excluded, conflicts with the clock contract need an owner decision (docs/integration/baseline1.md) | integration/baseline1 | draft, do not merge | owners: reconcile #141 to #145 with #151/#158; read the Baseline 1 acceptance run in openamr-upperbody-sw |
| Shared agent rules v2 (rollout step a) | #72 | in review | merge, then step (b) pin the harness |
| E-01 safety docs | #66 | draft | owner review |
| Devcontainer | #57 | in review | owner review |
| Nav2 behavior_server remap (#35) | #54 | in review | owner review |
| Station approach for manipulation goals: station goal checker 0.025 m / 0.05 rad, StationApproach (graceful) controller, StationPlanner (NavFn), behavior_trees/navigate_to_station.xml; default trees name general_goal_checker (Proposed, 17 Nov 2026) | nav/station-approach | draft | software lead: adopt, adapt or reject; NOTICE.md entry for the two Nav2 tree copies (platform lead); reconcile with #91, #92 (same tree file names) |
| Gemini 336L base camera simulation | #53 | draft | owner review |
| Named volume `openamrobot_maps` mounted at `/data/maps` in the platform container, unblocks openamrobot-ui #53 | #114 | approved (panthera-momagdii) | owner merge |
| Launch Nav2's map_saver_server so `/map_saver/save_map` is available (new `map_saver_launch.py`, included from `bringup.launch.py` and `sim_bringup_launch.py`) | #115 | changes requested (panthera-momagdii, use_sim_time + namespace fixed) | re-review |
| Wheel and caster friction reach the SDF (mu1, mu2, fdir1 instead of a raw surface); caster mu 0 is a frictionless slider on fixed joints, not a rolling caster | #135 | draft, changes requested (panthera-momagdii, 10 Nov 2026); hosted Gazebo job added 14 Nov 2026 | re-review; owner writes adopt, adapt or reject |
| Drive wheel loaded radius 0.100->0.098 m (Alex decision 15 Nov 2026: 2mm tyre compression under load, castors rigid) in `robo_urdf.urdf.xacro`/`gazebo_control.xacro`; new `test_ground_contact.py` (drive wheels and castors on one floor plane); `sil.yaml`, docs (`sil-topology.md`, `navigation/01_architecture.md`, docking `03_tf_frames.md`) and `test_sil_gazebo.py` repinned to match (panthera-momagdii's review, fixed in 79b5f15); 221 description tests pass | #146 | changes requested (panthera-momagdii) | re-review 79b5f15; owner: open a contract change request for `decisions.yaml` FRAMES-REP105, which still carries the superseded 0.100 sentence |
| ToF cliff pitch 10 deg down in URDF, SDF and docs (NEAR-FIELD-FRAMES, 20 Oct 2026) | #93 | ready | Cuma: floor-return check at the 23 to 27 Nov bench |
| 2.0 base visual meshes from CAD v0.2; GLB one node per part, battery envelope (24-25 Oct 2026) | #94, #95 | merged 25 Oct 2026 (1fc2a23, 4185a0a) | none |
| Recognisable base envelopes: LF105 cells, J401, NUCLEO-144, driver, sensors (25 Oct 2026) | #96 | merged (9a58e0b) | none |
| Base E-stop and RESET in the release 001 button holes, P-03 rev20.0 item 24 (27 Oct 2026) | #100 | merged (131e500) | mechanical owner: confirm the 22.4 mm hole |
| BOM Issue 7.14 and P-03 rev20.2 item 26 references: no click shield, TDK EV_ICM-42688-P (1 Nov 2026) | #104 | merged (b5dcb2d) | none |
| Legacy Gate A camera_link box no longer drawn above the 2.0 top cover; frame kept (1 Nov 2026) | #106 | merged (4926f40) | none |
| Sensor behaviour profiles (range, rate, noise per sensor frame, each value with its source; unknown stays null): config/sensor_profiles.yaml, test_sensor_profiles.py (14 Nov 2026) | night-shift/sensor-profiles-2026-11-14 | draft | Vineeth and Cuma review; Vineeth: next package reads the values into the Gazebo sensors (not changed here); Evans: IMU and TF-Luna rates |
| Clock contract (D2, Proposed): domains, rules C1 to C6, pause, reset and restart gates in docs/architecture/sil-topology.md section 6; config/clock_inventory.yaml and test_clock_inventory.py keep every clock read listed with its domain (16 Nov 2026) | ws1/clock-contract | draft | software lead accepts or changes the rules; then CTL-017 on Gazebo time through the `_now_ms` hook, node timers on ROS time; ZLAC8015D and canopen_common need an injectable clock in openamr-platform-fw first |
| Clock contract implemented for the base and lift SIL (Gate B): one fed `/clock` clock per process with a steady stall check, CTL-017 on Gazebo time through `_now_ms`, ZLAC8015D feedback and the host stand-in on the fed clock, node timers on ROS time, gate tests for pause, reset, restart and delayed frames; W12 discontinuity tests (forward jump with a report-only jump notice, duplicate and delayed /clock, first clock, clock loss, frames across a reset, health callback hold only), `Stamp` takes time and generation in one `snapshot()` read (review finding), protocol suite 113 tests with 3 strict xfails (17 Nov 2026); #160 first-clock power-on of the ZLAC8015D wrapper, 114 tests with 2 strict xfails | ws1/time-injection (stacked on ws1/clock-contract) | draft | owner: adopt, adapt or reject; platform lead: #159, #160; platform-fw #57 (injectable clock for zlac8015d_emulator and canopen_common), then re-vendor |

| Lift firmware-SIL path and clock/reset/stale-frame contract (P0-C, P0-D, Proposed): docs/architecture/clock-reset-contract.md (R1 to R10, gaps G1 to G6), test/test_lift_fw_sil.py (22 tests, 1 strict xfail #159); CTL-017 node silent before the first /clock, receive queue dropped at a reset, host stand-in new permit boot id at a reset; protocol suite 135 tests with 4 strict xfails (17 Nov 2026); three clocks (simulation, firmware, wall-clock supervision) explicit, CTL-017 node thread step dt and heartbeat on firmware time in the wrapper (mirrors platform-fw #58), test_lift_rtf.py: lift moves without fault at RTF 0.1 and 0.2, protocol suite 139 tests; merged ws1/time-injection 607a496 (#160), 140 tests with 3 strict xfails (17 Nov 2026) | ws1/lift-fw-sil (stacked on ws1/time-injection) | draft | owner: adopt, adapt or reject; platform lead: RPDO1 producer epoch proposal (#159), #58 then re-vendor |

## Decisions this repository implements

FRAMES-REP105, NEAR-FIELD-FRAMES, NAV-LIDAR, CAMERAS, IMU-TOPIC-OWNERSHIP, DRIVE-TRACK,
DOCK-NO-CONTACTS, DOCKING-NOT-CHARGING.

## Known gaps

- Visual meshes are generated by `tools/cad/step_to_description_meshes.py` (24 Oct 2026);
  the B2 hub motor STEP it reads is only on platform-hw PR #28. Open CAD drift: TF-Luna
  pitch and height, IMU height, ToF openings (build/cad_checks.md).
- Battery cell-group envelope (314 x 140 x 210 mm, provisional; project-lead decision 25 Oct
  2026) behind the centre bracket fits CAD v0.2 with at least 2 mm (build/cad_checks.md); BMS,
  power distribution, fuse and wiring are not modelled (free space reported). Register BOM-ISSUE-IN-FORCE
  still names BOM Issue 7.10; the sources here cite BOM Issue 7.14 with its 1 Nov changes (1 Nov 2026).
- IMU board envelope (1 Nov 2026) is still the MIKROE-4237 outline from the STEP; the TDK
  EV_ICM-42688-P outline is not modelled; imu_link pose and yaw unchanged (P-03 rev20.2 item 26).
- Base E-stop and RESET (27 Oct 2026) are placeholders from datasheet dimensions in the
  release 001 button holes (22.5 mm in CAD v0.2); the Schneider STEP models and the XB4 hole
  tolerance (22.4 mm, unconfirmed) are not verified; the mechanical owner confirms the hole.
- Sensor profiles (14 Nov 2026): Gazebo sensors do not read config/sensor_profiles.yaml yet; unknown values (LiDAR, camera and IMU bias and latency, ODR, beam angle, range noise) stay null until a datasheet or a bench test fills them; the RPLIDAR S3 and Gemini 336L datasheets are not in the datasheet index.
- Harness not pinned; `repository-quality` calls the reusable workflow at `@main` (step b).
- No `tools/verify.sh`; `quality/test` is skipped on main.
- Clock contract (17 Nov 2026, ws1/time-injection): 3 inventory sites still `to_change`, all vendored (canopen_common tick, dt and SDO timeout; ZLAC8015D receipt time and 2000h link loss) until platform-fw #57; a Gazebo pause over 250 ms trips the emulated 2000h link loss on host time. Renode skew not reported yet. Open (xfail): #159 frames from before a reset act after it. #160 closed in the wrapper (ZLAC8015D unpowered before the first /clock, 17 Nov 2026, Proposed; platform lead to confirm).
- Watchdog report-only on dfd44a5 plus this PR (17 Oct 2026): 46 decisions findings, 15
  public-extract price findings, 3 unpinned actions; fixed or tracked in step (c).

## Not verified

- SIL layer (14 Nov 2026): emulators and Gazebo only. No firmware in Renode, no real CAN bus, drive, lift or BMS.
- Real-robot motion, docking and sensor bring-up; simulation and fake hardware only, since dfd44a5.
