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
| Base and lift SIL on virtual CAN: package `openamrobot_sil` (ZLAC8015D, CTL-017 and Daly BMS emulators on `vcan_drive`, `vcan_aux`, `vcan_bms`, bridged to Gazebo; host test client; `sil.launch.py`), `docs/architecture/sil-topology.md`, workflow `sil-tests` (14 Nov 2026) | claude/optimistic-planck-frfht7 | draft | owner: adopt, adapt or reject; then Renode attach (platform-fw #51) and the combined model |
| Shared agent rules v2 (rollout step a) | #72 | in review | merge, then step (b) pin the harness |
| Motion limits per layer (SPEED-CEILING 0.5 m/s, MASS-LEDGER D-29 1.5 m/s2): `docs/architecture/motion_limits.md` and `openamrobot_sil/test/test_motion_limit_layers.py` (17 tests, sil-protocol count 57 to 74); 7 rows break the ceiling, declared against #172; no parameter changed, 19 Nov 2026 | sim/motion-limit-layers | draft | Nav2 owners and platform lead: decide the values in #172 (after #90) |
| E-01 safety docs | #66 | draft | owner review |
| Devcontainer | #57 | in review | owner review |
| Nav2 behavior_server remap (#35) | #54 | merged (8a7a5b1) | - |
| Gemini 336L base camera simulation | #53 | draft | owner review |
| twist_mux command arbitration + stop gate (#35/#119) | #90 | close at `2dd1c74` - Alex's two P1 items (mux-config drift test, reject-goal-during-Stop) landed at `175ed32`; Mohamed's 20 Nov review of that head found the reject fix used `/safety/stop_lock` (Bool) instead of `/safety/stop_state` (StopState) like `goal_pose_forwarder_node.py`'s own rule, plus P2 #3: `safety_stop_gate_node.py`'s `_cancel_all_goals()` cancels the real server and the public proxy name as two independent, equal-distance IPC calls with no ordering, so the real goal could finish `CANCELED` before the public cancel set `is_cancel_requested`, reporting `ABORTED`. Both fixed at `2dd1c74`: both proxies now use `/safety/stop_state` (matching the forwarder exactly) with a genuine full-chain test (real gate + real proxy + fake real server, proving rejection and that the fake server's execute path is never touched); the cancel race is closed with a bounded wait (0.5s, polled via an rclpy Future + one-shot timer) before falling back to `abort()`, with a deterministic RED/GREEN reproduction in both proxy test files. Also finalizes the #164 tree merge-order gap the 18 Nov review flagged (bounded-clear `ClearCostmapAroundRobot` + `goal_checker_id="general_goal_checker"` on both `*_no_backup.xml` trees) - #91/#92 closed, their fix now carried here. Verified: the four touched test files pass repeatedly; zero new flake8/pep257 findings against the pre-change baseline. `producer-tests` on `2dd1c74` read red from an intermittent hang (confirmed by a separate review: same hang on `0bb2e97`/`4fad3d8`, `175ed32` passed clean - not introduced by this round) that hit the 30-minute timeout; a rerun is expected to pass. Open, separately: `docs/safety/02_limits_and_watchdog.md:19`'s `min_velocity` still reads `[-0.20, 0.0, -0.5]` (pre-D-19, allows reverse) against `nav2_params.yaml:379`'s real, current `[0.0, 0.0, -0.5]` (no reverse, Kunal's own 10 Nov change) - docs/safety files are owner-edit only, paste-text given separately. | Mohamed: re-review and approve on `2dd1c74`; owner: rerun the hung `producer-tests` job, apply the `02_limits_and_watchdog.md` correction, second-human review, Nikolai's full Gazebo session (UI pass, Nav2 restart, gate restart still unmeasured) |
| Named volume `openamrobot_maps` mounted at `/data/maps` in the platform container, unblocks openamrobot-ui #53 | #114 | merged (035f5d8) | none |
| Launch Nav2's map_saver_server so `/map_saver/save_map` is available (new `map_saver_launch.py`, included from `bringup.launch.py` and `sim_bringup_launch.py`) | #115 | merged (f71f9f1) | none |
| Wheel and caster friction reach the SDF (mu1, mu2, fdir1 instead of a raw surface); caster mu 0 is a frictionless slider on fixed joints, not a rolling caster | #135 | draft, changes requested (panthera-momagdii, 10 Nov 2026); hosted Gazebo job added 14 Nov 2026 | re-review; owner writes adopt, adapt or reject |
| ToF cliff pitch 10 deg down in URDF, SDF and docs (NEAR-FIELD-FRAMES, 20 Oct 2026) | #93 | ready | Cuma: floor-return check at the 23 to 27 Nov bench |
| 2.0 base visual meshes from CAD v0.2; GLB one node per part, battery envelope (24-25 Oct 2026) | #94, #95 | merged 25 Oct 2026 (1fc2a23, 4185a0a) | none |
| Recognisable base envelopes: LF105 cells, J401, NUCLEO-144, driver, sensors (25 Oct 2026) | #96 | merged (9a58e0b) | none |
| Base E-stop and RESET in the release 001 button holes, P-03 rev20.0 item 24 (27 Oct 2026) | #100 | merged (131e500) | mechanical owner: confirm the 22.4 mm hole |
| BOM Issue 7.14 and P-03 rev20.2 item 26 references: no click shield, TDK EV_ICM-42688-P (1 Nov 2026) | #104 | merged (b5dcb2d) | none |
| Legacy Gate A camera_link box no longer drawn above the 2.0 top cover; frame kept (1 Nov 2026) | #106 | merged (4926f40) | none |
| Sensor behaviour profiles (range, rate, noise per sensor frame, each value with its source; unknown stays null): config/sensor_profiles.yaml, test_sensor_profiles.py (14 Nov 2026) | night-shift/sensor-profiles-2026-11-14 | draft | Vineeth and Cuma review; Vineeth: next package reads the values into the Gazebo sensors (not changed here); Evans: IMU and TF-Luna rates |

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
- Watchdog report-only on dfd44a5 plus this PR (17 Oct 2026): 46 decisions findings, 15
  public-extract price findings, 3 unpinned actions; fixed or tracked in step (c).

## Not verified

- SIL layer (14 Nov 2026): emulators and Gazebo only. No firmware in Renode, no real CAN bus, drive, lift or BMS.
- Real-robot motion, docking and sensor bring-up; simulation and fake hardware only, since dfd44a5.
