# STATE

Read this file before any work in this repository and update it in the same PR as the work.
The PR evidence check fails when STATE.md exists and a PR neither updates it nor says
"no change" with a reason. Keep it under 60 lines; facts only, each with a date or SHA.

## Repository

- Type and lifecycle: software, active
- Readiness: not yet assessed (17 Oct 2026, rollout step a)
- Owner role: software-lead (maintainers.yaml)
- Verification: CI workflows `nav-status-tests`, `drivers-bringup-tests`,
  `description-tests` (added 24 Oct 2026; 29 tests from 1 Nov 2026) and `repository-quality`; no
  `tools/verify.sh` yet. `repository-quality` PASS on main at
  dfd44a5 (16 Oct 2026).

## Current work

| Work package | Branch or PR | Status | Next step |
|---|---|---|---|
| Base mass from the roadmap model (P-03 rev20.5 item 29, one source: config/roadmap_mass_export.yaml, pinned at openamrobot-docs ff8ca57) | description/2026-11-08-base-mass | draft (10 Nov 2026) | base_link mass 49.948 kg, complete base mass 58.163 kg (hub motors 2 x 3.45 kg on the wheel links, provisional: not for dynamics acceptance; castors 4 x 0.3 kg; RPLIDAR S3 0.115 kg), whole-robot mass 87.11 kg; test_base_mass.py checks the expanded URDF and that nothing is counted twice; next: rebase on main, mass review, hub motor rotor/stator split, rerun Gazebo and Nav2, replace with weighed values |
| Shared agent rules v2 (rollout step a) | #72 | in review | merge, then step (b) pin the harness |
| E-01 safety docs | #66 | draft | owner review |
| Devcontainer | #57 | in review | owner review |
| Nav2 behavior_server remap (#35) | #54 | merged (8a7a5b1) | - |
| Gemini 336L base camera simulation | #53 | draft | owner review |
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii). Base `713574f`, head `621cde9`. D-13/D-14 `undock_robot` ledger bugs fixed (60521c3); D-19 fixed in `dock_trigger.py` for every one of its own entry points (32e9a3e); real-`dock_trigger`-subprocess integration tests added for clean-boot/exclusion (2e0b8f3); clean-boot bound tightened to 2.0s + 0.5s documented measurement tolerance; `enabled_producers` derived from `bringup.launch.py`'s own `use_docking` (febdbf2, 083067d); real CancelGoal-rejection now tracked, closing the rejected-cancel `xfail` (da3b0c7); default BTs no longer select `BackUp`, `waypoint_follower` no longer launched (256f25c); `behavior_plugins` no longer registers `backup`/`drive_on_heading`/`assisted_teleop`, DWB `min_vel_x` and `velocity_smoother.min_velocity[0]` both clamped to 0.0 (2d49315); nav-only `_release_blockers()` dock_trigger blocker fixed (b5b3585). CI correction (bf25cd8, 4862576): the 3 live tests that start a real `navigation_launch.py` (`test_navigation_launch_waypoint_follower_server_absent.py`, `test_navigation_launch_bt_xml_resolves_to_no_backup.py`, `test_navigation_launch_behavior_server_direct_actions_absent.py`) were added to `nav-status-tests.yml`'s hosted list and failed/vacuously-passed there: that container's "Install dependencies" step never installs `nav2-bt-navigator`/`nav2-behaviors`/`nav2-controller`/`nav2-planner`/`nav2-collision-monitor`/`nav2-lifecycle-manager`, only `nav2-msgs`/`nav2-velocity-smoother`, so navigation_launch.py's real Nav2 nodes never start there - the bt_xml test correctly failed on it, while the two absence tests kept silently vacuously passing. Pulled all 3 back to local-only; the missing nav2-* apt packages are a bigger dependency-footprint change, owner decision pending, not made unilaterally. This commit: panthera-momagdii's P2 re-review found the launch-sanity check added to the two absence tests only proved `safety_stop_gate_node` started, not `behavior_server` - it could fail to configure/activate while that unrelated service stayed up, and all three/one absences would hold vacuously. Fixed in both by waiting on `spin` (stays registered in `behavior_plugins`, does not reverse the chassis) becoming available as the positive control instead - confirmed this actually catches the failure mode (RED: temporarily dropped `spin` from `behavior_plugins`, both tests failed loudly with the new message; GREEN: reverted, both pass). D-13 status corrected: Alex's `#124` ruling already approved the release-rule rework itself - what remains is implementation (ledger still covers only `undock_robot`, not every enabled producer), not an open scope decision. nikgolobo verified the nav-only fix at `4862576` (confirmed fail-closed/excluded-by-name, same shape as `#125`'s `launch_docking_server` argument) and found a P2: `enabled_producers` is a fail-open lever if a profile misnames it - nothing compared the declared list to what the launch actually starts. Closed for both profiles in active use with 4 new tests in `test_bringup_derives_enabled_producers.py` cross-checking the declared `enabled_producers` string against whether the docking server (today's only `dock_robot`/`undock_robot` goal sender) actually gets included - confirmed via RED/GREEN (temporarily decoupled the docking-launch condition from `use_docking`, confirmed the new test caught it). nikgolobo's stronger option (startup validation inside `safety_stop_gate_node.py` itself: fail-closed at runtime, not just in CI) is deliberately NOT done here - flagged as a follow-up, not bundled into this already-large safety-path PR mid-review. nikgolobo also confirmed `_LEDGER_PRODUCERS = ('undock_robot',)` is coherent with what exists today (UI's Dock/Undock buttons publish `std_msgs/Bool`, never action goals - re-check once a `dock_robot`/`undock_robot` action sender is added). `#125`'s own opennav_docking `/undock_robot` server bypasses `dock_trigger.py`'s D-19 gate entirely once `#125` merges alongside this - 3 options on the table, needs Alex/owner decision; `test_d13_real_dock_trigger_clean_boot_and_exclusion.py` still not in hosted CI (same dependency-footprint tradeoff, owner decision pending); S4/S6/S7a and two-human safety review remain | implement D-13 for the remaining 3 producers; owner/Alex decision on the `#125` bypass, on the startup-validation follow-up, and on expanding `nav-status-tests.yml`'s apt footprint for full-stack hosted CI |
| Named volume `openamrobot_maps` mounted at `/data/maps` in the platform container, unblocks openamrobot-ui #53 | #114 | merged (035f5d8) | none |
| Launch Nav2's map_saver_server so `/map_saver/save_map` is available (new `map_saver_launch.py`, included from `bringup.launch.py` and `sim_bringup_launch.py`) | #115 | merged (f71f9f1) | none |
| Sensor frames to P-03 rev19.7 | #93 | draft | owner review |
| 2.0 base visual meshes from CAD v0.2; GLB one node per part, battery envelope (24-25 Oct 2026) | #94, #95 | merged 25 Oct 2026 (1fc2a23, 4185a0a) | none |
| Recognisable base envelopes: LF105 cells, J401, NUCLEO-144, driver, sensors (25 Oct 2026) | #96 | merged (9a58e0b) | none |
| Base E-stop and RESET in the release 001 button holes, P-03 rev20.0 item 24 (27 Oct 2026) | #100 | merged (131e500) | mechanical owner: confirm the 22.4 mm hole |
| BOM Issue 7.14 and P-03 rev20.2 item 26 references: no click shield, TDK EV_ICM-42688-P (1 Nov 2026) | #104 | merged (b5dcb2d) | none |
| Legacy Gate A camera_link box no longer drawn above the 2.0 top cover; frame kept (1 Nov 2026) | #106 | merged (4926f40) | none |

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
- Harness not pinned; `repository-quality` calls the reusable workflow at `@main` (step b).
- No `tools/verify.sh`; `quality/test` is skipped on main.
- Watchdog report-only on dfd44a5 plus this PR (17 Oct 2026): 46 decisions findings, 15
  public-extract price findings, 3 unpinned actions; fixed or tracked in step (c).

## Not verified

- Real-robot motion, docking and sensor bring-up; simulation and fake hardware only, since dfd44a5.
