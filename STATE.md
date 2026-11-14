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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, exact head `cc8b379` (panthera-momagdii's ninth review, two P1s - both fixed in this same push). **Hosted CI, exact head `7c70f45` (panthera-momagdii's own independent run, prior to this round's fix):** `nav-status-tests` run 43196520886 passed 274 tests (including both lock tests); `drivers-bringup-tests` run 43196521003 passed 21 launch tests; `repository-quality` run 43196521324 passed (`quality/test` skipped, expected). **NOT RUN:** full Nav2 launch integration, Gazebo, two-human safety review, physical acceptance. **Author-reported local evidence at `cc8b379`, not independently re-run:** 293 passed/1 skipped, 531 pre-existing lint findings (unchanged), zero leaked processes. **This round fixes both P1s panthera found in the pending-cancel-race fix itself:** (1) the heartbeat timer's own phase is fixed at construction, and the startup heartbeat attempt is a no-op whenever the real server is not ready yet, so the first successful heartbeat could land up to a full 2.0s period after the real server actually became ready - missing D-13's literal "within 2s" clean-boot bound (the prior round's 2.5s/3.0s test margins were covering for exactly this). Fixed: both proxies now poll `server_is_ready()` on a short 0.1s period and fire the first heartbeat the moment it turns true, self-cancelling once it has; the regular 2.0s periodic timer takes over from there. Both proxies' clean-boot regressions tightened back to the literal 2.0s the ruling requires - confirmed reliable across eight repeated local runs; D-13 row 1 in the PR body can now say MET, not "met within 3.0s". (2) `test_cancel_and_real_handle_install_are_mutually_exclusive` only proved `_handle_cancel` waits on the lock - it would stay green even if `_execute`'s own install-and-check moved back outside the lock. Added a second regression per proxy (`test_real_goal_handles_is_only_ever_written_while_the_lock_is_held`) that swaps `_real_goal_handles` for a dict recording any write made while the lock is not held, then drives a real goal through `_execute` end to end; RED-confirmed against a simulated regression (install moved outside the lock: the new test failed, the old one still passed, exactly the gap panthera described). Still open, unchanged, for the owner (Alex, being asked directly per panthera's own review): `goal_pose_nav` routing through the proxy vs. keeping status reconciliation; `dock_robot`'s own proxy (needs `#125`) vs. merging #90 now with D-13 row 1 recorded NOT MET for docking-enabled profiles; Nav2 packages in hosted CI so the two live launch-remap tests run hosted; two-human safety review with the platform lead; the human Gazebo Stop/Resume session | owner/Alex decision (panthera is asking directly) on: `goal_pose_nav` routing through the proxy vs. forwarder-side event path; `dock_robot`'s own proxy vs. merging #90 now with that gap recorded open; hosted-CI Nav2 packages; a human-run Gazebo session, two-person safety review, and physical robot acceptance |
| Named volume `openamrobot_maps` mounted at `/data/maps` in the platform container, unblocks openamrobot-ui #53 | #114 | merged (035f5d8) | none |
| Launch Nav2's map_saver_server so `/map_saver/save_map` is available (new `map_saver_launch.py`, included from `bringup.launch.py` and `sim_bringup_launch.py`) | #115 | merged (f71f9f1) | none |
| ToF cliff pitch 10 deg down in URDF, SDF and docs (NEAR-FIELD-FRAMES, 20 Oct 2026) | #93 | ready | Cuma: floor-return check at the 23 to 27 Nov bench |
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
