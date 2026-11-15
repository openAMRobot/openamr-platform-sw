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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed). Base `713574f`, exact head `971cd207148289a762a45584e31b6faba75ef9f6` (15 Nov 2026). Hosted CI last confirmed green on `162a19a` (panthera's review on `d29dfc0` fully answered there). **This push builds the forwarder Alex's 14 Nov owner decision authorized (item 1):** new `goal_pose_forwarder_node.py` replaces the nav-only profile's old `topic_tools relay` - every `/goal_pose` now becomes a real `NavigateToPose` goal on the PUBLIC `navigate_to_pose` action (through the proxy), not bt_navigator's internal `self_client_` (the exact defect Nikolai's 14 Nov Gazebo repro confirmed). `navigation_launch.py`'s `goal_pose` remap moves to `goal_pose_bt_navigator_unused` (nothing publishes there in either profile - orphans dock_trigger's own forward target, deliberately out of scope, #124's work). 5 new hosted tests (`test_goal_pose_forwarder_full_chain.py`, added to `nav-status-tests.yml`): Nikolai's repro, Stop-rejects-goal, Stop-during-goal-then-Resume, two-poses-one-active, zero-publishers-on-orphaned-topic. RED-verified standalone against the old relay (zero status samples ever arrive). Also added: `gate-crash-angular-repeat.yml` (workflow_dispatch, item 6's 20-run ask) - built and smoke-tested at 3 reps (0.3996/0.3998/0.4288s), not yet dispatched on hosted CI for the real 20. 292 local tests passing, zero leaked processes. **Unrelated, pre-existing, NOT fixed here (scope frozen):** `test_ui_cmd_vel_relay_node.py`'s `TestStaleCommandsAreDropped` (4 tests) fails even in isolation on this head; neither that file nor its node was touched by any commit in this PR - flagged for someone else to pick up. **Still open, not code:** `dock_robot` proxy waits on `#125`/`#78`; #136 conflicts at `navigation_launch.py:161`, Alex's order puts #136 first; two-human safety review and the Gazebo session remain un-run. | Kunal: dispatch the 20-run angular workflow and post min/median/max; owner/Alex: #136 merge-order, second human review, Gazebo session (Nikolai), approval (Mohamed) |
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
