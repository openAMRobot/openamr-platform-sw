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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, prior head `18c9e84` (panthera-momagdii's fourth review round; this round's D-19 fix below is on top of it, not yet pushed, 11 Nov 2026 - exact head SHA to follow once pushed). **Hosted CI, exact head `18c9e84` (panthera-momagdii's own independent run):** `nav-status-tests` run 43062286929 passed 261 tests + the live stop-lock invariant; `drivers-bringup-tests` passed its 18 launch tests + both lint jobs; `repository-quality` passed (`quality/test` skipped, expected). **NOT RUN at `18c9e84` (explicitly, not merely omitted):** no human Gazebo session, no two-person safety review, no physical robot acceptance - none of those were performed by anyone at this head. **Author-reported local evidence at `18c9e84`, not independently re-run by a reviewer:** 277 passed, 1 skipped, 531 pre-existing flake8/pep257 findings. **This round, fixing panthera-momagdii's P1 on `18c9e84`:** `bringup.launch.py`'s `use_docking` default was still unconditionally `'true'` for BOTH real and sim profiles - #125 (`fix/123-start-docking-server`, head `3713a428`) changes the REAL default to off (D-19, Alex's 11 Nov owner decision: opennav_docking exposes a direct `/undock_robot` the `#90` stop-gate constant cannot gate), and panthera's finding was that #90 must not merge alone with real docking still defaulting on. Carried #125's exact default-resolution fix onto this branch directly (`use_docking` now declares an empty default, resolved in `launch_setup` to `'true'` in sim / `'false'` on real) rather than waiting on merge-order coordination between the two PRs - this does not replace #125 itself (still open separately, same change, different branch) but removes the specific "#90 merges alone with real docking on" risk panthera flagged. 3 new regressions in `test_bringup_derives_enabled_producers.py` (`test_sim_default_use_docking_enables_all_four_producers`, `test_real_default_use_docking_excludes_both_dock_producers_by_name`, `test_real_default_use_docking_launches_no_docking_server`) prove the empty default resolves correctly per profile; RED-confirmed against the pre-fix file for the sim direction (the real direction happened to pass either way, since empty-string was already falsy by coincidence - kept as documentation coverage, not a RED-sensitive regression for this specific bug). `use_docking` default documented consistently in `README.md`, `docs/real_robot/01_bringup.md`, `docs/navigation/05_goal_routing.md` (all previously said "default true" for both profiles). Full local `openamrobot_bringup` regression: 16 passed, 1 skipped (up from 13); full local `openamrobot_nav2` regression unchanged at 277 passed, 1 skipped (confirming no cross-package regression); zero leaked processes. Still open, unchanged: D-13 for `navigate_through_poses`/`dock_robot`; the `goal_pose_nav` ledger-visibility gap (owner decision needed: route through the proxy vs. forwarder-side event); `#125` itself still needs its own merge (this round only carries its default-value fix onto #90, it does not close #125); interfaces#35 fork pin; nav2-* packages missing from hosted CI; two-human safety review and the Gazebo/physical acceptance sequence | owner/Alex decision on `goal_pose_nav` routing through the proxy vs. forwarder change and on #125/#90 merge order (this round reduces but does not eliminate that coordination need); the goal-owning-proxy pattern for the 2 remaining producers; interfaces pin; hosted-CI nav2 packages; a human-run Gazebo session for the 16 Nov readiness ask; a reviewer re-run of this round's local-only evidence once pushed |
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
