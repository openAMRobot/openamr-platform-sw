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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed). Base `713574f`, exact head `b9d49d0` (14 Nov 2026). **Alex's 13 Nov owner-decision review (7 items) is the current gating list; items 3, 4, 5 and 6 are done in this push, items 1 and 2 are owner/dependency blocked, item 7 is explicitly deferred to #113:** (3) `/cmd_vel`-vs-`collision_monitor` singleton check added to `tools/issue35_arbitration/stop_lock_invariants.py`, with `--no-check-cmd-vel` for the gate-standalone hosted-CI invocation that never starts `collision_monitor`. (4) `nav-status-tests.yml` now installs `nav2-bt-navigator`/`behaviors`/`controller`/`planner`/`collision-monitor`/`lifecycle-manager`/`opennav-docking` plus, after the FIRST push of this list made hosted CI fail all 5 re-enabled tests ("bt_navigator never came up") - root-caused via a clean `ros:jazzy-ros-base` container repro, not guessed: without `nav2-smoother`, `ros2 launch navigation_launch.py` throws "package 'nav2_smoother' not found" and aborts before spawning ANY node, which reads exactly like a timing problem but is a missing executable package; `nav2-rotation-shim-controller`/`nav2-dwb-controller`/`nav2-smac-planner` are pluginlib-loaded at runtime (not hard apt deps of nav2-controller/nav2-planner), so their absence FATALs controller_server's configure and makes lifecycle_manager abort the whole bringup, which the "...reaches_active..." test needs working - `nav2-smoother`/`nav2-rotation-shim-controller`/`nav2-dwb-controller`/`nav2-smac-planner` added; all 7 test cases across the 5 files confirmed passing in that same clean container with this complete package set. Local-only verification (not yet re-confirmed on hosted CI at this exact head) - check `gh api .../commits/<head>/check-runs` before trusting this is still green. (5) heartbeat readiness-poll and dual lock-enforcement tests - done in the prior push, unchanged. (6) new `test_gate_crash_full_chain_cmd_vel_bound.py`: real killable gate subprocess + real twist_mux/velocity_smoother/collision_monitor (the last needing a faked TF tree and `/scan_filtered`/`published_footprint`, confirmed by direct experiment that collision_monitor forwards nothing at all without them); measured last-nonzero-`/cmd_vel`-after-kill at ~0.33s, inside the 0.5s bound; RED-verified against an artificially tight bound; zero leaked processes, including run alongside the two adjacent full-chain test files. **Still open, owner/dependency decisions, not code:** (1) Alex wants `nikgolobo` to reproduce the `goal_pose_nav`/RViz-goal-bypasses-the-proxy scenario in Gazebo (3 CLI commands, ~15 min) before any forwarder-node fix is written - not yet reproduced as of 14 Nov. (2) `dock_robot`'s own proxy still waits on `#125` (open, `mergeable_state: behind`, not yet merged) then `#78` (assigned to `fridayali`, in progress elsewhere). Two-human safety review and the human Gazebo Stop/Resume session remain un-run, as always. | owner/Alex: `goal_pose_nav` reproduction result from `nikgolobo`; `dock_robot` proxy waits on `#125`/`#78`; two-person safety review and human Gazebo session remain |
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
