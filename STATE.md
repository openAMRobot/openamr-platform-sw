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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed). Base `713574f`, exact head `5c71dd4aef0c05194b2e156f4a72af54189aee2b` (14 Nov 2026). Hosted CI confirmed green on `d29dfc0` (284 tests, 0 skipped - items 3/4/5/6's CI changes all verified for real on GitHub, not just locally). **This push answers panthera's review on `d29dfc0`:** `_pending_cancels` (both proxies) now has the same lock-instrumentation regression `_real_goal_handles` already had - RED-verified by moving `add()` outside the lock, catches it. `_poll_for_first_heartbeat` (both proxies) no longer makes its own separate `server_is_ready()` check ahead of `_heartbeat()`'s own - collapsed to one check, `_heartbeat()` is now the single source of truth and returns whether it actually published; new regression RED-verified against the literal reintroduced old code. `stop_lock_invariants.py`'s `/cmd_vel` check is now actually RUN with `check_cmd_vel=True` against the real chain (new tests in `test_gate_crash_full_chain_cmd_vel_bound.py`), plus a case proving a second `/cmd_vel` publisher is caught. That same file now also covers rotation (`angular.z`, shipped max 0.5 rad/s / -3.0 rad/s² decel) alongside the existing linear case - the tighter one panthera calculated at ~0.467s vs ~0.3s linear, both still inside the 0.5s bound. `conftest.py`'s `OPENAMR_REQUIRE_TWIST_MUX` hard-require list now also covers `nav2_collision_monitor`/`tf2_ros`, closing the skip-behind-green-job gap for that same test file. All of the above: 291 local tests passing (full nav-status-tests.yml list), zero leaked processes - hosted re-confirmation at this exact head not yet run, check `gh api .../commits/5c71dd4aef0c05194b2e156f4a72af54189aee2b/check-runs` before trusting it. **Process fix per panthera's own ask:** this head is the last CODE commit of the push; STATE.md's own update is a separate, STATE-only commit on top naming this exact SHA, not bundled with it. **Alex's 13 Nov review, items 1/2/7 still open (3/4/5/6 done, confirmed above):** (1) `nikgolobo` REPRODUCED the `goal_pose_nav` routing defect on `d29dfc0` (14 Nov, Gazebo, nav-only) - confirmed: UI status/cancel see nothing, robot travelled 0.27-0.47m uncancelled across two runs. The forwarder-node fix itself (replace the `goal_pose`→`goal_pose_nav` relay) is not started - next real work once scoped. (2) `dock_robot`'s own proxy still waits on `#125` (open, `mergeable_state: behind`) then `#78` (assigned `fridayali`). Item 7 stays deferred to #113. **New from this review, not yet actioned:** #136 conflicts with this PR on `navigation_launch.py` line 161 (`UnlessCondition(use_composition)` vs this branch's `PythonExpression` line) - Alex's merge order puts #136 first; after it lands, merge main here and take #136's line. Two-human safety review and the Gazebo Stop/Resume session remain un-run. | owner/Alex: #136 merge-order conflict at `navigation_launch.py:161`; `goal_pose_nav` forwarder fix (scoped, not started); `dock_robot` proxy waits on `#125`/`#78`; two-person safety review and human Gazebo session remain |
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
