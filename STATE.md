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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, exact head `25baf3b` (panthera-momagdii's eighth review, one new P2 plus an evidence-staleness note - this round's fix below is on top of it, not yet pushed, 12 Nov 2026). **Hosted CI, exact head `25baf3b` (panthera-momagdii's own independent run):** `nav-status-tests` collected 272, passed 272; `drivers-bringup-launch-tests` passed 21, 0 skipped; `repository-quality` passed (`quality/test` skipped, expected). Live launch-remap tests NOT in the hosted pytest command (local-only, as before). **NOT RUN:** full Nav2 launch integration, Gazebo, two-human safety review, physical acceptance. **Author-reported local evidence, not independently re-run:** 289 passed/1 skipped. **This round fixes the new P2 and the evidence staleness it also flagged:** `_handle_cancel()`'s lookup-or-add (`_real_goal_handles.get(key)` -> `_pending_cancels.add(key)`) and `_execute()`'s install-and-check (`_real_goal_handles[key] = ...` -> `key in _pending_cancels`) in both proxies were two separate, unsynchronized steps - a cancel could observe no real handle yet, pause, then `_execute()` could install the handle and observe no pending cancel yet, and both sides would finish believing the other handles it; the cancel is silently lost and the real goal keeps running. Fixed by extending the SAME `threading.Lock` already used for the outstanding/heartbeat race (seventh review) to cover both critical sections in `navigate_to_pose_goal_proxy_node.py` and `navigate_through_poses_goal_proxy_node.py`. New deterministic regression in both proxy test files (`test_cancel_and_real_handle_install_are_mutually_exclusive`) holds the lock to simulate being inside either critical section and proves a concurrent `_handle_cancel()` call blocks until released; RED-confirmed against the unlocked code. Evidence staleness: the PR's own **Head:** field and its top "Not attempted" paragraph both still named the prior head/said `navigate_through_poses` was not attempted/said `#91`/`#92` were merged - all corrected (`#91`/`#92` confirmed still open via the GitHub API - that claim was simply wrong, not just stale). panthera also named STATE.md's `#114`/`#115` rows and a `#125` row as stale; checked directly against this branch's own STATE.md at the exact head reviewed - `#114`/`#115` already read "merged" correctly, and there is no `#125` row in this branch's STATE.md at all (that row lives on `#125`'s own branch) - those two specific claims do not hold against this branch's actual content, left unchanged. Full `openamrobot_nav2` regression: 291 passed, 1 skipped, 531 pre-existing flake8/pep257 findings (confirmed unchanged), zero leaked processes. Still open, unchanged: the `goal_pose_nav` internal-bypass owner decision; `dock_robot` not yet behind a proxy (needs `#125` first); the `#91`/`#92` BT-tree Integration Gate item; `#125`'s own merge; interfaces#35 fork pin; nav2-* packages missing from hosted CI; the 2s-vs-2.5s/3s clean-boot timing gap (owner decision needed); two-human safety review and the Gazebo/physical acceptance sequence | owner/Alex decision on: `goal_pose_nav` routing through the proxy vs. forwarder-side event path (D-13 approval blocker); the clean-boot timing bound; the `#91`/`#92` BT-tree merge order; `dock_robot`'s own proxy (needs #125); `#125`'s own merge; interfaces pin; hosted-CI nav2 packages; a human-run Gazebo session, two-person safety review, and physical robot acceptance |
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
