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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, prior head `2a40a9b`, this round not yet pushed (11 Nov 2026). Closes panthera-momagdii's second review round on top of the `navigate_to_pose_goal_proxy_node.py` work at `2a40a9b`: (1) `default_nav_to_pose_bt_xml`/`default_nav_through_poses_bt_xml` moved to TOP-LEVEL `bt_navigator.ros__parameters` keys - nested (the prior shape), Jazzy's `NavigateToPoseNavigator`/`NavigateThroughPosesNavigator` never read them and silently fell back to the stock BackUp-bearing trees, which throws in `on_activate` once `backup` is removed from `behavior_plugins` - bt_navigator never reached ACTIVE; new live test proves `/bt_navigator/get_state` reaches ACTIVE with the no-backup tree genuinely loaded, not just that the right string sits in an unread parameter. (2) Two async races in the proxy fixed: outstanding-count/'sent' bookkeeping moved into `handle_accepted_callback` (synchronous with acceptance) instead of the independently-scheduled `_execute` coroutine (P1); a cancel arriving before `send_goal_async` resolves is now tracked and replayed once the real handle exists, instead of silently dropped (P2); CANCELED-without-a-requested-cancel on the proxy's own goal handle now calls `abort()` not `canceled()` (the latter is an invalid transition once the gate can cancel `navigate_to_pose_real` directly, see next). (3) Safety-critical: `safety_stop_gate_node.py`'s `navigate_to_pose` cancel client and status subscription redirected from the proxy to `navigate_to_pose_real` - a goal submitted via RViz/`goal_pose_nav` bypasses the proxy entirely through bt_navigator's own internal `self_client_` (same node-level remap also renames it), so Stop never reached it and Resume could release while it was still driving; new regression `test_a_goal_that_bypassed_the_proxy_via_goal_pose_is_still_cancelled_and_blocks_resume` proves engage cancels it and Resume stays refused until it genuinely clears. Ripple fix applied to every other test file with its own fake cancel/status servers (`test_stop_contract_full_chain.py`, `test_sim_clock_readiness_full_chain.py`). (4) Discovery-grace-period race closed: the 0.5s barrier was anchored to node construction, so slow discovery could let it elapse before a genuinely active retained status sample had a chance to arrive; now anchored to the moment `get_publishers_info_by_topic` confirms a match, with construction time kept only as the fallback ceiling for a producer whose status publisher never exists at all - new regression `test_a_late_publisher_match_gets_its_own_fresh_grace_window` proves a late match still gets its own window instead of inheriting an already-expired one. Full `openamrobot_nav2` regression: 276 passed, 1 skipped, 531 pre-existing flake8/pep257 findings (unchanged by this round), zero leaked processes. Still open, same as before, plus one new item from this round: `goal_pose_nav`-sourced goals are still invisible to the goal-authority ledger itself (the proxy's heartbeat only ever reports its own outstanding count) - safety is covered end to end by (3)'s status-reconciliation fallback, but D-13's preferred low-latency ledger path does not see these goals; closing it needs either routing `goal_pose_nav` through the proxy or having its forwarder(s) call the action directly, not attempted here, needs owner input on which. Also still open: ledger coverage for `navigate_through_poses`/`dock_robot`; `#125`'s `/undock_robot` bypass; interfaces#35 repin; nav2-* packages missing from hosted CI; S4/S6/S7a, two-human safety review, and a real Gazebo session (BotshareAI's 12 Nov ask) | owner/Alex decision on `goal_pose_nav` routing through the proxy vs. forwarder change, the goal-owning-proxy pattern for the 2 remaining producers, `#125` bypass, interfaces pin, hosted-CI nav2 packages; a human-run Gazebo session for the 16 Nov readiness ask |
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
