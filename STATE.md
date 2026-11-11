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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI - BotshareAI represents owner/Alex-level decisions). Base `713574f`, head is this commit (fill exact SHA after push). D-13 implementation now covers `navigate_to_pose` end to end, closing BotshareAI's "one platform node that owns the goal handles" requirement for it: new `navigate_to_pose_goal_proxy_node.py` claims the public `navigate_to_pose` action name (bt_navigator remapped to `navigate_to_pose_real` in both navigation_launch.py GroupActions), relays every goal/feedback/result/cancel to the real server, and reports sent/done to the ledger via `goal_authority_events` - every existing sender (dock_trigger.py's own ActionClient, openamrobot-ui's direct one) is transparently redirected, no change needed in either, which also closes panthera's "untracked client" finding on the UI's A1 branch. `_LEDGER_PRODUCERS` now `('undock_robot', 'navigate_to_pose')`. 4 new tests prove goal relay, feedback/result passthrough, and - the single safety-critical property - that a cancel on the proxy's own goal handle actually reaches and cancels the real goal, not just the wrapper; plus a live launch test proving the remap takes effect against a real `navigation_launch.py`. Ripple fixes applied to every full-chain test file that previously treated `navigate_to_pose` as legacy-status-content (`test_safety_stop_gate_node.py`, `test_stop_contract_full_chain.py`, `test_sim_clock_readiness_full_chain.py`, `test_d13_real_dock_trigger_clean_boot_and_exclusion.py`) - all now heartbeat it instead; one test (`test_release_refused_when_the_cancel_is_rejected_and_status_was_already_clear`) retargeted to `navigate_through_poses` since the ledger mechanism structurally does not have the content-vs-cancel race the legacy path does. Also closed this round (panthera's P1/P2 re-review, before the proxy work): heartbeat wire format now REQUIRES its outstanding count and rejects it if negative, fail-closed on both; `_release_blockers()` reconciles the retained action-status sample for ledger producers against a gate-restart-after-producer-restart race, with an explicit discovery-grace-period barrier (not T_READY) closing the remaining startup race panthera found; a new live test proves a negative `cmd_vel` is clamped to zero at `velocity_smoother`'s real output (BotshareAI's session-profile ask). Full regression: 286 passed, 1 skipped, zero leaked processes. #91/#92 (Kunal's own, same repo) independently reintroduced D-19's BackUp/velocity-limit regression before #90's fix existed - both fixed (BackUp removed, pinned tests inverted, velocity limits clamped) and merged forward onto current main (STATE.md conflict resolved), pushed. `openamrobot-interfaces#35`'s failing build fixed (stale compatibility snapshot + missing CONTRACT_VERSION bump for StopState.msg), confirmed passing in hosted CI. Still open: ledger coverage for `navigate_through_poses`/`dock_robot` (same proxy pattern, not yet built - fast-follow); `#125`'s `/undock_robot` bypass (3 options, needs Alex); interfaces#35 pinned to a fork commit, repin once it merges (nikgolobo); nav2-* packages missing from hosted CI; S4/S6/S7a, two-human safety review, and a real Gazebo session proving clean-boot/manual-drive/Nav2-goal/Stop/Resume (BotshareAI's 12 Nov ask) all remain open - none of those were attempted here | owner/Alex decision on the goal-owning-proxy pattern for the 2 remaining producers, `#125` bypass, interfaces pin, hosted-CI nav2 packages; a human-run Gazebo session for the 16 Nov readiness ask |
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
