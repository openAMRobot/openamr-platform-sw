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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, prior head `8a290d8` (panthera-momagdii's sixth review; this round's fixes below are on top of it, not yet pushed, 12 Nov 2026 - exact head SHA to follow once pushed). **P1, a real regression in last round's own fix**: the "fail closed indefinitely once a status publisher is matched but has sent no sample" change (914a2dd, meant to close a slow-discovery race) broke D-13 test 1 - a real rclpy action server (bt_navigator) publishes NO status at all until its first goal state change (rclcpp_action Jazzy `server.cpp:609,:687`; Nikolai measured 0 status messages in 12s on an idle stack), so "matched but no sample" is the PERMANENT state for `navigate_to_pose` on a genuine clean boot, not a transient race - the indefinite block made a real clean boot indistinguishable from the bug it was meant to catch, and D-13 explicitly requires clean-boot Resume within ~2s with real servers and no goal ever sent. Reverted `safety_stop_gate_node.py`'s discovery-grace branch to the bounded-wait shape (trust the ledger's IDLE read once the match-anchored grace window elapses, same as `f482cfe`). The actual remaining race is closed at the proxy instead, exactly as panthera suggested: `navigate_to_pose_goal_proxy_node.py` now fires one heartbeat immediately at construction (D-13: "that node knows zero goals active, including at boot") and suppresses heartbeats while `navigate_to_pose_real` is not ready (`ActionClient.server_is_ready()`), so an absent real server ages the ledger entry back to UNKNOWN instead of this proxy ever reporting a false "idle". New test `D13TestOneCleanBootResumeTestCase` (in `test_navigate_to_pose_goal_proxy_node.py`) is D-13's own "test 1" end to end - real `ActionServer`/`ActionClient` pair, gate, and proxy, no goal ever sent, Resume within the heartbeat bound; RED-confirmed against the pre-revert code (reproduces panthera's exact repro message), GREEN after. The test that encoded the now-reverted (wrong) behavior (`test_release_stays_refused_past_the_grace_window_if_no_sample_ever_arrives`) is renamed and inverted to assert the correct (bounded-wait) outcome instead of deleted outright, so the bounded-wait shape itself stays covered. **P2, stale text**: `safety_stop_gate_node.py`'s and `test_d13_real_dock_trigger_clean_boot_and_exclusion.py`'s comments describing the proxy heartbeat as sufficient clean-boot evidence on its own, or describing `navigate_to_pose` as still blocking every clean boot, both corrected to reflect the two fixes above together. **P2, Integration Gate** (repo rules ask for this section explicitly): `navigation_launch.py`'s `param_substitutions` unconditionally rewrites both `default_*_bt_xml` keys to the `*_no_backup.xml` files regardless of what `nav2_params.yaml` itself says - after `#91`/`#92` land their own `navigate_to_pose.xml`/`navigate_through_poses.xml` (bounded `ClearCostmapAroundRobot`, named inline Waits), this rewrite will silently win and those trees will never load, with no test catching it, since `#90` also carries `#91`'s own `status_trackers.py` byte-for-byte and all three PRs touch `navigation_status_node.py`. NOT resolved unilaterally here (which tree set wins is an owner/Alex decision, not this PR's to make) - recorded as an explicit open item: if `#90` merges first, `#91`/`#92` must repoint the rewrite at their own trees, delete the no-backup copies, and update `test_navigation_launch_bt_xml_resolves_to_no_backup.py` as part of their own merge. Full `openamrobot_nav2` regression: 277 passed, 1 skipped (1 further test flaked under full-suite CPU load and passed cleanly in isolation - known pre-existing flakiness in a live-launch test, not a regression), 531 pre-existing flake8/pep257 findings (confirmed unchanged), zero leaked processes. Still open, unchanged: `navigate_through_poses`/`dock_robot` not yet behind the same proxy pattern (panthera is taking the `goal_pose_nav`-routing question to Alex); `#125`'s own merge; interfaces#35 fork pin; nav2-* packages missing from hosted CI; two-human safety review and the Gazebo/physical acceptance sequence | owner/Alex decision on the `#91`/`#92` BT-tree merge order (Integration Gate above) and on `goal_pose_nav` routing through the proxy vs. forwarder-side event path; the goal-owning-proxy pattern for `navigate_through_poses`/`dock_robot`; `#125`'s own merge; interfaces pin; hosted-CI nav2 packages; a human-run Gazebo session, two-person safety review, and physical robot acceptance for the 16 Nov readiness ask |
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
