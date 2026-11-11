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
| twist_mux command arbitration + stop gate (#35/#119) | #90 | changes requested (panthera-momagdii/Mohamed Sayed/BotshareAI). Base `713574f`, prior head `f482cfe` (reviewed by panthera-momagdii's third round; this round's fixes below are on top of it, not yet pushed, 11 Nov 2026 - exact head SHA to follow once pushed). **Hosted CI, exact head `f482cfe`** (panthera-momagdii's own independent run, not author-reported): `nav-status-tests` run 43026928569 passed 260 tests + the live stop-lock invariant; `drivers-bringup-tests` run 43026928657 passed its 18 launch tests + both lint jobs; `repository-quality` run 43026928960 passed (`quality/test` skipped, expected). The live bt_navigator-activation and dock-trigger clean-boot tests are NOT in hosted CI (local-only, see "Not verified" in the PR body for why). **Author-reported local evidence, this round, not yet independently re-run by a reviewer:** fixes panthera-momagdii's P1 finding on `f482cfe` - the discovery-grace fix in that head only re-anchored the WINDOW to publisher-match time, but once that window elapsed with `statuses` still `None` it appended no blocker and fell through, so a publisher known to exist that simply had not yet delivered its retained sample let Resume through with no actual status evidence. Now fails closed indefinitely once a publisher is matched and no sample has arrived - the window only ever delays the block, it never expires it once a publisher is confirmed; construction-anchored trust-the-silence stays only as the fallback for a producer whose publisher never appears at all. New regression `test_release_stays_refused_past_the_grace_window_if_no_sample_ever_arrives` reproduces panthera's exact repro (publisher matched, sample never sent, release attempted after the window) - RED-confirmed against `f482cfe`, GREEN after. Also fixed: two of this test file's OWN pre-existing grace-period tests were accidentally vacuous - they created an `orphaned` ledger entry, which makes `GoalAuthority.state()` return UNKNOWN unconditionally and blocks via the ledger branch before ever reaching the discovery-grace code they meant to exercise; rewritten to establish a clean (non-orphaned) IDLE ledger state instead. Stale comments panthera-momagdii (via ChatGPT Codex Connector) flagged also fixed: `goal_authority.py`'s module docstring and `safety_stop_gate_node.py`'s legacy-path comment both still described `navigate_to_pose` as not ledger-backed/legacy-only; both now correctly distinguish the proxy-backed public path, the internal `goal_pose_nav`/`self_client_` bypass (covered by status reconciliation on `navigate_to_pose_real`, not the ledger), and the still-legacy `navigate_through_poses`/`dock_robot` paths. Full local `openamrobot_nav2` regression on this round's working tree: 277 passed, 1 skipped (both pre-existing baselines, unchanged in kind from `f482cfe`), 531 pre-existing flake8/pep257 findings (confirmed unchanged by this round via stash-and-diff), zero leaked processes. Still open, unchanged from `f482cfe`: `goal_pose_nav`-sourced goals remain invisible to the goal-authority ledger itself (covered for safety by status reconciliation, not by the ledger's low-latency path) - needs an owner decision on routing `goal_pose_nav` through the proxy vs. a forwarder change; D-13 coverage for `navigate_through_poses`/`dock_robot`; `#125`'s `/undock_robot` bypass; interfaces#35 fork pin; nav2-* packages missing from hosted CI; two-human safety review and the Gazebo/physical acceptance sequence | owner/Alex decision on `goal_pose_nav` routing through the proxy vs. forwarder change, the goal-owning-proxy pattern for the 2 remaining producers, `#125` bypass, interfaces pin, hosted-CI nav2 packages; a human-run Gazebo session for the 16 Nov readiness ask; a reviewer re-run of this round's local-only evidence once pushed |
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
