# STATE

Read this file before any work in this repository and update it in the same PR as the work.
The PR evidence check fails when STATE.md exists and a PR neither updates it nor says
"no change" with a reason. Keep it under 60 lines; facts only, each with a date or SHA.

## Repository

- Type and lifecycle: software, active
- Readiness: not yet assessed (17 Oct 2026, rollout step a)
- Owner role: software-lead (maintainers.yaml)
- Verification (29 Oct 2026, main 131e500): CI workflows `nav-status-tests`,
  `drivers-bringup-tests`, `description-tests`, `repository-quality`; this PR adds
  `ros-workspace-tests` (colcon test, all seven packages, every PR) and `sim-launch-tests`
  (headless simulation launch tests, nightly). No `tools/verify.sh` yet (#101 proposes it).
- BOM in force (register BOM-ISSUE-IN-FORCE, .github 71c439e): Issue 7.10; 7.11 is a source only.
- Dates: Gate A (simulation) 6 Nov 2026; v2.0.0-rc.1 20 Nov 2026.

## Current work

| Work package | Branch or PR | Status | Next step |
|---|---|---|---|
| Tests and CI: PSW-01, PSW-03, PSW-08, PSW-15, PSW-18 (29 Oct 2026) | this PR | draft | work-package owner: adopt, adapt or reject |
| ROS 2 verify gate (#98) | #101 | in review | owner review |
| Command arbitration, twist_mux + stop gate (#35) | #90, review/issue35-arbitration | in review | owner review; then drop the PSW-03 xfail |
| Recovery BT (#35/N4) | #91, #92 | in review | owner review |
| SiteConfigStatus producer (#36) | #73 | in review | owner review |
| E-01 safety docs | #66 | draft | owner review |
| Devcontainer | #57 | in review | owner review |
| Gemini 336L base camera simulation | #53 | draft | owner review |
| Sensor frames to P-03 rev19.7 (ToF pitch) | #93 | draft | superseded by this PR if adopted |
| Shared agent rules v2 (rollout step a) | #72 | merged 16 Oct 2026 | step (b) pin the harness (`@main` today) |
| behavior_server through the reactive chain (#35) | #54 | merged 18 Oct 2026 | none |
| Base meshes and GLB (#94 to #96), sim scan filter (#97, #99) | merged | 25 to 27 Oct 2026 | none |
| Base E-stop and RESET in the release 001 button holes | #100 | merged 28 Oct 2026 | mechanical owner: confirm the 22.4 mm hole |

## Decisions this repository implements

FRAMES-REP105, NEAR-FIELD-FRAMES, NAV-LIDAR, CAMERAS, IMU-TOPIC-OWNERSHIP, DRIVE-TRACK,
DOCK-NO-CONTACTS, DOCKING-NOT-CHARGING.

## Known gaps

- Main 131e500 (29 Oct 2026): no twist_mux, dock_trigger and collision_monitor both publish
  /cmd_vel (PSW-03 xfail, #35, #88); apriltag_node exits non-zero on stop (PSW-15 xfail, #84).
- PSW-15 (29 Oct 2026): each launch_testing stop leaves one `gz sim` server running (the gz
  wrapper ends by SIGTERM); no tracking issue yet, so the nightly check fails until fixed.
- Near-field frames follow the register (us_* x +/-0.390, tof_* 10 deg down, pending the
  bench test); the CAD sensor-face offsets differ by +0.6 / -1.4 mm (this PR).
- Visual meshes come from `tools/cad/step_to_description_meshes.py`; the B2 hub motor STEP
  is only on platform-hw PR #28. Open CAD drift: TF-Luna height, IMU height, ToF openings.
- Provisional or placeholder geometry: battery cell-group envelope (25 Oct 2026; BMS and
  wiring not modelled), base E-stop and RESET (XB4 hole tolerance unconfirmed, #100).
- Copyright tests skipped in six packages without a tracking issue on main (#101 adds #102).

## Not verified

- Real-robot motion, docking and sensor bring-up; simulation and fake hardware only, since
  dfd44a5. Simulation launch tests are not physical acceptance and not safety evidence.
