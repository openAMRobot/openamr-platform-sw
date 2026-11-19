# STATE

Read this file before any work in this repository and update it in the same PR as the work.
The PR evidence check fails when STATE.md exists and a PR neither updates it nor says
"no change" with a reason. Keep it under 60 lines; facts only, each with a date or SHA.

## Repository

- Type and lifecycle: software, active
- Readiness: not yet assessed (17 Oct 2026, rollout step a)
- Owner role: software-lead (maintainers.yaml)
- Verification: CI workflows `nav-status-tests`, `drivers-bringup-tests` and
  `repository-quality`; no `tools/verify.sh` yet. `repository-quality` PASS on main at
  dfd44a5 (16 Oct 2026).

## Current work

| Work package | Branch or PR | Status | Next step |
|---|---|---|---|
| Shared agent rules v2 (rollout step a) | #72 | in review | merge, then step (b) pin the harness |
| E-01 safety docs | #66 | draft | owner review |
| Devcontainer | #57 | in review | owner review |
| Nav2 behavior_server remap (#35) | #54 | in review | owner review |
| Gemini 336L base camera simulation | #53 | draft | owner review |
| SiteConfigStatus producer (platform-sw#36) | #73 | changes requested (panthera-momagdii) - this push answers both open findings. P1 (7 Nov): MapLoaderNode's boot timer cancelled itself unconditionally on its first tick even if map_server was not yet discoverable, permanently losing the boot load with no retry - fixed, timer now stays periodic until the service is ready; new test_a_late_starting_map_server_is_still_loaded_at_boot, RED-verified against the pre-fix node. First-live-map identity (final pass, previously xfail, strict=True): navigation_status_node.py trusted a live /map message's content on first observation alone, with nothing to compare it against - an unverified map B, first-observed before map_yaml_path was ever updated, or surviving this node's own restart, both read as MATCHED. Fixed by wiring navigation_status_node.py to map_loader_node.py's own /map_identity (transient-local) as the only trust anchor - content is matched against the loader's own genuinely-recorded grid_fingerprint from a real ~/load_map response, never a self-referential "last trusted" guess; a restart still confirms immediately once a real identity is retained, since that persistence is of a verified fact, not an unverified one. The xfail test is now a real passing regression (renamed, strict mode removed), plus a new restart-with-a-genuine-identity test for the other half of the fix. 211 passed across the touched test files; zero new flake8/pep257 findings (confirmed by diffing against the pre-fix lint output line for line). | owner re-review; interfaces#28 remains a required, not-yet-merged dependency (DCO/merge-conflict/compatibility-snapshot fixes prepared separately) |

## Decisions this repository implements

FRAMES-REP105, NEAR-FIELD-FRAMES, NAV-LIDAR, CAMERAS, IMU-TOPIC-OWNERSHIP, DRIVE-TRACK,
DOCK-NO-CONTACTS, DOCKING-NOT-CHARGING.

## Known gaps

- Harness not pinned; `repository-quality` calls the reusable workflow at `@main` (step b).
- No `tools/verify.sh`; `quality/test` is skipped on main.
- Watchdog report-only on dfd44a5 plus this PR (17 Oct 2026): 46 decisions findings, 15
  public-extract price findings, 3 unpinned actions; fixed or tracked in step (c).

## Not verified

- Real-robot motion, docking and sensor bring-up; simulation and fake hardware only, since dfd44a5.
