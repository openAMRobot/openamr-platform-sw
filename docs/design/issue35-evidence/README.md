# Issue #35 simulation evidence (18 October 2026)

Development evidence only: Gazebo Harmonic, headless, real-time factor about 0.3, sim-only
fixtures F1 to F3 (`sim_fixture_overrides.diff`, `tools/issue35_arbitration/apply_sim_fixtures.sh`).
Not real-robot, bench or safety evidence. See `../issue35-command-arbitration.md`.

| File | Run | Shows |
|---|---|---|
| `main_sim_graph.txt` | main, sim, plain-node Nav2 | `/cmd_vel` publishers: dock_trigger, behavior_server (5), collision_monitor |
| `main_composable_graph.txt`, `prototype_composable_graph.txt` | composable Nav2, by hand | main: behavior_server (5) + collision_monitor on `/cmd_vel`; prototype: collision_monitor only |
| `prototype_sim_graph.txt` | prototype, sim | single final publisher, Twist everywhere, `use_stamped` false |
| `main_scan_selfview.txt`, `main_phase1_blocked.log` | main, no fixtures | body seen at scan 0 ± 46 deg; Phase 1 NavigateToPose timeout |
| `lifecycle_manager_reset_on_pause.log` | B0 (main + F1 to F3) | pausing collision_monitor resets the Nav2 stack |
| `b0_main_dock_run.csv.gz` | B0 | full docking run trace (cmdmon.py format) |
| `p1_prototype_dock_run.csv.gz`, `p5_prototype_dock_run.csv.gz` | prototype | docking runs that completed |
| `p4_phase1_bt_ack_timeout.log` | prototype | the run that aborted in Phase 1 (BT follow_path acknowledgement timeout) |
| `check_run_results.txt` | B0, P1, P5, P4 | C1 to C4 per run |
| `option_a_checks.txt` | standalone twist_mux | default TwistStamped output, silence on timeout, lock 255 vs teleop 255 and 100 |
| `scenarios_p3_committed_code.txt`, `scenarios_p2_velocity_timeout_0.3.txt` | prototype | handover, stop lock, publisher loss (1.0 s and 0.3 s velocity_timeout) |
