# openamrobot_docking — package documentation

This folder contains the engineering documentation for the autodocking pipeline shipped by `openamrobot_docking`.

## Layout

```
docs/
├── 00_overview.md             ← what the package is, design summary
├── 01_quickstart.md           ← run the docking sim end-to-end
├── 02_architecture.md         ← topology, node graph, lifecycle
├── 03_tf_frames.md            ← the TF chain (robot, dock, optical frames)
├── 04_apriltag.md             ← AprilTag sizing/family/detector params
├── 05_parameters.md           ← every dock_trigger.yaml parameter explained
├── 06_camera_calibration.md   ← intrinsics flow (real-robot only)
├── 07_reproduce_results.md    ← reproducing the simulated end state
├── 08_sequencer_4phase.md     ← the 4-phase docking flow, phase by phase
├── 09_troubleshooting.md      ← symptom → cause → fix matrix
├── 10_diagrams.md             ← sequence/state diagrams (text)
├── 11_changes_from_upstream.md ← what this revision changes vs prior pipelines
├── 12_lessons_learned.md      ← decisions diary, with rationale
└── legacy/                    ← original 9 docs from the controlled_approach era
```

## How to read

1. New here? Start at `00_overview.md` then `01_quickstart.md`.
2. Tuning the controller? `05_parameters.md` and `08_sequencer_4phase.md`.
3. Something went wrong? `09_troubleshooting.md` first, then `12_lessons_learned.md` for the deeper why.
4. Onboarding teammates onto the 4-phase pipeline? `08_sequencer_4phase.md` plus `10_diagrams.md`.

## Legacy docs

The `legacy/` subfolder contains the documentation written when the pipeline used `opennav_docking::SimpleChargingDock::controlled_approach`. The 4-phase sequencer replaced that flow because controlled_approach produced curved trajectories that did not arrive head-on at the dock. The legacy docs are preserved verbatim so the rationale remains traceable.

## Path / naming caveat

A few of the 4-phase docs in this folder were written before the migration to `openamr-platform-sw`. They may still reference:

- `omr_description` (now `openamrobot_description`)
- `openamrobot-docking-main` workspace path (now `openamr-platform-sw`)
- `simulation/config/...` (now split across `openamrobot_nav2/config/` and this package's `config/`)
- the `simulation.launch.py` entry point (now `docking_sim.launch.py`)

When in doubt, the canonical bringup is the platform-level `README.md` at the root of `openamr-platform-sw`, and the package-level `README.md` next to this folder. The docs here are the engineering record; treat them as the source for *why*, and the README as the source for *how to run today*.
