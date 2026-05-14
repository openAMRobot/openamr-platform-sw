# `tools/` — developer tools

**Currently empty.** Reserved for developer-facing utilities that are not part of the runtime stack.

## What belongs here

Scripts and small programs that help **developing on** the repo but don't ship with the robot, e.g.:

- workspace setup helpers (one-shot Ubuntu/Jazzy install, vcs-import, rosdep)
- TF / log inspection scripts (parse a `ros2 bag` and plot the docking trajectory)
- AprilTag printable generators / calibration helpers
- Bench/benchmark drivers for the Nav2 controller
- Lint runners (`ruff`, `clang-format`, `xmllint` on the URDF)

## What does NOT belong here

| Concern | Lives in |
|---|---|
| Runtime ROS 2 packages | `ros2/src/<package>/` |
| Sim-cleanup script (`kill_sim.sh`) | [`scripts/`](../scripts/) |
| CI workflows | [`.github/workflows/`](../.github/workflows/) |

The line between `scripts/` (operator-side, "run the robot" helpers) and `tools/` (developer-side, "work on the repo" helpers) is fuzzy — when in doubt, put it in `scripts/` if a non-developer might need it, otherwise here.
