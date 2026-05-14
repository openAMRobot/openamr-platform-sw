# docs/docking

Top-level pointer for autodocking documentation.

The autodocking pipeline source, configuration, world, and engineering documentation live in:

**[`ros2/src/openamrobot_docking/`](../../ros2/src/openamrobot_docking/)**

Two layers of documentation there:

- **Package README**: [`ros2/src/openamrobot_docking/README.md`](../../ros2/src/openamrobot_docking/README.md) — the runnable usage, the 4-phase pipeline overview, AprilTag setup, TF chain, coordinate conventions, troubleshooting matrix.
- **Engineering docs**: [`ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/) — the in-depth content (parameters, architecture, lessons learned, design-decision diary).

## Highlights

| Doc | What's in it |
|---|---|
| [`00_overview.md`](../../ros2/src/openamrobot_docking/docs/00_overview.md) | What the package is and why this design |
| [`01_quickstart.md`](../../ros2/src/openamrobot_docking/docs/01_quickstart.md) | Run the docking sim end-to-end |
| [`02_architecture.md`](../../ros2/src/openamrobot_docking/docs/02_architecture.md) | Topology, node graph, lifecycle |
| [`03_tf_frames.md`](../../ros2/src/openamrobot_docking/docs/03_tf_frames.md) | TF chain (robot, dock, optical frames) |
| [`04_apriltag.md`](../../ros2/src/openamrobot_docking/docs/04_apriltag.md) | AprilTag sizing/family/detector params |
| [`05_parameters.md`](../../ros2/src/openamrobot_docking/docs/05_parameters.md) | Every `dock_trigger.yaml` knob explained |
| [`07_reproduce_results.md`](../../ros2/src/openamrobot_docking/docs/07_reproduce_results.md) | Reproducing the simulated end state |
| [`08_sequencer_4phase.md`](../../ros2/src/openamrobot_docking/docs/08_sequencer_4phase.md) | The 4-phase docking flow, phase by phase |
| [`09_troubleshooting.md`](../../ros2/src/openamrobot_docking/docs/09_troubleshooting.md) | Symptom → cause → fix matrix |
| [`12_lessons_learned.md`](../../ros2/src/openamrobot_docking/docs/12_lessons_learned.md) | Design-decision diary |
| [`legacy/`](../../ros2/src/openamrobot_docking/docs/legacy/) | The original `controlled_approach` docs, kept for traceability |

## Running

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
# Then in a second terminal (with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp exported):
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```
