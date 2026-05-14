# docs/simulation

Top-level pointer for Gazebo Harmonic simulation documentation.

The reusable Gazebo bringup, ros↔gz bridge, and the generic walled world live in:

**`ros2/src/openamrobot_gazebo/`**

See its [README](../../ros2/src/openamrobot_gazebo/README.md) for the runnable usage.

Scenario-specific simulation assets are kept with the package that owns the scenario:

- Docking scenario (10×10 m room + AprilTag dock): `ros2/src/openamrobot_docking/worlds/docking_scenario.sdf`

For why we split simulation between a generic package (`openamrobot_gazebo`) and scenario-specific assets in consumer packages, see [architecture/01_repo_layout.md](../architecture/01_repo_layout.md).
