# Pull Request

## Summary

Describe what this Pull Request changes.

## Motivation

Explain why this change is needed (the user-facing problem, the lesson learned, the regression being fixed, etc.).

## Type of Change

- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Simulation update
- [ ] Configuration update
- [ ] Refactoring
- [ ] Test update
- [ ] Other

## Affected Areas

- [ ] `openamrobot_description` (URDF, meshes, Gazebo plugin tags)
- [ ] `openamrobot_gazebo` (sim bringup, ros↔gz bridge, worlds)
- [ ] `openamrobot_nav2` (Nav2 params, SLAM, RViz layout)
- [ ] `openamrobot_docking` (AprilTag pipeline, dock_trigger, dock world)
- [ ] `openamrobot_bringup` (sim_only, sim_nav launch compositions)
- [ ] ROS 2 launch files
- [ ] Configuration files (YAML params)
- [ ] Parameters / topics / services / actions
- [ ] TF frames
- [ ] Documentation (READMEs, in-package `docs/`)
- [ ] Tests
- [ ] Repository metadata (`.github/`, `LICENSE`, `CONTRIBUTING.md`, etc.)

## Testing

Describe exactly how this was tested. Example command sequence:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to openamrobot_docking openamrobot_bringup
source install/setup.bash
ros2 launch openamrobot_docking docking_sim.launch.py
# In another terminal:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

## Expected Result

Describe what should happen after running the test (e.g. "the robot completes the 4-phase docking sequence and stops ~0.9 m in front of the AprilTag").

## Smoke tests run

Tick which simulation entry points you exercised before opening the PR:

- [ ] `ros2 launch openamrobot_bringup sim_only.launch.py` (teleop moves the robot)
- [ ] `ros2 launch openamrobot_bringup sim_nav.launch.py` (Nav2 active, `2D Goal Pose` works)
- [ ] `ros2 launch openamrobot_docking docking_sim.launch.py` + `dock_trigger` (4-phase sequence completes)

## Screenshots / Logs

Add screenshots, terminal logs, RViz screenshots, `rqt_graph` images, or simulation screenshots if useful.

## ROS 2 / Robotics Checklist

- [ ] I tested the changed launch files.
- [ ] I documented new or changed parameters.
- [ ] I documented changed topics, services, actions, or TF frames.
- [ ] I checked that generated folders are not committed (`build/`, `install/`, `log/`, `__pycache__/`).
- [ ] I updated the relevant READMEs if behaviour changed.
- [ ] I added or updated simulation instructions if simulation files changed.
- [ ] I checked that the workspace can still be built with `colcon build`.

## Simulation Checklist

Complete this section if the PR changes simulation files (worlds, URDF, gz plugins, bridge config).

- [ ] Simulation launch command is documented.
- [ ] Required ROS 2 distro is documented (Jazzy at time of writing).
- [ ] Required Gazebo / Ignition version is documented (Harmonic at time of writing).
- [ ] Expected simulation behaviour is documented.
- [ ] Known limitations are documented.
- [ ] Third-party simulation assets are properly licensed or original (and recorded in `NOTICE.md`).

## Legal / Contribution Checklist

- [ ] I have read [`CONTRIBUTING.md`](../CONTRIBUTING.md).
- [ ] My commits include DCO sign-off using `git commit -s`.
- [ ] I have the right to submit this work.
- [ ] This contribution is original or properly licensed.
- [ ] This contribution is submitted under the MIT License of this repository.
- [ ] I did not add third-party code, meshes, models, images, datasets, or assets without license information in `NOTICE.md`.

## Additional Notes

Add anything else reviewers should know (deferred follow-ups, related issues, design alternatives considered).
