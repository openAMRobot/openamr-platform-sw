# Security Policy

## Reporting Security Issues

Please do not report security issues publicly in GitHub Issues.

If you discover a security vulnerability, unsafe behavior, exposed credential, or sensitive technical issue, please contact the maintainers privately:

    botshare.ai@gmail.com

Include:

- repository name
- affected files or components
- description of the issue
- steps to reproduce if applicable
- possible impact
- suggested fix if known

## Scope

This policy applies to:

- source code (all packages under `ros2/src/`)
- ROS 2 launch files
- configuration files (Nav2 params, SLAM params, AprilTag params, dock_trigger params, gz bridge config)
- URDF / xacro robot description
- Gazebo worlds and models shipped in the repository
- documentation that could expose unsafe deployment practices
- credentials, tokens, or private configuration accidentally committed to the repository

## Safety-Critical Robotics Notice

This repository contains software intended to run on **real autonomous mobile robots**. A bug in this codebase can cause physical harm.

Before deploying changes on physical hardware:

- test in simulation first (`ros2 launch openamrobot_bringup sim_nav.launch.py` and `ros2 launch openamrobot_docking docking_sim.launch.py`);
- verify emergency stop behavior;
- verify speed limits (`velocity_smoother.max_velocity` in `openamrobot_nav2/config/nav2_params.yaml`, `drive_speed` in `openamrobot_docking/config/dock_trigger.yaml`);
- verify the `collision_monitor` polygons match the **actual** robot footprint (the simulated config has `FootprintApproach.enabled: false` — re-enable on the real robot);
- verify sensor topics and TF frames are correctly remapped;
- test in a controlled environment with a person near an E-stop;
- do not operate near people without appropriate safety validation.

OpenAMRobot maintainers may remove or reject contributions that introduce unsafe behavior or unclear deployment risks.

## Disclosure

Once a security issue is confirmed and addressed:

- the fix is released through a standard Pull Request;
- the reporter is credited in the commit message and (with consent) in `AUTHORS.md`;
- the disclosure is described in `CHANGELOG.md` once a fix is publicly available.
