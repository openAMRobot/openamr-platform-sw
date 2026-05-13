# openamrobot_docking

ROS2 autodocking package for OpenAMRobot.

## Overview

This package contains the current AprilTag-based docking pipeline and its supporting files:

- launch files for the docking stack
- parameter files for the docking server, dock trigger, and detected dock pose publisher
- a C++ node that publishes the detected dock pose from TF
- a Python trigger node for dock and undock actions
- detailed documentation for setup, parameters, troubleshooting, and reproducibility

The focus of this repository is refinement, documentation, and release readiness rather than new feature development.

## Package structure

- `launch/` - launch files
- `config/` - parameter files
- `src/` - C++ sources
- `scripts/` - Python helper nodes
- `docs/` - detailed documentation

## Main entry point

Launch the docking stack with:

`ros2 launch openamrobot_docking openamrobot_docking.launch.py`

## Documentation

Start with:

- `docs/README.md`

