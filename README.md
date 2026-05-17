# OpenAMR Platform Software

ROS 2 software, simulation, navigation, docking, control, drivers, and bringup for the OpenAMRobot mobile robot platform.

## Repository status

Current maturity level: Experimental

This repository is under active development and serves as the main software repository for the OpenAMR mobile robot platform.

The project is intended for research, education, prototyping, and gradual development toward industrial-grade autonomous mobile robot capabilities.

## Current ROS 2 packages

```text
ros2/src/
├── openamrobot_description/
├── openamrobot_gazebo/
├── openamrobot_nav2/
└── openamrobot_docking/
```

## Planned ROS 2 packages

These directories are architecture placeholders and will become active ROS 2 packages when real functionality is added.

```text
ros2/src/
├── openamrobot_bringup/
├── openamrobot_control/
├── openamrobot_drivers/
└── openamrobot_perception/
```

## Package responsibilities

```text
openamrobot_description  -> robot model, URDF/Xacro, meshes, frames, sensors
openamrobot_gazebo       -> Gazebo/Gazebo Sim simulation, worlds, models, simulation launch
openamrobot_nav2         -> Nav2 configuration, maps, planners, controllers, navigation launch
openamrobot_docking      -> autodocking logic, docking nodes, docking configuration, docking launch

openamrobot_bringup      -> planned top-level launch composition
openamrobot_control      -> planned control integration and controller configuration
openamrobot_drivers      -> planned hardware driver integration
openamrobot_perception   -> planned perception modules and perception pipelines
```

Each package should own only its own responsibility. Packages may reference each other, but they should not duplicate each other’s files.

## Repository boundaries

This repository contains platform software only.

Hardware design files belong in:

```text
openamr-platform-hw
```

Firmware belongs in:

```text
openamr-platform-fw
```

Organization-wide documentation belongs in:

```text
openamrobot-docs
```

Shared ROS 2 messages, services, and actions belong in:

```text
openamrobot-interfaces
```

Shared communication contracts belong in:

```text
openamrobot-comm
```

User/operator interface belongs in:

```text
openamrobot-ui
```

## Getting started

Clone the repository:

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
```

Build the ROS 2 workspace:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Build a selected package:

```bash
colcon build --packages-select openamrobot_docking
```

## Development in GitHub Codespaces

1. Open the repository on GitHub.
2. Click `Code`.
3. Open the `Codespaces` tab.
4. Create a codespace from `main`.
5. Create a feature branch before making changes.

```bash
git checkout -b feature/your-feature-name
```

Before committing:

```bash
git status
git diff --stat
```

Do not commit generated folders such as:

```text
build/
install/
log/
ros2/build/
ros2/install/
ros2/log/
```

## Contributing

See `CONTRIBUTING.md`.

Key rule:

A contribution should modify only the package or directory related to the task.

For example, an autodocking contribution should normally modify only:

```text
ros2/src/openamrobot_docking/
```

## Community and discussions

Architecture questions, roadmap ideas, and collaboration topics should be discussed here:

```text
https://github.com/orgs/openAMRobot/discussions
```

## Safety notice

This repository may affect real robot behavior.

Users are responsible for validating:

- robot safety
- navigation behavior
- docking behavior
- motor control behavior
- sensor integration
- deployment suitability
- regulatory compliance

This software is provided for research, education, and development purposes.

## License

See `LICENSE`.