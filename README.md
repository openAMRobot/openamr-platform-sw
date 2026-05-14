# OpenAMR Platform Software

ROS 2 software, simulation, navigation, docking, control, drivers, and bringup for the OpenAMRobot mobile robot platform.

## Repository Status

Current maturity level: Experimental

This repository is under active development and will serve as the main software repository for the OpenAMR mobile robot platform.

## Quickstart — docking simulation

```bash
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to openamrobot_docking
source install/setup.bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Then in another terminal: `ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once`.

Full walkthrough in [`docs/getting_started/01_quickstart_docking_sim.md`](docs/getting_started/01_quickstart_docking_sim.md).

## Purpose

This repository contains the product-level software stack for the OpenAMR platform, including:

- robot description
- Gazebo simulation
- Nav2 navigation
- docking and autodocking
- robot bringup
- control integration
- hardware drivers
- perception modules
- configuration
- developer tools

## Repository Structure

```text
openamr-platform-sw/
├── ros2/
│   └── src/
│       ├── openamrobot_description/
│       ├── openamrobot_gazebo/
│       ├── openamrobot_nav2/
│       ├── openamrobot_docking/
│       ├── openamrobot_bringup/
│       ├── openamrobot_control/
│       ├── openamrobot_drivers/
│       └── openamrobot_perception/
│
├── simulation/
├── config/
├── scripts/
├── tools/
└── docs/
```

## Repository Boundaries

This repository contains software for the OpenAMR mobile robot platform.

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

Shared interfaces belong in:

```text
openamrobot-interfaces
```

Shared communication contracts belong in:

```text
openamrobot-comm
```

## Related Repositories

- `openamr-platform-hw` — mechanical, electrical, CAD, BOM, wiring
- `openamr-platform-fw` — embedded firmware
- `openamrobot-docs` — user-facing and architecture documentation
- `openamrobot-interfaces` — shared ROS 2 messages, services, actions
- `openamrobot-ui` — user interface and operator interface

## Safety Notice

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

## Contributing

See `CONTRIBUTING.md`.

## Security

See `SECURITY.md`.

## License

MIT License.