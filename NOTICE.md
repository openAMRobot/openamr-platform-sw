# Notice

OpenAMR Platform Software is part of the OpenAMRobot project.

This repository contains ROS 2 software, simulation, navigation, docking, control integration, drivers, perception modules, configuration, tools, and software documentation for the OpenAMR mobile robot platform.

---

# Project ownership

OpenAMRobot is an open robotics project focused on affordable, modular, and educational autonomous mobile robot technologies.

GitHub organization:

```text
https://github.com/openAMRobot
```

---

# Repository scope

This repository is for platform software only.

It may include:

- ROS 2 packages
- simulation support
- navigation configuration
- docking and autodocking logic
- robot description files
- software tools
- software documentation

It should not include:

- mechanical CAD files
- PCB design files
- embedded firmware source code
- private credentials
- proprietary third-party assets without permission

---

# Third-party software

This project may depend on or integrate with third-party open-source software, including but not limited to:

- ROS 2
- Nav2
- Gazebo / Gazebo Sim
- colcon
- CMake
- Python packages
- C++ libraries
- robotics simulation tools

Each third-party dependency is governed by its own license.

---

# Third-party assets

Simulation models, meshes, textures, maps, icons, and other assets must only be added if their source and license are known.

When adding third-party assets, contributors must document:

- asset name
- source URL
- author or organization
- license
- modification status
- file location in this repository

---

# Contributor responsibility

Contributors must ensure that they have the legal right to contribute any code, configuration, documentation, model, mesh, texture, map, or other asset submitted to this repository.

Do not submit:

- proprietary material without permission
- confidential material
- restricted files
- private credentials
- copied assets without known license
- third-party files without attribution

---

# Robotics safety notice

This repository may eventually affect real robot behavior.

Users and contributors are responsible for validating:

- robot motion safety
- navigation behavior
- docking behavior
- simulation-to-real-world assumptions
- hardware compatibility
- deployment environment safety

Software that works in simulation must not be assumed safe for real robot operation without additional testing.