# Changelog

All notable changes to OpenAMR Platform Software will be documented in this file.

This project follows a practical changelog format inspired by:
- Keep a Changelog
- semantic versioning principles
- robotics software development workflows

---

# [Unreleased]

## Added

- Repository governance files
- Contribution guidelines
- Security policy
- Repository architecture documentation
- Planned package architecture placeholders
- GitHub Codespaces contribution workflow
- ROS 2 package ownership rules
- Pull request scope guidelines

## Changed

- Repository structure documentation improved
- Contribution onboarding clarified
- Repository boundaries clarified
- ROS 2 package responsibilities documented

## Fixed

- Added `.gitignore` rules for ROS 2 / colcon generated folders
- Prevented accidental tracking of:
  - `build/`
  - `install/`
  - `log/`

---

# Planned milestone roadmap

The following milestones represent the intended technical roadmap and are subject to change.

## v0.1.0-simulation

Planned focus:
- Gazebo / Gazebo Sim integration
- robot simulation environment
- robot description integration
- simulation launch workflows
- basic ROS 2 workspace structure

---

## v0.2.0-navigation

Planned focus:
- Nav2 integration
- localization
- planner configuration
- controller configuration
- navigation testing in simulation

---

## v0.3.0-docking

Planned focus:
- autodocking architecture
- AprilTag-based docking
- docking simulation
- docking workflows
- docking launch integration

---

## v0.4.0-firmware

Planned focus:
- STM32 integration
- firmware communication architecture
- ROS 2 hardware bridges
- hardware abstraction direction
- industrial-grade communication interfaces

---

## v0.5.0-real-robot-bringup

Planned focus:
- real robot integration
- hardware bringup
- controller integration
- driver integration
- simulation-to-real-world validation

---

# Versioning philosophy

Before the first stable release, versions may remain experimental.

Example version tags:

```text
v0.1.0-experimental
v0.2.0-simulation
v0.3.0-navigation
v0.4.0-docking
```

A stable release should only be created after:

- reproducible build workflow exists
- simulation workflow is documented
- navigation workflow is documented
- docking workflow is documented
- contributor onboarding is stable
- repository structure is mature
- safety expectations are documented

---

# Changelog contribution guidance

When contributing:

- document important architectural changes
- document breaking changes
- document repository structure changes
- document new ROS 2 packages
- document major simulation or navigation milestones
- document docking-related milestones
- document hardware integration milestones

Small internal refactors may not require changelog entries unless they affect:
- contributors
- architecture
- APIs
- launch workflows
- package structure