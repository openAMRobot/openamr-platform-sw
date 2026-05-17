# Contributing to OpenAMR Platform Software

Thank you for your interest in contributing to OpenAMR Platform Software.

This repository contains the ROS 2 software stack for the OpenAMRobot mobile robot platform. The project includes robot description, simulation, navigation, docking, bringup, control integration, drivers, perception modules, configuration, tools, and software documentation.

The project is experimental and under active development. Contributions are welcome, but they must follow the repository structure and package ownership rules described below.

---

# 1. Repository scope

This repository is for platform software only.

It may contain:

- ROS 2 packages
- robot description files
- Gazebo / Gazebo Sim simulation support
- Nav2 integration
- docking and autodocking software
- bringup launch files
- control integration
- hardware driver integration
- perception modules
- software configuration
- software tools and scripts
- software documentation

It should not contain:

- mechanical CAD files
- PCB design files
- firmware source code
- unrelated experiments
- generated build artifacts
- private credentials or secrets

Repository boundaries:

```text
openamr-platform-sw      -> ROS 2 platform software
openamr-platform-fw      -> embedded firmware
openamr-platform-hw      -> mechanical and electrical hardware

openamrobot-docs         -> organization-wide documentation
openamrobot-interfaces   -> shared ROS 2 messages, services, and actions
openamrobot-comm         -> shared communication contracts
openamrobot-ui           -> user/operator interface organization-wide
```

---
# 2. ROS 2 package ownership rules

Each ROS 2 package should have one primary responsibility.

The current implemented ROS 2 packages are:

```text
openamrobot_description  -> robot model, URDF/Xacro, meshes, frames, sensors
openamrobot_gazebo       -> Gazebo/Gazebo Sim simulation, worlds, models, simulation launch
openamrobot_nav2         -> Nav2 configuration, maps, planners, controllers, navigation launch
openamrobot_docking      -> autodocking logic, docking nodes, docking configuration, docking launch
```

The planned future ROS 2 packages are:

```text
openamrobot_bringup      -> top-level launch composition for robot or simulation bringup
openamrobot_control      -> control integration and controller configuration
openamrobot_drivers      -> hardware driver integration
openamrobot_perception   -> perception modules and perception pipelines
```

These planned packages are part of the intended architecture, but they should only become active ROS 2 packages when real functionality is added.

A package may reference another package, but it should not duplicate files from that package.

Correct example:

```text
openamrobot_docking references openamrobot_gazebo from a launch file.
```

Incorrect example:

```text
openamrobot_docking contains copied Gazebo worlds, copied Nav2 parameters, or copied URDF files.
```
---

# 3. Pull request principle

A pull request should be small, focused, and reviewable.

Good PR examples:

```text
feat(docking): add AprilTag docking pose publisher
fix(nav2): tune local planner parameters for simulation
docs(docking): add autodocking test instructions
chore(repo): update contribution guidelines
```

Bad PR examples:

```text
update everything
big changes
robot work
fix stuff
```

If you are working on autodocking, your PR should normally modify only:

```text
ros2/src/openamrobot_docking/
```

If you are working on navigation, your PR should normally modify only:

```text
ros2/src/openamrobot_nav2/
```

If your work requires changes in multiple packages, explain clearly why each package must be changed.

---

# 4. How to contribute using GitHub Codespaces

## Step 1: Fork the repository

Open:

```text
https://github.com/openAMRobot/openamr-platform-sw
```

Click:

```text
Fork
```

## Step 2: Open Codespaces

In your fork, click:

```text
Code -> Codespaces -> Create codespace on main
```

## Step 3: Create a feature branch

```bash
git checkout main
git pull origin main
git checkout -b feature/short-description
```

Examples:

```bash
git checkout -b feature/autodocking-apriltag
git checkout -b fix/nav2-simulation-params
git checkout -b docs/docking-readme
```

## Step 4: Make only relevant changes

Work only in the package or directory related to your task.

Before committing, always check:

```bash
git status
git diff --stat
```

The changed files should match the scope of your task.

## Step 5: Build the ROS 2 workspace

From the repository root:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If you changed only one package, you may build only that package:

```bash
colcon build --packages-select openamrobot_docking
```

## Step 6: Run tests if available

```bash
colcon test
colcon test-result --verbose
```

## Step 7: Commit your work

Use a clear commit message.

```bash
git add <changed-files>
git commit -s -m "feat(docking): add AprilTag docking pose publisher"
```

The `-s` flag adds a Signed-off-by line and confirms that you have the right to contribute the code.

## Step 8: Push your branch

```bash
git push origin feature/short-description
```

## Step 9: Open a Pull Request

Open a Pull Request from your branch into:

```text
openAMRobot/openamr-platform-sw:main
```

---

# 5. Pull request description template

Use this structure in your PR description:

```markdown
## Summary

Briefly explain what this PR does.

## Scope

This PR modifies:

- `path/to/package_or_file`

This PR does not modify:

- unrelated packages
- generated build files
- unrelated documentation

## Motivation

Explain why this change is needed.

## Changes

- Added ...
- Changed ...
- Fixed ...

## How to test

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select <package_name>
source install/setup.bash
```

Add launch or test commands if relevant.

## Screenshots, logs, or videos

Attach screenshots, terminal logs, or videos if the change affects simulation, navigation, docking, or robot behavior.

## Safety impact

Does this PR affect real robot movement?

- [ ] No, simulation or documentation only
- [ ] Yes, it may affect robot behavior

If yes, explain the safety considerations.

## Checklist

- [ ] I created a focused branch
- [ ] I changed only files related to this PR
- [ ] I removed generated files such as `build`, `install`, and `log`
- [ ] I tested the build
- [ ] I updated documentation if needed
- [ ] I explained how to test the change
```

---

# 6. Autodocking contribution rule

Autodocking contributions should be isolated to:

```text
ros2/src/openamrobot_docking/
```

The docking package may include:

```text
config/       -> docking parameters and AprilTag configuration
launch/       -> docking launch files
scripts/      -> docking helper scripts
src/          -> C++ docking nodes
docs/         -> docking documentation
models/       -> docking-specific models, if required
README.md     -> package documentation
package.xml   -> package dependencies
CMakeLists.txt
```

The docking package should not include copied files from:

```text
openamrobot_gazebo
openamrobot_nav2
openamrobot_description
openamrobot_bringup
```

If docking needs simulation, reference `openamrobot_gazebo`.

If docking needs the robot model, reference `openamrobot_description`.

If docking needs navigation behavior, reference `openamrobot_nav2`.

Do not duplicate those files inside `openamrobot_docking`.

---

# 7. Generated files are not allowed

Do not commit:

```text
build/
install/
log/
ros2/build/
ros2/install/
ros2/log/
__pycache__/
*.pyc
```

If they appear in your PR, remove them before requesting review.

---

# 8. Coding and documentation expectations

Contributions should be:

- understandable
- modular
- documented
- testable
- safe for simulation and real robot usage
- consistent with ROS 2 package structure

Documentation should explain:

- what the change does
- why it is needed
- how to build it
- how to run it
- how to test it
- whether it affects real robot behavior

---

# 9. Discussions and questions

For architecture questions, roadmap discussions, or contribution planning, use GitHub Discussions:

```text
https://github.com/orgs/openAMRobot/discussions
```

Use Issues for concrete bugs or tasks.

Use Pull Requests for proposed code or documentation changes.