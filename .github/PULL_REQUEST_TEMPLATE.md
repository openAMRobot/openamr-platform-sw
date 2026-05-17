# Pull Request Summary

Briefly explain what this Pull Request does.

Example:

- add autodocking simulation support
- improve Nav2 configuration
- add docking launch workflow
- fix Gazebo simulation issue
- improve documentation

---

# Scope of changes

This PR modifies:

- [ ] `openamrobot_description`
- [ ] `openamrobot_gazebo`
- [ ] `openamrobot_nav2`
- [ ] `openamrobot_docking`
- [ ] `openamrobot_bringup`
- [ ] `openamrobot_control`
- [ ] `openamrobot_drivers`
- [ ] `openamrobot_perception`
- [ ] documentation
- [ ] CI/CD or repository configuration
- [ ] other (describe below)

Describe the exact scope:

```text
Example:
- added AprilTag docking launch file
- updated docking parameters
- improved Nav2 simulation tuning
```

---

# Motivation

Why is this change needed?

Examples:

- fix simulation issue
- improve docking reliability
- improve ROS 2 package structure
- add missing documentation
- prepare future hardware integration

---

# Architecture and package responsibility

Confirm that your contribution follows package ownership rules.

Checklist:

- [ ] I modified only files related to the intended package scope
- [ ] I did not duplicate files from other ROS 2 packages
- [ ] I referenced existing packages instead of copying their contents
- [ ] I verified that unrelated packages were not modified accidentally

Examples:

Correct:

```text
openamrobot_docking references openamrobot_gazebo launch workflows
```

Incorrect:

```text
openamrobot_docking contains copied Gazebo worlds or copied Nav2 configuration
```

---

# Build and test instructions

Explain how to build and test this contribution.

Example:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash

colcon build --packages-select openamrobot_docking

source install/setup.bash
```

Add launch commands, simulation commands, or testing commands if relevant.

---

# Simulation and hardware impact

Does this PR affect:

| Area | Yes | No |
|------|-----|----|
| Gazebo/Gazebo Sim simulation | [ ] | [ ] |
| Navigation | [ ] | [ ] |
| Autodocking | [ ] | [ ] |
| Robot description | [ ] | [ ] |
| Real robot motion | [ ] | [ ] |
| Hardware communication | [ ] | [ ] |
| Embedded integration | [ ] | [ ] |

If yes, explain the impact:

```text
Describe:
- expected behavior changes
- simulation changes
- docking behavior changes
- navigation changes
- hardware interaction changes
```

---

# Safety considerations

If this PR may affect real robot behavior, explain:

- possible risks
- expected robot behavior
- testing limitations
- emergency stop assumptions
- simulation-only assumptions

Example:

```text
This PR changes docking behavior in simulation only.
Real robot validation has not yet been performed.
```

---

# Screenshots, logs, or videos

If applicable, attach:

- screenshots
- terminal logs
- Gazebo screenshots
- RViz screenshots
- videos
- architecture diagrams

Especially for:

- simulation
- navigation
- docking
- robot motion
- visualization changes

---

# Dependency changes

Did this PR add or modify dependencies?

- [ ] No
- [ ] Yes

If yes, explain:

```text
- new ROS 2 package dependencies
- Python dependencies
- C++ libraries
- simulation assets
- external repositories
```

---

# Documentation updates

Checklist:

- [ ] Documentation updated if necessary
- [ ] README updated if necessary
- [ ] Launch instructions updated if necessary
- [ ] Configuration changes documented
- [ ] New workflows documented

---

# Generated files check

Checklist:

- [ ] I did not commit generated ROS 2 / colcon files
- [ ] `build/` is not included
- [ ] `install/` is not included
- [ ] `log/` is not included
- [ ] temporary files are not included

---

# Final contributor checklist

Before requesting review:

- [ ] I reviewed my own changes
- [ ] I checked `git diff --stat`
- [ ] I tested the build
- [ ] I tested launch workflows if applicable
- [ ] I kept the PR scope focused
- [ ] I avoided unrelated changes
- [ ] I followed repository structure rules
- [ ] I updated documentation if needed

---

# Additional notes

Add any additional information for reviewers here.