# Contributing to OpenAMR Platform Software

Thank you for your interest in contributing to `openamr-platform-sw`.

This repository contains the ROS 2 Jazzy software stack for the OpenAMRobot mobile robot platform — robot description, Gazebo Harmonic simulation, Nav2 navigation, AprilTag-based autodocking, and the launch compositions that wire it all together. The project is open-source, educational, and engineering-oriented. Contributions are welcome if they improve reliability, clarity, reproducibility, documentation, simulation, or hardware integration.

## Project Scope

This repository focuses on:

- robot description (URDF, meshes, Gazebo plugin tags) — `openamrobot_description`
- Gazebo Harmonic simulation bringup and the ROS↔gz bridge — `openamrobot_gazebo`
- Nav2 navigation stack, SLAM Toolbox configuration, RViz nav layout — `openamrobot_nav2`
- AprilTag-based autodocking pipeline and docking simulation — `openamrobot_docking`
- top-level launch compositions (sim_only, sim_nav, real-robot bringup) — `openamrobot_bringup`
- low-level control integration, hardware drivers, perception modules — `openamrobot_control`, `openamrobot_drivers`, `openamrobot_perception` (placeholders, work in progress)
- product-level configuration, simulation assets, scripts, tools — top-level `config/`, `simulation/`, `scripts/`, `tools/`, `docs/`

Hardware design, firmware, shared interfaces, and UI live in separate repositories (see the [related repositories table in the README](README.md#related-repositories)).

## Ways to Contribute

You can contribute by:

- reporting bugs (use a clear minimal reproduction)
- improving documentation (READMEs, in-package docs under `docs/`)
- improving launch files and parameter defaults
- adding or improving simulation assets (worlds, models)
- adding tests
- improving diagrams and educational material
- adding new packages within the project scope above (e.g. populating `openamrobot_drivers`)

## Contribution Workflow

1. Fork the repository and create a feature branch in your fork.
2. Use a clear branch name, for example:
   - `feature/add-amcl-localization`
   - `fix/cmd-vel-chain-collision-monitor`
   - `docs/improve-tf-frames-doc`
   - `chore/clean-build-artefacts`
3. Make focused changes. Prefer multiple small Pull Requests over one large one.
4. Commit with DCO sign-off (see below).
5. Push your branch to your fork.
6. Open a Pull Request against `openAMRobot/openamr-platform-sw:main`.
7. Complete the Pull Request description (summary + test plan).
8. Wait for maintainer review.
9. Apply requested changes if needed.

Do not push directly to `main` of the org repository. All changes go through Pull Requests.

## Building and testing locally

Before opening a PR, verify your changes build and run end-to-end:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to openamrobot_docking openamrobot_bringup
source install/setup.bash

# Smoke tests:
ros2 launch openamrobot_bringup sim_only.launch.py     # robot exists, teleop moves it
ros2 launch openamrobot_bringup sim_nav.launch.py      # Nav2 active, 2D Goal Pose works
ros2 launch openamrobot_docking docking_sim.launch.py  # AprilTag detected, dock_trigger completes
```

For docking-specific changes, exercise the trigger:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The PR description must include a **Test plan** section recording which of these smoke tests you ran and the outcomes.

## Developer Certificate of Origin

This repository uses the Developer Certificate of Origin (DCO).

By signing off your commits, you certify that you have the right to submit the contribution and that it may be included in this project under the repository license.

Each commit must include a sign-off line:

    Signed-off-by: Your Name <your.email@example.com>

The easiest way is to commit with `-s`:

    git commit -s -m "your commit message"

## Legal Notice for Contributions

By submitting a Pull Request to this repository, you confirm that:

- you have the right to submit the contributed work;
- your contribution is original or properly licensed;
- your contribution does not knowingly violate third-party rights;
- your contribution is provided under the MIT License used by this repository;
- OpenAMRobot may use, modify, publish, distribute, sublicense, and commercialize the contribution under the repository license;
- third-party code, assets, meshes, models, datasets, images, or configuration files are not added unless their license is clearly documented.

If you do not agree with these terms, do not submit a contribution.

## Third-Party Assets and Dependencies

Do not add third-party files unless their license is clear.

This applies to:

- meshes (STL, COLLADA, OBJ)
- textures, AprilTag images, calibration patterns
- Gazebo / Ignition models and worlds
- URDF / Xacro files copied from another project
- launch files copied from another project
- datasets
- configuration files
- external scripts and vendor libraries

If a third-party asset is required, document in [`NOTICE.md`](NOTICE.md):

- source (repository URL)
- author or project
- license
- modification status (verbatim / modified — describe what was changed)
- reason for inclusion

## ROS 2 and Robotics Contribution Standards

When changing ROS 2 code or configuration, document the effect on:

- nodes, topics, services, actions, parameters
- TF frames (`base_footprint`, `base_link`, `odom`, `map`, sensor frames, dock frames)
- launch files and their composition relationships
- simulation behaviour (`ros2 launch ... docking_sim`)
- hardware behaviour if applicable

If a parameter is added or changed, document in the relevant package README:

- parameter name and YAML location
- default value and units
- valid or recommended range
- practical impact (what does increasing/decreasing it do)
- dependencies (what other params it interacts with)
- possible failure modes

## Simulation Contributions

Simulation contributions must include:

- launch instructions (which `ros2 launch ...` command exercises the change)
- required ROS 2 distribution (Jazzy at time of writing)
- required Gazebo / Ignition version (Harmonic at time of writing)
- expected behaviour
- tested scenario (`walled_world.sdf` / `docking_scenario.sdf` / new world)
- known limitations
- required models, worlds, and configuration files
- screenshots or logs if useful

**Do not commit generated folders.** The repository `.gitignore` already excludes:

- `build/`, `install/`, `log/` (colcon outputs)
- `__pycache__/`, `*.pyc` (Python caches)
- Gazebo runtime caches

If you find any of these tracked in git, remove them with `git rm --cached -r <path>`.

## Documentation Standards

Documentation should be:

- concise
- technically accurate
- beginner-friendly where appropriate
- useful for real deployment, not only for simulation
- easy to maintain
- connected to the actual package structure (link to file paths, line numbers when relevant)

Prefer practical explanations over generic theory. When documenting a non-obvious design choice (e.g. why wheel collision radius ≠ kinematic radius, why we do not run a standalone `joint_state_publisher`), explain the **why** alongside the **what** so a future maintainer doesn't undo the choice.

## Pull Request Review

A Pull Request may be rejected or delayed if:

- it is outside the scope of this repository;
- it lacks documentation;
- it breaks existing launch or configuration behaviour;
- it introduces unclear third-party licensing;
- it does not include DCO sign-off;
- it adds generated or temporary files;
- it is too large and should be split into smaller Pull Requests.

## Recognition

Substantive contributions are recognized in three places:

1. **GitHub commit history** — every signed-off commit is preserved with author name and email. This is the authoritative record.
2. [`AUTHORS.md`](AUTHORS.md) — a curated list grouped by area of focus (Repository Architecture, Docking pipeline and simulation, etc.). If your Pull Request adds a substantive feature, documentation rewrite, or simulation asset, you may add yourself to the relevant section in the same PR.
3. **Top-level [`README.md`](README.md)** — a short *Contributors* section links to `AUTHORS.md` so visitors landing on the project page can find attribution in one click.

Trivial changes (typos, formatting, dependency bumps) are recognized through GitHub history only.

Maintainers may reorganize entries in `AUTHORS.md` to keep the file readable, but they will not remove a recognized contributor without consent.

Beyond the repository itself, the OpenAMRobot organization may also promote contributor work through release notes, social channels, and educational material, with the contributor's consent.

## Maintainers

This repository is maintained by the OpenAMRobot organization.

Maintainers are responsible for:

- reviewing Pull Requests
- protecting the project architecture and inter-package boundaries
- maintaining documentation quality
- checking licensing and contribution hygiene
- keeping the stack reproducible and useful for robotics education and engineering
