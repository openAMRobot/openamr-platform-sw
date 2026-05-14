# Workspace setup

How to clone, build, and source `openamr-platform-sw`.

## 1. Clone

```bash
cd ~
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw
```

## 2. System dependencies

See [01_quickstart_docking_sim.md](01_quickstart_docking_sim.md) for the full apt list (ROS 2 Jazzy + Nav2 + Gazebo Harmonic + apriltag_ros + slam_toolbox + ros_gz_*).

## 3. CycloneDDS (required)

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

This works around a Python-side crash in FastDDS that silently breaks the docking action client.

## 4. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to openamrobot_docking
source install/setup.bash
```

The `--packages-up-to` form builds the four packages in dependency order:

```
openamrobot_description → openamrobot_gazebo
                        → openamrobot_nav2 → openamrobot_docking
```

To build just one layer:

```bash
colcon build --packages-select openamrobot_description     # just the URDF
colcon build --packages-select openamrobot_gazebo          # add the simulator
colcon build --packages-select openamrobot_nav2            # add Nav2 + SLAM
colcon build --packages-select openamrobot_docking         # add the docking pipeline
```

## 5. Verify the build

```bash
ros2 pkg prefix openamrobot_description
ros2 pkg prefix openamrobot_gazebo
ros2 pkg prefix openamrobot_nav2
ros2 pkg prefix openamrobot_docking
```

Each command should print a path under `install/`. If any errors, re-source `install/setup.bash` and retry.

## 6. Next

- Run the docking sim: [01_quickstart_docking_sim.md](01_quickstart_docking_sim.md).
- Understand the layout: [`docs/architecture/01_repo_layout.md`](../architecture/01_repo_layout.md).
