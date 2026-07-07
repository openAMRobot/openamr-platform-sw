# openamrobot_nav2

ROS 2 package for OpenAMRobot Nav2 bringup and navigation configuration — combines **Nav2**, **SLAM Toolbox**, and **AMCL** into a ready-to-run navigation stack for **both simulation and the real robot** (one `nav2_params.yaml`, switched by `use_sim_time`).

## Contents

This package contains:

- Nav2 launch files
- Navigation parameters
- SLAM Toolbox configuration
- AMCL configuration
- Maps
- RViz navigation layouts
- Behavior trees

## Boundaries

This package should not contain:

- Robot URDF/xacro files
- Robot meshes
- Gazebo robot description files
- Docking controller logic

Those belong to the `openamrobot_description`, `openamrobot_gazebo`, and `openamrobot_docking` packages.

## Status

Experimental.

---

## Package Layout

```
openamrobot_nav2/
├── behavior_trees/             # Custom Nav2 behavior tree XML files
├── config/
│   ├── slam.yaml               # SLAM Toolbox parameters
│   ├── nav2_params.yaml        # Nav2 stack parameters
│   └── scan_body_filter.yaml   # Angular filter — clips rear 90° from /scan
├── launch/
│   ├── online_async_launch.py  # SLAM mapping (async mode)
│   ├── localization_launch.py  # AMCL localization on a saved map
│   ├── navigation_launch.py    # Full Nav2 navigation stack
│   └── sim_bringup_launch.py   # All-in-one: localization + nav + RViz
├── maps/
│   ├── my_map.pgm              # Pre-built occupancy grid image
│   └── my_map.yaml             # Map metadata (resolution 0.05 m/px)
├── rviz/
│   └── nav2_view.rviz          # RViz preset for navigation
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| ROS 2 | Jazzy |
| Nav2 | Jazzy release |
| SLAM Toolbox | Jazzy release |
| Gazebo | Harmonic |
| Python | 3.10+ |

Install Nav2 and SLAM Toolbox:

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-laser-filters \
  ros-jazzy-opennav-docking
```

---

## Building

```bash
cd openamr-platform-sw
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamrobot_nav2
source install/setup.bash
```

Do not mix install spaces. If you source `openamr-platform-sw/install`, build
from `openamr-platform-sw`; if you source `openamr-platform-sw/ros2/install`,
build from `openamr-platform-sw/ros2`.

---

## Usage

### 1 — Build a Map (SLAM)

```bash
ros2 launch openamrobot_nav2 online_async_launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo clock |
| `slam_params_file` | `config/slam.yaml` | Path to SLAM parameters |
| `autostart` | `true` | Auto-activate lifecycle node |
| `use_lifecycle_manager` | `false` | Enable bond connection |

Save the map when done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 2 — Localize on a Saved Map

```bash
ros2 launch openamrobot_nav2 localization_launch.py \
  map:=/path/to/my_map.yaml \
  use_sim_time:=true
```

| Argument | Default | Description |
|---|---|---|
| `map` | *(empty — uses params file)* | Full path to map YAML |
| `use_sim_time` | `false` | Set `true` for Gazebo |
| `params_file` | `config/nav2_params.yaml` | Nav2 parameters |
| `use_composition` | `false` | Run nodes in a composable container |
| `use_respawn` | `false` | Restart nodes on crash |
| `log_level` | `info` | ROS logging level |

### 3 — Run the Navigation Stack

```bash
ros2 launch openamrobot_nav2 navigation_launch.py use_sim_time:=true
```

Starts: `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, and `waypoint_follower`.

Current simulation wiring publishes Nav2 velocity commands directly to `/cmd_vel`,
which is bridged to Gazebo's diff-drive plugin. `velocity_smoother` and
`collision_monitor` remain configured in `nav2_params.yaml` for future tuning,
but they are not launched in the current direct-motion setup.

### 4 — All-in-One Simulation Bringup

Starts localization (with the bundled map), the full Nav2 stack, and RViz in one command:

```bash
ros2 launch openamrobot_nav2 sim_bringup_launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_rviz` | `false` | Start RViz with the bundled navigation view |

---

## Configuration

### SLAM Toolbox (`config/slam.yaml`)

| Parameter | Value | Effect |
|---|---|---|
| `resolution` | `0.05` | Map resolution (metres/cell) |
| `max_laser_range` | `10.0 m` | Maximum usable laser range |
| `map_update_interval` | `5.0 s` | How often the map image is updated |
| `do_loop_closing` | `true` | Enables loop-closure correction |
| `scan_topic` | `/scan` | Expected laser topic name |
| `mode` | `mapping` | Switch to `localization` to reuse a map |

### Nav2 (`config/nav2_params.yaml`)

| Node | Plugin / Key values |
|---|---|
| `planner_server` | SmacPlanner2D (A*), tolerance 0.5 m |
| `controller_server` | DWB local planner, 20 Hz, direct `/cmd_vel`, **max 0.16 m/s** / 2.0 rad/s; critics include **`ObstacleFootprint`** (checks the whole footprint, not just the centre — needed for the large non-circular base) |
| `local_costmap` | 3×3 m rolling window, VoxelLayer + InflationLayer (**radius 0.15 m**), **real footprint** (0.78×0.58 m octagon, base_link offset, +0.12 m padding) — not `robot_radius`; `obstacle_min_range 0.0` (body already removed by the scan filter) |
| `global_costmap` | StaticLayer + ObstacleLayer + InflationLayer (**radius 0.40 m**, cost_scaling 2.5 → plan stays centred / clear of walls), same real footprint |
| `amcl` | Differential motion model, 500–2000 particles, likelihood field |
| `velocity_smoother` | Config present but not launched in the current direct `/cmd_vel` setup |
| `collision_monitor` | Config present but not launched in the current direct `/cmd_vel` setup |
| `docking_server` | Parameters present for future Nav2 docking integration; current autodocking uses `openamrobot_docking` |

> **Shared sim + real profile.** `nav2_params.yaml` is the SAME file for simulation and the
> real robot — `use_sim_time` is set by the launch, not the file. The values above are the
> real-robot-validated tuning for the same physical 0.78×0.58 m robot (which Gazebo also
> simulates). The old sim `robot_radius: 0.22` was a simplification; the real footprint +
> `ObstacleFootprint` make both profiles accurate. `max_vel_x 0.16` is the real hardware
> limit (sim runs at the same speed). Switch profiles with
> `ros2 launch openamrobot_bringup bringup.launch.py sim:=true|false`.

### Motion Debugging

After launching, verify the command path:

```bash
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
```

Expected with the current launch: `/cmd_vel` has a Nav2 publisher and a Gazebo
bridge subscriber. If `velocity_smoother` or `collision_monitor` appears in the
launch log, an old install space is being sourced.

### Pre-built Map (`maps/my_map.yaml`)

| Property | Value |
|---|---|
| Resolution | 0.050 m/px |
| Origin | `[-5.391, -5.189, 0]` |
| Occupied threshold | 0.65 |
| Free threshold | 0.196 |

---

## Related Packages

- `openamrobot_description` — URDF/xacro robot model
- `openamrobot_gazebo` — Gazebo simulation bringup
- `openamrobot_docking` — Docking controller logic

---

## License

MIT
