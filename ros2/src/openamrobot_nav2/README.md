# openamrobot_nav2

ROS 2 package for the **OpenAMRobot** Nav2 navigation stack: bringup, parameters, SLAM, RViz layout.

## Contents

```
openamrobot_nav2/
├── config/
│   ├── nav2_params.yaml          ← Nav2 stack (RPP + NavfnPlanner + costmaps + smoother + collision_monitor)
│   └── slam_toolbox_params.yaml  ← SLAM Toolbox in mapping mode, 0.05 m grid, 10 m lidar range
├── launch/
│   └── nav2_bringup.launch.py    ← Nav2 + SLAM bringup (composes with openamrobot_gazebo)
├── rviz/
│   └── nav2_default.rviz         ← RobotModel + LaserScan + Map + GlobalCostmap + TF + Plan + LocalPlan
├── maps/                          ← user-saved maps (gitkept, empty by default)
└── behavior_trees/                ← optional custom BTs (gitkept, empty by default)
```

## Usage

The nav2 bringup is intentionally separated from the simulator launch — compose them in the parent launch (e.g. the docking simulation does this).

```bash
ros2 launch openamrobot_gazebo gz_simulator.launch.py &
ros2 launch openamrobot_nav2 nav2_bringup.launch.py
```

To open the RViz layout:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix openamrobot_nav2)/share/openamrobot_nav2/rviz/nav2_default.rviz
```

## Configuration highlights

- **Controller**: `RegulatedPurePursuitController` — `desired_linear_vel: 0.55 m/s`, `rotate_to_heading_angular_vel: 0.5 rad/s`.
- **Planner**: `NavfnPlanner` with `use_astar: true`, `tolerance: 1.0 m` (accepts a free cell within 1 m of the goal if the exact goal cell is in a transient inflation zone).
- **Costmaps**: global rolling 20×20 m with `obstacle_layer + inflation_layer` (`inflation_radius: 0.45 m`, `cost_scaling_factor: 8.0`); local rolling 3×3 m with `voxel_layer + inflation_layer`.
- **velocity_smoother**: linear `max_accel = 1.2 m/s²`, **`max_decel = -0.5 m/s²`** (softened to avoid overshoot with the caster meshes), `max_velocity = 0.7 m/s`.
- **collision_monitor**: `FootprintApproach` disabled by default — re-enable it for environments with real near-obstacle risk.

## What this package does NOT contain

- Robot URDF/xacro/meshes → `openamrobot_description`
- Gazebo plugins, worlds, ros_gz_bridge → `openamrobot_gazebo`
- Docking logic, AprilTag detection, dock world → `openamrobot_docking`

## Status

Experimental. Tuned in the docking simulation scenario; expect to adapt costmap radii / accel limits to the actual environment.
