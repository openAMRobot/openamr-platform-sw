# OpenAMR Platform Software

📖 **[README](README.md)** &nbsp;·&nbsp;
🤝 **[Contributing](CONTRIBUTING.md)** &nbsp;·&nbsp;
📜 **[License](LICENSE)** &nbsp;·&nbsp;
🔒 **[Security](SECURITY.md)** &nbsp;·&nbsp;
👥 **[Authors](AUTHORS.md)** &nbsp;·&nbsp;
📝 **[Changelog](CHANGELOG.md)** &nbsp;·&nbsp;
ℹ️ **[Notice](NOTICE.md)**

---

ROS 2 Jazzy software stack for the **OpenAMRobot** mobile robot platform: robot description, Gazebo Harmonic simulation, Nav2 navigation, AprilTag-based autodocking, and the launch compositions that wire it all together.

> Maturity: **experimental.** Tuned end-to-end in the docking simulation. Real-robot bringup (drivers, control, hardware integration) is in progress and lives under the `openamrobot_drivers`/`openamrobot_control` packages (not yet populated in this revision).

---

## Quickstart — docking simulation

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp     # one-time
git clone https://github.com/openAMRobot/openamr-platform-sw.git
cd openamr-platform-sw

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
source /opt/ros/jazzy/setup.bash

colcon build --packages-up-to openamrobot_docking openamrobot_bringup
source install/setup.bash

ros2 launch openamrobot_docking docking_sim.launch.py
# In another terminal, once Nav2 is active (~10 s):
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

The robot navigates to a staging zone, scans for the AprilTag, aligns, then drives onto the dock. Expect it to stop ~0.9 m in front of the tag.

> **Why CycloneDDS?** The default Jazzy FastDDS has a Python-side crash bug that makes `dock_trigger.py` exit silently when sending action goals. CycloneDDS is the supported transport for this revision.

---

## Three launch entry points

The stack is **composable**: each ROS 2 package owns a single concern and exposes one launch. Higher-level launches in `openamrobot_bringup` (and the docking-specific one in `openamrobot_docking`) compose them.

Each level below is a **complete, self-contained test scenario** — run the command in one terminal, follow the step-by-step in the next. **Pick the lowest level that gives you what you need** — every extra layer costs startup time and CPU.

> Before you start any of the three: make sure every terminal exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and has sourced `install/setup.bash`. See the [Quickstart](#quickstart--docking-simulation) above and [DDS / RMW](#dds--rmw-use-cyclonedds-not-fastdds) below.

### Level 1 — `sim_only`: Gazebo + the robot

Just the simulator and the URDF. **No Nav2, no RViz, no docking.** Use this to validate physics, friction, and sensors when the navigation stack would only get in the way.

**Terminal 1 — start the simulator:**

```bash
ros2 launch openamrobot_bringup sim_only.launch.py
```

Wait until Gazebo opens and you see the robot spawned at the world origin in the empty `walled_world.sdf` arena.

**Terminal 2 — drive it manually:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then use the keyboard to publish `/cmd_vel`:

| Key | Action |
|---|---|
| `i` / `,` | forward / backward |
| `j` / `l` | rotate left / right |
| `u` `o` `m` `.` | diagonals |
| `q` / `z` | increase / decrease overall speed |
| `w` / `x` | increase / decrease linear speed only |
| `e` / `c` | increase / decrease angular speed only |
| `k` or space | stop |

> The `teleop_twist_keyboard` window **must keep keyboard focus** for the keys to be picked up. Click on its terminal before typing.

**Pass condition:** the robot physically moves in Gazebo when you press `i`. If it doesn't, the `/cmd_vel` chain is broken — see [The `/cmd_vel` chain](#the-cmd_vel-chain--what-makes-the-robot-move) below.

### Level 2 — `sim_nav`: Gazebo + Nav2 + SLAM + RViz (no docking)

Adds Nav2 + SLAM Toolbox + the pre-configured RViz layout on top of level 1. Use this to tune the navigation controller, validate costmaps, or build maps. **No AprilTag, no dock.**

**Terminal 1 — start the stack:**

```bash
ros2 launch openamrobot_bringup sim_nav.launch.py
```

Wait **~10 seconds** for the Nav2 lifecycle nodes to come up:
- Gazebo opens with the empty arena (immediate)
- RViz opens with the Nav2 default layout (~2 s later)
- Nav2 starts in a separate `component_container` (~4 s later)
- SLAM Toolbox starts after a deliberate 4 s delay (so `/scan` and `/tf` are flowing when it boots — see [openamrobot_nav2/README.md](ros2/src/openamrobot_nav2/README.md))

You'll know Nav2 is ready when RViz stops printing TF warnings and the **GlobalCostmap** display fills in around the robot.

**In RViz — send a navigation goal:**

The `2D Goal Pose` tool is in the **top toolbar of RViz**, between `2D Pose Estimate` and `Publish Point`:

```
File Edit View Tools …                    [+ Add  Remove  Rename  Save…]
─────────────────────────────────────────────────────────────────────────
[Interact] [Move Camera] [Select] [Focus]   [Measure] [2D Pose Estimate]
                                            ┌──────────────────────┐
                                            │  2D Goal Pose   ←    │  ← THIS ONE
                                            └──────────────────────┘
                                            [Publish Point] [Nav2 Goal] …
```

To use it:

1. Click `2D Goal Pose` (or press `g` — keyboard shortcut for the tool).
2. Click somewhere on the map (the dark grey area RViz shows around the robot).
3. **Drag** in the direction you want the robot to face at the goal — release when the green arrow points the right way.

The robot should plan a path (shown as a coloured `Path` line in RViz) and start driving toward the goal. Once it arrives within the planner's `tolerance: 1.0` m, the goal completes.

> If you don't see `2D Goal Pose` in the toolbar, the RViz layout file isn't loaded. Run with `use_rviz:=False` and open RViz manually with the right config:
> ```bash
> ros2 run rviz2 rviz2 -d $(ros2 pkg prefix openamrobot_nav2)/share/openamrobot_nav2/rviz/nav2_default.rviz
> ```

**Pass condition:** the robot drives to the clicked goal and stops, with the lidar visibly building up a map as it moves.

### Level 3 — `docking_sim`: everything, including the dock

Adds the AprilTag detector, the dock model in the world, and the 4-phase docking sequencer (`dock_trigger.py`). **This is the full end-to-end pipeline.**

**Terminal 1 — kill leftover sim processes from a previous run** (Gazebo doesn't always clean up after Ctrl-C):

```bash
bash scripts/kill_sim.sh
```

**Terminal 1 — start the stack:**

```bash
ros2 launch openamrobot_docking docking_sim.launch.py
```

Wait **~10 seconds** for everything to come up:
- Gazebo opens with `docking_scenario.sdf` (10×10 m room, AprilTag dock against the north wall)
- The robot spawns at `(-4, -4, 0)` (south-west corner facing east)
- Nav2 + SLAM activate
- The AprilTag detector starts listening on `/camera/image_raw`
- `detected_dock_pose_publisher` starts (publishes `/detected_dock_pose` once the tag is in view)
- RViz opens with the docking layout

**Terminal 2 — trigger the docking sequence:**

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once
```

What you should see in the launch terminal (truncated, key lines only):

```
[dock_trigger.py-N] [INFO] [dock_trigger]: ── Phase 1/4: NavigateToPose → staging (4.0, 6.4, 1.57)
[dock_trigger.py-N] [INFO] [dock_trigger]: Goal succeeded
[dock_trigger.py-N] [INFO] [dock_trigger]: ── Phase 2/4: tag search + initial filter (40 samples)
[dock_trigger.py-N] [INFO] [dock_trigger]:    scanning to centre tag in camera …
[dock_trigger.py-N] [INFO] [dock_trigger]:    tag centred after K consecutive frames
[dock_trigger.py-N] [INFO] [dock_trigger]:    collected 40/40 samples, running avg pose …
[dock_trigger.py-N] [INFO] [dock_trigger]: ── Phase 3/4: align spin to perpendicular yaw
[dock_trigger.py-N] [INFO] [dock_trigger]: ── Phase 4/4: line-tracking advance
[dock_trigger.py-N] [INFO] [dock_trigger]:    visual servo distance reached — straight-line
[dock_trigger.py-N] [INFO] [dock_trigger]: Docked. Stopping.
```

**Pass condition:** the robot stops perpendicular to the tag, ~0.9 m in front of it (the `docking_distance` parameter). In RViz you can verify visually: the `RobotModel` should be centred on the perpendicular line through the dock's AprilTag.

If it fails at phase 2 with `tag never detected`, you have the AprilTag pipeline regression that bit us before — see the [troubleshooting matrix in openamrobot_docking](ros2/src/openamrobot_docking/README.md#troubleshooting).

> Want to launch from a different spawn pose? All three launches accept `spawn_x:=`, `spawn_y:=`, `spawn_yaw:=`. The docking launch additionally **re-projects the dock pose into the resulting map frame** so the sequencer still aims at the same physical tag:
> ```bash
> ros2 launch openamrobot_docking docking_sim.launch.py \
>     spawn_x:=2.0 spawn_y:=-3.0 spawn_yaw:=1.5707
> ```

---

## Architecture — package responsibilities

```
ros2/src/
├── openamrobot_description/   URDF + meshes + plugin tags (Gazebo, sensors)
├── openamrobot_gazebo/        Gazebo bringup + ROS<->gz bridge + worlds
├── openamrobot_nav2/          Nav2 stack params + SLAM + RViz layout
├── openamrobot_docking/       AprilTag pipeline + dock model + 4-phase sequencer + docking_sim launch
└── openamrobot_bringup/       Top-level launch compositions (sim_only, sim_nav)
```

Strict separation of concerns:

| Package | Knows about | Does NOT know about |
|---|---|---|
| `_description` | robot kinematics, mass, sensor placement, Gazebo plugins | worlds, Nav2, docking |
| `_gazebo` | Gazebo Harmonic, the ros↔gz bridge, generic worlds | the robot's role, navigation, docking |
| `_nav2` | Nav2 + SLAM params, RViz layout | Gazebo (works on the real robot too), docking |
| `_docking` | AprilTag, the 4-phase sequencer, the dock model + world | how Gazebo / Nav2 are launched |
| `_bringup` | how to compose the above | the internals of any one of them |

Each package can be replaced or refactored without touching the others, provided its public interface (the launch arguments and the ROS topic names) stays the same.

---

## The `/cmd_vel` chain — what makes the robot move

In the docking simulation, the velocity-command flow has six links. If any one breaks, the robot stops moving. This is the single most useful mental model when debugging the sim.

```
dock_trigger.py / Nav2 controller_server  ───>  /cmd_vel_nav
        (phases 2,3,4 of docking)              (the "raw" Nav2 output)
                                                          │
                                                          ▼
                              velocity_smoother  ───>  /cmd_vel_smoothed
                                  (accel/decel limits)
                                                          │
                                                          ▼
                              collision_monitor  ───>  /cmd_vel
                                  (zero on near obstacle)
                                                          │
                                                          ▼
                              ros_gz_bridge      ───>  gz cmd_vel topic
                                                          │
                                                          ▼
                              gz-sim-diff-drive  ───>  joints rotate
                                                          │
                                                          ▼
                              ODE contact solver ───>  friction force ───> robot moves
                                                          │
                                                          ▼
                              gz-sim-diff-drive  ───>  publishes odom + tf
                                                          │
                                                          ▼
                              ros_gz_bridge      ───>  /odom, /tf in ROS
```

Two things tend to break this chain in practice:

1. **The bridge isn't wired** — verify `ros2 topic echo /cmd_vel` shows messages reaching the bridge. If it does but the robot still doesn't move, the bridge isn't connected to the Gazebo side; check `openamrobot_gazebo/config/gz_bridge.yaml`.
2. **The wheels have no traction** — the wheel collision cylinder must be slightly larger than the kinematic `wheel_radius` so ODE has a contact depth. See [openamrobot_description/README.md](ros2/src/openamrobot_description/README.md#wheel-traction--why-collision-radius--kinematic-radius) for the full explanation.

For the second `tf` chain (odometry + map → robot pose), the relevant flow is:

```
SLAM Toolbox     ───>  map → odom        (corrects drift from lidar matching)
gz DiffDrive     ───>  odom → base_footprint    (integrated wheel odometry, via bridge)
robot_state_publisher  ───>  base_footprint → base_link → wheels, sensors, ...   (from URDF + joint_states)
```

`/joint_states` itself is published by Gazebo's `gz-sim-joint-state-publisher-system` (also via the bridge). Do **not** add a standalone `joint_state_publisher` node — it will publish zeros and fight the Gazebo source. See [openamrobot_gazebo/README.md](ros2/src/openamrobot_gazebo/README.md#do-not-add-a-standalone-joint_state_publisher).

---

## Repository layout

```
openamr-platform-sw/
├── ros2/                       ← the colcon workspace (build/install/log live at the repo root)
│   └── src/
│       ├── openamrobot_description/   robot URDF, meshes, Gazebo/sensor plugin tags
│       ├── openamrobot_gazebo/        Gazebo bringup, bridge config, generic worlds
│       ├── openamrobot_nav2/          Nav2 + SLAM Toolbox + RViz nav layout
│       ├── openamrobot_docking/       AprilTag + 4-phase docking sequencer + dock world
│       ├── openamrobot_bringup/       top-level launch compositions (sim_only, sim_nav)
│       ├── openamrobot_control/       (placeholder) low-level control integration
│       ├── openamrobot_drivers/       (placeholder) hardware drivers
│       └── openamrobot_perception/    (placeholder) perception modules beyond docking
│
├── simulation/                 ← reserved for product-level simulation assets (worlds/scenarios shared across packages)
├── config/                     ← reserved for product-level config (deployment, system-wide)
├── scripts/
│   └── kill_sim.sh             ← SIGKILL every sim-related process (use after Ctrl-C zombies)
├── tools/                      ← reserved for dev tools
└── docs/
    ├── docking/                 pointer → ros2/src/openamrobot_docking/docs/
    ├── navigation/              pointer → ros2/src/openamrobot_nav2/
    ├── simulation/              pointer → ros2/src/openamrobot_gazebo/
    └── safety/                  platform-wide safety considerations (TBD)
```

Build/install/log directories are created by colcon at the **repo root** (`./build`, `./install`, `./log`), not inside `ros2/`. This is why both the COLCON_IGNORE markers and the `colcon build` commands work from the top level.

---

## Detailed documentation

Each package ships its own README with the why-and-how:

| Package | What's in the README |
|---|---|
| [openamrobot_description](ros2/src/openamrobot_description/README.md) | URDF anatomy, frame conventions, mass/inertia, sensor placement, why collision radius ≠ kinematic radius |
| [openamrobot_gazebo](ros2/src/openamrobot_gazebo/README.md) | Gazebo bringup flow, complete bridge table, world descriptions, spawn pose convention, common pitfalls |
| [openamrobot_nav2](ros2/src/openamrobot_nav2/README.md) | Nav2 stack composition, parameter rationale, costmap layout, the cmd_vel chain, SLAM tuning |
| [openamrobot_docking](ros2/src/openamrobot_docking/README.md) | 4-phase sequencer, AprilTag detection, dock pose re-projection on spawn override, troubleshooting |
| [openamrobot_bringup](ros2/src/openamrobot_bringup/README.md) | The three launch levels, composition philosophy, when to use which |

Deeper engineering notes (lessons learned, parameter reference, design-decision diary) live under [ros2/src/openamrobot_docking/docs/](ros2/src/openamrobot_docking/docs/).

---

## DDS / RMW: use CycloneDDS, not FastDDS

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Set this in every terminal that runs ROS commands (add it to `~/.bashrc` once). The default Jazzy FastDDS has a Python-side crash bug that surfaces when `dock_trigger.py` sends action goals to Nav2 — the trigger process exits silently with no traceback. CycloneDDS does not have this bug.

---

## Killing zombie processes

If a previous launch was Ctrl-C'd and left things behind (gz-sim, parameter_bridge, slam_toolbox, dock_trigger, …):

```bash
bash scripts/kill_sim.sh
```

This SIGKILLs everything matching the patterns listed in the script. Run it before relaunching when `colcon`, `rviz`, or `gz sim` complains about already-running processes.

---

## Related repositories

| Repo | Contents |
|---|---|
| `openamr-platform-hw` | mechanical, electrical, CAD, BOM, wiring |
| `openamr-platform-fw` | embedded firmware |
| `openamrobot-docs` | user-facing and architecture documentation |
| `openamrobot-interfaces` | shared ROS 2 messages, services, actions |
| `openamrobot-ui` | user / operator interface |

Hardware design files, firmware, and shared interfaces are intentionally **not** in this repo — keep concerns separated so this stack stays reusable on different OpenAMRobot platform revisions.

---

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Safety notice

This repository can affect real robot behaviour. Users are responsible for validating safety, navigation behaviour, docking behaviour, motor control, sensor integration, deployment suitability, and regulatory compliance. The software is provided for research, education, and development.

## License

MIT. See [`LICENSE`](LICENSE). Attribution chain for the URDF (Brawner → Dhakal → Indulkar → this revision) is in [`NOTICE.md`](NOTICE.md).
