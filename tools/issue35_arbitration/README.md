# Issue #35 arbitration prototype: simulation tools

Review material for [issue #35](https://github.com/openAMRobot/openamr-platform-sw/issues/35).
Status: **prototype, Proposed**, simulation only. The design note is
[`docs/design/issue35-command-arbitration.md`](../../docs/design/issue35-command-arbitration.md).
Kunal Yadav (@skunal3318) owns the implementation; nothing here is approved.

**Never run these against a robot.** The scenario scripts publish velocity commands. They
refuse to start without `/clock`, which only a simulator publishes.

| File | What it does |
|---|---|
| `apply_sim_fixtures.sh` | Applies sim-only corrections to an install tree: F1 body filter for the yaw-pi lidar, F2 sim dock pose, F3 forward cone angle pi. F4 is recorded as rejected. |
| `cmdmon.py` | Records every cmd_vel-type message (with its topic's publisher set), lifecycle transitions, dock status, guard mode and Gazebo ground truth to CSV. |
| `check_run.py` | Offline check of a recorded docking run: C1 single final publisher, C2 monitor stays active, C3 rate limited before the protective stage, C4 docked. |
| `scenarios.py` | Live sim scenarios: priority and handover, stop lock over teleop, lock without a zero source, teleop and dock publisher loss. |
| `option_a_checks.py` | Standalone twist_mux in `/a_check` (no robot topics): output type with the default `use_stamped`, silence on timeout, lock 255 against teleop 255 and 100. |
| `scanstat.py` | Prints scan returns within 1 m (self-view measurement). |

## Run

ROS 2 Jazzy, Gazebo Harmonic, the packages in `package.xml` plus `ros-jazzy-twist-mux`.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2 && colcon build && source install/setup.bash
bash ../tools/issue35_arbitration/apply_sim_fixtures.sh install
export ROS_DOMAIN_ID=35
ros2 launch openamrobot_bringup bringup.launch.py sim:=true &
# ground truth: the bridge drops Pose_V names; the robot is entry 6 in walled_world
ros2 run ros_gz_bridge parameter_bridge \
  '/world/walled_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  --ros-args -r /world/walled_world/pose/info:=/gt_poses &
python3 ../tools/issue35_arbitration/cmdmon.py run.csv /cmd_vel_mux /cmd_vel_dock /cmd_vel_joy /cmd_vel_stop &
ros2 topic pub --once /dock_trigger std_msgs/msg/Bool '{data: true}'
# after 'docked':
python3 ../tools/issue35_arbitration/check_run.py run.csv
# fresh sim, robot at the spawn point:
python3 ../tools/issue35_arbitration/scenarios.py out/
python3 ../tools/issue35_arbitration/option_a_checks.py out/
```

`check_run.py` is the deliberate-fault check: a docking run recorded on `main` fails C1, C2
and C3, and the prototype passes all four (results in the design note).
