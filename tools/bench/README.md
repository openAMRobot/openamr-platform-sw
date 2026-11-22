<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Read-only bench diagnostics

Shell scripts for the bench: they read the state of the robot's host computer, its ROS 2 graph,
the LiDAR serial link and the CAN interfaces. **No script moves the robot.** None publishes a
message, calls a service, sends a CAN frame, changes a CAN link setting or a parameter, or
starts the base bridge. `test/test_bench_readonly.py` fails if any script gains such a command.

The motion and encoder scripts of the existing robot are in `../diagnostics/` and are separate:
most of those command the motors.

Each script writes to `bench_out/<script>-<UTC time>/` (change with `--out DIR`): one file per
command plus `summary.log`, and it ends with the list of what it did not check. Every command
has a timeout. A failed, blocked or timed-out command is recorded and the script continues.
The output is development evidence for the bench record, not physical, release or safety
acceptance.

## Order

Source ROS 2 and the workspace the robot runs (`/opt/ros/jazzy/setup.bash`, then the
workspace `install/setup.bash`) with the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` as the
shell that started the bringup.

| Step | Command | Needs connected |
|---|---|---|
| 1 | `tools/bench/ros_graph_snapshot.sh` | robot host with the bringup running |
| 2 | `tools/bench/lidar_identify.sh` | LiDAR on USB; driver running for section 3 |
| 3 | `tools/bench/lidar_identify.sh --probe --port /dev/serial/by-id/<adapter>` | LiDAR on USB, **driver stopped** |
| 4 | `tools/bench/can_readonly.sh --iface <name> --seconds 10` | CAN adapter with the interface already up |

Step 1 first, so the graph is recorded before anything is restarted. Without a device the
scripts still run and report the missing device as not checked.

## What each script records

- `ros_graph_snapshot.sh`: ROS and middleware variables, network interfaces and routes, nodes,
  each topic with type and publisher and subscriber counts (`topic_counts.tsv`), the endpoints
  of every topic whose name contains `cmd_vel`, the TF tree (`tf2_tools view_frames`, PDF and
  GV in the output folder), `/tf_static`, bounded rates of `/odom`, `/odom/unfiltered`, `/scan`
  and `/imu/data`, `ros2 doctor --report`.
- `lidar_identify.sh`: serial devices by id, udev properties, owner, group and mode, user
  groups, which process holds the port, kernel USB lines, the running LiDAR node (`/rplidar`
  on v0.0.1, `/sllidar_node` on main) with its parameters, `ros2 topic info /scan -v`,
  `ros2 node info`, a bounded `/scan` rate and header, and `tf2_echo base_link lidar_link`
  (frame names from the platform description). With `--probe` it starts the `sllidar_ros2`
  node alone for a bounded time at 115200 baud (RPLIDAR A1) and 1000000 baud (RPLIDAR S3),
  the rates in `openamrobot_drivers/launch/drivers.launch.py`, and records the S/N, firmware,
  hardware revision and health the driver logs. It refuses to probe while a LiDAR driver runs
  or another process holds the port. The probe spins the LiDAR's own scan motor; nothing else
  moves. The probe's scan goes to `/bench/lidar_probe/scan`, not `/scan`.
- `can_readonly.sh`: `ip -details -statistics link show` for CAN and vcan interfaces, state,
  bitrate and sample point, bus error counters, restart setting, whether LISTEN-ONLY is on,
  kernel counters, then a bounded `candump` with error frames and a count per CAN ID. It only
  accepts `--iface`, `--seconds` (1 to 120) and `--out`, and refuses any argument that looks
  like a frame, a sending tool or a link change. It does not bring an interface up or change
  its bitrate or mode. `candump` sends nothing, but a controller that is not in listen-only
  mode still acknowledges frames on the bus in hardware; the script reports the mode.

## Not covered here

The LiDAR model sticker (photograph it), the measured LiDAR mount, the deployed commit, CAN
termination, whether each bitrate matches the node on its bus, firmware flashing and anything
that needs motion. The hardwired stop chain is tested on its own record; nothing here is
safety evidence.

## Tests

```bash
python3 -m pytest -q tools/bench/test
shellcheck -x -P . tools/bench/*.sh
```

CI: `.github/workflows/bench-tools-tests.yml`. The tests read the scripts and run the argument
refusal of `can_readonly.sh`; they use no device.
