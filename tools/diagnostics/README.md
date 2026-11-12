# Diagnostics tools

Standalone scripts used to bring up and **diagnose the real robot** at the hardware/control
level. They talk to the Teensy firmware's `/debug/*` topics (see
`openamr-platform-fw/boards/teensy_4_0/linorobot2_overlay`) and to `/cmd_vel`, `/odom`,
`/scan`, `/imu/data`, the camera. They are **not** part of the runtime stack — run them by
hand when something misbehaves.

> Most read/command motors. **Wheels off the ground** (or clear space + a hand on the 24 V
> cut-off), and start at low PWM. Source ROS 2 + the robot's RMW (`rmw_cyclonedds_cpp`)
> first.

| Script | What it does |
|---|---|
| `openloop_test.py` | Drives both wheels at equal PWM via `/debug/openloop` (PID bypassed) and compares them — isolates a healthy motor/encoder from a closed-loop fault. |
| `powered_debug_test.py` | Small `/cmd_vel`, logs per-wheel rpm/counts/PWM, auto-aborts on runaway. |
| `guided_encoder_test.py` | Hand-spin encoder check. |
| `sign_test.py` | Encoder direction sign (forward → positive). |
| `encread.py` | Live encoder counts (hand turn). |
| `encpid.py` | Per-wheel target/measured/error during PID. |
| `high_rate_capture.py` | 50 Hz capture around a jerk (real oscillation vs encoder glitch). |
| `raw_debug_monitor.py` | Live print of `/debug/*`. |
| `yawtest.py` | Yaw/odometry ground-truth check. |
| `lidar_view.py` | Headless LiDAR view. |
| `cam_snapshot.py` | Grab a camera frame to a file. |

For the method and the faults these uncovered, see
[`docs/troubleshooting/diagnostics.md`](../../docs/troubleshooting/diagnostics.md).
