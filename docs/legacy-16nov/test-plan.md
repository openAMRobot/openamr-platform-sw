# 16 Nov session test plan: 2.0 base software on the v0.0.1 platform

Status: proposal. Owners: Mohamed (pinned versions, plan), Hamed and Arka (bench checks and
the integrated run), Kunal (ROS 2 and Nav2 command path), Evans (Teensy firmware).
Interface details and file references: [compatibility.md](compatibility.md).

Every value that is not known yet is marked TODO(owner). Do not fill one in from memory on
the day: measure it, record it, then proceed.

## 1. Pinned versions (session tuple)

Freeze this tuple before the rehearsal and do not change it on the day. Record the actual
SHAs in the session log if any row moves.

| Item | Pin | Note |
|---|---|---|
| openamr-platform-sw | this branch (legacy_v001 profile) on top of main `f71f9f1` | TODO(Mohamed) final SHA after review |
| openamr-platform-sw PR #90 (stop gate, twist_mux, UI relay) | head `2e0b8f3` or its merged successor | Required for Stop, Resume and manual drive through the chain. See compatibility row 11: Resume is refused without dock_trigger unless #90 changes. TODO(Kunal) |
| openamr-platform-sw PR #129 (EKF /imu/data_raw) | merged or not: either works with legacy_v001 | legacy_v001 sets imu0 itself |
| openamrobot-interfaces | PR #35 head `cb83b22` (StopState) | needed by #90; TODO(Mohamed) |
| openamrobot-ui | PR #50 head `cf27626` | `/ui/cmd_vel` and the Stop lock; served bundle digest TODO(Mohamed) |
| Teensy firmware | platform-fw PR #33 head `c3f7409` on linorobot2_hardware `aaf9d59`, debug topics compiled out | binary digest TODO(Evans) |
| micro_ros_agent | source build | SHA TODO(Mohamed) |
| sllidar_ros2 | Slamtec commit `34300099` | as in `openamrobot_drivers/launch/drivers.launch.py` |
| ROS 2 / Nav2 | Jazzy, nav2 1.3.13 binaries | `apt list --installed 'ros-jazzy-nav2*'` saved to the log |
| Map | map YAML and PGM of the session area | path and sha256 TODO(Arka) |

## 2. Preconditions (all must hold before any wheel turns)

| # | Check | Pass | Owner |
|---|---|---|---|
| P1 | Temporary mushroom E-stop fitted, cuts motor driver supply (not logic) (roadmap risk 3) | Pressed with wheels up at a commanded 0.2 m/s: wheels stop; release does not restart motion | Arka |
| P2 | Teensy flashed with the pinned image; digest matches | Digest recorded in the log equals the tuple | Evans, Arka |
| P3 | Host has the tuple installed, workspace built | `colcon build` clean; `ros2 pkg prefix openamrobot_bringup` resolves | Hamed |
| P4 | Device paths | Teensy and LiDAR by-id paths exist (`ls /dev/serial/by-id/`); if they differ from the defaults in `drivers.launch.py`, TODO(Arka) record them and pass them as overrides | Arka |
| P5 | Floor area | Clear area, map matches, people briefed, one person on the E-stop at all times | Arka |
| P6 | Battery | Pack voltage recorded; TODO(Arka) minimum voltage for the run | Arka |
| P7 | UI battery node | The UI package `battery` node is not started (it publishes a simulated value without a serial port) | Hamed |

## 3. Step order

Run in this order. Stop at the first FAIL, record it, and decide with Kunal whether to fix
and repeat or skip the rest of that block. Never skip a block whose later steps depend on it.

### A. Bring-up (wheels up, motors powered)

Command:

```
ros2 launch openamrobot_bringup legacy_v001.launch.py map:=<session map yaml>
```

| # | Step | Pass | Fail |
|---|---|---|---|
| A1 | Launch | All lifecycle nodes active within 30 s; no `waypoint_follower` node in `ros2 node list` | Any node in error, or `waypoint_follower` present |
| A2 | Topics | `/odom/unfiltered`, `/imu/data`, `/scan`, `/scan_filtered`, `/odom` publish (`ros2 topic hz`). `/imu/data_raw` absent | Any missing; if `/imu/data_raw` is present instead of `/imu/data`, stop and change `IMU_TOPIC` (compatibility row 6) |
| A3 | Rates | Record measured rates: `/odom/unfiltered` TODO(Hamed) (expected near 50 Hz), `/scan` TODO(Hamed), `/odom` near 30 Hz | Rate under half of the expected value |
| A4 | TF | `ros2 run tf2_tools view_frames`: map -> odom -> base_link -> {base_footprint, lidar_link, imu_link}; one parent each | Any frame with two parents, or a second odom -> base publisher |
| A5 | Single writer | `ros2 topic info /cmd_vel -v`: one publisher (collision_monitor), one subscriber (the Teensy via the agent) | More than one publisher |
| A6 | Status | `navigation_status_node` reports profile `legacy_v001`; nothing reports PROTECTED; base status shows missing | Any protection state reported better than true |

### B. Watchdog and manual driving (wheels up first, then floor at 0.20 m/s)

| # | Step | Pass | Fail |
|---|---|---|---|
| W1 | Wheels up: `ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.1}}'` (needs the lock released after #90), then Ctrl-C | Wheels turn, then stop within 200 ms of the last message (Teensy watchdog, 50 Hz loop) | Wheels keep turning |
| W2 | Wheels up: stop the micro-ROS agent process | Wheels stop; LED pattern recorded | Wheels keep turning |
| W3 | Wheels up: press the temporary E-stop while turning | Wheels stop; release does not restart motion | Any restart on release |
| M1 | Floor: drive forward and back with the UI joystick | Robot moves at no more than 0.20 m/s (from `/odom`), stops when the stick is released | Speed above 0.20 m/s, or no stop on release |
| M2 | Floor: turn in place both ways | Rotation direction matches the stick; `/odom` yaw rate sign matches (IMU sign check, compatibility row 7) | Sign mismatch |
| M3 | Floor: close the UI tab while driving | Robot stops within 0.25 s relay staleness plus the 200 ms watchdog | Robot keeps moving |
| M4 | Odometry scale: drive 2.0 m straight along a tape | Measured distance vs `/odom`: record the error, TODO(Hamed) acceptance threshold | n/a: record |

### C. Localization and goals (floor)

| # | Step | Pass | Fail |
|---|---|---|---|
| L1 | Set the initial pose in the UI or RViz | AMCL converges, scan aligns with the map | No convergence after 2 m of driving |
| G1 | One navigate_to_pose goal, 3 m in free space | Reaches the goal at no more than 0.20 m/s; stops | Wrong direction, overshoot beyond the goal tolerance, speed above limit |
| G2 | Goal behind an obstacle | Plans around it | Collision or approach inside the footprint |
| C1 | Cancel from the UI during G1 | Robot stops; goal reported CANCELED | Robot keeps moving |
| C2 | Send a new goal, then Stop (software) | Robot stops; lock engaged in the UI; the goal is cancelled | Any motion after Stop |
| C3 | navigate_through_poses with two poses (RViz Nav2 panel, waypoint mode "through poses"), then Stop | Robot stops; through-poses goal cancelled | Robot continues to the next pose |
| C4 | Confirm no waypoint follower | `ros2 action list` has no `/follow_waypoints` | Present |

### D. Stop and Resume with the A1 rule (P-03 item 30 A1)

| # | Step | Pass | Fail |
|---|---|---|---|
| S1 | Clean boot of the stack; measure time to Resume available | Resume available within 2 s after the stack is up (the gate starts locked) | Over 2 s, or Resume never available; if refused, record the reason (expected on PR #90 head without docking: `dock_trigger: status stale`) |
| S2 | Stop during a goal, then Resume | Lock clears; robot does not move until a new goal or manual input | Any motion on Resume |
| S3 | Stop, kill `bt_navigator`, try Resume | Resume refused (producer missing or stale = UNKNOWN) | Resume accepted |
| S4 | Stop while the joystick is held, try Resume | Refused while the manual input is nonzero and fresh | Accepted |
| S5 | Resume, then "Send last goal again" in the UI if available | A new goal is sent only on that explicit action | Goal resumes by itself |

### E. Recovery

| # | Step | Pass | Fail |
|---|---|---|---|
| R1 | Block the path fully during a goal | Recovery behaviours run (bounded); the goal aborts with a reason; robot stops | Endless recovery, or motion into the obstacle |
| R2 | Unplug the LiDAR USB during a goal | Robot stops (collision_monitor source timeout); status shows LiDAR stale | Robot keeps moving |
| R3 | Restart the stack without power-cycling the Teensy | Agent reconnects; stack up again; lock engaged | Teensy needs a power cycle (record) |

## 4. What to record

For every step: time, step ID, PASS/FAIL, operator, and a note.

- `ros2 bag record` for the whole floor session: `/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_smoothed`,
  `/odom`, `/odom/unfiltered`, `/imu/data`, `/scan`, `/scan_filtered`, `/tf`, `/tf_static`,
  `/amcl_pose`, `/plan`, `/collision_monitor_state`, `/safety/stop_lock`, `/safety/stop_state`,
  `/ui/cmd_vel`, `/navigation/status`, `/diagnostics`, `/rosout`.
- `ros2 node list`, `ros2 topic list -t`, `ros2 action list` and `view_frames` output after A1.
- Measured rates (A3), odometry scale error (M4), IMU sign (M2), stop times (W1, W3, M3).
- Video of every floor step.
- The installed tuple: `git rev-parse HEAD` for each workspace repo, the Teensy digest, the UI bundle digest, `apt list --installed 'ros-jazzy-*'`.

## 5. Rehearsal checklist (before 16 Nov)

| # | Item | When | Owner |
|---|---|---|---|
| H1 | Tuple frozen in the table above, all TODO(Mohamed) rows filled | TODO(Mohamed) | Mohamed |
| H2 | `colcon build` and `colcon test` for openamrobot_bringup and openamrobot_nav2 green on the tuple | TODO(Mohamed) | Mohamed |
| H3 | Decide between legacy_v001 and PR #131 hday_legacy (one profile on the day) | TODO(Mohamed) | Kunal |
| H4 | PR #90 Resume behaviour without docking resolved (compatibility row 11) | TODO(Mohamed) | Kunal |
| H5 | Dry run of block A in simulation is not possible with this profile (it needs the Teensy). Instead: launch legacy_v001 on the host with the Teensy unplugged and check that every node except the agent and the LiDAR comes up | TODO(Mohamed) | Hamed |
| H6 | Sensors-only bench on the rig, motors unpowered: blocks A1 to A4, A6 | TODO(Mohamed) | Arka |
| H7 | Temporary E-stop fitted and tested (P1) | TODO(Mohamed) | Arka, Evans |
| H8 | Teensy image without debug topics flashed, digest recorded (P2) | TODO(Mohamed) | Evans, Arka |
| H9 | Session map recorded and copied to the host (sha256 noted) | TODO(Mohamed) | Arka |
| H10 | Go/no-go against this plan | TODO(Mohamed) | Mohamed, Arka |
