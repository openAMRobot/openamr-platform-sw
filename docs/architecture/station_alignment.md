# Station-relative final alignment

Status: Proposed (17 Nov 2026). Implemented by the package `openamrobot_station_alignment` on top of the station
approach of platform-sw #164 (`navigate_to_station.xml`, `station_goal_checker`, `StationApproach`). Topic, action,
parameter and launch argument names in this document are new and stay Proposed until the software lead accepts them.
For Baseline 1 the station approach of #164 alone stays the mechanism; this layer is the long-term direction and is not
part of the Baseline 1 mission.

Simulation evidence only. Gazebo, the test-only camera and the trial numbers below are not robot, site, physical or
safety acceptance. Nothing here is a safety function.

## 1. Problem

The arm reaches the Build 1 work-shelf item for every base pose error up to 0.04 m and 0.08 rad (openamr-upperbody-sw
`docs/integration/station_approach.md`, IK sweep). #164 stops the base within 0.025 m and 0.05 rad of the goal as
AMCL sees it, which leaves 0.015 m to localization. At the station AMCL is 0.009 to 0.058 m off ground truth in
Baseline 1, with 279 of 360 beams returning nothing: the LiDAR plane (0.222 m) sees only 40 mm shelf posts and a few
obstacles. Better tolerances in Nav2 cannot fix that: the goal itself is wrong by the AMCL error. The station has to be
measured relative to the shelf.

## 2. Options

| Criterion | A. AprilTag plate on the shelf (chosen) | B. Shelf geometry from LiDAR or depth | C. opennav_docking |
|---|---|---|---|
| What is measured | full planar pose of the plate (two tags, 0.20 m baseline) in base_footprint | B1 LiDAR: front posts (2 to 3 hits each at 0.7 m with 1 deg beams); B2 depth: shelf face or board edge plane (distance and yaw; position along the shelf needs a feature) | whatever its dock plugin is fed; it would consume the same detection as A |
| Accuracy against 0.04 m / 0.08 rad | simulation, section 6 | not measured; B1 depends on the post layout and the RPLIDAR S3 range accuracy, which is not in the datasheet index; B2 depends on the Gemini 336L depth noise at 0.3 m, not measured | that of its detection, with a continuous closed loop on it |
| Robustness | explicit identity (ids 3 and 4, dock keeps 0 to 2); fails safe (no marker, abort); occlusion of one tag stops the estimate; lighting and print quality matter | no installation; but shelf types differ, products, people and other posts look alike; along-shelf position weak | as A; adds docking, undocking and staging states |
| Hardware needed | printed plate per station; calibrated base camera intrinsics and extrinsics | none for B1; B2 the depth stream | as A |
| Licence | apriltag_ros MIT, apriltag BSD-2-Clause, tag images BSD-2-Clause; all already used by `openamrobot_docking` | own code | Apache-2.0 (Nav2) |
| Command authority | none: goal refinement for Nav2's station tree | none if built the same way as A | docking_server runs its own controller and publishes velocity: a second command source beside controller_server, outside the station tree and its goal checker |
| Effort | small: reuses the docking package's detector, camera_info_sync and texture convention; this package | medium to large: segmentation, a shelf model per site, rejection of look-alikes | medium: plugin, staging pose, a non-charging dock type, plus a decision on command authority |

Verdict: A, as a goal refinement, not a controller. B stays a second source behind the same station pose topic for
sites where markers are not allowed (B2 first, because the face plane gives the binding axes: distance to the shelf
and yaw; the reach sweep tolerates -0.20 to +0.15 m along the shelf). C is rejected for now: it would put a second
velocity authority beside controller_server, which Alex excluded. If the owner later accepts docking_server as a
command source, the detector of A feeds it unchanged.

Existing work reused: `openamrobot_docking` (apriltag_ros configuration pattern, `camera_info_sync.py`, the 200 px
nearest-neighbour tag textures and the +x panel face convention, and the wide-baseline idea of its 3-tag bundle).
Rejected from it: `dock_trigger.py`'s sequencer, which publishes `/cmd_vel` itself (a second command authority).
`nav2_params.yaml` already holds `docking_server` parameters for a future opennav_docking integration; nothing here
launches it.

## 3. Design

```
 base camera ──image──> apriltag_ros ──detections + TF (camera -> station_marker_3/4)──┐
                                                                                     v
                                               station_marker_detector: station pose in base_footprint
                                                 /station_alignment/station_pose (PoseWithCovarianceStamped)
                                                                                     │
 caller ──NavigateToPose──> station_alignment (action server)                         │
                               1. NavigateToPose(nominal station, navigate_to_station.xml) ──> Nav2
                               2. robot stopped: mean of 8 station poses <───────────┘
                               3. residual within 0.03 m to the shelf, 0.02 m along it, 0.06 rad: succeed
                               4. else goal = (map <- base now) o (measured station): NavigateToPose ──> Nav2
                                  lateral <= 0.02 m: distance and yaw only, in place
                                  lateral  > 0.02 m: first a goal 0.30 m behind it, then the goal
                                  (up to 3 corrections; residual above 0.15 m / 0.30 rad aborts)

 Nav2 (unchanged): bt_navigator -> controller_server (StationApproach, station_goal_checker) -> cmd_vel_nav
                   -> velocity_smoother -> collision_monitor -> /cmd_vel
```

Why the corrected goal removes the AMCL error: AMCL's error is a map-frame offset E (a wrong map -> odom), so the map
pose it reports is E o truth. The camera measures the station relative to the robot, S_b = truth^-1 o station, with
no AMCL in it. The corrected goal is (E o truth) o S_b = E o station. Nav2 drives until AMCL reports that pose,
E o truth_final = E o station, so truth_final = station while E stays constant over the short correction
(`test_geometry.py::test_corrected_goal_is_the_true_station_when_the_measurement_is_exact`). A change of E during the
move is measured again in the next iteration.

Limits, from the first simulation runs:

- A residual below the station goal checker tolerance (0.025 m, 0.05 rad) cannot be driven, because the goal checker
  accepts it at once. Acceptance is therefore per axis and not below that tolerance: 0.03 m towards the shelf, 0.02 m
  along it, 0.06 rad. The worst accepted pose, hypot(0.03, 0.02) = 0.036 m and 0.06 rad, is inside the reach budget.
- A differential base cannot step sideways. A corrected goal 2 to 5 cm to the side made the StationApproach controller
  turn towards the path, drive a few centimetres and turn back, which moved less than the progress checker's 0.05 m
  in 12 s, so Nav2 aborted it (run 44139793973, three of five trials). A lateral residual up to 0.02 m is therefore
  left (the reach sweep tolerates -0.20 to +0.15 m along the shelf) and only distance and yaw are corrected in place;
  a larger one goes through a goal 0.30 m behind the corrected station, from which the controller approaches it on a
  curve. A correction that Nav2 fails is measured again and retried by backing off.
- AMCL keeps updating during a correction (every 0.05 m or 0.05 rad since #164), and an update moves the map pose of
  the goal. The next measurement sees it; a station tree in the odom frame, or no filter update during the final
  move, would remove it (owner decision, section 8).

Sample window, on simulation time only (`sampling.py`, 18 Nov 2026). Integrated in Baseline 1 at a real-time
factor of about 0.2 (openamr-upperbody-sw runs 44155397328, 44155429472, 44155442866), the alignment aborted in 3 of 6
runs with no sample in its 6 s window although the detector published at 5 Hz during it. The alignment's clock was
right (30.34 s at the stop, as the runner's real-time factor monitor), so the station poses arrived with image stamps
more than 6 s of simulation time old, and the fixed window that started when the robot had settled never saw a fresh
one. The window now counts only poses whose image was taken after the robot settled, as before, but waits for them:
done after 8 such poses; "marker not seen" when nothing at all arrives in 6 s; "no fresh sample" when poses arrive but
all are older, after 6 s plus a 15 s latency allowance. Every deadline (window, Nav2 goals) is on the node clock; no
wall clock is used. The report records the received count and the latency (arrival minus stamp) of the poses.

Result when the marker gives no usable pose after the #164 approach (Proposed): the base is where #164 stopped it,
which is the Baseline 1 behaviour without this layer, so the action SUCCEEDS with `error_code` 9900 (NOT_ALIGNED,
outside Nav2's codes) and the report outcome `not aligned: ...`; a caller that needs the alignment checks the code.
`on_marker_not_seen: abort` restores an abort. After a correction has moved the robot, a missing marker still aborts.
Accuracy limits are unchanged.

Station definition: the station is defined relative to the plate (`standoff` 0.713 m in front of the plate face,
`lateral` 0, facing the shelf). The plate becomes the reference of the station; item positions on the shelf must be
known relative to it.

## 4. Interfaces (all Proposed)

| Name | Type | Producer | Consumer |
|---|---|---|---|
| `/station_alignment/station_pose` | geometry_msgs/PoseWithCovarianceStamped, frame base_footprint, image stamp | station_marker_detector | station_alignment |
| `/station_alignment/navigate_to_station` | action nav2_msgs/action/NavigateToPose (goal: nominal station in map; behavior_tree ignored) | station_alignment | mission layer |
| `/station_alignment/report` | std_msgs/String, one JSON object per goal (iterations, residuals, sample window, outcome `aligned`, `not aligned: ...` or `aborted: ...`) | station_alignment | diagnostics, trial runner |
| `NavigateToPose.Result.error_code` 9900 NOT_ALIGNED | result code with status SUCCEEDED: #164 reached the station, no marker correction | station_alignment | mission layer |
| `station_marker_3`, `station_marker_4` | TF frames of apriltag_ros | apriltag_ros | station_marker_detector |
| `station_alignment.launch.py` arguments `use_sim_time`, `image_topic`, `camera_info_topic`, `tags_file`, `params_file` | launch arguments | | bringup |
| `config/station_alignment.yaml` | parameters (tag ids, offsets, standoff, accept and correction limits) | | both nodes |
| AprilTag 36h11 ids 3 and 4 | marker ids of a station; ids 0 to 2 stay the charging dock's | | site |

No message type is added; nothing changes in openamrobot-interfaces. Moving `station_pose` into a shared message (for
example with a station id and a quality field) would be a contract change request there.

## 5. Command authority

- Neither node publishes a velocity, runs a controller or starts a navigator. `station_alignment` is an action client of
  Nav2's `navigate_to_pose` with `navigate_to_station.xml`; motion is the `StationApproach` controller and
  `station_goal_checker` of #164, through the velocity smoother and the collision monitor, as for every goal.
- `test/test_command_authority.py` checks the sources (no Twist type, no cmd_vel string, publishers only the station
  pose and the report, the alignment node's only action types NavigateToPose, the robot launch starts only apriltag_ros
  and the two nodes, the simulation launch includes navigation_launch.py once).
- The trial runner checks the live graph: `/cmd_vel` is published by collision_monitor only, `/cmd_vel_nav` by
  controller_server and behavior_server only.

## 6. Simulation test and results

World: `worlds/station_alignment.sdf`, generated by `scene.py` from the Baseline 1 work-shelf layout (openamr-upperbody-sw
scene.yaml at d1c88be: work shelf, six aisle shelves, three obstacles, station (0, -2.085, -pi/2)) plus the marker plate
(two 0.125 m panels, black square 0.10 m, centres 0.10 m either side of the shelf centre, 0.32 m high, above the LiDAR
plane). No tote, item or dock. Its own map. The Baseline 1 world is not changed. Robot: `robo_urdf.urdf.xacro`
unchanged in a test wrapper that adds Gazebo ground truth and a test-only base camera (section 7). Nav2:
`openamrobot_nav2/config/nav2_params.yaml` of #164's head, unchanged.

Trial: drive to one of five start poses 1.3 to 2.2 m away with the default tree, send the station goal with
`navigate_to_station.xml` (#164 alone) and measure against ground truth, then send the same goal to
`/station_alignment/navigate_to_station` (#164 plus marker) and measure again. Pass: every trial completed, every
aligned pose within 0.04 m and 0.08 rad, the command publishers as in section 5.

Run 44143258467 (head 46f00fd, merge 3bab4bd), hosted ubuntu-24.04 runner, gz-sim 8.15.0, Nav2 Jazzy, CycloneDDS,
5 trials. Errors are Gazebo ground truth of base_footprint against the station (0, -2.085, -pi/2).

| Trial | #164 alone (m) | (rad) | AMCL - truth (m) | marker - truth (m) | (rad) | #164 + marker (m) | (rad) | corrections |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.0221 | 0.0089 | 0.0162 | 0.0042 | 0.0004 | 0.0341 | 0.0245 | 3 (backoff) |
| 2 | 0.0179 | 0.0661 | 0.0292 | 0.0049 | 0.0023 | 0.0171 | 0.0025 | 1 (in place) |
| 3 | 0.0295 | 0.0018 | 0.0266 | 0.0048 | 0.0003 | 0.0186 | 0.0086 | 1 (in place) |
| 4 | 0.0223 | 0.0092 | 0.0067 | 0.0044 | 0.0009 | 0.0223 | 0.0092 | 0 (accepted) |
| 5 | 0.0630 | 0.0087 | 0.0366 | 0.0052 | 0.0004 | 0.0176 | 0.0131 | 1 (in place) |

| | position mean / max (m) | yaw mean / max (rad) | beyond 0.04 m / 0.08 rad |
|---|---|---|---|
| #164 alone | 0.031 / 0.063 | 0.019 / 0.066 | 1 of 5 |
| #164 + marker | 0.022 / 0.034 | 0.012 / 0.025 | 0 of 5 |
| marker estimate - truth | 0.0047 / 0.0052 | 0.0009 / 0.0023 | |

`/cmd_vel` was published by collision_monitor only and `/cmd_vel_nav` by controller_server and behavior_server only.
The workflow's revert run (the package's code, configuration and assets removed) fails 6 tests and 3 test modules,
and passes only the check of #164's own tree.

Reading:

- #164 alone left the base 0.063 m from the station in trial 5, beyond the 0.04 m budget, with AMCL 0.037 m off. In
  the previous run on the same #164 configuration (44139793973) #164 alone ended at 0.043, 0.026, 0.029, 0.048 and
  0.037 m: 3 of 10 trials over both runs are beyond the budget. With the marker every trial ended inside it.
- The marker estimate is 4 to 5 mm from ground truth in every trial with a constant sign: a bias, not noise (spread
  of 8 averaged frames below 0.03 mm). Its source (tag size rounding in the render, camera model, PnP) is not found.
- Corrections in place converge in one step. The backoff route (trial 1, lateral 0.021 to 0.035 m) did not converge:
  the residual after the out-and-back move grew to 0.095 m before the last correction, because AMCL updates during
  the 0.6 m of motion. It still ended inside the budget (0.034 m), with little margin.
- N = 5 per arm in one world: enough to show the mechanism and the failure of #164 alone, not a distribution.

Earlier runs of this branch, kept as evidence of what failed: 44131991577 and 44135340319 ("marker not seen": the
detector blocked on TF lookups), 44137562343 (counters: both tags detected in most frames, TF at the image stamp
missing when the detection arrived), 44139793973 (marker within 5 mm; Nav2 aborted three lateral corrections on
the progress checker).

## 7. Hardware and calibration needed

- Plate: AprilTag 36h11 ids 3 and 4 (new ids per station when there are several), printed matt, flat and rigid, black
  square 0.10 m, centres 0.20 m apart, at 0.32 m above the floor on the shelf front below board 0 and above the LiDAR
  plane, plate face position known relative to the station and the items. Sizes are simulation working values; the
  real size must be measured and set in the tags file.
- Base camera intrinsics: the Gemini 336L colour stream's factory calibration, checked with a target
  (`openamrobot_docking/docs/06_camera_calibration.md`).
- Base camera extrinsics: base_footprint -> base_camera_color_optical_frame. Yaw of the mount matters most: 1 deg of
  camera yaw is 0.017 rad of station yaw and about 0.012 m of lateral error over the 0.713 m standoff. Target: yaw
  within 0.3 deg, pitch within 0.5 deg, position within 2 mm, by a calibration against a surveyed target.
- Dependency, not fixed here: the description has no Gazebo sensor on base_camera_link; its only camera sensor is the
  legacy camera_link one, 0.154 m higher and level (audits #89, upperbody-sw #24, platform-sw #53). The test wrapper
  carries its own sensor at the base_camera_link pose and the workflow checks the converted pose. When the
  description has a verified base camera sensor, the wrapper's sensor is removed.
- Head and wrist cameras are not used: the base camera sees the plate from the station without moving the lift or arms.

## 8. Decisions needed

| Decision | Owner |
|---|---|
| Accept or change the interfaces of section 4 (names, the NavigateToPose action type, frame base_footprint) | software lead |
| Marker plate per station: ids, size, height, placement on site shelves; whether markers are allowed at Use_Case_1 sites | software lead with the site owner |
| A tighter station goal checker for corrected goals (for example 0.01 m / 0.02 rad as a third goal checker selected by the tree), so the alignment can drive below 0.025 m; a #164 change | navigation owner |
| AMCL updates during the final correction move: keep, or run the final move in the odom frame or without filter updates | navigation owner |
| A progress checker for short station corrections (0.05 m in 12 s rejects a 2 cm move with turns); the backoff goal is the workaround here | navigation owner |
| Whether opennav_docking's docking_server may ever be a command source (option C) | software lead |
| Shelf-geometry source (option B2, depth plane fit) as a fallback behind the same topic | perception owner |
| Base camera extrinsic calibration procedure and the description's base camera sensor (dependency of section 7) | perception owner |
| NOTICE.md entry for the two tag textures (models/station_marker/PROVENANCE.md) | platform lead |

Owners: software lead (interfaces, command authority), navigation owner (#164 interaction, goal checker, AMCL),
perception owner (marker detection, camera calibration, option B). Names are in the pull request.

## 9. Not verified

- No real camera, plate, shelf or robot. Rendering, lighting, print quality, glare, motion blur and occlusion of the
  real site are not modelled.
- The test-only camera parameters (640 x 400 at 5 Hz, 94 deg) are working values, not the Gemini 336L's verified
  figures; camera noise is Gaussian pixel noise only.
- Extrinsic and intrinsic calibration errors are not injected: the simulated camera pose is exact.
- Only the Build 1 work-shelf station and one plate layout; other stations, lift heights and approach directions not
  tried.
- Option B and C are not prototyped or measured.
- The 4 to 5 mm marker bias is not explained.
- The alignment is not part of the Baseline 1 mission and was not run in the Baseline 1 world or with the arms,
  lift and firmware SIL.
