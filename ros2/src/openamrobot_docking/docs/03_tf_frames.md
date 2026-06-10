# TF Frames and Static Transforms

The docking pipeline requires a complete TF chain so the AprilTag pose
can be expressed in `map`. This document describes the chain, where
each link comes from, and how to verify it.

## Required TF chain

The dock carries a **3-tag bundle** (family 36h11, IDs `0`, `1`, `2`). `apriltag_ros` publishes one dynamic TF per detected tag — the same chain prefix, three different leaves:

```
map → odom → base_link → camera_link → camera_optical_frame ─┬→ charging_dock_tag_0
                                                              ├→ charging_dock_tag_1   (centre, docking target)
                                                              └→ charging_dock_tag_2
```

| Link | Source |
|---|---|
| `map → odom` | Localization (slam_toolbox in sim, AMCL or another localizer in real) |
| `odom → base_link` | Wheel odometry (DiffDrive plugin in sim, robot driver in real) |
| `base_link → camera_link` | Static, from URDF (`camera_joint`, fixed) — translation (0.35, 0, 0.22), no rotation |
| `camera_link → camera_optical_frame` | Static, from URDF (`camera_optical_joint`, fixed) — `rpy = (−π/2, 0, −π/2)` |
| `camera_optical_frame → charging_dock_tag_{0,1,2}` | Dynamic, from `apriltag_ros` — one TF per detected bundle tag |

Once the full chain exists:
- `detected_dock_pose_publisher` looks up `map → charging_dock_tag_1` (the **centre** tag, the docking target) and publishes it as a `PoseStamped` on `/detected_dock_pose`.
- `dock_trigger.py` consumes the **outer tags** (`_tag_0` and `_tag_2`) directly via TF to estimate the dock surface normal from their 0.90 m baseline. See [`13_perception_and_line.md`](13_perception_and_line.md).

## Optical vs body frames

`camera_link` is a **body** frame: +X forward, +Y left, +Z up
(REP-103). `camera_optical_frame` is the **optical** convention:
+X right, +Y down, +Z forward — the convention OpenCV's `solvePnP`
uses internally.

The transform between them is:

```xml
<joint name="camera_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
</joint>
```

This rotation is REP-103 standard. Don't omit it: apriltag_ros
publishes the tag pose in `camera_optical_frame`. If you point
apriltag at a body-convention frame, the resulting pose is rotated by
90° in unexpected ways.

The simulation's `dock_trigger.py` also looks up TF
`camera_optical_frame → charging_dock_tag_{0,1,2}` directly during
the centring scan, the dock-normal estimation, and the final visual
servo, so the rotation must be correct or those steps will mis-aim.

## Where the static transforms come from

### Simulation

Provided by the `openamrobot_description` package, loaded by
`robot_state_publisher` via `xacro` from
`openamrobot_description/urdf/openamrobot.urdf.xacro`. The URDF tree is:

```
base_link                  ← URDF root, ground projection (z = 0)
└── base_link                   ← base_joint lifts by 0.053 m so wheels touch ground
    ├── lidar_link              ← gpu_lidar sensor host
    ├── camera_link             ← (0.35, 0, 0.22), forward-mounted camera
    │   └── camera_optical_frame  ← rpy (−π/2, 0, −π/2) for solvePnP
    ├── left_wheel_link
    ├── right_wheel_link
    └── (4 caster + 4 caster-wheel links)
```

`base_link` is the URDF's root, NOT `base_link`. This matters for
the spawn z: `simulation.launch.py` spawns `base_link` at world
`z = 0`, and `base_joint` then puts `base_link` 5.3 cm higher so the
wheel centres are at world z = `wheel_radius` = 0.1 m. Spawning the
URDF root at non-zero z (as an earlier version did) left the drive
wheels floating above the ground — see Lesson 25 in
`12_lessons_learned.md`.

Gazebo's plugins publish dynamic transforms (`odom`, `joint_states`).
`robot_state_publisher` constructs the static portion from the URDF.

### Real robot

The real robot's URDF must publish the same chain. If your camera
driver only publishes `camera_link`, define
`camera_optical_frame` as a static transform either in the URDF
or via `static_transform_publisher`:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5708 0 -1.5708 \
  camera_link camera_optical_frame
```

(rpy in radians; xyz first, rpy second; parent then child.)

## Verifying the chain

```bash
# Whole tree:
ros2 run tf2_tools view_frames
# Then open frames.pdf

# Specific transforms:
ros2 run tf2_ros tf2_echo map base_link                              # localization is working
ros2 run tf2_ros tf2_echo base_link camera_link                      # camera mounted correctly
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame           # optical rotation
ros2 run tf2_ros tf2_echo camera_optical_frame charging_dock_tag_1   # centre tag visible

# End-to-end (the centre tag is the only one detected_dock_pose_publisher needs):
ros2 run tf2_ros tf2_echo map charging_dock_tag_1     # centre tag = docking target
ros2 run tf2_ros tf2_echo map charging_dock_tag_0     # outer-left  (normal estimator)
ros2 run tf2_ros tf2_echo map charging_dock_tag_2     # outer-right (normal estimator)
```

If `map → charging_dock_tag_1` returns a valid pose, the pipeline is
good. The outer-tag transforms must also be valid for the dock-normal
estimation in Phase 2/3 of the bundle sequencer.

## Common mistakes

- **Forgetting `camera_optical_frame`.** Tag pose is then in
  `camera_link` (body), apriltag_ros computes solvePnP assuming
  optical → result is rotated.
- **Spawning the URDF root with a non-zero z**. `base_link` must
  be at z = 0; the lift to `base_link` is handled by `base_joint`.
- **Spawning the robot at non-zero yaw in simulation**.
  `slam_toolbox` initialises the map at the robot's spawn pose. If
  you spawn at world `yaw = π/2`, the `map_x` axis aligns with
  `world_y` and dock coordinates rotate accordingly. The
  `simulation.launch.py` `spawn_yaw` argument handles this for the
  built-in dock — but any external map-frame coordinates you hard-code
  will not auto-rotate.
- **Using sim time without `use_sim_time: true`** on every node that
  talks to TF. TF timestamps are then in mismatched epochs and
  `tf2_echo` complains about extrapolation.

## Sim coordinate transformation summary

The map origin coincides with the robot's spawn pose. With the default
spawn `(0, 0, 0)` yaw = 0:

| Pose | World | Map |
|---|---|---|
| Robot spawn (`base_link`) | `(0, 0, 0)` | `(0, 0, 0)` |
| Bundle **centre tag** (id 1) | `(4.899, 0, 0.5)` | `(4.899, 0, 0.5)` |
| Bundle **outer-left tag** (id 0) | `(4.899, -0.45, 0.5)` | `(4.899, -0.45, 0.5)` |
| Bundle **outer-right tag** (id 2) | `(4.899, +0.45, 0.5)` | `(4.899, +0.45, 0.5)` |
| Robot at staging (phase 1 Nav2 goal, `staging_distance = 2.0`) | `(2.899, 0)` | `(2.899, 0)` |
| Robot at end of Phase 5 (camera→tag depth ≈ `docking_distance = 0.15`) | `(4.6, 0)` | `(4.6, 0)` |

(The simulation places AMCL at the robot's spawn pose, so map ≡ world in the default scenario.)

For a non-default spawn `(spawn_x, spawn_y, spawn_yaw)`,
`simulation.launch.py` re-projects the dock automatically into the new
map frame:

```
dock_in_map = R(−spawn_yaw) · (dock_in_world − spawn_in_world)
```

and overrides `dock_pose_{x,y,yaw}` on the `dock_trigger` node so the
sequencer keeps pointing at the same physical tag.
