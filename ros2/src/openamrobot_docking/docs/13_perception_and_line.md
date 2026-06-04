# Perception and the perpendicular line

How the AprilTag detection turns a camera image into a 6-DOF pose, what that
pose is measured *between*, and how the docking sequencer turns it into the
**perpendicular approach line** that Phase 4 tracks. Ends with how to
visualise that line live in RViz.

See also: [`08_sequencer_4phase.md`](08_sequencer_4phase.md) for the phase-by-phase
control, [`03_tf_frames.md`](03_tf_frames.md) for the full TF chain.

---

## 1. What the AprilTag pipeline actually gives us

It is **not** just an angle, and **not** just a distance. `apriltag_ros` gives
back the tag's **full 6-DOF pose** — position *and* orientation.

The steps:

1. The detector finds the **four corners** of each tag in the image, in pixels.
2. Given the camera intrinsics (`/camera_info`) **and** the tag's known
   physical size (the **0.16 m black-square edge** — 8/10 of the 0.20 m panel,
   the value `apriltag_ros` consumes for PnP — set in
   [`config/tags_36h11_sim.yaml`](../config/tags_36h11_sim.yaml)),
   it runs **solvePnP**: it solves for the 3D position + orientation that
   reproject those four corners exactly where they appear in the image.
3. Output: each detected tag's **position `(x, y, z)` in metres** and its
   **orientation (quaternion)**, expressed in `camera_optical_frame`. The 3-tag
   bundle (IDs 0/1/2 mapped to frames `charging_dock_tag_0`, `charging_dock_tag_1`,
   `charging_dock_tag_2`) is published as three TFs:
   `camera_optical_frame → charging_dock_tag_{0,1,2}`. The **centre** tag
   (`charging_dock_tag_1`) is the docking target; the **outer** tags
   (`charging_dock_tag_0` at `y = −0.45 m` and `charging_dock_tag_2` at
   `y = +0.45 m`) provide the wide baseline used to estimate the dock normal.

So the raw datum is a pose. Everything else (angle, distance, the tag normal)
is *derived* from it.

### Derived quantities

| Quantity | How it's computed | Used by |
|---|---|---|
| **Depth / distance** | `z` component (forward) of the tag in the optical frame, or `hypot(x, z)` | range gating |
| **Image-frame angle** | `atan2(x_optical, z_optical)` | the near-field **visual servo** (`lookup_tag_in_camera_optical`) |
| **Tag normal** | rotate the tag's local +Z axis by its quaternion (`quat_rotate_z`) | building the perpendicular line |

---

## 2. Measured *between what and what* — and the lever arm

The optical-frame convention is **+Z forward (into the scene), +X right, +Y down**,
with the origin at the camera's optical centre.

This matters because the camera is **not** at the centre of the robot. In Raj's
URDF ([`robo_urdf.urdf.xacro`](../../openamrobot_description/urdf/robo_urdf.urdf.xacro), `camera_joint`):

```
camera_link  is at  xyz = (0.35, 0, 0.20)  relative to  base_link
```

So the camera sits **0.35 m in front** of the robot origin and 0.20 m up.

Depending on the quantity, the reference point differs:

- **Tag pose (solvePnP)** — measured between the **camera optical centre** (front
  of the robot) and the **centre of the tag**.
- **Image-frame angle** `atan2(x, z)` — the horizontal angle between the
  **camera's forward axis** and the ray to the tag, measured *at the camera*.
- **Distance and lateral offset** of the line tracker — computed from
  **`base_link`** (the robot's centre/origin), via
  `lookup_robot_pose()` → `lookup_transform('map', 'base_link')`.

> ⚠️ **Lever arm.** "Where the camera looks" (front, +0.35 m) and "the point the
> controller regulates" (`base_link`) are 0.35 m apart. When the robot is not
> perfectly aligned, this offset couples heading into apparent lateral error and
> is a real source of final-alignment imprecision — a candidate to correct when
> improving the docking.

---

## 3. From tag pose to the perpendicular line

The **line is not in the image** — it is built in the **`map`** frame from the
averaged tag pose. Pipeline:

1. **Centre tag pose → map.** [`detected_dock_pose_publisher.cpp`](../src/detected_dock_pose_publisher.cpp)
   looks up the chained TF `map → charging_dock_tag_1` (the centre tag of the
   bundle) and republishes it as `/detected_dock_pose` (PoseStamped, 10 Hz).
2. **Estimate the dock normal from the outer tags.** Rather than averaging a
   single tag's noisy yaw, the current sequencer estimates the dock surface
   direction from the line through the **outer** tags
   `charging_dock_tag_0` and `charging_dock_tag_2` (90 cm apart). The
   perpendicular to that direction is the dock normal — a wide-baseline
   estimate that is far more stable than any single-tag PnP. The estimate is
   smoothed with a **proximity-weighted EMA** (closer samples carry more
   weight) so that the better near-field observations dominate the early
   far-field ones.
   *Note:* the older `TagRunningAverage` class (cumulative mean over 40
   single-tag detections) is still in the codebase for the legacy
   single-tag fall-back path, but the production bundle sequencer does not
   use it.
3. **Tag normal.** `quat_rotate_z(...)` takes the averaged orientation and
   returns the tag's local +Z axis in the `map` frame, projected onto the XY
   plane. For a flat panel, that is (approximately) the **normal to the tag
   face**.
4. **The line.** The perpendicular approach line is the straight line through
   the tag centre `(avg.x, avg.y)` along that normal direction. It is the ideal
   head-on approach axis.
5. **Heading target.** `perpendicular_yaw(rx, ry)` returns the yaw the robot
   must hold to be aligned on the line, facing the tag (the sign is
   disambiguated so the normal points from the tag toward the robot).
6. **Error signal.** `signed_lateral_offset(rx, ry, perp_yaw)` is the signed
   perpendicular distance from `base_link` to the line. Positive = robot is to
   the left looking toward the tag. **This is the error Phase 4 drives to zero.**

In one line: **tag → 6-DOF pose → normal + centre → perpendicular line → lateral
offset → steering command.**

### The two control regimes

- **Far field (line tracking, map frame).** Pure-pursuit on the perpendicular
  line: `desired_yaw = perp_yaw − atan2(lateral, line_lookahead_distance)`,
  then `omega = line_yaw_kp × (desired_yaw − robot_yaw)`. The running average
  keeps refining the line as the robot advances.
- **Near field (visual servo, image frame).** Once the line is stabilised, the
  average is frozen and steering switches to a closed loop on the image-frame
  angle: `omega = −visual_servo_kp × atan2(x_optical, z_optical)`. This is
  robust to the near-field solvePnP bias because it tracks where the tag *is* in
  the camera, not where map-frame solvePnP thinks it is.

See [`08_sequencer_4phase.md`](08_sequencer_4phase.md) and
[`05_parameters.md`](05_parameters.md) for the gains and the handover triggers.

---

## 4. Visualising the line in RViz

`dock_trigger.py` publishes the line as RViz markers while it runs, so you can
*see* the perpendicular line and the averaged tag centre, and watch the line
refine during the approach.

- **Topic:** `/docking/debug_markers` (`visualization_msgs/MarkerArray`)
- **Frame:** `map`
- **Contents:**
  - a green **LINE_STRIP** — the perpendicular approach line through the tag;
  - a red **SPHERE** — the running-average tag centre `(avg.x, avg.y)`.
- Toggle with the `publish_debug_markers` parameter (default `true`); the topic
  name is `debug_marker_topic`.

### Show it in RViz

The markers publish automatically — you just need to add the display once:

1. In RViz, click **Add** (bottom-left of the Displays panel).
2. Choose the **By topic** tab.
3. Select **`/docking/debug_markers` → MarkerArray** and click **OK**.
4. Trigger a dock (`ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once`)
   and watch the green line appear and refine as the robot advances.

To make it persistent, save the RViz config (File → Save Config), or add the
MarkerArray display to `openamrobot_nav2/rviz/nav2_view.rviz`.

### Quick check from the CLI

```bash
ros2 topic echo /docking/debug_markers --once   # markers are being published?
ros2 run rqt_image_view rqt_image_view /rgb_image   # what the camera/detector sees
```

---

## 5. Visualising the line in Gazebo

Gazebo does **not** consume ROS `visualization_msgs/Marker` — that is RViz only.
To draw the *same* live line inside the Gazebo GUI, the sequencer pushes a
`gz.msgs.Marker_V` to Gazebo's own marker service via the `gz` CLI. It uses a
thin green **CYLINDER** for the line (gz renders `LINE_STRIP` markers as 1-px
lines that are nearly invisible in the 3D view) plus a small red **SPHERE** at
the tag centre. (There are no Python gz-transport bindings on this
system, so the CLI is the pragmatic path.) **map ≡ world** in this sim (the
robot spawns at the world origin and AMCL is initialised there once converged),
so the map-frame estimate is sent straight to gz as world coordinates.

Implementation notes:

- The call is **throttled** (`gz_marker_period`, default 0.4 s) and run in a
  daemon thread, so spawning a subprocess never stalls the 20 Hz control loop.
- It targets the `gz_marker_service` parameter (default `/marker_array`).

Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `publish_gz_marker` | `true` | Mirror the line into the Gazebo GUI |
| `gz_marker_service` | `/marker_array` | gz transport service for `gz.msgs.Marker_V` |
| `gz_marker_period` | `0.4` | Minimum seconds between `gz` CLI calls |

It appears automatically in the Gazebo window during Phase 4 — no GUI step
needed (unlike RViz). To confirm the service is reachable, or to draw a marker
by hand:

```bash
gz service -l | grep marker          # is /marker_array (or /marker) advertised?
```

> ⚠️ This is a pragmatic hack, not a clean integration: it shells out to `gz`
> at a few Hz and depends on the GUI marker manager. If the service name differs
> on your Gazebo build, set `gz_marker_service` (some builds expose `/marker`
> for a single `gz.msgs.Marker` instead of `/marker_array`). For high-rate or
> production use, a C++ gz-transport node would be the proper route. RViz
> (section 4) remains the primary, robust visualisation.

---

## 6. Camera-centric docking redesign (IMPLEMENTED — experimental, needs tuning)

> Status: **implemented in `dock_trigger.py`**, replacing the old Phase 4
> line-tracking. Build passes; behaviour still needs on-robot/sim tuning. The
> phase-by-phase doc [`08_sequencer_4phase.md`](08_sequencer_4phase.md) describes
> the *previous* pipeline and is now partly out of date for the approach stage.

### Motivation

Two problems with the current approach:

1. **Map dependence.** Phase 4 line-tracking steers on the robot's *map* pose
   (`lookup_transform('map', 'base_link')`). If the wheels slip or odometry
   drifts, the map pose is wrong and the robot docks badly. The dock should
   close the loop on **direct tag perception** (camera → tag), using the map as
   little as possible, so wheel slip cannot corrupt the final approach.
2. **Inefficient line re-joining.** The current "find the line then steer back
   onto it" behaviour is not effective. Replace it with **direct repositioning
   onto points of the tag normal**.

Also, a single planar tag gives a **noisy normal** (orientation), worst when
viewed off-axis — see §1–§3. The redesign attacks that with a **3-tag dock**:
the orientation comes from the geometry between tags, not from a single tag's
solvePnP normal.

### The 3-tag dock

The dock model carries **three coplanar tags in a row, same height**
(`models/apriltag_dock/model.sdf`):

- **id1 (centre, y=0)** — the **dock target**: the point the robot drives onto,
  and the tag the final corrector tracks.
- **id0 / id2 (outer, y=±0.45 m)** — a **wide baseline** (0.90 m). The line
  joining their observed *centres* gives the dock's in-plane orientation
  precisely: the wide baseline turns a small position error into a tiny angular
  one. The **normal** is perpendicular to that baseline. No reliance on any
  single-tag solvePnP normal.
- All three tags are **0.20 m panels** (0.16 m black-square edge → global
  `size: 0.16` in `tags_36h11_sim.yaml`).

apriltag_ros publishes one TF per tag (`charging_dock_tag_0/1/2`); `dock_trigger`
reads all three and computes (centre = id1, normal ⟂ id0→id2).

### Pipeline

**Phase 1 — coarse approach (Nav2, map).** Bring the robot within view of the
tags. (Unchanged; map is fine for getting close.)

**Phase 1.5 — back off if too close.** After acquiring the tags, if the robot is
closer than `too_close_distance`, first reposition to `predock_distance` on the
(coarse) normal before doing anything else — observations from too close are
unreliable (tags fill / leave the FOV).

**Phase 2 — estimate the dock.** Centre the **middle tag** in the image, average
the three tags' map positions, then derive **centre = id1** and **normal ⟂
(id0→id2)** (disambiguated toward the robot). No manoeuvre needed — the three
tags give the normal directly.

**Phase 3 — go to the pre-dock point.** P₁ = `centre + predock_distance × N` (on
the normal, in front of the centre tag), oriented facing the dock.

**Phase 4 — re-estimate & verify (instead of line re-joining).**
- From P₁ (now roughly head-on, where the normal is best observed), recompute
  the normal / line → **N′**.
- Compare N′ to N:
  - **agree** (within tolerance) → the line is confirmed, continue.
  - **differ** → go to a closer point P₂ = `tag + 1.30 m × N′` on the refined
    normal. This *repositions directly onto the corrected axis* rather than
    running the old (ineffective) line re-join.

**Phase 5 — final approach: pure-pursuit on the perpendicular axis.**
Crucially **not** "centre the tag in the image" — that only points the robot
*at* the tag, so from a slightly off-axis start it arrives at an angle (not
perpendicular). Instead the robot does **pure-pursuit on the dock normal**:
- Every iteration the axis (dock centre + normal) is re-derived from the live
  3-tag perception and folded into a **running average**, so the axis **adapts**
  and stays smooth as the robot advances.
- Pure-pursuit steers the robot back **onto the axis while advancing**
  (`desired_yaw = normal − atan2(lateral, lookahead)`), so as the lateral offset
  → 0 the heading → the normal: it **arrives perpendicular**.
- The live axis estimate is filtered with a **proximity-weighted EMA**: the EMA
  weight grows as the robot gets closer (`∝ predock_distance / depth`, capped),
  so the samples taken **while advancing** (nearer = tags bigger in the image =
  more accurate) dominate the early far ones, instead of a flat mean that would
  let the early off-axis samples drag the estimate.
- This runs only in the **FAR** regime (camera depth > `freeze_axis_distance`,
  default 0.70 m), where the 3-tag estimate is clean.
- **NEAR** (≤ `freeze_axis_distance`): the averaged axis is **frozen** (close-up
  estimates are noisy and the outer tags start leaving the FOV — averaging them
  in caused an end-of-approach zig-zag) and the robot finishes on the
  **centre-tag visual corrector** (keep the centre tag centred). From an
  already-aligned pose this only trims the residual, so no zig-zag.
- Stops when the camera→centre-tag depth ≤ `docking_distance` (e.g. 0.15 m).

### Frames — what runs in map vs camera, and why the Gazebo line looks off

The approach control (Phase 5 FAR) runs in the **map** frame: the robot pose
(`map → base_link`), the three tag positions (`map → tag_i`) and the derived
axis are all in map, so they are **mutually consistent**. The tag positions are
re-read from live camera perception every iteration and folded into the EMA, so
the axis tracks the **real dock** (not a stale map guess), and pure-pursuit
steers the map-frame robot onto that axis. Over the short approach distance,
odometry drift is negligible, so this is robust in practice. The NEAR corrector
(Phase 5 close-up) uses the **camera-frame** angle to the centre tag directly.

**Why the green line can look misaligned in Gazebo.** The debug markers are
computed in `map`, but the Gazebo marker GUI draws them in the **world** frame.
If AMCL has shifted/rotated `map` relative to the world, the line shows up
tilted/offset **in Gazebo** even though, in `map` (where all the control lives),
it is perpendicular to the dock. That is why docking can be correct while the
Gazebo line looks crooked — it is an *display* artefact, not a control error.
**RViz with Fixed Frame = `map` shows the true (aligned) line.**

### Proposed parameters (to tune)

| Quantity | Default |
|---|---|
| Tag edge size (all three) | 0.16 m (0.20 m panel, global `size`) |
| Outer-tag baseline (id0↔id2) | 0.90 m (model.sdf, y=±0.45) |
| Pre-dock point P₁ | 2.0 m on the normal (`predock_distance`) — far enough to see all 3 tags |
| Refined pre-dock point P₂ (if N′ disagrees) | 1.50 m (`refined_predock_distance`) |
| Normal-agreement tolerance | 5° (`normal_tolerance_deg`) |
| Too-close threshold (Phase 1.5) | 1.0 m (`too_close_distance`) |
| Axis freeze depth (FAR → corrector) | 0.70 m (`freeze_axis_distance`) |
| Axis EMA weight (proximity-scaled) | 0.40 (`axis_filter_alpha`) |
| Final dock depth (camera→tag) | 0.15 m (`docking_distance`) |

> Note: `obs_lateral` / `obs_distance` (two-sided approach) and
> `visual_servo_min_depth` (old blind-advance handover) are now unused.

### Open questions / tuning

- `normal_tolerance_deg` (re-estimate agreement) and whether to iterate Phase 4.
- Baseline width vs FOV: wider baseline = better angle but the outer tags leave
  the FOV sooner on approach (mitigated by tracking the centre tag in Phase 5).
- Tag size 0.20 m vs detection range at the staging distance.

---

## 7. Command cheat-sheet

All commands assume the workspace is `~/Downloads/raj-integration/ros2`.

### Build

```bash
cd ~/Downloads/raj-integration/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                              # whole workspace
colcon build --symlink-install --packages-select openamrobot_docking   # docking only
```

> A YAML/param change (e.g. `dock_trigger.yaml`) needs no rebuild with
> `--symlink-install` — just relaunch the docking layer. A **model change**
> (`model.sdf`, tag textures) needs Gazebo relaunched (whole bringup). A node
> already running does **not** hot-reload — relaunch it.

### Source (in every new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Downloads/raj-integration/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Launch

```bash
# One command (Gazebo + Nav2 + docking, staggered)
ros2 launch openamrobot_docking bringup_sim.launch.py

# …or the three layers separately, in order:
ros2 launch openamrobot_gazebo gz_simulator.launch.py     # 1. sim
ros2 launch openamrobot_nav2   sim_bringup_launch.py      # 2. Nav2 + AMCL + RViz
ros2 launch openamrobot_docking openamrobot_docking.launch.py  # 3. docking
```

### Drive the robot

```bash
ros2 topic pub /dock_trigger std_msgs/msg/Bool "{data: true}" --once    # dock
ros2 topic pub /undock_robot std_msgs/msg/Bool "{data: true}" --once    # undock (reverse 1.5 m + 180°)

# Navigation goal (RViz "2D Goal Pose", or by hand). If docked, it undocks first:
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

### Visualise the line / debug

```bash
# RViz: Add → By topic → /docking/debug_markers → MarkerArray, Fixed Frame = map
ros2 topic hz   /docking/debug_markers          # markers publishing? (Phase 5)
ros2 topic echo /docking/debug_markers --once

# Gazebo marker service present?
gz service -l | grep marker

# AprilTag detections (should list id 0, 1, 2)
ros2 topic echo /apriltag/detections
ros2 run rqt_image_view rqt_image_view /rgb_image
```

### Inspect estimates (frames)

```bash
# Camera→centre-tag depth (should match the real distance after size fix)
ros2 run tf2_ros tf2_echo camera_optical_frame charging_dock_tag_1

# Tag positions in map (centre should be ~the physical dock)
ros2 run tf2_ros tf2_echo map charging_dock_tag_0
ros2 run tf2_ros tf2_echo map charging_dock_tag_1
ros2 run tf2_ros tf2_echo map charging_dock_tag_2

# AMCL map↔odom offset (≈0 ⇒ map≡world; large ⇒ Gazebo markers look skewed)
ros2 run tf2_ros tf2_echo map odom

# Confirm the running node has the latest params
ros2 param get /dock_trigger axis_filter_alpha
```

---

## 8. Camera & AprilTag — how the image becomes a pose

### Do we use OpenCV?
- **Not in our code.** `dock_trigger.py` and `detected_dock_pose_publisher.cpp`
  never touch the image — they only handle poses/TF.
- OpenCV is used **inside `apriltag_ros`**: `cv_bridge` (OpenCV) converts the ROS
  `sensor_msgs/Image` to a matrix and to grayscale for the detector, and
  `cv::solvePnP` estimates the tag pose (enabled by
  `pose_estimation_method: "pnp"` in `tags_36h11_sim.yaml`).
- **The camera itself is not OpenCV** — the image is *rendered* by Gazebo's 3D
  engine.

### How the camera is handled (the flow)
1. **Gazebo** simulates a camera sensor declared in the robot URDF
   (`openamrobot_description`). Each sim step it **renders an RGB image** and
   publishes a `camera_info` with the **intrinsics**: focal lengths `fx, fy`,
   optical centre `cx, cy`, distortion.
2. **`ros_gz_bridge`** turns the Gazebo topics into ROS topics. Note: the
   upstream bridge in `openamrobot_gazebo` publishes `/camera/camera_info`, but
   `apriltag_ros`'s `image_transport::CameraSubscriber` derives the info topic
   from the **image** topic name — with the image remapped to `/rgb_image` at
   the root it looks for `/camera_info` at the root. So this package's launch
   adds a small `camera_info_bridge` that bridges the gz `/camera_info` to the
   root ROS `/camera_info`.
3. **`apriltag_ros`** subscribes to `/rgb_image` + `/camera_info`.

On a real robot only step 1 changes (a real camera driver replaces Gazebo); the
rest of the chain is identical (use `apriltag.launch.yml` + rectified images).

### How AprilTag works
1. **Grayscale** the image (via cv_bridge).
2. **Quad detection**: adaptive black/white thresholding → edge segments →
   assemble candidate **quads** (the square tag borders).
3. **Decode**: read the bit grid inside each quad (6×6 for family **36h11**),
   match it against the family's codes, and recover the **ID** + orientation
   (error-corrected up to `max_hamming`). This is how id0 / id1 / id2 are told
   apart.
4. **Pose (solvePnP)**: given the tag's physical `size` (0.16 m) and the camera
   intrinsics, solve the 6-DOF pose so the four corners reproject where they
   appear in the image. Published as TF `camera_optical_frame →
   charging_dock_tag_i`.

That TF (one per tag) is the only thing the docking node consumes — which is why
`size` is critical: solvePnP uses it for the depth scale (a wrong `size` →
wrong distance → the "drives into the wall" bug, fixed by 0.20 → 0.16).
