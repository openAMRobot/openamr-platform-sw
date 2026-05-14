# Lessons Learned

This document collects the technical lessons from building and debugging the OpenAMRobot autonomous docking pipeline. Each lesson follows the same structure: context, symptom, root cause, fix, takeaway.

If you're hitting a problem **right now**, go to [`09_troubleshooting.md`](09_troubleshooting.md) for a quick reference. This document is for understanding *why* the fixes work and what you can learn from them.

---

## Lesson 1: The default RMW (FastDDS) breaks Python action clients on Jazzy

**Context:** We were bringing up the docking pipeline and wanted the simplest possible smoke test — publish `true` on `/dock_trigger` and watch `dock_trigger.py` send a `NavigateToPose` goal to Nav2.

**Symptom:** The Python node printed its startup banner, subscribed to the topic, received the `Bool`, started building the action goal, then exited silently. No traceback, no rclpy error, nothing in `journalctl`. Action servers on the C++ side (Nav2's `bt_navigator`) showed no incoming goal request.

**Why it happened:** ROS 2 Jazzy ships with FastDDS as the default `rmw_implementation`. There is a known crash in the FastDDS Python bindings that triggers when an `ActionClient` sends a goal whose message contains certain nested types — the process dies inside the C++ middleware before any Python-visible error is raised. The bug does not affect topic publishers/subscribers, which is why most of the stack appeared healthy.

**How we fixed it:** Switch the entire stack to CycloneDDS. Install once:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

Then in every terminal that runs ROS commands (or in `~/.bashrc`):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

This must be set **before** any ROS node starts in that terminal, including `ros2 launch`. Mixing CycloneDDS and FastDDS nodes in the same network usually works for plain topics but is not worth the risk.

**Lesson:** When a process dies silently with no Python traceback, suspect the layer below Python (DDS, C++ extensions, shared libraries). "Silent" is a strong diagnostic signal that the bug is at a boundary. See [`09_troubleshooting.md`](09_troubleshooting.md) for the full diagnostic.

---

## Lesson 2: A `package.xml` that omits runtime dependencies will pass CI and still break end users

**Context:** We audited the upstream repo before adding the simulation. A standard `rosdep install --from-paths src --ignore-src -y` on a fresh machine succeeded, but launching `openamrobot_docking.launch.py` then failed at runtime.

**Symptom:** `ros2 launch openamrobot_docking openamrobot_docking.launch.py` reported `Failed to load library libcamera_node.so` and `Could not find plugin AprilTagNode`. The launch file referenced ComposableNodes from `camera_ros`, `image_proc`, and `apriltag_ros`, none of which were installed.

**Why it happened:** `package.xml` only listed `launch`, `launch_ros`, `nav2_lifecycle_manager`, `opennav_docking`, `opennav_docking_msgs`, `rclpy`, and `std_msgs`. Five runtime packages were silently assumed to be present on the developer's machine but never declared:

- `camera_ros` — camera driver plugin in `launch/apriltag.launch.yml:12`
- `image_proc` — `RectifyNode` in `launch/apriltag.launch.yml:23`
- `apriltag_ros` — `AprilTagNode` in `launch/apriltag.launch.yml:36`
- `rclcpp_components` — the `component_container` executable
- `apriltag_msgs` — implicitly published by `AprilTagNode`

**How we fixed it:** Add the missing `<exec_depend>` entries to `package.xml`:

```xml
<exec_depend>camera_ros</exec_depend>
<exec_depend>image_proc</exec_depend>
<exec_depend>apriltag_ros</exec_depend>
<exec_depend>apriltag_msgs</exec_depend>
<exec_depend>rclcpp_components</exec_depend>
```

After that, `rosdep install` pulls them in on a fresh machine.

**Lesson:** `package.xml` is not optional documentation — it is the contract `rosdep` uses to bootstrap a workspace. Every plugin, every node executable, and every message type referenced from launch files must appear there. The fact that *your* machine has the package installed (from some other project) does not mean it will be available for the next person. See [`05_parameters.md`](05_parameters.md) for what each dependency contributes.

---

## Lesson 3: A missing static TF can make a node "succeed" while silently publishing nothing

**Context:** `detected_dock_pose_publisher` (C++ node in `src/`) is responsible for looking up `map → charging_dock_apriltag` from TF and republishing it as a `PoseStamped` on `/detected_dock_pose`. We launched the stack and saw the node start cleanly with no errors.

**Symptom:** `ros2 topic hz /detected_dock_pose` returned `no new messages`. The node was alive (visible in `ros2 node list`), `apriltag_ros` was publishing detections on `/apriltag/detections`, but the bridge between detection and a `map`-frame pose produced zero output.

**Why it happened:** The full TF chain required is:

```
map → odom → base_link → camera → charging_dock_apriltag
```

`apriltag_ros` publishes the last hop (`camera → charging_dock_apriltag`). SLAM publishes `map → odom`. Odometry publishes `odom → base_link`. **Nobody publishes `base_link → camera`** — and on the real-robot launch (`openamrobot_docking.launch.py`) this static transform was never wired up. `lookupTransform("map", "charging_dock_apriltag", ...)` returned `LookupException` on every cycle. The node silently caught the exception and didn't publish.

**How we fixed it:** In the simulation, the URDF declares `base_link → camera_rgb_frame → camera_rgb_optical_frame` as fixed joints, and `robot_state_publisher` publishes these as `/tf_static`. For real-robot deployments, the user must add a `static_transform_publisher` (or include it in the URDF processed by `robot_state_publisher`) measuring the camera's mounting offset from `base_link`.

**Lesson:** A node that "starts cleanly" is not the same as a node that's working. When a TF-dependent node produces no output, always run `ros2 run tf2_ros tf2_echo <parent> <child>` for every hop in the expected chain. The first hop that fails is your missing piece. Catch-and-ignore on `LookupException` is convenient for transient startup races but hides permanent breakage — log at least once per second when transforms can't be resolved.

---

## Lesson 4: `use_sim_time` must be propagated to every included launch file

**Context:** While porting the original real-robot launch to a Gazebo simulation, we noticed `apriltag.launch.yml` was included from the top-level launch file without explicit arguments.

**Symptom:** With the rest of the stack reading the Gazebo `/clock`, the AprilTag pipeline ran on wall-clock time. The downstream `detected_dock_pose_publisher` dropped every transform with `Lookup would require extrapolation into the future` because the AprilTag TF carried a wall-clock timestamp while `map → odom` carried a sim-clock timestamp.

**Why it happened:** In `launch/openamrobot_docking.launch.py:49–55`, the include block was:

```python
IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('openamrobot_docking'), 'launch', 'apriltag.launch.yml']
        )
    ),
),
```

No `launch_arguments={'use_sim_time': use_sim_time}` was passed, so the AprilTag stack defaulted to `use_sim_time:=false`. Meanwhile the sibling `detected_dock_pose_publisher` include (lines 41–48) *did* forward the argument — so the inconsistency was right there in the same file.

**How we fixed it:** Forward the argument explicitly:

```python
IncludeLaunchDescription(
    AnyLaunchDescriptionSource([...]),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
),
```

In our simulation launch (`simulation/launch/simulation.launch.py`) we use a single `use_sim_time=True` literal everywhere and route it through every include.

**Lesson:** Time is a global property in ROS 2, and `use_sim_time` is the most error-prone parameter in the entire ecosystem. The failure mode is always the same: TF extrapolation errors that look like a "noisy sensor" problem. When you see those errors, first check `ros2 param get <node> use_sim_time` on every node in the pipeline. They must all match.

---

## Lesson 5: Mesh-based collisions create phantom obstacles for an internally mounted lidar

**Context:** We started with the SolidWorks-exported URDF/SDF for the OMR robot, which used the full visual STL mesh as the collision geometry for `base_link`. The lidar is mounted at `(0.35135, -0.001025, 0.1683)` relative to base_link, **inside** the body's bounding box.

**Symptom:** Nav2 refused to plan with `Failed to create plan with tolerance of: 0.500000` even with no real obstacles in the way. RViz showed a ring of lethal cells right against the robot. `ros2 topic echo /scan` confirmed range returns of 0.2–0.3 m all the way around at lidar height.

**Why it happened:** The gpu_lidar plugin casts rays from the sensor origin outward and reports the first collision hit. Because the lidar origin sits inside the body's collision mesh, every horizontal ray hits the *interior* of the body shell within centimetres and reports those hits as obstacles. The lidar was, in effect, seeing itself.

**How we fixed it:** Replace the body collision with a simplified box that excludes the sensor positions, while keeping the visual mesh untouched (the robot still looks right):

```xml
<collision name="base_collision">
  <pose>0.014 0 0.07 0 0 0</pose>
  <geometry>
    <box><size>0.5 0.3 0.16</size></box>
  </geometry>
</collision>
```

The box's top is at `z = 0.07 + 0.08 = 0.15 m`, safely below the lidar (`z = 0.1683 m`) and the camera (`z = 0.22 m`). Width `y = 0.3 m` excludes the drive wheels at `y = ±0.234 m` so the lidar can see past them.

**Lesson:** Visual fidelity and collision fidelity are decoupled in Gazebo/SDF — exploit that. Keep collisions as simple primitives that contain only what the physics needs to know about, and never let a ray-casting sensor sit inside its own collision geometry. The same trick applies to URDFs intended for MoveIt or contact-based grasp planning.

---

## Lesson 6: Caster wheels with swivel joints are unstable in ODE — fix them

**Context:** Our OMR has four caster assemblies (front-left/right and back-left/right), each modelled as a swivel joint above a small sphere wheel. The original SDF declared all eight joints as `revolute`.

**Symptom:** `ros2 topic echo /joint_states` showed back caster wheel velocities oscillating at **±21 rad/s** while we commanded a straight-line motion. The robot drifted, got stuck, or shuddered in place.

**Why it happened:** Three properties of the model combined to create chaotic feedback in ODE:
1. The wheel sphere is frictionless (`mu=0`) and has no preferred rolling axis — it can spin about any direction.
2. The swivel joint above it is free-spinning with negligible damping.
3. The caster has a small horizontal offset from its swivel pivot, so angular momentum couples between the two joints.

The integrator amplified the resulting noise into bounded but loud oscillations.

**How we fixed it:** Convert all eight caster-related joints to `type="fixed"` in both the SDF and the URDF:

```xml
<joint name="fl_caster_joint" type="fixed">...</joint>
<joint name="fl_wheel_joint"  type="fixed">...</joint>
```

The casters become rigid skid pads with `mu=0` — they slide silently as the robot turns. This loses caster realism (real casters do swivel), but for navigation/docking work the trade-off is well worth it.

**Lesson:** Not every degree of freedom that exists on the real robot needs to be simulated. If a joint has no actuation, no measurement, and no impact on the high-level behaviour you're studying, lock it. Free joints with low damping are a common source of numerical instability in any rigid-body simulator.

---

## Lesson 7: Caster radius is a knife-edge tradeoff between traction and tipping

**Context:** After locking the caster joints, we still had to choose the right caster sphere radius. Too tall and the casters press into the ground, lifting the drive wheels. Too short and the robot tips during accelerations.

**Symptom:** With the original radius of `0.0175 m`, the drive wheels barely contacted the ground. `ros2 topic echo /joint_states` showed drive-wheel angular velocity but the robot didn't translate. With a softer caster stiffness (`kp=1e+3`) at intermediate radii, the robot moved but tipped during fast rotations.

**Why it happened:** With four casters and two drive wheels at the same height, the ground reaction is split across six contact points. The center of mass falls inside the polygon, so the casters take some of the load. Since the casters are frictionless, the part of the weight they bear is "wasted" — it doesn't contribute to traction. With `kp=1e+8` (very stiff), even sub-millimetre penetration produces tens of Newtons of force per caster, easily lifting the drive wheels off the ground.

**How we fixed it:** Set `radius = 0.016 m` so that the casters sit `0.4 mm above` the ground at rest. The drive wheels carry 100% of the weight. The robot is statically stable on two wheels because the CG is at `z ≈ 8 cm` and the wheel base is `41 cm` wide (a 3:1 width-to-height ratio is comfortable).

Stability under acceleration is then guaranteed by gentle accel limits, not by the casters:

```yaml
velocity_smoother:
  ros__parameters:
    max_accel: [1.2, 0.0, 1.5]
    max_decel: [-1.2, 0.0, -1.5]
```

…and a conservative RPP controller (`max_angular_accel: 0.6`, `rotate_to_heading_angular_vel: 0.5`).

**Lesson:** When a parameter sits at a hard tradeoff between two failure modes, the right answer is usually to remove one of the two by moving the constraint elsewhere. Here, we removed the "tipping" failure mode not by adding caster contact but by limiting the disturbances (accelerations) that cause tipping. See [`05_parameters.md`](05_parameters.md) for the velocity smoother limits.

---

## Lesson 8: ODE's `min_depth` parameter silently disables contact

**Context:** Even after fixing the caster radius, the drive wheel cylinders were visibly partly *below* the ground plane in Gazebo. The robot would rotate without translating, or skate sideways.

**Symptom:** The wheels appeared to sink ~1 mm into the ground. Visual inspection (Gazebo's wireframe view) showed the contact patch was below `z=0`.

**Why it happened:** The wheel collision parameters were copied from a TurtleBot3 SDF:

```xml
<contact><ode>
  <kp>1e+5</kp>
  <kd>1</kd>
  <max_vel>0.01</max_vel>
  <min_depth>0.001</min_depth>
</ode></contact>
```

`min_depth: 0.001` tells ODE to ignore contact penetration shallower than 1 mm. For a 4 kg robot on two wheels (~20 N per wheel) with `kp=1e+5`, the equilibrium penetration would be `20/1e+5 = 0.2 mm` — well below the threshold. So no contact force is applied. The wheel keeps sinking until it accumulates 1 mm of penetration and the contact finally activates, but by then the robot's geometry is wrong.

**How we fixed it:** Stiffen the contact and remove the threshold:

```xml
<contact><ode>
  <kp>1e+8</kp>
  <kd>1</kd>
  <min_depth>0.0</min_depth>
</ode></contact>
```

With `kp=1e+8`, the equilibrium penetration drops to `20/1e+8 = 0.2 µm` — negligible. The wheel sits cleanly on the ground.

**Lesson:** Copy-pasting contact parameters from another robot is dangerous — they were tuned for that robot's mass and stiffness. The two parameters that matter most are `kp` (contact stiffness) and `min_depth` (force activation threshold). Always check that the equilibrium penetration is well above `min_depth`, or set `min_depth=0` and rely on `kp` alone.

---

## Lesson 9: Asymmetric URDF inertials make a "straight" command curve

**Context:** With the wheels touching the ground correctly, we sent `Twist(linear.x=0.2)` to `/cmd_vel` and expected the robot to track a straight line.

**Symptom:** The robot drifted consistently to one side. Both drive wheels reported identical angular velocities in `/joint_states`, ruling out a controller asymmetry.

**Why it happened:** The SolidWorks-exported inertial blocks had small but non-zero off-diagonal terms (`ixy`, `ixz`, `iyz`) for each wheel, and these terms were **not perfectly mirrored** between the left and right wheel. Combined with a base_link centre-of-mass offset of `y = -0.017 m` (more weight on the right), the asymmetry was enough to make ground friction torque differ between the two wheels during motion — even at identical wheel velocities.

**How we fixed it:** Override the drive wheel inertials in the SDF to be identical, diagonal, and centred:

```xml
<inertial>
  <pose>0 0 0 0 0 0</pose>
  <mass>1.06112497715273</mass>
  <inertia>
    <ixx>0.00339900487986557</ixx>
    <ixy>0</ixy>  <ixz>0</ixz>
    <iyy>0.00642634969436629</iyy>
    <iyz>0</iyz>
    <izz>0.0033973895137008</izz>
  </inertia>
</inertial>
```

This is pragmatic — the real wheels are roughly axisymmetric anyway, so a diagonal inertial is closer to physical truth than the slightly noisy CAD export.

**Lesson:** CAD-exported inertials carry numerical noise from mesh discretisation. For parts that should be symmetric (left/right wheels, top/bottom plates), enforce the symmetry by hand. On the real robot, an asymmetric CoM is real and unavoidable — there you'd keep the asymmetry and let a closed-loop controller compensate. In simulation, where the goal is to study the controller, removing the asymmetry is cleaner.

---

## Lesson 10: A camera resolution that exceeds what the bridge can stream silently drops frames

**Context:** We initially configured the Gazebo camera at 1280×720 @ 15 Hz, hoping the higher resolution would give more pixels on the AprilTag and improve `solvePnP` accuracy.

**Symptom:** `ros2 topic hz /camera/image_raw` reported ~0.2 Hz instead of the configured 15 Hz. `CameraInfo` on `/camera/camera_info` continued at 15 Hz as expected. `apriltag_ros` printed sync warnings and only detected the tag intermittently.

**Why it happened:** Two compounding bottlenecks. First, the gz camera plugin renders at the simulation tick rate, which is linked to the real-time factor (RTF). On a typical dev machine, all the physics + plugins drop RTF below 0.3 with HD rendering, so the effective image rate is at most `0.3 × 15 = 4.5 Hz`. Second, the `ros_gz_bridge` converts each frame from gz's image type to `sensor_msgs/Image` in a single thread. At raw 1280×720 RGB × 15 Hz ≈ 40 MB/s, the bridge can't keep up and drops most frames. `CameraInfo` is tiny and passes through unaffected, masking the issue.

**How we fixed it:** Drop the camera back to 640×480 @ 15 Hz:

```xml
<update_rate>15</update_rate>
<image>
  <width>640</width>
  <height>480</height>
  <format>R8G8B8</format>
</image>
```

Compensate for the lower pixel count by enlarging the physical AprilTag from 0.25 m to 0.40 m. At 1.5 m distance with focal length ≈ 467 px, the tag image is ~125 px on a side → 12 px per code cell → `solvePnP` is sub-pixel accurate.

**Lesson:** Always verify the effective rate of a topic with `ros2 topic hz` before relying on its configured rate. The configured rate is a *request*, not a guarantee — the actual rate depends on every layer between source and consumer. When two related topics have different rates (image vs camera_info here), the lower one wins for every consumer that needs both. See [`08_sequencer_4phase.md`](08_sequencer_4phase.md) for the tag-size analysis.

---

## Lesson 11: The camera frame must be the optical frame for apriltag_ros

**Context:** With the camera streaming and AprilTag detecting the tag, we expected the published `/detected_dock_pose` to be aligned with the world axes.

**Symptom:** `/detected_dock_pose` looked rotated by some weird combination of 90° increments. The robot would otherwise behave normally, but the dock target was in the wrong direction.

**Why it happened:** Gazebo's camera plugin tags each image and each TF entry with the frame named by `<gz_frame_id>`. `apriltag_ros`'s `solvePnP` produces poses in OpenCV's "optical" convention (X right, Y down, Z forward into the scene). If `gz_frame_id` points at a body-style frame (`camera_rgb_frame`, with X forward, Y left, Z up), the output is rotated by the body-to-optical transform — a fixed `(-π/2, 0, -π/2)` rotation per REP-103.

**How we fixed it:** Set the gz frame to the optical frame and ensure the URDF static transform from body to optical carries the standard rotation:

```xml
<!-- SDF -->
<gz_frame_id>camera_rgb_optical_frame</gz_frame_id>
```

```xml
<!-- URDF -->
<joint name="camera_rgb_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
  <parent link="camera_rgb_frame"/>
  <child  link="camera_rgb_optical_frame"/>
</joint>
```

Now `apriltag_ros` publishes `camera_rgb_optical_frame → charging_dock_apriltag`, which composes cleanly with the URDF's static transforms back to `base_link`.

**Lesson:** REP-103 specifies that camera-related frames come in pairs: a body frame (mounted on the robot, ROS-style axes) and an optical frame (rotated, OpenCV-style axes). Every camera-driven library you'll use (apriltag, OpenCV, image_pipeline) expects the *optical* frame. Always wire them up as a pair and use the optical one in the camera_info / image headers.

---

## Lesson 12: SLAM's map frame inherits the robot's spawn yaw

**Context:** Early on we spawned the robot at `(0, 0)` with yaw=π/2 (facing north) and configured the dock pose in map coordinates expecting "map = world".

**Symptom:** Commanding navigation to `map (2.2, 0)` made the robot drive north when we expected east. The dock was at world `(4.9, 0)` and the robot never reached it.

**Why it happened:** SLAM Toolbox (and most SLAM implementations) initialises the map frame so that the robot's *initial pose* sits at `map (0, 0, 0)` regardless of the robot's actual world pose. If the robot spawns with yaw=π/2, then `map +x` aligns with `world +y`. Every map-frame coordinate you configure (dock pose, goal pose, waypoints) is interpreted in this rotated frame.

**How we fixed it:** Spawn the robot at world `(-4, -4, 0.05)` with **yaw=0**. Now map and world axes are aligned — only translated by the spawn position. We get the simple relation:

```
map_x = world_x + 4
map_y = world_y + 4
```

And the dock at world `(0, 4.9)` becomes map `(4.0, 8.9)`, which is what `nav2_sim_full.yaml` and `dock_trigger.yaml` both encode.

**Lesson:** If you control the world layout, simplify the math by aligning map and world frames at startup. If you can't (real robot with pre-existing map), measure the rotation explicitly and document it next to every map-frame coordinate. The bugs that come from confusing two coordinate frames are persistent because the symptoms look like "controller error" — the robot reaches the goal it was given; the goal was just in the wrong frame.

---

## Lesson 13: A `static_layer` costmap blocks Nav2 from planning until SLAM has produced a map

**Context:** Our first costmap configuration followed the standard Nav2 examples: a `static_layer` subscribed to `/map` from SLAM, plus an `obstacle_layer` and `inflation_layer`.

**Symptom:** Immediately after `ros2 launch`, the global planner failed with `Robot is out of bounds of the costmap` or `Failed to create plan`. RViz showed the global costmap entirely grey (NO_INFORMATION). After ~5 s the costmap would fill in and planning would start working.

**Why it happened:** At launch, SLAM Toolbox hasn't yet emitted its first map (it takes ~4–5 s to activate and build the initial occupancy grid). The `static_layer` waits for `/map` and stays empty until then. During that window the global costmap has no data, so every cell looks unknown and the planner refuses. Adding `obstacle_layer` alongside `static_layer` produced timing-dependent inconsistencies — sometimes one layer would win, sometimes the other, depending on which message arrived first.

**How we fixed it:** Drop `static_layer` entirely. Use a rolling-window global costmap with only `obstacle_layer + inflation_layer`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: true
      width: 20
      height: 20
      resolution: 0.05
      track_unknown_space: false
      plugins:
      - obstacle_layer
      - inflation_layer
      obstacle_layer:
        observation_sources: scan
        scan:
          obstacle_max_range: 8.0
          raytrace_max_range: 10.0
          topic: /scan
```

`rolling_window: true` keeps a 20×20 m window centred on the robot, so we never go "out of bounds". `track_unknown_space: false` makes unknown cells free (consistent with `allow_unknown: true` on the planner). The lidar populates the obstacle layer immediately on the first `/scan` message, so planning works as soon as Nav2 is active.

The drawback is that you lose the long-term static map. For a small known room this is fine; for long missions you'd reintroduce `static_layer` but delay Nav2 startup until SLAM has emitted its first map.

**Lesson:** When two systems have an implicit timing dependency (Nav2 waits on SLAM here), make the dependency explicit either with a clean separation (drop one) or with launch-level sequencing. Implicit "race that usually works" wiring is the source of intermittent bugs that disappear when you try to reproduce them.

---

## Lesson 14: `cost_scaling_factor` controls trajectory wobble more than `inflation_radius` does

**Context:** Even with a clean costmap and a working planner, the global plan would visibly shift between successive replans, making the robot snake gently down a corridor.

**Symptom:** RViz showed the green global plan jumping ±5–10 cm side to side on every planner cycle (1 Hz), as if the planner was indecisive about which side of the corridor to favour. The controller then tried to follow these jittering plans and produced visible oscillation in the executed motion.

**Why it happened:** With the default `cost_scaling_factor: 3.0`, the inflated cost decays slowly with distance from obstacles. Cells well inside free space still have non-trivial cost contributions from every nearby obstacle. As the lidar's noise nudges those obstacles by a few centimetres between scans, the optimal path through the cost field shifts accordingly.

**How we fixed it:** Push `cost_scaling_factor` from `3.0` to `8.0`:

```yaml
inflation_layer:
  cost_scaling_factor: 8.0
  inflation_radius: 0.45
```

With a higher scaling factor, the cost falls off sharply with distance. Cells more than ~25 cm from any obstacle have nearly zero cost. Lidar noise within that ~25 cm zone matters; beyond it, the planner sees a flat cost field and picks the same path every cycle.

**Lesson:** `inflation_radius` controls *how far* the inflation reaches; `cost_scaling_factor` controls *how steep* the falloff is. The two parameters have very different effects: a large radius with a steep scaling produces a sharp wall around obstacles (good for keeping plans stable); a small radius with a gentle scaling produces a soft gradient (good for keeping plans away from obstacles). When trajectories wobble despite no real moving obstacles, increase scaling before increasing radius.

---

## Lesson 15: Switch to A* with a wider tolerance to avoid Dijkstra's edge cases

**Context:** Even after the costmap was clean, the planner occasionally failed.

**Symptom:** Two distinct error messages appeared intermittently in the planner_server logs:

```
Failed to create plan with tolerance of: 0.500000
Failed to create a plan from potential when a legal potential was found. This shouldn't happen.
```

The second message is from `nav2_navfn_planner`'s Dijkstra implementation and refers to a known edge case where the potential field algorithm finds a valid solution but the path reconstruction step fails.

**Why it happened:** Dijkstra-based NavFn has a known issue with path reconstruction when the goal sits at the boundary of inflated cells. A* uses a different search and reconstruction strategy that avoids this corner case. The tolerance error happened when the goal cell itself was occupied by transient inflation (e.g., the dock briefly appearing as a lidar obstacle), and 0.5 m wasn't enough slack for the planner to find a nearby free cell.

**How we fixed it:** Switch planner mode and widen tolerance:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: true
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 1.0
      use_astar: true
```

`use_astar: true` switches the same plugin from Dijkstra to A* mode (the plugin supports both). `tolerance: 1.0` lets the planner accept any free cell within 1 m of the goal, which is more than enough for our docking use case where the staging pose has ~10 cm of inherent uncertainty anyway.

**Lesson:** Default values in robotics frameworks are conservative starting points, not optimal settings. When you see "rare" errors that have the structure of `<algorithm>: this shouldn't happen`, that's a documented edge case in that algorithm — switch algorithms rather than trying to suppress the symptom.

---

## Lesson 16: The Nav2 `cmd_vel` chain is three hops long — diagnose each hop

**Context:** After triggering docking, we expected `/cmd_vel` to carry the controller's output to the gz bridge and into the DiffDrive plugin. The robot didn't move.

**Symptom:** `/cmd_vel_nav` had data at 10 Hz (controller was producing output), but `/cmd_vel_smoothed` and `/cmd_vel` were both silent.

**Why it happened:** Nav2 Jazzy's `navigation_launch.py` rewires the conventional topic name `cmd_vel` for three nodes: `controller_server`, `behavior_server`, and `velocity_smoother`. The chain becomes:

```
controller_server      → /cmd_vel_nav
velocity_smoother      → /cmd_vel_smoothed (subscribes /cmd_vel_nav)
collision_monitor      → /cmd_vel          (subscribes /cmd_vel_smoothed)
ros_gz_bridge          → gz /cmd_vel       → DiffDrive plugin
```

Every hop is a separate node with its own subscribe/publish topic configuration. In our case the `collision_monitor` had `FootprintApproach` enabled with a poor footprint source, which detected a phantom imminent collision and clamped the output velocity to zero.

**How we fixed it:** Two things. First, disable `FootprintApproach` for the simulation:

```yaml
collision_monitor:
  FootprintApproach:
    enabled: false
```

Second, verify the topic names on `collision_monitor` match the chain:

```yaml
collision_monitor:
  cmd_vel_in_topic: cmd_vel_smoothed
  cmd_vel_out_topic: cmd_vel
```

The diagnostic recipe for any future "robot doesn't move" issue is:

```bash
ros2 topic hz /cmd_vel_nav         # 10 Hz when navigating? if no → controller_server
ros2 topic hz /cmd_vel_smoothed    # 10 Hz? if no → velocity_smoother (or its inputs)
ros2 topic hz /cmd_vel             # 10 Hz? if no → collision_monitor blocking
ros2 lifecycle nodes               # all 'active'?
```

The first hop with no data is where the chain breaks.

**Lesson:** Long pipelines fail by having one stage drop the data silently. The fix isn't to read more logs — it's to instrument every link with `topic hz` and find the first one that's quiet. Topic renaming via remaps makes pipelines flexible but also makes them harder to trace; consider documenting the chain in a comment in the launch file.

---

## Lesson 17: opennav_docking's curved approach is wrong for head-on AprilTag docking

**Context:** The upstream `openamrobot_docking` uses `opennav_docking::SimpleChargingDock` with `graceful_controller` for the approach phase. We tried it in simulation.

**Symptom:** Final docking pose was rarely head-on. The robot arrived at the dock from the side, often by ~10° of heading error and 5–10 cm of lateral offset — enough for a physical contact point to miss the mating connector.

**Why it happened:** Three compounding issues:

1. **Curved trajectory.** `graceful_controller` blends linear and angular velocity simultaneously, so the robot's path during the final approach is a smooth curve. If the staging pose has any lateral offset (typical, given Nav2's `xy_goal_tolerance` of 10–20 cm), the controller arcs to reach the dock — the final heading is correct but the arrival vector is not perpendicular to the tag.

2. **Detection noise.** A single AprilTag detection at staging distance has ~13 cm RMSE in our setup (0.40 m tag at 1.5 m with a 640×480 camera). One-shot detection feeds noise directly into the target.

3. **Pose ambiguity.** `solvePnP` on a planar tag has two valid solutions (the tag's two possible orientations that produce the same projection). When the tag is small in the image, the algorithm can flip between them frame to frame.

4. **Magic-constant rotations.** `opennav_docking`'s `external_detection_rotation_*` requires a 3-axis rotation to convert the tag's frame to its dock frame. The sign convention depends on which `apriltag_ros` version's `solvePnP` output you're using, and getting it wrong sends the robot through the wall.

**How we fixed it:** Rewrote `dock_trigger.py` as a custom 7-phase sequencer that bypasses `opennav_docking` entirely. The key insight is that the final approach should not be a smooth curve — it should be a straight line. So we explicitly decompose the docking into:

1. Rough staging via Nav2.
2. Hold 1 s to let the robot fully settle.
3. **Average 20 tag detections** to produce a low-noise dock pose (kills problem 2).
4. Compute the **parallel spot** (a point on the line perpendicular to the tag, at staging distance, on the robot's side). Sign is disambiguated by `dot(robot − tag, tag_normal)` (kills problem 3).
5. Rotate-drive to the parallel spot (corrects lateral offset accumulated from Nav2).
6. Spin in place to face the tag.
7. **Drive forward in a straight line, auto-calibrating** yaw from the live filtered tag pose. The target yaw depends only on the tag's *orientation* (not position), so the path is straight even with sub-decimetre lateral error.

See [`08_sequencer_4phase.md`](08_sequencer_4phase.md) for the full algorithm and parameters in [`05_parameters.md`](05_parameters.md).

**Lesson:** General-purpose docking frameworks make assumptions (smooth blend of linear and angular velocity, single-shot perception, fixed-frame target) that don't hold for precision AprilTag docking. When the assumptions don't match your problem, replacing the framework with explicit phases is often cleaner than tuning around it. The signs of mismatch were already visible in the upstream config — `use_external_detection_pose: false` was hiding the fact that the detection pipeline wasn't actually being used.

---

## Lesson 18: Temporal filtering plus auto-calibration beats one-shot perception

**Context:** Even after fixing the trajectory shape to be straight, we still needed accurate target pose to make a straight line land on the dock.

**Symptom:** Initial single-shot detection at staging distance produced a target pose with ~10 cm of lateral error. The robot drove straight, but to a point ~10 cm off the dock.

**Why it happened:** AprilTag detection accuracy improves dramatically as the tag fills more of the image. At 1.5 m the tag is ~125 px wide and `solvePnP` has good but imperfect accuracy. At 0.7 m the tag is ~270 px wide and accuracy is roughly 4× better. So the closer the robot gets, the better the perception — but the standard approach freezes the target at the staging distance, ignoring the more accurate detections collected during the approach.

**How we fixed it:** Two complementary techniques, both later replaced by a simpler design (see end of this lesson).

First, **temporal averaging at staging**: collect `filter_num_samples` detections (originally 20, then bumped to 40 in the current design) over `filter_max_collect_time` seconds, then average position and quaternion (renormalising the quaternion). This kills high-frequency noise and gives a clean starting target.

Second, **auto-calibration during the advance phase**: at each iteration of the advance, read the latest `/detected_dock_pose`, compute the target yaw (perpendicular to the tag), and low-pass filter it with α=0.15:

```python
yaw_target = (1 - alpha) * yaw_target + alpha * yaw_from_latest_tag
```

The target yaw depends only on the tag's *orientation*, not its position, so the robot's straight-line path is preserved.

**Update (current design):** the exponential low-pass auto-calibration was later replaced by the `TagRunningAverage` (true incremental running mean for position; sign-aligned componentwise mean for the quaternion). The running average keeps updating continuously throughout phases 2 and 4, has no arbitrary blend coefficient, and is provably the unbiased estimator for a static tag — see [`08_sequencer_4phase.md`](08_sequencer_4phase.md) and lesson 26 above. The principle of this lesson (re-read the sensor as it improves) still holds; only the filter shape changed.

**Lesson:** Sensors that improve with proximity should be re-read continuously, not snapshot once. When you have a control loop with a long approach to a precision target, design it so that the *most accurate* measurement (the one closest to the target) has the most influence on the final state. One-shot perception at the worst possible measurement distance is the opposite pattern.

---

## Lesson 19: Ctrl-C does not kill all processes a complex launch file spawns

**Context:** During iteration we relaunched the simulation dozens of times. Sometimes Ctrl-C in the launch terminal left zombies behind.

**Symptom:** Subsequent `ros2 launch openamrobot_docking simulation.launch.py` would fail with `port already in use`, `node name conflict`, or just hang. `ps aux | grep gz` showed `gz-sim` still running. `ros2 node list` showed nodes from the "previous" launch alongside the new ones.

**Why it happened:** `ros2 launch` propagates SIGINT to its direct children, but several of those children are themselves launchers (the ruby `gz` script, ComposableNode containers, the Python-spawned `slam_toolbox`). The signal doesn't always reach grandchildren. Worse, the Gazebo server runs in its own process group and survives most cleanup attempts.

**How we fixed it:** Wrote `scripts/kill_sim.sh` which SIGKILLs every process the launch spawns:

```bash
~/Downloads/openamrobot-docking-main/openamrobot_docking/scripts/kill_sim.sh
```

It targets `gz-sim`, `parameter_bridge`, `slam_toolbox`, `bt_navigator`, `controller_server`, `planner_server`, the ros_gz_bridge, the AprilTag container, RViz, and `dock_trigger.py`. Run it whenever a relaunch hangs or topics from a previous run are still visible.

**Lesson:** "Graceful shutdown" is a best-effort property of every Linux process tree, especially when multiple language runtimes (C++, Python, Ruby) are involved. For a development workflow, accept this and keep a hard-kill script handy. For production, deeper investigation into systemd units or container isolation is the right answer.

---

## Lesson 20: Validate every layer in isolation before integrating

**Context:** When we first built the simulation, several layers were broken simultaneously — no DiffDrive plugin (robot couldn't move), no lidar sensor block (no `/scan`), no camera sensor block (no `/camera/image_raw`). Trying to debug Nav2 in this state was hopeless because every layer it depended on was also broken.

**Symptom:** Multi-symptom failures: Nav2 couldn't plan (no costmap, because no scan), the docking server couldn't perceive (no camera), and the robot wouldn't move even when commanded directly (no DiffDrive plugin). All errors looked like "Nav2 is broken" because Nav2 was the top of the stack.

**Why it happened:** When N layers each have a bug, the top layer's error message describes only the highest-visibility symptom. Trying to fix that error directly leads down rabbit holes because each "fix" exposes the next layer's bug.

**How we fixed it:** Bottom-up validation, one layer at a time:

1. **Plugin layer.** Add DiffDrive to the SDF. Test by publishing manually: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`. Robot moves? Move on.
2. **Sensor layer.** Add lidar `<sensor>` block. Test: `ros2 topic hz /scan` returns 10 Hz. Move on.
3. **Camera layer.** Add camera `<sensor>` block. Test: `ros2 topic hz /camera/image_raw` returns 15 Hz, view in `rqt_image_view`. Move on.
4. **Localisation.** Launch SLAM. Test: `/map` published, `map → odom → base_link` chain complete in `ros2 run tf2_ros tf2_echo`. Move on.
5. **Navigation.** Launch Nav2 lifecycle, send a manual goal in RViz. Path planned and followed? Move on.
6. **Detection.** Launch apriltag + detected_dock_pose_publisher. Test: `/detected_dock_pose` updates as the robot moves.
7. **Docking.** Now trigger the full sequence.

**Lesson:** Build robotics stacks bottom-up and test each layer with a minimal external input before adding the layer above. The diagnostic commands for each layer (`topic hz`, `topic echo`, `tf2_echo`, `lifecycle nodes`) are cheap. The cost of not running them is hours of misattributed debugging at the top of the stack.

---

## Lesson 21: Documentation that drifts from code is worse than no documentation

**Context:** While auditing the upstream repo we found several documentation files that contradicted the active code.

**Symptom (selection):**

- `docs/04_apriltag.md` documented topic names `/rgb_camera/image_raw` and `/rgb_camera/camera_info`. The actual remaps in `launch/apriltag.launch.yml` used `/camera/camera/image_raw` and `/camera/camera/camera_info`. The `/rgb_camera/` namespace did not exist anywhere in the codebase — it was a leftover from a previous launch configuration.
- `docs/04_apriltag.md` told users to edit `apriltag_ros/launch/camera_36h11.launch.yml`, which is a read-only system file at `/opt/ros/jazzy/share/apriltag_ros/launch/`.
- `docs/05_parameters.md` referenced `apriltag_ros/cfg/tags_36h11.yaml` as the active config file. The actual active config was `config/tags_36h11.yaml` in this package.
- `docs/03_tf_frames.md` documented `camera_optical_frame` as the parent of the AprilTag TF. The actual frame published by `AprilTagNode` was `camera` (from `camera_info.header.frame_id`, which was set by the camera node's namespace).

**Why it happened:** The docs were written once against an earlier version of the code and never updated. The code evolved (topic remaps changed, frame names changed, file locations moved) but no one cross-checked the documentation against the new state. There's no automated test that fails when docs drift.

**How we fixed it:** Rewrote the docs to match the code, and tagged each doc with a "last verified against commit" line so future drift is at least observable.

**Lesson:** Documentation is a separate codebase that needs its own maintenance. The minimum bar is to grep through the docs whenever you rename a topic, move a file, or change a frame. Better is to have docs that reference the code directly (e.g., docstrings, generated reference). When you spot drift, fix it immediately — a doc that's wrong is worse than no doc because students trust it and waste time matching their setup to a fiction.

---

## Lesson 22: Don't ship a `_backup` config inside the package share

**Context:** The upstream repo had `config/openamrobot_docking_backup.yaml` alongside the active `config/openamrobot_docking.yaml`. Both were installed into the package share directory by `CMakeLists.txt:31`.

**Symptom:** Students reading the configs to understand the system found two files with similar names and different values:

| Parameter | Active config | Backup file |
|---|---|---|
| `home_dock.pose` | `[2.15, 0.0, 0.0]` | `[2.28, 0.0, 0.25]` |
| `initial_perception_timeout` | `25.0` | `15.0` |
| `dock_collision_threshold` | `0.25` | `0.5` |

Which one is "real"? Was someone in the middle of a change? Was the backup the previous known-good?

**Why it happened:** Someone wanted to save a known-good copy before experimenting. They named it `_backup` instead of using git, and never deleted it after the experiments stabilised. The `install(DIRECTORY config ...)` in `CMakeLists.txt` then shipped it to every user.

**How we fixed it:** Removed the backup file. Future "I want to save this" moments should be handled with `git stash`, a branch, or a tagged commit — not a parallel YAML in the same folder.

**Lesson:** Version control exists specifically to avoid backup-named files. Anything shipped in the package share is part of the public API of the package — users will read it and try to make sense of it. Keep that surface area minimal and intentional. If you must keep a historical config, put it under `docs/` with a clear comment explaining what it is.

---

## Lesson 23: A config flag set to `false` with a comment saying "enable" is worse than either

**Context:** While reading `config/openamrobot_docking.yaml` we found two stanzas like this:

```yaml
# CHANGED: enable stall detection so it stops if it physically can't move
use_stall_detection: false

# CHANGED: enable collision detection so it stops before hitting/pushing into wall
use_collision_detection: false
```

**Symptom:** Reading the YAML, you can't tell whether the original value was `true` (and someone disabled it later but didn't update the comment) or `false` (and the comment describes an intent that was never carried out).

**Why it happened:** Someone added a comment describing what enabling the feature would do, then either disabled the feature later for testing or never enabled it in the first place — and the comment got petrified in the as-found state.

**How we fixed it:** Either delete the misleading comment, or update it to explain the actual current state and why it's that way. The right pattern is:

```yaml
# DISABLED: stall detection was triggering false positives during rotation.
# Re-enable once the velocity smoother is tuned to avoid <0.05 m/s commanded.
use_stall_detection: false
```

**Lesson:** Comments next to config values should describe the *current* state and *why* it's that way, not aspirational future states. A comment that contradicts the value next to it is a trap for the next reader. If you find yourself writing "we should enable this someday" in a comment, file an issue instead.

---

## Lesson 24: Reserve a single source of truth for each tunable parameter

**Context:** The original repo had `dock_type: 'openamrobot_dock'` hardcoded in `scripts/dock_trigger.py:22` as a parameter default, but the same key was absent from `config/dock_trigger.yaml`. The dock plugin in `openamrobot_docking.yaml` was named `home_dock_plugin` (class `opennav_docking::SimpleChargingDock`) — so `'openamrobot_dock'` did not match any plugin name.

**Symptom:** No runtime error today (because `use_dock_id=true` makes `dock_type` unused), but the orphaned constant would cause a silent lookup failure the moment anyone flipped `use_dock_id` to `false`.

**Why it happened:** A parameter was added to the Python source as a default, never propagated to the YAML, and the YAML continued to be treated as the canonical config. Two sources of truth → one of them is wrong.

**How we fixed it:** Either declare all tunable parameters in the YAML and use it as the sole source, or document explicitly which parameters are intentionally code-only. For `dock_type` specifically, we either add it to `dock_trigger.yaml` matching a real plugin name, or remove it from the Python and from the goal message.

**Lesson:** For every parameter, there should be exactly one canonical place to set it. Defaults in code are fine for "rarely tuned, sensible default" parameters; for anything a user is expected to change per-robot or per-site, the YAML is the source of truth. Mixing the two creates configurations that look consistent but aren't. See [`05_parameters.md`](05_parameters.md) for the parameter inventory.

---

## Lesson 25: The URDF root spawn z is not the same as `base_link` z

**Context:** When migrating the simulation from an inline SDF model
(`simulation/models/omr_robot/model.sdf`) to a SolidWorks-exported
URDF/xacro (`omr_description/urdf/omr_robot.urdf.xacro`), we
introduced a `base_footprint` link as the URDF root. The xacro
applies a `base_joint` of `xyz="0 0 0.053467"` between
`base_footprint` and `base_link`, so that `base_link` sits 5.3 cm
above the ground projection and the wheel centres land exactly at
z = `wheel_radius` = 0.1 m.

We initially kept the old spawn argument `-z 0.053467`. With
`base_footprint` (the URDF root) spawned at z = 0.053467, `base_joint`
then lifted `base_link` to z = 0.107 and the wheel centres to
z = 0.153 — floating 5.3 cm above the ground.

**Symptom:** The robot looked correct in RViz, but visually slid
across the floor in Gazebo: only the casters (with full STL geometry)
touched the ground, and the drive wheels had no traction. The
docking sequence then exhibited slip during every spin and curve,
plus a 43 cm bias in the AprilTag detection (likely because the
camera was 5.3 cm higher than expected and the simulated
`camera_info` projection was inconsistent with the rendered image).

**Why it happened:** SDF and URDF have different conventions. In the
inline SDF, `base_link` was the root and the spawn pose set it
directly. In URDF with `base_footprint` as the root, the spawn pose
sets the ground projection, and the URDF lifts `base_link` from
there.

**How we fixed it:** Set `-z 0.0` in the spawn arguments in
`simulation.launch.py`. The drive wheels now touch the ground at
rest; slip disappears; the AprilTag map-frame bias dropped from
~43 cm to ~32 cm (the remainder is unrelated and discussed in
Lesson 28).

**Lesson:** When changing the URDF root, audit every place that
sets a spawn z, a static transform, or a ground-relative coordinate.
"Spawn at 0.05" is a different thing in two URDFs whose roots
differ. [`03_tf_frames.md`](03_tf_frames.md) now documents the
convention explicitly.

---

## Lesson 26: A continuous controller beats a discrete state machine for line tracking

**Context:** The earlier 7-phase sequencer included a discrete
"reverse-and-realign" sub-state machine that triggered whenever the
robot drifted off the dock's perpendicular axis during the final
advance. It would compute a target point on the axis, spin so the
robot's back faced it, reverse straight to it, then spin back to
perpendicular and re-advance.

In testing, this state machine looped: the curved reverse changed
the robot's yaw, which made the resumed advance drift laterally
again, which re-triggered the realign, which looped.

**Why it happened:** Each discrete step optimised one geometric
quantity (yaw, lateral position) at the cost of the other. The
robot could converge on one only by drifting on the other.

**How we fixed it:** Replaced the entire phase with a pure-pursuit
controller that drives
`desired_yaw = perp_yaw − atan2(lateral, line_lookahead_distance)`.
The `atan2` bounds the heading deviation and ensures
`desired_yaw → perp_yaw` as `lateral → 0`. With this formulation
there is no "off-axis equilibrium" — the only fixed point of the
closed loop is `lateral = 0` with the robot perpendicular to the
tag.

A short formal argument: define `lateral` and `yaw_err` as state;
the linearised dynamics are

    d(lateral)/dt = −v · yaw_err
    d(yaw_err)/dt = −K_yaw · yaw_err + K_yaw · (v / lookahead) · lateral

which is stable for any `K_yaw > 0` and `lookahead > 0`. The
earlier combined `omega = K_yaw · yaw_err − K_lat · lateral` had a
stable non-zero equilibrium `yaw_err = (K_lat / K_yaw) · lateral`,
which is exactly the off-axis bias we observed in the field.

**Lesson:** When two state variables are coupled via the
trajectory, a single continuous controller whose fixed point is the
joint zero of both is usually better than a state machine that
alternates between optimising each. The state machine is also much
harder to reason about. See
[`08_sequencer_4phase.md`](08_sequencer_4phase.md) for the full
controller.

---

## Lesson 27: Camera-frame angle is the right reference for centring scans

**Context:** Phase 2 (the tag-search scan) originally stopped as
soon as it received N consecutive fresh tag detections. The next
phases then assumed "tag detected" was equivalent to "tag visible
enough to filter accurately".

In practice, when the tag was at the edge of the camera frustum,
perspective distortion made `solvePnP` biased — the running average
inherited that bias, and the line-tracking phase then converged
onto a slightly wrong perpendicular axis.

**Why it happened:** "Tag detected" and "tag well measured" are not
the same thing. `apriltag_ros` happily detects a tag occupying 10
pixels at the edge of the image; `solvePnP`'s pose accuracy at that
configuration is poor.

**How we fixed it:** Made the scan a closed-loop centring
controller using the camera-frame angle directly. We look up TF
`camera_rgb_optical_frame → charging_dock_apriltag`, compute the
horizontal angle in the image as `atan2(X_optical, Z_optical)`, and
rotate the robot to bring it within ±2°. The scan exits only when
the tag has been within tolerance for `scan_consecutive_target`
consecutive frames.

Using the camera-frame angle (rather than the map-frame
`/detected_dock_pose`) makes the scan robust to any map-frame
solvePnP bias: even if the tag's *position* in `map` is off, the
camera-frame angle still reflects where the tag actually appears in
the image.

**Lesson:** When the controller's job is to centre the tag in the
camera, use the camera-frame measurement, not the world-frame one.
The world-frame estimate filters through TF chains and any error in
TF or camera calibration propagates; the camera-frame angle is
direct.

---

## Lesson 28: Don't run a closed-loop yaw correction inside the near-field

**Context:** Phase 4's pure-pursuit controller is well-behaved when
the robot is more than ~1 m from the tag. But as the robot
approaches, a few centimetres of lateral motion produce a large
angular change of the tag's position in the image. Closed-loop
corrections near the tag over-react: the robot oscillates left and
right trying to keep the tag centred.

**Why it happened:** The image-angle to a fixed target scales as
`atan(lateral / distance)`. At lateral = 5 cm and distance = 0.5 m,
that's 5.7°. At distance = 2 m, it's 1.4°. Same lateral offset,
4× the perceived angle. Any controller tuned for far-field
behaviour over-steers in the near-field.

**How we fixed it:** Introduced `visual_servo_distance` (default
1.4 m). For `distance > visual_servo_distance` the pure-pursuit
controller runs as designed. At the moment of crossing the
threshold, the robot does one in-place spin to the latest
running-average perpendicular yaw, then drives forward with
`omega = 0` until `docking_distance`.

The straight-line phase trades a small final lateral offset (bounded
by what the line-tracking phase already converged to) for a
guaranteed-stable final approach. There is no closed loop running
near the tag.

**Lesson:** Closed-loop control near a singular reference (tag
right in front of the camera, where small lateral shifts look huge)
is intrinsically unstable. The cleanest fix is to **open the loop**
in that regime — accept the residual offset and stop correcting.
This is conceptually similar to gain scheduling: the optimal gain
in the near-field is zero.

---

## Lesson 29: Auto-derive coordinate transforms in the launch instead of hard-coding them

**Context:** The dock pose in the map frame depends on where SLAM
toolbox initialises the map, which is the robot's spawn pose.
Initially `dock_trigger.yaml` hard-coded `dock_pose_*` for the
default spawn at `(-4, -4)`. Any test from a different spawn
required editing the yaml and rebuilding, with a risk of forgetting
the change.

**How we fixed it:** Added `spawn_x`, `spawn_y`, `spawn_yaw`
launch arguments to `simulation.launch.py` and an `OpaqueFunction`
that computes the dock's map-frame pose from the fixed world dock
pose:

    dock_in_map = R(−spawn_yaw) · (dock_in_world − spawn_in_world)

The result overrides `dock_pose_x`, `dock_pose_y`, `dock_pose_yaw`
on the `dock_trigger` node at launch time. Users can now invoke

    ros2 launch openamrobot_docking simulation.launch.py \
        spawn_x:=2.0 spawn_y:=-3.0 spawn_yaw:=1.57

and the docking sequencer keeps pointing at the same physical tag
without any yaml edit.

**Lesson:** Any coordinate that is derivable from a launch
argument should be derived in the launch, not duplicated in
multiple config files. The yaml then carries only the *defaults*
for the default invocation. Coordinate translation between frames
is a classic place to put `OpaqueFunction` to good use, because
launch substitution alone can't do the arithmetic.

---

## Summary — meta-lessons

A few patterns that emerged across all the lessons above:

1. **When the symptom is silent, the bug is at a boundary.** FastDDS crashing the Python action client (Lesson 1), missing TF making a node publish nothing (Lesson 3), missing `use_sim_time` causing extrapolation drops (Lesson 4), Nav2 cmd_vel chain breaking mid-pipeline (Lesson 16) — all of these gave no Python-visible error because the bug crossed a language boundary, a transform boundary, a time boundary, or a topic-remap boundary. The diagnostic rule: when something is "silent", probe the layer just below where you stopped looking.

2. **`use_sim_time` is the single most error-prone parameter in ROS 2.** It must be set consistently across every node, every included launch file, and every command-line tool. The failure mode is always TF extrapolation errors that look like sensor noise. Always check it first when you see timestamp-related errors.

3. **A configured rate is a request, not a guarantee.** Camera resolution exceeding the bridge's throughput (Lesson 10), planner failing because the costmap isn't populated yet (Lesson 13), controller publishing but the chain dropping the message (Lesson 16) — every "configured X Hz" needs to be verified with `ros2 topic hz` before you build anything on top of it.

4. **General-purpose frameworks make assumptions; verify they match your problem.** opennav_docking assumes a smooth-blend curved approach is acceptable (Lesson 17); Nav2's default planner assumes Dijkstra path reconstruction always succeeds (Lesson 15); the default costmap assumes a static map is available (Lesson 13). When the assumptions don't match, the right move is often to replace the component, not to fight its parameters. Read what the framework assumes before you trust its defaults.

5. **Bottom-up validation is cheaper than top-down debugging.** Every layer in a robotics stack has a 2-second diagnostic command (`topic hz`, `tf2_echo`, `lifecycle nodes`). Running them as you bring up the stack saves hours of misattributed debugging when the top of the stack inevitably has its own bugs added on top of the lower ones.
