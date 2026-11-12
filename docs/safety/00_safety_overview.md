# Safety overview

The OpenAMRobot motion-safety envelope is a **layered software + firmware** design. This
document states what each layer does, what it guarantees, and — deliberately up front — **what
is not covered**.

> **Status: this is not a certified functional-safety system.** There is currently **no
> hardware E-stop and no safety_io** (see §4, Gaps). The layers below reduce the chance and
> severity of a collision; they do not provide a guaranteed independent stop.

---

## 1. The layers, outermost to the wheels

Every navigation command passes through this chain before it reaches the motors:

```
planner/controller  ─►  footprint check (DWB)      ← refuses trajectories that touch (soft, per-cycle)
        │
        ▼
   /cmd_vel_nav
        │
        ▼
   velocity_smoother   ← clamps velocity + accel/decel to approved limits          [02_limits_and_watchdog.md]
        │
        ▼
   /cmd_vel_smoothed
        │
        ▼
   collision_monitor   ← reactive last-resort: slows/stops before footprint hits   [01_collision_monitor.md]
        │
        ▼
      /cmd_vel
        │
        ▼
   firmware (Teensy)   ← 200 ms /cmd_vel watchdog: zero the motors if commands stop [openamr-platform-fw]
        │
        ▼
      motors
```

| Layer | Where | Guarantees | Limits |
|---|---|---|---|
| **Footprint avoidance** | `controller_server` DWB `ObstacleFootprint` critic + (optionally) enlarged footprint | Won't *plan* a trajectory that brings the footprint into a lethal cell | Only for obstacles **in the costmap** — misses sub-18 cm objects (2D lidar) |
| **Velocity clamp** | `velocity_smoother` | Command never exceeds approved speed/accel/decel | Open-loop; does not know actual wheel speed |
| **Reactive collision guard** | `collision_monitor` | Slows/stops before a *projected* footprint collision from live scan | Depends on `/scan_filtered`; self-view false positives (see §3 below) |
| **Firmware watchdog** | Teensy firmware | Motors stop if `/cmd_vel` stops arriving (~200 ms) | Protects against a dead/stalled software stack, not against a *wrong* command |

The footprint-avoidance layer is a **navigation** concern (documented in
[`../navigation/02_costmaps.md`](../navigation/02_costmaps.md) and
[`../navigation/03_planner_controller.md`](../navigation/03_planner_controller.md)); the other
three are the safety layers detailed in this folder.

---

## 2. Design principle — put safety where it is cheap and reliable

The stack layers **software** guards (footprint check, collision monitor) on top of a simple
**firmware** guard (the watchdog). The watchdog is the most robust because it is independent of
the whole ROS stack: if the Pi 5 freezes, a node crashes, WiFi drops, or the DDS graph
partitions, the motors stop on their own within ~200 ms. Everything above it is best-effort and
depends on a healthy, correctly-configured software stack.

This mirrors the docking-research finding that robust behaviour comes from putting intelligence
where it is dependable — for docking that meant mechanical funnels; for safety it means a
firmware watchdog underneath the software guards.

---

## 3. Known false-positive: the docking self-view

The collision_monitor's reactive guard uses the lidar. During the final docking approach the
robot drives **toward** the dock panel and its own close-range geometry, which can register as an
imminent collision — a **self-view false positive** that would abort the dock. The docking
sequencer therefore disables its obstacle guard for the approach
(`obstacle_check_enabled:=false`); see [`01_collision_monitor.md`](01_collision_monitor.md) §4
and the docking docs
([`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md)).

---

## 4. Gaps — what is NOT covered (read this)

- **No hardware E-stop.** There is no physical emergency-stop button wired to cut motor power
  independently of software. Today the fastest "stop" is killing the command source (or the
  firmware watchdog timing out ~200 ms after commands cease). A hardware E-stop is a required
  addition for any un-tethered / around-people operation.
- **No safety_io / safety-rated inputs.** No safety-rated bumper, light curtain, or interlock is
  integrated.
- **2D-lidar blind spot.** Obstacles shorter than ~18 cm are invisible to the costmap **and** to
  the collision_monitor scan — no software layer can avoid them. Needs hardware (lower sensor,
  down-tilted depth camera, physical bumper). See
  [`../navigation/04_real_robot_tuning.md`](../navigation/04_real_robot_tuning.md) §3e.
- **Open-loop velocity smoothing.** The velocity_smoother clamps *commands*, not measured motion;
  it cannot detect that a wheel actually stalled.
- **Battery-dependent behaviour.** Below ~25 V at rest the drive is under-torqued and may fail to
  follow (or stop as expected); the pre-test battery rule in
  [`02_limits_and_watchdog.md`](02_limits_and_watchdog.md) is an operational mitigation, not an
  engineered safeguard.

These gaps are deliberate and tracked — the current envelope is adequate for **supervised
testing**, not for autonomous operation around people.

---

## Cross-links

- Reactive collision guard → [`01_collision_monitor.md`](01_collision_monitor.md)
- Velocity limits, watchdog, floors, battery rule → [`02_limits_and_watchdog.md`](02_limits_and_watchdog.md)
- Navigation stack → [`../navigation/`](../navigation/README.md)
- Firmware watchdog → `openamr-platform-fw` (`docs/safety/`)
