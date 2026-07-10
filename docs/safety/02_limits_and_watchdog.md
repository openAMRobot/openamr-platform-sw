# Velocity limits, watchdog, floors, and the battery rule

The rest of the safety envelope: the **velocity_smoother** clamp, the **firmware `/cmd_vel`
watchdog**, the measured **velocity floors** (and `min_turn_omega`), and the operational
**battery ≥ 25 V** pre-test rule. Values are quoted from
[`nav2_params.yaml`](../../ros2/src/openamrobot_nav2/config/nav2_params.yaml) and the measured
characterisation notes.

---

## 1. Velocity smoother — the command clamp

```yaml
velocity_smoother:
  smoothing_frequency: 20.0
  feedback: "OPEN_LOOP"
  scale_velocities: False
  max_velocity: [0.20, 0.0, 0.5]     # [vx, vy, vθ]
  min_velocity: [-0.20, 0.0, -0.5]
  max_accel:   [0.5, 0.0, 0.5]
  max_decel:   [-2.5, 0.0, -2.0]
  velocity_timeout: 1.0
```

- The limits are pinned to the **same** approved real-robot limits as the DWB controller
  (`max_vel_x: 0.20`, `max_vel_theta: 0.5`, `acc_lim_*: 0.5`, `decel_lim_x: -2.5`,
  `decel_lim_theta: -2.0`). Keeping them identical means the smoother never fights the controller
  — it enforces the ceiling, it doesn't reshape valid commands.
- **`feedback: "OPEN_LOOP"`** — the smoother clamps *commands* using its own model, not measured
  wheel speed. It is a limiter, not a closed-loop safety device (see the gap in
  [`00_safety_overview.md`](00_safety_overview.md) §4).
- **`velocity_timeout: 1.0`** — if commands stop arriving for 1 s the smoother ramps output to
  zero rather than latching the last command. This is a *software* stop-on-silence; the firmware
  watchdog (§2) is the independent backstop underneath it.

---

## 2. Firmware `/cmd_vel` watchdog (~200 ms)

The Teensy motor firmware runs an independent **command watchdog**: if a fresh `/cmd_vel`
message does not arrive within ~**200 ms**, it **zeros the motors**. This is the most robust
layer in the whole envelope because it does not depend on the ROS 2 stack at all — if the Pi 5
freezes, a node dies, WiFi drops, or the DDS graph partitions, the robot coasts to a stop within
~200 ms without any software action.

This watchdog is owned and documented by the firmware repo — see
**`openamr-platform-fw` (`docs/safety/`)** for the exact timeout, the zeroing behaviour, and how
it interacts with the driver fault codes. It is referenced here (not duplicated) so the software
safety picture is complete: the software layers above assume this backstop exists.

> Because Nav2 publishes `/cmd_vel` continuously while active (even a "stop" is an explicit zero
> at the smoother rate), a *healthy* stack refreshes the watchdog every cycle; a *stalled* stack
> lets it expire. That is the intended behaviour.

---

## 3. Velocity floors and `min_turn_omega`

The robot has a **minimum executable velocity** — commands below it judder or stall (stick-slip
+ coarse Hall commutation at low RPM, **not** a torque shortfall; the motor is well-sized).
Measured on the ground under load (2026-07-02):

| Axis | Reliable floor | Clean | Below the floor |
|---|---|---|---|
| Linear | **0.04 m/s** | 0.05 m/s | judder / stall |
| Angular | **0.15 rad/s** | 0.20 rad/s | stall / judder |

**Safety-relevant consequence:** a command **below** the floor is not a gentle motion — it is an
*unpredictable* one (the wheel may or may not break away). Motion loops must keep commands above
the floor:

- **Docking** applies this explicitly: a linear taper floored at **0.05 m/s**, scan rotation at
  **0.17 rad/s**, and a rotational-stiction floor **`min_turn_omega: 0.15`** (+ `turn_deadband:
  0.09`) that snaps any sub-floor yaw correction up to 0.15 rad/s or zeroes it. See
  [`../navigation/04_real_robot_tuning.md`](../navigation/04_real_robot_tuning.md) §2 and the
  docking docs.
- **Navigation** has **no** equivalent yaw floor in the DWB controller (`min_speed_theta: 0.0`);
  the coarse `yaw_goal_tolerance: 0.35` normally prevents it from chasing a sub-stiction
  correction. Treat this as a known limitation when tightening yaw tolerances — a documented gap,
  not a configured safeguard.

---

## 4. Battery ≥ 25 V before any navigation test (operational rule)

**Always confirm the 24 V pack is ≥ 25 V at rest before a navigation or avoidance test.** The
system is 2× 12 V lead-acid in series:

| At-rest voltage | State |
|---|---|
| ~25.5–26 V | Full |
| ~24 V | ~50 % |
| ≤ 23.5 V | Discharged |

**Why it's a safety/operational rule, not just a nicety:** the figures above are *at rest*;
**under motor load the voltage sags a further 1–2 V**. Below ~22 V under load the drivers
under-volt → soft torque (and a loose motor cable can drop a wheel out entirely) → the robot fails
to follow the plan and **hits obstacles the navigation was avoiding**. The recurring trap is
debugging Nav2 when the real fault is the battery. Before a test, meter the pack; if < 25 V at
rest, **recharge first** — do not conclude anything from an avoidance test on a soft bus.

Threshold table and pack details: **`openamr-platform-hw` (power documentation)**.

---

## 5. Quick checks

```bash
# smoother is clamping, not zeroing a valid command
ros2 topic hz /cmd_vel_smoothed
# final command reaches the base (watchdog stays fed while active)
ros2 topic hz /cmd_vel
# prove the motors respond independent of Nav2 (rules out low battery)
#   direct open-loop rpm test: /debug/openloop   (see the drivers docs)
```

---

## Cross-links

- The layered model + gaps (no E-stop) → [`00_safety_overview.md`](00_safety_overview.md)
- Reactive collision guard → [`01_collision_monitor.md`](01_collision_monitor.md)
- Velocity floors / sub-stiction detail → [`../navigation/04_real_robot_tuning.md`](../navigation/04_real_robot_tuning.md)
- Controller/smoother limit derivation → [`../navigation/03_planner_controller.md`](../navigation/03_planner_controller.md)
- Firmware watchdog → `openamr-platform-fw` (`docs/safety/`) · Battery thresholds → `openamr-platform-hw` (power)
