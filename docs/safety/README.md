# Safety — engineering documentation

This folder is the engineering record for the **motion-safety envelope** of the OpenAMRobot
platform: what stops or slows the robot, the limits its commands are clamped to, and — just as
important — the **gaps** (there is no hardware E-stop / safety_io yet).

This is an **honest** account. The current safety is a layered *software + firmware* envelope,
not a certified functional-safety system. Read [`00_safety_overview.md`](00_safety_overview.md)
before relying on any single layer.

For 2.0, the hardware E-stop chain is defined in E-01 rev1.8 (team document, 8 Nov 2026; page proposed in openamr-platform-hw#27): two dual-channel E-stops, a Pilz PNOZ s3 safety relay and two Gigavac GX11CAC contactors. The chain is hardware only, with no software and no bus in the safety path. The base controller only observes it (relay output and E-stop spare contacts, as diagnostics) and adds a secondary software inhibit; how those observations reach ROS is proposed in openamrobot-interfaces#30, not decided. Not built or tested yet; until then the gaps in `00_safety_overview.md` stand.

---

## Layout

```
docs/safety/
├── README.md                    ← this file (index)
│
├── 00_safety_overview.md        the layered model, what each layer guarantees, the GAPS
├── 01_collision_monitor.md      Nav2 collision_monitor: zones, approach behaviour, self-view caveat
└── 02_limits_and_watchdog.md    velocity_smoother limits, firmware 200 ms watchdog,
                                 velocity floors + min_turn_omega, the battery ≥25 V rule
```

---

## How to read

| Goal | Read |
|---|---|
| **Understand the whole safety envelope + what it does NOT cover** | `00_safety_overview.md` |
| **How the robot avoids/slows for obstacles reactively** | `01_collision_monitor.md` |
| **What clamps velocity, the firmware watchdog, the pre-test rules** | `02_limits_and_watchdog.md` |

---

## Running the diagnostic commands (env prelude)

Every bare `ros2 …` snippet in this series assumes a sourced ROS 2 + workspace overlay and the
project's DDS settings. Run this block **once per shell**, before any command below — it is not
repeated in each doc:

```bash
# ROS 2 + the OpenAMRobot workspace overlays
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash
source ~/openamr-platform-sw/ros2/install/setup.bash
# Project DDS settings — MUST match every node on the graph (robot uses CycloneDDS, domain 0)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

A mismatched `RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID` makes nodes report `active` while topics
stay empty (see [`../real_robot/`](../real_robot/README.md) for the DDS notes).

---

## Related

- Navigation stack these layers protect → [`../navigation/`](../navigation/README.md)
- Docking self-view obstacle caveat → [`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md)
- Firmware motion safety (the `/cmd_vel` watchdog) → `openamr-platform-fw` (`docs/safety/`)
- Power / battery thresholds → `openamr-platform-hw` (power documentation)
