# openamrobot_sil

Base and lift software-in-the-loop layer on Linux virtual CAN. One vcan interface per real
bus, one emulated device per bus, motion in Gazebo Harmonic. Topology, node IDs, sources,
clock handling and the ROS interface: [`docs/architecture/sil-topology.md`](../../../docs/architecture/sil-topology.md).

Development evidence only. Emulators and simulation are not real-robot, mechanical,
electrical or safety acceptance. No emulator holds a safety function.

## Parts

| Part | File | What it is |
|---|---|---|
| ZLAC8015D emulator | `openamrobot_sil/zlac_gazebo.py`, node `zlac8015d_emulator` | CAN1. The vendored openamr-platform-fw emulator (NMT, heartbeat, SDO, velocity mode, link-loss object 2000h, faults). Its speed demand drives the Gazebo wheels, and 606Ch reports the Gazebo wheel speeds |
| CTL-017 emulator | `openamrobot_sil/ctl017_gazebo.py`, node `ctl017_emulator` | CAN3. The vendored emulator of the v1 object dictionary (lift command, three stop sensors, Hall count, faults) on the Gazebo lift joint, plus the permit gate |
| Daly BMS emulator | `openamrobot_sil/daly_bms.py`, node `daly_bms_emulator` | CAN2. Voltage, current, state of charge, MOS state, failure bytes. Protocol unconfirmed |
| Pack and power model (proposal) | `openamrobot_sil/battery_model.py`, `openamrobot_sil/power.py`, inside `daly_bms_emulator` | Lumped 8S LiFePO4 (OCV(SOC), series resistance, coulomb counting on the injected clock, bench charger, drive load estimate) and the power distribution (BAT and SAFE bus, RSD-60G-5, RSD-100B-24, DDR-240B-24, switch states as inputs). Parameters with source and confidence in `config/power_model.yaml`. See `docs/architecture/sil-battery-model.md` |
| Host test client | `openamrobot_sil/stm32_host.py`, node `stm32_host_client` | Plays the STM32 CAN role until the firmware runs in Renode. Test stand-in, no safety function |
| Vendored code | `openamrobot_sil/vendor/can_sil/` | openamr-platform-fw `tools/can_sil`, unmodified, pinned. See `PROVENANCE.md` there |

Every node opens its interface through `openamrobot_sil/canbus.py`, which refuses any
interface whose name does not start with `vcan`.

## Run

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to openamrobot_sil
source install/setup.bash
ros2 launch openamrobot_sil sil.launch.py
```

The launch file creates `vcan_drive`, `vcan_bms` and `vcan_aux`. That needs root or
passwordless sudo and the `vcan` kernel module (Ubuntu: `linux-modules-extra-$(uname -r)`).
Without them it prints a line starting `BLOCKED:` and stops.

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /sil/base/odom
ros2 topic pub -1 /sil/lift/permit std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /sil/lift/command std_msgs/msg/Int8 "{data: 1}"
ros2 topic pub -1 /sil/stm32/can1_silent std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /sil/drive/inject std_msgs/msg/String "{data: oc-lo}"
```

With `stm32:=renode` the host test client is not started and a Renode machine can attach to
the three interfaces. `gazebo:=false` starts the emulators alone with their internal plant models.

## Tests

```bash
# protocol tests on vcan (creates vcan_siltest)
python3 -m pytest ros2/src/openamrobot_sil/test
# the vendored platform-fw tests, unmodified
cd ros2/src/openamrobot_sil/openamrobot_sil/vendor/can_sil/tests
CAN_SIL_IFACE=vcan_siltest python3 -m pytest test_zlac8015d.py test_ctl017.py
# end to end with Gazebo
python3 -m pytest ros2/src/openamrobot_sil/test_integration
```

A machine without vcan fails these with `BLOCKED:`. `SIL_TEST_BUS=virtual` runs the protocol
tests on python-can's in-process bus instead. That checks the logic and is not vcan evidence.
CI: `.github/workflows/sil-tests.yml`.
