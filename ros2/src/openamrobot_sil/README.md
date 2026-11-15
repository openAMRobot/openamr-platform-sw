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
| Host test client | `openamrobot_sil/stm32_host.py`, node `stm32_host_client` | Plays the STM32 CAN role until the firmware runs in Renode. Test stand-in, no safety function |
| Firmware in Renode | `openamrobot_sil/renode_fw.py`, node `renode_firmware`, `renode/sil_renode_helpers.py` | `stm32:=renode` and `stm32:=renode_readonly`: the openamr-platform-fw telemetry image (pinned commit) in Renode 1.16.1, FDCAN1, FDCAN2 and FDCAN3 on `vcan_drive`, `vcan_bms` and `vcan_aux` through Renode's SocketCAN bridges. `renode`: preset RenodeTest, publishes the USART3 console and the wheel speeds. `renode_readonly`: preset TelemetryUrosSerialSimBench, starts the micro-ROS agent on USART3 (serial: simulation and bench only) and follows the image's `/base_fw/*` topics. Read-only image: telemetry only |
| Drive bus audit | node `drive_bus_audit` | Captures `vcan_drive` (candump file) and checks that SDO requests to the drive are uploads, one at a time. Test fixture |
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

`gazebo:=false` starts the emulators alone with their internal plant models.

### Firmware in the loop (telemetry image in Renode)

```bash
# fetch platform-fw at the pinned commit, build presets RenodeTest and TelemetryUrosSerialSimBench
# and the micro-ROS agent (ROS 2 Jazzy sourced), fetch Renode 1.16.1 (SHA-256 checked)
bash ros2/src/openamrobot_sil/scripts/sil_renode_firmware.sh --uros .sil-renode
set -a; . .sil-renode/sil_renode.env; set +a
ros2 launch openamrobot_sil sil.launch.py stm32:=renode_readonly
ros2 topic echo /sil/fw/wheel_speeds
ros2 topic echo /base_fw/status   # the image's own status topic (DDS domain 0)
ros2 topic pub -1 /sil/fw/command std_msgs/msg/String "{data: reset}"
```

The build needs `arm-none-eabi-gcc` (the versions pinned in platform-fw `firmware-nightly.yml`),
CMake, Ninja and access to github.com; `--uros` also needs `libstdc++-arm-none-eabi-newlib`,
colcon, catkin_pkg, empy 3, lark and ROS 2 Jazzy. `stm32:=renode` runs the image with no host
client, so nobody commands the drive. `stm32:=renode_readonly` runs the micro-ROS image and adds
the host test client, which then sends the drive commands as SIL receive PDOs and no SDO frame,
and plays the CAN3 and CAN2 roles. The image's participant is on DDS domain 0: run with
`ROS_DOMAIN_ID` unset or 0. What is real and what is emulated: sections 4.1 and 4.2 of the
topology document.

## Tests

```bash
# protocol tests on vcan (creates vcan_siltest)
python3 -m pytest ros2/src/openamrobot_sil/test
# the vendored platform-fw tests, unmodified
cd ros2/src/openamrobot_sil/openamrobot_sil/vendor/can_sil/tests
CAN_SIL_IFACE=vcan_siltest python3 -m pytest test_zlac8015d.py test_ctl017.py
# end to end with Gazebo and the host test client
python3 -m pytest ros2/src/openamrobot_sil/test_integration/test_sil_gazebo.py
# Renode only (no vcan, no Gazebo); needs the variables of sil_renode.env (--default-too build)
cd ros2/src/openamrobot_sil/test_renode && python3 -m pytest test_renode_node.py test_renode_uros_node.py
# end to end with Gazebo and the firmware image in Renode
python3 -m pytest ros2/src/openamrobot_sil/test_integration/test_sil_renode.py
```

A machine without vcan fails these with `BLOCKED:`. `SIL_TEST_BUS=virtual` runs the protocol
tests on python-can's in-process bus instead. That checks the logic and is not vcan evidence.
CI: `.github/workflows/sil-tests.yml`.
