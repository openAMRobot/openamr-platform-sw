# Base and lift software-in-the-loop topology

Status: Proposed (14 Nov 2026). Implemented by the package `openamrobot_sil`. Topic names,
launch arguments and the configuration file in this document are new and stay Proposed until
the software lead accepts them.

This layer is development evidence only. Emulators, fixtures and Gazebo are not real-robot,
mechanical, electrical or safety acceptance. No emulator holds a safety function. The hardwired
E-stop chain is not modelled and nothing here is evidence about it.

## 1. Purpose

The STM32 base controller talks to three devices over three CAN buses. This layer puts one
emulated device on one Linux virtual CAN interface per real bus. The motion of the emulated
drive and lift happens in Gazebo Harmonic, and their feedback comes from Gazebo. The firmware
can then run in Renode and attach to the same interfaces through Renode's SocketCAN bridge.
Until the firmware is ready, a host test client plays its CAN role.

```
 /cmd_vel, /sil/lift/*            micro-ROS over UART pty or TAP (section 5)
        |                                      |
 +------v---------------+          +-----------v-----------+
 | host test client     |    or    | STM32 image in Renode |
 | (stm32:=host)        |          | (stm32:=renode)       |
 +---+--------+-------+-+          +---+--------+--------+-+
     |        |       |                |        |        |      SocketCAN
 ====+========|=======|================+========|========|===  vcan_drive  (CAN1)
 =============+=======|=========================+========|===  vcan_bms    (CAN2)
 =====================+==================================+===  vcan_aux    (CAN3)
     |        |       |
 +---v----+ +-v----+ +v--------+
 | ZLAC   | | Daly | | CTL-017 |   emulators (openamrobot_sil)
 | 8015D  | | BMS  | | lift    |
 +---+----+ +------+ +----+----+
     | wheel speed demand |  lift speed demand
     | wheel speeds back  |  lift position back
 +---v--------------------v----+
 | Gazebo Harmonic, SIL model  |   ros_gz_bridge, config/gz_bridge_sil.yaml
 +-----------------------------+
```

## 2. Buses

| Real bus | vcan interface | Real bitrate | Protocol | Devices | Emulator |
|---|---|---|---|---|---|
| CAN1 (FDCAN1) | `vcan_drive` | 500 kbit/s, provisional [S1] [S5] | CANopen, traction only [S1] | STM32 and one ZLAC8015D dual-axis driver, 2 nodes [S1] | `zlac8015d_emulator` |
| CAN2 (FDCAN2), isolated | `vcan_bms` | 250 kbit/s, proposed, to confirm on the delivered BMS [S1] | Vendor protocol, 29-bit identifiers, polling proposed at 5 Hz [S1] | STM32 and Daly BMS, 2 nodes [S1] | `daly_bms_emulator` |
| CAN3 (FDCAN3) | `vcan_aux` | 500 kbit/s [S3] [S5]. The BOM sheet says "to be confirmed" [S1] | CANopen, no safety function [S1] | STM32 and the upper-body controller CTL-017 [S1] [S3] | `ctl017_emulator` |
| Arm L, Arm R | not created here | CAN FD, per the arm kit revision [S1] | Arm driver on the Jetson, no STM32 arm master [S1] | Arm actuator chains through USB adapters [S1] | Owned by the upper-body emulator, outside this package |

Notes:

- The three base buses stay separate, as on the robot [S1]. Nothing in this package bridges
  two vcan interfaces.
- Frame size. `scripts/sil_vcan.sh` sets MTU 16 (classic CAN frame) on the three base buses,
  because all three are classic CAN [S1]. A CAN FD frame cannot be sent on them. A CAN FD bus
  (the arm buses, not created here) takes `IFACE:fd` and MTU 72.
- Guard. The emulators, the host test client and the script accept an interface only if its
  name starts with `vcan`, its device is under `/sys/devices/virtual/net/` and its link type is
  CAN. A hardware adapter renamed to a vcan name is refused.
- A vcan interface has no bit timing, no arbitration delay, no error frames and no bus-off.
  The bitrates above are documentation of the real bus. They are not enforced or tested here.
- I8 rev 4 [S2] defines the host interface of the base controller. It states no CAN node ID
  and no bitrate. Every bus value above comes from the BOM sheet and the sources named.

## 3. Node IDs and identifiers

| Bus | Device | ID | Source | State of the value |
|---|---|---|---|---|
| CAN1 | ZLAC8015D | CANopen node 1 | [S5] `zlac8015d.node_id` (vendor CANopen routine V1.07) and [S6] | Not in I8 rev 4 or the BOM. Open item M11 of the phase 0 conflict register [S9] |
| CAN1 | ZLAC8015D heartbeat | 0x701, 100 ms | [S6] proposed value D-24 | Proposed, not bench-validated. Unit of 1017h to settle by capture [S6] |
| CAN1 | ZLAC8015D link-loss time, object 2000h | 250 ms | [S6] proposed value D-24. Factory value 0 = off | Proposed, not bench-validated (M10 in [S9]) |
| CAN1 | SDO | 0x601 request, 0x581 response | CiA 301 with node 1 | Standard |
| CAN2 | Daly BMS address | 0x01 | [S7] (Daly protocol V1.0 as recorded there) and [S5] | Unconfirmed for the delivered unit |
| CAN2 | Host address | 0x40 | [S7] | Unconfirmed |
| CAN2 | Request and response | 0x18 DD 01 40 and 0x18 DD 40 01, DD = Data ID 0x90 to 0x98 | [S7] | Unconfirmed. Newer BMS firmware may use another protocol [S7] |
| CAN3 | CTL-017 | CANopen node 0x10 | [S3] [S5] | v1 baseline, a proposal |
| CAN3 | CTL-017 objects | NMT 0x000, EMCY 0x090, TPDO1 0x190, TPDO2 0x290, RPDO1 0x210, RPDO2 0x310, SDO 0x610 and 0x590, heartbeat 0x710 | [S3] | v1 baseline, a proposal |
| CAN3 | Base controller heartbeat | 0x701, consumed by CTL-017 with 300 ms | [S3] object 1016h:01 | v1 baseline, a proposal |
| CAN3 | Lift permit PDO | 0x410, 6 bytes, period 50 ms, freshness 200 ms | none | SIL test values. See section 3.1 |

The values live in `ros2/src/openamrobot_sil/config/sil.yaml`. Each line there names its source.

### 3.1 Lift permit gate

The v1 dictionary [S3] has no permit object. The permit contract proposal [S4] lists what the
owner decision fixed and what is open. Fixed: the base controller is the only producer, the
permit is false at boot, CTL-017 refuses MOVE_TO_STOP and START_HOMING without a valid permit,
it stops and drops the target when the permit is withdrawn or expires, nothing is restored or
replayed, and a heartbeat is not a permit. Open: COB-ID, field widths, the permit code, period,
timeout and the result code of a refused command.

The emulator implements the fixed rules only. It takes the open values from `sil.yaml`
(`lift.permit`). They are SIL test values. They are not proposals and no firmware should copy
them. Vectors P8 and P9 of [S4] (duplicate frames, sequence going backwards) are open choices
and are not modelled. With `lift.permit.enabled: false` the emulator is the plain v1 baseline.

## 4. Where the STM32 attaches

Two modes, selected by the launch argument `stm32`.

`stm32:=host` (default). The node `stm32_host_client` opens the three vcan interfaces and sends
what the firmware will send: drive bring-up and wheel targets on CAN1, Daly polls on CAN2, NMT
start, heartbeat, lift commands, keep-alive and permit frames on CAN3. It is a test stand-in.
It copies no firmware logic, has no command timeout and takes no stop decision.

`stm32:=renode`. Nothing takes the STM32 role. A Renode machine attaches to each interface
with a SocketCAN bridge and a CAN hub. Sketch, not run in this repository:

```
emulation CreateSocketCANBridge "bridge_drive" "vcan_drive"
emulation CreateCANHub "can1"
connector Connect sysbus.fdcan1 can1
connector Connect bridge_drive can1

emulation CreateSocketCANBridge "bridge_bms" "vcan_bms"
emulation CreateCANHub "can2"
connector Connect sysbus.fdcan2 can2
connector Connect bridge_bms can2

emulation CreateSocketCANBridge "bridge_aux" "vcan_aux"
emulation CreateCANHub "can3"
connector Connect sysbus.fdcan3 can3
connector Connect bridge_aux can3
```

Facts from the firmware repository [S8]: the telemetry image boots in Renode on the STM32H743
platform description plus an additions file, because Renode has no STM32H723 platform. That
additions file adds FDCAN3. The image already uses `emulation CreateCANHub "can1"` with
`sysbus.fdcan1`. It is read-only on CAN1 today: it sends no targets and does not enable the
drive. So with `stm32:=renode` the base will not move until the owner's drive code exists.
The bridge command names above follow the Renode documentation and must be checked against the
Renode version that the firmware CI pins.

## 5. Host link

The host link carries `/cmd_vel` in and odometry and status out (I8 rev 4 [S2]: `/base/status`
at 10 Hz, `/base/aux_status` at 5 Hz, a non-latching command timeout). I8 rev 4 does not
choose the transport. Two options for the emulated machine:

| Option | Renode side | Host side | Use |
|---|---|---|---|
| A. UART pty | `emulation CreateUartPtyTerminal "hostlink" "/tmp/openamr_stm32_uart"` and `connector Connect sysbus.<uart> hostlink` | micro-ROS agent in serial mode on that pty | First choice. The telemetry image already runs a UART console in Renode [S8] |
| B. Ethernet over TAP | `emulation CreateSwitch "sw"`, `emulation CreateTap "tap_oamr" "tap"`, both connected with the MCU Ethernet peripheral | micro-ROS agent in UDP mode on the TAP address | When the firmware has an Ethernet transport. Renode's Ethernet model for this MCU family is not checked |

Neither option is built in this PR. With `stm32:=host` there is no host link: the host test
client subscribes to `/cmd_vel` directly and publishes `/sil/base/odom`.

## 6. Clocks

Three clocks exist.

| Clock | Owner | Used for |
|---|---|---|
| Gazebo simulation time | Gazebo, 1 ms step, real-time factor 1.0 (`worlds/sil_flat.sdf`), on `/clock` | Physics, joint states, stamps of nodes with `use_sim_time` |
| Renode virtual time | Renode | Everything inside the firmware: its timeouts, its heartbeat, its status stamps |
| Host monotonic time | Linux | Every timer inside the emulators: heartbeat periods, the 2000h link-loss time, the CTL-017 command deadline and heartbeat consumer, the permit freshness time. The vendored emulators read `time.monotonic()` |

A vcan frame carries no time. It is delivered at once.

Rule for this phase: all three clocks run at wall-clock rate and are not synchronised.

- Gazebo runs with real-time factor 1.0. A Gazebo that runs slower makes the plant slow
  against the protocol timers. Example: the CTL-017 model raises fault F02 when the Hall rate
  falls below 43 counts per second [S3]. The lift gives about 86 counts per second at
  18 mm/s, so a real-time factor below about 0.5 trips that fault.
- Renode must hold its virtual time to host time. This is to be confirmed on the pinned
  Renode version. An emulated machine that runs slower than real time will see the drive
  link-loss time and the lift deadlines expire early.
- Consumers judge freshness by local receipt time. I8 rev 4 [S2] already requires that: the
  header stamp of the base controller is its own monotonic time and is diagnostic only. So no
  consumer needs Gazebo time and Renode time to agree.
- All timing checks in the tests are plausibility checks on thread timing. They are not timing
  measurements.

Later option, not built: lock-step. Gazebo is stepped by a fixed number of iterations per
Renode time quantum, and the emulators take their time from the same source. That needs an
injectable clock in the vendored emulators, which is a change in openamr-platform-fw first.

## 7. Data flow and ROS interface (Proposed)

Drive. The ZLAC8015D emulator ramps its speed demand as the vendored model does. The node
converts both wheel speeds to a body twist and publishes it for the Gazebo DiffDrive plugin of
the unchanged base model. The conversion uses the wheel radius 0.100 m and the wheel
separation 0.400 m of that plugin (a test checks they agree). The plugin's own acceleration
limits apply on top. Object 606Ch reports the wheel joint speeds that Gazebo measures. 60FFh
takes whole r/min, so 0.2 m/s becomes 19 r/min and 0.199 m/s.

Lift. The CTL-017 emulator sends the speed of its motor model to a velocity controller on
`lift_joint`. The joint position sets the Hall count and the three stop sensors. Joint
positions 0, 0.175 m and 0.350 m are the stops L1000, L1175 and L1350 [S3]. The lift in the
default model is a stand-in with the joint name and limits of the combined model in
openamr-upperbody-sw [S5]: 0 to 0.350 m, 0.018 m/s, 800 N. The launch argument `model` takes
another xacro.

BMS. Static values with injection (pack defaults from [S10]). No link to Gazebo.

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | into the host test client | Base command. It is not bridged to Gazebo |
| `/sil/base/odom` | nav_msgs/Odometry | out of the host test client | Odometry from the wheel speeds read over CAN1 |
| `/sil/gz/odom` | nav_msgs/Odometry | out of Gazebo | DiffDrive plugin odometry, a reference |
| `/sil/gz/joint_states` | sensor_msgs/JointState | out of Gazebo | Wheel and lift joints, 100 Hz |
| `/sil/drive/gz_cmd_vel`, `/sil/lift/gz_cmd_vel` | Twist, std_msgs/Float64 | emulators to Gazebo | Speed demands |
| `/sil/drive/emulator_state`, `/sil/lift/emulator_state`, `/sil/bms/emulator_state` | std_msgs/String (JSON) | out of the emulators | Internal state for tests |
| `/sil/drive/inject`, `/sil/lift/inject`, `/sil/bms/inject` | std_msgs/String | into the emulators | Fault injection |
| `/sil/drive/status`, `/sil/lift/status`, `/sil/bms/status` | std_msgs/String (JSON) | out of the host test client | What the client reads over CAN |
| `/sil/bms/battery_state` | sensor_msgs/BatteryState | out of the host test client | Voltage, current, state of charge |
| `/sil/stm32/can1_silent` | std_msgs/Bool | into the host test client | true: send nothing on CAN1 (link-loss test) |
| `/sil/lift/command` | std_msgs/Int8 | into the host test client | 0, 1, 2: move to stop. -1 stop, -2 homing, -3 clear fault |
| `/sil/lift/permit`, `/sil/lift/permit_silent` | std_msgs/Bool | into the host test client | Permit state, and "send no permit frames" |

Launch: `ros2 launch openamrobot_sil sil.launch.py`. Arguments: `stm32` (host, renode),
`create_vcan`, `gazebo`, `gui`, `config_file`, `model`, `world`.

## 8. Not modelled and open

- E-stop chain, contactors, brakes, motor power removal and charge inhibit. Out of scope.
- CAN physical layer: bit timing, termination, error frames, bus-off, bus load.
- ZLAC8015D: PDOs (the vendor mapping is not recorded), position objects, the brake output,
  axis order and sign (Q-02 in [S5]), what the drive does when 2000h expires [S6]. The
  vendored emulator ramps to zero.
- Drive torque, current and wheel slip. The Gazebo wheels follow a speed command.
- CTL-017: the open permit choices of section 3.1, and the firmware alignment items of
  the emulator alignment note in [S4].
- Daly: the protocol itself is unconfirmed [S7]. Cell voltage frames 0x95 to 0x97 get no answer.
- Renode attachment and the host link are described, not run.
- The combined robot model with the real lift and arms [S5] is not used in the tests.

## 9. Sources

| Ref | Source |
|---|---|
| S1 | BOM Issue 7.15, sheet CAN buses. Copy: audits branch `audit/2026-09-28`, `night-shift/inputs/digital-twin/bom/CAN_buses.csv` |
| S2 | I8 Base Controller Interface Proposal rev 4 (10 Nov 2026). Copy: same branch, `night-shift/inputs/digital-twin/docs/` |
| S3 | CTL-017 v1 object dictionary, openamr-upperbody-fw PR #15, `docs/ctl017_object_dictionary.md` at 1a68c50. Read through the vendored `ctl017_od_v1.py` |
| S4 | openamr-upperbody-fw branch `night-shift/fw2-ctl017-infra-2026-11-14` at 8dc7236: `docs/ctl017_permit_contract.md`, `docs/ctl017_emulator_alignment.md` |
| S5 | Actuator ledger, openamr-upperbody-sw PR #65 at 8c4c548, `ros2/src/openamrobot_upperbody_description/config/actuator_ledger.yaml`, and `urdf/combined_robot.xacro` of the same commit |
| S6 | openamr-platform-fw `docs/architecture/can1-drive-link.md` (PR #49, read at fcf81f8) and the ZLAC8015D PRs #38 and #49 |
| S7 | openamr-platform-fw PR #51 at fcf81f8, `firmware/telemetry/include/oamr/daly_rx.h` |
| S8 | openamr-platform-fw PR #51 at fcf81f8, `tools/ci/renode_telemetry.py`, `tests/renode/stm32h723-additions.repl` |
| S9 | Digital twin phase 0 conflict register, audits branch `audit/2026-09-28`, `digital-twin/phase0/conflict_register.md` (items M10, M11) |
| S10 | E-01 Safety and Power Distribution Baseline rev1.8: 8S LiFePO4 pack, 25.6 V nominal bus, 84 Ah |
| S11 | Vendored emulators: openamr-platform-fw `tools/can_sil` at ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d (draft PR #53). PR #54 holds the same `tools/can_sil` files at that commit |
