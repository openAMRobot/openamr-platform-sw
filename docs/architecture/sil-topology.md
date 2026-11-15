# Base and lift software-in-the-loop topology

Status: Proposed (14 Nov 2026; section 4.1 added 15 Nov 2026; sections 4.3 to 4.7, 5.1 and 6.1
added 15 Nov 2026). Implemented by the package `openamrobot_sil`. Topic names,
launch arguments and the configuration file in this document are new and stay Proposed until
the software lead accepts them.

This layer is development evidence only. Emulators, fixtures and Gazebo are not real-robot,
mechanical, electrical or safety acceptance. No emulator holds a safety function. The hardwired
E-stop chain is not modelled and nothing here is evidence about it.

Drive behaviour is provisional in every result. The ZLAC8015D emulator follows the vendor
manual 1.03 and the CANopen routine 1.08 as filed in the hardware repository [S16]. The V4.2
document package is not available. No value of the emulated drive was compared with a drive.

Run profiles. This layer is the base part of the `firmware_sil` profile of the audit
baseline, and its injection topics and negative tests are the `faults` profile [S15]. A
result of this layer may claim the frames on the bus, the order of exchanges, and how the
real code reacts to what the emulator sends. It may not claim that the emulator behaves like
the device. Blocked and skipped tests are not passes. No launch file sets a seed, so a
repeated run is not guaranteed to be identical.

## 1. Purpose

The STM32 base controller talks to three devices over three CAN buses. This layer puts one
emulated device on one Linux virtual CAN interface per real bus. The motion of the emulated
drive and lift happens in Gazebo Harmonic, and their feedback comes from Gazebo. A host test
client plays the CAN role of the STM32, or the platform-fw telemetry image runs in Renode on
`vcan_drive` through Renode's SocketCAN bridge (section 4.1: a firmware-in-the-loop milestone
for telemetry only).

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

`stm32:=renode` and `stm32:=renode_readonly`: the firmware binary runs in Renode on
`vcan_drive`, `vcan_bms` and `vcan_aux`. Sections 4.1 and 4.2.

### 4.1 Firmware in the loop: the telemetry image on the three buses (Proposed, 15 Nov 2026)

This is a **firmware-in-the-loop milestone for telemetry**. It is not the robot firmware and
not the whole base controller: the image only reads the drive, the lift controller and the BMS.
It is development evidence only, like the rest of this layer.

What is real:

- The firmware binary: `boards/stm32/telemetry_image` of openamr-platform-fw at commit
  `763bdd3ffe4a5a582445f72d4845d8a0eedeba3c` (head of `fw/telemetry-uros`, platform-fw PR #55,
  pinned after its hosted checks were green on that commit) [S13]: CMake preset `RenodeTest`
  for `stm32:=renode` (JSON console on USART3) and `TelemetryUrosSerialSimBench` for
  `stm32:=renode_readonly` (micro-ROS client on USART3, section 4.2). The commit and the presets
  are recorded in `launch/sil.launch.py` and `openamrobot_sil/renode_fw.py`, and the tests print
  them. `scripts/sil_renode_firmware.sh` fetches that commit, checks it, and builds the ELFs
  (`--uros` also builds the micro-ROS image and the agent); nothing in openamr-platform-fw is
  changed. At start-up the node `renode_firmware` checks the banner (console) or the first
  `/base_fw/status` message (micro-ROS): `"params":"test"`, the pinned commit, no `-dirty`.
  Anything else stops the launch (test `test_image_without_test_parameters_is_refused` runs the
  Debug build, which prints `"params":"default"`, and expects the node to fail).

What is emulated:

| Part | Here |
|---|---|
| MCU | Renode 1.16.1 (tarball and SHA-256 pinned in platform-fw `firmware-nightly.yml` at that commit, checked by the build script), `platforms/cpus/stm32h743.repl` plus platform-fw `tests/renode/stm32h723-additions.repl` and the PWR CSR1 tag, set up as platform-fw `tools/ci/renode_telemetry.py` does. Renode has no STM32H723 platform |
| CAN2, CAN3 | FDCAN2 and FDCAN3 on hubs `can2` and `can3`, joined to `vcan_bms` and `vcan_aux` by two more SocketCAN bridges. The image only receives there: it decodes the CTL-017 heartbeat, TPDO1, TPDO2 and EMCY (openamr-upperbody-fw PR #17 dictionary [S14]) and feeds the Daly frames to its decoder, which stays locked, so battery values are reported only as `"locked"`. Its transmit gate for both buses refuses every frame (platform-fw host test). The host test client keeps the CAN3 and CAN2 roles: NMT start and commands of the lift, Daly polls |
| CAN1 | FDCAN1 on a Renode CAN hub, joined to `vcan_drive` by Renode's own SocketCAN bridge (`machine CreateSocketCANBridge "can1_bridge" "vcan_drive"`, then `connector Connect can1_bridge can1`). No relay process was written: Renode 1.16.1 has the bridge (its `scripts/complex/socketcan_bridge` example), and the tests show frames crossing both ways |
| Drive | The SIL ZLAC8015D emulator on `vcan_drive`, wheels from Gazebo. It answers the image's SDO uploads and sends its heartbeat, as for the host client. The platform-fw Python drive stub is not used |
| Console (`renode`) | USART3 to a file backend; the node publishes each line and the wheel speeds |
| micro-ROS (`renode_readonly`) | USART3 on a Renode pseudo-terminal; the node starts `micro_ros_agent serial` on it (section 4.2) and republishes the wheel speeds in rad/s |
| Drive commands (`renode_readonly` only) | The host test client, as SIL receive PDOs (below) |

Read-only boundary. The image sends only SDO initiate-upload requests to node 1 (606Ch:01,
606Ch:02, 6041h, 603Fh, 2026h:05), one at a time [S8]. With `stm32:=renode` nobody commands the
drive, so the base does not move. With `stm32:=renode_readonly` the host test client sends the
drive commands and **no SDO frame at all**, so the drive's SDO channel belongs to the firmware:

- RPDO1 `200h + node`, 8 bytes: 60FFh:01 and 60FFh:02 as int32, every 20 ms.
- RPDO2 `300h + node`, 2 bytes: 6040h, the enable sequence 06h, 07h, 0Fh once.
- 2000h is not written in the run. The emulator starts with the configured link-loss time
  stored, as if it had been saved with 2010h before.

These PDO layouts are SIL test values, not a proposal and not the drive's mapping (none is
recorded). The client's CAN1 handle refuses any frame to `600h + node`.

The drive bus audit (`drive_bus_audit`) captures `vcan_drive` for the whole run to a candump
file and checks every SDO frame of node 1. It starts before Renode and the host client, and both
wait for it. A request that is not an upload, or a request while another is still outstanding
(no answer for the same object and less than 20 ms of wall time since it), is a finding. Who
sent an upload cannot be seen on the wire, so the test also compares the uploads on the bus with
the image's own `tx` counter. The test fails on any finding, and a control test then sends one
foreign SDO frame and checks that the audit sees it.

What Renode does not model, so nothing here checks it:

- Clock tree. Renode's RCC model does not keep the PLL and divider registers; the HAL derives a
  SystemCoreClock that is not the board's. A Renode hook sets the SysTick model frequency to
  every value written to SystemCoreClock, so the tick is self-consistent, not verified. Renode
  drops that hook on a machine reset; the node pauses the machine, resets it and sets the hook
  again before it runs (the platform-fw harness does the same from PR #55 on, and checks that
  the status line rate after a reset equals the rate before it).
- CAN bit timing and bit rate (500 and 250 kbit/s are not checked), acknowledge, arbitration, error
  frames, error counters and bus-off. The hub and vcan deliver every frame whole.
- UART inversion (RXINV), UART errors, baud rate and console wire time.
- Pin levels and alternate functions, GPIO, the MPU set-up, RCC_RSR reset flags (the banner
  reports `reset_cause` unknown).

Reset and fresh start. `/sil/fw/command` `reset` resets the machine inside the same Renode
instance: the sequence restarts at 0 and the boot id increases (RAM_D3 record kept). `restart`
quits Renode and starts a new instance (and, in `renode_readonly`, a new agent): the sequence
starts at 0 and the boot id is 1. The tests assert the sequence and the boot id separately. In
`renode_readonly` the client reconnects after each of them, so the first status message of a
boot carries a sequence of a few periods, not 0.

Renode commands, as `renode_fw.resc()` writes them:

```
mach create "openamr"
machine LoadPlatformDescription @platforms/cpus/stm32h743.repl
machine LoadPlatformDescription @<fw>/tests/renode/stm32h723-additions.repl
sysbus Tag <0x58024804, 0x58024807> "CSR1" 0x2000
macro reset ... sysbus LoadELF @<telemetry_image.elf> ... cpu VectorTableOffset 0x08000000
runMacro $reset
cpu PerformanceInMips 5
sysbus.usart3 CreateFileBackend @<console> true
emulation CreateCANHub "can1"
connector Connect sysbus.fdcan1 can1
machine CreateSocketCANBridge "can1_bridge" "vcan_drive"
connector Connect can1_bridge can1
emulation CreateCANHub "can2"
connector Connect sysbus.fdcan2 can2
machine CreateSocketCANBridge "can2_bridge" "vcan_bms"
connector Connect can2_bridge can2
emulation CreateCANHub "can3"
connector Connect sysbus.fdcan3 can3
machine CreateSocketCANBridge "can3_bridge" "vcan_aux"
connector Connect can3_bridge can3
include @renode/sil_renode_helpers.py
sil_follow_systick
sil_throttle_start 5
start
```

In `renode_readonly` the lines `cpu PerformanceInMips 5`, the file backend and
`sil_throttle_start` are replaced by
`emulation CreateUartPtyTerminal "uros_pty" "<work_dir>/usart3-<n>.pty" true` and
`connector Connect sysbus.usart3 uros_pty` (section 6 explains why there is no throttle).

### 4.2 micro-ROS from the firmware (`stm32:=renode_readonly`, Proposed, 15 Nov 2026)

The image of preset `TelemetryUrosSerialSimBench` publishes its snapshot as micro-ROS topics
over USART3 instead of the JSON console [S13]. **Serial micro-ROS is for simulation and the
bench only; the robot target is UDP over Ethernet** (not written; the firmware keeps the
transport behind one interface). The node `renode_firmware` starts `micro_ros_agent serial
--dev <pty> -b 115200` once Renode has created the pseudo-terminal, restarts it with every fresh
Renode start, and stops the launch if it ends on its own. The agent is built by platform-fw
`tools/ci/build_uros_agent.sh` at the pinned firmware commit (micro-ROS-Agent jazzy at a pinned
commit, Micro-XRCE-DDS-Agent v2.4.3, system Fast DDS).

| Topic (image, Proposed) | Type | Used here |
|---|---|---|
| `/base_fw/wheel_speeds` | `std_msgs/Float32MultiArray`, 606Ch:01 and :02 in drive units, NaN unless valid | republished as `/sil/fw/wheel_speeds` in rad/s (0.1 r/min per unit, SIL value); compared with Gazebo |
| `/base_fw/drive_heartbeat` | `std_msgs/String` | stopped-drive test |
| `/base_fw/tof/front`, `/base_fw/tof/rear` | `sensor_msgs/Range`, NaN unless valid | not used (no TF-Luna on the SIL buses) |
| `/base_fw/status` | `std_msgs/String`, JSON | build check (commit, params), boot id, sequence, uptime, CAN1 requests sent (`tx`, compared with the bus audit), CAN2/CAN3 frames received, lift heartbeat, state and position, `"bms":"locked"`, missed deadlines |

The image's participant is on DDS domain 0 (rclc default); the node and the tests refuse to run
with another `ROS_DOMAIN_ID`. Best effort, one message per topic per 100 ms of virtual time.

End-to-end tests (`test_integration/test_sil_renode.py`, Gazebo-integrated):

- wheel speed topic from the firmware matches the Gazebo wheel joints within 20 % at 0.2 m/s;
- a stopped drive gives `lost` and NaN wheel speeds, never 0;
- lift position reported by the firmware (TPDO1 decoded on CAN3) follows the Gazebo lift joint
  during a move to L1175: within 10 mm at the TPDO1 sample time, within 2 mm at the stop;
- battery values appear only as `"locked"` while Daly frames are decoded (`bms_dec` rises);
- the ratio of Renode virtual time to wall time is measured from the image's uptime against the
  host clock and reported (goal above 0.8, not a pass condition; section 6);
- reset and fresh start, bus audit, as in section 4.1.

There is still no command path from ROS to the firmware: the client has no subscriber and no
service.

### 4.3 Transmit audit with the sender known (Proposed, 15 Nov 2026)

A capture of a bus cannot say who sent a frame. So the read-only proof is taken at the
firmware side. Renode's `FrameSent` event of each FDCAN model fires for the frames that
peripheral transmits, and for no frame of any other bus member. The helpers write them to
`<work>/fwtx-<start>.log` (candump -L lines stamped with Renode virtual time, label `fdcan1`,
`fdcan2` or `fdcan3`). A second tap records every frame any member puts on a hub
(`hub-<start>.log`). The node feeds the firmware tap to `renode_fw.FirmwareTxAudit` and
publishes the result on `/sil/fw/tx_audit`, over every boot and every Renode start of a run.

The only frame the audit allows is an SDO initiate-upload request (40h, 8 bytes, 4 zero data
bytes) from FDCAN1 to node 1. Findings:

| Finding | Frame |
|---|---|
| `nmt` | COB-ID 000h |
| `sdo_download` | an SDO request that writes, to any node: initiate download, download segment, block download |
| `restricted_write` | a write to 6040h, 60FFh or 2000h: an SDO download of that index, or a receive PDO that the SIL drive maps to it (200h + node, 300h + node) |
| `pdo_write` | any other frame in a receive PDO range (200h to 57Fh) |
| `sdo_other` | an SDO request that is neither a write nor a plain expedited upload request |
| `not_allowed` | an upload request from another peripheral or to another node |
| `other` | everything else: SYNC, TIME, EMCY, transmit PDO ranges, heartbeat, extended, remote and CAN FD frames |

FDCAN2 and FDCAN3. The pinned image starts all three peripherals (CCCR.INIT reads 0 on each
while it runs) and counts what it receives on CAN2 and CAN3 (`c2_rx`, `c3_rx`). The audit must
show zero frames transmitted by FDCAN2 and by FDCAN3. Both Renode suites check it: with the
console image, frames put on the CAN2 and CAN3 hubs raise the image's counters by exactly the
number sent and the tap holds nothing from those peripherals; with the micro-ROS image on
`vcan_bms` and `vcan_aux`, the image reports received frames and the tap of the whole run holds
frames of FDCAN1 only.

What makes a clean audit mean something:

- The tap agrees with the image: its upload count equals the image's own `tx` counter, up to
  the requests sent after the last status.
- Controls. `test/test_readonly_audit.py` gives the audit 28 forbidden frames, one at a time,
  and requires a finding for each. In Renode, an SDO download of 6040h, an NMT start and a
  receive PDO sent by another member of the CAN1 hub are on the hub tap and not in the firmware
  tap, so the tap attributes by sender. The control of #141 that injects a foreign SDO frame
  into `vcan_drive` is kept.
- Object 2000h is the link-loss time on the drive and the read-only LiftStatus record on the
  CTL-017 [S3]. A write to it is a finding on every bus. `config/ctl017_od_pin.json` pins that
  dictionary by commit and file hash (`scripts/sil_pin_ctl017_od.py check` in CI), and
  `test/test_ctl017_od_pin.py` holds the emulator's dictionary equal to it.

Two CAN clients on `vcan_drive` (audit item SW-11 [S15]). In `renode_readonly` the image and
the host test client share the bus by design: the image reads by SDO, the client commands by SIL
receive PDOs. The command writer in this mode is the stand-in, and the firmware under test
owns no joint. The guards are: the firmware-side audit above; no NMT frame on `vcan_drive` at
all; the receive PDOs on the bus are the ones the host client counted; the client's handle
refuses SDO frames; and a second SDO owner is detected, because the bus then carries more
uploads than the firmware tap (section 4.4).

### 4.4 Negative tests (Proposed, 15 Nov 2026)

These are the `faults` profile for the firmware image, run with the console image against the
platform-fw drive stub inside Renode; bus loss and the duplicate owner also run with the
micro-ROS image on vcan. Each one checks what the image reports.
An unknown value is never reported as 0.

| Case | How | What is required |
|---|---|---|
| Bus loss | FDCAN1 is detached from its hub inside Renode (`can1_off`), then attached again | heartbeat lost only after the image's 300 ms timeout by its own clock, wheel values unknown, NaN on the topic, recovery. The image keeps sending uploads: Renode models no acknowledge, so this is not what a real bus loss does to the controller (section 4.7) |
| Stale telemetry | the drive stops answering while its heartbeat goes on | heartbeat alive, wheel values timeout or stale and unknown, recovery |
| Stale console | the emulation is paused | the node sets `stale`, publishes NaN and does not repeat the last values. Freshness is judged by receipt time: `stale_after_s`, 2 s of wall time |
| Reset mid-run | the node's `reset` | a second boot in the same Renode; the transmit audit goes on and stays clean |
| UART corruption | TF-Luna port USART2: frames with a wrong checksum, cut frames, noise | no corrupted frame becomes a range (count of accepted frames unchanged), the image counts `tf_bad`, the value goes stale and unknown, good frames recover |
| Duplicate SDO owner | a second client sends upload requests to node 1 | the bus carries more uploads than the firmware tap, the image counts the answers it did not ask for (`sdo_un`, `sdo_wo`), no foreign answer becomes a value |

Harness hazards found on the way. They are properties of the harness, and the tests avoid them:

- Bytes written into a UART model from the Renode monitor thread while the CPU runs made the
  CPU abort (`Trying to execute code outside RAM or ROM`, the same address each time) in 3 of
  9 runs. The same bytes scheduled from the machine's time domain never did: 4 runs of 400
  rounds each. The helpers inject UART bytes and CAN frames only from the time domain. This
  does not show that the image is free of a receive race on a board; see section 4.7.
- A UART pty that nobody reads stops the whole emulation after about 20 kB (section 5.1).
- A reset can cut a console line in half. The node counts it as a bad line. The test probe of
  #141 stopped receiving on such a line; it now counts it too.

### 4.5 Replay of a candump trace (Proposed, 15 Nov 2026)

`openamrobot_sil/candump_replay.py` puts a recorded drive on `vcan_drive` in place of the
emulator, so a bench capture (`candump -L`) can be dropped in later:
`SIL_REPLAY_TRACE=<file>` for `test_integration/test_sil_replay.py`. A trace cannot be played
back blindly, because the image asks for one object at a time on its own schedule:

- Frames the drive sent unasked (heartbeat, EMCY, transmit PDOs) go out at their trace time.
- SDO answers are queued per object, in trace order. A request gets the next recorded answer
  for its object. An empty queue, or the end of the trace, gives no answer.
- Frames of the master side in the trace (requests, NMT, receive PDOs) are never sent.

The test takes its expected values from the trace: the image must print wheel values that are
in the trace, in trace order, then report the heartbeat lost and the values unknown once the
trace has ended. The only trace today is `traces/zlac_synthetic_v1.candump`, written by
`scripts/sil_make_trace.py`. It has the shape of a capture and is not one.

### 4.6 Run profile and single writers (audit baseline [S15])

| Resource | Writer in `renode_readonly` | Writer in `renode` |
|---|---|---|
| Wheel joints (motion) | Gazebo DiffDrive, driven by the drive emulator | same, nobody commands |
| Drive commands on `vcan_drive` | host test client, SIL receive PDOs | none |
| SDO channel of node 1 | the firmware image | the firmware image |
| NMT, commands and permit on `vcan_aux`, polls on `vcan_bms` | host test client; the image only receives | none; the emulators run alone |
| `/base_fw/*` | the firmware image, through the agent | not published (console image) |
| `/sil/fw/wheel_speeds`, `/sil/fw/status`, `/sil/fw/tx_audit` | `renode_firmware` | same |
| `/sil/fw/odom` | `fw_odometry` | same |
| `/odom`, `odom` to `base_footprint` on `/tf`, `/joint_states` | `fw_odometry`, only with `robot_state:=true` | same |
| Other transforms on `/tf` | `robot_state_publisher` | same |

SW-08. With `robot_state:=true` the node `fw_odometry` is the one writer of `/odom`, of the
`odom` to `base_footprint` transform and of `/joint_states` (a copy of the Gazebo joint states),
so `robot_state_publisher` has a joint source and the tree is complete. `/odom` is integrated
from the wheel speeds the image reports, by receipt time, so it is coarse. While the image has
no valid wheel value nothing is published on `/odom` and `/tf`, and `/sil/fw/odom_state` says
`valid: false`: unknown is not standing still. The default is `false`, so existing runs are
unchanged. The integration test checks one publisher per topic. Not done: Nav2 or MoveIt on
top of this layer, and the frame chain of the real path (SW-10: `odom` to `base_link` from an
EKF). The stand-in follows the simulated chain.

### 4.7 What Renode does not model, and what covers it

The machine is Renode's STM32H743 description with the platform-fw additions (FDCAN3 and one
PWR register tag). It is not a model of the STM32H723. A pass here is not evidence about any of
the following. The right column names the static check or the bench measurement that does
cover the item. Bench measurements are open until someone records them.

| Not modelled | Why a pass here says nothing | Static check today | Bench measurement that covers it |
|---|---|---|---|
| STM32H723 clock tree | Renode's RCC model keeps no PLL or divider registers. The HAL derives a SystemCoreClock that is not the board's, and a harness hook makes the SysTick model follow it. The tick is self-consistent, not verified | review of the generated `SystemClock_Config` against the clock plan; the target builds of platform-fw (`STM32H723 Debug`, `Release`) | MCO or a toggled pin on a scope or counter against the planned frequency; HSE start-up; uptime drift against a reference clock |
| Pin multiplexing | No GPIO alternate function is modelled. A peripheral works in Renode whatever pin it is mapped to | review of the CubeMX `.ioc` and `HAL_*_MspInit` against the schematic, pin by pin; the platform-fw base controller I/O document | continuity and a scope on each bus and UART pin on the NUCLEO-H723ZG and on the controller board; loopback per interface |
| FDCAN bit timing | Hub and vcan deliver whole frames with no bit time. 500 kbit/s and 250 kbit/s are not checked, nor sample point or oscillator tolerance | calculation of prescaler, segments and sample point from the kernel clock, reviewed against the bit rate of each bus | bit time and sample point on a scope or analyser; error counters at zero over a long run with the real drive, BMS and lift node; a second node with known timing |
| FDCAN acknowledge, arbitration, error frames, bus-off | The hub acknowledges nothing and loses nothing. A detached FDCAN1 still "sends" | host tests of the platform-fw CANopen code for the state it reports | unplugged and shorted bus on the bench: transmit error counter, bus-off, recovery, what the image prints |
| DMA | No DMA request, no descriptor placement, no cache or MPU effect is modelled. The telemetry image receives by interrupt on CAN and on the sensor UARTs | `tools/ci/check_dma_placement.py` of platform-fw (descriptors in RAM_D2) on the base controller target; post-link guard of the image | long transfers at full rate on the board with a pattern check; overrun and half-transfer counters |
| Electrical behaviour | Levels, termination, isolation of CAN2, transceiver delay, EMC, supply dips, reset pin and brown-out do not exist here | schematic and layout review in the hardware repository | termination and common-mode on a scope; isolation test of CAN2; supply and reset tests; EMC is a laboratory test |
| UART inversion, errors, baud rate, wire time | Bytes arrive whole and at once. RXINV, framing, noise and overrun errors are not produced; a console line takes no time | host tests of the UART receive code in platform-fw (`tests/host/telemetry_image`) | each sensor on its port at its baud rate; injected framing errors; console line time at 115200 baud |
| Interrupt timing and races | Interrupts are taken at block boundaries of the emulated CPU and virtual time is not cycle accurate. The CPU abort of section 4.4 came from the harness thread; it neither shows nor excludes a race on a board | cppcheck and the sanitiser host tests of platform-fw | sustained traffic and UART bursts on the board with the stack marks and counters of the image |
| Reset flags, MPU, watchdog | `RCC_RSR` reads 0 (reset cause always unknown). The MPU set-up has no effect. The image has no watchdog | none here | reset by each cause on the board and the banner it prints |
| Timing in general | Virtual time lags wall time by a load-dependent factor (section 6) | none | any latency or period claim needs the board |

Nothing in this table is a safety argument. The hardwired safety chain is outside this layer.

## 5. Host link

The host link carries `/cmd_vel` in and odometry and status out (I8 rev 4 [S2]: `/base/status`
at 10 Hz, `/base/aux_status` at 5 Hz, a non-latching command timeout). I8 rev 4 does not
choose the transport. Two options for the emulated machine:

| Option | Renode side | Host side | Use |
|---|---|---|---|
| A. UART pty | `emulation CreateUartPtyTerminal "uros_pty" "<work_dir>/usart3-<n>.pty"` and `connector Connect sysbus.usart3 uros_pty` | micro-ROS agent in serial mode on that pty | **Built** for `stm32:=renode_readonly` (section 4.2), outbound telemetry only. Simulation and bench only |
| B. Ethernet over TAP | `emulation CreateSwitch "sw"`, `emulation CreateTap "tap_oamr" "tap"`, both connected with the MCU Ethernet peripheral | micro-ROS agent in UDP mode on the TAP address | The robot target (UDP over Ethernet). Not built: the firmware has no UDP transport yet. Renode's Ethernet model for this MCU family is not checked |

Option A carries no `/cmd_vel`: the image publishes only. With `stm32:=host` there is no host
link: the host test client subscribes to `/cmd_vel` directly and publishes `/sil/base/odom`.

### 5.1 The pty relay (Proposed, 15 Nov 2026)

A Renode UART pty that nobody reads fills its kernel buffer, and Renode then blocks in the
write. Measured with Renode 1.16.1: the whole emulation stopped after about 20 kB. A real UART
loses bytes that nobody listens to and the MCU runs on. So the agent is not given Renode's pty.
The node `renode_firmware` puts a relay between them (`openamrobot_sil/hostlink.py`): it always
drains `<work_dir>/usart3-<n>.pty`, offers `<work_dir>/usart3-<n>.link` to the agent, passes
bytes both ways, and drops and counts bytes from the image when nobody reads the link. An
agent that stops or stalls therefore never stops the image. `/sil/fw/status` carries the
relay's counters (`relay`).

Tests: host tests of the relay on a pty pair (300 kB with no reader never blocks the writer);
with the console image, a reader of a relayed pty on USART3 leaves for 45 status lines and
returns, the image keeps its sequence and `tick_miss` stays 0; with the micro-ROS image, the
integration test requires the agent to work through the relay with bytes in both directions
and no relay error. Not tested: stopping and restarting the agent during a run.

## 6. Clocks

Three clocks exist.

| Clock | Owner | Used for |
|---|---|---|
| Gazebo simulation time | Gazebo, 1 ms step, real-time factor 1.0 (`worlds/sil_flat.sdf`), on `/clock` | Physics, joint states, stamps of nodes with `use_sim_time` |
| Renode virtual time | Renode | Everything inside the firmware: its timeouts, its heartbeat, its status stamps |
| Host monotonic time | Linux | Every timer inside the emulators: heartbeat periods, the 2000h link-loss time, the CTL-017 command deadline and heartbeat consumer, the permit freshness time. The vendored emulators read `time.monotonic()` |

A vcan frame carries no time. It is delivered at once.

Rule for this phase: the clocks are not synchronised. Gazebo and the host clock run at
wall-clock rate. Renode virtual time runs at most at wall-clock rate, and here slower.

- Gazebo runs with real-time factor 1.0. A Gazebo that runs slower makes the plant slow
  against the protocol timers. Example: the CTL-017 model raises fault F02 when the Hall rate
  falls below 43 counts per second [S3]. The lift gives about 86 counts per second at
  18 mm/s, so a real-time factor below about 0.5 trips that fault.
- Renode and the telemetry image (section 4.1), up to platform-fw `fcf81f8`. The image's idle
  task spun, so Renode's virtual time advanced with the instructions it executed divided by the
  CPU's MIPS setting, not with the host clock. At the default setting it ran about 15 times slower than wall time
  on the agent's 4-core machine. The harness sets `cpu PerformanceInMips 5` and a throttle:
  every 5 ms slice of virtual time takes at least 5 ms of wall time (`sil_throttle_start`).
  So virtual time never runs ahead of wall time by more than one slice. It may lag: the
  measured ratio was 0.3 to 1.0 on the agent's machine, depending on load
  (`test_virtual_time_does_not_run_ahead_of_wall_time` prints it). The image's timeouts are
  in virtual time (SDO 40 ms, heartbeat consumer 300 ms, report period 100 ms in the
  RenodeTest build). The drive emulator answers in host time within a few milliseconds and sends
  its heartbeat every 100 ms of host time. A Renode that lags therefore sees fast answers and
  more frequent heartbeats, never late ones. A Renode that ran ahead would see answers late
  and the heartbeat as lost, which is why the throttle exists. The drive's 2000h link-loss
  time does not depend on Renode in `renode_readonly`: the host client feeds it.
  The audit's 20 ms rule rests on the same bound: a 40 ms virtual SDO timeout takes at least
  30 ms of wall time.
- From the pinned commit `763bdd3` on, the idle task executes WFI [S13]. While the core sleeps,
  Renode advances virtual time at host speed (it does not skip idle time ahead), so the image no
  longer needs a throttle to stay behind wall time. Measured in the platform-fw harness: 10 s of
  virtual time took 9.4 to 10.1 s of wall time without throttle; with the 5 ms throttle on top
  the ratio fell to 0.62 to 0.71, because the throttle's sleeps add to Renode's own pacing.
  `stm32:=renode_readonly` (micro-ROS image) therefore runs at Renode's default CPU speed
  without throttle. The node measures the ratio from the image's `up_us` against the host
  monotonic clock over 10 s and publishes it in `/sil/fw/status` (`time_ratio`); the tests
  print it. Goal: above 0.8; it is reported, not a pass condition. Measured on the development
  container (15 Nov 2026): 0.92 to 1.07 (above 1 for a few seconds after a start, when Renode
  catches up after the slow boot). `stm32:=renode` keeps the console image's former settings
  (5 MIPS, throttle; measured 0.55 in the Renode-only test): with WFI those settings are now
  the cause of a ratio below 0.8 there, and removing them is left for a follow-up because the
  console tests were tuned against them.
- Wall time against Gazebo time: Gazebo runs at real-time factor 1.0, so the 3 s of the wheel
  speed test are 3 s of wall time and about 3 s of Gazebo time, and between 0.9 s and 3 s of
  Renode virtual time.
- Consumers judge freshness by local receipt time. I8 rev 4 [S2] already requires that: the
  header stamp of the base controller is its own monotonic time and is diagnostic only. So no
  consumer needs Gazebo time and Renode time to agree.
- All timing checks in the tests are plausibility checks on thread timing. They are not timing
  measurements.

SW-07 (clocks are not shared [S15]). This stays true and is the contract of this phase:
three clocks, no synchronisation. Each clock is measured, and the two numbers are reported
separately and never combined:

- Renode virtual/wall ratio, measured inside Renode. The helpers print a `SIL_TIME` sample
  every 100 ms of virtual time (`sil_time_start`): virtual time and wall time since the start.
  The node publishes it in `/sil/fw/status` (`time`). It is a second measurement next to
  `time_ratio`, which uses the image's uptime: `time` also covers boot and does not depend on
  what the image reports.
- Gazebo real-time factor. The integration test measures it from the stamps of the Gazebo
  joint states against their arrival time. Below 0.8 the test fails with `BLOCKED:`: the plant
  would be slow against the wall-clock timers of the emulators. Above 1.05 it fails.

### 6.1 Timeout assertions the tests may make

Renode virtual time advances no faster than wall time. How strictly depends on the image:

- Console image (`stm32:=renode`): the throttle holds the rule per 5 ms slice, by
  construction, and a Renode that fell behind does not catch up.
  `test_virtual_time_never_runs_ahead_of_wall_time` requires the shortest slice to have taken at
  least its length in wall time.
- micro-ROS image (`stm32:=renode_readonly`): Renode's own time source holds the rule over
  the run, not per slice. After a slow stretch Renode may run faster than wall time for a while.

With T a time in the image (virtual) and W a wall time:

| Assertion | Allowed | Why |
|---|---|---|
| "The image reports X no earlier than T after the event that starts its timeout", measured in wall time | console image only: `not_before_wall_s(T)`, one slice of slack | T of virtual time takes at least T of wall time under the throttle. Not with the micro-ROS image, which can catch up |
| "Between two of its own stamps the image waited at least or at most T" (ages, `up_us`, counters) | yes | the image's own clock |
| "A host device that answers within W is on time for an image timeout T", for W well below T | yes with the throttle; with the micro-ROS image only as plausibility | W of wall time is at most W of virtual time under the throttle |
| "The image reports X within W of wall time" as a pass or fail of the firmware | no | Renode may lag by any factor. It can only be a liveness wait of the harness (`liveness_wait_s(T)`); a run that exceeds it is BLOCKED, not a firmware failure |
| "A host device with a wall-clock timeout W sees the image's periodic frame in time" | no, unless the measured ratio of that run is above period / W for the whole test | a lagging image sends late in wall time |
| "Two events of the image are W apart in wall time" as a period or latency measurement | no | wall time between image events is not a property of the firmware here |
| Any bound that mixes Gazebo time with Renode virtual time | no | the clocks are not synchronised. Gazebo motion is compared with what the image reports only by value, or at the sample time the image gives |

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
| `/sil/fw/console` | std_msgs/String | out of `renode_firmware` | One USART3 JSON line of the firmware image, unchanged |
| `/sil/fw/wheel_speeds` | std_msgs/Float64MultiArray | out of `renode_firmware` | [ch1, ch2] rad/s from 606Ch:01 and :02 as the image reports them (0.1 r/min per unit, the emulator's scale). NaN when the image has no valid sample, never 0 |
| `/sil/fw/status` | std_msgs/String (JSON) | out of `renode_firmware` | Pinned commit and preset, Renode starts, boots, banner, the image's tx total |
| `/sil/fw/command` | std_msgs/String | into `renode_firmware` | `reset` (machine reset in the same Renode), `restart` (fresh Renode), `pause`, `resume`, `can1_off`, `can1_on`, `can_state`. `monitor <command>` only with the node parameter `test_commands` |
| `/sil/drive_bus/audit` | std_msgs/String (JSON) | out of `drive_bus_audit` | SDO audit counters of the `vcan_drive` capture |
| `/sil/fw/tx_audit` | std_msgs/String (JSON) | out of `renode_firmware` | Firmware-side audit of every frame FDCAN1, FDCAN2 and FDCAN3 transmitted (section 4.3) |
| `/sil/fw/odom`, `/sil/fw/odom_state` | nav_msgs/Odometry, std_msgs/String (JSON) | out of `fw_odometry` | Odometry from the wheel speeds the image reports, and its valid flag and counters (section 4.6) |
| `/odom`, `/tf`, `/joint_states` | Odometry, TFMessage, JointState | out of `fw_odometry`, only with `robot_state:=true` | One writer each in the Renode modes (SW-08) |

Launch: `ros2 launch openamrobot_sil sil.launch.py`. Arguments: `stm32` (host, renode,
renode_readonly), `create_vcan`, `gazebo`, `gui`, `config_file`, `model`, `world`, and for the
Renode modes `renode`, `fw_elf`, `fw_dir` (defaults `$SIL_RENODE`, `$SIL_FW_ELF`, `$SIL_FW_DIR`
from `scripts/sil_renode_firmware.sh`), `renode_work_dir`, `capture_file`.

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
- Renode runs only the read-only telemetry image, on CAN1, CAN2 and CAN3 (section 4.1). The
  host link runs as serial micro-ROS, outbound only (section 4.2); UDP over Ethernet, the robot
  target, is not built. No firmware commands the drive, the lift or the BMS.
- The combined robot model with the real lift and arms [S5] is not used in the tests.
- Everything in section 4.7.
- A bench capture for the replay (section 4.5). The trace is synthetic.
- Stopping and restarting the micro-ROS agent during a run (section 5.1).

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
| S12 | Renode 1.16.1 portable release, `scripts/complex/socketcan_bridge` (SocketCAN bridge example) and `platforms/cpus/stm32h743.repl` of that release |
| S13 | openamr-platform-fw PR #55 (`fw/telemetry-uros`) at 763bdd3ffe4a5a582445f72d4845d8a0eedeba3c: `boards/stm32/telemetry_image/README.md`, `tools/ci/renode_uros.py`, `tools/ci/build_uros_agent.sh` |
| S14 | openamr-upperbody-fw PR #17 at 8dc7236e1eb38da68dabe063cd37aa11ba4c1faa, `docs/ctl017_object_dictionary.md` (CTL-017 object dictionary, proposal v1) |
| S11 | Vendored emulators: openamr-platform-fw `tools/can_sil` at ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d (draft PR #53). PR #54 holds the same `tools/can_sil` files at that commit |
| S15 | Digital twin phase 1 audit baseline, audits branch `audit/2026-09-28` at be17653, `digital-twin/phase1/run_profiles.md` (profiles `firmware_sil`, `faults`) and `single_writer_map.md` (SW-07, SW-08, SW-11) |
| S16 | ZLAC8015D vendor manual 1.03 and CANopen routine 1.08, as filed in openamr-platform-hw. The V4.2 package is not available |
