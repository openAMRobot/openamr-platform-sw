# Base and lift software-in-the-loop topology

Status: Proposed (14 Nov 2026; section 4.1 added 15 Nov 2026). Implemented by the package `openamrobot_sil`. Topic names,
launch arguments and the configuration file in this document are new and stay Proposed until
the software lead accepts them.

This layer is development evidence only. Emulators, fixtures and Gazebo are not real-robot,
mechanical, electrical or safety acceptance. No emulator holds a safety function. The hardwired
E-stop chain is not modelled and nothing here is evidence about it.

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
`vcan_drive`. Section 4.1.

### 4.1 Firmware in the loop: the telemetry image on the drive bus (Proposed, 15 Nov 2026)

This is a **firmware-in-the-loop milestone for telemetry**. It is not the robot firmware and
not the whole base controller: the image only reads the drive. It is development evidence
only, like the rest of this layer.

What is real:

- The firmware binary: `boards/stm32/telemetry_image` of openamr-platform-fw at commit
  `fcf81f8e2fe2fa9000685ab233907adbfadeec2a`, CMake preset `RenodeTest` [S8]. The commit and
  the preset are recorded in `launch/sil.launch.py` and `openamrobot_sil/renode_fw.py`, and the
  tests print them. `scripts/sil_renode_firmware.sh` fetches that commit, checks it, and builds
  the ELF; nothing in openamr-platform-fw is changed. At start-up the node `renode_firmware`
  checks the banner: `"params":"test"`, the pinned commit, no `-dirty`. Any other banner stops
  the launch (test `test_image_without_test_parameters_is_refused` runs the Debug build, which
  prints `"params":"default"`, and expects the node to fail).

What is emulated:

| Part | Here |
|---|---|
| MCU | Renode 1.16.1 (tarball and SHA-256 pinned in platform-fw `firmware-nightly.yml` at that commit, checked by the build script), `platforms/cpus/stm32h743.repl` plus platform-fw `tests/renode/stm32h723-additions.repl` and the PWR CSR1 tag, set up as platform-fw `tools/ci/renode_telemetry.py` does. Renode has no STM32H723 platform |
| CAN1 | FDCAN1 on a Renode CAN hub, joined to `vcan_drive` by Renode's own SocketCAN bridge (`machine CreateSocketCANBridge "can1_bridge" "vcan_drive"`, then `connector Connect can1_bridge can1`). No relay process was written: Renode 1.16.1 has the bridge (its `scripts/complex/socketcan_bridge` example), and the tests show frames crossing both ways |
| Drive | The SIL ZLAC8015D emulator on `vcan_drive`, wheels from Gazebo. It answers the image's SDO uploads and sends its heartbeat, as for the host client. The platform-fw Python drive stub is not used |
| Console | USART3 to a file backend; the node publishes each line and the wheel speeds |
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
  again before it runs (the platform-fw harness does not, so its boot after a reset runs with a
  fast tick).
- CAN bit timing and bit rate (500 kbit/s is not checked), acknowledge, arbitration, error
  frames, error counters and bus-off. The hub and vcan deliver every frame whole.
- UART inversion (RXINV), UART errors, baud rate and console wire time.
- Pin levels and alternate functions, GPIO, the MPU set-up, RCC_RSR reset flags (the banner
  reports `reset_cause` unknown).

Reset and fresh start. `/sil/fw/command` `reset` resets the machine inside the same Renode
instance: the sequence restarts at 0 and the boot id increases (RAM_D3 record kept). `restart`
quits Renode and starts a new instance: the sequence starts at 0 and the boot id is 1. The tests
assert the sequence and the boot id separately.

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
include @renode/sil_renode_helpers.py
sil_follow_systick
sil_throttle_start 5
start
```

Not done: CAN2 and CAN3 stay with the host test client in `renode_readonly` (the image does not
use FDCAN2 or FDCAN3), and there is no host link.

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

## 6. Clocks (contract, Proposed)

Status: Proposed under decision D2 of the digital twin plan (15 Nov 2026). Owner of the contract:
software lead. Firmware timer semantics stay with the platform lead. Section 6.6 records what
is implemented.

### 6.1 Domains

| Domain | Source | Runs during a Gazebo pause |
|---|---|---|
| `gazebo` | Gazebo simulation time on `/clock`, step 1 ms (`fast`) or 1/1500 s (`fine`) | no |
| `renode` | Renode virtual time of the emulated MCU | yes, unless Renode is paused too |
| `ros` | ROS time of a node with `use_sim_time:=true`; equals `gazebo` in every SIL launch | no |
| `steady` | host monotonic time (`time.monotonic()`, `ClockType.STEADY_TIME`) | yes |

A vcan frame carries no time. It is delivered at once; an injected delay is counted in the
receiver's domain.

### 6.2 Rules

- **C1** Device timers follow the plant. Every timer that models device behaviour (protocol
  timeouts, heartbeat periods and consumers, the 2000h link-loss time, CTL-017 deadlines, Hall
  rate windows, permit freshness, plant integration steps) runs on `gazebo` when the plant is
  Gazebo, and on `steady` when the plant is internal (no Gazebo). The Linux stand-in for the base
  controller (`stm32:=host`) is a device in this sense.
- **C2** Firmware in Renode keeps Renode virtual time. Renode is held to host time until
  lock-step exists. Every run reports the Renode virtual-to-host ratio next to the Gazebo
  real-time factor, and the skew between Renode time and Gazebo time over the run. The skew bound
  is set in WS4 with the platform lead; until then it is reported, not enforced.
- **C3** Firmware watchdog semantics are not simulation parameters. No SIL profile lengthens a
  firmware or emulator timeout to make a slow runner pass. A slow runner is handled by C1 pacing
  and by the scene's real-time-factor guard (scenario BLOCKED), never by a longer timeout.
- **C4** ROS nodes use ROS time. Node timers, stamps and freshness checks inside ROS nodes use
  `ros`; a node does not pace a device timer on its own steady clock.
- **C5** `steady` is for process health only: thread liveness, node readiness, launch and test
  deadlines, and blocking receive calls as an upper bound of waiting. A `steady` check never raises
  a device fault; it reports "simulation stalled" separately, with the wall time and the last
  simulation time.
- **C6** Freshness is judged by receipt time in the receiver's own domain (I8 rev 4 [S2]: the base
  controller's header stamp is diagnostic only). No consumer needs two domains to agree.

**C2 in practice: Renode and the telemetry image** (section 4.1, from #141, measured 15 Nov 2026).
The image's idle task spins, so Renode's virtual time advances with the instructions it executes
divided by the CPU's MIPS setting, not with the host clock. At the default setting it ran about 15
times slower than wall time on the agent's 4-core machine. The harness sets
`cpu PerformanceInMips 5` and a throttle: every 5 ms slice of virtual time takes at least 5 ms of
wall time (`sil_throttle_start`). So virtual time never runs ahead of wall time by more than one
slice. It may lag: the measured ratio was 0.31 to 0.35 in CI and 0.39 to 0.46 locally
(`test_virtual_time_does_not_run_ahead_of_wall_time` prints it). The image's timeouts are in
virtual time (SDO 40 ms, heartbeat consumer 300 ms, report period 100 ms in the RenodeTest
build). The drive emulator answers within a few milliseconds of host time and its heartbeat
producer still runs on host time (vendored node loop, openamr-platform-fw #57 and #58). A
Renode that lags therefore sees fast answers and more frequent heartbeats, never late ones; a
Renode that ran ahead would see answers late and the heartbeat as lost, which is why the
throttle exists. In `renode_readonly` the drive's 2000h link-loss time is fed by the host
client's receive PDOs, not by Renode. The audit's 20 ms rule rests on the throttle bound: a
40 ms virtual SDO timeout takes at least 30 ms of wall time. The Renode-to-Gazebo skew is not
reported yet (C2 asks for it). All timing checks of the Renode tests are plausibility checks on
thread timing, not timing measurements.

### 6.3 Events and their gates (D3)

| Event | Required behaviour | Gate test |
|---|---|---|
| Gazebo paused 5 s wall time, then resumed | `gazebo` timers freeze, so no device timeout fires in the pause. A pause longer than 1 s wall time (proposal) counts as a stall under C5: on resume every drive target is zero, the lift permit is invalid until a new permit frame arrives, and motion needs a new command sent after the resume | drive, enable and lift permit do not survive; nothing re-arms by itself |
| Simulation time reset or moved backward | every emulated device takes it as a power cycle: boot-up frame, targets and enable dropped, permit invalid, lift unhomed unless at a stop sensor | same |
| Renode restart | the firmware boots fresh; devices see heartbeat loss, then boot-up; nothing from before the restart is valid | same |
| micro-ROS agent restart | the firmware keeps its own timeouts (`renode`) and stops on link loss as its production code does | same |
| CAN frames delayed by an injected time | the receiver judges the age by its receipt time (C6); a late frame is not treated as fresh | stale command refused |
| `/clock` jumps forward (one step longer than 1 s simulation time, proposal) | not a new generation. Device timers see the elapsed time, so a timeout that expires in it acts as on the device (C3): a stamp, permit or keep-alive from before the jump is stale, the lift with the permit gate halts on the expired permit without a fault, without the gate it raises F07, and the drive's wheel feedback is stale. The coordinator reports the jump ("simulation time jumped forward", with both times; not a device fault) next to any fault it lets expire. Nothing re-arms by itself | stale permit or feedback never fresh, old command not executed again, jump reported |
| the same `/clock` time twice | not an advance and not a reset: no generation change, no stall while the time still advances; a paused Gazebo that republishes its frozen time still stalls after 1 s | no generation change, no false stall |
| `/clock` delayed on `steady` time while simulation time is continuous | C5 only: a gap up to the 1 s stall bound changes nothing; a longer gap is one stall (report with wall time and last simulation time), handled as a pause; the next time resumes without a jump report | stall exactly above the bound |
| before the first `/clock` | no device timer runs and no command is accepted: CTL-017 stays in INIT, the host stand-in sends nothing; the coordinator declares no stall | no frame, no command accepted |
| `/clock` lost, then back | stall reported once; a command accepted during the pause, or held by the stand-in, does not move an actuator after the return; the drive comes back with a zero target | nothing moves from a command before the stall |
| frames queued before a reset, delivered after it | a frame received after a reset meets the power-cycled node (PRE-OPERATIONAL, then INIT) and is not executed; frames carry no time or generation, so a frame delayed in transport past the node's restart cannot be told apart (gap) | queued frames rejected |

### 6.4 Inventory

`ros2/src/openamrobot_sil/config/clock_inventory.yaml` lists every clock read and timer in this
package, the domain it runs on now, the domain these rules assign, and where the change has to
happen. `test/test_clock_inventory.py` fails when a clock read is added or removed without an
inventory change.

### 6.5 Clock interface

`openamrobot_sil/sim_time.py` is the interface every device model takes instead of reading host
time: `now()`, `generation` (increases on every reset or backward jump, so a command, enable or
permit stamped earlier is never fresh again), `snapshot()` (time and generation read together) and
`paused`. `FedClock` keeps time and generation in one tuple that a `/clock` update replaces in a
single assignment, so a `Stamp` taken on another thread never pairs a pre-reset time with the new
generation, or the reverse. `FedClock` is fed from `/clock`;
`ManualClock` drives tests; `SteadyClock` is host time for an internal plant and for health checks.
`Stamp` and `is_fresh` judge freshness by receipt time in the receiver's domain (C6); a stamp
taken before a stall (`stalls` counter) is not fresh after it either. `ClockCoordinator` feeds a
`FedClock` and declares the stall on `steady` time (C5); `ClockWatch` lets a device model see a
reset or a stall at its next step. A simulated
communication deadline (a device timeout in its own domain) and a host communication failure (a
process that stopped, seen in `steady`) are separate checks and never share a timer.

### 6.6 Current state (17 Nov 2026)

Implemented in `openamrobot_sil` (inventory: 3 sites `to_change`, all in vendored code):

- **One clock per process.** Every CAN node gets `use_sim_time` from the launch (`gazebo:=true`).
  `nodes/common.py` `NodeClock` feeds one `FedClock` from `/clock` through a `ClockCoordinator`.
  A steady health timer (100 ms, C5) declares a stall when `/clock` has not advanced for longer
  than 1 s wall time (proposal), logs "simulation stalled" with the wall time and the last
  simulation time, holds the Gazebo plant command at zero, and never raises a device fault. It
  also reports a stopped emulator or stand-in thread. Without Gazebo the clock is `steady` (C1).
- **CTL-017.** `GazeboCtl017` overrides the vendored `_now_ms()` hook with the given clock, so the
  Hall rate window, stall, move deadline, keep-alive, heartbeat consumer, standstill and permit
  freshness run on `gazebo`. No timeout value and no vendored file changed (C3). A backward time
  is a power cycle (boot-up frame, PRE-OPERATIONAL, targets and permit dropped, unhomed unless at
  a stop sensor). A stall is the vendored STOP command plus an invalid permit, at the stall and
  again at the resume, so neither a command nor a permit frame from before or during the pause
  carries over; the host keep-alive of the old command does not restart motion.
- **ZLAC8015D.** `GazeboZlac8015d` judges the Gazebo wheel feedback by receipt time on the given
  clock (C6). A reset or a stall is the vendored `power_cycle()`: targets zero, drive disabled,
  boot-up frame.
- **Host stand-in** (`stm32_host.py`): loop pacing, odometry time, keep-alive, heartbeat, NMT
  retry, permit period and the BMS answer timeout run on the given clock; host sleeps are only
  an upper bound of one wait (at most 2 ms). After a reset or a stall it drops the lift command
  and the drive twist and brings the drive up again with a zero target.
- **Node timers.** Device-step and publish timers run on ROS time (`device_timer`, C4) and stop
  in a pause; only the health timer runs on `steady` (`health_timer`, C5).
- Gate tests: `test/test_clock_gates.py` runs CTL-017, the lift joint and the stand-in in
  lock-step on a fed clock with a manual wall clock (pause, reset, restart, delayed permit frames,
  runner speed, forward jump, duplicate times, first clock, clock loss, frames queued across a
  reset) and the drive on a manual or coordinated clock; `test/test_sim_time.py` covers the
  coordinator for jumps, duplicates and delayed `/clock`; `test_integration/test_sil_gazebo.py`
  pauses Gazebo 5 s wall time and resets it through `gz service` on `/world/sil_flat/control`.
- **Health callback: simulation-only hold, not a second command authority.** The steady health
  timer of `NodeClock` (C5) logs reports and, once per stall, calls the node's `hold_plant`, which
  publishes a zero command to the Gazebo plant topic (`/sil/drive/gz_cmd_vel`,
  `/sil/lift/gz_cmd_vel`). That is its only effect: it sends no CAN frame and does not change the
  emulated device; the devices drop motion at their own next step through `ClockWatch`.
  `test_node_clock.py` checks this for the drive and the lift node.
- Forward jumps: `ClockCoordinator` counts a forward step above `JUMP_BOUND_S` (1 s simulation
  time, proposal, the stall bound value) and `NodeClock` logs `jump_report()`. Report only: no
  device decision reads it and no timeout changed (C3).
- Open gap, tracked, tests `xfail` (strict): #159 frames from before a reset act after it (the
  ZLAC8015D handles a frame from its receive queue after the reset power cycle; an RPDO1 delayed
  in transport past the CTL-017 restart runs, since RPDO1 has no producer epoch). A platform lead
  decision.
- First clock, ZLAC8015D (#160, 17 Nov 2026, Proposed, platform lead to confirm): as the CTL-017
  node, `GazeboZlac8015d` is not powered before the first `/clock`: no boot-up, no heartbeat, every
  received frame dropped, no step; at the first clock time it powers up through the vendored
  `power_cycle()`. Wrapper only, no vendored file changed.

Not yet on `gazebo` (openamr-platform-fw #57 first, then a re-vendor):
`canopen_common.py` (node tick and `dt`, heartbeat producer period, SDO client timeout) and
`zlac8015d_emulator.py` (receipt time and the 2000h link-loss time). Consequences until then: a
Gazebo pause longer than the 250 ms link-loss time trips the emulated 2000h link loss in the pause
(the stall power cycle follows, so nothing survives the resume either way); the drive ramp and the
heartbeat producers run on host time; the CTL-017 timers are evaluated on a 5 ms host tick, so at
a real-time factor far above 1 a check sees coarser simulation steps.

Runner speed: with CTL-017 on `gazebo`, the F02 Hall rate window counts Hall pulses from the
Gazebo joint against Gazebo time, so it no longer depends on the real-time factor. The 16 Nov
Build 1 observation (F02 at a real-time factor of 0.55, [S3]: 43 counts per second minimum,
about 86 at 18 mm/s) came from the window running on host time; the lock-step test runs the lift
at about a third of real time without F02 and raises F02 when the override is removed.

Renode is not attached in these tests; its virtual-to-host ratio and its skew against Gazebo
(C2) are not reported yet and are not assumed zero.

Later option, not built: lock-step. Gazebo is stepped by a fixed number of iterations per Renode
time quantum, and the emulators take their time from the same source.

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
| `/sil/fw/command` | std_msgs/String | into `renode_firmware` | `reset` (machine reset in the same Renode), `restart` (fresh Renode) |
| `/sil/drive_bus/audit` | std_msgs/String (JSON) | out of `drive_bus_audit` | SDO audit counters of the `vcan_drive` capture |

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
- Renode runs only the read-only telemetry image, on CAN1 only (section 4.1). The host link is
  described, not run. No firmware commands the drive, the lift or the BMS.
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
| S12 | Renode 1.16.1 portable release, `scripts/complex/socketcan_bridge` (SocketCAN bridge example) and `platforms/cpus/stm32h743.repl` of that release |
| S11 | Vendored emulators: openamr-platform-fw `tools/can_sil` at ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d (draft PR #53). PR #54 holds the same `tools/can_sil` files at that commit |
