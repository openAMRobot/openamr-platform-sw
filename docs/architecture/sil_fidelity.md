# SIL fidelity: what the base, lift and arm software-in-the-loop models, and what it does not

Status: review record, 19 Nov 2026. Development evidence only. Nothing in this table is
real-robot, mechanical, electrical or safety evidence. It describes code at the commits named in
the evidence column; most of it is in open draft pull requests, not on `main`.

## Level of modelling

Every model in this layer is **signal level**: CAN frames, UART bytes and register reads, with
values computed by Python or C++ state machines and by Gazebo rigid-body physics. **There is no
circuit-level simulation** anywhere in openamr-platform-sw, openamr-platform-fw or
openamr-upperbody-sw at the commits read: no SPICE or electrical model, no motor phase currents,
no CAN physical layer (bit timing, dominant/recessive levels, termination, ACK, error frames,
bus-off), no UART line levels or baud timing, no power stage, no battery electrochemistry. The
one exception to "no timing" is the arm bus, where frame transmission time is computed from a
bit count (see row "Arm buses"); that is arithmetic, not a physical-layer model.

Evidence key: sw = openAMRobot/openamr-platform-sw, fw = openAMRobot/openamr-platform-fw,
ub = openAMRobot/openamr-upperbody-sw. `#N` is a pull request, the 7-hex value its head.

## Fidelity table

| Question | Answer | Evidence |
|---|---|---|
| Firmware in Renode: target build or substitute? | The ELF is the **STM32H723 target build** of the read-only **telemetry image** (`boards/stm32/telemetry_image`, define `STM32H723xx`, linker script `STM32H723xG_flash.ld`, CubeMX HAL of the H723 project). It is not the robot base-controller firmware. The **machine model is a substitute**: Renode 1.16.1 has no H723 platform, so `platforms/cpus/stm32h743.repl` is loaded plus `tests/renode/stm32h723-additions.repl` (adds FDCAN3 as an MCAN at 0x4000D400) and a PWR CSR1 tag | fw #55 763bdd3 (`CMakeLists.txt`, `cubemx/CMakeLists.txt`, `stm32h723-additions.repl`); sw #144 53a69e9 `renode_fw.py`; reproduced locally 19 Nov 2026, below |
| Peripherals modelled | Cortex-M7 core, NVIC, SysTick (frequency forced by a hook to the HAL's SystemCoreClock, re-installed after each machine reset), FDCAN1, FDCAN2, FDCAN3 (Renode MCAN, frames whole), USART3 (console file or pseudo-terminal) | sw #144 `docs/architecture/sil-topology.md` 4.1 |
| Peripherals stubbed or absent | RCC (PLL and divider registers not kept: unhandled reads and writes), FDCAN clock calibration unit (`CAN_CCU`, non-existing peripheral, reads 0), PWR (tags), RCC_RSR reset flags (read 0, reset cause "unknown"), EXTI, GPIO levels and alternate functions, MPU. UART4/5/7/8, USART2/6 are initialised but not connected in the SIL launch (the fw harness feeds TF-Luna bytes on USART2; the sw SIL does not). **Timers (TIM): not used by the image** (time base is SysTick only). **ADC: not used by the image and not modelled.** Ethernet: `MX_ETH_Init` not called. No IWDG/WWDG in the image | local Renode run (below); fw #55 `tlm_time.c`, `cubemx_main_reuse.c` |
| micro-ROS live in Renode? | **Yes, only in `stm32:=renode_readonly` on sw #144** (draft, not on `main`): serial XRCE over a Renode pseudo-terminal on USART3 to `micro_ros_agent serial` (115200), image preset `TelemetryUrosSerialSimBench`. Outbound only: `/base_fw/status`, `/base_fw/wheel_speeds`, `/base_fw/drive_heartbeat`, `/base_fw/tof/*`. No subscriber, no service, no `/cmd_vel` into the firmware. UDP over Ethernet (the robot target) is not built. DDS domain 0 enforced | sw #144 `renode_fw.py`, `nodes/renode_fw_node.py`; sil-tests run 43765960526 (Renode steps green) |
| CAN FD or classic, per bus | CAN1 drive, CAN2 BMS, CAN3 lift: **classic CAN** in firmware (`FDCAN_FRAME_CLASSIC` on FDCAN1/2/3; nominal 48 MHz / (6 x 16) = 500 kbit/s CAN1 and CAN3, / (12 x 16) = 250 kbit/s CAN2; not checked in Renode) and on vcan (MTU 16 set by `sil_vcan.sh` from sw #149). Arm L and Arm R: **CAN FD with BRS**, `vcan_arm_l`, `vcan_arm_r`, MTU 72 | fw #55 `OpenAMRobot/Core/Src/main.c`; sw #149 b60c60a `sil_vcan.sh`; ub #67 3a5bb11 (merged) `arm_emulator.cpp`, `setup_vcan.sh` |
| CAN IDs, CAN1 `vcan_drive` | ZLAC8015D node 1: SDO 0x601/0x581, heartbeat 0x701 (100 ms), NMT 0x000. SIL-only PDOs in `renode_readonly`: RPDO1 0x201 (60FFh:01/02), RPDO2 0x301 (6040h). The telemetry image sends only SDO uploads (606Ch:01/02, 6041h, 603Fh, 2026h:05) | sw main 69847ab `config/sil.yaml`; sw #144 sil-topology 4.1 |
| CAN IDs, CAN2 `vcan_bms` | Daly, 29-bit: request 0x18 DD 01 40, response 0x18 DD 40 01, DD = 0x90..0x98 (0x95..0x97 unanswered). Protocol unconfirmed for the delivered unit | sw main `daly_bms.py` |
| CAN IDs, CAN3 `vcan_aux` | CTL-017 node 0x10: NMT 0x000, EMCY 0x090, TPDO1 0x190, TPDO2 0x290, RPDO1 0x210, RPDO2 0x310, SDO 0x610/0x590, heartbeat 0x710. Base heartbeat 0x701 consumed (300 ms). Permit PDO 0x410 (6 bytes, 50 ms, 200 ms freshness): SIL test value, not a contract | sw main `config/sil.yaml`, vendored `ctl017_od_v1.py` |
| CAN IDs, arm buses | Damiao motors 0x01..0x08 (J1..J7, gripper), replies 0x11..0x18, register frames 0x7FF | ub #67 `README.md`, `codec.hpp` |
| Device emulated: ZLAC8015D | Host Python (vendored fw `tools/can_sil` at ad4b023, byte-checked). CiA 402 enable 06/07/0F, velocity mode, 606Ch from Gazebo wheel speed, 2000h link-loss (ramps to zero; real stop mode unknown), 1017h heartbeat producer. No PDO mapping of the real drive, no torque, current or slip | sw main `zlac_gazebo.py`, vendor checksum test |
| Device emulated: CTL-017 lift | Host Python (vendored). NMT, TPDO/RPDO v1 dictionary, Hall count and three stop sensors from the Gazebo lift joint, faults F01..F09, RPDO1 keep-alive 300 ms, base heartbeat consumer 300 ms, permit gate (fixed rules only) | sw main `ctl017_gazebo.py` |
| Device emulated: Daly BMS | Host Python, platform-sw only (not vendored). Static values with injection; no link to Gazebo, no cell model; answers on receipt | sw main `daly_bms.py` |
| Device emulated: Damiao (OpenArm) | Host C++ inside a `gz_ros2_control` controller: MIT, position-velocity, velocity, position-force modes, state reply in the vendor layout; motion through Gazebo joint effort; damping attenuation above 200 Hz (numerical, switchable) | ub #67 `motor_model.cpp`, `emulator_controller.cpp` |
| Clock: ZLAC8015D | Wheel feedback freshness: Gazebo time (sw #158). **2000h link-loss, node tick and heartbeat period: host monotonic** until fw #58 is merged and re-vendored (`clock_inventory.yaml` status `to_change`) | sw #158 607a496 `config/clock_inventory.yaml`; fw #58 ff7e79b |
| Clock: CTL-017 | All timers on Gazebo time through the `_now_ms` override (sw #158); host monotonic on `main` | sw #158 `ctl017_gazebo.py` |
| Clock: Daly BMS | No timer. Host-side poll timeout of the test client on Gazebo time (sw #158) | sw #158 inventory |
| Clock: Damiao | Physics step and MIT law on the controller period (Gazebo time). Frame receipt, bus timing model and `can_timeout_s` (default 0, off) on host `steady_clock` | ub #67 `arm_emulator.cpp` `clock_now()` |
| Clock: firmware in Renode | Renode virtual time (SysTick). Not locked to Gazebo or host. Measured ratio to wall time 0.92 to 1.07 (micro-ROS image, WFI idle, no throttle) and about 0.55 to 0.83 in console mode (5 MIPS cap and throttle) | sw #144 sil-topology 6, run 43741217008 |
| Behaviour across resets (#159) | Gazebo time going backwards is a new generation: ZLAC and CTL-017 power cycle. **Open defects (strict xfail):** ZLAC frames already queued at the reset are applied after the power cycle (drive enabled with an old target); a CTL-017 RPDO1 delayed across a reset runs, because RPDO1 carries no producer epoch. CTL-017 queued frames are already rejected (PRE-OPERATIONAL) | sw #158 `test_clock_gates.py` (2 xfail), issue sw #159 |
| Behaviour before the first clock (#160) | sw #158: ZLAC is "not powered" before the first `/clock`: no boot-up, no heartbeat, frames dropped, then `power_cycle()`. CTL-017 already stays in INIT. Proposed, awaiting the firmware owner | sw #158, issue sw #160 |
| Behaviour on a Gazebo pause | A stall longer than 1 s (host steady time) power cycles ZLAC and CTL-017. This is a SIL policy, not device behaviour. With firmware in Renode, which keeps running during a Gazebo pause, the image would see a drive boot-up; not yet tested together (sw #166 reconciles) | sw #158 `zlac_gazebo.py` `_follow_clock` |
| Heartbeat behaviour (described, not changed) | ZLAC produces 0x701 every 100 ms, consumes none. CTL-017 produces 0x710 every 100 ms and consumes 0x701 at 300 ms after the first one, then F07 and stop. Telemetry image (RenodeTest values) consumes the ZLAC and CTL-017 heartbeats at 300 ms virtual time and reports "lost"; the default build disables both consumers. The image sends no heartbeat. The host test client sends 0x701 on CAN3 | vendored `ctl017_emulator.py`, `zlac8015d_emulator.py`; fw #55 `tlm_params.h` |
| Watchdog behaviour (described, not changed) | Telemetry image: **no IWDG, no WWDG, no command watchdog**; a halt in that image is diagnostic only. Host test client: no command timeout. Drive-side 2000h link-loss is the only command-loss reaction in the default SIL. fw #56 (`tests/sim_reference`, simulation only, not in any board image, used by sw #147 `stm32:=simref`) has a 200 ms command watchdog and 300 ms drive heartbeat consumer as unapproved test values | fw #55 `tlm_main.c`; fw #56 b6c8909 `simref_timing.h` |
| Fault injection, base | `/sil/drive/inject`: oc-lo, oc-hi, uv, enc-lo, enc-hi, busoff (and clear). `/sil/lift/inject`: stall and further CTL-017 hooks (Hall slip, rate, reversed, current, motor power, Hall supply, E-stop 2 input). `/sil/bms/inject`: failure bytes, MOS, SOC, voltage, current. `/sil/stm32/can1_silent`, `/sil/lift/permit_silent`. Tested end to end: CAN1 silence and link loss, permit withdrawal, BMS values; vendored suite: ZLAC faults and stop-write timeout or abort, CTL-017 faults | sw main nodes, `test_integration/test_sil_gazebo.py`, vendored tests (36) |
| Fault injection, arms | CAN loss, single-motor silence, fault codes 0x8..0xE, over-voltage, temperature, E-stop power cut, supply voltage; each tested against the real upstream driver | ub #67 `test_real_driver` (10) |
| Fault injection, clock | Reset, stall, resume, forward jump, duplicate time, first clock; tested on `ManualClock` and the node clock | sw #158 `test_clock_gates.py`, `test_sim_time.py` |
| Not modelled at all | E-stop chain, contactors, brakes, motor power removal, charge inhibit, CAN physical layer, bus load, UART errors, wheel torque and slip, battery behaviour under load | sw main sil-topology 8 |

## Local reproduction, 19 Nov 2026

- Telemetry image `RenodeTest` built from fw 763bdd3 with `sil_renode_firmware.sh` of sw #144
  (Ubuntu `gcc-arm-none-eabi` 13.2.1; ELF SHA-256 differs from the CI build, toolchain differs).
- fw `tools/ci/renode_telemetry.py` on Renode 1.16.1: `nominal`, `drive_silent`,
  `heartbeat_stops`, `reset_mid_run`, `aux_buses`: 5 passed. Renode log: unhandled RCC and
  `CAN_CCU` accesses, as listed above.
- The sw Renode node tests and the Gazebo end-to-end tests were not run locally (no ROS 2, no
  vcan in the review container): BLOCKED locally, green in hosted CI only.
