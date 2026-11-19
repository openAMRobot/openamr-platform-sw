# SIL fidelity: what the base, lift and arm software-in-the-loop models, and what it does not

Status: review record, 19 Nov 2026. Development evidence only. Nothing in this table is
real-robot, mechanical, electrical or safety evidence. It describes code at the commits named in
the evidence column; most of it is in open draft pull requests, not on `main`.

Parts: (1) signal-level fidelity of the SIL layer (below); (2) physical fidelity audit per
subsystem; (3) fidelity roadmap; (4) hardware readiness gates, no motion.

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

## Part 2. Physical fidelity audit, 19 Nov 2026

**The host emulators (ZLAC8015D, CTL-017, Daly BMS, Damiao, and the STM32 host stand-in
`stm32_host.py`) are protocol and behaviour stand-ins. They are not validated electrical
models.** They reproduce frames, state machines and selected reactions as written in the
sources below; none was compared with a delivered device.

**Nothing in this audit is physically verified. No bench record exists for any OpenAMRobot 2.0
subsystem.** The reality gap register (audits d776cec `digital-twin/phase1/reality_gap_register.csv`,
188 rows) has no row with a measured real value; the only physical measurement it cites is the
Gate A (legacy) track of 0.46 m, taken on the existing unit, not on the 2.0 wheel assembly.

Class key: **implemented model** = the behaviour is coded from a named source and tested in
simulation; **approximation** = a simplified model whose numbers are not from the real part;
**placeholder** = a value or frame with no real source (or a default); **not modelled** = no
layer represents it; **physically verified** = a bench or robot record exists (none today).
Effect key: N navigation, M manipulation, H hardware integration.

Additional evidence keys: hw = openAMRobot/openamr-platform-hw main 8ed2b06; reg =
openAMRobot/.github `decisions.yaml` at 87c8486; RG-x = row of the reality gap register above;
MP-x = item of audits d776cec `digital-twin/phase1/measurement_plan.md`; up = upstream
openarm_description 8dbfcc9; ub #77 = 323a2ce, ub #69 = 4270e53.

### Subsystem table

| Subsystem | Simulated value | Source of the simulated value | Real value and its source | Class | Effect | Improvement needed |
|---|---|---|---|---|---|---|
| Drive wheel collision and contact | Flat cylinder r 0.100 m, length 0.046 m; `mu1` 1.5, `mu2` 0.5, `fdir1` (1 0 0); contact stiffness and slip tags not applied | sw main `robo_urdf.urdf.xacro` (wheel links), `gazebo_control.xacro` (friction macro, "simulation estimates") | Tyre 200 +/- 2 mm, width 49.4 +/- 1 mm, Shore A 65 +/- 5 (hw `datasheets/ZLTech/wheel_ZLLG80ASM250/readme.md`, B2 V1.01 drawing). Tyre to floor friction unknown (RG-MEC-011); three engine setups, none named reference (RG-MEC-012) | approximation (geometry), placeholder (friction) | N high: heading error in turns, see F1 and F2 | Point or narrow contact (sphere r 0.098 m or a 4 mm cylinder; robot-description owner and software lead decide); then measured friction (MP-04) |
| Wheel radius | 0.100 m in collision, DiffDrive and `config/sil.yaml`; sw #146 68642a8 (open) sets collision and DiffDrive to 0.098 m | sw main `gazebo_control.xacro`, `openamrobot_sil/config/sil.yaml`; sw #146 | reg FRAMES-REP105: wheel radius 0.100 m with base_link 0.098 m intended (2 mm tyre compression, no suspension). Owner decision of 19 Nov 2026: 0.100 m nominal, 0.098 m effective rolling radius, pending register approval (.github #94). Loaded rolling radius not measured (RG-MEC-008) | placeholder (effective value) | N: odometry scale; +/- 2 mm drawing tolerance alone is 1 % of distance | Register entry with both values; bench straight-run calibration (MP-03, MP-04) |
| Track (wheel separation) | 0.400 m in the wheel joints, DiffDrive and `sil.yaml` | sw main `robo_urdf.urdf.xacro`, `gazebo_control.xacro` | reg DRIVE-TRACK open: 0.400 m is the model placeholder; Gate A unit measured 0.46 m (legacy); the 2.0 wheel assembly will change it (RG-MEC-009) | placeholder | N: yaw odometry and turn kinematics scale with the track | CAD of the 2.0 wheel assembly, then measure on the built base (MP-03) |
| Base mass and inertia | base_link 49.948 kg, complete base 58.163 kg, robot 87.11 kg; inertia from component boxes, no products; whole 3.45 kg hub motor on each spinning wheel link | sw main `config/roadmap_mass_export.yaml` (pinned docs ff8ca57) | reg MASS-LEDGER: every mass is an estimate under ledger review; candidate base_link 53.2 kg. CAD-substituted base_link 52.692 kg (RG-MEC-002); hub motor 3.60 kg +/- 5 % on the drawing (RG-MEC-006); nothing weighed (RG-MEC-001) | approximation | N (stopping, wheel loads), M (tip margin at lift top, RG-MEC-026) | Unique physical-instance ledger, then weigh and measure CoM (MP-01, MP-02) |
| Castors | Four fixed spheres r 0.026 m, `mu` 0 (frictionless sliders), housing boxes 0.060 x 0.060 x 0.040 m; no swivel, no rolling | sw main `robo_urdf.urdf.xacro`, `gazebo_control.xacro` ("caster rolling and contact dynamics are not modelled") | Blickle LPA-VSTH 35K: wheel 35 mm, 15 mm offset, 100 kg at 3 km/h (RG-MEC-013); rolling resistance unknown (RG-MEC-014); six-contact load split unknown (RG-MEC-017) | approximation | N: no castor drag or swivel torque in turns | Castor model proposal (audits #90, Proposed); measure rolling and swivel (MP-04) |
| Drive motor ZLLG80ASM250 and ZLAC8015D | ZLAC emulator: CiA 402 velocity mode, 60FFh range +/- 1000 r/min, 6083h/6084h default 500 ms read as 0 to 1000 r/min (reference speed unconfirmed), 2000h link-loss ramps to zero, 1017h heartbeat 100 ms read as ms. Wheel motion through a body twist into DiffDrive: accel +/- 0.5 m/s2, +/- 1.0 rad/s2, velocity +/- 1.0 m/s, +/- 2.0 rad/s. No torque, no current limit, no effort limit on the wheel joints. Nav2 on main: max 0.20 m/s, decel -2.5 m/s2 | vendored fw `tools/can_sil` (ad4b023) `zlac8015d_emulator.py`; sw main `zlac_gazebo.py`, `gazebo_control.xacro`, `openamrobot_nav2/config/nav2_params.yaml` | Motor: 24 V, 5.5 N m and 6.5 A rated, 16 N m and 19 A peak, 200 r/min rated, 260 r/min peak (hw readme, B2 V1.01). Driver current limits unknown (RG-DRV-014); ramp and quick stop unknown (RG-DRV-012, RG-DRV-013); 606Ch unit, encoder counts, axis signs unknown (RG-DRV-009 to RG-DRV-011). reg SPEED-CEILING: Build 1 ceiling 0.5 m/s. reg MASS-LEDGER: deceleration 1.5 m/s2 travel and 1.0 m/s2 hold-and-move, identical in Nav2, smoother, ZLAC8015D and Gazebo, in a separate PR after sw #90 | implemented model (protocol), placeholder (dynamics) | N: stop distance and acceleration differ per layer; H: units, signs and PDO mapping assumed | V4.2 manual, then wheels-up captures (MP-09, MP-10); one PR aligning the limits per MASS-LEDGER; DiffDrive velocity limit at or below the 0.5 m/s ceiling |
| CANopen timing on CAN1 | SDO only in the telemetry image; host client writes both targets each 20 ms; heartbeat consumers as in Part 1 | Part 1 rows; fw #55 763bdd3 | 500 kbit/s classic, worst-case 8-byte frame 270 us (RG-COM-009, design value); SDO response time unknown (RG-COM-010); one SDO per 20 ms tick gives up to 120 ms target latency and 20 ms left to right skew (RG-COM-012, analysis) | implemented model (frames), not modelled (bus timing) | N, H | CAN1 capture on the bench (MP-09, MP-15) |
| Brake | None. A stopped or unpowered base rolls freely in Gazebo (RG-DRV-016) | n.a. | reg DRIVETRAIN: 24 V, 12.5 W per motor, 6 N m holding, release at most 100 ms, engagement at most 30 ms without diode; 5 N m against 6 N m still open (D-25) | not modelled | H, N (hold on slopes, release delay before motion) | Brake timing capture (MP-11); any plant model is written by the electrical owner. Described only; no agent writes brake logic |
| Lift CTL-017 and DOLD Hexalift V4 | SIL stand-in prismatic joint 0 to 0.350 m, 0.018 m/s, 800 N; carriage 1.0 kg test value. Emulator: 18 mm/s full speed (no-load), travel 360 mm test value, stop sensors at 5, 180, 355 mm, window 1 mm, 0.2092 mm per Hall count, moving current 1000 mA placeholder, homing, faults F01 to F09, 300 ms supervision | sw main `openamrobot_sil/urdf/sil_robot.urdf.xacro`; vendored `ctl017_emulator.py` (each value marked TODO for the firmware owner) | reg LIFT: 350 mm stroke, 800 N push, 18 mm/s, 800 N held with power off (DOLD, 14 Oct 2026); stop sensors SICK IME12-04BNSZC0S at z 120, 295, 470 mm above the column base. Loaded speed, Hall counts per mm, switching window, running current unknown (RG-DRV-018 to RG-DRV-021); catalogue maximum 7 A | implemented model (protocol, stops, homing), placeholder (plant values) | M (shoulder height and timing), H | Lift bench items (MP-14); holding test is a release gate in reg LIFT |
| Arm motors (Damiao, OpenArm) | Emulator inside `gz_ros2_control`: MIT law kp 70, 70, 70, 60, 10, 10, 10 and kd 2.75 to 0.5 from the upstream config; controller 100 Hz (500 and 750 Hz only with ub #66); frame time from bit count; `can_timeout_s` 0 (off); damping attenuation above 200 Hz | ub #67 (merged) `motor_model.cpp`, `arm_emulator.cpp`; RG-DRV-007, RG-TIM-001 | Peak 40, 27, 7 N m; rated 20, 9, 3 N m (RG-DRV-001); upstream real bring-up 750 Hz, project rate not decided (RG-TIM-001); motor inner loop, reply latency and jitter, friction, damping and rotor inertia unknown (RG-TIM-002, RG-COM-005, RG-MEC-034 to RG-MEC-036) | implemented model (protocol, MIT law), placeholder (dynamics) | M high | Arm dynamics and bus timing on the bench (MP-06, MP-07, MP-08) |
| BMS Daly | Constants: 25.6 V, 0.0 A, SOC 80 %, 84 Ah, 25 C, 8 cells, cell voltage = pack / 8; answers on receipt; 0x95 to 0x97 unanswered; no link to Gazebo, no cell or load model | sw main `openamrobot_sil/daly_bms.py` | Pack 8S LiFePO4, 25.6 V nominal, 84 Ah; 28.4 V routine charge, 29.2 V ceiling; lower limit, sag and internal resistance unknown (RG-DRV-023 to RG-DRV-025); protocol, timing and current sign for the delivered unit unknown (RG-COM-016 to RG-COM-018); reg BMS-FAULTS | placeholder | H (protocol and signs), N (no battery effect on speed or run time) | Protocol document for the delivered order code; CAN2 capture (MP-13); then a battery model linked to load |
| Power distribution | None: no SAFE bus, precharge, regeneration, converters, fuses or contactors | Part 1 row "Not modelled at all" | RG-DRV-026 to RG-DRV-036 (precharge 940 uF per drive with 50 to 60 A inrush, regen clamp pending D-23, converter 10 A limit, fuse ratings) | not modelled | H (arm rail brownout, inrush, regen) | Power bus measurements (MP-12), then an owner-written model. Contactor and charge-inhibit logic: described only |
| LiDAR RPLIDAR S3 | `gpu_lidar` 360 samples (1.0 deg), 10 Hz, range 0.40 to 10.0 m, resolution 0.01 m, gaussian sd 0.001 m; lidar_link (0.330, 0, 0.124), yaw pi | sw main `gazebo_control.xacro` (no source stated for these numbers) | Min 0.05 m; max 40 m white, 15 m black; 10 Hz typical (10 to 20 Hz); 0.1125 deg at 10 Hz; accuracy +/- 30 mm (not a standard deviation); noise sd and latency unknown (sw `config/sensor_profiles.yaml`, RG-SEN-002 to RG-SEN-005). CAD Rev B centre at x 0.321 against 0.330 (RG-SEN-006) | approximation | N (near-field returns below 0.40 m absent, angular resolution 9 times coarser) | Gazebo reads `sensor_profiles.yaml` (sw #156); static-target noise test (MP-17) |
| IMU ICM-42688-P | Gazebo IMU 50 Hz, no noise, gz `imu` bridged to `/imu`, on imu_link (0.0544, 0, 0.1210) yaw -pi/2 | sw main `gazebo_control.xacro`, `openamrobot_gazebo/config/gz_bridge.yaml` | Gyro noise 2.8 mdps/sqrt(Hz), accel 70 ug/sqrt(Hz) (DS-000347 rev 1.9 via `sensor_profiles.yaml`); ODR, bias and timestamp skew unknown (RG-SEN-012 to RG-SEN-014). reg IMU-TOPIC-OWNERSHIP: firmware publishes `/imu/data_raw`; the EKF fuses angular_velocity.z only. Main `ekf.yaml` still reads `/imu/data` (sw issue #62) | approximation (ideal sensor) | N (EKF yaw) | Datasheet noise into the sensor; ODR from the firmware owner; static bias test (MP-19) |
| Base camera Gemini 336L | platform-sw main: no sensor on base_camera_link; the only sim camera is the legacy Gate A `camera_link` frame, 1.047 rad (60 deg) hfov, 640 x 480, 12 Hz. Build 1 (ub #77): RGB-D on base_camera_link, 1.047 rad, 640 x 480, 6 Hz. MuJoCo (ub #69): camera on the legacy `camera_optical_frame`, fovy 46.8 deg derived from that Gazebo sensor | sw main `gazebo_control.xacro`; ub #77 `build1_robot.urdf.xacro`; ub #69 `engine_overrides.yaml` | Colour 94 x 68 deg, depth 90 x 65 deg, depth 0.17 to 20 m, up to 1280 x 800 (vendor page via `sensor_profiles.yaml`, RG-SEN-007); PDF datasheet not filed; latency unknown | placeholder (FOV, intrinsics, noise) | N (docking tags, RG-SEN-032), embodied-AI data | FOV and resolution from the profile; file the datasheet; calibrate intrinsics on the robot (MP-18) |
| Head camera ZED Mini | Pose per the description's head camera frames (register HEAD-CAMERA-IDENTITY: lens 0.060 m ahead of and 0.223 m above the arm mount, pitch baseline 30 deg; not re-checked numerically here). Build 1: one RGB-D sensor, 1.57 rad (90 deg), 640 x 360, 6 Hz. MuJoCo (ub #69): fovy unknown, MuJoCo default 45 deg | ub `upper_sensors.xacro` comment; ub #77; ub #69 `engine_overrides.yaml` | ZED-121210: FOV 102 x 57 deg raw, about 85 x 54 deg rectified, as quoted in the Use_Case_1 requirements; datasheet not filed; stereo, not one RGB-D; rate, latency, noise unknown (RG-SEN-029) | placeholder (FOV, intrinsics) | M (object pose from the head camera) | File the datasheet; stereo pair or rectified RGB-D with the rectified FOV; calibration (MP-18) |
| Wrist cameras | Frames on openarm_left/right_ee_base_link at (0, +/-0.05, -0.03) m, rpy 0, status `placeholder` in `datums.yaml`. Build 1: colour 1.2 rad, 320 x 240, 5 Hz, marked UNVERIFIED. MuJoCo: default fovy 45 deg | ub main a3a93c6 `config/datums.yaml`, `upper_sensors.xacro`; ub #77; ub #69 | up `camera_view_frame` joint on the same parent: (0.0218353, +/-0.024136, -0.086895) m, rpy (-/+2.1009, 1.1424, -/+2.1431). SKU, intrinsics and rate unknown (RG-SEN-031) | placeholder | M high (grasp perception, dataset camera poses) | See F3; take the upstream frame after the mount is checked in CAD; kit SKU and intrinsics |
| Base controller and firmware timing | STM32 is a host stand-in (`stm32_host.py`) in the default SIL; Renode runs the telemetry image, not the base controller (Part 1); control cycle 20 ms in the host client | Part 1; sw main `stm32_host.py` | Base cycle 20 ms is an assumption, not a decision (RG-TIM-005); `/base/status` 10 Hz and `/base/aux_status` 5 Hz frozen in I8 rev 4, transport not chosen (RG-COM-022); drive heartbeat 100 ms and 2000h 250 ms proposed (D-24, RG-COM-013, RG-FLT-012); BMS poll 5 Hz proposed (RG-TIM-010); clock tree 192 MHz not verified on a board (RG-TIM-009) | implemented model (protocol), placeholder (timing values) | H | fw #58 re-vendor; CAN captures on the bench (MP-15); owner decisions on D-24 |
| Fault handling | Injection hooks and tests as in Part 1; arm driver reports healthy on CAN loss and power cut (RG-FLT-003, reproduced); arm move to zero at full gains on activation (RG-FLT-001) | Part 1 rows; ub #67 | Real reactions unknown: ZLAC link loss (ramp, quick stop, freewheel or fault, RG-FLT-012); BMS hard-fault disconnect and STM32 stop-and-lock are register text (reg BMS-FAULTS), not implemented in any image | implemented model (injection), placeholder (device reactions) | H, M | Wheels-up link-loss test (MP-10), arm fault tests with arms supported (MP-08). Watchdog and interlock logic: described only |

### Known findings

**F1. Flat wheel collision and turn slip.** The drive wheel collision is a flat cylinder 46 mm
wide (r 0.100 m). On a rigid floor it touches along a line across the tread, so an in-place turn
scrubs the whole width while DiffDrive holds the wheel speed. Proposed: a sphere r 0.098 m or a
4 mm cylinder. The robot-description owner and the software lead decide; sw #146 changes only
the radius.

**F2. Grocery odometry drift, run 44299879412 (ub, branch `demo/grocery-dark-store` at
6170e6a, job 134786543960).** Read from the job log (`GROCERY ODOM` lines, one sample every 5 s
of Gazebo time, odometry never reset):

| Interval | Base motion | Position error odom against ground truth | Yaw error |
|---|---|---|---|
| nav_to_shelf, t 11.9 to 21.9 | straight, v 0.20 m/s | 0.005 to 0.043 m | 0.000 to +0.026 rad |
| t 21.9 to 26.9 | in-place turn, w -0.36 rad/s | 0.061 m | +0.026 to -0.076 rad |
| t 37.0 to 41.2 | in-place turn, w -0.26 rad/s | 0.065 to 0.073 m | +0.035 to +0.093 rad |
| t 41.2 to 106.3 | short retreat (0.28 m) and turn into the aisle | 0.030 m | +0.093 to +0.159 rad |
| packing approach, t 106.3 to 136.3 | straight, v 0.15 to 0.20 m/s | 0.030 to 0.844 m (+0.123 to +0.144 m per 5 s) | +0.143 to +0.159 rad, constant |
| return, t 200.2 to 220.2 | straight, opposite direction | 0.850 to 0.399 m (falls) | +0.189 to +0.214 rad |

The guard failure (0.907 m against 0.56 m) is the packing approach. On every straight
5 s step the odometry step length equals the ground-truth step within 0.9975 to 1.0048, and its
direction differs by 0.146 to 0.214 rad, the yaw error at that time. The growth rate matches
v x yaw error (0.18 m/s x 0.15 rad x 5 s = 0.135 m per 5 s), and the error shrinks when the robot
drives back the other way. **So "slip in turns" is supported:** the yaw error changes in the
intervals that contain turns and stays constant on straights; the position error grows on the
straight stretch only as the lever arm of that heading error, with no straight-line slip and no
radius scale error visible in this run. Limits: one run, no seed, 5 s sampling, so single turns
are not resolved; the arm-pose dependence of base yaw reported on 13 Nov is not separated here.

**F3. Wrist camera frames.** Against up `camera_view_frame` on the same parent link, our
placeholder frames are 66.2 mm away on each side; the full rotation between the frames is
104.5 deg, and our link x axis is 103.0 deg from the upstream frame's x axis. Which axis of
`camera_view_frame` is the view direction is to be confirmed before adoption.

**F4. Camera fields of view.** No simulator uses a datasheet FOV: base 60 deg against 94 deg
colour or 90 deg depth; head 90 deg (Gazebo) or 45 deg (MuJoCo default) against 102 deg raw or
about 85 deg rectified; wrist 69 deg (Gazebo) or 45 deg (MuJoCo) against unknown. No
distortion, calibration or noise from the parts.

**F5. BMS constants.** The Daly emulator returns fixed values (25.6 V, 0.0 A, SOC 80 %) unless
a test injects others; it has no link to load, time or the simulation.

**F6. No power model.** No layer models the battery under load, the SAFE bus, precharge,
regeneration, converters, fuses, contactors or the brake supply.

## Part 3. Fidelity roadmap

Ranked by effect on navigation, then manipulation, then hardware integration. Owners from
CODEOWNERS and openAMRobot/.github `maintainers.yaml` roles, or the Team doc section named in the
gap register; "to be assigned by Alex" where neither names one. Effort: S = description or
configuration change with a test, about a day; M = several days or a supplier document; L = needs
the bench or built hardware.

| Rank | Improvement | Effect | Owner | Effort | Evidence that closes it |
|---|---|---|---|---|---|
| 1 | Drive wheel contact shape (sphere r 0.098 m or 4 mm cylinder) | N high (F1, F2) | robot-description owner (@fridayali) and software lead (@panthera-momagdii), CODEOWNERS `ros2/src/openamrobot_description/` | S | Test on the expanded URDF; grocery and Baseline 1 reruns with yaw error per turn reported; later the rolling test MP-04 |
| 2 | Effective rolling radius 0.098 m and nominal 0.100 m in the register; then sw #146 | N (odometry scale) | platform lead (@BotshareAI, register), with the description owners | S | Register entry and its check test (.github #94); bench straight-run calibration MP-03, MP-04 |
| 3 | One set of velocity and deceleration limits in Nav2, smoother, ZLAC8015D and Gazebo (reg MASS-LEDGER 1.5 and 1.0 m/s2; SPEED-CEILING 0.5 m/s; DiffDrive today 0.5 m/s2 and 1.0 m/s) | N (stopping), H | platform lead (register owner); implementation by the platform-sw code owners (@skunal3318, @panthera-momagdii), after sw #90 | M | Test that the four layers carry identical values; recorded stop tests at 0.2, 0.3 and 0.5 m/s (reg SPEED-CEILING) |
| 4 | ZLAC8015D behaviour from the V4.2 manual: ramp reference, 2000h reaction, units, signs, PDO mapping, current limits | H, N | electrical owner (@evans-kwashie), Team doc s7 Firmware and Base Controller | M then L | V4.2 manual filed; wheels-up captures MP-09, MP-10 |
| 5 | Wrist camera frames from up `camera_view_frame`, and datasheet FOVs for base, head and wrist cameras | M high, embodied-AI data | wrist and head: Team doc s7 Manipulation (RG-SEN-030, RG-SEN-031); FOV: Team doc s7 Perception and Simulation (RG-SEN-007, RG-SEN-029); upperbody-sw code owners (@Brinex45, @panthera-momagdii) | S | CAD check of the wrist mount; filed Orbbec and ZED Mini datasheets; frame and FOV tests; intrinsic calibration MP-18 |
| 6 | Track from the 2.0 wheel assembly | N | robot-description owner (Team doc s7 Mechanical and CAD); register DRIVE-TRACK owner platform lead | S after CAD | CAD Rev of the wheel assembly; measurement on the built base MP-03 |
| 7 | Gazebo LiDAR from `sensor_profiles.yaml` (0.05 m minimum, 0.1125 deg, noise) | N | Team doc s7 Perception and Simulation (sw #156) | M | Profile-consumption test; static-target noise test MP-17 |
| 8 | Base mass, CoM and inertia from the weighed robot | N, M (tip margin) | ledger: Team doc s7 Mechanical and CAD; weighing: Team doc s7 Bench and Integration | L | Signed mass ledger, MP-01, MP-02 |
| 9 | Castor swivel and rolling model | N | Team doc s7 Mechanical and CAD (audits #90 proposal) | M | MP-04 rolling and swivel measurements |
| 10 | Arm joint dynamics and controller rate | M | Team doc s7 Manipulation | L | MP-06, MP-07 |
| 11 | Lift plant values (loaded speed, Hall counts per mm, stop window, current) | M, H | electrical owner (@evans-kwashie) | L | MP-14 |
| 12 | IMU noise, ODR and bias | N | ODR: electrical owner; noise and bias: Team doc s7 Perception and Simulation | S then L | Firmware ODR value; MP-19 |
| 13 | BMS protocol for the delivered unit, then a battery model | H | electrical owner (Team doc s7 Hardware, Firmware and Safety) | M then L | Protocol document; MP-13 |
| 14 | Power bus model | H | electrical owner (Team doc s7 Hardware, Firmware and Safety) | L | MP-12; model written by the owner |
| 15 | Brake timing in the plant | H | electrical owner | L | MP-11; model written by the owner, never by an agent |
| 16 | Real base-controller timing values (cycle, SDO latency, heartbeats) | H | electrical owner | L | MP-15; D-24 decision |

## Part 4. Hardware readiness gates, no motion

**No physical robot motion is permitted until gates (b), (c) and (d) are recorded.** Gate (a) is
a bench board without motors. Nothing in this part instructs motion; the only commands given
are build or read-only commands. A recorded gate means its evidence is filed and the owner has
written pass in the record.

| Gate | Current status (19 Nov 2026) | Owner |
|---|---|---|
| (a) STM32 bench bring-up | Not started. fw #22 e7bcbd9: CI Debug, Release, host tests and DCO green; not approved; never flashed (its README) | electrical owner (@evans-kwashie, fw CODEOWNERS) |
| (b) LiDAR on the v0.0.1 robot | Open. `/scan` has 0 publishers while the driver runs (reported from the robot 17 Nov 2026; cause not identified) | on site: Team doc s7 Bench and Integration; platform-sw code owners (@skunal3318, @panthera-momagdii) |
| (c) Stop gate deployment | Open, blocked. `safety_stop_gate_node` exists only in sw #90 (4fad3d8, not merged) | platform-sw code owners; go by the platform lead (reg STOP-SCOPE owner) |
| (d) Teleop topic | Open. Robot teleop publishes `cmd_vel_teleop`; the tested gate config expects `cmd_vel_joy`; no contract change request filed | platform-sw code owners; software lead approves the contract |

### (a) STM32 bench bring-up (board only, no motors)

- Board: NUCLEO-H723ZG with the STM32H723ZG (reg BASE-CONTROLLER-IO, BASE-CONTROLLER-GATES).
- Firmware: fw #22 (`build: add STM32H723 CubeMX project`, head e7bcbd9) is a peripheral
  baseline: clock 192 MHz from HSE bypass, FDCAN1/2/3 configured but **not started**, SPI4,
  UART4/5/7/8, USART2/6, Ethernet pins; the main loop is empty; no motion, actuator, safety or
  communication stack. Four GPIO outputs, PE2, PE3, PG2 and PG3, are driven low at init and are
  not named in the README (the fw I/O note lists four MB7060 trigger GPIOs); confirm against the
  pin map before anything is wired to them.
- Build (from fw #22 `.github/workflows/firmware-stm32.yml`, in `boards/stm32/OpenAMRobot`):
  `cmake --preset Debug && cmake --build --preset Debug`, then
  `sha256sum build/Debug/OpenAMRobot.elf` and `arm-none-eabi-size build/Debug/OpenAMRobot.elf`.
  The repository has no STM32 flash procedure (`docs/flashing/build-and-flash.md` is Teensy);
  flashing is the owner's step.
- Power-up checks without motors: board on its USB only; no CAN transceiver to the drive, no
  ZLAC8015D, no motor or brake supply connected; record supply current, that the debugger halts
  the core in the main loop with no fault handler entered, and the level of PE2, PE3, PG2, PG3.
  The IMU check (WHO_AM_I 0x47, reg BASE-CONTROLLER-GATES evidence) needs firmware that reads
  SPI4; fw #22 has none, so it is BLOCKED in this image.
- Pass criterion: the built ELF hash is recorded and matches the flashed image; the core runs
  the main loop with no fault; outputs stay at their init levels.
- Evidence: build log, ELF SHA-256, programmer output, photo of the bench showing no motor power,
  the review approval of fw #22 or the commit flashed.

### (b) LiDAR on the v0.0.1 robot

Builds: release v0.0.1 (tag commit 9eaae60) starts `rplidar_ros` `rplidar_composition` at
115200 baud on the CP2102 by-id path (RPLIDAR A1, legacy). Main starts `sllidar_ros2`
`sllidar_node`, `lidar_model` default `a1` (115200, `Standard`), `s3` = 1000000, `DenseBoost`
(`openamrobot_drivers/launch/drivers.launch.py`). Which build runs on the robot is not recorded in
a repository: record it first. hw `manufacturing/bom/components-bom.md` names the existing unit's
LiDAR as the legacy (existing robot) A1M8 "by shape", sticker unconfirmed.

Why "driver runs, 0 publishers" narrows the cause: upstream `rplidar_ros` (ros2 branch, read at
24cc9b6) creates the `scan` publisher only after the serial connect, device info, health check
and scan start succeed, so a live node that failed any of these has no publisher. `sllidar_ros2`
at the pinned 3430009 creates it in the constructor, so with that driver 0 publishers means the
process is exiting and respawning, or the query runs in another ROS domain or middleware.

Read-only checks on site, in order (node name `/rplidar` on v0.0.1, `/sllidar_node` on main):

0. Same graph: `echo "$ROS_DOMAIN_ID $RMW_IMPLEMENTATION"` in the launching and the querying
   shell; `ros2 node list`; `ros2 topic info /scan -v`; `ps -ef | grep -E 'rplidar|sllidar'`.
1. Serial device and permissions: `ls -l /dev/serial/by-id/ /dev/ttyUSB*`; `id -nG`
   (expect `dialout` or a udev mode that allows the user); `lsusb`; `journalctl -k | grep -iE
   'cp210|ttyUSB'`; `fuser -v /dev/ttyUSB0` (another process holding the port);
   `ros2 param get <node> serial_port` and `readlink -f` of that path.
2. udev: `udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID'`;
   `ls /etc/udev/rules.d/`; `systemctl status ModemManager` (it can open USB serial ports).
3. Baud rate against the model: read the sticker (A1 115200, S3 1000000);
   `ros2 param get <node> serial_baudrate`.
4. Driver parameters and log: `ros2 param dump <node>` (channel_type, serial_port, frame_id
   lidar_link, scan_mode, angle_compensate); `grep -iE 'cannot bind|time out|health|start scan|scan mode'
   ~/.ros/log/latest/launch.log`.

Pass criterion: `ros2 topic info /scan` shows exactly one publisher, the driver; `ros2 topic hz
/scan` is recorded and compared with the model's datasheet; frame_id is `lidar_link`; stamps
advance. Evidence: deployed commit, the outputs above, the driver log, the sticker photo.

### (c) Stop gate deployment

- Where the node is: `ros2/src/openamrobot_nav2/openamrobot_nav2/safety_stop_gate_node.py` only
  on sw #90 (`issue35-stop-gate`, 4fad3d8, open, not merged). It is not in main 69847ab, not in
  v0.0.1, not in the integration heads sw #161 (fb25be1) or #167 (dccd5e0). The robot runs
  without it.
- What must be deployed, at recorded commits: a platform-sw build containing sw #90 (after it
  merges, or a head the owners name); openamrobot-interfaces 50ed777 (`openamr_nav_msgs`
  with `StopState`, pinned in sw #90 `nav-status-tests.yml`); the `twist_mux` package; the sw #90
  navigation launch, which wires twist_mux, `cmd_vel_mux`, `safety_stop_gate_node`,
  `cmd_vel_gated`, velocity_smoother, collision_monitor, `/cmd_vel`. The operator Stop (software)
  and Resume controls are openamrobot-ui #50 (draft); the services are `/safety/stop_lock_engage`
  and `/safety/stop_lock_release` (std_srvs/Trigger). Stop (software) is a functional stop,
  base-only, not a safety function (reg STOP-SCOPE).
- Record, part 1 (motor power removed): deployed SHAs; `ros2 node list` shows
  `/safety_stop_gate_node` and `/twist_mux`; `ros2 topic info /cmd_vel -v` has one publisher
  (collision_monitor); `/safety/stop_state` reports locked at boot; release is refused before
  T_READY (1.0 s) and accepted within 2 s of a clean boot (reg STOP-RELEASE); after engage,
  `cmd_vel_gated` is zero. A bag of `/safety/*`, `cmd_vel_*` and `/cmd_vel`.
- Record, part 2 (Stop and Resume with the wheels off the ground): run only after (b) and (d) are
  recorded and the platform lead has given the go; this document does not instruct it. The record
  names the command source, the time Stop was pressed, the time `cmd_vel_gated`, `/cmd_vel` and
  the wheel feedback reach zero, and shows that Resume clears the inhibit without restarting
  motion (reg STOP-RELEASE).
- Pass criterion: both parts recorded on the deployed commits, two human approvals of the record.

### (d) Teleop topic contract

- `cmd_vel_joy` is the manual input in sw #90: `config/twist_mux.yaml` (`manual`, timeout 0.25 s,
  priority 100), the release check of `safety_stop_gate_node.py` (input timeouts, `cmd_vel_joy`
  0.25 s), its publisher `ui_cmd_vel_relay_node.py` (from `/ui/cmd_vel`), and the tests
  `test_safety_stop_gate_node.py`, `test_stop_contract_full_chain.py`.
- `cmd_vel_teleop` has no publisher in any openAMRobot repository (org code search, 19 Nov 2026:
  one audits text and one upperbody-sw test fixture). It is the default input of Nav2's
  AssistedTeleop behaviour, registered in main and v0.0.1 `nav2_params.yaml` and removed by sw #90.
  The robot's teleop tool is outside the repositories; `ros2 topic info /cmd_vel_teleop -v` names it.
- Consequence: with sw #90 deployed, commands on `cmd_vel_teleop` reach no twist_mux input and
  the gate's quiet-input check does not see that source. On v0.0.1 they reach `/cmd_vel` only
  through an AssistedTeleop goal, around the collision monitor (2026-09-28 audit, nav health draft).
- Contract change needed (not in sw #90): a contract change request naming one manual teleop
  topic. Either the robot's teleop is remapped to `cmd_vel_joy` (no change to the gate, but the
  UI relay and teleop then share one mux input, which the owner must accept), or a new named input
  is added to `twist_mux.yaml` and to the gate's input list. The second touches the stop path:
  human-authored, two approvals including the platform lead. Either way one PR updates the
  launch remap, the configuration, the tests (failing on the old name), `docs/navigation` and
  STATE.md.
- Pass criterion: the change request and the merged PR recorded; on the robot `cmd_vel_teleop`
  has 0 publishers and the teleop publisher is on the contract topic.
