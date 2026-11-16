# Lift drive model (candidate, F1)

Status: candidate, Proposed. Simulation evidence only (L2: unit tests on a stand-in plant). Not
evidence of real-robot, mechanical, electrical or safety behaviour.

Code: `ros2/src/openamrobot_sil/openamrobot_sil/models/lift_drive.py`. Parameters:
`ros2/src/openamrobot_sil/config/models/lift_drive.yaml`. Tests:
`ros2/src/openamrobot_sil/test/test_lift_drive.py` (run by the `sil-tests` step "SIL protocol
tests"). Not wired into any launch file or scene yet.

The model covers the DOLD Hexalift V4 350 mm column (BS-HM240G350530-MFF24) driven by the
upper-body controller CTL-017 (NUCLEO-G474RE with an Infineon BLDC-SHIELD_IFX007T, BOM Issue 7.15
row CTL-017, board choice still to be confirmed by the firmware owner).

## 1. Ownership

- Gazebo DART owns the motion of `lift_joint`. The model never integrates the lift position. Each
  `step(position, velocity)` takes the joint state from the plant and returns the axial effort on
  the carriage (N, positive up) and the observables: armature voltage and current, reported current
  in mA, Hall count, Hall supply flag, stop sensor bits, end switch states.
- The CTL-017 controller owns every decision (moves, stops, faults, permit). The model takes the
  controller's bridge output (enable, signed PWM duty) and the supply voltage as inputs. It holds
  no E-stop, brake, contactor, watchdog, motor-enable or permit logic. Motor power removal by the
  hardwired chain is an input (`set_bus_voltage(0)`).
- Time comes only from the injected clock of `openamrobot_sil/sim_time.py` (`now()`,
  `generation`, `paused`), clock contract D2 in `sil-topology.md` section 6. A test fails if the
  model reads host time.

## 2. Equations

Averaged H-bridge and armature, reduced to the carriage axis. The rotary constants Ke, Kt, gear
ratio N and spindle lead are not known separately, so the model uses the lumped constant
K = Ke 2 pi N / lead (V s/m, equal to N/A for Kt = Ke):

```
V_a  = d * V_bus                              d in [-1, 1], positive drives up
V_a  = R_tot I + L dI/dt + K v                R_tot = R_winding + R_bridge path
|I| <= min(I_supply_limit, I_bridge_limit)    10 A converter limit, 55 A IFX007T minimum
F_m  = K (I - I_f)                            motor force at the nut, I_f = no-load current
raising:  F_m = F_ax / eta                    forward efficiency
lowering: F_m = eta_b F_ax                    eta_b = 2 - 1/eta (small-angle screw relation)
F_ax = W + F_ext + sign(v) (F_c + c |v|)      W = (m_carriage + m_arms + m_payload) g
self-locking  <=>  eta_b < 0  <=>  eta < 0.5
```

Constants from the anchors (section 3):

```
K     = (V_rated - R_tot I_0) / v_0                  = 1277 V s/m
eta K = F_rated / (I_rated - I_0)                    = 133.3 N/A,  eta = 0.104
I_f   = I_0                                          = 1.0 A
eta_b = 2 - 1/eta                                    = -7.58 (self-locking)
```

Drive speed. For the present armature voltage and axial load the model solves the steady state
of the two equations above for the speed v_d (raising, lowering self-locking, lowering
back-drivable, lowering against friction). If the needed current exceeds the current limit the
drive cannot move: v_d = 0 (stall).

Effort to Gazebo. The spindle is stiff; the model couples the carriage to v_d through a numerical
damper b (not a physical value):

```
effort = W + F_ext + b (v_d - v)
clamped to [-F_pull_rated, F_holding], and while driving up to eta K (I_limit - I_f) - F_c
```

With the transmission self-locking and no drive (bridge off, duty 0 or power removed) v_d = 0:
the carriage is held. A back-drivable transmission without drive resists only with
min(W, K I_f / eta_b + F_c): the load lowers if it exceeds that.

Current. The reported current is the armature current at the measured joint speed,
I = (V_a - K v) / R_tot, clamped to the limit and filtered with L / R_tot when L is known. A
blocked carriage (jam, end of travel) shows the stall current V_bus / R_tot, limited to 10 A.
Bridge off: terminals open, I = 0. Reported value: |I| * kILIS_typ / kILIS_actual in whole mA
(TPDO2 format).

Stability. b dt / m must stay below 1 for a 1 ms Gazebo step: with b = 10 000 N s/m and the
19 kg carried mass it is 0.53. A mass difference between the parameter file and the Gazebo links
gives a speed error of delta m g / b (about 1 mm/s per kg).

Sensing. Hall count increments with the joint motion, floor(count), sign per `0x2101:02`; lost
pulses stay lost (as in the vendored emulator). Stop sensors switch within +-1 mm of their
positions, release with an optional hysteresis. End switches are active at or beyond their
positions.

Reset (gate D3). A clock generation change drops the motion command (duty 0, bridge off) and the
current, and re-bases the Hall count on the new position, so the reset jump is not counted as
motion. Motion needs a new command in the new generation. While the clock is paused the model
time does not advance.

## 3. Parameters and sources

Every entry in the YAML file has value, unit, source, confidence, uncertainty, hardware revision,
owner role, test, last verified (against its source, never against hardware) and measurement item.

| Parameter | Value | Confidence | Source | MP |
|---|---|---|---|---|
| Bus voltage | 24 V | design_decision | E-01 rev1.8, SAFE 24 V converter DDR-240B-24 | MP-12 |
| Supply current limit | 10 A | design_decision | E-01 rev1.8 ("limit output current to 10 A or less") | MP-12 |
| Bridge current limit | 55 A | datasheet | [IFX007T data sheet Rev 1.0](https://www.infineon.com/dgdl/Infineon-IFX007T-DS-v01_00-EN.pdf), min of 55 / 77 / 98 A | |
| Bridge path resistance | 10 mohm | datasheet | IFX007T, path RDS(on) typ at 25 C | |
| Current sense ratio | 19 500 | datasheet | IFX007T, dkILIS 15 000 / 19 500 / 24 000 | MP-14 |
| Current report resolution | 1 mA | design_decision | CTL-017 OD v1, TPDO2 motor_current | |
| PWM frequency | 25 kHz | estimate | [Motor Control Shield with IFX007T user manual](https://www.farnell.com/datasheets/2792904.pdf), example only; not used | |
| Rated voltage | 24 V | datasheet | BOM row MEC-018, actuator ledger | |
| No-load speed v_0 | 0.018 m/s | datasheet | BOM row MEC-018 | MP-14 |
| Maximum current | 7 A | datasheet | E-01 rev1.8, catalogue maximum | MP-14 |
| Rated push / pull | 800 N / 400 N | datasheet | BOM row MEC-018 | |
| Holding force, self-locking | 800 N, true | supplier_confirmation | written supplier confirmation of 14 Oct 2026 (E-01 rev1.8) | MP-05 |
| No-load current I_0 | 1.0 A | estimate | same as the emulator placeholder `MOVING_CURRENT_MA` | MP-14 |
| Current at rated push | 7 A | estimate | assumption: maximum current at rated push | MP-14 |
| Winding resistance | 1.0 ohm | estimate | no source; speed at 800 N about 74 % of v_0 | MP-14 |
| Winding inductance | null | unknown | fallback 0 | MP-14 |
| Gear ratio, spindle lead | null | unknown | not in any input; only K is used | MP-14 |
| Back efficiency | null | unknown | fallback 2 - 1/eta | MP-05 |
| Static, viscous friction | null | unknown | fallback 0 (losses in I_0) | MP-05 |
| Coast distance | null | unknown | fallback: no coast | MP-05 |
| Carriage link mass | 6.721 kg | cad | mass export `lift_carriage_link`; believed to include the 2.81 kg moving stage estimate | MP-01 |
| Arms | 12.26 kg | estimate | 2 x 6.13 kg upstream nominal | MP-01 |
| Payload | 0 kg | design_decision | scenario input | |
| Hall pitch | 0.2092 mm | estimate | OD v1 `0x2101:01` and emulator `MM_PER_HALL` | MP-14 |
| Hall count sign | up positive | design_decision | OD v1 `0x2101:02` | MP-14 |
| Stop sensors | 0, 0.175, 0.350 m | design_decision | OD v1 `0x2100:01..03` less the 5 mm joint zero; L1000, L1175, L1350 | MP-14 |
| Stop sensor window | +-1 mm | estimate | emulator `SENSOR_WINDOW_MM` | MP-14 |
| Stop sensor hysteresis | null | unknown | [IME12-04BNSZC0S data sheet](https://motionworld.com/assets/SICK-IME12_04BNSZC0S-Datasheet.pdf): 5 to 15 % radial; along travel unknown | MP-14 |
| End switches | -0.005, 0.355 m | estimate | emulator `TRAVEL_MM` 360; catalogue stroke 350 mm | MP-14 |
| End switch cuts motor | null | unknown | fallback false (stall) | MP-14 |
| Drive coupling b | 10 000 N s/m | design_decision | numerical, section 2 | |

Controller board, for reference only (not modelled): STM32G474RE, Cortex-M4 up to 170 MHz, HRTIM
with 184 ps resolution, five 12-bit ADCs, three FDCAN
([ST product page](https://www.st.com/en/microcontrollers/stm32g474re.html)). Shield: three
IFX007T half bridges, 8 to 24 V nominal, 30 A average limit from the PCB, 250 W brushed DC
(user manual above).

Model results at 24 V with the default parameters (candidate values, not measurements):

| Axial load | Raising speed, current | Lowering speed, current |
|---|---|---|
| 0 N | 18.0 mm/s, 1.0 A | 18.0 mm/s, 1.0 A |
| 186 N (carriage and arms) | 16.9 mm/s, 2.4 A | 17.1 mm/s, 2.1 A |
| 800 N | 13.3 mm/s, 7.0 A | 14.2 mm/s, 5.7 A |
| above 1200 N | stall, 10 A | |

At the carried load the Hall rate is about 81 counts/s, inside the CTL-017 move window of 43 to
130 counts/s; the homing duty of 50 % (down) gives about 37 counts/s, inside 15 to 86.

## 4. Fault conditions F01 to F09

The fault definitions are those of the vendored CTL-017 emulator and the v1 object dictionary
(openamr-upperbody-fw PR #15). The vendored emulator is not changed. `inject(name)` applies the
plant condition, `inject('F0n')` the canonical condition for that fault, `clear(name=None)` removes
one or all. The tests show the plant observable crossing the dictionary default threshold; they
do not run the controller.

| Fault | Emulator definition | Plant condition (`inject`) | Observable and threshold |
|---|---|---|---|
| F01 | both end switches active | `end_switch_short` | both limit inputs true |
| F02 | Hall count inconsistent | `hall_reversed`, `hall_rate` (pct), `hall_slip` (m, default 4 mm) | more than 10 counts against the direction; rate below 43 counts/s; more than 2 mm cross-check error at a stop sensor edge |
| F03 | over-current or stall | `overload` (force_n, default 1000 N), `jam`, `hall_loss` | above 7000 mA for 100 ms after 200 ms blanking; no Hall pulse for 300 ms while driving |
| F04 | travel timeout | `undervoltage` (volts, default 15 V) | 175 mm takes longer than 1.5 x 175 mm / 18 mm/s + 2 s while the rate stays inside the window |
| F05 | Hall supply lost while moving | `hall_supply_loss` | supply flag false, no pulses, carriage still moving |
| F06 | invalid stored record, unused in v1 | none: `inject('F06')` raises | |
| F07 | keep-alive or heartbeat missing during a move | `keepalive_loss` | flag only, for the CAN layer; the plant is unchanged |
| F08 | standstill not confirmed 500 ms after drive off | `back_drive` (creep_m_s, default 2 mm/s) | counts still change in the 100 ms quiet window before 500 ms |
| F09 | two or more stop sensors active | `stop_sensor_short` (index) | two stop sensor bits set |

Motor power removed during a move (RG-FLT-019) is not a fault in the emulator. In the model the
current drops to 0 and the self-locking transmission holds the carriage; a back-drivable
transmission lowers.

## 5. Not modelled

- PWM ripple, dead time, slew rate, the IFX007T chopping current limitation and its fault flag,
  overtemperature and undervoltage shutdown of the bridge. The bridge is averaged.
- Inrush and start-up transients beyond the optional L / R lag; rotor and spindle inertia; the
  converter's behaviour above its current limit (modelled as a hard clamp) and its voltage droop.
- Thermal behaviour of the motor and the bridge, duty cycle limits of the column.
- Hall signal form (type, quadrature, edges), Hall supply wiring across an E-stop (RG-DRV-036),
  ADC quantisation and filtering of the current sense, the sense ratio of the delivered part.
- Axial play, reversal hysteresis in the Hall count, stiffness of the telescopic stages, energy
  chain forces, coast after drive off (RG-DRV-022, RG-MEC-022, RG-MEC-023, RG-MEC-030).
- Stop sensor switching frequency (2 kHz), repeatability and temperature drift.
- Brake and coast stop modes of the bridge (`0x2101:31`): both stop at once with a self-locking
  spindle.
- Any controller behaviour: thresholds, latching, recovery, permit, supervision.

## 6. Reality gap rows and measurement items

Rows of the reality gap register (audits branch `audit/2026-09-28`,
`digital-twin/phase1/reality_gap_register.csv`) that this model addresses, and what closes them.

| Row | Effect | Model after this change | Closes with |
|---|---|---|---|
| RG-DRV-018 | Lift speed under load | speed against load from the motor equation, candidate constants | MP-14: stopwatch and Hall count over the stroke with the upper body |
| RG-DRV-019 | Hall signal type and counts per mm | 0.2092 mm per count, estimate | MP-14: scope on both Hall lines over a measured stroke |
| RG-DRV-020 | Lift motor current | current from load and speed instead of a 1000 mA constant | MP-14: current probe up and down with load, and a blocked start |
| RG-DRV-021 | Stroke, stop positions, sensor window | parameters, emulator test values | MP-14: height gauge at each switch point, up and down |
| RG-DRV-022 | Holding with power off, coast | self-locking hold, no coast | MP-05: stop and power cut during motion with load |
| RG-DRV-035 | SAFE 24 V converter loading | lift current up to the 10 A limit | MP-12: current at 1 kHz over a stroke, converter voltage at start |
| RG-MEC-022 | Friction, backlash, self-locking | self-locking from the efficiency; friction null | MP-05: power-off hold test, axial play, reversal hysteresis |
| RG-FLT-018 | Fault set F01 to F09 | plant condition per fault, F06 has none | MP-14: stall, sensor disagreement and keep-alive loss on the real column |
| RG-FLT-019 | Motor power removed during a move | current 0, carriage held | MP-14: E-stop during motion with load |
| RG-SEN-025 | IME12 stop sensors | switching window and optional hysteresis | MP-14: switch point and hysteresis against the flag at 18 mm/s |
| RG-COM-019 | CAN3 bit rate and node id | not touched: the model has no bus | MP-14: CAN capture of boot-up |
| RG-TIM-011 | Lift protocol timers against the plant clock | the model runs on the injected clock | wiring the model and CTL-017 onto Gazebo time (clock contract C1) |

## 7. Next steps

1. Owner review of the candidate constants (firmware owner for the electrical values, mechanical
   owner for masses and the transmission).
2. Wire the model between `GazeboCtl017` and an effort interface on `lift_joint` in a separate
   change, with a check that the carried mass equals the Gazebo link masses.
3. Replace the estimates with MP-14, MP-05 and MP-01 results as the column arrives.
