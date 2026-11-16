# ZLAC8015D drive and hub motor model (candidate, F1)

Status: Proposed (16 Nov 2026). A candidate model, not a specification of the drive. Fidelity F1
(datasheet-informed) at most. Development evidence only: nothing here shows how the real drive,
motor or brake behaves, and nothing here is safety evidence.

- Code: `ros2/src/openamrobot_sil/openamrobot_sil/models/drive_motor.py`
- Parameters: `ros2/src/openamrobot_sil/config/models/zlac8015d_motor.yaml` (each value with unit,
  source, confidence, uncertainty, hardware revision, owner role, test, date checked and the
  measurement-plan item that closes it)
- Tests: `ros2/src/openamrobot_sil/test/test_drive_motor.py` (pure Python, run in the `sil-tests`
  step "SIL protocol tests")
- Not wired: no launch file or Gazebo scene uses the model yet. Wiring is the next gate, after the
  clock contract (section 6 of `sil-topology.md`) is accepted.

## 1. Boundary

Gazebo (DART) owns the wheel motion. Per step the model receives the wheel angular velocity
(and optionally the wheel angle) of each wheel joint and returns the motor torque to apply to
that joint. It never integrates a wheel speed or angle itself. The step length is the delta of
the injected clock (`now()`, `generation`, `paused`, `openamrobot_sil/sim_time.py`); the model
reads no host time and adds no entry to `config/clock_inventory.yaml`.

Inputs: 6040h controlword, 60FFh:01/:02 targets, bus voltage, brake coil power per wheel, CAN
frame receipts (for 2000h), wheel speed and angle from the plant, fault injection.
Outputs: torque per wheel joint, 6041h, 603Fh, 606Ch, 6064h, 6077h, 2035h, brake state and
brake holding capacity per wheel, link-loss state.

## 2. Equations (per axis)

DC-equivalent winding, one per motor:

    V = R I + L dI/dt + Ke w        tau = Kt I

- The RL circuit uses its exact zero-order-hold discretisation over substeps of at most
  `max_substep`: `I+ = a I + (1 - a)(V - Ke w)/R`, `a = exp(-R h / L)`. It is stable for any step;
  the tests check steps of 0.1 ms and 2 ms against each other.
- Current loop: the drive's current PI (2019h, 201Ah) has no documented scaling, so it is replaced
  by an ideal current controller limited by the available voltage `|V| <= utilisation * Vbus`.
  When the back-EMF approaches that voltage the current, and with it the torque, falls: the bus
  voltage limits the top speed.
- Speed loop: PI on the ramped target, `I_ref = kp e + ki integral(e)`, saturated at the drive
  current limit (2015h), with conditional integration (no windup).
- Ramp: profile velocity mode, 6083h (acceleration) or 6084h (deceleration) as the time to
  change by `ramp_reference_speed`.
- Target: 60FFh in whole counts; one count is `1 / given_speed_resolution_setting` r/min.
- Disabled or faulted: zero torque, shaft free (2007h factory value 0).

## 3. Parameters and sources

Drive values come from the ZLTECH ZLAC8015D CANopen Communication Routine V1.07, a document
labelled V4.0 ([public mirror](https://github.com/kaver06/ZLAC8015D_CANOpen_Motor_Controller_simulator),
commit 41e2f57, Docs/Datasheets), and the ZLAC8015 Servo Driver Manual V1.01 in the same mirror.
The delivered drive is V4.2; its manual is not filed, so every drive value is a V4.0 value until
confirmed. Motor and brake values come from the actuator ledger (openamr-upperbody-sw PR #65),
which cites the ZLLG80ASM250-L-B2 drawing V1.01 and the ZLTECH written answer of 18 Oct 2026.

| Parameter | Value | Confidence | Source | Closes with |
|---|---|---|---|---|
| phase_resistance, phase_inductance | unknown | unknown | none on file | MP-09 |
| torque_constant | 0.85 N*m/A | estimate | 5.5 N*m / 6.5 A rated (16 / 19 peak gives 0.84) | MP-09 |
| back_emf_constant | 0.85 V*s/rad | estimate | equal to Kt in the model; 24 V / 27.227 rad/s gives 0.88 | MP-09 |
| current_limit | 30 A | datasheet | 2015h factory 300 x 0.1 A; manual 2.1 peak max 30 A | MP-09 |
| voltage_utilisation | unknown | unknown | modulation not documented | MP-09 |
| speed_kp, speed_ki (SI) | unknown | unknown | 201Dh = 500 and 201Eh = 100 raw, scaling not documented | MP-09 |
| profile_acceleration_time, profile_deceleration_time | 0.5 s | datasheet | 6083h, 6084h factory 500 ms | MP-09 |
| ramp_reference_speed | unknown | unknown | not documented (the emulator assumes 1000 r/min) | MP-09 |
| target_velocity_limit | 1000 r/min | datasheet | 60FFh range, 2008h | MP-09 |
| given_speed_resolution_setting | 1 | datasheet | 2026h factory 1 = 1 r/min | MP-09 |
| speed_resolution_subindex | 5 | datasheet, contradicted | 2026h object table; the version history says :06 | MP-09 |
| actual_velocity_unit | 0.1 r/min | datasheet | 606Ch | MP-09 |
| encoder_lines | 1024 | datasheet | ledger (BOM DRV-001); 200Eh factory 1024 | MP-09 |
| encoder_counts_per_rev | unknown | unknown | quadrature decoding not documented; 6064h reported as None | MP-09 |
| over_voltage_threshold, under_voltage_threshold | 60 V, 16 V | datasheet (typical) | manual V1.01 2.1 | MP-10 |
| link_loss_time | 0 (off) | datasheet | 2000h factory value; D-24 proposes 0.25 s | MP-10 |
| link_loss_reaction | ramp to 0 with 6084h, no fault | design_decision | vendored emulator; real reaction unknown | MP-10 |
| fault_high_half_axis | unknown | unknown | the routine contradicts itself | MP-10 |
| brake_release_time | 0.1 s (at most) | supplier_confirmation | ZLTECH 18 Oct 2026 | MP-11 |
| brake_engage_time | 0.03 s (at most, no diode; 0.1 s with a plain diode) | supplier_confirmation | ZLTECH 18 Oct 2026 | MP-11 |
| brake_holding_torque | 6 N*m (5 N*m earlier, D-25 open) | supplier_confirmation | ZLTECH 18 Oct 2026 | MP-11 |
| motor_temperature_threshold | 80 degC | datasheet | 2013h factory 800 x 0.1 degC (not used) | MP-09 |

Counts by confidence (41 parameters, 16 Nov 2026): datasheet 24, supplier_confirmation 3,
design_decision 4, estimate 2, unknown 8. The model refuses to run
while a required value is unknown; the tests pass explicit TEST values, labelled as such.

## 4. Drive behaviour modelled

- CiA 402 reduced: 06h ready, 07h switched on, 0Fh operation enabled (only from ready or switched
  on), other values switch on disabled, bit 7 fault reset.
- 603Fh: bits from routine section 3.7. Over-voltage, under-voltage and EEPROM error are common
  faults (low half). Over-current, overload, encoder tolerance, reference voltage, Hall, motor
  temperature and encoder error are per axis; the caller names the half. Faults latch; a fault
  reset clears them only when the cause is gone, then a new enable sequence is needed. A power
  cycle clears faults (manual 3.5).
- Over-voltage and under-voltage are also detected from the bus voltage input against the typical
  thresholds of manual V1.01. All other faults are injection only: `inject(name, half)`,
  `clear(name)`.
- 2000h communication loss: any frame to the node restarts the timer; expiry is measured on the
  injected clock, so a paused simulation never expires it. Factory value 0 keeps it off.
- Clock generation change (gate D3): targets dropped, ramp, current and integrator cleared, the
  drive returns to switch on disabled and needs a new enable. Latched faults stay.
- Brake: a plant-side delay. The coil input comes from outside; the state goes engaged,
  releasing (release time), released, engaging (engage time). Holding capacity is the full
  holding torque until released and zero until engaged. The model decides nothing about braking.

## 5. Not modelled

- Thermal state: no motor thermal resistance or time constant is on file. Over-temperature is an
  injected fault only; the 2013h threshold is recorded but unused.
- Overload function (2012h factor, 2016h time): semantics not documented; injection only.
- Quick stop (605Ah, 6085h), shutdown and disable options (605Bh, 605Ch): not modelled here; the
  vendored emulator keeps its own reduced version.
- The drive's speed estimation from the encoder, speed observer (2022h to 2025h), feedforward
  (201Fh), smoothing (2018h, 201Bh, 201Ch) and current PI: replaced by the reduced loop above.
- Commutation, Hall sensors, PWM ripple, torque ripple, cogging, iron losses, regeneration into
  the bus and the regen resistor (2027h), bus capacitance and inrush.
- Rotor inertia, friction and damping: the Gazebo plant owns them (RG-DRV-015).
- Brake torque build-up shape, brake wear, brake slip, a brake fault and who powers the coil.
- Left to right skew, SDO timing, PDOs, NMT and heartbeat: the vendored emulator covers the
  protocol side.

## 6. Reality-gap rows addressed

Register: openAMRobot/audits branch audit/2026-09-28, `digital-twin/phase1/reality_gap_register.csv`.
"Addressed" means the model now has a place for the behaviour with sourced or explicitly unknown
values. No row closes: closing needs the bench capture of the named item.

| Row | Effect | What the model does | Closes with |
|---|---|---|---|
| RG-DRV-008 | 60FFh unit and range | whole counts, range 1000 r/min, resolution setting | MP-09 |
| RG-DRV-009 | 606Ch unit, 2026h resolution | 0.1 r/min counts; :05 against :06 recorded as contradicted | MP-09 |
| RG-DRV-010 | encoder counts per revolution | 6064h from the plant angle once counts per revolution is known; None until then | MP-09 |
| RG-DRV-011 | axis order and sign | axis signs as SIL parameters | MP-09 |
| RG-DRV-012 | acceleration and deceleration ramp | 6083h, 6084h ramp; reference speed unknown | MP-09 |
| RG-DRV-014 | torque and current limits, torque-speed curve | current limit, back-EMF top speed against bus voltage | MP-09 |
| RG-DRV-016 | holding brake delays and torque | brake states with release and engage delays, holding capacity | MP-11 |
| RG-DRV-017 | drive input range and voltage trips | typical 60 V and 16 V from manual V1.01, detected from the bus input | MP-10 |
| RG-DRV-025 | battery sag feeding the drive | bus voltage is an input (no pack model here) | MP-12 |
| RG-FLT-012 | reaction to communication loss (2000h) | timer on the injected clock; reaction recorded as a design choice | MP-10 |
| RG-FLT-015 | 603Fh codes and fault reset | full bit list, latch, reset, halves without wheel meaning | MP-10 |
| RG-SEN-026 | wheel encoder resolution and direction | quantised 606Ch and 6064h; resolution unknown | MP-09 |
| RG-TIM-007 | encoder feedback path in SIL | feedback derived from the plant state each step, no own ramp | MP-09 |

Not addressed: RG-DRV-013 (quick stop), RG-DRV-015 (friction, damping, inertia: Gazebo),
RG-FLT-013, RG-FLT-014, RG-FLT-016, RG-FLT-017, RG-TIM-012 and RG-COM-009 to RG-COM-015
(protocol, bus and timing rows belong to the emulator and the bench).

## 7. Measurement-plan items

From `digital-twin/phase1/measurement_plan.md` in the same audits branch:

- MP-09 (wheel drive on CAN1, wheels up): winding R and L, Kt and Ke, voltage utilisation, speed
  gains, ramp reference speed, encoder counts per revolution, 2026h sub-index, axis signs.
- MP-10 (wheel drive faults and boot): voltage trip thresholds, 2000h reaction, 603Fh halves,
  fault reset with a standing cause.
- MP-11 (brake and stop-chain timing): brake release and engage delays, holding torque.
- MP-12 (power bus): bus voltage under load.
