# SIL battery and power model: lumped 8S LiFePO4 pack and power distribution (proposal)

Status: proposal for the owners (power and BMS owner; SIL owner). Not a validated model and
never safety evidence: the electrical model is lumped (decision D11).
Package: `openamrobot_sil`. Modules `openamrobot_sil/battery_model.py` (pack, charger, drive
load estimate, emulator glue), `openamrobot_sil/power.py` (buses, rails, converters, switch
states, energy) and `openamrobot_sil/model_params.py` (parameter loader and schema check).
Parameters: `config/power_model.yaml`. Run settings and scenario inputs: `config/sil.yaml`
section `bms.model`. Tests: `test/test_battery_model.py`, `test/test_power_model.py`.

## Why

Before this change the Daly BMS emulator reported fixed values and no layer tied drive, arm or
lift current to pack voltage, current or SOC (RG-DRV-023, RG-DRV-024, RG-DRV-025). No layer had
a rail, a converter or a precharge (RG-DRV-026, RG-DRV-031, RG-DRV-035, RG-SEN-001). Drive and
arm models need a bus voltage to read.

## Time and reset

`PackSimulation.tick()` takes `dt` from an injected clock object with `now()`, `generation` and
`paused` (the clock contract in [sil-topology.md](sil-topology.md)). No model reads host time.
A paused clock or a clock that did not advance integrates nothing. A new generation (simulation
reset, gate D3) returns the pack SOC, charger, injected current, drive estimate, switch states,
rail loads, converter states and energy counters to their configured initial state. Until the
clock contract is merged, the node passes a duck-typed host clock (generation 0, never paused);
the models see only the object.

## Pack: equivalent circuit

One lumped pack, eight cells in series (8S1P EVE LF105, BOM PWR-001):

    V_terminal = n * OCV_cell(SOC) - I * R_series         I > 0 discharges
    SOC(t + dt) = SOC(t) - 100 * I * dt / (3600 * C_Ah)   coulomb counting, clamped 0..100
    R_series   = n * R_cell + R_busbars + R_cable + R_bms_path,   R_copper = rho * L / A

Loads are a constant-current part `I0` plus a constant-power part `P`, solved together:
`V^2 - (OCV - R*I0) V + P*R = 0`, upper root. Above the maximum power point the voltage
collapses to half the source voltage.

One RC pair is not modelled: no source gives its parameters. The LF105 specification gives a
DC resistance of at most 1.8 mOhm for a 10 s, 1 C pulse at 50 % SOC and an AC resistance of
0.32 +/- 0.05 mOhm at 1 kHz. The model uses the 1.8 mOhm upper bound as the series resistance,
so an instant step sags more than the cell alone would and the slow recovery is absent.
Temperature is out of scope: no source gives resistance or capacity against temperature for
the delivered cells.

Bench charger: constant current, then constant voltage at the charger terminals
(`I <= (V_charger - OCV) / R + I_load`); it stops at the taper current or when the counter is
full. Without a polarisation branch the taper does not appear, so the charge time equals
capacity over charger current.

## Power distribution

Topology after E-01 rev1.8 section 3. Rail names follow the component inventory; there is no
12 V rail (the inventory lists none, and the compute carrier is never fed from 12 V).

    pack -> F0 -> disconnect -> BAT bus
      BAT bus -> compute branch (F-J401 10 A, direct)
      BAT bus -> F-5V 5 A  -> RSD-60G-5   -> 5 V rail (STM32, RPLIDAR S3, near-field sensors, IMU)
      BAT bus -> F-24V 7.5 A -> RSD-100B-24 -> regulated 24 V rail (PNOZ, coils, lamp, panel, hub, fan)
      BAT bus -> charge branch (F-CHG) <- bench charger
      BAT bus -> K1 -> K2 (Kpre + resistor + 1 A fuse across K2) -> SAFE bus
        SAFE bus -> traction (F-TRAC 40 A, ZLAC8015D)
        SAFE bus -> left arm, right arm (F-ARM-L, F-ARM-R 30 A, direct; gate ARM-VOLT)
        SAFE bus -> F-LIFT 15 A -> DDR-240B-24 -> SAFE 24 V (lift, brake release)

Converter, per step:

    P_in = V_out * I_out / eta,   I_in = P_in / V_in        (constant-power load on its bus)
    allowed = peak_fraction * I_rated   for the first peak_s above I_rated,
              limit_fraction * I_rated  after that
    I_out = min(I_demand, allowed);  V_out = V_nom * allowed / I_demand when limited

The output is off below `input_min_v` at once (hold-up is not modelled) and comes on
`setup_s` after the input reaches `input_on_v` (rise time not modelled; one step of lag).

SAFE bus: with K1 and K2 closed it equals the BAT bus. With K1 and Kpre closed and K2 open:

    V_safe(t + dt) = V_safe + (V_bat - V_safe) * (1 - exp(-dt / (R_pre * C_safe)))
    I_pre = (V_bat - V_safe) / R_pre

SAFE loads are served only with K1 and K2 closed. When the SAFE bus is cut it is set to 0 V
(discharge through the loads is not modelled). `safe_step_v` reports the voltage step when K2
closes, which is the inrush that precharge exists to limit.

`bus_voltage('bat' | 'safe')` is the interface a drive or arm model reads.
`energy()` and `take_interval_energy()` give energy per account (pack output, pack loss,
charger input, precharge loss, each converter's output and loss, each direct branch).

### Switches are inputs, never decisions

The disconnect, the BMS discharge and charge switches, K1, K2 and Kpre are states set from
outside (`PowerModel.set_switch`, `/sil/bms/inject switch <name> open|closed`; the `mos`
injection sets the BMS switches and the reported MOS flags together). They take effect after
the documented operate and release times. The model contains no E-stop, PNOZ, contactor,
precharge sequencing, brake, motor-enable, watchdog or charge-inhibit logic: a deep discharge,
an overload or a fuse rating exceeded raises a flag and nothing else. Flags: per converter
`input_undervoltage`, `input_overvoltage`, `output_on`, `overload`, `current_limited`,
`over_fuse_rating`; per branch `undervoltage`, `over_fuse_rating`; `bat_energised`,
`safe_energised`, `precharge_over_fuse_rating`, `pack_over_continuous_rating`. A fuse rating
flag is not a trip: no trip curve is on file.

## Parameters

Every parameter in `config/power_model.yaml` has `value, unit, source, confidence,
uncertainty, hardware_revision, owner, test, last_verified, measurement_item`. Confidence is
one of datasheet, supplier_confirmation, design_decision, cad, estimate, unknown. A parameter
without a robot value has `value: null`, a measurement item and a `sil_default` that the SIL
runs with; `defaults_in_use` in the state output lists them. `test_power_model.py` checks the
schema and that every named test exists. `last_verified` is null everywhere: nothing has been
measured on the robot's own parts.

| Parameter | Value | Confidence | Source |
|---|---|---|---|
| Series cells | 8 | design_decision | BOM Issue 7.15 PWR-001 |
| Capacity | 105 Ah | datasheet | EVE LF105 specification PBRI-LF105-D06-01 rev D ([copy](https://www.rcscomponents.kiev.ua/datasheets/PBRI-LF105-D06-01-2022-1.pdf)); E-01 rev1.8 says 84 Ah (open conflict) |
| Cell OCV against SOC | null (SIL table) | unknown | No OCV table in the LF105 specification (rev D, rev F). Placeholder LiFePO4 shape, 2.50 to 3.45 V |
| Cell cut-off, discharge and charge | 2.5 V, 3.65 V | datasheet | LF105 rev D |
| Cell continuous rating | 1 C | datasheet | LF105 rev D (flag only) |
| Cell DC resistance | 1.8 mOhm | datasheet | LF105 rev D, DCR upper bound (10 s, 1 C, 50 % SOC, 25 C) |
| Busbars, cable | 8 x 56 x 20 x 2 mm Cu; 0.72 m of 25 mm2 | estimate; design_decision | Mass ledger PWR-024 estimate; BOM CBL-001 |
| BMS path (MOSFETs, F0, disconnect, tray link, lugs) | null (5 mOhm) | unknown | BMS not ordered, no datasheet |
| Bench charger | 28.4 V, 20 A | design_decision | E-01 rev1.8, BOM PWR-005 |
| Charger taper | null (2 A) | unknown | |
| Robot mass | 86.3 kg | design_decision | Mass ledger D-26 |
| Rolling resistance, drive efficiency, regen fraction, drive standby | null (0.02, 0.70, 0, 10 W) | unknown | Torque-speed curve and ZLAC input data not on file |
| RSD-60G-5 | 9 to 36 V in, 5 V 12 A 60 W, eta 0.89, limit 105 to 135 %, setup 100 ms | datasheet | MEAN WELL RSD-60 specification ([copy](https://pdf.elecfans.com/MEANWELL/RSD-60G-24.html)) |
| RSD-100B-24 | 16.8 to 31.2 V in, 24 V 4.2 A 100.8 W, eta 0.89, limit 105 to 135 %, setup 800 ms | datasheet | MEAN WELL RSD-100 specification 2013-04-26 ([copy](https://nonprod.docs.rs-online.com/05dc/0900766b80ef2584.pdf)); efficiency also [octopart](https://octopart.com/part/mean-well/RSD-100B-24) |
| DDR-240B-24 | 16.8 to 33.6 V in, UVLO on 16.8 / off 16.5 V, 24 V 10 A 240 W, eta 0.90, 150 % for 3 s then 105 to 135 %, setup 500 ms | datasheet | MEAN WELL DDR-240 specification ([copy](https://manuals.plus/mean-well/ddr-240b-24-240w-din-rail-type-dc-dc-converter-manual)); datasheet index URL https://www.meanwell.com/Upload/PDF/DDR-240/DDR-240-SPEC.pdf |
| Fuses F-5V, F-24V, F-LIFT, F-J401, F-TRAC, F-ARM-L/R, precharge | 5, 7.5, 15, 10, 40, 30, 1 A | design_decision | E-01 rev1.8, E-01 BOM rev1.2 (ratings, no trip curves) |
| Compute input minimum | 19 V | design_decision | Component inventory CMP-002 (carrier SKU to confirm) |
| Traction and arm input minimum | null (0 V: no flag) | unknown | ZLAC8015D V4.2 input rating open; gate ARM-VOLT |
| SAFE bus capacitance | 940 uF +/- 10 % plus null (0) | supplier_confirmation; unknown | ZLTECH via E-01 rev1.8; arm and converter capacitance not measured |
| Precharge resistor | null (47 Ohm) | unknown | Sized by the power owner (D-22). The SIL value keeps the start current (29.2 V / 47 Ohm = 0.62 A) below the 1 A precharge fuse; it is not a design value |
| Contactor operate and release | null (0 s) | unknown | GX11CAC data not read into the twin |

Parameters by confidence: 36 datasheet, 1 supplier_confirmation, 24 design_decision, 0 cad,
5 estimate, 14 unknown (80 in total; `Params.by_confidence()`). The 14 unknown run on their
`sil_default`.

The MEAN WELL site was not reachable from the session that wrote this; the values were read
from the copies named and need a check against the files in the datasheet index.

Conflicts kept open, not resolved here:
- Capacity 105 Ah (BOM PWR-001, LF105 specification) against 84 Ah (E-01 rev1.8 charging
  paragraph, RG-DRV-023).
- Regulated 24 V converter position: on the BAT bus (E-01 rev1.8, used here) against "behind
  K1/K2" (BOM Electrical sheet COIL row). Brake coils: SAFE 24 V (E-01 rev1.8, used here)
  against the 24 V rail (BOM Rail loads sheet).
- RSD-100B-24 input range: 16.8 to 31.2 V (2013 sheet) against 14.4 to 33.6 V (distributor
  listing); revision of the ordered unit unknown.

## Drive current estimate

The ZLAC8015D emulator has no torque or phase current. The estimate uses the wheel speeds of
606Ch (`/sil/drive/emulator_state`, `actual_rpm`) at the injected clock time:

    a_i     = r * d(omega_i)/dt
    F_i     = m/2 * (a_i + g * c_rr * sign(v_i))
    P_mech  = sum_i F_i * r * omega_i
    P_elec  = P_mech / eta (P_mech >= 0) or P_mech * regen_fraction (P_mech < 0), plus standby

No hub motor or yaw inertia, slope, castor scrub or speed-dependent efficiency. It is an
order-of-magnitude load on the SAFE bus (`traction_w`), not a motor model.

## Not modelled

Cell imbalance and minimum cell, temperature, RC relaxation, hysteresis, ageing,
self-discharge, BMS protection thresholds and the BMS deciding its own disconnect (the switch
state is an input: RG-DRV-037, RG-FLT-020), converter rise and hold-up curves, converter
light-load efficiency, fuse trip curves, SAFE bus discharge, inrush current into the SAFE bus
when K2 closes without precharge (only the voltage step is reported), regeneration onto the
SAFE bus and the regen clamp (RG-DRV-027), grounding and isolation (RG-DRV-034), the lift
stop-sensor and Hall supply across an E-stop (RG-DRV-036), compute boot (RG-BOO-013), BMS
wake-up and protocol timing (RG-BOO-012, RG-COM-016, RG-COM-017), the Daly current sign
(RG-COM-018, configured `current_sign_discharge`). The Daly protocol stays as it is,
unconfirmed by the supplier.

## Reality gap rows and measurements

| Row | What this change gives | Closed by |
|---|---|---|
| RG-DRV-023 | Pack voltage, current and SOC follow the rail and drive loads | MP-12 drive and charge log |
| RG-DRV-024 | Bus voltage follows SOC (placeholder OCV table) | MP-12 discharge curve of the delivered pack |
| RG-DRV-025 | Series resistance from the LF105 DCR bound, copper and an unknown BMS path | MP-12 step-load test 0, 20, 60, 80 A |
| RG-DRV-026 | Precharge RC on the SAFE bus from externally set K1, K2, Kpre; 940 uF supplier value | MP-12 scope of SAFE bus and K2 current; D-22 sizing |
| RG-DRV-027 | Not modelled (regen fraction 0) | MP-12 stops from 0.2, 0.3, 0.5 m/s |
| RG-DRV-031 | Converter UVLO and setup times from the datasheets; no hold-up | MP-12 rail captures at power-on, reset, E-stop |
| RG-DRV-034 | BMS discharge switch as an input drops every rail | MP-12 bench: open the discharge MOS |
| RG-DRV-035 | DDR-240B-24 rating, peak and current limit flags | MP-12 lift current and converter output |
| RG-DRV-036 | Not modelled | MP-12 |
| RG-DRV-037 | Thresholds not modelled; switch state is an input | MP-13 discharge to cut-off |
| RG-COM-016, 017, 018 | Unchanged: protocol provisional, sign configured | MP-13 CAN2 captures |
| RG-FLT-020 | Brown-out of every rail on an open BMS switch (input) | MP-13 provoked protection |
| RG-BOO-012 | Not modelled | MP-13 |
| RG-BOO-013 | Compute branch undervoltage flag only | MP-12 cold boots |
| RG-SEN-001 | 5 V rail load as an input (RPLIDAR S3 running current) | MP-12 5 V rail at power-on |

## Using it

`bms.model.enabled: true` runs the pack model in the `daly_bms_emulator` node every `period_s`,
with `dt` from the clock; `bms.model.power.enabled: true` adds the power model. Injection on
`/sil/bms/inject`:

| Command | Effect |
|---|---|
| `step <A>` | extra constant BAT bus current; `step 0` ends it |
| `charge on`, `charge off` | bench charger connected or not |
| `soc <percent>` | sets the model SOC (and the reported SOC) |
| `model off`, `model on` | freeze or resume the model |
| `switch <name> open\|closed` | input state of `disconnect`, `bms_discharge`, `bms_charge`, `k1`, `k2`, `kpre` |
| `load <name> <value>` | rail load: `compute_w`, `rail_5v_a`, `rail_24v_a`, `safe_24v_a`, `arm_left_a`, `arm_right_a` |
| `mos <charge> <discharge>` | reported MOS flags and the BMS switch inputs together |
| `voltage <V>`, `current <A>` | fixed values; they switch the model off so the value stays |

`/sil/bms/emulator_state` carries the pack keys (`ocv_v`, `resistance_ohm`, `model_current_a`,
`drive_w`, `step_a`, `charger_on`, `charge_done`, `net_discharged_ah`, `model_enabled`), the
clock keys (`clock_generation`, `model_resets`) and `power` (bus and rail voltages, switches,
loads, flags, energy, `defaults_in_use`).

## Scenarios in the tests

The two CAN2 scenarios run the emulator and the host poll (`BmsHost`) on vcan; without vcan
they fail with `BLOCKED:`. `SIL_TEST_BUS=virtual` runs them on the python-can in-process bus
as a logic check, which is not vcan evidence.

1. Low pack voltage under a current step: SOC 3.5 %, then a 60 A step. The host reads a sag
   below 8 x 2.8 V (a commissioning candidate in BOM Decisions D65, not an approved setting)
   and the recovery after the step.
2. Charge from a low state: 20 A with the opposite sign, terminal voltage at most 28.4 V, full
   after 0.95 x 105 / 20 = 4.99 h of model time, then zero current.
