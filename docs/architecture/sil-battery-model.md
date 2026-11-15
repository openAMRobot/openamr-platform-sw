# SIL battery model: lumped 8S LiFePO4 pack behind the Daly BMS emulator (proposal)

Status: proposal for the owners (power and BMS: Evans; SIL: Mohamed). Not a validated model.
Package: `openamrobot_sil`, module `openamrobot_sil/battery_model.py`, configuration
`config/sil.yaml` section `bms.model`, tests `test/test_battery_model.py`.

## Why

Before this change the Daly BMS emulator reported fixed values. The real pack sags under drive
and arm current, and the arms see 25.6 to 29.2 V on the SAFE bus (E-01 rev1.8). Low-voltage
behaviour of the host and the arm supply range could not be exercised in SIL.

## What is modelled

One lumped pack, eight cells in series (8S1P EVE LF105, BOM PWR-001, M-01 rev19.10):

    V_terminal = 8 * OCV_cell(SOC) - I * R_series        I > 0 discharges
    SOC(t + dt) = SOC(t) - 100 * I * dt / (3600 * C_Ah)  coulomb counting, clamped 0..100

Loads are a constant-current part (arms, injected step) plus a constant-power part
(electronics, drive estimate). For a constant-power load the voltage and the current are
solved together (`V^2 - (OCV - R*I0) V + P*R = 0`, upper root). Above the maximum power point
the voltage collapses to half the source voltage; the scenario tests do not go there.

The model output goes into the emulator's existing frames: 0x90 (pack voltage, current,
SOC), 0x91 (cell min and max, pack voltage / 8, as before), 0x93 (remaining capacity from
SOC). The frame layout and the decoder status are unchanged: the Daly protocol is still
provisional for the delivered unit (platform-fw PR #51 decoder, `daly_bms.py` docstring).

## Not modelled

Cell imbalance and the minimum-cell value, temperature, relaxation and diffusion (RC
branches), hysteresis between charge and discharge OCV, ageing, self-discharge, BMS protection
and its own disconnect, charge inhibit, the KCHG relay and the PNOZ, and any decision the
firmware takes on low voltage. These are out of scope by the work package; nothing here is a
safety function.

Because there is no polarisation branch, the constant-voltage taper of a real charge does not
appear: the charge ends when the coulomb counter reaches 100 percent (or below the taper
current, which this model does not reach). Charging time therefore equals capacity over
charger current.

## Parameters

Tags: [M] manufacturer or project document; [E] estimate, method given; [U] unknown,
placeholder with `TODO(Evans)` in `sil.yaml`, not for acceptance.

| Parameter | Value | Tag | Source or method |
|---|---|---|---|
| Series cells | 8 | M | BOM PWR-001 and PWR-023 (8S1P); E-01 rev1.8 |
| Capacity | 105 Ah | M | BOM PWR-001 (8 x EVE LF105, 105 Ah, 2.688 kWh), M-01 rev19.10. E-01 rev1.8 charging paragraph says 84 Ah: open conflict (see below) |
| Pack voltage nominal, ceiling | 25.6 V, 29.2 V | M | E-01 BOM rev1.2 BAT row. Used as test limits only |
| Cell OCV against SOC | table in `sil.yaml` | U | Placeholder of the LiFePO4 shape, not EVE data. Method: 3.20 V per cell nominal (BOM Sizing sheet, B-02 basis), flat plateau 3.25 to 3.33 V from 20 to 90 percent; 100 percent at 3.45 V so that the rest voltage stays below the 3.55 V per cell charger setting (E-01 rev1.8); 0 percent at 2.50 V (generic LiFePO4 discharge end, not from a filed EVE sheet). Replace with the EVE LF105 product specification (version F is cited in the mass ledger, not filed in the Datasheet Index) or a measured pulse test |
| Cell DC resistance | 0.5 mOhm | U | Placeholder. The EVE LF105 specification is not filed in the project and was not reachable from the runner |
| Busbars | 8 x Cu 56 x 20 x 2 mm | E | Mass ledger PWR-024 estimate; R = rho L / A with rho = 1.68e-8 Ohm m (annealed copper, 20 C). 0.19 mOhm total, contact resistance not included |
| Main cable | 25 mm2, 0.36 m each way | E | BOM cable estimate CBL-001 (0.36 m with allowance) doubled for the return; section from the BOM Electrical BAT row. 0.48 mOhm |
| BMS path, F0, disconnect, PWR-018, K1/K2, lugs | 5 mOhm together | U | Placeholder. Measure the drop from cell terminals to the SAFE bus at a known current |
| Series resistance total | 9.67 mOhm | E | Sum of the four rows above (`series_resistance_ohm`) |
| Bench charger | 28.4 V, 20 A | M | E-01 rev1.8 (Daly 28.4 V / 20 A, PWR-005) |
| Charger taper current | 2 A | U | Placeholder |
| Electronics load | 150 W | E | BOM Electrical CPU row: 150 W branch provision. A provision, not a measurement. The other rails are UNKNOWN in the Rail loads sheet |
| Arm load at rest | 0 A | U | Placeholder. Envelope 15 A continuous, 25 A peak per arm (BOM Electrical ARM-L, ARM-R). Use the `step` injection for arm current |
| Robot mass | 86.3 kg | M | Mass ledger, decision D-26, P-03 item 30 A6 |
| Rolling resistance | 0.02 | U | Placeholder |
| Drive efficiency, battery to wheel | 0.70 | U | Placeholder |
| Regeneration fraction | 0 | U | Placeholder; 0 gives the pack no credit for braking |
| Drive standby power when enabled | 10 W | U | Placeholder |
| Current direction on the Daly frame | discharge negative | U | The decoder names no direction; `current_sign_discharge` in `sil.yaml` |

## Drive current estimate

The ZLAC8015D emulator has no torque or phase current: it ramps a speed. The estimate uses
what it does report, the wheel speeds of 606Ch (`/sil/drive/emulator_state`,
`actual_rpm`, which equals the Gazebo wheel speed when Gazebo is attached):

    a_i     = r * d(omega_i)/dt                       wheel rim acceleration
    F_i     = m/2 * (a_i + g * c_rr * sign(v_i))      half the robot per wheel
    P_mech  = sum_i F_i * r * omega_i
    P_elec  = P_mech / eta  (P_mech >= 0)   or   P_mech * regen_fraction  (P_mech < 0)
            + standby power while the drive is enabled

Limits of the estimate: no rotational inertia of the hub motors, no yaw inertia, no slope,
no scrub of the castors in a turn, a constant efficiency over the whole speed and torque range,
and a derivative taken from 20 Hz state samples. It is an order-of-magnitude load for voltage
tests, not a motor model. When the drive state is older than `state_timeout_s` the drive
load is zero, and `drive_w` in `/sil/bms/emulator_state` shows it.

Example with the placeholders: 0.5 m/s steady gives 86.3 x 9.81 x 0.02 x 0.5 / 0.7 = 12 W
plus 10 W standby; accelerating at 1.5 m/s2 through 0.3 m/s adds 86.3 x 1.5 x 0.3 / 0.7 =
55 W. The drive is a small load next to the 150 W electronics provision and the arm envelope,
which is the reason the arm current step is the main low-voltage scenario.

## Using it

`bms.model.enabled: true` (default) runs the model in the `daly_bms_emulator` node every
`period_s`. Injection on `/sil/bms/inject`:

| Command | Effect |
|---|---|
| `step <A>` | extra constant load current (arm or other step); `step 0` ends it |
| `charge on`, `charge off` | bench charger connected or not |
| `soc <percent>` | sets the model SOC (and the reported SOC) |
| `model off`, `model on` | freeze or resume the model |
| `voltage <V>`, `current <A>` | fixed values as before; they switch the model off so the value stays |

`/sil/bms/emulator_state` adds `ocv_v`, `resistance_ohm`, `model_current_a`, `drive_w`,
`step_a`, `charger_on`, `charge_done`, `net_discharged_ah`, `model_enabled`.

## Scenarios in the tests

Both run the emulator and the host poll (`BmsHost`) on a CAN bus: vcan by default. Without
vcan they fail with `BLOCKED:` (conftest), never pass. `SIL_TEST_BUS=virtual` runs them on the
python-can in-process bus as a logic check, which is not vcan evidence.

1. Low pack voltage under a current step: SOC 3.5 percent, 150 W electronics, then a 60 A
   step (pack continuous envelope, E-01 BOM rev1.2). The host reads a sag below 8 x 2.8 V and
   the recovery after the step. 2.8 V per cell is a commissioning candidate in BOM Decisions
   D65, not an approved setting; the test only shows that the model can cross it.
2. Charge from a low state: SOC 5 percent, charger on. The host reads 20 A of charge current
   with the opposite sign, a terminal voltage above rest and at most 28.4 V; full after
   0.95 x 105 / 20 = 4.99 h of model time, then zero current and 105000 mAh remaining.

## Open conflicts

- Capacity: 105 Ah (BOM PWR-001, M-01) against 84 Ah in the E-01 rev1.8 charging paragraph.
  The model uses 105 Ah until the owner settles it.
- The emulator constructor keeps its old default of 84 Ah for the existing protocol tests;
  the node passes the configured value.
