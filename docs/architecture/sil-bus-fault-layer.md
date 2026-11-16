# SIL bus fault and timing layer (Proposed)

Status: Proposed (W4 of the digital twin plan, Gate D preparation, 16 Nov 2026). Development
evidence only. The layer runs on Linux vcan or python-can's in-process `virtual` bus. It is not
evidence of real-bus, firmware, electrical or safety behaviour.

Code: `ros2/src/openamrobot_sil/openamrobot_sil/models/bus_faults.py`.
Parameters: `ros2/src/openamrobot_sil/config/models/bus_faults.yaml`.
Tests: `ros2/src/openamrobot_sil/test/test_bus_faults.py` (27 tests, run by `sil-tests`).

## 1. Why a model

vcan and the `virtual` bus deliver every frame whole and at once. They have no bit rate, no
arbitration, no error frames, no error counters and no bus-off. The emulators and the host
client therefore always see a perfect bus. This layer adds an imperfect bus around them, and it
computes the physical quantities that vcan cannot carry. Error states are modelled values, not
observations.

## 2. Parts

| Part | What it does |
|---|---|
| `FaultyBus` | Wraps a python-can bus it is given. Per rule (arbitration ID under a mask, extended or not, direction `tx`, `rx` or `both`; first match wins): fixed delay, jitter (`none`, `uniform`, `normal`, `exponential`), loss probability, duplication, a reordering window, fixed outage windows and random outage bursts. Frames leave at the first `poll()` or `recv()` at or after their due time in the injected clock. |
| `CanBusModel` | The shared wire of one bus. Frame bits for classic CAN and CAN FD, bit stuffing worst case, none or a fraction; one frame on the wire at a time; at each free moment the lowest identifier wins; bus load over the run and over a window; an arbitration delay estimate. |
| `CanErrorCounters` | TEC and REC of one node with error-active, error-passive and bus-off, bus-off injection and recovery. |
| `UartFaultModel` | Byte loss, framing and parity errors, noise bursts and baud mismatch on a byte stream. |
| `load_config`, `build_can`, `build_uart` | Build the above from the YAML file and a named profile. |

### 2.1 Clock and reset

Every component takes an injected clock with `now()`, `generation` and `paused`
(`openamrobot_sil/sim_time.py`, clock contract in `sil-topology.md` section 6). The module
reads no host time and imports no time module (tested). Delays are counted in the clock of the
receiver of the frame (rule C6). While the clock is paused nothing becomes due.

A new `generation` (a simulation reset or a backward jump) flushes every queued transmit and
receive frame and every frame waiting on or occupying the wire (gate D3: no stale command
survives a reset). The flush count and the reset time go into the metrics. Error counters are
kept across a reset: a reset of the simulation is not a reset of a CAN controller.

### 2.2 Determinism

One seed per `FaultyBus` or `UartFaultModel`, recorded in its metrics. Each rule draws from its
own stream seeded from the seed, the bus name and the rule name. Outage and noise bursts draw
from a separate stream as clock time advances, so the burst schedule does not depend on the
number of frames. The same seed and the same sequence of calls give the same result.

### 2.3 Interface guard (decision D7)

The layer never opens an interface. `check_virtual_bus` accepts python-can's `VirtualBus` and a
SocketCAN bus whose channel name starts with `vcan`, and refuses everything else with
`NotVirtualCan`. The stronger guard of PR #149 (sysfs virtual device and CAN link type) is not
on this branch. Once it is merged, `check_virtual_bus` should call it as well.

## 3. Bus physics

Classic CAN data frame, `n` data bytes: 47 + 8n bits with an 11-bit identifier and 67 + 8n with
a 29-bit identifier. Stuffing applies from SOF to the end of the CRC (34 + 8n and 54 + 8n
bits). The worst case is floor((g + 8n - 1) / 4) stuff bits. An 8-byte standard frame is 135
bits, 270 us at 500 kbit/s. The reality gap register gives the same value for CAN1 (RG-COM-009).

CAN FD (an estimate): the arbitration phase (17 bits standard, 36 extended) and the tail from
the CRC delimiter (13 bits) at the nominal rate. ESI, DLC, data, the stuff count, CRC 17 or 21
and the fixed stuff bits at the data rate when BRS is set. With 10 percent stuffing an 8-byte
frame at 1 / 5 Mbit/s is about 52 us. This agrees with the upper-body arm bus model
(RG-COM-001), which is also an estimate.

Bus load is the busy time of the wire (frames and error frames) divided by the elapsed clock
time. Frames from nodes that are not wrapped can be added with `observe()`. The arbitration
estimate gives an M/D/1 mean wait rho C / (2 (1 - rho)), which ignores priority, and a worst-case
blocking of one maximal frame. CAN is not pre-emptive.

Error model (ISO 11898-1 fault confinement):

- A transmit error adds 8 to TEC. A successful transmission subtracts 1.
- A receive error adds 1 to REC, or 8 for the severe cases. A successful reception subtracts 1.
  Above 127, a successful reception sets REC to 127 (the standard allows 119 to 127).
- TEC or REC at 128 or more means error-passive. TEC at 256 or more means bus-off.
- Bus-off ends after 128 occurrences of 11 recessive bits, counted as 1408 bit times of clock
  time from the start of recovery: 2.816 ms at 500 kbit/s, 5.632 ms at 250 kbit/s. This is a
  lower bound, because on a busy bus each frame end gives one occurrence.
- In `manual` mode, recovery waits for `request_recovery()`, like Linux restart-ms 0. In `auto`
  mode, it starts on entering bus-off after `restart_delay_s`.
- A rule's `tx_error_p` makes a transmission fail. The model then puts the frame and a 23-bit
  error frame on the wire, adds 8 to TEC and retransmits automatically. An error-passive
  transmitter waits 8 more bits. A frame is dropped when its node goes bus-off. A bus-off node
  sends and receives nothing.

The ISO exceptions are not modelled. These are the ACK error of an error-passive transmitter
and stuff errors during arbitration.

## 4. UART model

`apply(data)` returns what the receiver reads. Byte `i` is on the line at
`t_start + i * char_time` (`char_time` = bits per character / baud, 10 bits for 8N1). The checks
run per byte in this order:

1. Noise burst active: random bit errors.
2. Baud mismatch: the stop bit is sampled `(bits - 0.5) * |mismatch|` bit times off. At half a
   bit or more, every byte is a framing error. The tolerance is 0.5 / 9.5 = 5.26 % for 8N1,
   both ends together, with ideal sampling.
3. Byte loss.
4. Framing error.
5. Parity error (only when a parity bit is configured).

An error byte reads as `\x00` (`null`, termios with IGNPAR and PARMRK off), is removed (`drop`)
or reads as `\xff\x00` and the byte (`mark`, PARMRK).

## 5. Metrics (DT-12)

`FaultyBus.metrics()` returns a dict that can be serialised to JSON for the scenario output. It
holds the bus, interface, seed, generation and rules. It also holds `frames` and `bytes`, and per
direction: frames in and out, bytes, loss, outage drops, duplicates, reorderings and bus-off
drops, with latency `p50_s`, `p95_s`, `p99_s` and `max_s` (nearest rank). On the wire it holds
frames, bits, error frames, load percent (whole run and window) and the arbitration estimate.
`error_state` holds TEC, REC, the state, the bus-off count and every transition with its clock
time, generation and cause. The reset section holds the flushed frames and each reset.
`UartFaultModel.metrics()` gives the byte counts per fault, the noise burst count and the
mismatch tolerance.

## 6. Parameters and their sources

Each physical value in the YAML file is a record with `value, unit, source, confidence,
uncertainty, hardware_revision, owner, test, last_verified, measurement_item`. An unknown
value is null and names a measurement item of the phase 1 measurement plan. Nothing is measured:
`last_verified` is null everywhere. The `profiles` section holds scenario fault rates (SIL test
values). They are not properties of the real bus.

| Parameter | Value | Confidence | Source | MP |
|---|---|---|---|---|
| CAN1 drive, nominal bit rate | 500 kbit/s classic | design_decision (provisional) | BOM CAN buses sheet; `sil-topology.md` section 2 | MP-15 |
| CAN1 worst 8-byte frame | 270 us | estimate | RG-COM-009 | MP-15 |
| CAN2 BMS, nominal bit rate | 250 kbit/s classic | design_decision (proposed) | BOM CAN buses sheet | MP-13 |
| CAN3 aux, nominal bit rate | 500 kbit/s classic | design_decision (proposal, BOM: to be confirmed) | RG-COM-019 | MP-14 |
| Arm L and R, nominal / data | 1 / 5 Mbit/s CAN FD | manufacturer_document (upstream default) | RG-COM-001 | MP-07 |
| Arm bus-off recovery | manual (restart-ms 0) | manufacturer_document | RG-COM-004 | MP-07 |
| CAN1, CAN2, CAN3 bus-off recovery | null | unknown | RG-FLT-014 | MP-10, MP-14 |
| CAN1 real frame loss rate | null | unknown | RG-COM-009 | MP-09 |
| Daly response delay | null | unknown | RG-COM-017 | MP-13 |
| Arm motor reply latency | null | unknown | RG-COM-005 | MP-07 |
| Error counter rules (8, 1, 128, 256, 128 x 11) | as section 3 | manufacturer_document (ISO 11898-1) | ISO 11898-1:2015 fault confinement | MP-10 |
| Error frame length | 23 bits | estimate (worst case of 17 to 23) | ISO 11898-1:2015 | MP-15 |
| TF-Luna UART (tof_front, tof_rear) | 115200 8N1 | manufacturer_document | firmware peripheral note; `drivers.launch.py` | MP-20 |
| MB7060 UART (4 ultrasonic) | 9600 8N1, RX inverted | manufacturer_document | firmware peripheral note | MP-20 |
| RPLIDAR S3 UART | 1 000 000 8N1, via USB on the Jetson | manufacturer_document | vendor datasheet; `drivers.launch.py` | MP-17 |
| UART byte error rates | null | unknown | none | MP-17, MP-20 (no UART error capture listed yet) |

Records by confidence (67 records, YAML aliases counted once per bus or UART): 44
manufacturer_document, 14 unknown, 6 design_decision, 3 estimate, 0 measured. 17 values are null:
the 14 unknown values and 3 data bit rates that do not apply to classic CAN.

## 7. Reality gap rows addressed

The layer gives each of these rows a modelled behaviour that can be tested in SIL. None of the
rows is closed. Each one still needs its bench capture.

- RG-COM-001, RG-COM-002: CAN FD frame time with and without BRS on the arm buses.
- RG-COM-004, RG-COM-023, RG-FLT-014: error counters, error-passive and bus-off with recovery,
  injectable on any bus.
- RG-COM-005, RG-COM-010, RG-COM-017: reply and response delay and jitter can be injected per
  identifier (the real values are null).
- RG-COM-009, RG-COM-024: frame time, serialisation, arbitration and load on CAN1 (no
  transceiver delay or sample point).
- RG-COM-012: frame-level latency of the target writes, measurable through the metrics. The
  one-transfer-per-tick firmware pattern is not reproduced here.
- RG-TIM-004: arm bus cycle time can be computed from frame times and load.
- RG-TIM-006: delays run on the injected clock, and a reset flushes queued frames.
- RG-TIM-010: BMS answer delay and outage bursts against the poll period.

## 8. Limits

- vcan cannot carry error frames. Error states exist only in the model. A firmware or emulator
  that reads the bus state from the kernel still sees error-active.
- No sample point, transceiver loop delay, termination, reflections, cable length, EMC or
  isolator delay.
- Arbitration is modelled only between frames that pass through `FaultyBus` handles on the same
  `CanBusModel`. Other frames add load through `observe()` but take no wire time.
- Delivery resolution is the caller's polling period in the same clock.
- The CAN FD bit count and the M/D/1 wait are estimates. The ISO exceptions are not modelled.
- The UART baud mismatch model is a threshold with ideal sampling. There are no partial error
  rates near the threshold, and USB adapter buffering is not modelled.
- No emulator, node or launch file uses the layer yet. Wiring it in is the next work package.
- No safety logic: the layer does not implement or change any E-stop, brake, contactor,
  watchdog, motor-enable or charge-inhibit behaviour. A modelled bus-off or a lost frame is not
  safety evidence.
