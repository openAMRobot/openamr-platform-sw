"""Lumped 8S LiFePO4 pack model for the Daly BMS emulator (proposal).

What it models: open-circuit voltage against state of charge, one series resistance,
capacity and coulomb counting. Terminal voltage = OCV(SOC) - I * R, I positive = discharge.
What it does not model: cell imbalance, temperature, relaxation (RC branches: no source gives
their parameters), ageing, hysteresis, BMS protection or charge inhibit. Nothing here decides a
fault; the numbers only feed the emulator frames and the power model (power.py).

Time: PackSimulation.tick() takes dt from an injected clock (duck-typed: now(), generation,
paused; the clock contract of docs/architecture/sil-topology.md). No model here reads host time.
A new clock generation (simulation reset) returns every part to its configured initial state.

Every parameter carries its source, confidence and measurement item in config/power_model.yaml
(model_params.Params). A parameter with no robot value runs on its sil_default, and the state
output lists it under defaults_in_use.

Pack current = drive estimate + constant loads. The drive estimate is not a motor model:
the ZLAC8015D emulator has no torque or phase current, so the wheel torque is estimated from
the body dynamics of the robot (mass x acceleration + rolling resistance) at the wheel speeds
the emulator reports, and divided by a drive efficiency. See DriveLoadEstimator.
"""
import bisect
import math

G = 9.81  # m/s2, standard gravity


class OcvCurve:
    """Piecewise-linear cell OCV against SOC (percent)."""

    def __init__(self, points):
        pts = sorted((float(s), float(v)) for s, v in points)
        if len(pts) < 2 or pts[0][0] != 0.0 or pts[-1][0] != 100.0:
            raise ValueError('OCV table must span 0 to 100 percent')
        if any(b[1] < a[1] for a, b in zip(pts, pts[1:])):
            raise ValueError('OCV must not fall with rising SOC')
        self.soc = [p[0] for p in pts]
        self.v = [p[1] for p in pts]

    def cell_v(self, soc_percent):
        s = min(100.0, max(0.0, soc_percent))
        i = bisect.bisect_right(self.soc, s)
        if i >= len(self.soc):
            return self.v[-1]
        s0, s1, v0, v1 = self.soc[i - 1], self.soc[i], self.v[i - 1], self.v[i]
        return v0 + (v1 - v0) * (s - s0) / (s1 - s0)


def series_resistance_ohm(p):
    """Pack series resistance from its parts (config bms.model.resistance).

    cells: n_series x cell DC resistance (UNKNOWN placeholder).
    busbars and main cable: copper R = rho * L / A, geometry from the mass ledger busbar
    estimate and the BOM cable estimate (method in the doc).
    bms_path_ohm: BMS MOSFETs, fuse, disconnect, connector, contactors (UNKNOWN placeholder).
    """
    rho = p['copper_resistivity_ohm_m']
    bus = p['busbar_count'] * rho * p['busbar_length_m'] / p['busbar_section_m2']
    cable = rho * p['cable_length_m'] / p['cable_section_m2']
    return p['n_series'] * p['cell_dc_resistance_ohm'] + bus + cable + p['bms_path_ohm']


class PackModel:
    """Lumped pack: OCV(SOC) - I * R, coulomb counting. I > 0 discharges."""

    def __init__(self, ocv, n_series, capacity_ah, resistance_ohm, soc_percent):
        self.ocv = ocv
        self.n_series = int(n_series)
        self.capacity_ah = float(capacity_ah)
        self.resistance_ohm = float(resistance_ohm)
        self.initial_soc_percent = min(100.0, max(0.0, float(soc_percent)))
        self.reset()

    def reset(self):
        """Configured initial state."""
        self.soc_percent = self.initial_soc_percent
        self.current_a = 0.0
        self.voltage_v = self.ocv_v()
        self.charge_ah = 0.0  # net discharged Ah since start, for tests and logs

    def ocv_v(self):
        return self.n_series * self.ocv.cell_v(self.soc_percent)

    def solve(self, current_a=0.0, power_w=0.0):
        """Terminal voltage and current for a constant-current part plus a constant-power part.

        V = OCV - R * (I0 + P / V)  gives  V^2 - (OCV - R*I0) V + P R = 0, upper root.
        When the pack cannot deliver P (no real root), the voltage collapses to half of
        (OCV - R*I0), the maximum-power point; the caller sees that as a deep sag.
        """
        e = self.ocv_v() - self.resistance_ohm * current_a
        disc = e * e - 4.0 * power_w * self.resistance_ohm
        v = (e + math.sqrt(disc)) / 2.0 if disc > 0 else e / 2.0
        if v <= 0:
            return 0.0, current_a
        return v, current_a + power_w / v

    def step(self, dt, current_a=0.0, power_w=0.0):
        """Advance by dt seconds. Returns (terminal voltage, pack current)."""
        v, i = self.solve(current_a, power_w)
        self.apply(dt, v, i)
        return v, i

    def apply(self, dt, voltage_v, current_a):
        self.voltage_v, self.current_a = voltage_v, current_a
        dah = current_a * dt / 3600.0
        self.charge_ah += dah
        self.soc_percent = min(100.0, max(0.0, self.soc_percent - 100.0 * dah / self.capacity_ah))


class Charger:
    """Constant current, then constant voltage at the charger terminals; ends at a taper
    current or when the coulomb counter reaches 100 percent.

    Charging current is negative in the PackModel convention (I > 0 discharges).
    """

    def __init__(self, voltage_v, current_a, taper_a):
        self.voltage_v = float(voltage_v)
        self.current_a = float(current_a)
        self.taper_a = float(taper_a)
        self.reset()

    def reset(self):
        """Configured initial state: off. Whether it is on is an input (bench operator)."""
        self.on = False
        self.done = False

    def current(self, pack, load_a=0.0):
        """Charger output current (A, >= 0) for the present pack state."""
        if not self.on or self.done:
            return 0.0
        if pack.soc_percent >= 100.0:
            # A lumped model has no polarisation branch, so the CV taper of a real pack does
            # not appear; the charge ends when the coulomb counter is full.
            self.done = True
            return 0.0
        headroom = (self.voltage_v - pack.ocv_v()) / pack.resistance_ohm + load_a
        i = max(0.0, min(self.current_a, headroom))
        if i < self.current_a and i - load_a <= self.taper_a:
            self.done = True
            return 0.0
        return i


class DriveLoadEstimator:
    """Battery-side drive current from the wheel speeds of the ZLAC8015D emulator.

    Per wheel: force = m/2 * a + m/2 * g * c_rr * sign(v), torque = force * r,
    mechanical power = torque * omega. Electrical power = mech / eta when positive, and
    mech * regen_fraction when negative (regen_fraction 0: no credit). Plus a standby power
    while the drive is enabled. The power is drawn from the pack at the pack voltage.
    """

    def __init__(self, p, wheel_radius_m):
        self.mass_kg = p['robot_mass_kg']
        self.c_rr = p['rolling_resistance']
        self.eta = p['drive_efficiency']
        self.regen = p['regen_fraction']
        self.standby_w = p['drive_enabled_standby_w']
        self.r = float(wheel_radius_m)
        self.reset()

    def reset(self):
        self._last = None  # (t, (omega_l, omega_r))

    def power_w(self, t, wheel_rad_s, enabled=True):
        """Electrical drive power (W) at time t (s) for wheel speeds (left, right) rad/s."""
        w = tuple(float(x) for x in wheel_rad_s)
        if self._last is None or t <= self._last[0]:
            acc = (0.0, 0.0)
        else:
            dt = t - self._last[0]
            acc = tuple(self.r * (w[i] - self._last[1][i]) / dt for i in (0, 1))
        self._last = (t, w)
        total = 0.0
        for i in (0, 1):
            v = self.r * w[i]
            roll = self.c_rr * G * math.copysign(1.0, v) if abs(v) > 1e-6 else 0.0
            force = 0.5 * self.mass_kg * (acc[i] + roll)
            mech = force * v
            total += mech / self.eta if mech >= 0 else mech * self.regen
        if enabled:
            total += self.standby_w
        return total


def from_config(bms_cfg, wheel_radius_m, params):
    """(PackModel, Charger, DriveLoadEstimator, loads) from the bms section of sil.yaml and the
    parameters of config/power_model.yaml (model_params.Params)."""
    m = bms_cfg['model']
    pk = params.section('pack')
    if pk['n_series'] != bms_cfg['n_cells'] or pk['capacity_ah'] != bms_cfg['capacity_ah']:
        raise ValueError('sil.yaml bms and power_model.yaml pack disagree on cells or capacity')
    ocv = OcvCurve(pk['cell_ocv_v'])
    pack = PackModel(ocv, pk['n_series'], pk['capacity_ah'], series_resistance_ohm(pk),
                     bms_cfg['soc_percent'])
    ch = params.section('charger')
    charger = Charger(ch['voltage_v'], ch['current_a'], ch['taper_a'])
    drive = DriveLoadEstimator(params.section('drive'), wheel_radius_m)
    loads = m['loads']
    return pack, charger, drive, loads


def power_from_config(bms_cfg, params, pack, charger):
    """PowerModel with the initial switch states and rail loads of sil.yaml bms.model.power."""
    from openamrobot_sil.power import PowerModel
    pw = bms_cfg['model']['power']
    return PowerModel(params, pack, charger, pw['initial_switches'], pw['initial_loads'],
                      pw.get('converters_start_on', True))


class PackSimulation:
    """Ties the pack model to a DalyBmsEmulator. No ROS. One step() per tick.

    Inputs: drive power (from DriveLoadEstimator), constant loads or a PowerModel, an injected
    current step, the charger. Output: the emulator's pack voltage, current and SOC.
    current_sign_discharge maps model current (I > 0 discharges) to the reported current.

    With a PowerModel, the rail loads, switch states and the traction power (drive_w) go through
    it and the constant loads are not used. tick() advances by the time of the injected clock.
    """

    def __init__(self, emu, pack, charger, loads, current_sign_discharge=-1, clock=None,
                 power=None, drive=None):
        self.emu = emu
        self.pack = pack
        self.charger = charger
        self.power = power
        self.drive = drive
        self.clock = clock
        self.const_w = float(loads.get('electronics_w', 0.0))
        self.const_a = float(loads.get('arms_a', 0.0))
        self.sign = -1.0 if current_sign_discharge < 0 else 1.0
        self._gen = clock.generation if clock is not None else None
        self._last_t = clock.now() if clock is not None else None
        self.resets = 0
        self._reset_inputs()
        self.publish()

    def _reset_inputs(self):
        self.step_a = 0.0      # injected extra current, for current-step scenarios
        self.drive_w = 0.0
        self.enabled = True

    def reset(self):
        """Back to the configured initial state (gate D3), then publish it."""
        self.pack.reset()
        self.charger.reset()
        if self.power is not None:
            self.power.reset()
        if self.drive is not None:
            self.drive.reset()
        self._reset_inputs()
        self.resets += 1
        self.publish()

    def tick(self):
        """Advance by the injected clock's elapsed time. A new generation resets; a paused
        clock or a clock that did not advance integrates nothing."""
        c = self.clock
        if c.generation != self._gen:
            self._gen = c.generation
            self._last_t = c.now()
            self.reset()
            return 0.0
        now = c.now()
        dt = now - self._last_t
        self._last_t = now
        if c.paused or dt <= 0.0:
            return 0.0
        self.step(dt)
        return dt

    def step(self, dt):
        if not self.enabled:
            return
        if self.power is not None:
            self.power.set_load('traction_w', self.drive_w)
            self.power.step(dt, extra_a=self.step_a)
            self.publish()
            return
        load_a = self.const_a + self.step_a
        # Loads first, then the charger supplies what it can up to its limit.
        v, i_load = self.pack.solve(load_a, self.const_w + self.drive_w)
        i_chg = self.charger.current(self.pack, i_load)
        if i_chg > 0:
            # Charger connected: the pack sees load current minus charger current.
            i_net = i_load - i_chg
            v = self.pack.ocv_v() - self.pack.resistance_ohm * i_net
            self.pack.apply(dt, v, i_net)
        else:
            self.pack.apply(dt, v, i_load)
        self.publish()

    def publish(self):
        self.emu.set_values(pack_voltage_v=round(self.pack.voltage_v, 3),
                            current_a=self.sign * self.pack.current_a,
                            soc_percent=self.pack.soc_percent)

    def state(self):
        out = {'model_enabled': self.enabled, 'ocv_v': round(self.pack.ocv_v(), 3),
               'resistance_ohm': self.pack.resistance_ohm,
               'model_current_a': round(self.pack.current_a, 3),
               'drive_w': round(self.drive_w, 1), 'step_a': self.step_a,
               'charger_on': self.charger.on, 'charge_done': self.charger.done,
               'net_discharged_ah': round(self.pack.charge_ah, 4)}
        if self.power is not None:
            out['power'] = self.power.state()
        if self.clock is not None:
            out['clock_generation'] = self._gen
            out['model_resets'] = self.resets
        return out


RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def wheel_rad_s_from_drive_state(state, left_sign=1, right_sign=1):
    """Wheel speeds (left, right) rad/s from /sil/drive/emulator_state (606Ch, 0.1 r/min)."""
    rpm = state.get('actual_rpm') or [0.0, 0.0]
    return (left_sign * float(rpm[0]) * RPM_TO_RAD_S, right_sign * float(rpm[1]) * RPM_TO_RAD_S)
