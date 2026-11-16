# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Lift drive model: CTL-017 H-bridge, column motor, transmission, sensing and fault conditions.

Candidate model, fidelity F1. Documentation and sources: docs/architecture/sil-lift-drive-model.md.
Parameters: config/models/lift_drive.yaml. Simulation evidence only, never hardware or safety
evidence.

Ownership (architecture contract of the digital twin plan):

- Gazebo DART owns the motion of the prismatic ``lift_joint``. This model never integrates the
  lift position. Each ``step`` takes the joint position and velocity from the plant and returns
  the axial effort (N, positive up) the column applies to the carriage, plus the electrical and
  sensor observables.
- The CTL-017 controller owns every decision (moves, stops, faults, permit). This model takes its
  output (bridge enable, signed PWM duty) and the supply voltage as inputs. It contains no E-stop,
  brake, contactor, watchdog, motor-enable or permit logic; motor power removal is an input.
- Time comes only from the injected clock (``now()``, ``generation``, ``paused``), see
  ``openamrobot_sil.sim_time``. A generation change (simulation reset) drops the motion command
  (gate D3): motion needs a new command afterwards.

Equations (reduced order, lumped to the carriage axis; rotary constants are not known separately):

    V_a = d * V_bus                         averaged H-bridge, d in [-1, 1]
    V_a = (R + R_bridge) I + L dI/dt + K v  armature, K = Ke * 2 pi * N / lead  [V s/m = N/A]
    F_m = K (I - I_f sign)                  motor force at the nut, I_f motor friction current
    raising:  F_m = F_ax / eta              forward efficiency eta
    lowering: F_m = eta_b F_ax              back efficiency eta_b = 2 - 1/eta (self-locking if < 0)
    F_ax = W + F_ext + sign(v) (F_c + c |v|),  W = m_carried g

K, eta and I_f follow from the documented anchors (no-load speed at rated voltage, current at rated
push) and the estimated winding resistance and no-load current; see ``derive_constants``.

The drive speed v_d the motor sustains at the present voltage and load is solved algebraically.
The effort returned to Gazebo couples the carriage to that speed through a numerical damper b:

    effort = W + F_ext + b (v_d - v), clamped to [-rated_pull, holding] and, while driving up,
    to the force the current limit allows.

The motor current reported is the armature current at the measured joint speed,
I = (V_a - K v) / R_tot, clamped to the supply and bridge limits, filtered by L / R_tot when L is
known. A blocked carriage (v = 0) therefore shows the stall current.
"""
import math
import os

import yaml

FIELDS = ('value', 'unit', 'source', 'confidence', 'uncertainty', 'hardware_revision', 'owner',
          'test', 'last_verified', 'measurement_item')
CONFIDENCE = ('datasheet', 'supplier_confirmation', 'design_decision', 'cad', 'estimate',
              'unknown')

# Fallback used by the model when a parameter value is null (unknown).
FALLBACKS = {
    'current_sense_ratio_actual': None,   # reported current = motor current
    'winding_inductance_h': 0.0,
    'gear_ratio': None,
    'spindle_lead_m': None,
    'back_efficiency': None,              # derived from the forward efficiency
    'static_friction_n': 0.0,
    'viscous_friction_n_s_per_m': 0.0,
    'coast_distance_m': None,
    'stop_sensor_hysteresis_m': 0.0,
    'end_switch_cuts_motor': False,
}

# Plant conditions that make the CTL-017 v1 controller raise each fault, as the vendored
# ctl017_emulator.py and the v1 object dictionary (openamr-upperbody-fw PR #15) define them.
# name: (fault id, what the plant does)
CONDITIONS = {
    'end_switch_short': ('F01', 'both end switch inputs active at once'),
    'hall_reversed': ('F02', 'Hall count runs against the commanded direction'),
    'hall_slip': ('F02', 'Hall count offset from the true position (stop sensor cross-check)'),
    'hall_rate': ('F02', 'Hall count rate scaled outside the rate window'),
    'jam': ('F03', 'transmission blocked: no motion, stall current'),
    'overload': ('F03', 'extra axial load raises the motor current above the over-current level'),
    'hall_loss': ('F03', 'Hall signal lost while driving: no pulses (stall detection)'),
    'undervoltage': ('F04', 'supply below nominal: slower travel, the move times out'),
    'hall_supply_loss': ('F05', 'Hall supply lost: no pulses and the supply-good input false'),
    'keepalive_loss': ('F07', 'no plant condition: flag for the CAN layer to stop the keep-alive'),
    'back_drive': ('F08', 'transmission no longer self-locking: carriage drifts after drive off'),
    'stop_sensor_short': ('F09', 'a stop sensor input stuck active (two or more sensors active)'),
}
# F06 is "invalid stored record; unused in v1 (no stored position, D5)": no plant condition.
FAULT_TO_CONDITION = {'F01': 'end_switch_short', 'F02': 'hall_reversed', 'F03': 'jam',
                      'F04': 'undervoltage', 'F05': 'hall_supply_loss', 'F07': 'keepalive_loss',
                      'F08': 'back_drive', 'F09': 'stop_sensor_short'}

DEFAULT_YAML = os.path.join('config', 'models', 'lift_drive.yaml')


def _default_path():
    here = os.path.dirname(os.path.abspath(__file__))
    source = os.path.join(os.path.dirname(os.path.dirname(here)), DEFAULT_YAML)
    if os.path.exists(source):
        return source
    from ament_index_python.packages import get_package_share_directory  # installed layout
    return os.path.join(get_package_share_directory('openamrobot_sil'), DEFAULT_YAML)


def load_parameters(path=None):
    """Read and check the parameter file. Returns {name: entry dict}."""
    with open(path or _default_path()) as f:
        doc = yaml.safe_load(f)
    params = doc['parameters']
    for name, entry in params.items():
        missing = [k for k in FIELDS if k not in entry]
        if missing:
            raise ValueError('%s: missing %s' % (name, ', '.join(missing)))
        if entry['confidence'] not in CONFIDENCE:
            raise ValueError('%s: confidence %r' % (name, entry['confidence']))
        if entry['value'] is None and name not in FALLBACKS:
            raise ValueError('%s: null value without a model fallback' % name)
        if entry['value'] is None and entry['measurement_item'] is None:
            raise ValueError('%s: unknown value without a measurement item' % name)
    return params


def derive_constants(p):
    """Lumped constants from the anchors. p: {name: value}. Returns a dict."""
    r_tot = p['winding_resistance_ohm'] + p['bridge_path_resistance_ohm']
    i0 = p['no_load_current_a']
    k = (p['rated_voltage_v'] - r_tot * i0) / p['no_load_speed_m_s']          # V s/m = N/A
    eta_k = p['rated_push_force_n'] / (p['current_at_rated_push_a'] - i0)    # N/A
    eta = eta_k / k
    # The whole no-load current is taken as motor-side loss; the weight of the column's own
    # moving stage during the catalogue no-load run is inside the estimate's uncertainty.
    i_f = i0
    eta_b = p['back_efficiency'] if p['back_efficiency'] is not None else 2.0 - 1.0 / eta
    return {'r_tot': r_tot, 'k': k, 'eta': eta, 'eta_b': eta_b, 'i_f': i_f}


class LiftDrive:
    """Lift motor, transmission, sensing and fault-condition model. One instance per column."""

    def __init__(self, clock, params=None, path=None):
        entries = params if params is not None else load_parameters(path)
        self.entries = entries
        self.assumed = sorted(n for n, e in entries.items() if e['value'] is None)
        self.p = {n: (FALLBACKS[n] if e['value'] is None else e['value'])
                  for n, e in entries.items()}
        self.c = derive_constants(self.p)
        if self.p['transmission_self_locking'] and self.c['eta_b'] >= 0.0:
            raise ValueError('self-locking column but back efficiency %.3f >= 0' % self.c['eta_b'])
        self.clock = clock
        self._gen = clock.generation
        self._t = clock.now()
        self.resets = 0
        self.inject_state = {}
        self._duty = 0.0
        self._enabled = False
        self._cmd_gen = None
        self.bus_voltage_v = self.p['bus_voltage_nominal_v']
        self.external_force_n = 0.0
        self.current_a = 0.0
        self._stop_state = [False] * len(self.p['stop_sensor_positions_m'])
        self._count_f = None
        self._x_prev = 0.0
        self.last = None

    # --- inputs -----------------------------------------------------------------------------
    def command(self, duty, enabled=True):
        """Bridge command from the controller: signed duty in [-1, 1], positive drives up."""
        if not -1.0 <= duty <= 1.0:
            raise ValueError('duty must lie in [-1, 1]')
        self._duty = float(duty)
        self._enabled = bool(enabled)
        self._cmd_gen = self.clock.generation

    def command_from_ctl017(self, drive_on, direction, duty_permille):
        """CTL-017 drive state (emulator drive_on, dir, 0x2101:09 or :10) to a bridge command."""
        self.command(direction * duty_permille / 1000.0 if drive_on else 0.0, bool(drive_on))

    def set_bus_voltage(self, volts):
        """SAFE 24 V supply at the bridge. 0 V: motor power removed (E-stop chain, an input)."""
        self.bus_voltage_v = max(0.0, float(volts))

    @property
    def carried_mass_kg(self):
        p = self.p
        return p['carriage_link_mass_kg'] + p['arms_mass_kg'] + p['payload_mass_kg']

    def load_force_n(self):
        """Weight carried by the column (N, downward positive). Gazebo applies the same gravity
        to the carriage links; this file and the link masses must agree."""
        return self.carried_mass_kg * self.p['gravity_m_s2']

    # --- fault conditions -----------------------------------------------------------------------
    def inject(self, name, **kw):
        """Apply a plant condition by name, or by fault id (F01..F09 except F06)."""
        if name == 'F06':
            raise ValueError('F06 (invalid stored record) has no plant condition in CTL-017 v1')
        name = FAULT_TO_CONDITION.get(name, name)
        if name not in CONDITIONS:
            raise ValueError('unknown condition %r' % name)
        defaults = {'overload': {'force_n': 1000.0}, 'undervoltage': {'volts': 15.0},
                    'hall_rate': {'pct': 40.0}, 'stop_sensor_short': {'index': 0},
                    'hall_slip': {'m': 0.004}, 'back_drive': {'creep_m_s': 0.002}}
        value = dict(defaults.get(name, {}))
        value.update(kw)
        if name == 'hall_slip' and self._count_f is not None:
            self._count_f += value['m'] / self.p['hall_pitch_m']
        self.inject_state[name] = value
        return CONDITIONS[name][0]

    def clear(self, name=None):
        names = list(self.inject_state) if name is None else [FAULT_TO_CONDITION.get(name, name)]
        for n in names:
            self.inject_state.pop(n, None)   # a slip already counted stays in the count

    @property
    def keepalive_lost(self):
        return 'keepalive_loss' in self.inject_state

    # --- step -----------------------------------------------------------------------------------
    def _check_clock(self):
        """dt from the injected clock; a new generation drops the command (gate D3)."""
        now = self.clock.now()
        if self.clock.generation != self._gen:
            self._gen = self.clock.generation
            self._t = now
            self.resets += 1
            self._duty, self._enabled, self._cmd_gen = 0.0, False, None
            self.current_a = 0.0
            self._count_f = None   # re-base on the reset position: a reset is not Hall motion
            self._stop_state = [False] * len(self._stop_state)
            return 0.0
        if self.clock.paused:
            return 0.0
        dt = max(0.0, now - self._t)
        self._t = now
        return dt

    def _self_locking(self):
        return self.p['transmission_self_locking'] and 'back_drive' not in self.inject_state

    def _supply_v(self):
        v = self.bus_voltage_v
        if 'undervoltage' in self.inject_state:
            v = min(v, self.inject_state['undervoltage']['volts'])
        return v

    def _axial_load(self):
        extra = self.inject_state.get('overload', {}).get('force_n', 0.0)
        return self.load_force_n() + self.external_force_n + extra

    def _i_limit(self):
        return min(self.p['supply_current_limit_a'], self.p['bridge_current_limit_a'])

    def drive_speed(self, v_a, load_n):
        """Quasi-static carriage speed (m/s) the motor sustains at armature voltage v_a and axial
        load load_n, and the current it needs. Returns (v_d, i_needed); v_d = 0: cannot move."""
        c, p = self.c, self.p
        k, r, eta, eta_b, i_f = c['k'], c['r_tot'], c['eta'], c['eta_b'], c['i_f']
        fc, cv = p['static_friction_n'], p['viscous_friction_n_s_per_m']
        ilim = self._i_limit()
        if v_a > 0.0:   # raising: I = i_f + (W + Fc + c v) / (eta K)
            i_static = i_f + (load_n + fc) / (eta * k)
            if i_static > ilim:
                return 0.0, ilim
            v = (v_a - r * i_static) / (k + r * cv / (eta * k))
            if v <= 0.0:
                return 0.0, min(ilim, v_a / r)
            return v, i_static + cv * v / (eta * k)
        if v_a < 0.0:   # lowering
            f_ax0 = load_n - fc
            if eta_b < 0.0 and f_ax0 > 0.0:   # self-locking: the motor drives the load down
                # I = -i_f + eta_b (F_ax0 + c v) / K, v < 0
                i_static = -i_f + eta_b * f_ax0 / k
                v = (v_a - r * i_static) / (k + r * eta_b * cv / k)
                i = i_static + eta_b * cv * v / k
            elif f_ax0 > 0.0:                 # back-drivable: the load helps
                i_static = -i_f + eta_b * f_ax0 / k
                v = (v_a - r * i_static) / (k + r * eta_b * cv / k)
                i = i_static + eta_b * cv * v / k
            else:                              # pulling down against friction
                i_static = -i_f + f_ax0 / (eta * k)
                v = (v_a - r * i_static) / (k + r * cv / (eta * k))
                i = i_static + cv * v / (eta * k)
            if abs(i) > ilim:
                return 0.0, -ilim
            return min(0.0, v), i
        return 0.0, 0.0

    def step(self, position_m, velocity_m_s):
        """One model step at the clock's present time. Returns the output dict (also self.last)."""
        dt = self._check_clock()
        p, c = self.p, self.c
        x, v = float(position_m), float(velocity_m_s)
        load = self._axial_load()
        lower_end, upper_end = p['end_switch_positions_m']
        at_lower, at_upper = x <= lower_end, x >= upper_end
        v_bus = self._supply_v()
        powered = self._enabled and v_bus > 0.0 and self._cmd_gen == self.clock.generation
        v_a = self._duty * v_bus if powered else 0.0
        cut = p['end_switch_cuts_motor'] and ((v_a > 0 and at_upper) or (v_a < 0 and at_lower))
        if cut:
            powered, v_a = False, 0.0
        jam = 'jam' in self.inject_state
        blocked = jam or (v_a > 0 and at_upper) or (v_a < 0 and at_lower)

        # drive speed and mode
        if powered and v_a != 0.0 and not blocked:
            v_d, _ = self.drive_speed(v_a, load)
            mode = 'drive' if v_d != 0.0 else 'stall'
        elif powered and v_a != 0.0:
            v_d, mode = 0.0, 'stall'
        elif self._self_locking() or 'back_drive' in self.inject_state:
            v_d, mode = 0.0, 'hold'
        else:
            v_d, mode = None, 'free'

        # armature current at the measured speed (stall current when blocked)
        ilim = self._i_limit()
        if powered:
            v_meas = 0.0 if blocked else v
            i_target = (v_a - c['k'] * v_meas) / c['r_tot']
            i_target = max(-ilim, min(ilim, i_target))
        else:
            i_target = 0.0   # bridge off or no supply: terminals open, no current
        tau = p['winding_inductance_h'] / c['r_tot']
        if tau > 0.0:
            self.current_a = i_target + (self.current_a - i_target) * math.exp(-dt / tau)
        else:
            self.current_a = i_target

        # effort on the carriage
        b = p['drive_coupling_n_s_per_m']
        if mode == 'free':
            # back-drivable transmission, no drive: motor friction and guide friction resist
            resist = c['k'] * c['i_f'] / c['eta_b'] + p['static_friction_n']
            effort = min(load, resist) - p['viscous_friction_n_s_per_m'] * v
        else:
            if mode == 'hold' and 'back_drive' in self.inject_state:
                v_d = -self.inject_state['back_drive']['creep_m_s']
            effort = load + b * (v_d - v)
            if mode == 'drive' and v_a > 0.0:
                cap = c['eta'] * c['k'] * (ilim - c['i_f']) - p['static_friction_n']
                effort = min(effort, cap)
        effort = max(-p['rated_pull_force_n'], min(p['holding_force_n'], effort))
        self._update_hall(x)

        out = {
            't': self.clock.now(),
            'generation': self.clock.generation,
            'effort_n': effort,
            'drive_speed_m_s': v_d,
            'mode': mode,
            'armature_voltage_v': v_a,
            'current_a': self.current_a,
            'current_report_ma': self._current_report_ma(),
            'hall_count': self.hall_count,
            'hall_supply_ok': 'hall_supply_loss' not in self.inject_state,
            'stop_sensors': self._stop_bits(x),
            'lower_limit': at_lower or 'end_switch_short' in self.inject_state,
            'upper_limit': at_upper or 'end_switch_short' in self.inject_state,
            'keepalive_lost': self.keepalive_lost,
        }
        self.last = out
        return out

    # --- sensing --------------------------------------------------------------------------------
    def _current_report_ma(self):
        actual = self.p['current_sense_ratio_actual']
        gain = 1.0 if actual is None else self.p['current_sense_ratio'] / actual
        res = self.p['current_report_resolution_a']
        return int(round(abs(self.current_a) * gain / res) * res * 1000)

    def _update_hall(self, x):
        """Incremental count like the emulator: lost pulses stay lost."""
        pitch = self.p['hall_pitch_m']
        if self._count_f is None:
            self._count_f = x / pitch
        elif not ('hall_loss' in self.inject_state or 'hall_supply_loss' in self.inject_state):
            direction = -1.0 if 'hall_reversed' in self.inject_state else 1.0
            scale = self.inject_state.get('hall_rate', {}).get('pct', 100.0) / 100.0
            self._count_f += (x - self._x_prev) / pitch * scale * direction
        self._x_prev = x

    @property
    def hall_count(self):
        """Hall count, quantised to whole counts; sign per 0x2101:02."""
        if self._count_f is None:
            return 0
        sign = 1 if self.p['hall_count_up_positive'] else -1
        return sign * int(math.floor(self._count_f + 1e-9))

    def _stop_bits(self, x):
        window = self.p['stop_sensor_window_m']
        hyst = self.p['stop_sensor_hysteresis_m']
        bits = 0
        for i, pos in enumerate(self.p['stop_sensor_positions_m']):
            d = abs(x - pos)
            on = d <= window + (hyst if self._stop_state[i] else 0.0)
            self._stop_state[i] = on
            if on:
                bits |= 1 << i
        if 'stop_sensor_short' in self.inject_state:
            bits |= 1 << self.inject_state['stop_sensor_short']['index']
        return bits
