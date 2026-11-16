# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""CANDIDATE reduced-order model of one ZLAC8015D drive and its two ZLLG80 hub motors (F1).

This is a candidate model, not a specification of the real drive. It is datasheet-informed at
most (fidelity F1); every parameter, its source and its confidence is in
config/models/zlac8015d_motor.yaml, and docs/architecture/sil-drive-motor-model.md lists what is
not modelled.

Architecture contract (senior review, 16 Nov 2026): Gazebo DART owns the wheel motion. This model
never integrates a wheel position or speed. Per step it takes the wheel angular velocity (and,
optionally, the wheel angle) from the plant and returns the motor torque to apply to each wheel
joint. Time comes only from the injected clock (``now()``, ``generation``, ``paused``); the step is
the clock delta. No host time is read here.

Per axis (DC-equivalent of the motor, one winding):
    V = R I + L dI/dt + Ke w          tau = Kt I
The RL circuit is integrated with its exact zero-order-hold discretisation, so it is stable for
any step. The drive's current loop (2019h/201Ah, raw gains with an undocumented scaling) is
replaced by an ideal current controller limited by the available voltage ``utilisation * Vbus``;
the speed loop is a PI on the ramped target (6083h/6084h) with the current reference saturated
at the drive's current limit (2015h).

The brake is a plant-side delay only: the coil input comes from outside and the model reports
the resulting state after the documented release and engage times. Nothing here decides when to
brake, enables the motor on its own or acts as a safety function. 603Fh faults, the 2000h
communication loss and the CiA 402 state are reported as the drive documents them; they are
functional telemetry of a simulated device, never safety evidence.
"""
import math

import yaml

from openamrobot_sil.sim_time import Stamp, is_fresh

RPM_TO_RAD_S = 2.0 * math.pi / 60.0
TWO_PI = 2.0 * math.pi

# 603Fh bits, ZLAC8015D CANopen Communication Routine V1.07 (labelled V4.0) section 3.7.
# 'common' faults concern both motors and are reported in the low half (as the vendored
# emulator does); per-axis faults go into the half the caller names. The routine gives the two
# halves opposite wheel meanings in section 3.7 and in its 603Fh object row, so no half has a
# wheel meaning here (parameter fault_high_half_axis stays unknown).
FAULTS = {
    'over_voltage': (0x0001, 'common'),
    'under_voltage': (0x0002, 'common'),
    'eeprom_error': (0x0100, 'common'),
    'over_current': (0x0004, 'axis'),
    'overload': (0x0008, 'axis'),
    'encoder_tolerance': (0x0020, 'axis'),
    'reference_voltage_error': (0x0080, 'axis'),
    'hall_error': (0x0200, 'axis'),
    'over_temperature': (0x0400, 'axis'),
    'encoder_error': (0x0800, 'axis'),
}

# 6041h bits as the vendored emulator and CiA 402 use them.
SW_READY = 0x0001
SW_SWITCHED_ON = 0x0002
SW_ENABLED = 0x0004
SW_FAULT = 0x0008
SW_VOLTAGE = 0x0010
SW_QUICK_STOP_N = 0x0020

# CiA 402 states, reduced.
SWITCH_ON_DISABLED = 'switch_on_disabled'
READY = 'ready_to_switch_on'
SWITCHED_ON = 'switched_on'
ENABLED = 'operation_enabled'
FAULT = 'fault'

# Brake states (plant only).
BRAKE_ENGAGED = 'engaged'
BRAKE_RELEASING = 'releasing'
BRAKE_RELEASED = 'released'
BRAKE_ENGAGING = 'engaging'

# Parameters the model cannot run without. Unknown ones are null in the YAML; a caller (a test)
# passes explicit values for them.
REQUIRED = (
    'phase_resistance', 'phase_inductance', 'torque_constant', 'back_emf_constant',
    'current_limit', 'voltage_utilisation', 'speed_kp', 'speed_ki',
    'profile_acceleration_time', 'profile_deceleration_time', 'ramp_reference_speed',
    'target_velocity_limit', 'given_speed_resolution_setting', 'actual_velocity_unit',
    'actual_current_unit', 'bus_voltage_unit', 'link_loss_time', 'brake_release_time',
    'brake_engage_time', 'brake_holding_torque', 'axis_sign_1', 'axis_sign_2', 'max_substep',
)


class MissingParameter(ValueError):
    """A required parameter is null (unknown) and no explicit value was passed."""


def load_parameters(path, overrides=None):
    """Values of config/models/zlac8015d_motor.yaml, with explicit overrides.

    An override name must exist in the file, so a typo cannot add a parameter silently.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    params = {name: entry['value'] for name, entry in doc['parameters'].items()}
    for name, value in (overrides or {}).items():
        if name not in params:
            raise KeyError('unknown parameter %r' % name)
        params[name] = value
    return params


def parameter_entries(path):
    with open(path) as f:
        return yaml.safe_load(f)['parameters']


class _Brake:
    """Holding brake as a delay: the coil input is external, the state follows after a delay."""

    def __init__(self, release_time, engage_time, holding_torque):
        self.release_time = float(release_time)
        self.engage_time = float(engage_time)
        self.holding_torque = float(holding_torque)
        self.coil_powered = False
        self.state = BRAKE_ENGAGED  # engaged with power off (ZLTECH 18 Oct 2026)
        self._since = 0.0

    def set_coil(self, powered, t):
        powered = bool(powered)
        if powered == self.coil_powered:
            return
        self.coil_powered = powered
        self._since = t
        if powered:
            self.state = BRAKE_RELEASED if self.release_time <= 0 else BRAKE_RELEASING
        else:
            self.state = BRAKE_ENGAGED if self.engage_time <= 0 else BRAKE_ENGAGING

    def update(self, t):
        if self.state == BRAKE_RELEASING and t - self._since >= self.release_time:
            self.state = BRAKE_RELEASED
        elif self.state == BRAKE_ENGAGING and t - self._since >= self.engage_time:
            self.state = BRAKE_ENGAGED

    def settle(self, t):
        """New clock generation: the delay restarts nowhere; the brake takes its settled state."""
        self.state = BRAKE_RELEASED if self.coil_powered else BRAKE_ENGAGED
        self._since = t

    def torque_capacity(self):
        """Holding torque available to the plant: full until released, none until engaged."""
        if self.state in (BRAKE_ENGAGED, BRAKE_RELEASING):
            return self.holding_torque
        return 0.0


class Zlac8015dDriveModel:
    """Two axes of one ZLAC8015D in profile velocity mode (6060h = 3), axis 1 and axis 2.

    Inputs: controlword, targets, bus voltage, brake coil power, CAN frame receipts, wheel speed
    and angle from the plant. Output of step(): motor torque per wheel joint (N*m).
    """

    def __init__(self, clock, params, bus_voltage):
        missing = [k for k in REQUIRED if params.get(k) is None]
        if missing:
            raise MissingParameter('unknown parameters need explicit values: %s' % ', '.join(missing))
        self.clock = clock
        self.p = dict(params)
        p = self.p
        for name in ('phase_resistance', 'phase_inductance', 'torque_constant',
                     'back_emf_constant', 'current_limit', 'max_substep'):
            if not p[name] > 0:
                raise ValueError('%s must be > 0' % name)
        self.signs = (float(p['axis_sign_1']), float(p['axis_sign_2']))
        self.link_loss_ms = int(p['link_loss_time'] * 1000) if p['link_loss_time'] else 0
        self.counts_per_rev = p.get('encoder_counts_per_rev')
        self.bus_voltage = float(bus_voltage)
        self.brakes = [_Brake(p['brake_release_time'], p['brake_engage_time'],
                              p['brake_holding_torque']) for _ in range(2)]
        self._generation = clock.generation
        self._t = clock.now()
        self.state = SWITCH_ON_DISABLED
        self._active = set()     # injected fault causes present now: (name, half)
        self._measured = set()   # causes the model detects itself (bus voltage thresholds)
        self._latched = 0        # 603Fh as reported, until a fault reset with the cause gone
        self._last_rx = Stamp(clock)
        self.link_lost = False
        self.link_loss_events = 0
        self._reset_axes()

    def _reset_axes(self):
        self.target_counts = [0, 0]
        self.ramp_rpm = [0.0, 0.0]
        self.current = [0.0, 0.0]
        self._integral = [0.0, 0.0]
        self.efforts = (0.0, 0.0)
        self._omega = [0.0, 0.0]
        self._angle = [None, None]

    # --- inputs ---------------------------------------------------------------------------
    def on_frame(self):
        """A CAN frame for this node arrived now (restarts the 2000h timer, as the emulator)."""
        self._last_rx = Stamp(self.clock)
        self.link_lost = False

    def write_controlword(self, cw):
        """6040h. 06h ready, 07h switched on, 0Fh operation enabled, bit 7 fault reset."""
        self.on_frame()
        cw = int(cw)
        if cw & 0x80:
            self._fault_reset()
            return
        if self.state == FAULT:
            return
        low = cw & 0x0F
        if low == 0x0F:
            if self.state in (READY, SWITCHED_ON, ENABLED):
                self.state = ENABLED
        elif low == 0x07:
            if self.state in (READY, SWITCHED_ON, ENABLED):
                self.state = SWITCHED_ON
        elif low == 0x06:
            self.state = READY
        else:
            self.state = SWITCH_ON_DISABLED

    def set_target(self, axis, counts):
        """60FFh:01 (axis 1) or :02 (axis 2), whole counts of the given speed resolution."""
        self.on_frame()
        if isinstance(counts, bool) or not isinstance(counts, int):
            raise TypeError('60FFh takes whole counts (r/min at resolution 1)')
        rpm = counts / float(self.p['given_speed_resolution_setting'])
        if abs(rpm) > self.p['target_velocity_limit']:
            raise ValueError('60FFh out of range: %d' % counts)
        self.target_counts[self._axis(axis)] = counts

    def set_bus_voltage(self, volts):
        self.bus_voltage = float(volts)

    def set_brake_coil(self, axis, powered):
        """Coil supply of one brake, decided outside this model (SAFE 24 V converter, E-01)."""
        self.brakes[self._axis(axis)].set_coil(powered, self.clock.now())

    def inject(self, fault_name, half=None):
        """Make a fault cause present. Per-axis faults need half 'high' or 'low' of 603Fh."""
        if fault_name not in FAULTS:
            raise ValueError('unknown fault %r; known: %s' % (fault_name, ', '.join(sorted(FAULTS))))
        kind = FAULTS[fault_name][1]
        if kind == 'common':
            half = 'low'
        elif half not in ('high', 'low'):
            raise ValueError("per-axis fault needs half='high' or half='low'")
        self._active.add((fault_name, half))
        self._latch()

    def clear(self, fault_name=None):
        """Remove a fault cause (all when None). The report stays latched until a fault reset."""
        if fault_name is None:
            self._active.clear()
        else:
            self._active = {c for c in self._active if c[0] != fault_name}

    def power_cycle(self):
        """Drive power off and on: faults clear (manual V1.01, 3.5), targets and enable are lost."""
        self._latched = 0
        self.state = SWITCH_ON_DISABLED
        self.link_lost = False
        self._last_rx = Stamp(self.clock)
        self._reset_axes()

    # --- step -----------------------------------------------------------------------------
    def step(self, wheel_velocity, wheel_angle=None):
        """Advance to clock.now(). wheel_velocity, wheel_angle: (axis 1, axis 2), rad, rad/s."""
        now = self.clock.now()
        self._omega = [float(w) for w in wheel_velocity]
        self._angle = list(wheel_angle) if wheel_angle is not None else [None, None]
        if self.clock.generation != self._generation:
            self._new_generation(now)
            return self.efforts
        dt = now - self._t
        if self.clock.paused or dt <= 0.0:
            return self.efforts
        self._t = now
        for b in self.brakes:
            b.update(now)
        self._check_link()
        self._check_voltage()
        efforts = []
        for axis in (0, 1):
            efforts.append(self._step_axis(axis, dt))
        self.efforts = tuple(efforts)
        return self.efforts

    def _step_axis(self, axis, dt):
        p = self.p
        sign = self.signs[axis]
        omega = sign * self._omega[axis]
        enabled = self.state == ENABLED
        # Profile velocity ramp (6083h/6084h: time from 0 to the reference speed).
        target = 0.0
        if enabled and not self.link_lost:
            target = self.target_counts[axis] / float(p['given_speed_resolution_setting'])
        ramp = self.ramp_rpm[axis]
        decel = abs(target) < abs(ramp) or target * ramp < 0
        t_ramp = p['profile_deceleration_time'] if decel else p['profile_acceleration_time']
        if t_ramp <= 0:
            ramp = target
        else:
            step = p['ramp_reference_speed'] * dt / t_ramp
            ramp += max(-step, min(step, target - ramp))
        self.ramp_rpm[axis] = ramp
        if not enabled:
            # PWM off; 2007h factory 0 does not lock the shaft: no torque, the wheel is free.
            self.current[axis] = 0.0
            self._integral[axis] = 0.0
            return 0.0
        # PI speed loop, current reference saturated at the drive current limit (2015h).
        limit = p['current_limit']
        err = ramp * RPM_TO_RAD_S - omega
        unsat = p['speed_kp'] * err + p['speed_ki'] * self._integral[axis]
        i_ref = max(-limit, min(limit, unsat))
        if unsat == i_ref or err * unsat < 0:
            self._integral[axis] += err * dt  # conditional integration, no windup
        # RL winding, exact discretisation, substeps of at most max_substep.
        r, ind, ke = p['phase_resistance'], p['phase_inductance'], p['back_emf_constant']
        v_max = p['voltage_utilisation'] * max(0.0, self.bus_voltage)
        n = max(1, int(math.ceil(dt / p['max_substep'] - 1e-9)))
        h = dt / n
        a = math.exp(-r * h / ind)
        i = self.current[axis]
        for _ in range(n):
            v = ke * omega + r * (i_ref - a * i) / (1.0 - a)
            v = max(-v_max, min(v_max, v))
            i = a * i + (1.0 - a) * (v - ke * omega) / r
        self.current[axis] = i
        return sign * p['torque_constant'] * i

    def _check_link(self):
        if self.link_loss_ms <= 0 or self.link_lost:
            return
        if not is_fresh(self._last_rx, self.clock, self.link_loss_ms / 1000.0):
            self.link_lost = True  # reaction: target ramps to 0 with 6084h, no fault (emulator)
            self.link_loss_events += 1

    def _check_voltage(self):
        ov, uv = self.p.get('over_voltage_threshold'), self.p.get('under_voltage_threshold')
        for name, cond in (('over_voltage', ov is not None and self.bus_voltage > ov),
                           ('under_voltage', uv is not None and self.bus_voltage < uv)):
            if cond:
                self._measured.add((name, 'low'))
            else:
                self._measured.discard((name, 'low'))
        self._latch()

    def _latch(self):
        for name, half in self._active | self._measured:
            bit = FAULTS[name][0]
            self._latched |= bit << 16 if half == 'high' else bit
        if self._latched:
            self.state = FAULT

    def _fault_reset(self):
        self._latched = 0
        self._latch()  # a cause still present latches again at once
        if not self._latched:
            self.state = SWITCH_ON_DISABLED

    def _new_generation(self, now):
        """Simulation reset (gate D3): target dropped, plant state cleared, a new enable needed."""
        self._generation = self.clock.generation
        self._t = now
        if self.state != FAULT:
            self.state = SWITCH_ON_DISABLED
        self._last_rx = Stamp(self.clock)
        self.link_lost = False
        omega, angle = self._omega, self._angle
        self._reset_axes()
        self._omega, self._angle = omega, angle
        for b in self.brakes:
            b.settle(now)

    # --- reports, as the drive gives them --------------------------------------------------
    def statusword(self):
        sw = SW_VOLTAGE | SW_QUICK_STOP_N
        if self.state == FAULT:
            return sw | SW_FAULT
        if self.state in (READY, SWITCHED_ON, ENABLED):
            sw |= SW_READY
        if self.state in (SWITCHED_ON, ENABLED):
            sw |= SW_SWITCHED_ON
        if self.state == ENABLED:
            sw |= SW_ENABLED
        return sw

    def error_code(self):
        """603Fh, 32 bit, two 16-bit halves without a wheel meaning."""
        return self._latched

    def velocity_actual(self, axis):
        """606Ch counts (0.1 r/min each per the routine) from the plant wheel speed."""
        i = self._axis(axis)
        rpm = self.signs[i] * self._omega[i] / RPM_TO_RAD_S
        return int(round(rpm / self.p['actual_velocity_unit']))

    def position_actual(self, axis):
        """6064h counts from the plant wheel angle; None while counts per revolution is unknown."""
        i = self._axis(axis)
        if self.counts_per_rev is None or self._angle[i] is None:
            return None
        return int(math.floor(self.signs[i] * self._angle[i] / TWO_PI * self.counts_per_rev))

    def current_actual(self, axis):
        """6077h counts (0.1 A each per the routine)."""
        return int(round(self.current[self._axis(axis)] / self.p['actual_current_unit']))

    def bus_voltage_actual(self):
        """2035h counts (0.01 V each per the routine)."""
        return int(round(self.bus_voltage / self.p['bus_voltage_unit']))

    def brake_state(self, axis):
        return self.brakes[self._axis(axis)].state

    def brake_torque_capacity(self, axis):
        return self.brakes[self._axis(axis)].torque_capacity()

    @staticmethod
    def _axis(axis):
        if axis not in (1, 2):
            raise ValueError('axis must be 1 or 2')
        return axis - 1
