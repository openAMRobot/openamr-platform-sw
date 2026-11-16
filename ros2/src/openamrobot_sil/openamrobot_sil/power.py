"""Lumped power distribution model behind the pack model (proposal).

Topology (E-01 rev1.8 section 3): the pack feeds the BAT bus through F0 and the disconnect.
On the BAT bus: the compute branch (direct), the 5 V rail (RSD-60G-5), the regulated 24 V rail
(RSD-100B-24) and the charge branch. K1 then K2 in series make the SAFE bus, with Kpre and its
resistor across K2. On the SAFE bus: traction (ZLAC8015D), both arms (direct) and the SAFE 24 V
rail (DDR-240B-24). There is no 12 V rail.

What decides what: the disconnect, the BMS discharge and charge switches, K1, K2 and Kpre are
INPUTS (set_switch). This model never opens or closes any of them and has no E-stop, PNOZ,
watchdog, motor-enable, brake or charge-inhibit logic (decision D11: the electrical model is
lumped and is never safety evidence). Flags are states for logs and tests; nothing acts on them.

Time comes only from the dt passed to step(); PackSimulation.tick() takes dt from the injected
clock. Parameters and their sources: config/power_model.yaml (model_params.Params).

Equations (docs/architecture/sil-battery-model.md):
  pack:      V = OCV(SOC) - R * I, I = I0 + P / V (battery_model.PackModel.solve)
  converter: P_in = V_out * I_out / eta, I_in = P_in / V_in; output only after setup_s with the
             input at or above input_on_v; off below input_min_v; demand above the allowed
             current (peak_fraction * rated for peak_s, then limit_fraction * rated) is cut to it,
             and the output voltage falls in proportion (resistive load).
  precharge: V_safe += (V_bat - V_safe) * (1 - exp(-dt / (R_pre * C))),
             I_pre = (V_bat - V_safe) / R_pre
"""
import math

SWITCHES = ('disconnect', 'bms_discharge', 'bms_charge', 'k1', 'k2', 'kpre')
LOADS = ('compute_w', 'rail_5v_a', 'rail_24v_a', 'safe_24v_a', 'arm_left_a', 'arm_right_a',
         'traction_w')
# Which converter feeds which output load, and which output load each branch takes.
CONVERTER_LOAD = {'rsd_60g_5': 'rail_5v_a', 'rsd_100b_24': 'rail_24v_a',
                  'ddr_240b_24': 'safe_24v_a'}
BRANCH_LOAD = {'compute': 'compute_w', 'traction': 'traction_w', 'arm_left': 'arm_left_a',
               'arm_right': 'arm_right_a'}


class Switch:
    """Externally set switch state with operate and release delays (seconds)."""

    def __init__(self, closed, operate_s=0.0, release_s=0.0):
        self.initial = bool(closed)
        self.operate_s = float(operate_s)
        self.release_s = float(release_s)
        self.reset()

    def reset(self):
        self.commanded = self.initial
        self.closed = self.initial
        self._pending_s = 0.0

    def command(self, closed):
        closed = bool(closed)
        if closed != self.commanded:
            self.commanded = closed
            self._pending_s = 0.0

    def advance(self, dt):
        if self.closed == self.commanded:
            return
        self._pending_s += dt
        delay = self.operate_s if self.commanded else self.release_s
        if self._pending_s >= delay - 1e-12:
            self.closed = self.commanded


class Converter:
    """One DC/DC converter: UVLO, setup delay, current limit, efficiency."""

    def __init__(self, name, p, start_on):
        self.name = name
        self.input_bus = p['input_bus']
        self.vin_min = float(p['input_min_v'])
        self.vin_on = float(p.get('input_on_v', p['input_min_v']))
        self.vin_max = float(p['input_max_v'])
        self.vout_nom = float(p['output_v'])
        self.rated_a = float(p['rated_current_a'])
        self.eta = float(p['efficiency'])
        self.limit_a = float(p['limit_fraction']) * self.rated_a
        self.peak_a = max(float(p['peak_fraction']) * self.rated_a, self.limit_a)
        self.peak_s = float(p['peak_s'])
        self.setup_s = float(p['setup_s'])
        self.fuse_a = float(p['fuse_a'])
        self.start_on = bool(start_on)
        self.reset()

    def reset(self):
        self.input_ok = self.start_on
        self.output_on = self.start_on
        self._ok_s = self.setup_s if self.start_on else 0.0
        self._over_s = 0.0
        self.demand_a = 0.0
        self.vin = 0.0
        self.vout = self.vout_nom if self.start_on else 0.0
        self.iout = 0.0
        self.pin_w = 0.0
        self.iin = 0.0

    def allowed_a(self):
        return self.peak_a if self._over_s < self.peak_s else self.limit_a

    def output(self, demand_a):
        """(V_out, I_out) delivered for a demand at the present state."""
        if not self.output_on or demand_a <= 0.0:
            return (self.vout_nom if self.output_on else 0.0), 0.0
        allowed = self.allowed_a()
        if demand_a <= allowed:
            return self.vout_nom, demand_a
        return self.vout_nom * allowed / demand_a, allowed

    def planned_input_w(self, demand_a):
        v, i = self.output(demand_a)
        return v * i / self.eta

    def update(self, dt, vin, demand_a):
        """Advance the state by dt with input voltage vin; returns the input power drawn."""
        self.vin, self.demand_a = vin, demand_a
        # Input below the lockout: output off in the same step (hold-up is not modelled).
        if self.input_ok and vin < self.vin_min:
            self.input_ok = False
            self.output_on = False
            self._ok_s = 0.0
        v, i = self.output(demand_a)
        self.vout, self.iout = v, i
        self.pin_w = v * i / self.eta
        self.iin = self.pin_w / vin if vin > 0 else 0.0
        # Turn-on and setup time take effect for the next step (one-step lag, documented).
        if not self.input_ok and vin >= self.vin_on:
            self.input_ok = True
            self._ok_s = 0.0
        elif self.input_ok:
            self._ok_s += dt
        self.output_on = self.input_ok and self._ok_s >= self.setup_s - 1e-12
        self._over_s = self._over_s + dt if (self.output_on and demand_a > self.rated_a) else 0.0
        return self.pin_w

    def flags(self):
        return {'input_undervoltage': not self.input_ok,
                'input_overvoltage': self.vin > self.vin_max,
                'output_on': self.output_on,
                'overload': self.output_on and self.demand_a > self.rated_a,
                'current_limited': self.output_on and self.demand_a > self.allowed_a(),
                'over_fuse_rating': self.iin > self.fuse_a}


class Branch:
    """Direct branch from a bus: constant power (W) or constant current (A) load."""

    def __init__(self, name, p, load_key):
        self.name = name
        self.bus = p['bus']
        self.vin_min = float(p['input_min_v'])
        self.fuse_a = float(p['fuse_a'])
        self.load_key = load_key
        self.is_power = load_key.endswith('_w')
        self.reset()

    def reset(self):
        self.vin = 0.0
        self.iin = 0.0

    def flags(self):
        return {'undervoltage': 0.0 < self.vin < self.vin_min,
                'over_fuse_rating': self.iin > self.fuse_a}


class PowerModel:
    """BAT bus, SAFE bus, rails, switch states and energy accounting around a PackModel."""

    def __init__(self, params, pack, charger, initial_switches, initial_loads,
                 converters_start_on=True):
        self.params = params
        self.pack = pack
        self.charger = charger
        sb = params.section('safe_bus')
        self.c_safe = sb['drive_input_capacitance_f'] + sb['other_input_capacitance_f']
        self.r_pre = sb['precharge_resistance_ohm']
        self.pre_fuse_a = sb['precharge_fuse_a']
        self.cont_max_a = params.get('pack.cell_max_continuous_c') * pack.capacity_ah
        self.switches = {}
        for name in SWITCHES:
            closed = initial_switches.get(name, False)
            if name in ('k1', 'k2', 'kpre'):
                self.switches[name] = Switch(closed, sb['contactor_operate_s'],
                                             sb['contactor_release_s'])
            else:
                self.switches[name] = Switch(closed)
        unknown = set(initial_switches) - set(SWITCHES)
        if unknown:
            raise ValueError('unknown switches %s' % sorted(unknown))
        self.converters = {n: Converter(n, params.section('converters.' + n), converters_start_on)
                           for n in params.names('converters')}
        self.branches = {n: Branch(n, params.section('branches.' + n), BRANCH_LOAD[n])
                         for n in params.names('branches')}
        self.initial_loads = {k: float(initial_loads.get(k, 0.0)) for k in LOADS}
        unknown = set(initial_loads) - set(LOADS)
        if unknown:
            raise ValueError('unknown loads %s' % sorted(unknown))
        self.reset()

    # --- inputs ----------------------------------------------------------------------------

    def set_switch(self, name, closed):
        """External input: commanded state of a switch. The model never calls this itself."""
        self.switches[name].command(closed)

    def set_load(self, name, value):
        if name not in LOADS:
            raise KeyError(name)
        self.loads[name] = float(value)

    # --- outputs ---------------------------------------------------------------------------

    def bus_voltage(self, bus='safe'):
        """Voltage of the 'bat' or 'safe' bus (V) at the end of the last step: the input a drive
        or arm model reads."""
        return self.v_bat if bus == 'bat' else self.v_safe

    def rail_voltage(self, converter):
        return self.converters[converter].vout

    def energy(self):
        return dict(self._energy)

    def take_interval_energy(self):
        """Energy (J) per account since the previous call, then start a new interval."""
        out = dict(self._interval)
        self._interval = {k: 0.0 for k in self._interval}
        return out

    # --- model -----------------------------------------------------------------------------

    def reset(self):
        """Configured initial state (gate D3: a new clock generation starts from here).
        The pack is reset by its owner (PackSimulation.reset) before this call."""
        for s in self.switches.values():
            s.reset()
        for c in self.converters.values():
            c.reset()
        for b in self.branches.values():
            b.reset()
        self.loads = dict(self.initial_loads)
        self.v_bat = self.pack.ocv_v() if self._bat_connected() else 0.0
        self.v_safe = self.v_bat if self._safe_closed() else 0.0
        self.i_pre = 0.0
        self.i_chg = 0.0
        self.safe_step_v = 0.0
        accounts = ('pack_out_j', 'pack_loss_j', 'charger_in_j', 'precharge_loss_j') + tuple(
            'conv_%s_out_j' % n for n in self.converters) + tuple(
            'conv_%s_loss_j' % n for n in self.converters) + tuple(
            'branch_%s_j' % n for n in self.branches)
        self._energy = {k: 0.0 for k in accounts}
        self._interval = dict(self._energy)

    def _bat_connected(self):
        return self.switches['disconnect'].closed

    def _loads_served(self):
        return self._bat_connected() and self.switches['bms_discharge'].closed

    def _safe_closed(self):
        return self._loads_served() and self.switches['k1'].closed and self.switches['k2'].closed

    def _demand(self, conv):
        return self.loads[CONVERTER_LOAD[conv.name]]

    def step(self, dt, extra_a=0.0):
        """Advance by dt seconds. extra_a: injected BAT bus current (current-step scenarios)."""
        k2_was = self.switches['k2'].closed
        for s in self.switches.values():
            s.advance(dt)
        served = self._loads_served()
        safe = self._safe_closed()
        precharging = (served and self.switches['k1'].closed and not self.switches['k2'].closed
                       and self.switches['kpre'].closed)
        # Constant-current part i0 and constant-power part p of the pack load.
        i0, p = 0.0, 0.0
        if served:
            i0 += extra_a
            for c in self.converters.values():
                if c.input_bus == 'bat' or safe:
                    p += c.planned_input_w(self._demand(c))
            for b in self.branches.values():
                if b.bus == 'bat' or safe:
                    if b.is_power:
                        p += self.loads[b.load_key]
                    else:
                        i0 += self.loads[b.load_key]
            if precharging and self.r_pre > 0:
                i0 += max(0.0, self.v_bat - self.v_safe) / self.r_pre
        v, i_load = self.pack.solve(i0, p) if served else (self.pack.ocv_v(), 0.0)
        # Charger through the charge branch on the BAT bus (charge switch of the BMS).
        self.i_chg = 0.0
        if self._bat_connected() and self.switches['bms_charge'].closed:
            self.i_chg = self.charger.current(self.pack, i_load)
        i_net = i_load - self.i_chg
        if self.i_chg > 0:
            v = self.pack.ocv_v() - self.pack.resistance_ohm * i_net
        self.pack.apply(dt, v, i_net)
        self.v_bat = v if self._bat_connected() and (served or self.i_chg > 0) else 0.0
        # SAFE bus.
        v_safe_before = self.v_safe
        if safe:
            self.v_safe = self.v_bat
            self.i_pre = 0.0
        elif precharging and self.r_pre > 0:
            tau = self.r_pre * self.c_safe
            self.i_pre = max(0.0, self.v_bat - self.v_safe) / self.r_pre
            if tau > 0:
                self.v_safe += (self.v_bat - self.v_safe) * (1.0 - math.exp(-dt / tau))
            else:
                self.v_safe = self.v_bat
        else:
            self.v_safe = 0.0   # discharge through the loads is not modelled
            self.i_pre = 0.0
        self.safe_step_v = (self.v_bat - v_safe_before) if (safe and not k2_was) else 0.0
        # Rails and branches at the new bus voltages.
        add = {'pack_out_j': v * i_net * dt,
               'pack_loss_j': i_net * i_net * self.pack.resistance_ohm * dt,
               'charger_in_j': v * self.i_chg * dt,
               'precharge_loss_j': self.i_pre * self.i_pre * self.r_pre * dt}
        for c in self.converters.values():
            vin = self.bus_voltage(c.input_bus)
            fed = served and (c.input_bus == 'bat' or safe)
            pin = c.update(dt, vin, self._demand(c) if fed else 0.0)
            add['conv_%s_out_j' % c.name] = c.vout * c.iout * dt
            add['conv_%s_loss_j' % c.name] = (pin - c.vout * c.iout) * dt
        for b in self.branches.values():
            b.vin = self.bus_voltage(b.bus)
            fed = served and (b.bus == 'bat' or safe)
            load = self.loads[b.load_key] if fed and b.vin > 0 else 0.0
            if b.is_power:
                b.iin = load / b.vin if b.vin > 0 else 0.0
                add['branch_%s_j' % b.name] = load * dt
            else:
                b.iin = load
                add['branch_%s_j' % b.name] = b.vin * load * dt
        for k, e in add.items():
            self._energy[k] += e
            self._interval[k] += e
        return v, i_net

    def flags(self):
        out = {'bat_energised': self.v_bat > 0.0, 'safe_energised': self.v_safe > 0.0,
               'precharge_over_fuse_rating': self.i_pre > self.pre_fuse_a,
               'pack_over_continuous_rating': abs(self.pack.current_a) > self.cont_max_a}
        for c in self.converters.values():
            out.update({'%s.%s' % (c.name, k): v for k, v in c.flags().items()})
        for b in self.branches.values():
            out.update({'%s.%s' % (b.name, k): v for k, v in b.flags().items()})
        return out

    def state(self):
        return {'v_bat': round(self.v_bat, 3), 'v_safe': round(self.v_safe, 3),
                'safe_step_v': round(self.safe_step_v, 3),
                'switches': {n: s.closed for n, s in self.switches.items()},
                'rails_v': {n: round(c.vout, 3) for n, c in self.converters.items()},
                'loads': dict(self.loads), 'flags': self.flags(),
                'energy_j': {k: round(v, 1) for k, v in self._energy.items()},
                'defaults_in_use': self.params.defaults_in_use()}
