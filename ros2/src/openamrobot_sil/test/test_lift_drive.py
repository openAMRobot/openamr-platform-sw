# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Lift drive model (models/lift_drive.py): motor, transmission, sensing, fault conditions, reset.

Pure Python. The point-mass plant below stands in for Gazebo DART in these tests only: it owns the
carriage motion, the model only returns the effort. Thresholds come from the vendored CTL-017 v1
object dictionary defaults, so a fault test shows the plant condition crosses the controller's
documented threshold. It does not run the controller. Simulation evidence only.
"""
import copy
import math
import pathlib
import re
import time

import pytest

from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.models import lift_drive as ld
from openamrobot_sil.models.lift_drive import LiftDrive, load_parameters
from openamrobot_sil.sim_time import ManualClock

DT = 0.001
OD = c.build_od()


def od(key):
    return OD.get(*key)


class Plant:
    """Gazebo stand-in: carriage of the model's carried mass, gravity, hard joint limits."""

    def __init__(self, drive, clock, x=0.0, limits=(-0.005, 0.355), mass=None):
        self.drive, self.clock = drive, clock
        self.m = drive.carried_mass_kg if mass is None else mass
        self.g = drive.p['gravity_m_s2']
        self.x, self.v = x, 0.0
        self.limits = limits
        self.out = drive.step(self.x, self.v)
        self.log = []

    def run(self, seconds, every=None):
        for _ in range(int(round(seconds / DT))):
            self.clock.advance(DT)
            self.out = self.drive.step(self.x, self.v)
            a = (self.out['effort_n'] - self.m * self.g) / self.m
            self.v += a * DT
            self.x += self.v * DT
            lo, hi = self.limits
            if self.x <= lo or self.x >= hi:
                self.x = min(hi, max(lo, self.x))
                self.v = 0.0
            self.log.append((self.clock.now(), self.x, self.v, dict(self.out)))
        return self.out


def rig(x=0.0, params=None, **kw):
    clock = ManualClock()
    drive = LiftDrive(clock, params=params)
    return drive, clock, Plant(drive, clock, x=x, **kw)


def params_with(**values):
    p = copy.deepcopy(load_parameters())
    for name, value in values.items():
        p[name]['value'] = value
    return p


def counts_in(plant, seconds):
    n0 = plant.out['hall_count']
    plant.run(seconds)
    return plant.out['hall_count'] - n0


# --- parameters ------------------------------------------------------------------------------
def test_parameters_have_every_field():
    params = load_parameters()
    for name, entry in params.items():
        assert set(ld.FIELDS) <= set(entry), name
        assert entry['confidence'] in ld.CONFIDENCE, name
        if entry['value'] is None:
            assert entry['confidence'] == 'unknown', name
            assert entry['measurement_item'], name
            assert name in ld.FALLBACKS, name


def test_every_named_test_exists():
    text = pathlib.Path(__file__).read_text()
    for name, entry in load_parameters().items():
        if entry['test']:
            fname, func = entry['test'].split('::')
            assert fname == pathlib.Path(__file__).name, name
            assert re.search(r'^def %s\(' % func, text, re.M), (name, func)


def test_parameter_file_without_a_field_is_refused(tmp_path):
    text = pathlib.Path(ld._default_path()).read_text().replace(
        "    measurement_item: MP-12\n", "", 1)
    bad = tmp_path / 'bad.yaml'
    bad.write_text(text)
    with pytest.raises(ValueError, match='missing measurement_item'):
        load_parameters(str(bad))


def test_derived_constants_are_self_locking():
    drive = LiftDrive(ManualClock())
    assert 0.0 < drive.c['eta'] < 0.5
    assert drive.c['eta_b'] < 0.0
    with pytest.raises(ValueError, match='self-locking'):
        LiftDrive(ManualClock(), params=params_with(back_efficiency=0.2))


def test_no_load_speed_anchor():
    drive = LiftDrive(ManualClock())
    v, i = drive.drive_speed(24.0, 0.0)
    assert v == pytest.approx(0.018, rel=1e-9)
    assert i == pytest.approx(drive.p['no_load_current_a'])


def test_rated_push_current_anchor():
    drive = LiftDrive(ManualClock())
    v, i = drive.drive_speed(24.0, 800.0)
    assert i == pytest.approx(7.0)
    assert 0.0 < v < 0.018


def test_load_force_is_the_carried_weight():
    drive = LiftDrive(ManualClock())
    assert drive.carried_mass_kg == pytest.approx(6.721 + 12.26)
    assert drive.load_force_n() == pytest.approx((6.721 + 12.26) * 9.81)


# --- load-dependent behaviour ----------------------------------------------------------------
def test_speed_falls_and_current_rises_with_payload():
    speeds, currents = [], []
    for payload in (0.0, 10.0, 30.0):
        drive, clock, plant = rig(params=params_with(payload_mass_kg=payload))
        drive.command(1.0)
        plant.run(1.0)
        v_d, i_d = drive.drive_speed(24.0, drive.load_force_n())
        assert plant.v == pytest.approx(v_d, rel=0.01)
        assert plant.out['current_a'] == pytest.approx(i_d, rel=0.02)
        speeds.append(plant.v)
        currents.append(plant.out['current_a'])
    assert speeds[0] > speeds[1] > speeds[2]
    assert currents[0] < currents[1] < currents[2]


def test_lowering_is_driven_and_slows_with_load_when_self_locking():
    drive = LiftDrive(ManualClock())
    v_light, i_light = drive.drive_speed(-24.0, 100.0)
    v_heavy, i_heavy = drive.drive_speed(-24.0, 600.0)
    assert v_light < 0.0 and v_heavy < 0.0
    assert abs(v_heavy) < abs(v_light)   # more thread friction to drive through
    assert i_light < 0.0 and i_heavy < i_light


def test_friction_parameters_slow_the_lift():
    base = LiftDrive(ManualClock()).drive_speed(24.0, 186.0)[0]
    with_fric = LiftDrive(ManualClock(), params=params_with(
        static_friction_n=50.0, viscous_friction_n_s_per_m=1000.0)).drive_speed(24.0, 186.0)[0]
    assert 0.0 < with_fric < base


def test_current_is_limited_by_the_supply():
    drive, clock, plant = rig(x=0.1)
    drive.inject('jam')
    drive.command(1.0)
    plant.run(0.1)
    assert plant.out['current_a'] == pytest.approx(10.0)
    drive2, _, plant2 = rig(x=0.1, params=params_with(bridge_current_limit_a=5.0))
    drive2.inject('jam')
    drive2.command(1.0)
    plant2.run(0.1)
    assert plant2.out['current_a'] == pytest.approx(5.0)    # the lower of the two limits
    assert LiftDrive(ManualClock()).drive_speed(24.0, 1300.0) == (0.0, 10.0)


def test_effort_stays_within_the_column_ratings():
    drive, clock, plant = rig(x=0.1)
    drive.command(1.0)
    for _ in range(3):
        plant.run(0.2)
        plant.v = -1.0       # violent disturbance
        plant.run(0.001)
        assert -400.0 <= plant.out['effort_n'] <= 800.0
    assert max(e[3]['effort_n'] for e in plant.log) <= 800.0


def test_coupling_is_stable_at_the_gazebo_step():
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(0.5)
    tail = [v for _, _, v, _ in plant.log[-200:]]
    assert max(tail) - min(tail) < 1e-6


def test_inductance_gives_a_first_order_current_rise():
    drive, clock, plant = rig(x=0.1, params=params_with(winding_inductance_h=0.00101))
    drive.inject('jam')
    drive.command(1.0)
    plant.run(0.001)                    # tau = L / R = 1 ms
    assert plant.out['current_a'] == pytest.approx(10.0 * (1 - math.exp(-1)), rel=0.01)
    plant.run(0.01)
    assert plant.out['current_a'] == pytest.approx(10.0, rel=1e-3)


# --- power removed, drive off ----------------------------------------------------------------
def test_power_removed_during_a_move_holds_when_self_locking():
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(1.0)
    assert plant.v > 0.015
    drive.set_bus_voltage(0.0)          # motor power removed (input from the hardwired chain)
    plant.run(0.05)
    x_held = plant.x
    plant.run(10.0)
    assert plant.out['mode'] == 'hold'
    assert plant.out['current_a'] == 0.0
    assert abs(plant.x - x_held) < 1e-5
    assert plant.out['effort_n'] == pytest.approx(drive.load_force_n(), abs=0.5)


def test_power_removed_lowers_a_back_drivable_transmission():
    # back-driving needs eta_b * load above the motor-side loss K * I_f
    params = params_with(transmission_self_locking=False, back_efficiency=0.3,
                         no_load_current_a=0.05, payload_mass_kg=20.0)
    drive, clock, plant = rig(x=0.2, params=params)
    drive.command(1.0)
    plant.run(0.5)
    drive.set_bus_voltage(0.0)
    plant.run(0.5)
    assert plant.out['mode'] == 'free'
    assert plant.x < 0.2


def test_drive_off_stops_within_one_hall_count():
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(1.0)
    drive.command(0.0, enabled=False)
    plant.run(0.005)
    assert abs(counts_in(plant, 1.0)) <= 1


# --- sensing ---------------------------------------------------------------------------------
def test_hall_scale_matches_the_vendored_emulator():
    drive = LiftDrive(ManualClock())
    assert drive.p['hall_pitch_m'] * 1e3 == pytest.approx(c.MM_PER_HALL)
    assert drive.p['hall_pitch_m'] * 1e9 == pytest.approx(od(c.P_HALL_NM))
    assert [x * 1e3 + 5.0 for x in drive.p['stop_sensor_positions_m']] == pytest.approx(
        [s for s in c.SENSOR_POS_MM])


def test_hall_rate_at_the_nominal_speed():
    drive = LiftDrive(ManualClock())
    # no load: 18 mm/s gives 86 counts/s (sil-topology.md section 6)
    v0, _ = drive.drive_speed(24.0, 0.0)
    assert round(v0 / drive.p['hall_pitch_m']) == 86
    assert 86 // 2 == od(c.P_MOVE_MIN)    # the F02 window lower edge is half the nominal rate
    # in closed loop at the carried load the counted rate is v / pitch and inside the window
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(0.3)
    cps = counts_in(plant, 1.0)
    assert abs(cps - plant.v / drive.p['hall_pitch_m']) <= 1
    assert od(c.P_MOVE_MIN) < cps < od(c.P_MOVE_MAX)


def test_homing_duty_speed_is_inside_the_homing_window():
    drive, clock, plant = rig(x=0.3)
    drive.command_from_ctl017(True, -1, od(c.P_DUTY_HOMING))
    assert drive._duty == -0.5
    plant.run(0.3)
    cps = -counts_in(plant, 1.0)
    assert od(c.P_HOME_MIN) < cps < od(c.P_HOME_MAX)


def test_stop_sensors_switch_at_their_positions():
    drive = LiftDrive(ManualClock())
    for i, pos in enumerate((0.0, 0.175, 0.350)):
        assert drive.step(pos, 0.0)['stop_sensors'] == 1 << i
        assert drive.step(pos + 0.0009, 0.0)['stop_sensors'] == 1 << i
        assert drive.step(pos + 0.0011, 0.0)['stop_sensors'] == 0
        assert drive.step(pos - 0.0011, 0.0)['stop_sensors'] == 0
    assert drive.step(0.1, 0.0)['stop_sensors'] == 0


def test_stop_sensor_hysteresis_widens_the_release():
    drive = LiftDrive(ManualClock(), params=params_with(stop_sensor_hysteresis_m=0.0005))
    assert drive.step(0.1764, 0.0)['stop_sensors'] == 0      # approach: window 1 mm
    assert drive.step(0.1759, 0.0)['stop_sensors'] == 2
    assert drive.step(0.1764, 0.0)['stop_sensors'] == 2      # release: window + hysteresis
    assert drive.step(0.1766, 0.0)['stop_sensors'] == 0


def test_end_switches_and_stall_at_the_end_of_travel():
    drive, clock, plant = rig(x=0.34)
    drive.command(1.0)
    plant.run(1.5)
    assert plant.x == pytest.approx(0.355)
    assert plant.out['upper_limit'] and not plant.out['lower_limit']
    assert plant.out['mode'] == 'stall'
    assert plant.out['current_a'] == pytest.approx(10.0)
    drive, clock, plant = rig(x=0.34, params=params_with(end_switch_cuts_motor=True))
    drive.command(1.0)
    plant.run(1.5)
    assert plant.out['upper_limit'] and plant.out['current_a'] == 0.0


def test_current_report_gain_and_resolution():
    drive, clock, plant = rig(x=0.1)
    drive.command(1.0)
    plant.run(1.0)
    assert plant.out['current_report_ma'] == round(plant.out['current_a'] * 1000)
    drive, clock, plant = rig(x=0.1, params=params_with(current_sense_ratio_actual=15000))
    drive.command(1.0)
    plant.run(1.0)
    assert plant.out['current_report_ma'] == round(plant.out['current_a'] * 19500 / 15000 * 1000)


# --- fault conditions F01 to F09 (thresholds: CTL-017 OD v1 defaults) -------------------------
def test_every_fault_but_f06_has_a_plant_condition():
    ids = {f for f, _ in ld.CONDITIONS.values()}
    assert ids == {'F01', 'F02', 'F03', 'F04', 'F05', 'F07', 'F08', 'F09'}
    drive = LiftDrive(ManualClock())
    for fid, name in ld.FAULT_TO_CONDITION.items():
        assert drive.inject(fid) == fid
        assert name in drive.inject_state
    drive.clear()
    assert drive.inject_state == {}
    with pytest.raises(ValueError, match='F06'):
        drive.inject('F06')
    with pytest.raises(ValueError):
        drive.inject('no_such_condition')


def test_f01_end_switch_short_sets_both_limits():
    drive = LiftDrive(ManualClock())
    drive.inject('end_switch_short')
    out = drive.step(0.1, 0.0)
    assert out['lower_limit'] and out['upper_limit']
    drive.clear('F01')
    out = drive.step(0.1, 0.0)
    assert not (out['lower_limit'] or out['upper_limit'])


def test_f02_hall_reversed_counts_against_the_direction():
    drive, clock, plant = rig()
    drive.inject('hall_reversed')
    drive.command(1.0)
    assert -counts_in(plant, 0.5) > od(c.P_WRONG_DIR)


def test_f02_hall_rate_below_the_window():
    drive, clock, plant = rig()
    drive.inject('hall_rate', pct=40.0)
    drive.command(1.0)
    plant.run(0.3)
    window = od(c.P_HALL_WINDOW) / 1000.0
    cps = counts_in(plant, window) / window
    assert 0 < cps < od(c.P_MOVE_MIN)


def test_f02_hall_slip_fails_the_cross_check_at_a_sensor():
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(1.0)
    drive.inject('hall_slip')
    while not plant.out['stop_sensors'] & 2:
        plant.run(DT)
    counted_m = plant.out['hall_count'] * drive.p['hall_pitch_m']
    error_um = abs(counted_m - 0.175) * 1e6
    assert error_um > od(c.P_XCHECK_TOL)


def test_f03_overload_current_above_the_level_for_longer_than_the_filter():
    drive, clock, plant = rig(x=0.1)
    drive.inject('overload', force_n=1000.0)
    drive.command(1.0)
    plant.run(od(c.P_INRUSH_MS) / 1000.0)
    over = [o['current_report_ma'] > od(c.P_OC_MA)
            for _, _, _, o in plant.log[-1:]]
    plant.log.clear()
    plant.run(od(c.P_OC_MS) / 1000.0 + 0.01)
    assert over[0] and all(o['current_report_ma'] > od(c.P_OC_MA) for *_, o in plant.log)
    assert plant.v > 0.0      # still moving: over-current, not stall


@pytest.mark.parametrize('condition', ['jam', 'hall_loss'])
def test_f03_stall_no_hall_pulse_while_driving(condition):
    drive, clock, plant = rig(x=0.1)
    drive.command(1.0)
    plant.run(od(c.P_HALL_GRACE) / 1000.0)
    drive.inject(condition)
    assert counts_in(plant, od(c.P_STALL_MS) / 1000.0 + 0.01) == 0
    assert plant.out['armature_voltage_v'] > 0.0
    if condition == 'hall_loss':
        assert plant.v > 0.015          # the carriage still moves, only the signal is gone


def test_f04_undervoltage_makes_the_move_time_out_inside_the_rate_window():
    drive, clock, plant = rig()
    drive.inject('undervoltage', volts=15.0)
    drive.command(1.0)
    timeout_s = (175.0 / od(c.P_MOVE_SPEED) * 1e3 * od(c.P_TIMEOUT_PCT) / 100.0
                 + od(c.P_TIMEOUT_MARGIN) / 1000.0)
    plant.run(1.0)
    window = od(c.P_HALL_WINDOW) / 1000.0
    cps = counts_in(plant, window) / window
    assert od(c.P_MOVE_MIN) <= cps <= od(c.P_MOVE_MAX)       # no F02 on the way
    assert 0.175 / plant.v > timeout_s                       # F04 before L1175


def test_f05_hall_supply_loss_while_moving():
    drive, clock, plant = rig(x=0.1)
    drive.command(1.0)
    plant.run(0.5)
    drive.inject('hall_supply_loss')
    plant.run(0.001)
    assert not plant.out['hall_supply_ok'] and plant.v > 0.0
    assert counts_in(plant, 0.5) == 0


def test_f07_keepalive_loss_is_a_flag_only():
    drive, clock, plant = rig(x=0.1)
    drive.command(1.0)
    plant.run(0.5)
    v = plant.v
    drive.inject('F07')
    plant.run(0.5)
    assert plant.out['keepalive_lost'] and plant.v == pytest.approx(v, rel=1e-6)


def test_f08_back_drive_keeps_counting_after_drive_off():
    standstill_s = od(c.P_STANDSTILL_MS) / 1000.0
    quiet_s = od(c.P_QUIET_MS) / 1000.0
    for inject, expect_moving in ((False, False), (True, True)):
        drive, clock, plant = rig(x=0.2)
        drive.command(1.0)
        plant.run(0.5)
        if inject:
            drive.inject('back_drive')
        drive.command(0.0, enabled=False)
        plant.run(standstill_s - quiet_s)
        assert (counts_in(plant, quiet_s) != 0) == expect_moving


def test_f09_stop_sensor_short_gives_two_active_sensors():
    drive = LiftDrive(ManualClock())
    drive.inject('stop_sensor_short', index=0)
    bits = drive.step(0.175, 0.0)['stop_sensors']
    assert bin(bits).count('1') >= 2


# --- clock: reset, pause, no host time -----------------------------------------------------
def test_reset_drops_the_motion_command():
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(1.0)
    assert plant.v > 0.015
    clock.reset(0.0)                       # simulation reset: new generation
    plant.x, plant.v = 0.0, 0.0            # Gazebo puts the joint back
    plant.out = drive.step(plant.x, plant.v)
    assert drive.resets == 1 and plant.out['armature_voltage_v'] == 0.0
    assert plant.out['hall_count'] == 0    # re-based, no pulses from the reset jump
    plant.run(1.0)
    assert abs(plant.x) < 1e-5 and plant.out['mode'] == 'hold'
    drive.command(1.0)                     # a new command in the new generation moves again
    plant.run(0.5)
    assert plant.v > 0.015


def test_pause_freezes_the_model_time():
    drive, clock, plant = rig(x=0.1, params=params_with(winding_inductance_h=0.01))
    drive.inject('jam')
    drive.command(1.0)
    plant.run(0.002)
    i = plant.out['current_a']
    clock.paused = True
    for _ in range(100):
        out = drive.step(plant.x, plant.v)
    assert out['current_a'] == pytest.approx(i)


def test_no_host_time_reads(monkeypatch):
    source = pathlib.Path(ld.__file__).read_text()
    assert not re.search(r'\btime\.[a-z_]+\(|datetime|perf_counter|\bimport time\b', source)

    def forbidden(*_a, **_k):
        raise AssertionError('host time read by the lift drive model')
    for name in ('monotonic', 'time', 'perf_counter', 'monotonic_ns', 'time_ns'):
        monkeypatch.setattr(time, name, forbidden)
    drive, clock, plant = rig()
    drive.command(1.0)
    plant.run(0.2)
    assert plant.out['t'] == pytest.approx(0.2)
