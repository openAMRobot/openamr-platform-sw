"""Power distribution model (power.py) and its parameter file (config/power_model.yaml).

No bus is needed. These tests check model maths and bookkeeping against the documented
sources; they are not electrical, safety or real-robot evidence (decision D11).
"""
import copy
import math
import os

import pytest
import yaml

from conftest import PACKAGE_DIR
from openamrobot_sil import battery_model as bm
from openamrobot_sil import config, model_params
from openamrobot_sil.power import SWITCHES, Converter, PowerModel

CFG = config.load(os.path.join(PACKAGE_DIR, 'config', 'sil.yaml'))
BMS = CFG['bms']
PARAMS_PATH = os.path.join(PACKAGE_DIR, 'config', BMS['model']['parameters_file'])
DOC = yaml.safe_load(open(PARAMS_PATH, encoding='utf-8'))
PARAMS = model_params.Params(DOC)
POWER = BMS['model']['power']
ALL_CLOSED = {'disconnect': True, 'bms_discharge': True, 'bms_charge': True, 'k1': True,
              'k2': True, 'kpre': False}
NO_LOAD = {}


def make(switches=None, loads=None, start_on=True, params=PARAMS, soc=None):
    pack, charger, _, _ = bm.from_config(BMS, CFG['drive']['wheel_radius'], params)
    if soc is not None:
        pack.initial_soc_percent = soc
        pack.reset()
    return PowerModel(params, pack, charger, dict(ALL_CLOSED, **(switches or {})),
                      loads if loads is not None else NO_LOAD, start_on)


def run(pm, seconds, dt=0.01, **kw):
    for _ in range(int(round(seconds / dt))):
        pm.step(dt, **kw)


def edited(path, value):
    """Params with one value replaced (path like 'safe_bus.contactor_operate_s')."""
    doc = copy.deepcopy(DOC)
    node = doc
    *parents, leaf = path.split('.')
    for k in parents:
        node = node[k]
    node[leaf]['value'] = value
    node[leaf].pop('sil_default', None)
    if node[leaf]['confidence'] == 'unknown':
        node[leaf]['confidence'] = 'estimate'
    return model_params.Params(doc)


# --- parameter file ----------------------------------------------------------------------

def test_power_parameters_follow_the_schema():
    """Every parameter has value, unit, source, confidence, uncertainty, hardware revision,
    owner, test, last verified and measurement item; unknown means a null value with an MP
    item and a declared sil_default; the validator rejects a broken entry."""
    assert model_params.validate(DOC) == []
    counts = PARAMS.by_confidence()
    assert sum(counts.values()) == len(PARAMS.meta)
    assert counts['unknown'] == len([k for k, p in PARAMS.meta.items()
                                     if p['confidence'] == 'unknown'])
    for name in PARAMS.defaults_in_use():
        assert PARAMS.meta[name]['measurement_item'].startswith('MP-'), name
    bad = copy.deepcopy(DOC)
    bad['pack']['bms_path_ohm']['value'] = 0.004        # a guessed number for an unknown part
    del bad['pack']['capacity_ah']['owner']
    problems = model_params.validate(bad)
    assert any('bms_path_ohm: confidence unknown but a value' in p for p in problems)
    assert any('capacity_ah: missing owner' in p for p in problems)
    with pytest.raises(ValueError):
        model_params.Params(bad)


def test_every_test_named_in_the_parameter_file_exists():
    here = os.path.dirname(os.path.abspath(__file__))
    source = ''.join(open(os.path.join(here, f), encoding='utf-8').read()
                     for f in os.listdir(here) if f.startswith('test_') and f.endswith('.py'))
    for name, p in PARAMS.meta.items():
        assert 'def %s(' % p['test'] in source, (name, p['test'])


def test_converter_parameters_come_from_the_inventory_parts():
    """The three converters are the component inventory parts PWR-007, PWR-012 and PWR-030,
    with their datasheet ratings; there is no 12 V rail."""
    parts = {n: PARAMS.get('converters.%s.part' % n) for n in PARAMS.names('converters')}
    assert parts == {'rsd_60g_5': 'MEAN WELL RSD-60G-5', 'rsd_100b_24': 'MEAN WELL RSD-100B-24',
                     'ddr_240b_24': 'MEAN WELL DDR-240B-24'}
    for n in parts:
        c = PARAMS.section('converters.' + n)
        assert c['output_v'] * c['rated_current_a'] == pytest.approx(c['rated_power_w'], rel=0.01)
        assert PARAMS.meta['converters.%s.efficiency' % n]['confidence'] == 'datasheet'
        assert c['output_v'] != 12.0
    # The bus never exceeds the 29.2 V ceiling (E-01 rev1.8), inside every converter input range.
    assert all(PARAMS.get('converters.%s.input_max_v' % n) >= 29.2 for n in parts)


def test_rails_follow_the_e01_topology():
    pm = make()
    assert {n: c.input_bus for n, c in pm.converters.items()} == {
        'rsd_60g_5': 'bat', 'rsd_100b_24': 'bat', 'ddr_240b_24': 'safe'}
    assert {n: b.bus for n, b in pm.branches.items()} == {
        'compute': 'bat', 'traction': 'safe', 'arm_left': 'safe', 'arm_right': 'safe'}


# --- pack and buses ----------------------------------------------------------------------

def test_sag_equals_i_times_r_and_bus_voltage_feeds_a_drive_model():
    """bus_voltage('safe') is the pack terminal voltage with K1 and K2 closed: OCV - I * R."""
    pm = make(loads={'traction_w': 1000.0, 'arm_left_a': 10.0})
    ocv = pm.pack.ocv_v()
    v, i = pm.step(0.01)
    assert pm.bus_voltage('safe') == pm.bus_voltage('bat') == v
    assert v == pytest.approx(ocv - pm.pack.resistance_ohm * i, abs=1e-9)
    assert i == pytest.approx(10.0 + 1000.0 / v, rel=1e-9)
    assert ocv - v == pytest.approx(i * pm.pack.resistance_ohm, rel=1e-6)
    pm.set_switch('k1', False)
    pm.step(0.01)
    assert pm.bus_voltage('safe') == 0.0 and pm.bus_voltage('bat') > 0.0


def test_efficiency_power_balance():
    """Pack terminal power = direct branch power + converter output / efficiency."""
    loads = {'compute_w': 150.0, 'rail_5v_a': 6.0, 'rail_24v_a': 3.0, 'safe_24v_a': 8.0,
             'arm_left_a': 5.0, 'arm_right_a': 4.0, 'traction_w': 300.0}
    pm = make(loads=loads)
    run(pm, 1.0)
    v, i = pm.step(0.01)
    conv = 0.0
    for c in pm.converters.values():
        assert c.pin_w == pytest.approx(c.vout * c.iout / c.eta, rel=1e-12)
        key = {'rsd_60g_5': 'rail_5v_a', 'rsd_100b_24': 'rail_24v_a',
               'ddr_240b_24': 'safe_24v_a'}[c.name]
        assert c.vout == c.vout_nom and c.iout == loads[key]
        conv += c.pin_w
    direct = 150.0 + 300.0 + v * (5.0 + 4.0)
    assert v * i == pytest.approx(conv + direct, rel=1e-9)
    e = pm.take_interval_energy()
    for n, c in pm.converters.items():
        assert e['conv_%s_loss_j' % n] == pytest.approx(
            e['conv_%s_out_j' % n] * (1.0 / c.eta - 1.0), rel=1e-9)
    assert [c.eta for c in pm.converters.values()] == [0.9, 0.89, 0.89]   # sorted names


def test_energy_per_interval():
    pm = make(loads={'compute_w': 200.0})
    run(pm, 10.0, dt=0.1)
    e = pm.take_interval_energy()
    assert e['branch_compute_j'] == pytest.approx(200.0 * 10.0, rel=1e-9)
    assert e['pack_out_j'] == pytest.approx(200.0 * 10.0, rel=1e-9)
    assert e['pack_loss_j'] == pytest.approx(
        (200.0 / pm.v_bat) ** 2 * pm.pack.resistance_ohm * 10.0, rel=1e-3)
    assert pm.take_interval_energy()['pack_out_j'] == 0.0      # a new interval starts
    run(pm, 1.0, dt=0.1)
    assert pm.take_interval_energy()['pack_out_j'] == pytest.approx(200.0, rel=1e-9)
    assert pm.energy()['pack_out_j'] == pytest.approx(2200.0, rel=1e-9)


def test_pack_flag_above_the_continuous_cell_rating():
    pm = make()
    pm.step(0.01, extra_a=104.0)
    assert not pm.flags()['pack_over_continuous_rating']
    pm.step(0.01, extra_a=106.0)                      # 1 C of 105 Ah
    assert pm.flags()['pack_over_continuous_rating']


def test_bms_disconnect_drops_every_rail():
    """An open BMS discharge switch (an input, RG-FLT-020) takes the BAT bus and every rail
    down; the model reports it and does nothing else."""
    pm = make(loads={'compute_w': 150.0, 'rail_5v_a': 1.0})
    run(pm, 0.1)
    pm.set_switch('bms_discharge', False)
    pm.step(0.01)
    assert pm.bus_voltage('bat') == 0.0 and pm.bus_voltage('safe') == 0.0
    assert pm.pack.current_a == 0.0
    assert all(c.vout == 0.0 and not c.output_on for c in pm.converters.values())
    assert pm.flags()['bat_energised'] is False
    assert pm.switches['k1'].commanded and pm.switches['k2'].commanded


# --- converters --------------------------------------------------------------------------

def test_converter_current_limit_flag():
    """RSD-60G-5: 12 A rated, constant current from 105 % (datasheet lower bound)."""
    pm = make(loads={'rail_5v_a': 11.0})
    run(pm, 0.1)
    f = pm.flags()
    assert not f['rsd_60g_5.overload'] and not f['rsd_60g_5.current_limited']
    pm.set_load('rail_5v_a', 12.3)
    run(pm, 0.1)
    f = pm.flags()
    assert f['rsd_60g_5.overload'] and not f['rsd_60g_5.current_limited']
    pm.set_load('rail_5v_a', 14.0)
    run(pm, 0.1)
    c = pm.converters['rsd_60g_5']
    assert pm.flags()['rsd_60g_5.current_limited']
    assert c.iout == pytest.approx(12.6) and c.vout == pytest.approx(5.0 * 12.6 / 14.0)
    pm.set_load('rail_5v_a', 1.0)
    run(pm, 0.1)
    assert not pm.flags()['rsd_60g_5.current_limited']          # automatic recovery


def test_converter_peak_then_constant_current():
    """DDR-240B-24: up to 150 % for 3 s, then constant current at 105 %."""
    pm = make(loads={'safe_24v_a': 14.0})
    run(pm, 2.9)
    c = pm.converters['ddr_240b_24']
    assert c.iout == pytest.approx(14.0) and pm.flags()['ddr_240b_24.overload']
    run(pm, 0.2)
    assert c.iout == pytest.approx(10.5) and pm.flags()['ddr_240b_24.current_limited']


def test_converter_input_undervoltage_turns_the_rail_off():
    """DDR-240B-24 lockout: off below 16.5 V, back on from 16.8 V after its setup time."""
    c = Converter('ddr', PARAMS.section('converters.ddr_240b_24'), start_on=True)
    for vin, on in ((25.0, True), (16.6, True), (16.4, False), (16.7, False)):
        c.update(0.01, vin, 1.0)
        assert c.output_on is on, vin
        assert c.flags()['input_undervoltage'] is (not on)
    c.update(0.01, 16.8, 1.0)
    t = 0.0
    while not c.output_on:
        c.update(0.01, 16.8, 1.0)
        t += 0.01
    assert t == pytest.approx(0.5, abs=0.011)
    r = Converter('rsd5', PARAMS.section('converters.rsd_60g_5'), start_on=True)
    r.update(0.01, 8.9, 1.0)
    assert not r.output_on and r.vout == 0.0


def test_converter_output_appears_after_its_setup_time():
    """Cold start: 5 V after 100 ms, regulated 24 V after 800 ms, SAFE 24 V after 500 ms."""
    pm = make(start_on=False)
    seen = {}
    t = 0.0
    while len(seen) < 3 and t < 2.0:
        pm.step(0.01)
        t += 0.01
        for n, c in pm.converters.items():
            if c.output_on and n not in seen:
                seen[n] = t
    assert seen['rsd_60g_5'] == pytest.approx(0.1, abs=0.021)
    assert seen['rsd_100b_24'] == pytest.approx(0.8, abs=0.021)
    assert seen['ddr_240b_24'] == pytest.approx(0.5, abs=0.021)


# --- branches ----------------------------------------------------------------------------

def test_branch_fuse_rating_flag():
    pm = make(loads={'arm_left_a': 29.0})
    pm.step(0.01)
    assert not pm.flags()['arm_left.over_fuse_rating']
    pm.set_load('arm_left_a', 31.0)
    pm.step(0.01)
    assert pm.flags()['arm_left.over_fuse_rating']
    pm.set_load('rail_5v_a', 12.0)                 # 60 W / 0.89 at about 26 V: 2.6 A < 5 A
    pm.step(0.01)
    assert not pm.flags()['rsd_60g_5.over_fuse_rating']


def test_branch_undervoltage_flag():
    """Compute branch: flag below the 19 V carrier input (CMP-002) during a deep sag."""
    pm = make(soc=0.0, loads={'compute_w': 150.0})
    pm.step(0.01)
    assert not pm.flags()['compute.undervoltage']
    pm.step(0.01, extra_a=60.0)
    assert pm.bus_voltage('bat') < 19.0
    assert pm.flags()['compute.undervoltage']


# --- switches: inputs only ---------------------------------------------------------------

def test_contactor_state_does_not_change_by_itself():
    """Deep discharge, overcurrent, converter limits and undervoltage flags: no switch moves.
    Switches change only through set_switch (decision D11)."""
    for start in (ALL_CLOSED, {'k1': False, 'k2': False, 'kpre': False}):
        pm = make(switches=start, soc=1.0,
                  loads={'safe_24v_a': 20.0, 'arm_left_a': 40.0, 'compute_w': 400.0})
        before = {n: (s.commanded, s.closed) for n, s in pm.switches.items()}
        run(pm, 5.0, dt=0.05, extra_a=150.0)
        assert any(pm.flags().values())
        assert {n: (s.commanded, s.closed) for n, s in pm.switches.items()} == before
    assert set(SWITCHES) == set(pm.switches)


def test_contactor_follows_its_input_after_the_operate_time():
    params = edited('safe_bus.contactor_operate_s', 0.05)
    pm = make(switches={'k2': False}, params=params)
    pm.set_switch('k2', True)
    run(pm, 0.04)
    assert not pm.switches['k2'].closed and pm.bus_voltage('safe') == 0.0
    run(pm, 0.02)
    assert pm.switches['k2'].closed and pm.bus_voltage('safe') == pm.bus_voltage('bat')
    pm.set_switch('k2', False)                     # release time sil_default 0
    pm.step(0.01)
    assert not pm.switches['k2'].closed and pm.bus_voltage('safe') == 0.0


def test_precharge_follows_the_rc_curve():
    """K1 closed, K2 open, Kpre closed: V_safe = V_bat (1 - exp(-t / (R C)))."""
    pm = make(switches={'k2': False, 'kpre': True})
    tau = (PARAMS.get('safe_bus.precharge_resistance_ohm') *
           (PARAMS.get('safe_bus.drive_input_capacitance_f') +
            PARAMS.get('safe_bus.other_input_capacitance_f')))
    assert tau == pytest.approx(47.0 * 9.4e-4)
    dt = tau / 100.0
    pm.step(dt)
    assert pm.i_pre == pytest.approx(pm.bus_voltage('bat') / 47.0, rel=1e-3)
    assert pm.flags()['precharge_over_fuse_rating'] is False     # 0.6 A < 1 A fuse
    for _ in range(99):
        pm.step(dt)
    assert pm.bus_voltage('safe') == pytest.approx(pm.bus_voltage('bat') * (1 - math.exp(-1)),
                                                   rel=2e-3)
    # The SAFE loads are not served while K2 is open; closing K2 is an input.
    assert pm.converters['ddr_240b_24'].iout == 0.0
    pm.set_switch('k2', True)
    pm.step(dt)
    assert pm.bus_voltage('safe') == pm.bus_voltage('bat')
    assert pm.safe_step_v == pytest.approx(pm.bus_voltage('bat') * math.exp(-1), rel=0.01)


# --- reset -------------------------------------------------------------------------------

def test_power_reset_returns_to_the_configured_initial_state():
    pm = make(switches=POWER['initial_switches'], loads=POWER['initial_loads'])
    initial = pm.state()
    pm.set_switch('k1', False)
    pm.set_load('compute_w', 999.0)
    run(pm, 2.0, extra_a=50.0)
    assert pm.state() != initial
    pm.pack.reset()
    pm.reset()
    assert pm.state() == initial
    assert pm.take_interval_energy()['pack_out_j'] == 0.0


def test_sil_power_section_names_known_switches_and_loads():
    pm = make(switches=POWER['initial_switches'], loads=POWER['initial_loads'])
    assert {n: s.closed for n, s in pm.switches.items()} == POWER['initial_switches']
    with pytest.raises(ValueError):
        make(switches={'estop': True})
    with pytest.raises(KeyError):
        pm.set_load('lidar_w', 1.0)
