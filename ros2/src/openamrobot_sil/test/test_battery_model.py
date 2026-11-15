"""Lumped 8S LiFePO4 pack model: model maths (no bus) and fault scenarios over CAN2.

The scenario tests run the Daly BMS emulator and the host poll on a CAN bus from conftest:
vcan by default, BLOCKED (a failure, never a pass) when vcan cannot be created. The model
maths tests need no bus.
"""
import os
import re

import pytest

from conftest import PACKAGE_DIR, wait_for
from openamrobot_sil import battery_model as bm
from openamrobot_sil import config, daly_bms
from openamrobot_sil.stm32_host import BmsHost

CFG_PATH = os.path.join(PACKAGE_DIR, 'config', 'sil.yaml')
CFG = config.load(CFG_PATH)
BMS = CFG['bms']
WHEEL_R = CFG['drive']['wheel_radius']
# Comparison level for the low-voltage scenario: 2.8 V per cell is a commissioning candidate
# in BOM Decisions D65, "not an approved setting". The test checks that the model can cross
# it under a current step; it does not decide what the robot does when it does.
CANDIDATE_LOW_CELL_V = 2.8


def model():
    return bm.from_config(BMS, WHEEL_R)


# --- sources -----------------------------------------------------------------------------

def test_every_model_number_has_a_source_tag():
    """No unexplained number: each numeric line under bms.model carries [M], [E], [U] or
    'SIL test value', or is an OCV table row whose table header is tagged."""
    lines = open(CFG_PATH, encoding='utf-8').read().splitlines()
    start = lines.index('  model:')
    in_table = False
    for line in lines[start + 1:]:
        if line and not line.startswith('    '):
            break
        body = line.split('#')[0]
        if re.match(r'^\s+- \[', body):
            assert in_table, line
            continue
        in_table = 'ocv_cell_v' in body and '[U]' in line
        if re.search(r':\s*-?[\d.]', body):
            assert re.search(r'\[M\]|\[E\]|\[U\]|SIL test value', line), line
            if '[U]' in line:
                assert 'TODO(Evans)' in line, line


def test_capacity_and_series_count_follow_the_bom():
    assert BMS['n_cells'] == 8
    assert BMS['capacity_ah'] == 105.0


# --- model maths -------------------------------------------------------------------------

def test_ocv_is_monotonic_and_interpolates():
    ocv = bm.OcvCurve(BMS['model']['ocv_cell_v'])
    values = [ocv.cell_v(s) for s in range(0, 101)]
    assert values == sorted(values)
    assert ocv.cell_v(-5) == ocv.cell_v(0) and ocv.cell_v(120) == ocv.cell_v(100)
    assert ocv.cell_v(7.5) == pytest.approx((3.00 + 3.20) / 2)
    with pytest.raises(ValueError):
        bm.OcvCurve([(0, 3.3), (50, 3.2), (100, 3.4)])
    with pytest.raises(ValueError):
        bm.OcvCurve([(10, 3.0), (100, 3.4)])


def test_pack_voltage_stays_inside_the_documented_range():
    """Rest voltage between 0 and 100 percent never exceeds the 29.2 V ceiling (E-01 rev1.8)."""
    pack, _, _, _ = model()
    for soc in range(0, 101, 5):
        pack.soc_percent = soc
        assert 8 * 2.5 <= pack.ocv_v() <= 29.2


def test_series_resistance_sums_its_parts():
    r = dict(BMS['model']['resistance'], n_series=8)
    copper = 1.68e-8 * (8 * 0.056 / 4.0e-5 + 0.72 / 2.5e-5)
    assert bm.series_resistance_ohm(r) == pytest.approx(
        8 * r['cell_dc_resistance_ohm'] + copper + r['bms_path_ohm'])


def test_terminal_voltage_sags_by_i_times_r_and_coulomb_counting():
    pack, _, _, _ = model()
    ocv = pack.ocv_v()
    v, i = pack.step(36.0, current_a=50.0)
    assert i == 50.0
    assert v == pytest.approx(ocv - 50.0 * pack.resistance_ohm)
    # 50 A for 36 s = 0.5 Ah of 105 Ah
    assert pack.soc_percent == pytest.approx(BMS['soc_percent'] - 100 * 0.5 / 105.0)
    assert pack.charge_ah == pytest.approx(0.5)


def test_constant_power_load_solves_v_and_i_together():
    pack, _, _, _ = model()
    v, i = pack.solve(current_a=2.0, power_w=500.0)
    assert v == pytest.approx(pack.ocv_v() - pack.resistance_ohm * i)
    assert i == pytest.approx(2.0 + 500.0 / v)
    # Above the maximum power point the pack collapses to half its source voltage.
    v_max, _ = pack.solve(power_w=1e6)
    assert v_max == pytest.approx(pack.ocv_v() / 2)


def test_drive_estimate_at_constant_speed_is_rolling_loss_over_efficiency():
    _, _, drive, _ = model()
    p = BMS['model']['drive']
    omega = 0.5 / WHEEL_R
    drive.power_w(0.0, (omega, omega))
    got = drive.power_w(1.0, (omega, omega))
    rolling = p['robot_mass_kg'] * bm.G * p['rolling_resistance'] * 0.5
    assert got == pytest.approx(rolling / p['drive_efficiency'] + p['drive_enabled_standby_w'])
    assert drive.power_w(2.0, (omega, omega), enabled=False) == pytest.approx(
        rolling / p['drive_efficiency'])


def test_drive_estimate_accelerating_and_braking():
    _, _, drive, _ = model()
    p = BMS['model']['drive']
    m, eta = p['robot_mass_kg'], p['drive_efficiency']
    drive.power_w(0.0, (0.0, 0.0), enabled=False)
    # 1.5 m/s2 (P-03 item 30 A6 travel deceleration, used here as acceleration) to 0.3 m/s
    omega = 0.3 / WHEEL_R
    got = drive.power_w(0.2, (omega, omega), enabled=False)
    force = m * (1.5 + bm.G * p['rolling_resistance'])
    assert got == pytest.approx(force * 0.3 / eta)
    # Braking from 0.3 m/s at 1.5 m/s2: negative mechanical power, regen fraction 0 -> 0 W
    braked = drive.power_w(0.4, (0.0, 0.0), enabled=False)
    assert braked == pytest.approx(0.0)
    # Turning on the spot draws power on both wheels
    drive.power_w(1.0, (-omega, omega), enabled=False)
    assert drive.power_w(2.0, (-omega, omega), enabled=False) > 0


def test_drive_state_conversion_uses_the_axis_signs():
    w = bm.wheel_rad_s_from_drive_state({'actual_rpm': [60.0, -30.0]}, 1, -1)
    assert w == pytest.approx((2 * 3.141592653589793, 3.141592653589793))
    assert bm.wheel_rad_s_from_drive_state({}) == (0.0, 0.0)


def test_charger_cc_until_full_then_stops():
    pack, charger, _, _ = model()
    pack.soc_percent = 5.0
    assert charger.current(pack) == 0.0       # off
    charger.on = True
    assert charger.current(pack) == pytest.approx(20.0)
    pack.soc_percent = 100.0
    assert charger.current(pack) == 0.0 and charger.done


# --- scenarios over CAN2 (vcan, BLOCKED without it) ---------------------------------------

class Rig:
    def __init__(self, make_bus, soc_percent, **loads):
        pack, charger, _, cfg_loads = model()
        pack.soc_percent = soc_percent
        lo = dict(cfg_loads, **loads)
        self.emu = daly_bms.DalyBmsEmulator(make_bus(), n_cells=8, capacity_ah=105.0).start()
        self.sim = bm.PackSimulation(self.emu, pack, charger, lo,
                                     BMS['model']['current_sign_discharge'])
        self.host = BmsHost(make_bus()).start()

    def read(self):
        """Wait until the host has a poll answer of the present emulator values."""
        want_v = round(self.sim.pack.voltage_v, 1)
        want_soc = round(self.sim.pack.soc_percent, 1)
        assert wait_for(lambda: self.host.state().get('pack_voltage_v') is not None and
                        abs(self.host.state()['pack_voltage_v'] - want_v) < 0.051 and
                        abs(self.host.state()['soc_percent'] - want_soc) < 0.051, 2.0), (
            self.host.state(), want_v, want_soc)
        return self.host.state()

    def close(self):
        self.host.stop()
        self.emu.stop()


@pytest.fixture
def rig_factory(make_bus):
    rigs = []

    def factory(*args, **kw):
        rigs.append(Rig(make_bus, *args, **kw))
        return rigs[-1]
    yield factory
    for r in rigs:
        r.close()


def test_scenario_low_pack_voltage_under_a_current_step(rig_factory):
    """Low SOC, then an arm and drive current step: the host reads the sag over CAN2 and the
    voltage recovers when the step ends. No fault decision is tested (out of scope)."""
    # 3.5 percent: placeholder OCV 2.85 V per cell, just above the candidate level
    rig = rig_factory(soc_percent=3.5, electronics_w=150.0, arms_a=0.0)
    sim = rig.sim
    sim.step(0.1)
    rest = rig.read()
    ocv = sim.pack.ocv_v()
    assert rest['pack_voltage_v'] > 8 * CANDIDATE_LOW_CELL_V
    # 60 A continuous envelope of the pack (E-01 BOM rev1.2 BAT row) on top of electronics
    sim.step_a = 60.0
    sim.step(0.1)
    sag = rig.read()
    assert sag['pack_voltage_v'] < rest['pack_voltage_v'] - 0.4
    assert sag['pack_voltage_v'] < 8 * CANDIDATE_LOW_CELL_V
    assert sag['pack_voltage_v'] == pytest.approx(
        ocv - sim.pack.resistance_ohm * sim.pack.current_a, abs=0.1)
    # Discharge is reported with the configured sign
    assert sag['current_a'] == pytest.approx(
        BMS['model']['current_sign_discharge'] * sim.pack.current_a, abs=0.1)
    assert abs(sag['current_a']) > 60.0
    sim.step_a = 0.0
    sim.step(0.1)
    back = rig.read()
    assert back['pack_voltage_v'] == pytest.approx(rest['pack_voltage_v'], abs=0.15)


def test_scenario_charge_from_a_low_state(rig_factory):
    """Bench charger on at 5 percent: charging current is reported with the opposite sign,
    the terminal voltage sits above OCV and below 28.4 V, SOC climbs to full in about
    capacity / current hours, then the charger stops."""
    rig = rig_factory(soc_percent=5.0, electronics_w=0.0, arms_a=0.0)
    sim = rig.sim
    sim.step(1.0)
    rest = rig.read()
    sim.charger.on = True
    sim.step(1.0)
    charging = rig.read()
    sign = BMS['model']['current_sign_discharge']
    assert charging['current_a'] == pytest.approx(-sign * 20.0, abs=0.1)
    assert charging['pack_voltage_v'] > rest['pack_voltage_v']
    assert charging['pack_voltage_v'] <= 28.4 + 1e-6
    hours = 0.0
    while not sim.charger.done and hours < 10:
        sim.step(60.0)
        hours += 60.0 / 3600.0
        assert sim.pack.voltage_v <= 28.4 + 1e-6
    # 95 percent of 105 Ah at 20 A is 4.99 h
    assert hours == pytest.approx(0.95 * 105.0 / 20.0, abs=0.05)
    full = rig.read()
    assert full['soc_percent'] == pytest.approx(100.0)
    sim.step(1.0)
    after = rig.read()
    assert after['current_a'] == pytest.approx(0.0, abs=0.05)
    mos = daly_bms.decode_response(daly_bms.ID_MOS, rig.emu.payload(daly_bms.ID_MOS))
    assert mos['remaining_mah'] == 105000
