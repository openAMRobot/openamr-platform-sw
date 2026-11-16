# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Candidate ZLAC8015D drive and hub motor model (openamrobot_sil.models.drive_motor).

Pure Python, no CAN, no ROS, no host time: every test runs on a ManualClock. A small inertia
plant in this file stands in for Gazebo; it is a test fixture, not a model of the robot.

TEST values below are explicit test inputs for parameters that are unknown (null) in
config/models/zlac8015d_motor.yaml. They are not values of the real drive or motor.
"""
import math
import pathlib

import pytest

from openamrobot_sil.models.drive_motor import (
    BRAKE_ENGAGED, BRAKE_ENGAGING, BRAKE_RELEASED, BRAKE_RELEASING, FAULTS, REQUIRED,
    SW_ENABLED, SW_FAULT, MissingParameter, Zlac8015dDriveModel, load_parameters,
    parameter_entries)
from openamrobot_sil.sim_time import ManualClock

PKG = pathlib.Path(__file__).resolve().parents[1]
YAML = PKG / 'config' / 'models' / 'zlac8015d_motor.yaml'
RPM = 2.0 * math.pi / 60.0

TEST = {  # TEST values, not drive or motor values
    'phase_resistance': 0.5,
    'phase_inductance': 0.0005,
    'voltage_utilisation': 1.0,
    'speed_kp': 5.0,
    'speed_ki': 50.0,
    'ramp_reference_speed': 1000.0,
}
CONFIDENCE = {'datasheet', 'supplier_confirmation', 'design_decision', 'cad', 'estimate', 'unknown'}
FIELDS = {'value', 'unit', 'source', 'confidence', 'uncertainty', 'hardware_revision', 'owner',
          'test', 'last_verified', 'measurement_item'}


def make(clock=None, bus=24.0, **extra):
    clock = clock or ManualClock()
    params = load_parameters(YAML, dict(TEST, **extra))
    return clock, Zlac8015dDriveModel(clock, params, bus)


def enable(drive):
    for cw in (0x06, 0x07, 0x0F):
        drive.write_controlword(cw)


class Plant:
    """Test stand-in for Gazebo: J dw/dt = tau - b w - load, per wheel."""

    def __init__(self, inertia=0.05, damping=0.0, load=0.0):
        self.j, self.b, self.load = inertia, damping, load
        self.w = [0.0, 0.0]
        self.angle = [0.0, 0.0]

    def run(self, clock, drive, seconds, dt=0.001, frames=True, record=None):
        for _ in range(int(round(seconds / dt))):
            clock.advance(dt)
            if frames:
                drive.on_frame()
            tau = drive.step(tuple(self.w), tuple(self.angle))
            for i in (0, 1):
                self.w[i] += dt * (tau[i] - self.b * self.w[i] - self.load) / self.j
                self.angle[i] += dt * self.w[i]
            if record is not None:
                record.append((clock.now(), self.w[0], drive.current[0]))


def test_parameter_file_rules():
    entries = parameter_entries(YAML)
    for name, e in entries.items():
        assert FIELDS <= set(e), (name, FIELDS - set(e))
        assert e['confidence'] in CONFIDENCE, name
        if e['value'] is None or e['confidence'] == 'unknown':
            assert e['value'] is None and e['confidence'] == 'unknown', name
            assert str(e['measurement_item']).startswith('MP-'), name
        else:
            assert e['source'], name
    assert set(REQUIRED) <= set(entries)
    # the supplier brake answer of 18 Oct 2026, as the actuator ledger records it
    assert entries['brake_release_time']['value'] == 0.1
    assert entries['brake_engage_time']['value'] == 0.03
    assert entries['brake_release_time']['confidence'] == 'supplier_confirmation'


def test_unknown_parameters_need_explicit_values():
    clock = ManualClock()
    with pytest.raises(MissingParameter) as e:
        Zlac8015dDriveModel(clock, load_parameters(YAML), 24.0)
    for name in TEST:
        assert name in str(e.value)
    with pytest.raises(KeyError):
        load_parameters(YAML, {'no_such_parameter': 1})


def test_target_takes_whole_counts_in_range():
    _, drive = make()
    drive.set_target(1, 1000)
    drive.set_target(2, -1000)
    with pytest.raises(ValueError):
        drive.set_target(1, 1001)
    with pytest.raises(TypeError):
        drive.set_target(1, 10.5)


def test_no_torque_until_enabled_in_sequence():
    clock, drive = make()
    plant = Plant()
    drive.set_target(1, 100)
    drive.write_controlword(0x0F)  # 0Fh without 06h first: stays switch on disabled
    plant.run(clock, drive, 0.1)
    assert plant.w[0] == 0.0 and drive.statusword() & SW_ENABLED == 0
    enable(drive)
    plant.run(clock, drive, 0.1)
    assert plant.w[0] > 0.0 and drive.statusword() & SW_ENABLED


def test_step_response_follows_the_ramp():
    clock, drive = make()
    plant = Plant()
    enable(drive)
    drive.set_target(1, 100)
    rec = []
    plant.run(clock, drive, 0.4, record=rec)
    # 6083h 0.5 s to the TEST reference of 1000 r/min: 100 r/min after 0.05 s
    at = {round(t, 3): w / RPM for t, w, _ in rec}
    assert 25.0 < at[0.025] < 50.0  # behind the 50 r/min ramp value, not ahead
    assert abs(rec[-1][1] / RPM - 100.0) < 1.0
    # bounded overshoot with the TEST gains; the real drive overshoot is unknown (RG-DRV-012)
    assert 100.0 < max(w for _, w, _ in rec) / RPM < 110.0


def test_current_limit_bounds_torque():
    clock, drive = make(current_limit=10.0)
    plant = Plant(inertia=5.0)
    enable(drive)
    drive.set_target(1, 200)
    rec = []
    plant.run(clock, drive, 0.3, record=rec)
    peak = max(abs(i) for _, _, i in rec)
    assert 9.9 < peak <= 10.0 + 1e-9
    assert abs(drive.efforts[0]) <= 0.85 * 10.0 + 1e-9
    assert drive.current_actual(1) == 100  # 6077h, 0.1 A per count


def test_back_emf_limits_top_speed():
    tops = {}
    for bus in (24.0, 18.0):  # both above the 16 V under-voltage threshold
        clock, drive = make(bus=bus)
        plant = Plant(damping=0.01)
        enable(drive)
        drive.set_target(1, 500)  # far above what the voltage allows
        plant.run(clock, drive, 3.0)
        tops[bus] = plant.w[0]
        assert tops[bus] < bus / 0.85  # never above V / Ke
        assert tops[bus] > 0.95 * bus / 0.85
    assert abs(tops[24.0] / tops[18.0] - 24.0 / 18.0) < 0.03


def test_stable_and_consistent_up_to_2_ms_steps():
    finals = []
    for dt in (0.0001, 0.002):
        clock, drive = make()
        plant = Plant(damping=0.05, load=1.0)
        enable(drive)
        drive.set_target(1, 150)
        rec = []
        plant.run(clock, drive, 1.0, dt=dt, record=rec)
        assert all(math.isfinite(w) and abs(i) <= 30.0 + 1e-9 for _, w, i in rec)
        finals.append(plant.w[0] / RPM)
    assert abs(finals[0] - 150.0) < 1.0 and abs(finals[1] - 150.0) < 1.0


def test_encoder_quantisation():
    clock, drive = make(encoder_counts_per_rev=4096)  # TEST value, counts per rev is unknown
    drive.step((12.34 * RPM, 0.0), (1.0, -1.0))
    assert drive.velocity_actual(1) == 123      # 606Ch, 0.1 r/min per count
    assert drive.position_actual(1) == 651      # floor(1 / 2pi * 4096)
    assert drive.position_actual(2) == -652
    _, unknown = make()
    unknown.step((0.0, 0.0), (1.0, 1.0))
    assert unknown.position_actual(1) is None   # no invented resolution


def test_brake_release_and_engage_delays():
    clock, drive = make()
    assert drive.brake_state(1) == BRAKE_ENGAGED
    assert drive.brake_torque_capacity(1) == 6.0
    drive.set_brake_coil(1, True)
    for t, state in ((0.099, BRAKE_RELEASING), (0.002, BRAKE_RELEASED)):
        clock.advance(t)
        drive.step((0.0, 0.0))
        assert drive.brake_state(1) == state
    assert drive.brake_torque_capacity(1) == 0.0
    assert drive.brake_state(2) == BRAKE_ENGAGED
    drive.set_brake_coil(1, False)
    for t, state in ((0.029, BRAKE_ENGAGING), (0.002, BRAKE_ENGAGED)):
        clock.advance(t)
        drive.step((0.0, 0.0))
        assert drive.brake_state(1) == state
    assert drive.brake_torque_capacity(1) == 6.0


@pytest.mark.parametrize('name', sorted(FAULTS))
def test_faults_are_reported_latched_and_need_a_reset(name):
    clock, drive = make()
    plant = Plant()
    enable(drive)
    drive.set_target(1, 100)
    plant.run(clock, drive, 0.05)
    bit, kind = FAULTS[name]
    drive.inject(name, half=None if kind == 'common' else 'high')
    plant.run(clock, drive, 0.01)
    expected = bit if kind == 'common' else bit << 16
    assert drive.error_code() == expected
    assert drive.statusword() & SW_FAULT and drive.efforts == (0.0, 0.0)
    drive.write_controlword(0x80)  # cause still present: stays latched
    assert drive.error_code() == expected
    drive.clear(name)
    plant.run(clock, drive, 0.01)
    assert drive.error_code() == expected  # latched until reset
    drive.write_controlword(0x80)
    assert drive.error_code() == 0 and not drive.statusword() & SW_FAULT
    assert not drive.statusword() & SW_ENABLED  # a new enable is needed
    enable(drive)
    assert drive.statusword() & SW_ENABLED


def test_voltage_faults_from_the_bus_input():
    clock, drive = make(bus=25.6)
    enable(drive)
    assert drive.bus_voltage_actual() == 2560
    drive.set_bus_voltage(61.0)
    clock.advance(0.001)
    drive.step((0.0, 0.0))
    assert drive.error_code() == FAULTS['over_voltage'][0]
    drive.set_bus_voltage(25.6)
    clock.advance(0.001)
    drive.step((0.0, 0.0))
    drive.write_controlword(0x80)
    assert drive.error_code() == 0
    drive.set_bus_voltage(15.0)
    clock.advance(0.001)
    drive.step((0.0, 0.0))
    assert drive.error_code() == FAULTS['under_voltage'][0]


def test_link_loss_on_the_injected_clock():
    clock, drive = make(link_loss_time=0.25)  # D-24 proposal; factory value is 0 (off)
    plant = Plant()
    enable(drive)
    drive.set_target(1, 100)
    plant.run(clock, drive, 0.3)
    w_before = plant.w[0]
    plant.run(clock, drive, 0.249, frames=False)
    assert not drive.link_lost
    plant.run(clock, drive, 0.002, frames=False)
    assert drive.link_lost and drive.link_loss_events == 1
    assert drive.error_code() == 0  # no fault, as the vendored emulator
    plant.run(clock, drive, 0.1, frames=False)
    assert plant.w[0] < w_before  # target ramps to 0 with 6084h
    # host time does not count: a paused clock never expires the timer
    clock.paused = True
    for _ in range(1000):
        clock.advance(0.01)
        drive.step(tuple(plant.w))
    assert drive.link_loss_events == 1


def test_factory_link_loss_time_is_off():
    clock, drive = make()
    plant = Plant()
    enable(drive)
    drive.set_target(1, 50)
    plant.run(clock, drive, 2.0, frames=False)
    assert not drive.link_lost and abs(plant.w[0] / RPM - 50.0) < 1.0


def test_pause_holds_the_model():
    clock, drive = make()
    plant = Plant()
    enable(drive)
    drive.set_target(1, 100)
    plant.run(clock, drive, 0.02)
    ramp, efforts = list(drive.ramp_rpm), drive.efforts
    clock.paused = True
    clock.advance(1.0)
    assert drive.step(tuple(plant.w)) == efforts and drive.ramp_rpm == ramp


def test_generation_reset_drops_target_and_needs_enable():
    clock, drive = make()
    plant = Plant()
    enable(drive)
    drive.set_target(1, 100)
    plant.run(clock, drive, 0.2)
    clock.reset(0.0)
    plant.w = [0.0, 0.0]
    assert drive.step((0.0, 0.0)) == (0.0, 0.0)
    assert drive.target_counts == [0, 0] and not drive.statusword() & SW_ENABLED
    plant.run(clock, drive, 0.1)
    assert plant.w[0] == 0.0
    enable(drive)
    drive.set_target(1, 100)
    plant.run(clock, drive, 0.1)
    assert plant.w[0] > 0.0
