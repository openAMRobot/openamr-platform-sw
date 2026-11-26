"""Unit tests of the lift emulator's start-up decision and of its diagnostics (no ROS, no vcan).

The emulator starts IDLE and homed only when exactly one stop sensor is active at the moment
INIT ends, and the sensor window is +-SENSOR_WINDOW_MM around the sensor position. These tests
pin that behaviour down and check that the observe-only diagnostics describe it without
changing it. Development evidence only.
"""
from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.ctl017_gazebo import GazeboCtl017, JOINT_ZERO_MM, STATE_NAMES


class FakeBus:
    """Records what the emulator would send; nothing is sent in these tests."""

    def __init__(self):
        self.sent = []

    def send(self, msg, *args, **kwargs):
        self.sent.append(msg)


def start_state(joint_m, diag=None):
    """Attach the Gazebo plant at joint_m, let INIT end, return (emulator, lines)."""
    lines = []
    emu = GazeboCtl017(FakeBus())
    emu.diag = (lambda text: lines.append(text)) if diag is None else diag
    emu.attach_external_plant(joint_m)
    emu.t0 -= 1000                      # INIT_SETTLE_MS has long passed, as in the node
    emu.tick(0.005)                     # the first tick after start() decides the state
    return emu, lines


def test_joint_at_the_l1000_stop_starts_idle_and_homed():
    emu, _ = start_state(0.0)
    assert emu.sensor_bits() == 0b001
    assert STATE_NAMES[emu.state] == 'IDLE' and emu.homed and emu.position_valid


def test_joint_outside_the_sensor_window_starts_unhomed_and_nothing_homes_it_later():
    emu, _ = start_state(0.0015)        # 1.5 mm above the stop, window is +-1.0 mm
    assert emu.sensor_bits() == 0
    assert STATE_NAMES[emu.state] == 'UNHOMED' and not emu.homed
    for _ in range(50):
        emu.tick(0.005)
    assert STATE_NAMES[emu.state] == 'UNHOMED'


def test_the_window_edge_is_inside_and_just_beyond_it_is_outside():
    assert c.SENSOR_WINDOW_MM == 1.0 and JOINT_ZERO_MM == c.SENSOR_POS_MM[0]
    inside, _ = start_state(0.001)      # exactly 1.0 mm from the sensor position
    outside, _ = start_state(0.0011)    # 1.1 mm
    assert STATE_NAMES[inside.state] == 'IDLE'
    assert STATE_NAMES[outside.state] == 'UNHOMED'


def test_diagnostics_report_the_inputs_and_the_outcome_of_the_decision():
    _, lines = start_state(0.0015)
    text = '\n'.join(lines)
    assert 'attach joint_m=0.001500 joint_mm=6.500 true_mm=6.500' in text
    before = [ln for ln in lines if ln.startswith('finish_init before')][0]
    after = [ln for ln in lines if ln.startswith('finish_init after')][0]
    assert 'state=INIT' in before and 'sensor_bits=0b000' in before
    assert 'offsets_mm=[1.5, -173.5, -348.5]' in before and 'in_window=[False, False, False]' in before
    assert 'state=UNHOMED' in after and 'homed=False' in after


def test_a_failing_observer_cannot_change_the_result():
    def broken(text):
        raise RuntimeError('diagnostics down')

    for joint_m, want in ((0.0, 'IDLE'), (0.0015, 'UNHOMED')):
        with_bad, _ = start_state(joint_m, diag=broken)
        without, _ = start_state(joint_m)
        assert STATE_NAMES[with_bad.state] == STATE_NAMES[without.state] == want

    silent = GazeboCtl017(FakeBus())    # no observer set at all
    silent.attach_external_plant(0.0)
    silent.t0 -= 1000
    silent.tick(0.005)
    assert STATE_NAMES[silent.state] == 'IDLE'


def test_state_names_follow_the_emulator_constants():
    assert STATE_NAMES[c.INIT] == 'INIT' and STATE_NAMES[c.UNHOMED] == 'UNHOMED'
    assert STATE_NAMES[c.HOMING] == 'HOMING' and STATE_NAMES[c.IDLE] == 'IDLE'
    assert STATE_NAMES[c.MOVING] == 'MOVING' and STATE_NAMES[c.FAULT] == 'FAULT'
    assert len(STATE_NAMES) == 7
