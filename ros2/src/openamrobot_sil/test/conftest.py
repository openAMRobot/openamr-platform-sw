"""Fixtures of the SIL protocol tests.

SIL_TEST_BUS=vcan (default): Linux SocketCAN on the vcan interface SIL_TEST_IFACE (default
vcan_siltest). The interface is created when missing. When that is impossible the tests FAIL
with a line starting "BLOCKED:". A blocked run is not a pass and is not skipped.

SIL_TEST_BUS=virtual: python-can's in-process bus, for machines without SocketCAN. A run on
it checks the emulator logic and is not vcan evidence.
"""
import itertools
import os
import subprocess
import threading
import time

import pytest

from openamrobot_sil.canbus import open_bus

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
REPO_ROOT = os.path.abspath(os.path.join(PACKAGE_DIR, '..', '..', '..'))
BUS_KIND = os.environ.get('SIL_TEST_BUS', 'vcan')
IFACE = os.environ.get('SIL_TEST_IFACE', 'vcan_siltest')
_counter = itertools.count()


@pytest.fixture(scope='session')
def can_channel_base():
    if BUS_KIND == 'virtual':
        return None
    if BUS_KIND != 'vcan':
        pytest.fail('SIL_TEST_BUS must be vcan or virtual, got %r' % BUS_KIND)
    r = subprocess.run(['sh', os.path.join(PACKAGE_DIR, 'scripts', 'sil_vcan.sh'), 'up', IFACE],
                       capture_output=True, text=True)
    if r.returncode != 0:
        pytest.fail('BLOCKED: no vcan interface %s: %s' % (IFACE, r.stderr.strip()),
                    pytrace=False)
    return IFACE


@pytest.fixture
def make_bus(can_channel_base):
    """Factory of bus handles that all sit on one CAN bus for the duration of a test."""
    channel = can_channel_base or 'virtual:sil-test-%d' % next(_counter)
    buses = []

    def factory():
        bus = open_bus(channel)
        buses.append(bus)
        return bus
    yield factory
    for bus in buses:
        bus.shutdown()


def wait_for(cond, timeout=3.0, step=0.005):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return bool(cond())


def collect(bus, seconds):
    end = time.monotonic() + seconds
    out = []
    while time.monotonic() < end:
        msg = bus.recv(timeout=max(0.0, end - time.monotonic()))
        if msg is not None:
            out.append(msg)
    return out


class FakeWheels:
    """Stands in for Gazebo: the wheels turn at the speed the drive emulator demands."""

    def __init__(self, emu, scale=1.0):
        self.emu, self.scale = emu, scale
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            left, right = self.emu.demand_rad_s()
            self.emu.set_wheel_feedback(left * self.scale, right * self.scale)
            time.sleep(0.01)

    def stop(self):
        self._run = False
        self._thread.join(timeout=1)


class FakeLift:
    """Stands in for the Gazebo lift joint: integrates the motor speed demand."""

    def __init__(self, emu, position_m=0.0, upper_m=0.350):
        self.emu, self.position_m, self.upper_m = emu, position_m, upper_m
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _loop(self):
        last = time.monotonic()
        while self._run:
            time.sleep(0.01)
            now = time.monotonic()
            self.position_m += self.emu.demand_velocity_m_s() * (now - last)
            self.position_m = min(self.upper_m, max(0.0, self.position_m))
            last = now
            self.emu.set_joint_position(self.position_m)

    def stop(self):
        self._run = False
        self._thread.join(timeout=1)
