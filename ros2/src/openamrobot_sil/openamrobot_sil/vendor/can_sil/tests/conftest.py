import os
import subprocess
import sys
import time

import can
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

IFACE = os.environ.get("CAN_SIL_IFACE", "vcan0")


def _sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


@pytest.fixture(scope="session")
def vcan():
    """Make sure a vcan interface exists and is up; skip cleanly when that is impossible."""
    if _sh("ip link show %s" % IFACE).returncode != 0:
        prefix = "" if os.geteuid() == 0 else "sudo -n "
        for cmd in ("modprobe vcan", "ip link add dev %s type vcan" % IFACE):
            r = _sh(prefix + cmd)
            if r.returncode != 0:
                pytest.skip("cannot create %s (%s): %s" % (IFACE, cmd, r.stderr.strip()))
    r = _sh(("" if os.geteuid() == 0 else "sudo -n ") + "ip link set up %s" % IFACE)
    state = _sh("ip -br link show %s" % IFACE).stdout
    if r.returncode != 0 and "UP" not in state and "UNKNOWN" not in state:
        pytest.skip("cannot bring up %s" % IFACE)
    return IFACE


def _bus(iface):
    return can.Bus(interface="socketcan", channel=iface, receive_own_messages=False)


@pytest.fixture
def master(vcan):
    bus = _bus(vcan)
    yield bus
    bus.shutdown()


@pytest.fixture
def make_bus(vcan):
    buses = []

    def factory():
        b = _bus(vcan)
        buses.append(b)
        return b
    yield factory
    for b in buses:
        b.shutdown()


def drain(bus):
    while bus.recv(timeout=0) is not None:
        pass


def collect(bus, seconds):
    """All frames seen on the bus for the given time."""
    end = time.monotonic() + seconds
    out = []
    while time.monotonic() < end:
        m = bus.recv(timeout=max(0.0, end - time.monotonic()))
        if m is not None:
            out.append(m)
    return out


def wait_for(cond, timeout=3.0, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()
