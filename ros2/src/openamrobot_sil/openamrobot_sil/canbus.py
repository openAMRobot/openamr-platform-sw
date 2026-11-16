"""Open the CAN interfaces of the SIL layer. Virtual interfaces only."""
import can

VCAN_PREFIX = 'vcan'
VIRTUAL_PREFIX = 'virtual:'


class NotVirtualCan(ValueError):
    """The channel is not a Linux vcan interface."""


def open_bus(channel):
    """Return a python-can bus on a Linux vcan interface.

    Every emulator and the host test client go through this function. It refuses any
    interface whose name does not start with "vcan", so nothing in this package can talk to
    a physical CAN adapter and to real drives. "virtual:<name>" selects python-can's
    in-process bus. That is for unit tests on machines without SocketCAN and is not vcan
    evidence.
    """
    if channel.startswith(VIRTUAL_PREFIX):
        return can.Bus(interface='virtual', channel=channel, receive_own_messages=False)
    if not channel.startswith(VCAN_PREFIX):
        raise NotVirtualCan(
            'openamrobot_sil attaches to vcan interfaces only, refused: %r' % channel)
    return can.Bus(interface='socketcan', channel=channel, receive_own_messages=False)
