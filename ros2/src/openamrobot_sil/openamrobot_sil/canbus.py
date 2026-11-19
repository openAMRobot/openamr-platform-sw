"""Open the CAN interfaces of the SIL layer. Virtual interfaces only."""
import os

import can

VCAN_PREFIX = 'vcan'
VIRTUAL_PREFIX = 'virtual:'
# Root of sysfs. A module constant so a test can point it at a fake tree. Not configurable
# at run time: no parameter, argument or environment variable changes it.
SYSFS_ROOT = '/sys'
ARPHRD_CAN = 280          # /sys/class/net/<iface>/type of every CAN interface
CAN_MTU = 16              # classic CAN frame (struct can_frame)
CANFD_MTU = 72            # CAN FD frame (struct canfd_frame)


class NotVirtualCan(ValueError):
    """The channel is not a Linux virtual CAN interface."""


def virtual_can_reason(channel, sysfs_root=None):
    """Return None when channel is a Linux virtual CAN interface, else why it is not.

    Three conditions, all required:
    1. the name starts with "vcan";
    2. the kernel device sits under /sys/devices/virtual/net/, so it has no hardware
       behind it. A real adapter renamed to "vcan_drive" sits under a bus device instead;
    3. the link type is CAN. A dummy or veth interface is virtual too, but is not CAN.
    """
    root = SYSFS_ROOT if sysfs_root is None else sysfs_root
    if not channel.startswith(VCAN_PREFIX) or '/' in channel:
        return 'the name does not start with "vcan"'
    link = os.path.join(root, 'class', 'net', channel)
    if not os.path.exists(link):
        return 'the interface does not exist'
    virtual = os.path.realpath(os.path.join(root, 'devices', 'virtual', 'net'))
    if os.path.dirname(os.path.realpath(link)) != virtual:
        return 'the device is not under /sys/devices/virtual/net/ (a hardware interface)'
    try:
        with open(os.path.join(link, 'type'), encoding='ascii') as f:
            link_type = int(f.read().strip())
    except (OSError, ValueError):
        return 'the link type cannot be read'
    if link_type != ARPHRD_CAN:
        return 'the link type is %d, not CAN (%d)' % (link_type, ARPHRD_CAN)
    return None


def open_bus(channel):
    """Return a python-can bus on a Linux virtual CAN interface.

    Every emulator and the host test client go through this function. It refuses any
    interface that virtual_can_reason() rejects, so nothing in this package can talk to a
    physical CAN adapter and to real drives, whatever the adapter is named.
    "virtual:<name>" selects python-can's in-process bus. That is for unit tests on machines
    without SocketCAN and is not vcan evidence.
    """
    if channel.startswith(VIRTUAL_PREFIX):
        return can.Bus(interface='virtual', channel=channel, receive_own_messages=False)
    reason = virtual_can_reason(channel)
    if reason is not None:
        raise NotVirtualCan(
            'openamrobot_sil attaches to virtual CAN interfaces only, refused %r: %s' % (
                channel, reason))
    return can.Bus(interface='socketcan', channel=channel, receive_own_messages=False)
