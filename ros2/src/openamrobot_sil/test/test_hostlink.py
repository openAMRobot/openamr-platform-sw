"""The host link relay without Renode: a pty pair stands in for Renode's UART pty."""
import os
import time
import tty

import pytest

from conftest import wait_for
from openamrobot_sil.hostlink import PtyRelay


@pytest.fixture
def rig(tmp_path):
    master, slave = os.openpty()         # master: the "UART" side that Renode would write
    tty.setraw(slave)
    os.set_blocking(master, False)
    renode_pty = str(tmp_path / 'renode-pty')
    os.symlink(os.ttyname(slave), renode_pty)
    relay = PtyRelay(renode_pty, str(tmp_path / 'link'))
    assert wait_for(lambda: relay.opened)
    yield master, relay, str(tmp_path / 'link')
    relay.close()
    os.close(master)
    os.close(slave)


def read_all(fd, seconds=0.5):
    data, end = b'', time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            data += os.read(fd, 65536)
        except BlockingIOError:
            time.sleep(0.01)
    return data


def test_bytes_cross_both_ways(rig):
    uart, relay, link = rig
    host = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        os.write(uart, b'{"t":"st"}\n')
        assert wait_for(lambda: relay.to_host_bytes == 11)
        assert read_all(host) == b'{"t":"st"}\n'
        os.write(host, b'\x7e\x00\x01')
        assert wait_for(lambda: relay.to_fw_bytes == 3)
        assert read_all(uart) == b'\x7e\x00\x01'      # no echo of the first direction
    finally:
        os.close(host)


def test_a_link_nobody_reads_never_blocks_the_uart_side(rig):
    """300 kB with no reader: far more than a pty buffer. The writer is never blocked, the
    surplus is dropped and counted, and a reader that comes later gets recent bytes."""
    uart, relay, link = rig
    line = b'%07d ' + b'x' * 91 + b'\n'
    for i in range(3000):
        chunk = line % i
        end = time.monotonic() + 2.0
        while chunk:
            try:
                chunk = chunk[os.write(uart, chunk):]
            except BlockingIOError:
                assert time.monotonic() < end, 'the relay stopped draining at line %d' % i
                time.sleep(0.001)
    assert wait_for(lambda: relay.to_host_bytes + relay.dropped_bytes == 300000)
    assert relay.flushes > 10 and relay.dropped_bytes > 0 and relay.errors == 0, relay.state()
    host = os.open(link, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        read_all(host, 0.2)
        os.write(uart, line % 999999)
        assert b'0999999 ' in read_all(host)
    finally:
        os.close(host)
    assert relay.state()['alive']
