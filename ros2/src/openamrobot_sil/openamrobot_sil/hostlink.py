"""Host link of the firmware image: a UART of the Renode machine as a pty for a host program.

Renode offers a UART as a pty (emulation CreateUartPtyTerminal). When nothing reads that pty,
its kernel buffer fills (about 20 kB measured with Renode 1.16.1) and Renode then blocks in
the write: the whole emulation stops. A real UART does not do that: bytes sent to a cable
nobody listens on are lost and the MCU runs on.

PtyRelay keeps the Renode pty drained and offers a second pty (link_path) to the host program,
for example a micro-ROS agent in serial mode. Bytes from the image go to the host program
when it reads; when it does not, they are dropped and counted. Bytes from the host program go
to the image. A host program that closes the link and opens it again finds a running image.

Test harness only. It carries bytes; it decides nothing.
"""
import os
import select
import termios
import threading
import time
import tty


class PtyRelay:
    def __init__(self, renode_pty, link_path):
        self.renode_pty = renode_pty
        self.link_path = link_path
        self.to_host_bytes = 0       # image -> link (read by the host program, or flushed)
        self.dropped_bytes = 0       # image -> nobody: refused, the link was full
        self.flushes = 0             # times the unread bytes of a full link were thrown away
        self.to_fw_bytes = 0         # host program -> image
        self.opened = False
        self.errors = 0
        self.last_error = None
        self._run = True
        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)      # no echo, no line editing: a byte pipe
        os.set_blocking(self._master, False)
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.unlink(link_path)
        os.symlink(os.ttyname(self._slave), link_path)
        self._renode = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open_renode(self):
        try:
            fd = os.open(self.renode_pty, os.O_RDWR | os.O_NOCTTY)
        except OSError:
            return None
        tty.setraw(fd)
        self.opened = True
        return fd

    def _loop(self):
        # The loop must not end while the image runs: an undrained Renode pty stops the
        # emulation. Every error is counted and the loop goes on.
        while self._run:
            if self._renode is None:
                self._renode = self._open_renode()
                if self._renode is None:
                    time.sleep(0.05)
                    continue
            try:
                readable, _, _ = select.select([self._renode, self._master], [], [], 0.1)
            except (OSError, ValueError) as exc:
                self._error(exc)
                time.sleep(0.05)
                continue
            if self._renode in readable:
                try:
                    data = os.read(self._renode, 4096)
                except OSError as exc:
                    # Renode closed its pty (quit or restart): wait for the next one.
                    self._error(exc)
                    os.close(self._renode)
                    self._renode = None
                    self.opened = False
                    time.sleep(0.05)
                    continue
                self._to_host(data)
            if self._master in readable:
                try:
                    data = os.read(self._master, 4096)
                    if self._renode is not None:
                        os.write(self._renode, data)
                        self.to_fw_bytes += len(data)
                except OSError as exc:
                    self._error(exc)

    def _error(self, exc):
        self.errors += 1
        self.last_error = repr(exc)

    def _to_host(self, data):
        try:
            written = os.write(self._master, data)
            self.to_host_bytes += written
            self.dropped_bytes += len(data) - written
        except BlockingIOError:
            # Nobody reads the link. Old bytes are thrown away, as a cable would lose them,
            # so a reader that comes back gets current bytes.
            self.dropped_bytes += len(data)
            self.flushes += 1
            try:
                termios.tcflush(self._slave, termios.TCIFLUSH)
            except (termios.error, OSError) as exc:
                self._error(exc)
        except OSError as exc:
            self.dropped_bytes += len(data)
            self._error(exc)

    def state(self):
        return {'renode_pty': self.renode_pty, 'link': self.link_path, 'opened': self.opened,
                'to_host_bytes': self.to_host_bytes, 'dropped_bytes': self.dropped_bytes,
                'to_fw_bytes': self.to_fw_bytes, 'flushes': self.flushes,
                'errors': self.errors,
                'last_error': self.last_error, 'alive': self._thread.is_alive()}

    def close(self):
        self._run = False
        self._thread.join(timeout=1)
        for fd in (self._renode, self._master, self._slave):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        if os.path.islink(self.link_path):
            os.unlink(self.link_path)
