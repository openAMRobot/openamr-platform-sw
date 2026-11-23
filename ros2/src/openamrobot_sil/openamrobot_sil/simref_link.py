"""Client side of the local socket of the SIMULATION ONLY reference process.

The reference (openamr-platform-fw tests/sim_reference, simref_main.c) takes the STM32 role
on vcan in stm32:=simref. Its robot-side inputs (Y32 SAFETY_OK, base E-stop contact, RESET,
charger fault, contactor feedback) and its host-side inputs (session, /cmd_vel, supervisory
commands, lift commands, simulated arm evidence) arrive here as text datagrams. It answers
with a JSON status. Development evidence only, never safety evidence.
"""
import json
import os
import random
import socket
import threading

PIN_NAMES = ('y32', 'estop_contact', 'reset', 'charger_fault', 'contactor_fb')
# Nominal levels of a robot at rest: SAFETY_OK high, E-stop released, RESET not pressed,
# charger normal, contactor closed. SIL test values.
NOMINAL_PINS = {'y32': 1, 'estop_contact': 0, 'reset': 0, 'charger_fault': 0, 'contactor_fb': 1}
SUPERVISORY = ('ui_stop', 'ui_resume', 'jetson_stop_release', 'charge_cmd',
               'charge_cmd_removed', 'charger_ack')


class SimrefLink:
    """Unix datagram client. Thread-safe; keeps the pin levels and the host session."""

    def __init__(self, server_path, client_path=None, epoch=None):
        self.server_path = server_path
        self.client_path = client_path or '%s.client.%d' % (server_path, os.getpid())
        self.epoch = epoch if epoch is not None else (random.getrandbits(31) or 1)
        self.seq = 0
        self.pins = dict(NOMINAL_PINS)
        self.lock = threading.Lock()
        self.status = None
        self.status_count = 0
        try:
            os.unlink(self.client_path)
        except FileNotFoundError:
            pass
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(self.client_path)
        # never block: poll() runs in the bridge's executor, next to the /cmd_vel callback
        self.sock.setblocking(False)

    def close(self):
        self.sock.close()
        try:
            os.unlink(self.client_path)
        except FileNotFoundError:
            pass

    def send(self, line):
        try:
            self.sock.sendto(line.encode(), self.server_path)
            return True
        except (FileNotFoundError, ConnectionRefusedError, OSError):
            return False  # the process is not (yet) running; the periodic resend recovers

    def next_seq(self):
        with self.lock:
            self.seq += 1
            return self.seq

    # periodic messages
    def keepalive(self):
        self.send('subscribe')
        self.send('session %d' % self.epoch)

    def heartbeat(self):
        self.send('hb %d' % self.epoch)

    def send_pins(self):
        with self.lock:
            levels = [int(bool(self.pins[n])) for n in PIN_NAMES]
        self.send('pins %d %d %d %d %d' % tuple(levels))

    # events
    def set_pin(self, name, level):
        if name not in PIN_NAMES:
            raise ValueError('unknown pin %r' % name)
        with self.lock:
            self.pins[name] = int(bool(level))
        self.send('pin %s %d' % (name, int(bool(level))))

    def set_pins(self, levels):
        """Change several pins and send all levels in one datagram."""
        for name in levels:
            if name not in PIN_NAMES:
                raise ValueError('unknown pin %r' % name)
        with self.lock:
            for name, level in levels.items():
                self.pins[name] = int(bool(level))
        self.send_pins()

    def cmd(self, v, w):
        return self.send('cmd %d %d %.6f %.6f' % (self.epoch, self.next_seq(), v, w))

    def supervisory(self, name):
        if name not in SUPERVISORY:
            raise ValueError('unknown supervisory command %r' % name)
        return self.send('sup %s %d %d' % (name, self.epoch, self.next_seq()))

    def lift(self, op, stop=0):
        return self.send('lift %d %d' % (op, stop))

    def arms(self, pose='STOWED', moving=False, fault=False):
        m, f = int(bool(moving)), int(bool(fault))
        return self.send('arms %s %d %d %s %d %d %s' % (pose, m, f, pose, m, f, pose))

    def poll(self):
        """Read every pending status datagram; returns the latest status or None."""
        latest = None
        while True:
            try:
                data = self.sock.recv(16384)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                latest = json.loads(data.decode())
            except ValueError:
                continue
        if latest is not None:
            with self.lock:
                self.status = latest
                self.status_count += 1
        return latest
