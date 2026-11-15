"""Test helper: start the renode_firmware node and follow what it publishes.

Shared by test_renode (Renode only, no vcan) and test_integration/test_sil_renode.py
(Gazebo-integrated). Missing Renode or firmware is a FAILED test with a "BLOCKED:" line.
"""
import json
import math
import os
import threading
import time

import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from openamrobot_sil import renode_fw


def renode_inputs():
    """(renode, elf, fw_dir) from the environment, or a BLOCKED failure."""
    renode = os.environ.get('SIL_RENODE', '')
    elf = os.environ.get('SIL_FW_ELF', '')
    fw_dir = os.environ.get('SIL_FW_DIR', '')
    for name, path in (('SIL_RENODE', renode), ('SIL_FW_ELF', elf), ('SIL_FW_DIR', fw_dir)):
        if not path or not os.path.exists(path):
            pytest.fail('BLOCKED: %s=%r does not exist (scripts/sil_renode_firmware.sh sets it)'
                        % (name, path), pytrace=False)
    return renode, elf, fw_dir


def pinned_line():
    return 'firmware %s@%s preset %s, Renode %s' % (
        renode_fw.FW_REPOSITORY, renode_fw.FW_COMMIT, renode_fw.FW_PRESET,
        renode_fw.RENODE_VERSION)


class FwProbe(Node):
    def __init__(self, name='sil_fw_probe'):
        super().__init__(name)
        self.lock = threading.Lock()
        self.records = []        # (monotonic time, record)
        self.wheels = []         # (monotonic time, (ch1, ch2))
        self.status = None
        self.audit = None
        self.create_subscription(String, '/sil/fw/console', self.on_console, 200)
        self.create_subscription(String, '/sil/fw/status', self.on_status, 10)
        self.create_subscription(String, '/sil/drive_bus/audit', self.on_audit, 10)
        self.create_subscription(Float64MultiArray, '/sil/fw/wheel_speeds', self.on_wheels, 50)
        self.cmd_pub = self.create_publisher(String, '/sil/fw/command', 10)

    def on_console(self, msg):
        with self.lock:
            self.records.append((time.monotonic(), json.loads(msg.data)))

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_audit(self, msg):
        self.audit = json.loads(msg.data)

    def on_wheels(self, msg):
        with self.lock:
            self.wheels.append((time.monotonic(), tuple(msg.data)))

    def command(self, text):
        self.cmd_pub.publish(String(data=text))

    def boots(self):
        """Boots the node has seen, from /sil/fw/status (republished every 200 ms, so a probe
        that subscribed after a banner still sees it; console messages are not replayed)."""
        return list((self.status or {}).get('boots', []))

    def banner(self):
        return (self.status or {}).get('banner')

    def boot_lines(self, boot_id, t0):
        """Status lines of boot boot_id that arrived after t0, in order."""
        with self.lock:
            return [r for t, r in self.records
                    if t >= t0 and r['t'] == 'st' and r.get('boot') == boot_id]

    def lines_after(self, t0):
        with self.lock:
            return [(t, r) for t, r in self.records if t >= t0 and r['t'] == 'st']

    def last_status_line(self):
        with self.lock:
            for _, r in reversed(self.records):
                if r['t'] == 'st':
                    return r
        return None

    def wait(self, cond, timeout, step=0.05):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return True
            except (KeyError, TypeError, IndexError, AttributeError):
                pass
            time.sleep(step)
        return False


class Spinner:
    def __init__(self, node):
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown()


def run_after_banner(probe, command, timeout=120.0):
    """Send command ('reset' or 'restart'); return (boot before, new boot, time of the command).

    A boot is the node's record {renode_start, boot, boot_record, ...} of one banner."""
    assert probe.wait(lambda: probe.boots(), 30.0), 'no boot before %s' % command
    before = probe.boots()
    t_cmd = time.monotonic()
    probe.command(command)
    assert probe.wait(lambda: len(probe.boots()) > len(before), timeout), (
        'no new banner %.0f s after %s' % (timeout, command))
    return before[-1], probe.boots()[len(before)], t_cmd


def is_nan(x):
    return isinstance(x, float) and math.isnan(x)
