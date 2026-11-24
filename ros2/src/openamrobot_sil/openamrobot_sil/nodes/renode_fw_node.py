"""ROS 2 node: run the platform-fw telemetry image in Renode and publish its console.

The image is the RenodeTest build of openamr-platform-fw at renode_fw.FW_COMMIT. Renode runs
it on the STM32H743 description plus the platform-fw additions, with FDCAN1 on a CAN hub that
Renode's SocketCAN bridge joins to vcan_drive. USART3 goes to a file that this node follows.

Published:
  /sil/fw/console        std_msgs/String, one JSON line of USART3 per message, unchanged
  /sil/fw/wheel_speeds   std_msgs/Float64MultiArray, [ch1, ch2] in rad/s from 606Ch:01 and :02
                         as the image reports them; NaN when the image has no valid sample
  /sil/fw/status         std_msgs/String, JSON: Renode starts, boots, banner, counters
Commands on /sil/fw/command (std_msgs/String):
  reset     machine reset inside the same Renode instance
  restart   quit Renode and start a fresh instance

The node exits with status 1, and sil.launch.py shuts the launch down, when the banner is not
the one of the pinned RenodeTest build ("params":"test", pinned commit), when no banner
arrives in time, or when Renode ends on its own. It reads what the image prints. It decides
nothing about motion.
"""
import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

from openamrobot_sil import config, renode_fw
from openamrobot_sil.nodes.common import outside_gazebo_timer, publish_json


class RenodeFirmwareNode(Node):
    def __init__(self):
        super().__init__('sil_renode_firmware')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('config_file', ''), ('renode', 'renode'), ('elf', ''), ('fw_dir', ''),
            ('helpers', ''), ('work_dir', '/tmp/openamrobot_sil_renode'),
            ('can_link', 'socketcan'), ('mips', renode_fw.RENODE_MIPS),
            ('banner_timeout_s', 120.0), ('wait_for_audit', False))}
        self.p = p
        if p['can_link'] not in ('socketcan', 'none'):
            raise ValueError('can_link must be socketcan or none, got %r' % p['can_link'])
        self.can_interface = None
        if p['can_link'] == 'socketcan':
            cfg = config.load(p['config_file'])
            self.can_interface = cfg['buses']['drive']['interface']
            if not self.can_interface.startswith('vcan'):
                raise ValueError('the Renode bridge attaches to vcan only, refused: %r'
                                 % self.can_interface)
        for key in ('elf', 'fw_dir', 'helpers'):
            if not p[key] or not os.path.exists(p[key]):
                raise FileNotFoundError('BLOCKED: parameter %s=%r does not exist' % (key, p[key]))
        self.additions = os.path.join(p['fw_dir'], renode_fw.FW_ADDITIONS)
        os.makedirs(p['work_dir'], exist_ok=True)
        self.console_pub = self.create_publisher(String, '/sil/fw/console', 100)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/sil/fw/wheel_speeds', 10)
        self.status_pub = self.create_publisher(String, '/sil/fw/status', 10)
        self.create_subscription(String, '/sil/fw/command', self.on_command, 10)
        self.audit_seen = not p['wait_for_audit']
        if p['wait_for_audit']:
            self.create_subscription(String, '/sil/drive_bus/audit', self.on_audit, 10)
        self.lock = threading.Lock()
        self.proc = None
        self.starts = 0
        self.boots = []          # one entry per banner: renode start, boot id, record, last seq, tx
        self.banner = None
        self.lines = 0
        self.fatal = None
        self.expected_exit = False
        self._console = None
        self._parser = None
        self._t_start = None
        outside_gazebo_timer(self, 0.02, self.poll)
        outside_gazebo_timer(self, 0.2, self.publish_status)
        self.get_logger().info('firmware %s@%s preset %s, Renode %s, CAN1 on %s' % (
            renode_fw.FW_REPOSITORY, renode_fw.FW_COMMIT, renode_fw.FW_PRESET,
            renode_fw.RENODE_VERSION, self.can_interface or 'an internal hub only'))
        if self.audit_seen:
            self.start_renode()

    # ---- Renode process --------------------------------------------------------------
    def start_renode(self):
        self.starts += 1
        console = os.path.join(self.p['work_dir'], 'usart3-%d.log' % self.starts)
        script = os.path.join(self.p['work_dir'], 'sil-%d.resc' % self.starts)
        log = os.path.join(self.p['work_dir'], 'renode-%d.log' % self.starts)
        for old in (console, console + '.1'):
            if os.path.exists(old):
                os.unlink(old)
        self._console_path = console
        with open(script, 'w', encoding='utf-8') as f:
            f.write(renode_fw.resc(os.path.abspath(self.p['elf']), os.path.abspath(self.additions),
                                   console, os.path.abspath(self.p['helpers']),
                                   self.can_interface, self.p['mips']))
        self._log = open(log, 'w', encoding='utf-8')
        self.proc = subprocess.Popen(
            [self.p['renode'], '--disable-gui', '--console'], stdin=subprocess.PIPE,
            stdout=self._log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        self.monitor('include @%s' % script)
        # Renode creates the file (and renames one that exists), so it is opened once it is there.
        self._console = None
        self._parser = renode_fw.ConsoleParser()
        self._t_start = time.monotonic()
        self.banner = None
        self.get_logger().info('Renode start %d (pid %d), script %s, console %s' % (
            self.starts, self.proc.pid, script, console))

    def monitor(self, command):
        self.proc.stdin.write(command + '\n')
        self.proc.stdin.flush()

    def stop_renode(self):
        if self.proc is None:
            return
        self.expected_exit = True
        try:
            self.monitor('quit')
            self.proc.wait(timeout=15)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            self.proc.kill()
            self.proc.wait(timeout=5)
        self._log.close()
        if self._console:
            self._console.close()
        self.proc = None
        self.expected_exit = False

    # ---- ROS callbacks ---------------------------------------------------------------
    def on_audit(self, _msg):
        if not self.audit_seen:
            self.audit_seen = True
            self.start_renode()

    def on_command(self, msg):
        cmd = msg.data.strip()
        with self.lock:
            if self.proc is None:
                self.get_logger().warning('command %r: Renode is not running' % cmd)
                return
            if cmd == 'reset':
                for line in renode_fw.reset_commands():
                    self.monitor(line)
                self.get_logger().info('machine reset inside Renode start %d' % self.starts)
            elif cmd == 'restart':
                self.stop_renode()
                self.start_renode()
            else:
                self.get_logger().warning('unknown command %r (reset, restart)' % cmd)

    def fail(self, reason):
        self.fatal = reason
        self.get_logger().error(reason)
        self.publish_status()
        raise SystemExit(1)

    def poll(self):
        with self.lock:
            if self.proc is None:
                return
            if self.proc.poll() is not None and not self.expected_exit:
                self.fail('Renode ended on its own with status %d (log in %s)' % (
                    self.proc.returncode, self.p['work_dir']))
            if self._console is None and os.path.exists(self._console_path):
                self._console = open(self._console_path, 'r', encoding='utf-8', errors='replace')
            text = self._console.read() if self._console is not None else ''
            for raw, record in self._parser.feed(text):
                self.lines += 1
                self.console_pub.publish(String(data=raw))
                if record is not None:
                    self.on_record(record)
            if (self.banner is None
                    and time.monotonic() - self._t_start > self.p['banner_timeout_s']):
                self.fail('no boot banner from the image %.0f s after Renode start %d' % (
                    self.p['banner_timeout_s'], self.starts))

    def on_record(self, record):
        if record['t'] == 'boot':
            try:
                renode_fw.check_banner(record)
            except renode_fw.BannerError as exc:
                self.banner = record
                self.fail('banner check failed: %s' % exc)
            self.banner = record
            self.boots.append({'renode_start': self.starts, 'boot': record.get('boot'),
                               'boot_record': record.get('boot_record'), 'seq': None,
                               'tx': 0, 'lines': 0})
            self.get_logger().info('banner ok: commit %s params %s boot %s (%s)' % (
                record['commit'], record['params'], record['boot'], record['boot_record']))
        elif record['t'] == 'st' and self.boots:
            b = self.boots[-1]
            b['seq'] = record.get('seq')
            b['tx'] = record.get('cnt', {}).get('tx', 0)
            b['lines'] += 1
            self.publish_wheels(record)

    def publish_wheels(self, status):
        try:
            ch1, ch2 = renode_fw.wheel_rad_s(status)
        except (KeyError, TypeError):
            ch1 = ch2 = math.nan
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label='ch1_ch2_rad_s', size=2, stride=2)]
        msg.data = [ch1, ch2]
        self.wheel_pub.publish(msg)

    def publish_status(self):
        publish_json(self.status_pub, {
            'fw_repository': renode_fw.FW_REPOSITORY, 'fw_commit': renode_fw.FW_COMMIT,
            'fw_preset': renode_fw.FW_PRESET, 'renode_version': renode_fw.RENODE_VERSION,
            'can_link': self.p['can_link'], 'can_interface': self.can_interface,
            'renode_starts': self.starts, 'running': self.proc is not None,
            'banner': self.banner, 'boots': self.boots, 'lines': self.lines,
            'bad_lines': len(self._parser.bad_lines) if self._parser else 0,
            'fw_tx_total': sum(b['tx'] for b in self.boots), 'fatal': self.fatal})

    def shutdown(self):
        with self.lock:
            self.stop_renode()


def main():
    rclpy.init()
    node = RenodeFirmwareNode()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code or 0
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
