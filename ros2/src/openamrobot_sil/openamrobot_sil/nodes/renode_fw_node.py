"""ROS 2 node: run the platform-fw telemetry image in Renode and publish what it reports.

transport:=console (stm32:=renode): the RenodeTest build of openamr-platform-fw at
renode_fw.FW_COMMIT; USART3 goes to a file that this node follows (JSON console).
transport:=uros (stm32:=renode_readonly): the TelemetryUrosSerialSimBench build of the same
commit; USART3 is a Renode pseudo-terminal, and this node starts the micro-ROS agent
(micro_ros_agent serial) on it, restarts it with every fresh Renode start, and follows the
image's topics /base_fw/* (DDS domain 0). Serial micro-ROS is for simulation and bench only.

Renode runs the image on the STM32H743 description plus the platform-fw additions, with FDCAN1,
FDCAN2 and FDCAN3 on CAN hubs that Renode's SocketCAN bridges join to vcan_drive, vcan_bms and
vcan_aux (the image only receives on the last two).

Published:
  /sil/fw/console        std_msgs/String, one JSON line of USART3 per message, unchanged
                         (console transport only)
  /sil/fw/wheel_speeds   std_msgs/Float64MultiArray, [ch1, ch2] in rad/s from 606Ch:01 and :02
                         as the image reports them; NaN when the image has no valid sample
  /sil/fw/status         std_msgs/String, JSON: Renode starts, boots, banner, counters, and
                         (uros) the measured ratio of Renode virtual time to wall time
Commands on /sil/fw/command (std_msgs/String):
  reset     machine reset inside the same Renode instance
  restart   quit Renode and start a fresh instance

The node exits with status 1, and sil.launch.py shuts the launch down, when the banner is not
the one of the pinned RenodeTest build ("params":"test", pinned commit), when no banner
arrives in time, or when Renode ends on its own. It reads what the image prints. It decides
nothing about motion.
"""
import json
import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension, String

from openamrobot_sil import config, renode_fw
from openamrobot_sil.nodes.common import publish_json, steady_timer


class RenodeFirmwareNode(Node):
    def __init__(self):
        super().__init__('sil_renode_firmware')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('config_file', ''), ('renode', 'renode'), ('elf', ''), ('fw_dir', ''),
            ('helpers', ''), ('work_dir', '/tmp/openamrobot_sil_renode'),
            ('can_link', 'socketcan'), ('mips', renode_fw.RENODE_MIPS),
            ('banner_timeout_s', 120.0), ('wait_for_audit', False),
            ('transport', 'console'), ('uros_agent', ''))}
        self.p = p
        if p['can_link'] not in ('socketcan', 'none'):
            raise ValueError('can_link must be socketcan or none, got %r' % p['can_link'])
        if p['transport'] not in ('console', 'uros'):
            raise ValueError('transport must be console or uros, got %r' % p['transport'])
        self.uros = p['transport'] == 'uros'
        self.can_interface = None
        self.aux_interfaces = None
        if p['can_link'] == 'socketcan':
            cfg = config.load(p['config_file'])
            self.can_interface = cfg['buses']['drive']['interface']
            self.aux_interfaces = (cfg['buses']['bms']['interface'],
                                   cfg['buses']['aux']['interface'])
            for iface in (self.can_interface,) + self.aux_interfaces:
                if not iface.startswith('vcan'):
                    raise ValueError('the Renode bridges attach to vcan only, refused: %r'
                                     % iface)
        keys = ('elf', 'fw_dir', 'helpers') + (('uros_agent',) if self.uros else ())
        if self.uros and os.environ.get('ROS_DOMAIN_ID', '0') not in ('', '0'):
            raise RuntimeError('BLOCKED: the image publishes on DDS domain %d; ROS_DOMAIN_ID '
                               'is %r' % (renode_fw.UROS_DOMAIN_ID, os.environ['ROS_DOMAIN_ID']))
        for key in keys:
            if not p[key] or not os.path.exists(p[key]):
                raise FileNotFoundError('BLOCKED: parameter %s=%r does not exist' % (key, p[key]))
        self.additions = os.path.join(p['fw_dir'], renode_fw.FW_ADDITIONS)
        os.makedirs(p['work_dir'], exist_ok=True)
        self.console_pub = self.create_publisher(String, '/sil/fw/console', 100)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/sil/fw/wheel_speeds', 10)
        self.status_pub = self.create_publisher(String, '/sil/fw/status', 10)
        self.agent = None
        self._agent_log = None
        self._pty = None
        self.uros_last = None
        self.time_ratio = renode_fw.TimeRatio()
        if self.uros:
            qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            ns = renode_fw.UROS_NAMESPACE
            self.create_subscription(String, ns + '/status', self.on_uros_status, qos)
            self.create_subscription(Float32MultiArray, ns + '/wheel_speeds', self.on_uros_wheels,
                                     qos)
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
        steady_timer(self, 0.02, self.poll)
        steady_timer(self, 0.2, self.publish_status)
        self.get_logger().info('firmware %s@%s preset %s, Renode %s, CAN1 on %s, CAN2/3 on %s' % (
            renode_fw.FW_REPOSITORY, renode_fw.FW_COMMIT,
            renode_fw.FW_PRESET_UROS if self.uros else renode_fw.FW_PRESET,
            renode_fw.RENODE_VERSION, self.can_interface or 'an internal hub only',
            self.aux_interfaces or 'nothing'))
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
        self._pty = (os.path.join(self.p['work_dir'], 'usart3-%d.pty' % self.starts)
                     if self.uros else None)
        if self._pty and os.path.lexists(self._pty):
            os.unlink(self._pty)
        with open(script, 'w', encoding='utf-8') as f:
            f.write(renode_fw.resc(os.path.abspath(self.p['elf']), os.path.abspath(self.additions),
                                   console, os.path.abspath(self.p['helpers']),
                                   self.can_interface, self.p['mips'],
                                   aux_interfaces=self.aux_interfaces, uros_pty=self._pty))
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

    def start_agent(self):
        """micro_ros_agent serial on the image's USART3 (the Renode pseudo-terminal)."""
        log = os.path.join(self.p['work_dir'], 'agent-%d.log' % self.starts)
        self._agent_log = open(log, 'w', encoding='utf-8')
        self.agent = subprocess.Popen(
            [self.p['uros_agent'], 'serial', '--dev', self._pty, '-b', '115200'],
            stdout=self._agent_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.get_logger().info('micro-ROS agent (pid %d) on %s, log %s' % (
            self.agent.pid, self._pty, log))

    def stop_agent(self):
        if self.agent is None:
            return
        if self.agent.poll() is None:
            os.killpg(self.agent.pid, signal.SIGINT)
            try:
                self.agent.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.agent.pid, signal.SIGKILL)
                self.agent.wait(timeout=5)
        self._agent_log.close()
        self.agent = None

    def stop_renode(self):
        self.stop_agent()
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
                for line in renode_fw.reset_commands(throttle=not self.uros):
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
            if self.uros:
                if self.agent is None and os.path.exists(self._pty):
                    self.start_agent()
                elif self.agent is not None and self.agent.poll() is not None:
                    self.fail('micro-ROS agent ended on its own with status %d' %
                              self.agent.returncode)
                if (self.banner is None
                        and time.monotonic() - self._t_start > self.p['banner_timeout_s']):
                    self.fail('no /base_fw/status from the image %.0f s after Renode start %d' % (
                        self.p['banner_timeout_s'], self.starts))
                return
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

    def on_uros_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.uros_last = st
            self.time_ratio.add(time.monotonic(), st.get('up_us', 0),
                                (self.starts, st.get('boot')))
            b = self.boots[-1] if self.boots else None
            new_boot = (b is None or b['renode_start'] != self.starts
                        or b['boot'] != st.get('boot'))
            if new_boot:
                try:
                    renode_fw.check_uros_status(st)
                except renode_fw.BannerError as exc:
                    self.banner = st
                    self.fail('status check failed: %s' % exc)
                self.banner = {'source': '/base_fw/status', 'commit': st.get('commit'),
                               'params': st.get('params'), 'boot': st.get('boot'),
                               'boot_record': st.get('rec')}
                self.boots.append({'renode_start': self.starts, 'boot': st.get('boot'),
                                   'boot_record': st.get('rec'), 'seq': None, 'tx': 0,
                                   'lines': 0})
                self.get_logger().info('status ok: commit %s params %s boot %s (%s)' % (
                    st.get('commit'), st.get('params'), st.get('boot'), st.get('rec')))
            b = self.boots[-1]
            b['seq'] = st.get('seq')
            b['tx'] = st.get('tx', 0)
            b['lines'] += 1
            self.lines += 1

    def on_uros_wheels(self, msg):
        ch1, ch2 = renode_fw.uros_wheel_rad_s(list(msg.data)[:2])
        out = Float64MultiArray()
        out.layout.dim = [MultiArrayDimension(label='ch1_ch2_rad_s', size=2, stride=2)]
        out.data = [ch1, ch2]
        self.wheel_pub.publish(out)

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
            'fw_preset': renode_fw.FW_PRESET_UROS if self.uros else renode_fw.FW_PRESET,
            'transport': self.p['transport'], 'aux_interfaces': self.aux_interfaces,
            'time_ratio': self.time_ratio.ratio(), 'uros_status': self.uros_last,
            'agent_running': self.agent is not None and self.agent.poll() is None,
            'renode_version': renode_fw.RENODE_VERSION,
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
