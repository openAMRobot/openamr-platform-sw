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
  /sil/fw/tx_audit       std_msgs/String, JSON: FirmwareTxAudit over every frame FDCAN1, FDCAN2
                         and FDCAN3 transmitted since the node started (all Renode starts).
                         Renode records them at each peripheral, so the sender is known
Commands on /sil/fw/command (std_msgs/String):
  reset     machine reset inside the same Renode instance
  restart   quit Renode and start a fresh instance
  pause, resume          stop and continue the emulation (stale telemetry test)
  can1_off, can1_on      detach FDCAN1 from its hub and attach it again (bus loss test)
  can_state              ask Renode for the CCCR registers of the three FDCANs
  monitor <command>      any Renode monitor command; only with parameter test_commands:=true

USART3 of the micro-ROS image is not handed to the agent directly: a relay drains Renode's
pseudo-terminal and offers a second one to the agent (openamrobot_sil/hostlink.py), so an
agent that stops or stalls never blocks the emulation. Without a line or status message for
stale_after_s the node publishes NaN wheel speeds and sets "stale" in its status.

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

from openamrobot_sil import config, hostlink, renode_fw
from openamrobot_sil.nodes.common import publish_json, steady_timer


class RenodeFirmwareNode(Node):
    def __init__(self):
        super().__init__('sil_renode_firmware')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('config_file', ''), ('renode', 'renode'), ('elf', ''), ('fw_dir', ''),
            ('helpers', ''), ('work_dir', '/tmp/openamrobot_sil_renode'),
            ('can_link', 'socketcan'), ('mips', renode_fw.RENODE_MIPS),
            ('banner_timeout_s', 120.0), ('wait_for_audit', False),
            ('transport', 'console'), ('uros_agent', ''), ('stub', ''),
            ('hostlink_uart', ''), ('hostlink_pty', ''), ('stale_after_s', 2.0),
            ('test_commands', False))}
        self.p = p
        if p['can_link'] not in ('socketcan', 'none', 'stub'):
            raise ValueError('can_link must be socketcan, none or stub, got %r' % p['can_link'])
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
        keys = (('elf', 'fw_dir', 'helpers') + (('uros_agent',) if self.uros else ())
                + (('stub',) if p['can_link'] == 'stub' else ()))
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
        self.tx_audit_pub = self.create_publisher(String, '/sil/fw/tx_audit', 10)
        self.tx_audit = renode_fw.FirmwareTxAudit()
        self.hub_frames = 0
        self.reports = {}        # last SIL_TIME, SIL_CAN and TLM_STUB line of the Renode log
        self.time_first = None   # first and last SIL_TIME sample of the current Renode start
        self.time_last = None
        self.time_window = []
        self.stale = False
        self._stale_pub = 0.0
        self._last_line_wall = None
        self._tails = {}
        self.relay = None
        # Console image only: a second pty on a UART, relayed, for the relay tests.
        self.hostlink = None
        if p['hostlink_uart'] and p['hostlink_pty'] and not self.uros:
            self.hostlink = (p['hostlink_uart'], os.path.join(p['work_dir'], 'hostlink-renode'))
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
        self.create_subscription(String, '/sil/fw/command', self.on_command, 100)
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
        self.fw_tap = os.path.join(self.p['work_dir'], 'fwtx-%d.log' % self.starts)
        self.hub_tap = os.path.join(self.p['work_dir'], 'hub-%d.log' % self.starts)
        for old in (console, console + '.1', self.fw_tap, self.hub_tap):
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
                                   aux_interfaces=self.aux_interfaces, uros_pty=self._pty,
                                   fw_tap=self.fw_tap, hub_tap=self.hub_tap,
                                   stub=(os.path.abspath(self.p['stub'])
                                         if self.p['can_link'] == 'stub' else None),
                                   hostlink=self.hostlink))
        self._tails = {'log': [log, 0, ''], 'fw': [self.fw_tap, 0, ''],
                       'hub': [self.hub_tap, 0, '']}
        self.time_first = self.time_last = None
        self.time_window = []
        self._last_line_wall = None
        self.stale = False
        # The relay drains Renode's pty from the start; the agent gets the relay's link.
        self._link = None
        if self.uros:
            self._link = os.path.join(self.p['work_dir'], 'usart3-%d.link' % self.starts)
            self.relay = hostlink.PtyRelay(self._pty, self._link)
        elif self.hostlink:
            self.relay = hostlink.PtyRelay(self.hostlink[1], self.p['hostlink_pty'])
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
        """micro_ros_agent serial on the relay's link to the image's USART3."""
        log = os.path.join(self.p['work_dir'], 'agent-%d.log' % self.starts)
        self._agent_log = open(log, 'w', encoding='utf-8')
        self.agent = subprocess.Popen(
            [self.p['uros_agent'], 'serial', '--dev', self._link, '-b', '115200'],
            stdout=self._agent_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.get_logger().info('micro-ROS agent (pid %d) on %s, log %s' % (
            self.agent.pid, self._link, log))

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
        self.read_taps()     # the last frames of the instance that ended
        if self.relay is not None:
            self.relay.close()
            self.relay = None
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
            simple = {'pause': ['pause'], 'resume': ['start'],
                      'can1_off': ['sil_can_link "can1" "sysbus.fdcan1" 0'],
                      'can1_on': ['sil_can_link "can1" "sysbus.fdcan1" 1'],
                      'can_state': ['sil_can_state']}
            if cmd == 'reset':
                for line in renode_fw.reset_commands(throttle=not self.uros):
                    self.monitor(line)
                self.get_logger().info('machine reset inside Renode start %d' % self.starts)
            elif cmd == 'restart':
                self.stop_renode()
                self.start_renode()
            elif cmd in simple:
                for line in simple[cmd]:
                    self.monitor(line)
                self.get_logger().info('%s in Renode start %d' % (cmd, self.starts))
            elif cmd.startswith('monitor ') and self.p['test_commands']:
                self.monitor(cmd[len('monitor '):])
            else:
                self.get_logger().warning('unknown command %r' % cmd)

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
            self.read_taps()
            self.check_stale()
            if self.uros:
                if self.agent is None and self.relay.opened:
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
                self._last_line_wall = time.monotonic()
                self.console_pub.publish(String(data=raw))
                if record is not None:
                    self.on_record(record)
            if (self.banner is None
                    and time.monotonic() - self._t_start > self.p['banner_timeout_s']):
                self.fail('no boot banner from the image %.0f s after Renode start %d' % (
                    self.p['banner_timeout_s'], self.starts))

    def check_stale(self):
        """Freshness by local receipt time (I8 rev 4): without a console line or a status
        message for stale_after_s the last wheel values are not repeated, NaN is published."""
        now = time.monotonic()
        stale = (self._last_line_wall is not None
                 and now - self._last_line_wall > self.p['stale_after_s'])
        if stale and (not self.stale or now - self._stale_pub > 0.2):
            self._stale_pub = now
            self.publish_wheel_pair(math.nan, math.nan)
        self.stale = stale

    def _new_lines(self, key):
        """Complete lines appended to one of the followed files since the last call."""
        path, offset, partial = self._tails[key]
        try:
            with open(path, 'r', encoding='utf-8', errors='replace') as f:
                f.seek(offset)
                text = f.read()
                offset = f.tell()
        except OSError:
            return []
        *lines, partial = (partial + text).split('\n')
        self._tails[key] = [path, offset, partial]
        return lines

    def read_taps(self):
        """Follow the Renode log (SIL_ report lines) and the two CAN taps."""
        if not self._tails:
            return
        for line in self._new_lines('log'):
            for tag in ('SIL_TIME', 'SIL_CAN', 'TLM_STUB'):
                at = line.find(tag + ' {')
                if at < 0:
                    continue
                try:
                    data = json.loads(line[at + len(tag) + 1:])
                except ValueError:
                    continue
                self.reports[tag] = data
                if tag == 'SIL_TIME':
                    self.time_first = self.time_first or data
                    self.time_last = data
                    self.time_window = (self.time_window + [data])[-30:]
        for line in self._new_lines('fw'):
            try:
                frame = renode_fw.parse_candump(line)
            except ValueError:
                frame = None
            if frame is not None:
                self.tx_audit.feed_frame(frame)
        self.hub_frames += sum(1 for line in self._new_lines('hub') if line.strip())

    def time_status(self):
        """Renode virtual time against wall time, measured inside Renode (SIL_TIME samples of
        this start). Not the same number as time_ratio, which uses the image's uptime."""
        if not self.time_last or self.time_first is self.time_last:
            return None
        last, window = self.time_last, self.time_window
        return {'virtual_s': last['virtual_us'] / 1e6, 'wall_s': last['wall_us'] / 1e6,
                'ratio': renode_fw.time_ratio(self.time_first, last),
                'ratio_recent': renode_fw.time_ratio(window[0], window[-1]),
                'throttle': bool(last['throttle']), 'slice_us': last['slice_us'],
                'min_slice_wall_us': last['min_slice_wall_us'],
                'systick_hz': last['systick_hz']}

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
                               'tx': 0, 'lines': 0, 'tick_miss': 0})
            self.get_logger().info('banner ok: commit %s params %s boot %s (%s)' % (
                record['commit'], record['params'], record['boot'], record['boot_record']))
        elif record['t'] == 'st' and self.boots:
            b = self.boots[-1]
            b['seq'] = record.get('seq')
            b['tx'] = record.get('cnt', {}).get('tx', 0)
            b['tick_miss'] = record.get('cnt', {}).get('tick_miss', 0)
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
                                   'lines': 0, 'tick_miss': 0})
                self.get_logger().info('status ok: commit %s params %s boot %s (%s)' % (
                    st.get('commit'), st.get('params'), st.get('boot'), st.get('rec')))
            b = self.boots[-1]
            b['seq'] = st.get('seq')
            b['tx'] = st.get('tx', 0)
            b['tick_miss'] = st.get('tick_miss', 0)
            b['lines'] += 1
            self.lines += 1
            self._last_line_wall = time.monotonic()

    def on_uros_wheels(self, msg):
        ch1, ch2 = renode_fw.uros_wheel_rad_s(list(msg.data)[:2])
        self.publish_wheel_pair(ch1, ch2)

    def publish_wheels(self, status):
        try:
            ch1, ch2 = renode_fw.wheel_rad_s(status)
        except (KeyError, TypeError):
            ch1 = ch2 = math.nan
        self.publish_wheel_pair(ch1, ch2)

    def publish_wheel_pair(self, ch1, ch2):
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
            'fw_tx_total': sum(b['tx'] for b in self.boots), 'fatal': self.fatal,
            'relay': self.relay.state() if self.relay else None,
            'time': self.time_status(), 'stale': self.stale,
            'stale_after_s': self.p['stale_after_s'], 'reports': self.reports,
            'hub_frames': self.hub_frames,
            'tick_miss_total': sum(b['tick_miss'] for b in self.boots)})
        audit = self.tx_audit.summary()
        audit.update(renode_starts=self.starts, fw_tx_total=sum(b['tx'] for b in self.boots))
        publish_json(self.tx_audit_pub, audit)

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
