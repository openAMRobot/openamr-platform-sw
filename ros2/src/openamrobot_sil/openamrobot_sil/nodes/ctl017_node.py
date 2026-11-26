"""ROS 2 node: CTL-017 lift node emulator on vcan_aux, bridged to the Gazebo lift joint."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from openamrobot_sil import config
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.ctl017_gazebo import GazeboCtl017, STATE_NAMES
from openamrobot_sil.nodes.common import joint_value, publish_json, steady_timer


class Ctl017Node(Node):
    def __init__(self):
        super().__init__('sil_ctl017')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        self.use_gazebo = self.declare_parameter('gazebo', True).value
        self.joint = cfg['lift']['joint']
        self.bus = open_bus(cfg['buses']['aux']['interface'])
        permit = config.permit_config(cfg)
        self.emu = GazeboCtl017(self.bus, cfg['buses']['aux']['ctl017_node_id'], permit=permit)
        self.emu.diag = self._diag
        self.started = False
        # start-up diagnostics, observe only (LIFTDIAG lines in the launch log)
        self._dj = {'msgs': 0, 'first': None, 'last_m': None, 'snap5': False, 'snap10': False}
        self._last_state, self._last_bits, self._bit_events = None, None, 0
        self._dg = {'t0': time.monotonic(), 'graph': {}, 'times': [], 'stamps': [], 'done': False}
        self._safe_diag(self._diag, 'subscribed /sil/gz/joint_states as %s (reliable, volatile, '
                                    'depth 10); publishing /sil/lift/gz_cmd_vel' %
                        self.get_fully_qualified_name())
        self._diag_timer = self.create_timer(0.1, self._on_diag_timer)
        self.cmd_pub = self.create_publisher(Float64, '/sil/lift/gz_cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/sil/lift/emulator_state', 10)
        self.create_subscription(JointState, '/sil/gz/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/sil/lift/inject', self.on_inject, 10)
        steady_timer(self, 0.01, self.on_cmd)
        steady_timer(self, 0.05, self.on_state)
        self.inject = {
            'stall': lambda: self.emu.inject_stall(True),
            'stall-clear': lambda: self.emu.inject_stall(False),
            'hall-supply-off': lambda: self.emu.set_hall_supply(False),
            'hall-supply-on': lambda: self.emu.set_hall_supply(True),
            'motor-power-off': lambda: self.emu.set_motor_power(False),
            'motor-power-on': lambda: self.emu.set_motor_power(True),
            'estop2-pressed': lambda: self.emu.set_estop2(True),
            'estop2-released': lambda: self.emu.set_estop2(False),
            'power-cycle': self.emu.power_cycle,
        }
        if not self.use_gazebo:
            self.start_emulator()
        else:
            self.get_logger().info('waiting for joint %s from Gazebo' % self.joint)

    def _diag(self, text):
        """One LIFTDIAG line: wall clock (as the other log stamps and the vcan captures) and the
        monotonic clock of the emulator's own t0 / _now_ms."""
        self.get_logger().info('LIFTDIAG wall=%.3f mono_ms=%d %s' % (
            time.time(), int(time.monotonic() * 1000), text))

    @staticmethod
    def _safe_diag(fn, *args):
        """Run a diagnostics call; a failure in it must never reach the node's normal paths."""
        try:
            fn(*args)
        except Exception:
            pass

    def _on_diag_timer(self):
        """Graph visibility of the bridge as this node sees it, and the first-second count."""
        self._safe_diag(self._diag_graph)

    def _diag_graph(self):
        g = self._dg
        now = time.monotonic()
        if now - g['t0'] > 30.0:
            self._diag_timer.cancel()
            return
        for kind, topic in (('publishers', '/sil/gz/joint_states'),
                            ('subscribers', '/sil/lift/gz_cmd_vel')):
            infos = (self.get_publishers_info_by_topic(topic) if kind == 'publishers'
                     else self.get_subscriptions_info_by_topic(topic))
            who = sorted('%s %s %s' % (i.node_name, i.qos_profile.reliability.name,
                                       i.qos_profile.durability.name) for i in infos)
            state = (len(infos), tuple(who))
            if g['graph'].get((kind, topic)) != state:
                g['graph'][(kind, topic)] = state
                self._diag('graph %s of %s = %d %s' % (kind, topic, len(infos), who))
        if not g['done'] and g['times'] and now - g['times'][0] > 1.0:
            g['done'] = True
            first_second = [t for t in g['times'] if t - g['times'][0] <= 1.0]
            self._diag('first_second messages=%d header_stamps %s .. %s' % (
                len(first_second), g['stamps'][0], g['stamps'][len(first_second) - 1]))

    def _diag_joint(self, msg, position):
        d = self._dj
        d['msgs'] += 1
        g = self._dg
        if len(g['times']) < 400:
            g['times'].append(time.monotonic())
            g['stamps'].append('%d.%09d' % (msg.header.stamp.sec, msg.header.stamp.nanosec))
        stamp = '%d.%09d' % (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if d['msgs'] <= 3:
            self._diag('joint_msg n=%d joints=%d has_%s=%s lift_m=%s header_stamp=%s started=%s' % (
                d['msgs'], len(msg.name), self.joint, position is not None,
                None if position is None else '%.6f' % position, stamp, self.started))
        if position is None:
            return
        now = time.monotonic()
        if d['first'] is None:
            d['first'] = now
            self._diag('first_lift_joint_msg n=%d lift_m=%.6f header_stamp=%s' % (
                d['msgs'], position, stamp))
        since = now - d['first']
        if since <= 10.0 and (d['last_m'] is None or abs(position - d['last_m']) >= 5e-5):
            d['last_m'] = position
            self._diag('joint_pos n=%d t_since_first=%.3f lift_m=%.6f header_stamp=%s' % (
                d['msgs'], since, position, stamp))
        for key, at in (('snap5', 5.0), ('snap10', 10.0)):
            if not d[key] and since >= at:
                d[key] = True
                self._diag('later_observation t_since_first=%.3f lift_m=%.6f (not the initial '
                           'position)' % (since, position))

    def _diag_state(self, d):
        state, bits = d['state'], d['stop_sensors']
        name = STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else state
        if self._last_state is None:
            self._diag('first_publication state=%s true_mm=%.3f sensor_bits=0b%s position_mm=%s '
                       'joint_msgs_so_far=%d' % (
                           name, d['true_mm'], format(bits, '03b'), d['position_mm'],
                           self._dj['msgs']))
        elif state != self._last_state:
            self._diag('state_transition %s->%s true_mm=%.3f sensor_bits=0b%s' % (
                STATE_NAMES[self._last_state] if 0 <= self._last_state < len(STATE_NAMES)
                else self._last_state, name, d['true_mm'], format(bits, '03b')))
        if self._last_bits is not None and bits != self._last_bits and self._bit_events < 20:
            self._bit_events += 1
            self._diag('sensor_bits_change 0b%s->0b%s true_mm=%.3f state=%s' % (
                format(self._last_bits, '03b'), format(bits, '03b'), d['true_mm'], name))
        self._last_state, self._last_bits = state, bits

    def start_emulator(self):
        self.emu.start()
        self.started = True
        self.get_logger().info(
            'CTL-017 emulator node 0x%02X started, plant %s, permit gate %s' % (
                self.emu.node_id, 'gazebo' if self.emu.external else 'internal',
                'on' if self.emu.permit_cfg else 'off'))

    def on_joints(self, msg):
        position = joint_value(msg, self.joint, 'position')
        self._safe_diag(self._diag_joint, msg, position)
        if position is None or not self.use_gazebo:
            return
        if not self.started:
            self._safe_diag(self._diag, 'attaching joint_msgs_received=%d' % self._dj['msgs'])
            self.emu.attach_external_plant(position)
            self.start_emulator()
        else:
            self.emu.set_joint_position(position)

    def on_cmd(self):
        msg = Float64()
        msg.data = self.emu.demand_velocity_m_s() if self.started else 0.0
        self.cmd_pub.publish(msg)

    def on_state(self):
        if self.started:
            state = self.emu.state_dict()
            publish_json(self.state_pub, state)
            self._safe_diag(self._diag_state, state)

    def on_inject(self, msg):
        fn = self.inject.get(msg.data.strip())
        if fn is None:
            self.get_logger().warning('unknown injection %r, known: %s' % (
                msg.data, ', '.join(sorted(self.inject))))
            return
        fn()
        self.get_logger().info('injected %s' % msg.data.strip())

    def shutdown(self):
        if self.started:
            self.emu.stop()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = Ctl017Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
