"""ROS 2 node: minimal Daly BMS emulator on vcan_bms."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openamrobot_sil import config
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.daly_bms import DalyBmsEmulator
from openamrobot_sil.nodes.common import publish_json, steady_timer


class DalyNode(Node):
    def __init__(self):
        super().__init__('sil_daly_bms')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        b, bus_cfg = cfg['bms'], cfg['buses']['bms']
        self.bus = open_bus(bus_cfg['interface'])
        self.emu = DalyBmsEmulator(
            self.bus, bus_cfg['bms_address'], bus_cfg['host_address'], b['n_cells'],
            b['n_temps'], b['pack_voltage_v'], b['current_a'], b['soc_percent'],
            b['capacity_ah']).start()
        self.state_pub = self.create_publisher(String, '/sil/bms/emulator_state', 10)
        self.create_subscription(String, '/sil/bms/inject', self.on_inject, 10)
        steady_timer(self, 0.2, lambda: publish_json(self.state_pub, self.emu.state()))
        self.get_logger().info('Daly BMS emulator (protocol unconfirmed) on %s' %
                               bus_cfg['interface'])

    def on_inject(self, msg):
        """Commands: failure <byte> <mask>, clear, mos <charge 0|1> <discharge 0|1>,
        soc <percent>, voltage <V>, current <A>, offline, online."""
        words = msg.data.split()
        try:
            cmd, args = words[0], words[1:]
            if cmd == 'failure':
                self.emu.inject_failure(int(args[0], 0), int(args[1], 0))
            elif cmd == 'clear':
                self.emu.clear_failures()
            elif cmd == 'mos':
                self.emu.set_mos(int(args[0]), int(args[1]))
            elif cmd == 'soc':
                self.emu.set_values(soc_percent=float(args[0]))
            elif cmd == 'voltage':
                self.emu.set_values(pack_voltage_v=float(args[0]))
            elif cmd == 'current':
                self.emu.set_values(current_a=float(args[0]))
            elif cmd in ('offline', 'online'):
                self.emu.offline = cmd == 'offline'
            else:
                raise ValueError('unknown command')
        except (IndexError, ValueError) as exc:
            self.get_logger().warning('bad injection %r: %s' % (msg.data, exc))
            return
        self.get_logger().info('injected %s' % msg.data)

    def shutdown(self):
        self.emu.stop()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = DalyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
