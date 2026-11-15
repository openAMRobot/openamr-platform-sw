"""ROS 2 node: ZLAC8015D emulator on vcan_drive, bridged to the Gazebo wheels."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from openamrobot_sil import config
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.nodes.common import joint_value, publish_json, steady_timer
from openamrobot_sil.zlac_gazebo import GazeboZlac8015d, wheel_speeds_to_twist


class ZlacNode(Node):
    def __init__(self):
        super().__init__('sil_zlac8015d')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        self.drive = cfg['drive']
        self.bus = open_bus(cfg['buses']['drive']['interface'])
        # stm32:=renode_readonly: SIL receive PDOs on, 2000h stored before the run.
        rpdo = self.declare_parameter('rpdo', False).value
        stored = self.declare_parameter('stored_link_loss', False).value
        self.emu = GazeboZlac8015d(
            self.bus, cfg['buses']['drive']['zlac_node_id'], self.drive['heartbeat_ms'],
            self.drive['left_sign'], self.drive['right_sign'], rpdo=rpdo,
            stored_link_loss_ms=self.drive['link_loss_ms'] if stored else None).start()
        self.cmd_pub = self.create_publisher(Twist, '/sil/drive/gz_cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/sil/drive/emulator_state', 10)
        self.create_subscription(JointState, '/sil/gz/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/sil/drive/inject', self.on_inject, 10)
        steady_timer(self, 0.02, self.on_cmd)
        steady_timer(self, 0.05, lambda: publish_json(self.state_pub, self.emu.state()))
        self.inject = {
            'oc-lo': lambda: self.emu.inject_over_current('low'),
            'oc-hi': lambda: self.emu.inject_over_current('high'),
            'uv': self.emu.inject_under_voltage,
            'enc-lo': lambda: self.emu.inject_encoder_error('low'),
            'enc-hi': lambda: self.emu.inject_encoder_error('high'),
            'busoff': lambda: self.emu.set_bus_off(True),
            'busoff-clear': lambda: self.emu.set_bus_off(False),
            'power-cycle': self.emu.power_cycle,
        }
        self.get_logger().info('ZLAC8015D emulator node %d on %s' % (
            self.emu.node_id, cfg['buses']['drive']['interface']))

    def on_joints(self, msg):
        left = joint_value(msg, 'left_wheel_joint', 'velocity')
        right = joint_value(msg, 'right_wheel_joint', 'velocity')
        if left is not None and right is not None:
            self.emu.set_wheel_feedback(left, right)

    def on_cmd(self):
        left, right = self.emu.demand_rad_s()
        msg = Twist()
        msg.linear.x, msg.angular.z = wheel_speeds_to_twist(
            left, right, self.drive['wheel_radius'], self.drive['wheel_separation'])
        self.cmd_pub.publish(msg)

    def on_inject(self, msg):
        fn = self.inject.get(msg.data.strip())
        if fn is None:
            self.get_logger().warning('unknown injection %r, known: %s' % (
                msg.data, ', '.join(sorted(self.inject))))
            return
        fn()
        self.get_logger().info('injected %s' % msg.data.strip())

    def shutdown(self):
        self.emu.stop()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = ZlacNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
