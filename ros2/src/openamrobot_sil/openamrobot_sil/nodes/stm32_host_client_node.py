"""ROS 2 node: host test client that plays the STM32 CAN role on the three vcan buses.

Test stand-in only, see openamrobot_sil/stm32_host.py. Not started with stm32:=renode.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int8, String

from openamrobot_sil import config
from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.nodes.common import publish_json, steady_timer
from openamrobot_sil.stm32_host import BmsHost, DriveHost, LiftHost

LIFT_STOP_CMD = -1
LIFT_HOMING_CMD = -2
LIFT_CLEAR_FAULT_CMD = -3


class HostClientNode(Node):
    def __init__(self):
        super().__init__('sil_stm32_host_client')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        buses, d = cfg['buses'], cfg['drive']
        self.buses = [open_bus(buses[k]['interface']) for k in ('drive', 'aux', 'bms')]
        permit = config.permit_config(cfg)
        self.drive = DriveHost(self.buses[0], buses['drive']['zlac_node_id'], d['link_loss_ms'],
                               d['wheel_radius'], d['wheel_separation'], d['left_sign'],
                               d['right_sign']).start()
        self.lift = LiftHost(
            self.buses[1], buses['aux']['ctl017_node_id'], buses['aux']['core_node_id'], permit,
            cfg['lift']['permit']['period_ms'] / 1000.0 if permit else 0.05).start()
        self.bms = BmsHost(self.buses[2], buses['bms']['bms_address'],
                           buses['bms']['host_address'])
        self.bms.period_s = 1.0 / cfg['bms']['poll_hz']
        self.bms.start()

        self.odom_pub = self.create_publisher(Odometry, '/sil/base/odom', 10)
        self.drive_pub = self.create_publisher(String, '/sil/drive/status', 10)
        self.lift_pub = self.create_publisher(String, '/sil/lift/status', 10)
        self.bms_pub = self.create_publisher(String, '/sil/bms/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/sil/bms/battery_state', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, '/sil/stm32/can1_silent', self.on_can1_silent, 10)
        self.create_subscription(Int8, '/sil/lift/command', self.on_lift_command, 10)
        self.create_subscription(Bool, '/sil/lift/permit', self.on_permit, 10)
        self.create_subscription(Bool, '/sil/lift/permit_silent', self.on_permit_silent, 10)
        steady_timer(self, 0.02, self.on_odom)
        steady_timer(self, 0.05, self.on_status)
        self.get_logger().warning(
            'host test client active: test stand-in for the STM32 on vcan, no safety function')

    def on_cmd_vel(self, msg):
        self.drive.set_twist(msg.linear.x, msg.angular.z)

    def on_can1_silent(self, msg):
        self.drive.set_silent(msg.data)
        self.get_logger().info('CAN1 traffic %s' % ('stopped' if msg.data else 'resumed'))

    def on_lift_command(self, msg):
        value = msg.data
        if value in (0, 1, 2):
            self.lift.command(c.OP_MOVE_TO_STOP, value)
        elif value == LIFT_STOP_CMD:
            self.lift.command(c.OP_STOP)
        elif value == LIFT_HOMING_CMD:
            self.lift.command(c.OP_START_HOMING)
        elif value == LIFT_CLEAR_FAULT_CMD:
            self.lift.command(c.OP_CLEAR_FAULT)
        else:
            self.get_logger().warning('lift command %d not known' % value)

    def on_permit(self, msg):
        self.lift.set_permit(msg.data)

    def on_permit_silent(self, msg):
        self.lift.permit_silent = bool(msg.data)

    def on_odom(self):
        s = self.drive.state()
        if s['samples'] == 0:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = s['x']
        msg.pose.pose.position.y = s['y']
        msg.pose.pose.orientation.z = math.sin(s['yaw'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(s['yaw'] / 2.0)
        msg.twist.twist.linear.x = s['v']
        msg.twist.twist.angular.z = s['w']
        self.odom_pub.publish(msg)

    def on_status(self):
        publish_json(self.drive_pub, self.drive.state())
        publish_json(self.lift_pub, self.lift.state())
        b = self.bms.state()
        publish_json(self.bms_pub, b)
        if 'pack_voltage_v' in b:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = float(b['pack_voltage_v'])
            msg.current = float(b['current_a'])
            msg.percentage = float(b['soc_percent']) / 100.0
            msg.present = True
            self.battery_pub.publish(msg)

    def shutdown(self):
        for worker in (self.drive, self.lift, self.bms):
            worker.stop()
        for bus in self.buses:
            bus.shutdown()


def main():
    rclpy.init()
    node = HostClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
