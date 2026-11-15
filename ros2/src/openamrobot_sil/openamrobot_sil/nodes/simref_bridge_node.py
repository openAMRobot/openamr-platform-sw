"""ROS 2 node: bridge between ROS and the SIMULATION ONLY reference process (stm32:=simref).

The only path from ROS to the drive in this mode: /cmd_vel, the output of the existing Nav2
stop gate and command arbitration (controller -> velocity_smoother -> collision_monitor ->
/cmd_vel), becomes the reference's motion command. The reference alone decides whether the
drive follows it. This node publishes the reference's state and reasons, and the odometry
and wheel joint states it computes from 606Ch read over CAN1. It is the only publisher of
/odom, /joint_states and the odom -> base_footprint transform in this mode.

Simulated robot pins (Y32, E-stop contact, RESET, charger fault, contactor feedback) are set
with /sil/simref/pin ("<name> <0|1>"); supervisory commands with /sil/simref/supervisory.
Development evidence only, never safety evidence.
"""
import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from tf2_ros import TransformBroadcaster

from openamrobot_sil import config
from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.nodes.common import steady_timer
from openamrobot_sil.nodes.stm32_host_client_node import (LIFT_CLEAR_FAULT_CMD, LIFT_HOMING_CMD,
                                                           LIFT_STOP_CMD)
from openamrobot_sil.simref_link import SimrefLink


class SimrefBridgeNode(Node):
    def __init__(self):
        super().__init__('sil_simref_bridge')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        sref = cfg.get('simref') or {}
        path = sref.get('socket', '/tmp/openamrobot_simref.sock')
        self.link = SimrefLink(self.declare_parameter('socket', path).value)
        self.arms_pose = sref.get('arms_pose', 'STOWED')
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_pub = self.create_publisher(String, '/sil/simref/state', 10)
        self.reason_pub = self.create_publisher(String, '/sil/simref/reasons', 50)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, '/sil/simref/pin', self.on_pin, 10)
        self.create_subscription(String, '/sil/simref/supervisory', self.on_supervisory, 10)
        self.create_subscription(String, '/sil/simref/arms', self.on_arms, 10)
        self.create_subscription(Int8, '/sil/lift/command', self.on_lift_command, 10)
        self.last_reason_n = -1
        self.last_log_n = 0
        self.arms_moving = False
        steady_timer(self, 1.0, self.link.keepalive)
        steady_timer(self, 0.1, self.on_period)
        steady_timer(self, 0.02, self.on_status)
        self.link.keepalive()
        self.get_logger().warning(
            'stm32:=simref bridge: SIMULATION ONLY reference in the STM32 role, socket %s, '
            'session epoch %d. Not safety evidence.' % (self.link.server_path, self.link.epoch))

    # ROS -> reference
    def on_cmd_vel(self, msg):
        self.link.cmd(msg.linear.x, msg.angular.z)

    def on_pin(self, msg):
        """"<name> <0|1>" or several pairs; several pairs reach the reference in one datagram
        (the same instant for every pin)."""
        words = msg.data.split()
        try:
            if len(words) < 2 or len(words) % 2:
                raise ValueError('expected <name> <0|1> pairs')
            pairs = [(words[i], int(words[i + 1])) for i in range(0, len(words), 2)]
            if len(pairs) == 1:
                self.link.set_pin(*pairs[0])
            else:
                self.link.set_pins(dict(pairs))
        except ValueError as exc:
            self.get_logger().warning('bad pin message %r: %s' % (msg.data, exc))

    def on_supervisory(self, msg):
        try:
            self.link.supervisory(msg.data.strip())
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def on_arms(self, msg):
        words = msg.data.split()
        if words:
            self.arms_pose = words[0]
            self.arms_moving = len(words) > 1 and words[1] == '1'

    def on_lift_command(self, msg):
        value = msg.data
        if value in (0, 1, 2):
            self.link.lift(c.OP_MOVE_TO_STOP, value)
        elif value == LIFT_STOP_CMD:
            self.link.lift(c.OP_STOP)
        elif value == LIFT_HOMING_CMD:
            self.link.lift(c.OP_START_HOMING)
        elif value == LIFT_CLEAR_FAULT_CMD:
            self.link.lift(c.OP_CLEAR_FAULT)

    def on_period(self):
        self.link.heartbeat()
        self.link.send_pins()
        self.link.arms(self.arms_pose, self.arms_moving)

    # reference -> ROS
    def on_status(self):
        s = self.link.poll()
        if s is None:
            return
        out = String()
        out.data = json.dumps(s, sort_keys=True)
        self.state_pub.publish(out)
        log = s.get('reason_log') or []
        if s.get('reason_log_n', 0) < self.last_log_n:
            self.last_reason_n = -1  # a new process instance: its reason log starts again
        self.last_log_n = s.get('reason_log_n', 0)
        for e in log:
            if e['n'] > self.last_reason_n:
                r = String()
                r.data = json.dumps({'reason': e['reason'], 't_us': e['t_us'], 'n': e['n'],
                                     'state': s['state']})
                self.reason_pub.publish(r)
                self.last_reason_n = e['n']
        odom = s.get('odom') or {}
        if not odom.get('valid'):
            return
        now = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = odom['x']
        msg.pose.pose.position.y = odom['y']
        msg.pose.pose.orientation.z = math.sin(odom['yaw'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(odom['yaw'] / 2.0)
        msg.twist.twist.linear.x = odom['v']
        msg.twist.twist.angular.z = odom['w']
        self.odom_pub.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom['x']
        t.transform.translation.y = odom['y']
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)
        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [float(p) for p in odom['wheel_pos']]
        js.velocity = [float(v) for v in odom['wheel_rad_s']]
        self.joint_pub.publish(js)

    def shutdown(self):
        self.link.close()


def main():
    rclpy.init()
    node = SimrefBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
