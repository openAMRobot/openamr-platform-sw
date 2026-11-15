"""ROS 2 node: odometry from the wheel speeds the firmware image reports (audit item SW-08).

Subscribes /sil/fw/wheel_speeds. Publishes:
  /sil/fw/odom          nav_msgs/Odometry, always
  /sil/fw/odom_state    std_msgs/String (JSON): pose, valid flag, counters
and, only with robot_state:=true, as the single writer of each in the Renode modes:
  /odom                 the same odometry
  /tf                   odom -> base_footprint
  /joint_states         the Gazebo joint states of /sil/gz/joint_states, unchanged, so that
                        robot_state_publisher has a joint source

While the image has no valid wheel value the pose is held, the twist is NaN, nothing is
published on /odom and /tf, and odom_state says valid false: unknown is not standing still.
Stand-in for the odometry of the base controller. See openamrobot_sil/fw_odometry.py.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from openamrobot_sil import config
from openamrobot_sil.fw_odometry import FwOdometry
from openamrobot_sil.nodes.common import STEADY, publish_json


class FwOdometryNode(Node):
    def __init__(self):
        super().__init__('sil_fw_odometry')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        self.robot_state = self.declare_parameter('robot_state', False).value
        d = cfg['drive']
        self.odo = FwOdometry(d['wheel_radius'], d['wheel_separation'], d['left_sign'],
                              d['right_sign'])
        self.sil_pub = self.create_publisher(Odometry, '/sil/fw/odom', 10)
        self.state_pub = self.create_publisher(String, '/sil/fw/odom_state', 10)
        self.odom_pub = self.tf_pub = self.joint_pub = None
        if self.robot_state:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.create_subscription(JointState, '/sil/gz/joint_states',
                                     self.joint_pub.publish, 10)
        self.create_subscription(Float64MultiArray, '/sil/fw/wheel_speeds', self.on_wheels, 50)

    def on_wheels(self, msg):
        if len(msg.data) != 2:
            return
        valid = self.odo.update(msg.data[0], msg.data[1], STEADY.now().nanoseconds / 1e9)
        o = self.odo
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = o.x, o.y
        odom.pose.pose.orientation.z = math.sin(o.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(o.yaw / 2.0)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = o.v, o.w
        self.sil_pub.publish(odom)
        publish_json(self.state_pub, dict(o.state(), robot_state=self.robot_state))
        if self.robot_state and valid:
            self.odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, 'base_footprint'
            tf.transform.translation.x, tf.transform.translation.y = o.x, o.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.publish(TFMessage(transforms=[tf]))


def main():
    rclpy.init()
    node = FwOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
