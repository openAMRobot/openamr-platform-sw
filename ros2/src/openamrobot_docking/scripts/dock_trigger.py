#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from opennav_docking_msgs.action import DockRobot, UndockRobot


class DockTrigger(Node):
    def __init__(self):
        super().__init__('dock_trigger')

        self.declare_parameter('trigger_topic', 'dock_trigger')
        self.declare_parameter('use_dock_id', True)
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('dock_type', 'openamrobot_dock')
        self.declare_parameter('navigate_to_staging_pose', True)
        self.declare_parameter('undock_on_false', False)

        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.use_dock_id = self.get_parameter('use_dock_id').value
        self.dock_id = self.get_parameter('dock_id').value
        self.dock_type = self.get_parameter('dock_type').value
        self.navigate_to_staging_pose = self.get_parameter('navigate_to_staging_pose').value
        self.undock_on_false = self.get_parameter('undock_on_false').value

        self.dock_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, 'undock_robot')

        self.sub = self.create_subscription(Bool, self.trigger_topic, self.trigger_cb, 10)
        self.get_logger().info(
            f"Dock trigger ready on '{self.trigger_topic}'. true=dock, false={'undock' if self.undock_on_false else 'ignore'}")

    def trigger_cb(self, msg: Bool):
        if msg.data:
            self.send_dock_goal()
        elif self.undock_on_false:
            self.send_undock_goal()

    def send_dock_goal(self):
        if not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('DockRobot action server not available')
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = bool(self.use_dock_id)
        if goal.use_dock_id:
            goal.dock_id = str(self.dock_id)
        goal.dock_type = str(self.dock_type)
        goal.navigate_to_staging_pose = bool(self.navigate_to_staging_pose)

        self.get_logger().info(
            f"Sending dock goal: use_dock_id={goal.use_dock_id}, dock_id='{goal.dock_id}', dock_type='{goal.dock_type}'")
        self.dock_client.send_goal_async(goal)

    def send_undock_goal(self):
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('UndockRobot action server not available')
            return

        goal = UndockRobot.Goal()
        goal.dock_type = str(self.dock_type)

        self.get_logger().info(f"Sending undock goal: dock_type='{goal.dock_type}'")
        self.undock_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = DockTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
