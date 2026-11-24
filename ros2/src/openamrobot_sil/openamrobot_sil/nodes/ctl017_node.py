"""ROS 2 node: CTL-017 lift node emulator on vcan_aux, bridged to the Gazebo lift joint."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from openamrobot_sil import config
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.ctl017_gazebo import GazeboCtl017
from openamrobot_sil.nodes.common import (NodeClock, device_timer, joint_value, publish_json,
                                          thread_check)


class Ctl017Node(Node):
    def __init__(self):
        super().__init__('sil_ctl017')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        self.use_gazebo = self.declare_parameter('gazebo', True).value
        self.joint = cfg['lift']['joint']
        self.bus = open_bus(cfg['buses']['aux']['interface'])
        permit = config.permit_config(cfg)
        self.cmd_pub = self.create_publisher(Float64, '/sil/lift/gz_cmd_vel', 10)
        # One simulation clock per process (clock contract C1): Gazebo time with use_sim_time.
        self.sim = NodeClock(self, on_stall=self.hold_plant)
        self.emu = GazeboCtl017(self.bus, cfg['buses']['aux']['ctl017_node_id'], permit=permit,
                                clock=self.sim.clock)
        self.sim.add_check(thread_check('CTL-017 emulator', self.emu))
        self.started = False
        self.state_pub = self.create_publisher(String, '/sil/lift/emulator_state', 10)
        self.create_subscription(JointState, '/sil/gz/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/sil/lift/inject', self.on_inject, 10)
        device_timer(self, 0.01, self.on_cmd)
        device_timer(self, 0.05, self.on_state)
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

    def start_emulator(self):
        self.emu.start()
        self.started = True
        self.get_logger().info(
            'CTL-017 emulator node 0x%02X started, plant %s, permit gate %s' % (
                self.emu.node_id, 'gazebo' if self.emu.external else 'internal',
                'on' if self.emu.permit_cfg else 'off'))

    def on_joints(self, msg):
        position = joint_value(msg, self.joint, 'position')
        if position is None or not self.use_gazebo:
            return
        if not self.started:
            self.emu.attach_external_plant(position)
            self.start_emulator()
        else:
            self.emu.set_joint_position(position)

    def on_cmd(self):
        msg = Float64()
        msg.data = self.emu.demand_velocity_m_s() if self.started else 0.0
        self.cmd_pub.publish(msg)

    def hold_plant(self):
        """Stall (C5): hold the Gazebo lift command at zero, so the resume starts from rest."""
        self.cmd_pub.publish(Float64(data=0.0))

    def on_state(self):
        if self.started:
            publish_json(self.state_pub, dict(self.emu.state_dict(), sim_clock=self.sim.status()))

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
