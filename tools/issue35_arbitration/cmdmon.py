#!/usr/bin/env python3
"""Record cmd_vel-type messages, lifecycle transitions and ground truth as CSV.

Each cmd row carries the publisher set of its topic (Jazzy MessageInfo has no
publisher gid), plus dock status, guard mode and Gazebo ground truth rows.

Scratch review tool for issue #35 (not repository code).
usage: cmdmon.py OUT.csv [extra Twist topics...]
"""
import sys
import time

from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.msg import TransitionEvent
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

TWIST_TOPICS = ['/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_teleop']
LIFECYCLE = ['/collision_monitor', '/velocity_smoother', '/behavior_server',
             '/controller_server']


class Mon(Node):
    def __init__(self, out, extra):
        super().__init__('cmd_review_monitor')
        self.f = open(out, 'w', buffering=1)
        self.f.write('wall,sim,kind,topic,source,vx,wz,extra\n')
        self.pubs = {}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.subs = []
        for t in TWIST_TOPICS + extra:
            stamped = t.endswith(':stamped')
            name = t.split(':')[0]
            typ = TwistStamped if stamped else Twist
            self.subs.append(self.create_subscription(
                typ, name, self._mk_cb(name, stamped), qos))
        for n in LIFECYCLE:
            self.subs.append(self.create_subscription(
                TransitionEvent, n + '/transition_event', self._mk_lc(n), 10))
        self.subs.append(self.create_subscription(
            String, '/dock_trigger_status', self._status, 10))
        self.subs.append(self.create_subscription(
            String, '/docking/guard_mode',
            lambda m: self._row('mode', '/docking/guard_mode', 'dock_guard_mode', extra=m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.subs.append(self.create_subscription(
            TFMessage, '/gt_poses', self._gt, 10))
        self._last_gt = 0.0
        self.create_timer(1.0, self._refresh)

    def _refresh(self):
        self.pubs = {}
        for t in [s.topic_name for s in self.subs]:
            try:
                self.pubs[t] = {i.node_name for i in self.get_publishers_info_by_topic(t)
                                if i.node_name != 'cmd_review_monitor'}
            except Exception:
                pass

    def _row(self, kind, topic, source, vx='', wz='', extra=''):
        sim = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(f'{time.time():.4f},{sim:.4f},{kind},{topic},{source},{vx},{wz},{extra}\n')

    def _mk_cb(self, topic, stamped):
        def cb(msg, info):
            # Jazzy MessageInfo carries no publisher gid; report the publisher
            # set instead (one name = unambiguous source).
            src = '|'.join(sorted(self.pubs.get(topic, []))) or 'unknown'
            tw = msg.twist if stamped else msg
            self._row('cmd', topic, src, f'{tw.linear.x:.4f}', f'{tw.angular.z:.4f}')
        return cb

    def _mk_lc(self, node):
        def cb(msg):
            change = f'{msg.start_state.label}->{msg.goal_state.label}'
            self._row('lifecycle', node, node, extra=change)
        return cb

    def _status(self, msg):
        self._row('status', '/dock_trigger_status', 'dock_trigger', extra=msg.data)

    def _gt(self, msg):
        # ros_gz_bridge drops Pose_V names; the robot model is entry 6
        # (ground, 4 walls, dock, robot) in walled_world, checked by driving it.
        for tf in msg.transforms[6:7]:
            if True:
                now = time.time()
                if now - self._last_gt >= 0.02:
                    self._last_gt = now
                    p = tf.transform.translation
                    q = tf.transform.rotation
                    self._row('gt', '/gt_poses', 'gazebo',
                              extra=f'{p.x:.4f};{p.y:.4f};{q.z:.5f};{q.w:.5f}')


def main():
    rclpy.init()
    n = Mon(sys.argv[1], sys.argv[2:])
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
