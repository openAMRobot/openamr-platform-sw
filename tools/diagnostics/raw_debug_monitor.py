#!/usr/bin/env python3
"""Moniteur des topics debug du firmware (counts bruts par roue).
Roues en l'air, a la main, SANS alim 24V.

/debug/left  (Vector3) : x=rpm cible, y=rpm mesure, z=counts bruts  (MOTOR1 gauche)
/debug/right (Vector3) : idem droite (MOTOR2)

Affiche en direct, ~3 Hz. Ctrl-C pour arreter.
But : verifier que les counts (z) bougent quand on tourne la roue.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

# Les publishers debug du firmware sont en BEST_EFFORT -> on doit matcher.
QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Mon(Node):
    def __init__(self):
        super().__init__('raw_debug_monitor')
        self.create_subscription(Vector3, '/debug/left', self.cl, QOS)
        self.create_subscription(Vector3, '/debug/right', self.cr, QOS)
        self.l = (0.0, 0.0, 0.0)
        self.r = (0.0, 0.0, 0.0)

    def cl(self, m):
        self.l = (m.x, m.y, m.z)

    def cr(self, m):
        self.r = (m.x, m.y, m.z)


def main():
    rclpy.init()
    node = Mon()
    print("Tourne chaque roue a la main : les COUNTS (cnt) doivent bouger.")
    print("Ctrl-C pour arreter.\n", flush=True)
    last = -1.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last >= 0.33:
                last = now
                lc, lm, lz = node.l
                rc, rm, rz = node.r
                print(f"G cible={lc:+6.1f} mes={lm:+6.1f} cnt={int(lz):+9d}   |   "
                      f"D cible={rc:+6.1f} mes={rm:+6.1f} cnt={int(rz):+9d}",
                      flush=True)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


main()
