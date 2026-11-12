#!/usr/bin/env python3
"""Test SOUS PUISSANCE avec telemetrie debug (counts bruts + pwm).
ROUES EN L'AIR. 24V sur les drivers. Main sur la coupure 24V.

Commande une petite vitesse avant et logue, par roue :
  rpm mesure, counts bruts, delta counts (par intervalle), et le PWM.

But : voir si les counts DROITS decrochent (delta ~0) pendant que le PWM
grimpe/sature -> windup confirme.

Usage : python3 ~/powered_debug_test.py [vitesse] [duree]
  defaut : 0.1 m/s, 5 s

Securites :
  - auto-abort si |pwm| > PWM_ABORT (saturation) OU |rpm mesure| > RPM_ABORT
  - zeros envoyes a la fin / a l'abort / Ctrl-C
  - firmware coupe aussi les moteurs si plus de cmd_vel pendant 200 ms
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
PWM_MAX = 1023          # PWM_BITS=10 -> max 1023
PWM_ABORT = 850         # |pwm| au-dela = saturation -> abort
RPM_ABORT = 45          # |rpm mesure| au-dela = emballement -> abort

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)


class T(Node):
    def __init__(self):
        super().__init__('powered_debug_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/debug/left', self.cl, BE)
        self.create_subscription(Vector3, '/debug/right', self.cr, BE)
        self.create_subscription(Vector3, '/debug/pwm', self.cp, BE)
        self.l = (0.0, 0.0, 0.0)   # cible, mesure, counts
        self.r = (0.0, 0.0, 0.0)
        self.pwm = (0.0, 0.0, 0.0)  # pwm gauche, pwm droite

    def cl(self, m):
        self.l = (m.x, m.y, m.z)

    def cr(self, m):
        self.r = (m.x, m.y, m.z)

    def cp(self, m):
        self.pwm = (m.x, m.y, m.z)

    def send(self, vx):
        t = Twist()
        t.linear.x = vx
        self.pub.publish(t)

    def stop(self, n=15):
        for _ in range(n):
            self.send(0.0)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = T()
    t0 = time.time()
    while time.time() - t0 < 0.6:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"CMD lin.x={SPEED:+.3f} m/s pendant {DURATION:.0f}s  "
          f"(abort si |pwm|>{PWM_ABORT} ou |rpm|>{RPM_ABORT})")
    print(" t   | cmd  | GAUCHE rpm  cnt    d | DROITE rpm  cnt    d | pwmG  pwmD")
    print("-----+------+----------------------+----------------------+-----------",
          flush=True)

    aborted = None
    prev_lz = node.l[2]
    prev_rz = node.r[2]
    t0 = time.time()
    last = -1.0
    try:
        while time.time() - t0 < DURATION:
            node.send(SPEED)
            rclpy.spin_once(node, timeout_sec=0.05)
            pl, pr = node.pwm[0], node.pwm[1]
            if abs(pl) > PWM_ABORT or abs(pr) > PWM_ABORT:
                aborted = f"PWM sature (G={pl:.0f} D={pr:.0f})"
                break
            if abs(node.l[1]) > RPM_ABORT or abs(node.r[1]) > RPM_ABORT:
                aborted = f"RPM emballe (G={node.l[1]:.0f} D={node.r[1]:.0f})"
                break
            now = time.time()
            if now - last >= 0.2:
                last = now
                lz, rz = node.l[2], node.r[2]
                dl, dr = int(lz - prev_lz), int(rz - prev_rz)
                prev_lz, prev_rz = lz, rz
                print(f"{now - t0:4.1f} | {SPEED:+.2f} | "
                      f"{node.l[1]:+6.1f} {int(lz):+7d} {dl:+5d} | "
                      f"{node.r[1]:+6.1f} {int(rz):+7d} {dr:+5d} | "
                      f"{pl:+5.0f} {pr:+5.0f}", flush=True)
    except KeyboardInterrupt:
        aborted = "Ctrl-C"
    finally:
        node.stop()

    print("\n----- FIN -----")
    print(f"abort = {aborted if aborted else 'non (duree ecoulee)'}")
    print(f"dernier pwm  G={node.pwm[0]:+.0f}  D={node.pwm[1]:+.0f}")
    print(f"dernier rpm  G={node.l[1]:+.1f}  D={node.r[1]:+.1f}")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


main()
