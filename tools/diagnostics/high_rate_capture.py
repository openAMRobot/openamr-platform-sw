#!/usr/bin/env python3
"""Capture 50 Hz autour d'un jerk de la roue droite.
ROUES EN L'AIR. 24V. Main sur la coupure.

Commande 0.05 m/s, enregistre CHAQUE message /debug/right (pleine cadence),
detecte le 1er jerk (|rpm droit| > JERK), continue un peu, puis dump la fenetre
autour du jerk : counts bruts droits echantillon par echantillon + delta.

Interpretation :
  - counts qui SAUTENT puis REVIENNENT (delta +N puis -N) sur 1-2 echantillons,
    net ~0  -> FAUSSE impulsion (glitch electrique encodeur droit / masse).
  - counts qui derivent franchement dans un sens -> VRAI mouvement (driver/moteur).

Usage : python3 ~/high_rate_capture.py [vitesse] [duree_max]
  defaut : 0.05 m/s, 10 s
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
DUR_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
JERK = 30.0          # |rpm droit| au-dela = jerk detecte
AFTER = 0.4          # s a continuer apres le jerk
PWM_ABORT = 850

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)


class Cap(Node):
    def __init__(self):
        super().__init__('high_rate_capture')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/debug/left', self.cl, BE)
        self.create_subscription(Vector3, '/debug/right', self.cr, BE)
        self.create_subscription(Vector3, '/debug/pwm', self.cp, BE)
        self.cntL = 0.0
        self.pwmR = 0.0
        self.pwmL = 0.0
        self.samples = []      # (t, cntL, cntR, rpmR, pwmR)
        self.jerk_t = None
        self.t0 = time.time()

    def cl(self, m):
        self.cntL = m.z

    def cp(self, m):
        self.pwmL = m.x
        self.pwmR = m.y

    def cr(self, m):
        t = time.time() - self.t0
        self.samples.append((t, self.cntL, m.z, m.y, self.pwmR))
        if self.jerk_t is None and abs(m.y) > JERK:
            self.jerk_t = t

    def send(self, vx):
        msg = Twist()
        msg.linear.x = vx
        self.pub.publish(msg)

    def stop(self, n=15):
        for _ in range(n):
            self.send(0.0)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = Cap()
    t0 = time.time()
    while time.time() - t0 < 0.6:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.t0 = time.time()

    print(f"CMD {SPEED:+.3f} m/s, capture 50 Hz, jerk si |rpm_D|>{JERK}", flush=True)
    t0 = time.time()
    try:
        while time.time() - t0 < DUR_MAX:
            node.send(SPEED)
            rclpy.spin_once(node, timeout_sec=0.02)
            if abs(node.pwmR) > PWM_ABORT or abs(node.pwmL) > PWM_ABORT:
                print(">>> pwm sature, abort", flush=True)
                break
            if node.jerk_t is not None and (time.time() - node.t0) > node.jerk_t + AFTER:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    s = node.samples
    print(f"\n{len(s)} echantillons. jerk_t = {node.jerk_t}")
    if node.jerk_t is None:
        print("Aucun jerk detecte (pas d'emballement sur ce run).")
    else:
        # indice du 1er echantillon au jerk
        ji = next((i for i, x in enumerate(s) if x[0] >= node.jerk_t), len(s) - 1)
        a = max(0, ji - 20)
        b = min(len(s), ji + 12)
        print("Fenetre autour du jerk (>>> = echantillon du jerk) :")
        print("   t(s)  | cntL    dL | cntR     dR  | rpm_D | pwm_D")
        print("---------+------------+--------------+-------+------")
        prevL = prevR = None
        for i in range(a, b):
            t, cl, cr, rpm, pwm = s[i]
            dL = "" if prevL is None else f"{int(cl - prevL):+4d}"
            dR = "" if prevR is None else f"{int(cr - prevR):+5d}"
            prevL, prevR = cl, cr
            mark = ">>>" if i == ji else "   "
            print(f"{mark}{t:6.2f} | {int(cl):6d} {dL:>4} | {int(cr):7d} {dR:>5} | "
                  f"{rpm:+5.1f} | {pwm:+5.0f}", flush=True)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


main()
