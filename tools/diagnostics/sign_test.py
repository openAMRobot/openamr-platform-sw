#!/usr/bin/env python3
"""Test de SIGNE des encodeurs (roues en l'air, a la main, SANS alim 24V).

But : verifier que rouler une roue dans le sens AVANT du robot donne une
vitesse mesuree POSITIVE. Si la droite sort negative en avant ->
MOTOR2_ENCODER_INV est faux -> cause probable de l'emballement.

Lance dans TON terminal :  python3 ~/sign_test.py
Affiche en continu vg et vd signes. Ctrl-C pour arreter.

    v_droite = lin.x + ang.z * (LR/2)
    v_gauche = lin.x - ang.z * (LR/2)
"""
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

LR = 0.45
HALF = LR / 2.0
SEUIL = 0.01


class Mon(Node):
    def __init__(self):
        super().__init__('sign_test')
        self.create_subscription(Odometry, '/odom/unfiltered', self.cb, 10)
        self.vl = 0.0
        self.vr = 0.0

    def cb(self, msg):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        self.vr = lin + ang * HALF
        self.vl = lin - ang * HALF


def fleche(v):
    if v > SEUIL:
        return "AVANT (+)"
    if v < -SEUIL:
        return "ARRIERE(-)"
    return "  ~0      "


def main():
    rclpy.init()
    node = Mon()
    print("Roule chaque roue dans le sens AVANT du robot.")
    print("Attendu : les DEUX doivent afficher AVANT (+).")
    print("Ctrl-C pour arreter.\n", flush=True)
    t0 = time.time()
    last = -1.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last >= 0.4:
                last = now
                print(f"GAUCHE vg={node.vl:+.3f} {fleche(node.vl)}   |   "
                      f"DROITE vd={node.vr:+.3f} {fleche(node.vr)}", flush=True)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


main()
