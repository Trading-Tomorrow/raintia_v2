#!/usr/bin/env python3
"""Navega ao ponto 'dir' e captura a imagem da camara (para ver o dano)."""
import math, time, rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav2_simple_commander.robot_navigator import BasicNavigator
from cv_bridge import CvBridge
import cv2
from rclpy.node import Node

ALVO = (5.31, 1.86, 1.215)   # de frente para o flanco direito (-Y) do carro

class Cap(Node):
    def __init__(self):
        super().__init__('captura_dir')
        self.br = CvBridge(); self.img = None
        self.create_subscription(Image, '/Tiago_Lite/Astra_rgb/image_color', self.cb, 10)
        self.nav = BasicNavigator()
    def cb(self, m):
        try: self.img = self.br.imgmsg_to_cv2(m, 'bgr8')
        except Exception: pass
    def run(self):
        self.nav.waitUntilNav2Active()
        p = PoseStamped(); p.header.frame_id = 'map'
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x = ALVO[0]; p.pose.position.y = ALVO[1]
        p.pose.orientation.z = math.sin(ALVO[2]/2); p.pose.orientation.w = math.cos(ALVO[2]/2)
        self.nav.goToPose(p)
        t = time.time()
        while not self.nav.isTaskComplete():
            if time.time()-t > 90:
                self.nav.cancelTask(); break
            rclpy.spin_once(self, timeout_sec=0.2)
        for _ in range(25):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.img is not None:
            cv2.imwrite('captura_dir.jpg', self.img)
            print('GUARDADO captura_dir.jpg')
        else:
            print('SEM IMAGEM')

def main():
    rclpy.init(); c = Cap()
    try: c.run()
    finally:
        c.destroy_node(); rclpy.shutdown()

main()
