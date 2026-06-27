#!/usr/bin/env python3
"""
Calibra o wheel_radius medindo a distancia LINEAR real (lidar) vs odometria.
Aponta a uma parede, anda em frente d_odom, mede d_real pela reducao do alcance
frontal. wheel_radius_novo = wheel_radius_atual * (d_real / d_odom).
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

RADIUS_ATUAL = 0.0985
ALVO_M = 0.4              # distancia por ensaio
VEL_LIN = 0.15
N_ENSAIOS = 4

class Calib(Node):
    def __init__(self):
        super().__init__('calibrar_linear')
        self.scan = None; self.amin = None; self.ainc = None
        self.x = None; self.y = None; self.yaw = None
        self.create_subscription(LaserScan, '/scan', self._scan, 1)
        self.create_subscription(Odometry, '/odom', self._odom, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        self.scan = np.array(m.ranges, dtype=np.float32)
        self.amin = m.angle_min; self.ainc = m.angle_increment

    def _odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def frescos(self, n=8):
        for _ in range(n): rclpy.spin_once(self, timeout_sec=0.1)

    def idx_frente(self):
        return int(round((0.0 - self.amin)/self.ainc))   # angulo 0 = frente

    def range_frente(self):
        i = self.idx_frente()
        seg = self.scan[max(0,i-15):i+15]
        seg = seg[np.isfinite(seg) & (seg > 0.05)]
        return float(np.median(seg)) if len(seg) > 5 else -1.0

    def gira(self, vel, t):
        cmd = Twist(); cmd.angular.z = vel
        t0 = time.time()
        while time.time()-t0 < t:
            self.pub.publish(cmd); rclpy.spin_once(self, timeout_sec=0.05)
        self.parar()

    def parar(self):
        for _ in range(8): self.pub.publish(Twist()); time.sleep(0.03)

    def aponta_parede(self):
        # roda ate ter uma parede a frente entre 0.8 e 4 m
        self.frescos()
        for _ in range(40):
            rf = self.range_frente()
            if 0.8 < rf < 4.0:
                return rf
            self.gira(0.4, 0.6)
            self.frescos()
        return self.range_frente()

    def anda(self, alvo):
        self.frescos(); x0, y0 = self.x, self.y
        cmd = Twist(); cmd.linear.x = VEL_LIN
        d = 0.0; t0 = time.time()
        while d < alvo and time.time()-t0 < 12:
            self.pub.publish(cmd); rclpy.spin_once(self, timeout_sec=0.05)
            d = math.hypot(self.x-x0, self.y-y0)
        self.parar(); self.frescos(8)
        return math.hypot(self.x-x0, self.y-y0)

    def run(self):
        self.frescos(10)
        if self.scan is None or self.x is None:
            print('ERRO: sem scan/odom'); return
        soma_odom, soma_real = 0.0, 0.0
        n_ok = 0
        for i in range(N_ENSAIOS):
            rf0 = self.aponta_parede()
            if not (0.8 < rf0 < 4.0):
                print(f'ensaio {i+1}: sem parede frontal boa (rf={rf0:.2f}), skip'); continue
            d_odom = self.anda(ALVO_M)
            self.frescos(8)
            rf1 = self.range_frente()
            d_real = rf0 - rf1
            # recuar para nao colar na parede
            cmd = Twist(); cmd.linear.x = -VEL_LIN
            t0=time.time()
            while time.time()-t0 < d_odom/VEL_LIN + 0.5:
                self.pub.publish(cmd); rclpy.spin_once(self, timeout_sec=0.05)
            self.parar()
            if d_real <= 0.05:
                print(f'ensaio {i+1}: d_real invalido ({d_real:.3f}), skip'); continue
            soma_odom += d_odom; soma_real += d_real; n_ok += 1
            print(f'ensaio {i+1}: odom={d_odom:.3f}m  real(lidar)={d_real:.3f}m  '
                  f'racio={d_odom/d_real:.3f}', flush=True)
            time.sleep(0.3)
        if n_ok == 0 or soma_real <= 0:
            print('ERRO: nenhum ensaio valido'); return
        racio = soma_odom/soma_real
        novo = RADIUS_ATUAL * (soma_real/soma_odom)
        print('\n==== RESULTADO LINEAR ====')
        print(f'odom total={soma_odom:.3f}m  real total={soma_real:.3f}m  ({n_ok} ensaios)')
        print(f'racio (odom/real) = {racio:.4f}')
        print(f'wheel_radius: {RADIUS_ATUAL} -> {novo:.4f}')

def main():
    rclpy.init(); n = Calib()
    try: n.run()
    finally:
        n.parar(); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
