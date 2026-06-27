#!/usr/bin/env python3
"""
Calibra o wheel_separation_multiplier medindo a rotacao REAL com o lidar
(correlacao do scan antes/depois) vs a rotacao reportada pela odometria.

Faz N rotacoes controladas. Para cada uma:
  - rotacao_odom  = variacao do yaw da /odom
  - rotacao_real  = deslocamento angular do scan (cross-correlacao)
multiplier_novo = multiplier_atual * (rotacao_odom / rotacao_real)

(Se a odometria reporta MAIS rotacao do que a real, sep efetiva e pequena demais
 -> aumentar o multiplier.)
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MULT_ATUAL = 0.987          # valor atual no ros2_control.yml
ALVO_RAD = 0.6              # rotacao alvo por ensaio (~34 graus)
VEL_ANG = 0.3              # rad/s (lento p/ minimizar erro)
N_ENSAIOS = 4

class Calib(Node):
    def __init__(self):
        super().__init__('calibrar_odom')
        self.scan = None
        self.yaw = None
        self.ainc = None
        self.create_subscription(LaserScan, '/scan', self._scan, 1)
        self.create_subscription(Odometry, '/odom', self._odom, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        r = np.array(m.ranges, dtype=np.float32)
        r[~np.isfinite(r)] = 0.0           # inf/nan -> 0 (mascara depois)
        self.scan = r
        self.ainc = abs(m.angle_increment)

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def espera(self, attr, t=5.0):
        t0 = time.time()
        while time.time()-t0 < t and getattr(self, attr) is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return getattr(self, attr) is not None

    def frescos(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.1)

    def rotacao_real(self, r0, r1):
        # encontra o shift k que melhor alinha r1 sobre r0 (so pontos validos)
        n = len(r0)
        best_k, best_err = 0, 1e18
        maxk = int(1.2 * ALVO_RAD / self.ainc)     # janela de procura
        for k in range(-maxk, maxk+1):
            r1s = np.roll(r1, k)
            mask = (r0 > 0.05) & (r1s > 0.05)
            if mask.sum() < n*0.3:
                continue
            err = np.mean(np.abs(r0[mask]-r1s[mask]))
            if err < best_err:
                best_err, best_k = err, k
        return abs(best_k) * self.ainc

    def parar(self):
        self.pub.publish(Twist())

    def gira(self, alvo):
        self.frescos(); y0 = self.yaw
        cmd = Twist(); cmd.angular.z = VEL_ANG
        acc = 0.0; yprev = y0
        t0 = time.time()
        while acc < alvo and time.time()-t0 < 15:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            d = self.yaw - yprev
            d = math.atan2(math.sin(d), math.cos(d))
            acc += abs(d); yprev = self.yaw
        self.parar()
        for _ in range(10):
            self.pub.publish(Twist()); time.sleep(0.05)
        self.frescos(8)
        return acc

    def run(self):
        if not (self.espera('scan') and self.espera('yaw')):
            print('ERRO: sem scan/odom'); return
        soma_odom, soma_real = 0.0, 0.0
        for i in range(N_ENSAIOS):
            self.frescos(8)
            r0 = self.scan.copy()
            odom_rot = self.gira(ALVO_RAD)
            r1 = self.scan.copy()
            real_rot = self.rotacao_real(r0, r1)
            soma_odom += odom_rot; soma_real += real_rot
            print(f'ensaio {i+1}: odom={math.degrees(odom_rot):.1f}deg  '
                  f'real(lidar)={math.degrees(real_rot):.1f}deg  '
                  f'racio={odom_rot/real_rot if real_rot>0 else 0:.3f}', flush=True)
            time.sleep(0.5)
        if soma_real <= 0:
            print('ERRO: rotacao real medida = 0'); return
        racio = soma_odom / soma_real
        mult_novo = MULT_ATUAL * racio
        print('\n==== RESULTADO ====')
        print(f'odom total={math.degrees(soma_odom):.1f}deg  real total={math.degrees(soma_real):.1f}deg')
        print(f'racio medio (odom/real) = {racio:.4f}')
        print(f'wheel_separation_multiplier: {MULT_ATUAL} -> {mult_novo:.4f}')

def main():
    rclpy.init()
    n = Calib()
    try:
        n.run()
    finally:
        n.parar()
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
